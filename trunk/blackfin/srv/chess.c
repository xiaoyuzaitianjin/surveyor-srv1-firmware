/*
 *    MAIN.C
 *    Tom Kerrigan's Simple Chess Program (TSCP)
 *
 *    Copyright 1997 Tom Kerrigan
 */


#include "srv.h"
#include "print.h"
#include "string.h"
#include "setjmp.h"
#include "malloc.h"
#include "chess.h"

/* see the beginning of think() */
int errjmp[41];
BOOL stop_search;

/* get_ms() returns the milliseconds elapsed since midnight,
   January 1, 1970. */

BOOL ftime_ok = FALSE;  /* does ftime return milliseconds? */
int get_ms()
{
    return (readRTC());
}

int scan(char *st) {
    while (1) {
        *st = getch();
        if ((*st == 0x04) || (*st == 0x1B))  // EOF or ESC
            return 0;
        if (*st <= 0x20) {
            *st = 0;
            return 1;
        }
        st++;
    }
}

void init_mem() {
    pv = (move **)malloc(MAX_PLY * MAX_PLY * sizeof(move));
    history = (int **)malloc(64 * 64 * sizeof(int));
    hist_dat = (void *)malloc(HIST_STACK * sizeof(hist_t));
    gen_dat = (void *)malloc(GEN_STACK * sizeof(gen_t));
}

void free_mem() {
    free((char *)pv);
    free((char *)history);
    free((char *)hist_dat);
    free((char *)gen_dat);
}

void flush_input() {
    unsigned char ch;
    while (getchar(&ch))
        continue;
}

/* main() is basically an infinite loop that either calls
   think() when it's the computer's turn to move or prompts
   the user for a command (and deciphers it). */

int chess()
{
    int computer_side;
    char s[256];
    int m;

    init_mem();
    init_hash();
    init_board();
    open_book();
    gen();

    printf("\n\r");
    printf("Tom Kerrigan's Simple Chess Program (TSCP)\n\r");
    printf("version 1.81, 2/5/03\n\r");
    printf("Copyright 1997 Tom Kerrigan\n\r");
    printf("\n\r");
    printf("\"help\" displays a list of commands.\n\r");
    printf("\n\r");
    print_board();
    flush_input();
    
    computer_side = EMPTY;
    max_time = 1 << 25;
    max_depth = 4;
    for (;;) {
        if (side == computer_side) {  /* computer's turn */
            
            /* think about the move and make it */
            think(1);
            if (!pv[0][0].u) {
                printf("(no legal moves)\n\r");
                computer_side = EMPTY;
                continue;
            }
            printf("Robot's move: %s\n\r", move_str(pv[0][0].b));
            makemove(pv[0][0].b);
            ply = 0;
            gen();
            print_result();
            print_board();
            continue;
        }

        /* get user input */
        printf("tscp> ");
        if (scan( s) == 0) {
            free_mem();
            return 0;
        }
        if (!strcmp(s, "on")) {
            computer_side = side;
            continue;
        }
        if (!strcmp(s, "off")) {
            computer_side = EMPTY;
            continue;
        }
        if (!strcmp(s, "st")) {
            scan( s);
            max_time = atoi(s) * 1000;
            max_depth = 32;
            continue;
        }
        if (!strcmp(s, "sd")) {
            max_depth = atoi(s);
            max_time = 1 << 25;
            continue;
        }
        if (!strcmp(s, "undo")) {
            if (!hply)
                continue;
            computer_side = EMPTY;
            takeback();
            ply = 0;
            gen();
            print_board();
            continue;
        }
        if (!strcmp(s, "new")) {
            computer_side = EMPTY;
            init_board();
            gen();
            continue;
        }
        if (!strcmp(s, "d")) {
            print_board();
            continue;
        }
        if (!strcmp(s, "bye")) {
            printf("Goodbye!\n");
            break;
        }
        if (!strcmp(s, "help")) {
            printf("on - robot plays for the side to move\n");
            printf("off - robot stops playing\n");
            printf("st n - search for n seconds per move\n");
            printf("sd n - search n ply per move\n");
            printf("undo - takes back a move\n");
            printf("new - starts a new game\n");
            printf("d - display the board\n");
            printf("bye - exit the program\n");
            printf("Enter moves in coordinate notation, e.g., e2e4, e7e8Q\n");
            continue;
        }

        /* maybe the user entered a move? */
        m = parse_move(s);
        if (m == -1 || !makemove(gen_dat[m].m.b))
            printf("Illegal move.\n\r");
        else {
            ply = 0;
            gen();
            print_result();
            print_board();
        }
    }
    free_mem();
    return 0;
}


/* parse the move s (in coordinate notation) and return the move's
   index in gen_dat, or -1 if the move is illegal */

int parse_move(char *s)
{
    int from, to, i;

    /* make sure the string looks like a move */
    if (s[0] < 'a' || s[0] > 'h' ||
            s[1] < '0' || s[1] > '9' ||
            s[2] < 'a' || s[2] > 'h' ||
            s[3] < '0' || s[3] > '9')
        return -1;

    from = s[0] - 'a';
    from += 8 * (8 - (s[1] - '0'));
    to = s[2] - 'a';
    to += 8 * (8 - (s[3] - '0'));

    for (i = 0; i < first_move[1]; ++i)
        if (gen_dat[i].m.b.from == from && gen_dat[i].m.b.to == to) {

            /* if the move is a promotion, handle the promotion piece;
               assume that the promotion moves occur consecutively in
               gen_dat. */
            if (gen_dat[i].m.b.bits & 32)
                switch (s[4]) {
                    case 'N':
                        return i;
                    case 'B':
                        return i + 1;
                    case 'R':
                        return i + 2;
                    default:  /* assume it's a queen */
                        return i + 3;
                }
            return i;
        }

    /* didn't find the move */
    return -1;
}


/* move_str returns a string with move m in coordinate notation */

char *move_str(move_bytes m)
{
    static char str[6];

    char c;

    if (m.bits & 32) {
        switch (m.promote) {
            case KNIGHT:
                c = 'n';
                break;
            case BISHOP:
                c = 'b';
                break;
            case ROOK:
                c = 'r';
                break;
            default:
                c = 'q';
                break;
        }
        sprintf(str, "%c%d%c%d%c",
                COL(m.from) + 'a',
                8 - ROW(m.from),
                COL(m.to) + 'a',
                8 - ROW(m.to),
                c);
    }
    else
        sprintf(str, "%c%d%c%d",
                COL(m.from) + 'a',
                8 - ROW(m.from),
                COL(m.to) + 'a',
                8 - ROW(m.to));
    return str;
}


/* print_board() prints the board */

void print_board()
{
    int i;
    
    printf("\n8 ");
    for (i = 0; i < 64; ++i) {
        switch (color[i]) {
            case EMPTY:
                printf(" .");
                break;
            case LIGHT:
                printf(" %c", piece_char[piece[i]]);
                break;
            case DARK:
                printf(" %c", piece_char[piece[i]] + ('a' - 'A'));
                break;
        }
        if ((i + 1) % 8 == 0 && i != 63)
            printf("\n%d ", 7 - ROW(i));
    }
    printf("\n\n   a b c d e f g h\n\n\r");
}




/* print_result() checks to see if the game is over, and if so,
   prints the result. */

void print_result()
{
    int i;

    /* is there a legal move? */
    for (i = 0; i < first_move[1]; ++i)
        if (makemove(gen_dat[i].m.b)) {
            takeback();
            break;
        }
    if (i == first_move[1]) {
        if (in_check(side)) {
            if (side == LIGHT)
                printf("0-1 {Black mates}\n\r");
            else
                printf("1-0 {White mates}\n\r");
        }
        else
            printf("1/2-1/2 {Stalemate}\n\r");
    }
    else if (reps() == 3)
        printf("1/2-1/2 {Draw by repetition}\n\r");
    else if (fifty >= 100)
        printf("1/2-1/2 {Draw by fifty move rule}\n\r");
}


/* init_board() sets the board to the initial game state. */

void init_board()
{
    int i;

    for (i = 0; i < 64; ++i) {
        color[i] = init_color[i];
        piece[i] = init_piece[i];
    }
    side = LIGHT;
    xside = DARK;
    castle = 15;
    ep = -1;
    fifty = 0;
    ply = 0;
    hply = 0;
    set_hash();  /* init_hash() must be called before this function */
    first_move[0] = 0;
}


/* init_hash() initializes the random numbers used by set_hash(). */

void init_hash()
{
    int i, j, k;

    for (i = 0; i < 2; ++i)
        for (j = 0; j < 6; ++j)
            for (k = 0; k < 64; ++k)
                hash_piece[i][j][k] = hash_rand();
    hash_side = hash_rand();
    for (i = 0; i < 64; ++i)
        hash_ep[i] = hash_rand();
}


/* hash_rand() XORs some shifted random numbers together to make sure
   we have good coverage of all 32 bits. (rand() returns 16-bit numbers
   on some systems.) */

int hash_rand()
{
    int i;
    int r = 0;

    for (i = 0; i < 32; ++i)
        r ^= rand() << i;
    return r;
}


/* set_hash() uses the Zobrist method of generating a unique number (hash)
   for the current chess position. Of course, there are many more chess
   positions than there are 32 bit numbers, so the numbers generated are
   not really unique, but they're unique enough for our purposes (to detect
   repetitions of the position). 
   The way it works is to XOR random numbers that correspond to features of
   the position, e.g., if there's a black knight on B8, hash is XORed with
   hash_piece[BLACK][KNIGHT][B8]. All of the pieces are XORed together,
   hash_side is XORed if it's black's move, and the en passant square is
   XORed if there is one. (A chess technicality is that one position can't
   be a repetition of another if the en passant state is different.) */

void set_hash()
{
    int i;

    hash = 0;    
    for (i = 0; i < 64; ++i)
        if (color[i] != EMPTY)
            hash ^= hash_piece[color[i]][piece[i]][i];
    if (side == DARK)
        hash ^= hash_side;
    if (ep != -1)
        hash ^= hash_ep[ep];
}


/* in_check() returns TRUE if side s is in check and FALSE
   otherwise. It just scans the board to find side s's king
   and calls attack() to see if it's being attacked. */

BOOL in_check(int s)
{
    int i;

    for (i = 0; i < 64; ++i)
        if (piece[i] == KING && color[i] == s)
            return attack(i, s ^ 1);
    return TRUE;  /* shouldn't get here */
}


/* attack() returns TRUE if square sq is being attacked by side
   s and FALSE otherwise. */

BOOL attack(int sq, int s)
{
    int i, j, n;

    for (i = 0; i < 64; ++i)
        if (color[i] == s) {
            if (piece[i] == PAWN) {
                if (s == LIGHT) {
                    if (COL(i) != 0 && i - 9 == sq)
                        return TRUE;
                    if (COL(i) != 7 && i - 7 == sq)
                        return TRUE;
                }
                else {
                    if (COL(i) != 0 && i + 7 == sq)
                        return TRUE;
                    if (COL(i) != 7 && i + 9 == sq)
                        return TRUE;
                }
            }
            else
                for (j = 0; j < offsets[piece[i]]; ++j)
                    for (n = i;;) {
                        n = mailbox[mailbox64[n] + offset[piece[i]][j]];
                        if (n == -1)
                            break;
                        if (n == sq)
                            return TRUE;
                        if (color[n] != EMPTY)
                            break;
                        if (!slide[piece[i]])
                            break;
                    }
        }
    return FALSE;
}


/* gen() generates pseudo-legal moves for the current position.
   It scans the board to find friendly pieces and then determines
   what squares they attack. When it finds a piece/square
   combination, it calls gen_push to put the move on the "move
   stack." */

void gen()
{
    int i, j, n;

    /* so far, we have no moves for the current ply */
    first_move[ply + 1] = first_move[ply];

    for (i = 0; i < 64; ++i)
        if (color[i] == side) {
            if (piece[i] == PAWN) {
                if (side == LIGHT) {
                    if (COL(i) != 0 && color[i - 9] == DARK)
                        gen_push(i, i - 9, 17);
                    if (COL(i) != 7 && color[i - 7] == DARK)
                        gen_push(i, i - 7, 17);
                    if (color[i - 8] == EMPTY) {
                        gen_push(i, i - 8, 16);
                        if (i >= 48 && color[i - 16] == EMPTY)
                            gen_push(i, i - 16, 24);
                    }
                }
                else {
                    if (COL(i) != 0 && color[i + 7] == LIGHT)
                        gen_push(i, i + 7, 17);
                    if (COL(i) != 7 && color[i + 9] == LIGHT)
                        gen_push(i, i + 9, 17);
                    if (color[i + 8] == EMPTY) {
                        gen_push(i, i + 8, 16);
                        if (i <= 15 && color[i + 16] == EMPTY)
                            gen_push(i, i + 16, 24);
                    }
                }
            }
            else
                for (j = 0; j < offsets[piece[i]]; ++j)
                    for (n = i;;) {
                        n = mailbox[mailbox64[n] + offset[piece[i]][j]];
                        if (n == -1)
                            break;
                        if (color[n] != EMPTY) {
                            if (color[n] == xside)
                                gen_push(i, n, 1);
                            break;
                        }
                        gen_push(i, n, 0);
                        if (!slide[piece[i]])
                            break;
                    }
        }

    /* generate castle moves */
    if (side == LIGHT) {
        if (castle & 1)
            gen_push(E1, G1, 2);
        if (castle & 2)
            gen_push(E1, C1, 2);
    }
    else {
        if (castle & 4)
            gen_push(E8, G8, 2);
        if (castle & 8)
            gen_push(E8, C8, 2);
    }
    
    /* generate en passant moves */
    if (ep != -1) {
        if (side == LIGHT) {
            if (COL(ep) != 0 && color[ep + 7] == LIGHT && piece[ep + 7] == PAWN)
                gen_push(ep + 7, ep, 21);
            if (COL(ep) != 7 && color[ep + 9] == LIGHT && piece[ep + 9] == PAWN)
                gen_push(ep + 9, ep, 21);
        }
        else {
            if (COL(ep) != 0 && color[ep - 9] == DARK && piece[ep - 9] == PAWN)
                gen_push(ep - 9, ep, 21);
            if (COL(ep) != 7 && color[ep - 7] == DARK && piece[ep - 7] == PAWN)
                gen_push(ep - 7, ep, 21);
        }
    }
}


/* gen_caps() is basically a copy of gen() that's modified to
   only generate capture and promote moves. It's used by the
   quiescence search. */

void gen_caps()
{
    int i, j, n;

    first_move[ply + 1] = first_move[ply];
    for (i = 0; i < 64; ++i)
        if (color[i] == side) {
            if (piece[i]==PAWN) {
                if (side == LIGHT) {
                    if (COL(i) != 0 && color[i - 9] == DARK)
                        gen_push(i, i - 9, 17);
                    if (COL(i) != 7 && color[i - 7] == DARK)
                        gen_push(i, i - 7, 17);
                    if (i <= 15 && color[i - 8] == EMPTY)
                        gen_push(i, i - 8, 16);
                }
                if (side == DARK) {
                    if (COL(i) != 0 && color[i + 7] == LIGHT)
                        gen_push(i, i + 7, 17);
                    if (COL(i) != 7 && color[i + 9] == LIGHT)
                        gen_push(i, i + 9, 17);
                    if (i >= 48 && color[i + 8] == EMPTY)
                        gen_push(i, i + 8, 16);
                }
            }
            else
                for (j = 0; j < offsets[piece[i]]; ++j)
                    for (n = i;;) {
                        n = mailbox[mailbox64[n] + offset[piece[i]][j]];
                        if (n == -1)
                            break;
                        if (color[n] != EMPTY) {
                            if (color[n] == xside)
                                gen_push(i, n, 1);
                            break;
                        }
                        if (!slide[piece[i]])
                            break;
                    }
        }
    if (ep != -1) {
        if (side == LIGHT) {
            if (COL(ep) != 0 && color[ep + 7] == LIGHT && piece[ep + 7] == PAWN)
                gen_push(ep + 7, ep, 21);
            if (COL(ep) != 7 && color[ep + 9] == LIGHT && piece[ep + 9] == PAWN)
                gen_push(ep + 9, ep, 21);
        }
        else {
            if (COL(ep) != 0 && color[ep - 9] == DARK && piece[ep - 9] == PAWN)
                gen_push(ep - 9, ep, 21);
            if (COL(ep) != 7 && color[ep - 7] == DARK && piece[ep - 7] == PAWN)
                gen_push(ep - 7, ep, 21);
        }
    }
}


/* gen_push() puts a move on the move stack, unless it's a
   pawn promotion that needs to be handled by gen_promote().
   It also assigns a score to the move for alpha-beta move
   ordering. If the move is a capture, it uses MVV/LVA
   (Most Valuable Victim/Least Valuable Attacker). Otherwise,
   it uses the move's history heuristic value. Note that
   1,000,000 is added to a capture move's score, so it
   always gets ordered above a "normal" move. */

void gen_push(int from, int to, int bits)
{
    gen_t *g;
    
    if (bits & 16) {
        if (side == LIGHT) {
            if (to <= H8) {
                gen_promote(from, to, bits);
                return;
            }
        }
        else {
            if (to >= A1) {
                gen_promote(from, to, bits);
                return;
            }
        }
    }
    g = &gen_dat[first_move[ply + 1]++];
    g->m.b.from = (char)from;
    g->m.b.to = (char)to;
    g->m.b.promote = 0;
    g->m.b.bits = (char)bits;
    if (color[to] != EMPTY)
        g->score = 1000000 + (piece[to] * 10) - piece[from];
    else
        g->score = history[from][to];
}


/* gen_promote() is just like gen_push(), only it puts 4 moves
   on the move stack, one for each possible promotion piece */

void gen_promote(int from, int to, int bits)
{
    int i;
    gen_t *g;
    
    for (i = KNIGHT; i <= QUEEN; ++i) {
        g = &gen_dat[first_move[ply + 1]++];
        g->m.b.from = (char)from;
        g->m.b.to = (char)to;
        g->m.b.promote = (char)i;
        g->m.b.bits = (char)(bits | 32);
        g->score = 1000000 + (i * 10);
    }
}


/* makemove() makes a move. If the move is illegal, it
   undoes whatever it did and returns FALSE. Otherwise, it
   returns TRUE. */

BOOL makemove(move_bytes m)
{
    
    /* test to see if a castle move is legal and move the rook
       (the king is moved with the usual move code later) */
    if (m.bits & 2) {
        int from, to;

        if (in_check(side))
            return FALSE;
        switch (m.to) {
            case 62:
                if (color[F1] != EMPTY || color[G1] != EMPTY ||
                        attack(F1, xside) || attack(G1, xside))
                    return FALSE;
                from = H1;
                to = F1;
                break;
            case 58:
                if (color[B1] != EMPTY || color[C1] != EMPTY || color[D1] != EMPTY ||
                        attack(C1, xside) || attack(D1, xside))
                    return FALSE;
                from = A1;
                to = D1;
                break;
            case 6:
                if (color[F8] != EMPTY || color[G8] != EMPTY ||
                        attack(F8, xside) || attack(G8, xside))
                    return FALSE;
                from = H8;
                to = F8;
                break;
            case 2:
                if (color[B8] != EMPTY || color[C8] != EMPTY || color[D8] != EMPTY ||
                        attack(C8, xside) || attack(D8, xside))
                    return FALSE;
                from = A8;
                to = D8;
                break;
            default:  /* shouldn't get here */
                from = -1;
                to = -1;
                break;
        }
        color[to] = color[from];
        piece[to] = piece[from];
        color[from] = EMPTY;
        piece[from] = EMPTY;
    }

    /* back up information so we can take the move back later. */
    hist_dat[hply].m.b = m;
    hist_dat[hply].capture = piece[(int)m.to];
    hist_dat[hply].castle = castle;
    hist_dat[hply].ep = ep;
    hist_dat[hply].fifty = fifty;
    hist_dat[hply].hash = hash;
    ++ply;
    ++hply;

    /* update the castle, en passant, and
       fifty-move-draw variables */
    castle &= castle_mask[(int)m.from] & castle_mask[(int)m.to];
    if (m.bits & 8) {
        if (side == LIGHT)
            ep = m.to + 8;
        else
            ep = m.to - 8;
    }
    else
        ep = -1;
    if (m.bits & 17)
        fifty = 0;
    else
        ++fifty;

    /* move the piece */
    color[(int)m.to] = side;
    if (m.bits & 32)
        piece[(int)m.to] = m.promote;
    else
        piece[(int)m.to] = piece[(int)m.from];
    color[(int)m.from] = EMPTY;
    piece[(int)m.from] = EMPTY;

    /* erase the pawn if this is an en passant move */
    if (m.bits & 4) {
        if (side == LIGHT) {
            color[m.to + 8] = EMPTY;
            piece[m.to + 8] = EMPTY;
        }
        else {
            color[m.to - 8] = EMPTY;
            piece[m.to - 8] = EMPTY;
        }
    }

    /* switch sides and test for legality (if we can capture
       the other guy's king, it's an illegal position and
       we need to take the move back) */
    side ^= 1;
    xside ^= 1;
    if (in_check(xside)) {
        takeback();
        return FALSE;
    }
    set_hash();
    return TRUE;
}


/* takeback() is very similar to makemove(), only backwards :)  */

void takeback()
{
    move_bytes m;

    side ^= 1;
    xside ^= 1;
    --ply;
    --hply;
    m = hist_dat[hply].m.b;
    castle = hist_dat[hply].castle;
    ep = hist_dat[hply].ep;
    fifty = hist_dat[hply].fifty;
    hash = hist_dat[hply].hash;
    color[(int)m.from] = side;
    if (m.bits & 32)
        piece[(int)m.from] = PAWN;
    else
        piece[(int)m.from] = piece[(int)m.to];
    if (hist_dat[hply].capture == EMPTY) {
        color[(int)m.to] = EMPTY;
        piece[(int)m.to] = EMPTY;
    }
    else {
        color[(int)m.to] = xside;
        piece[(int)m.to] = hist_dat[hply].capture;
    }
    if (m.bits & 2) {
        int from, to;

        switch(m.to) {
            case 62:
                from = F1;
                to = H1;
                break;
            case 58:
                from = D1;
                to = A1;
                break;
            case 6:
                from = F8;
                to = H8;
                break;
            case 2:
                from = D8;
                to = A8;
                break;
            default:  /* shouldn't get here */
                from = -1;
                to = -1;
                break;
        }
        color[to] = side;
        piece[to] = ROOK;
        color[from] = EMPTY;
        piece[from] = EMPTY;
    }
    if (m.bits & 4) {
        if (side == LIGHT) {
            color[m.to + 8] = xside;
            piece[m.to + 8] = PAWN;
        }
        else {
            color[m.to - 8] = xside;
            piece[m.to - 8] = PAWN;
        }
    }
}

/* open_book() opens the opening book file and initializes the random number
   generator so we play random book moves. */

int book_file;

void open_book()
{
    char *cp;
    cp = (char *) FLASH_BUFFER;
    read_user_sector(5);
    if (*cp == 0) {
        printf("Opening book missing.\n\r");
        book_file = 0;
    } else {
        book_file = 1;
    }    
}

int getline(char *st, char *src) {
    int cnt;
    
    cnt = 0;
    while (*src >= 0x20) {
        *st++ = *src++;
        cnt++;
    }
    return cnt;
}

/* book_move() returns a book move (in integer format) or -1 if there is no
   book move. */

int book_move()
{
    char line[256];
    char book_line[256];
    int i, j, m;
    int move[50];  /* the possible book moves */
    int count[50];  /* the number of occurrences of each move */
    int moves = 0;
    int total_count = 0;
    char *cp;
    int ix;

    if (!book_file || hply > 25)
        return -1;

    /* line is a string with the current line, e.g., "e2e4 e7e5 g1f3 " */
    line[0] = '\0';
    j = 0;
    for (i = 0; i < hply; ++i)
        j += sprintf(line + j, "%s ", move_str(hist_dat[i].m.b));

    /* compare line to each line in the opening book */
    cp = (char *) FLASH_BUFFER;
    while ((ix = getline(book_line, cp))) {
        cp += ix;
        if (book_match(line, book_line)) {

            /* parse the book move that continues the line */
            m = parse_move(&book_line[strlen(line)]);
            if (m == -1)
                continue;
            m = gen_dat[m].m.u;

            /* add the book move to the move list, or update the move's
               count */
            for (j = 0; j < moves; ++j)
                if (move[j] == m) {
                    ++count[j];
                    break;
                }
            if (j == moves) {
                move[moves] = m;
                count[moves] = 1;
                ++moves;
            }
            ++total_count;
        }
    }

    /* no book moves? */
    if (moves == 0)
        return -1;

    /* Think of total_count as the set of matching book lines.
       Randomly pick one of those lines (j) and figure out which
       move j "corresponds" to. */
    j = rand() % (unsigned int)total_count;
    for (i = 0; i < moves; ++i) {
        j -= count[i];
        if (j < 0)
            return move[i];
    }
    return -1;  /* shouldn't get here */
}


/* book_match() returns TRUE if the first part of s2 matches s1. */

BOOL book_match(char *s1, char *s2)
{
    int i;

    for (i = 0; i < (signed int)strlen(s1); ++i)
        if (s2[i] == '\0' || s2[i] != s1[i])
            return FALSE;
    return TRUE;
}

/* the board representation */
int color[64];  /* LIGHT, DARK, or EMPTY */
int piece[64];  /* PAWN, KNIGHT, BISHOP, ROOK, QUEEN, KING, or EMPTY */
int side;  /* the side to move */
int xside;  /* the side not to move */
int castle;  /* a bitfield with the castle permissions. if 1 is set,
                white can still castle kingside. 2 is white queenside.
                4 is black kingside. 8 is black queenside. */
int ep;  /* the en passant square. if white moves e2e4, the en passant
            square is set to e3, because that's where a pawn would move
            in an en passant capture */
int fifty;  /* the number of moves since a capture or pawn move, used
               to handle the fifty-move-draw rule */
int hash;  /* a (more or less) unique number that corresponds to the
              position */
int ply;  /* the number of half-moves (ply) since the
             root of the search tree */
int hply;  /* h for history; the number of ply since the beginning
              of the game */

/* gen_dat is some memory for move lists that are created by the move
   generators. The move list for ply n starts at first_move[n] and ends
   at first_move[n + 1]. */
gen_t *gen_dat;
//gen_t gen_dat[GEN_STACK];
int first_move[MAX_PLY];

/* the history heuristic array (used for move ordering) */
int **history;
//int history[64][64];

/* we need an array of hist_t's so we can take back the
   moves we make */
hist_t *hist_dat;
//hist_t hist_dat[HIST_STACK];

/* the engine will search for max_time milliseconds or until it finishes
   searching max_depth ply. */
int max_time;
int max_depth;

/* the time when the engine starts searching, and when it should stop */
int start_time;
int stop_time;

int nodes;  /* the number of nodes we've searched */

/* a "triangular" PV array; for a good explanation of why a triangular
   array is needed, see "How Computers Play Chess" by Levy and Newborn. */
move **pv;
//move pv[MAX_PLY][MAX_PLY];
int pv_length[MAX_PLY];
BOOL follow_pv;

/* random numbers used to compute hash; see set_hash() in board.c */
int hash_piece[2][6][64];  /* indexed by piece [color][type][square] */
int hash_side;
int hash_ep[64];

/* Now we have the mailbox array, so called because it looks like a
   mailbox, at least according to Bob Hyatt. This is useful when we
   need to figure out what pieces can go where. Let's say we have a
   rook on square a4 (32) and we want to know if it can move one
   square to the left. We subtract 1, and we get 31 (h5). The rook
   obviously can't move to h5, but we don't know that without doing
   a lot of annoying work. Sooooo, what we do is figure out a4's
   mailbox number, which is 61. Then we subtract 1 from 61 (60) and
   see what mailbox[60] is. In this case, it's -1, so it's out of
   bounds and we can forget it. You can see how mailbox[] is used
   in attack() in board.c. */

int mailbox[120] = {
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1,  0,  1,  2,  3,  4,  5,  6,  7, -1,
     -1,  8,  9, 10, 11, 12, 13, 14, 15, -1,
     -1, 16, 17, 18, 19, 20, 21, 22, 23, -1,
     -1, 24, 25, 26, 27, 28, 29, 30, 31, -1,
     -1, 32, 33, 34, 35, 36, 37, 38, 39, -1,
     -1, 40, 41, 42, 43, 44, 45, 46, 47, -1,
     -1, 48, 49, 50, 51, 52, 53, 54, 55, -1,
     -1, 56, 57, 58, 59, 60, 61, 62, 63, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
     -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
};

int mailbox64[64] = {
    21, 22, 23, 24, 25, 26, 27, 28,
    31, 32, 33, 34, 35, 36, 37, 38,
    41, 42, 43, 44, 45, 46, 47, 48,
    51, 52, 53, 54, 55, 56, 57, 58,
    61, 62, 63, 64, 65, 66, 67, 68,
    71, 72, 73, 74, 75, 76, 77, 78,
    81, 82, 83, 84, 85, 86, 87, 88,
    91, 92, 93, 94, 95, 96, 97, 98
};


/* slide, offsets, and offset are basically the vectors that
   pieces can move in. If slide for the piece is FALSE, it can
   only move one square in any one direction. offsets is the
   number of directions it can move in, and offset is an array
   of the actual directions. */

BOOL slide[6] = {
    FALSE, FALSE, TRUE, TRUE, TRUE, FALSE
};

int offsets[6] = {
    0, 8, 4, 4, 8, 8
};

int offset[6][8] = {
    { 0, 0, 0, 0, 0, 0, 0, 0 },
    { -21, -19, -12, -8, 8, 12, 19, 21 },
    { -11, -9, 9, 11, 0, 0, 0, 0 },
    { -10, -1, 1, 10, 0, 0, 0, 0 },
    { -11, -10, -9, -1, 1, 9, 10, 11 },
    { -11, -10, -9, -1, 1, 9, 10, 11 }
};


/* This is the castle_mask array. We can use it to determine
   the castling permissions after a move. What we do is
   logical-AND the castle bits with the castle_mask bits for
   both of the move's squares. Let's say castle is 1, meaning
   that white can still castle kingside. Now we play a move
   where the rook on h1 gets captured. We AND castle with
   castle_mask[63], so we have 1&14, and castle becomes 0 and
   white can't castle kingside anymore. */

int castle_mask[64] = {
     7, 15, 15, 15,  3, 15, 15, 11,
    15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15,
    15, 15, 15, 15, 15, 15, 15, 15,
    13, 15, 15, 15, 12, 15, 15, 14
};


/* the piece letters, for print_board() */
char piece_char[6] = {
    'P', 'N', 'B', 'R', 'Q', 'K'
};


/* the initial board state */

int init_color[64] = {
    1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1,
    6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
};

int init_piece[64] = {
    3, 1, 2, 4, 5, 2, 1, 3,
    0, 0, 0, 0, 0, 0, 0, 0,
    6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6,
    0, 0, 0, 0, 0, 0, 0, 0,
    3, 1, 2, 4, 5, 2, 1, 3
};

#define DOUBLED_PAWN_PENALTY        10
#define ISOLATED_PAWN_PENALTY        20
#define BACKWARDS_PAWN_PENALTY        8
#define PASSED_PAWN_BONUS            20
#define ROOK_SEMI_OPEN_FILE_BONUS    10
#define ROOK_OPEN_FILE_BONUS        15
#define ROOK_ON_SEVENTH_BONUS        20


/* the values of the pieces */
int piece_value[6] = {
    100, 300, 300, 500, 900, 0
};

/* The "pcsq" arrays are piece/square tables. They're values
   added to the material value of the piece based on the
   location of the piece. */

int pawn_pcsq[64] = {
      0,   0,   0,   0,   0,   0,   0,   0,
      5,  10,  15,  20,  20,  15,  10,   5,
      4,   8,  12,  16,  16,  12,   8,   4,
      3,   6,   9,  12,  12,   9,   6,   3,
      2,   4,   6,   8,   8,   6,   4,   2,
      1,   2,   3, -10, -10,   3,   2,   1,
      0,   0,   0, -40, -40,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0
};

int knight_pcsq[64] = {
    -10, -10, -10, -10, -10, -10, -10, -10,
    -10,   0,   0,   0,   0,   0,   0, -10,
    -10,   0,   5,   5,   5,   5,   0, -10,
    -10,   0,   5,  10,  10,   5,   0, -10,
    -10,   0,   5,  10,  10,   5,   0, -10,
    -10,   0,   5,   5,   5,   5,   0, -10,
    -10,   0,   0,   0,   0,   0,   0, -10,
    -10, -30, -10, -10, -10, -10, -30, -10
};

int bishop_pcsq[64] = {
    -10, -10, -10, -10, -10, -10, -10, -10,
    -10,   0,   0,   0,   0,   0,   0, -10,
    -10,   0,   5,   5,   5,   5,   0, -10,
    -10,   0,   5,  10,  10,   5,   0, -10,
    -10,   0,   5,  10,  10,   5,   0, -10,
    -10,   0,   5,   5,   5,   5,   0, -10,
    -10,   0,   0,   0,   0,   0,   0, -10,
    -10, -10, -20, -10, -10, -20, -10, -10
};

int king_pcsq[64] = {
    -40, -40, -40, -40, -40, -40, -40, -40,
    -40, -40, -40, -40, -40, -40, -40, -40,
    -40, -40, -40, -40, -40, -40, -40, -40,
    -40, -40, -40, -40, -40, -40, -40, -40,
    -40, -40, -40, -40, -40, -40, -40, -40,
    -40, -40, -40, -40, -40, -40, -40, -40,
    -20, -20, -20, -20, -20, -20, -20, -20,
      0,  20,  40, -20,   0, -20,  40,  20
};

int king_endgame_pcsq[64] = {
      0,  10,  20,  30,  30,  20,  10,   0,
     10,  20,  30,  40,  40,  30,  20,  10,
     20,  30,  40,  50,  50,  40,  30,  20,
     30,  40,  50,  60,  60,  50,  40,  30,
     30,  40,  50,  60,  60,  50,  40,  30,
     20,  30,  40,  50,  50,  40,  30,  20,
     10,  20,  30,  40,  40,  30,  20,  10,
      0,  10,  20,  30,  30,  20,  10,   0
};

/* The flip array is used to calculate the piece/square
   values for DARK pieces. The piece/square value of a
   LIGHT pawn is pawn_pcsq[sq] and the value of a DARK
   pawn is pawn_pcsq[flip[sq]] */
int flip[64] = {
     56,  57,  58,  59,  60,  61,  62,  63,
     48,  49,  50,  51,  52,  53,  54,  55,
     40,  41,  42,  43,  44,  45,  46,  47,
     32,  33,  34,  35,  36,  37,  38,  39,
     24,  25,  26,  27,  28,  29,  30,  31,
     16,  17,  18,  19,  20,  21,  22,  23,
      8,   9,  10,  11,  12,  13,  14,  15,
      0,   1,   2,   3,   4,   5,   6,   7
};

/* pawn_rank[x][y] is the rank of the least advanced pawn of color x on file
   y - 1. There are "buffer files" on the left and right to avoid special-case
   logic later. If there's no pawn on a rank, we pretend the pawn is
   impossibly far advanced (0 for LIGHT and 7 for DARK). This makes it easy to
   test for pawns on a rank and it simplifies some pawn evaluation code. */
int pawn_rank[2][10];

int piece_mat[2];  /* the value of a side's pieces */
int pawn_mat[2];  /* the value of a side's pawns */

int eval()
{
    int i;
    int f;  /* file */
    int score[2];  /* each side's score */

    /* this is the first pass: set up pawn_rank, piece_mat, and pawn_mat. */
    for (i = 0; i < 10; ++i) {
        pawn_rank[LIGHT][i] = 0;
        pawn_rank[DARK][i] = 7;
    }
    piece_mat[LIGHT] = 0;
    piece_mat[DARK] = 0;
    pawn_mat[LIGHT] = 0;
    pawn_mat[DARK] = 0;
    for (i = 0; i < 64; ++i) {
        if (color[i] == EMPTY)
            continue;
        if (piece[i] == PAWN) {
            pawn_mat[color[i]] += piece_value[PAWN];
            f = COL(i) + 1;  /* add 1 because of the extra file in the array */
            if (color[i] == LIGHT) {
                if (pawn_rank[LIGHT][f] < ROW(i))
                    pawn_rank[LIGHT][f] = ROW(i);
            }
            else {
                if (pawn_rank[DARK][f] > ROW(i))
                    pawn_rank[DARK][f] = ROW(i);
            }
        }
        else
            piece_mat[color[i]] += piece_value[piece[i]];
    }

    /* this is the second pass: evaluate each piece */
    score[LIGHT] = piece_mat[LIGHT] + pawn_mat[LIGHT];
    score[DARK] = piece_mat[DARK] + pawn_mat[DARK];
    for (i = 0; i < 64; ++i) {
        if (color[i] == EMPTY)
            continue;
        if (color[i] == LIGHT) {
            switch (piece[i]) {
                case PAWN:
                    score[LIGHT] += eval_light_pawn(i);
                    break;
                case KNIGHT:
                    score[LIGHT] += knight_pcsq[i];
                    break;
                case BISHOP:
                    score[LIGHT] += bishop_pcsq[i];
                    break;
                case ROOK:
                    if (pawn_rank[LIGHT][COL(i) + 1] == 0) {
                        if (pawn_rank[DARK][COL(i) + 1] == 7)
                            score[LIGHT] += ROOK_OPEN_FILE_BONUS;
                        else
                            score[LIGHT] += ROOK_SEMI_OPEN_FILE_BONUS;
                    }
                    if (ROW(i) == 1)
                        score[LIGHT] += ROOK_ON_SEVENTH_BONUS;
                    break;
                case KING:
                    if (piece_mat[DARK] <= 1200)
                        score[LIGHT] += king_endgame_pcsq[i];
                    else
                        score[LIGHT] += eval_light_king(i);
                    break;
            }
        }
        else {
            switch (piece[i]) {
                case PAWN:
                    score[DARK] += eval_dark_pawn(i);
                    break;
                case KNIGHT:
                    score[DARK] += knight_pcsq[flip[i]];
                    break;
                case BISHOP:
                    score[DARK] += bishop_pcsq[flip[i]];
                    break;
                case ROOK:
                    if (pawn_rank[DARK][COL(i) + 1] == 7) {
                        if (pawn_rank[LIGHT][COL(i) + 1] == 0)
                            score[DARK] += ROOK_OPEN_FILE_BONUS;
                        else
                            score[DARK] += ROOK_SEMI_OPEN_FILE_BONUS;
                    }
                    if (ROW(i) == 6)
                        score[DARK] += ROOK_ON_SEVENTH_BONUS;
                    break;
                case KING:
                    if (piece_mat[LIGHT] <= 1200)
                        score[DARK] += king_endgame_pcsq[flip[i]];
                    else
                        score[DARK] += eval_dark_king(i);
                    break;
            }
        }
    }

    /* the score[] array is set, now return the score relative
       to the side to move */
    if (side == LIGHT)
        return score[LIGHT] - score[DARK];
    return score[DARK] - score[LIGHT];
}

int eval_light_pawn(int sq)
{
    int r;  /* the value to return */
    int f;  /* the pawn's file */

    r = 0;
    f = COL(sq) + 1;

    r += pawn_pcsq[sq];

    /* if there's a pawn behind this one, it's doubled */
    if (pawn_rank[LIGHT][f] > ROW(sq))
        r -= DOUBLED_PAWN_PENALTY;

    /* if there aren't any friendly pawns on either side of
       this one, it's isolated */
    if ((pawn_rank[LIGHT][f - 1] == 0) &&
            (pawn_rank[LIGHT][f + 1] == 0))
        r -= ISOLATED_PAWN_PENALTY;

    /* if it's not isolated, it might be backwards */
    else if ((pawn_rank[LIGHT][f - 1] < ROW(sq)) &&
            (pawn_rank[LIGHT][f + 1] < ROW(sq)))
        r -= BACKWARDS_PAWN_PENALTY;

    /* add a bonus if the pawn is passed */
    if ((pawn_rank[DARK][f - 1] >= ROW(sq)) &&
            (pawn_rank[DARK][f] >= ROW(sq)) &&
            (pawn_rank[DARK][f + 1] >= ROW(sq)))
        r += (7 - ROW(sq)) * PASSED_PAWN_BONUS;

    return r;
}

int eval_dark_pawn(int sq)
{
    int r;  /* the value to return */
    int f;  /* the pawn's file */

    r = 0;
    f = COL(sq) + 1;

    r += pawn_pcsq[flip[sq]];

    /* if there's a pawn behind this one, it's doubled */
    if (pawn_rank[DARK][f] < ROW(sq))
        r -= DOUBLED_PAWN_PENALTY;

    /* if there aren't any friendly pawns on either side of
       this one, it's isolated */
    if ((pawn_rank[DARK][f - 1] == 7) &&
            (pawn_rank[DARK][f + 1] == 7))
        r -= ISOLATED_PAWN_PENALTY;

    /* if it's not isolated, it might be backwards */
    else if ((pawn_rank[DARK][f - 1] > ROW(sq)) &&
            (pawn_rank[DARK][f + 1] > ROW(sq)))
        r -= BACKWARDS_PAWN_PENALTY;

    /* add a bonus if the pawn is passed */
    if ((pawn_rank[LIGHT][f - 1] <= ROW(sq)) &&
            (pawn_rank[LIGHT][f] <= ROW(sq)) &&
            (pawn_rank[LIGHT][f + 1] <= ROW(sq)))
        r += ROW(sq) * PASSED_PAWN_BONUS;

    return r;
}

int eval_light_king(int sq)
{
    int r;  /* the value to return */
    int i;

    r = king_pcsq[sq];

    /* if the king is castled, use a special function to evaluate the
       pawns on the appropriate side */
    if (COL(sq) < 3) {
        r += eval_lkp(1);
        r += eval_lkp(2);
        r += eval_lkp(3) / 2;  /* problems with pawns on the c & f files
                                  are not as severe */
    }
    else if (COL(sq) > 4) {
        r += eval_lkp(8);
        r += eval_lkp(7);
        r += eval_lkp(6) / 2;
    }

    /* otherwise, just assess a penalty if there are open files near
       the king */
    else {
        for (i = COL(sq); i <= COL(sq) + 2; ++i)
            if ((pawn_rank[LIGHT][i] == 0) &&
                    (pawn_rank[DARK][i] == 7))
                r -= 10;
    }

    /* scale the king safety value according to the opponent's material;
       the premise is that your king safety can only be bad if the
       opponent has enough pieces to attack you */
    r *= piece_mat[DARK];
    r /= 3100;

    return r;
}

/* eval_lkp(f) evaluates the Light King Pawn on file f */

int eval_lkp(int f)
{
    int r = 0;

    if (pawn_rank[LIGHT][f] == 6);  /* pawn hasn't moved */
    else if (pawn_rank[LIGHT][f] == 5)
        r -= 10;  /* pawn moved one square */
    else if (pawn_rank[LIGHT][f] != 0)
        r -= 20;  /* pawn moved more than one square */
    else
        r -= 25;  /* no pawn on this file */

    if (pawn_rank[DARK][f] == 7)
        r -= 15;  /* no enemy pawn */
    else if (pawn_rank[DARK][f] == 5)
        r -= 10;  /* enemy pawn on the 3rd rank */
    else if (pawn_rank[DARK][f] == 4)
        r -= 5;   /* enemy pawn on the 4th rank */

    return r;
}

int eval_dark_king(int sq)
{
    int r;
    int i;

    r = king_pcsq[flip[sq]];
    if (COL(sq) < 3) {
        r += eval_dkp(1);
        r += eval_dkp(2);
        r += eval_dkp(3) / 2;
    }
    else if (COL(sq) > 4) {
        r += eval_dkp(8);
        r += eval_dkp(7);
        r += eval_dkp(6) / 2;
    }
    else {
        for (i = COL(sq); i <= COL(sq) + 2; ++i)
            if ((pawn_rank[LIGHT][i] == 0) &&
                    (pawn_rank[DARK][i] == 7))
                r -= 10;
    }
    r *= piece_mat[LIGHT];
    r /= 3100;
    return r;
}

int eval_dkp(int f)
{
    int r = 0;

    if (pawn_rank[DARK][f] == 1);
    else if (pawn_rank[DARK][f] == 2)
        r -= 10;
    else if (pawn_rank[DARK][f] != 7)
        r -= 20;
    else
        r -= 25;

    if (pawn_rank[LIGHT][f] == 0)
        r -= 15;
    else if (pawn_rank[LIGHT][f] == 2)
        r -= 10;
    else if (pawn_rank[LIGHT][f] == 3)
        r -= 5;

    return r;
}

/* think() calls search() iteratively. Search statistics
   are printed depending on the value of output:
   0 = no output
   1 = normal output
   2 = xboard format output */

void think(int output)
{
    int i, j, x;

    /* try the opening book first */
    pv[0][0].u = book_move();
    if (pv[0][0].u != -1)
        return;

    /* some code that lets us longjmp back here and return
       from think() when our time is up */
    stop_search = FALSE;
    setjmp(errjmp);
    if (stop_search) {
        
        /* make sure to take back the line we were searching */
        while (ply)
            takeback();
        return;
    }

    start_time = get_ms();
    stop_time = start_time + max_time;

    ply = 0;
    nodes = 0;

    memset((unsigned char *)pv, 0, MAX_PLY * MAX_PLY * sizeof(move));
    memset((unsigned char *)history, 0, 64 * 64 * sizeof(int));
    if (output == 1)
        printf("ply      nodes  score  pv\n");
    for (i = 1; i <= max_depth; ++i) {
        follow_pv = TRUE;
        x = search(-10000, 10000, i);
        if (output == 1)
            printf("%3d  %9d  %5d ", i, nodes, x);
        else if (output == 2)
            printf("%d %d %d %d",
                    i, x, (get_ms() - start_time) / 10, nodes);
        if (output) {
            for (j = 0; j < pv_length[0]; ++j)
                printf(" %s", move_str(pv[0][j].b));
            printf("\n\r");
        }
        if (x > 9000 || x < -9000)
            break;
    }
}


/* search() does just that, in negamax fashion */

int search(int alpha, int beta, int depth)
{
    int i, j, x;
    BOOL c, f;

    /* we're as deep as we want to be; call quiesce() to get
       a reasonable score and return it. */
    if (!depth)
        return quiesce(alpha,beta);
    ++nodes;

    /* do some housekeeping every 1024 nodes */
    if ((nodes & 1023) == 0)
        checkup();

    pv_length[ply] = ply;

    /* if this isn't the root of the search tree (where we have
       to pick a move and can't simply return 0) then check to
       see if the position is a repeat. if so, we can assume that
       this line is a draw and return 0. */
    if (ply && reps())
        return 0;

    /* are we too deep? */
    if (ply >= MAX_PLY - 1)
        return eval();
    if (hply >= HIST_STACK - 1)
        return eval();

    /* are we in check? if so, we want to search deeper */
    c = in_check(side);
    if (c)
        ++depth;
    gen();
    if (follow_pv)  /* are we following the PV? */
        sort_pv();
    f = FALSE;

    /* loop through the moves */
    for (i = first_move[ply]; i < first_move[ply + 1]; ++i) {
        sort(i);
        if (!makemove(gen_dat[i].m.b))
            continue;
        f = TRUE;
        x = -search(-beta, -alpha, depth - 1);
        takeback();
        if (x > alpha) {

            /* this move caused a cutoff, so increase the history
               value so it gets ordered high next time we can
               search it */
            history[(int)gen_dat[i].m.b.from][(int)gen_dat[i].m.b.to] += depth;
            if (x >= beta)
                return beta;
            alpha = x;

            /* update the PV */
            pv[ply][ply] = gen_dat[i].m;
            for (j = ply + 1; j < pv_length[ply + 1]; ++j)
                pv[ply][j] = pv[ply + 1][j];
            pv_length[ply] = pv_length[ply + 1];
        }
    }

    /* no legal moves? then we're in checkmate or stalemate */
    if (!f) {
        if (c)
            return -10000 + ply;
        else
            return 0;
    }

    /* fifty move draw rule */
    if (fifty >= 100)
        return 0;
    return alpha;
}


/* quiesce() is a recursive minimax search function with
   alpha-beta cutoffs. In other words, negamax. It basically
   only searches capture sequences and allows the evaluation
   function to cut the search off (and set alpha). The idea
   is to find a position where there isn't a lot going on
   so the static evaluation function will work. */

int quiesce(int alpha,int beta)
{
    int i, j, x;

    ++nodes;

    /* do some housekeeping every 1024 nodes */
    if ((nodes & 1023) == 0)
        checkup();

    pv_length[ply] = ply;

    /* are we too deep? */
    if (ply >= MAX_PLY - 1)
        return eval();
    if (hply >= HIST_STACK - 1)
        return eval();

    /* check with the evaluation function */
    x = eval();
    if (x >= beta)
        return beta;
    if (x > alpha)
        alpha = x;

    gen_caps();
    if (follow_pv)  /* are we following the PV? */
        sort_pv();

    /* loop through the moves */
    for (i = first_move[ply]; i < first_move[ply + 1]; ++i) {
        sort(i);
        if (!makemove(gen_dat[i].m.b))
            continue;
        x = -quiesce(-beta, -alpha);
        takeback();
        if (x > alpha) {
            if (x >= beta)
                return beta;
            alpha = x;

            /* update the PV */
            pv[ply][ply] = gen_dat[i].m;
            for (j = ply + 1; j < pv_length[ply + 1]; ++j)
                pv[ply][j] = pv[ply + 1][j];
            pv_length[ply] = pv_length[ply + 1];
        }
    }
    return alpha;
}


/* reps() returns the number of times the current position
   has been repeated. It compares the current value of hash
   to previous values. */

int reps()
{
    int i;
    int r = 0;

    for (i = hply - fifty; i < hply; ++i)
        if (hist_dat[i].hash == hash)
            ++r;
    return r;
}


/* sort_pv() is called when the search function is following
   the PV (Principal Variation). It looks through the current
   ply's move list to see if the PV move is there. If so,
   it adds 10,000,000 to the move's score so it's played first
   by the search function. If not, follow_pv remains FALSE and
   search() stops calling sort_pv(). */

void sort_pv()
{
    int i;

    follow_pv = FALSE;
    for(i = first_move[ply]; i < first_move[ply + 1]; ++i)
        if (gen_dat[i].m.u == pv[0][ply].u) {
            follow_pv = TRUE;
            gen_dat[i].score += 10000000;
            return;
        }
}


/* sort() searches the current ply's move list from 'from'
   to the end to find the move with the highest score. Then it
   swaps that move and the 'from' move so the move with the
   highest score gets searched next, and hopefully produces
   a cutoff. */

void sort(int from)
{
    int i;
    int bs;  /* best score */
    int bi;  /* best i */
    gen_t g;

    bs = -1;
    bi = from;
    for (i = from; i < first_move[ply + 1]; ++i)
        if (gen_dat[i].score > bs) {
            bs = gen_dat[i].score;
            bi = i;
        }
    g = gen_dat[from];
    gen_dat[from] = gen_dat[bi];
    gen_dat[bi] = g;
}


/* checkup() is called once in a while during the search. */

void checkup()
{
    /* is the engine's time up? if so, longjmp back to the
       beginning of think() */
    if (get_ms() >= stop_time) {
        stop_search = TRUE;
        longjmp(errjmp, 1);
    }
}
