#include "../interpreter.h"
#include "../picoc.h"

/* mark where to end the program for platforms which require this */
int PicocExitBuf[41];

/* deallocate any storage */
void PlatformCleanup()
{
}

/* get a line of interactive input */
char *PlatformGetLine(char *Buf, int MaxLen, const char *Prompt)
{
    unsigned int ix;
    char ch;
    
    printf(Prompt);

    ix = 0;
    
    // If the first character is \n or \r, eat it
    ch = getch();
    if (ch == '\n' || ch == '\r')
    {
        // And get the next character
        ch = getch();
    }
    
    while (ix < MaxLen) 
    {
        // ESC character or ctrl-c (to avoid problem with TeraTerm) - exit
        if (ch == 0x1B)
        { 
            printf("LLeaving PicoC\n");
            return NULL;
        }
        else if (ch == 0x03) 
        { 
            printf("Leaving PicoC\n");
            return NULL;
        }
        // Backspace character has to be handled special
        else if (ch == 0x08)
        {
            // Remove the latest character from our buffer
            if (ix > 0)
            {
                // Send a space and then backspace again
                putchar(' ');
                putchar(0x08);
                Buf[ix] = 0x00;
                ix--;
            }
        }
        else if (ch == '\n') 
        {
            Buf[ix] = ch;  // if newline, send newline character followed by null
            ix++;
            Buf[ix] = 0;
            
            return Buf;
        }
        else
        {
            // Handle the normal storage of the byte
            Buf[ix] = ch;
            ix++;
        }
        ch = getch();
    }
    return NULL;
}

/* write a character to the console */
void PlatformPutc(unsigned char OutCh, union OutputStreamInfo *Stream)
{
    if (OutCh == '\n')
        putchar('\r');
        
    putchar(OutCh);
}

/* read a character */
int PlatformGetCharacter()
{
    return getch();
}

/* exit the program */
void PlatformExit(int RetVal)
{
    PicocExitValue = RetVal;
    PicocExitBuf[40] = 1;
    longjmp(PicocExitBuf, 1);
}

