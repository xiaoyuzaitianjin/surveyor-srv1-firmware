#include "myfunc.h"
#include "print.h"

void myfunc() {
    unsigned char ch;
    
    ch = getch();
    switch (ch) {
        case 'h':   // test myfunc - string is "%h"
            printf("myfunc:  hello ! \n\r");
            break;
    }
    return;
}

