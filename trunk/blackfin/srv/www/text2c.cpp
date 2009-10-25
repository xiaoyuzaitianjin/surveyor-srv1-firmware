// text2c.cpp 
//

#include "stdafx.h"
#include <stdio.h>


int main(int argc, char* argv[])
{
    bool eol = true;

    int ch;
    while ((ch = getchar()) != EOF) {
        if (eol) {
            printf ("\"");
            eol = false;
        }
        if (ch == '"')
            printf ("\\\"");
        else if (ch == '\\')
            printf ("\\\\");
        else if (ch == '\n') {
            printf ("\\r\\n\"\n");
            eol = true;
        }
        else
            putchar (ch);
    }

	return 0;
}

