#include <cdefBF537.h>
#include "srv.h"
#include "uart.h"
#include "string.h"
#include "print.h"
#include "jpeg.h"

static unsigned char base64[64] = {
   'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
   'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
   'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
   'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};


char index_head[] = "<html>\n<head>\n<script language=\"JavaScript\">\n<!--\nvar time = null\nfunction refresh() {\nwindow.location.reload();\n}\nsetTimeout(\'refresh()\', 500)\n//-->\n</script>\n<title>SRV-1 Blackfin</title>\n</head>\n";
char index_body1[] = "<body>\n<h3>SRV-1 Blackfin</h3>\n<br>\n";
char index_jpeg1[] = "<img src=\"data:image/jpeg;base64,";
char index_jpeg2[] = "\" alt=\"SRV-1 Blackfin\" />\n<br>\n";
char index_body2[] = "<a href=\"/index.html\">reload</a>\n<br>\n</body>\n</html>\n";
#define BUFSIZE 256

void httpd()
{
    int i, ret, t0;
    char ch;
    unsigned char *cp, b0, b1, b2, b3;
    unsigned int image_size;
    unsigned char *output_start, *output_end; 
    static char buffer[BUFSIZE+1]; 

    buffer[0] = 'G';
    ret = 1;
    t0 = readRTC();
    while (((readRTC()-t0) < 1000) && (ret < BUFSIZE)){
        ch = getch();
        buffer[ret++] = ch;
        if (ch == '\n')
            break;
    }

    printf("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
    printf("%s", index_head);
    printf("%s", index_body1);
    printf("time = %d\r\n<br>", readRTC());
    printf("%s", index_jpeg1);
    grab_frame();
    output_start = (unsigned char *)JPEG_BUF;
    output_end = encode_image((unsigned char *)FRAME_BUF, output_start, quality, 
            FOUR_TWO_TWO, imgWidth, imgHeight); 
    image_size = (unsigned int)(output_end - output_start);
    led1_on();
    cp = (unsigned char *)JPEG_BUF;
    switch (image_size % 3) {  // pad image for base64 encoding
        case 0:
            break;
        case 1:
            *(cp + image_size - 1) = 0;
            image_size += 1;
            break;
        case 2:
            *(cp + image_size - 1) = 0;
            *(cp + image_size - 2) = 0;
            image_size += 2;
            break;
    } 

    for (i=0; i<image_size; i+=3) {
        b0 = ((*cp & 0xFC) >> 2); 
        b1 = ((*cp & 0x03) << 4) | ((*(cp+1) & 0xF0) >> 4); 
        b2 = ((*(cp+1) & 0x0F) << 2) | ((*(cp+2) & 0xC0) >> 6); 
        b3 = *(cp+2) & 0x3F;
        putchar(base64[b0]);
        putchar(base64[b1]);
        putchar(base64[b2]);
        putchar(base64[b3]);
        cp += 3;
    }
    printf("%s", index_jpeg2);
    printf("%s", index_body2);
    printf("\r\n\r\n");
}

