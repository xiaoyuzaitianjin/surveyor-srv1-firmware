#include <cdefBF537.h>
#include "srv.h"
#include "uart.h"
#include "string.h"
#include "print.h"
#include "jpeg.h"

char index_html[] = "<html>\n<head>\n<title>SRV-1 Blackfin</title>\n</head>\n<body>\n<h3>SRV-1 Blackfin</h3>\n<br>\n<img src=\"/robot.jpg\">\n</body>\n</html>\n";

#define BUFSIZE 256

struct {
    char *ext;
    char *filetype;
} extensions [] = {
    {"gif", "image/gif" },  
    {"jpg", "image/jpeg"}, 
    {"jpeg","image/jpeg"},
    {"png", "image/png" },  
    {"htm", "text/html" },  
    {"html","text/html" },  
    {0,0} };

void httpd()
{
    int j, buflen, len;
    int i, ret, t0;
    char ch, *fstr;
    unsigned char *cp;
    unsigned int image_size;
    unsigned char *output_start, *output_end; 
    static char buffer[BUFSIZE+1]; 

    buffer[0] = 'G';
    ret = 1;
    t0 = readRTC();
    while ((readRTC()-t0) < 1000) {
        ch = getch();
        buffer[ret++] = ch;
        if (ch == '\n')
            break;
    }

    if(ret > 0 && ret < BUFSIZE)    /* return code is valid chars */
        buffer[ret]=0;      /* terminate the buffer */
    else 
        buffer[0]=0;

    for(i=0;i<ret;i++)      /* remove CF and LF characters */
        if(buffer[i] == '\r' || buffer[i] == '\n')
        	buffer[i]='*';

    if( strncmp(buffer,"GET ",4) && strncmp(buffer,"get ",4) )
        return;

    for(i=4;i<BUFSIZE;i++) { /* null terminate after the second space to ignore extra stuff */
        if(buffer[i] == ' ') { /* string is "GET URL " +lots of other stuff */
            buffer[i] = 0;
            break;
        }
    }

    for(j=0;j<i-1;j++)      /* check for illegal parent directory use .. */
        if(buffer[j] == '.' && buffer[j+1] == '.')
            return;

    if( !strncmp(&buffer[0],"GET /\0",6) || !strncmp(&buffer[0], "get /\0",6) ) /* convert no filename to index file */
        (void)strcpy(buffer,"GET /index.html");

    /* work out the file type and check we support it */
    buflen=strlen(buffer);
    fstr = (char *)0;
    for(i=0;extensions[i].ext != 0;i++) {
          len = strlen(extensions[i].ext);
          if( !strncmp(&buffer[buflen-len], extensions[i].ext, len)) {
            fstr =extensions[i].filetype;
            break;
        }
    }
    if(fstr == 0) 
        return;

    if ((i==1) || (i==2)) {
        grab_frame();
        output_start = (unsigned char *)JPEG_BUF;
        output_end = encode_image((unsigned char *)FRAME_BUF, output_start, quality, 
                FOUR_TWO_TWO, imgWidth, imgHeight); 
        image_size = (unsigned int)(output_end - output_start);
        //printf("HTTP/1.0 200 OK\r\nContent-Type: %s\r\n\r\n", fstr);
        printf("HTTP/1.1 200 OK\r\nAccept-Ranges: bytes\n\rContent-Length: %d\n\rContent-Type: %s\r\nConnection: close\r\n\r\n", image_size, fstr);
        led1_on();
        cp = (unsigned char *)JPEG_BUF;
        for (i=0; i<image_size; i++) 
            putchar(*cp++);
    } else {      
        printf("HTTP/1.0 200 OK\r\nContent-Type: %s\r\n\r\n", fstr);
        printf("%s", index_html);
    }
    printf("\r\n\r\n");
}
