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

extern void httpd_get();
extern void httpd_post();
extern void clean_buffer(char *);
extern void base64_camera_frame();

#define BUFSIZE 256

char *names[] = {  // index from file name to flash sector:  
    "/00.html",  // sector 10-11
    "/01.html",  // sector 12-13
    "/02.html",  // sector 14-15
    "/03.html",  // sector 16-17
    "/04.html",  // sector 18-19
    "/05.html",  // sector 20-21
    "/06.html",  // sector 22-23
    "/07.html",  // sector 24-25
    "/08.html",  // sector 26-27
    "/09.html"   // sector 28-29
};

void httpd_get()
{
    int i, j, ret, t0;
    char ch;
    char *cp;
    static char buffer[BUFSIZE+1]; 
    char *method;
    char *path;
    char *protocol;

    buffer[0] = 'G';
    ret = 1;
    t0 = readRTC();
    while (((readRTC()-t0) < 1000) && (ret < BUFSIZE)){
        ch = getch();
        buffer[ret++] = ch;
        if (ch == '\n')
            break;
    }

    method = strtok(buffer, " ");
    path = strtok(0, " ");
    protocol = strtok(0, "\r");
    if (!method || !path || !protocol) 
        return;
    //printf("method: %s   path: %s   protocol: %s\r\n", method, path, protocol);

    if (strcmp(method, "GET") != 0) {
        printf("HTTP/1.1 501 Not supported\r\nMethod is not supported.\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
        return;
    }
    
    j = sizeof(names) / 4;  // compute number of entries in names[] table
    
    if ((strcmp(path, "/") == 0) || (strcmp(path, "/index.html") == 0)) {
        i = 0;
    } else if (strncmp(path, "/robot.cgi?", 11) == 0) {
        switch(path[11]) {
            case 'l':
                *pPORTHIO |= 0x0280;
                break;
            case 'L':
                *pPORTHIO &= 0xFD7F;
                break;
            case '4':
            case '8':
            case '6':
            case '0':
            case '5':
            case '.':
            case '1':
            case '2':
            case '3':
                pwm1_mode = PWM_PWM;
                if (base_speed == 0)
                    base_speed = 40;
                motor_set(path[11], base_speed, &lspeed, &rspeed);
                break;
            case '+':
                base_speed += 10;
                if (base_speed > 90)
                    base_speed = 90;
                motor_set(path[11], base_speed, &lspeed, &rspeed);
                break;
            case '-':
                base_speed -= 10;
                if (base_speed < 20)
                    base_speed = 20;
                motor_set(path[11], base_speed, &lspeed, &rspeed);
                break;
        }
        i = 0;
    } else {
        for (i=0; i<j; i++) 
            if (strcmp(path, names[i]) == 0)
                break;
    }
    
    if (i == j) {
        printf("HTTP/1.1 404 Not found\r\nFile not found.\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
        return;
    }

    read_double_sector((i*2) + 10, 1);  // set quiet flag
    cp = FLASH_BUFFER;
    clean_buffer(cp);  // last character of html file should be '<'.  clean out anything else

    printf("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n");
    while ((*cp != 0) && (cp < (unsigned char *)(FLASH_BUFFER+0x00020000))) {
        if ((*cp == '$') && (*(cp+1) == '$')) {
            if (strncmp(cp, "$$camera$$", 10) == 0) {
                base64_camera_frame();
                cp+=10;
            }
        }
        putchar(*cp++);
    }
}

void base64_camera_frame() {
    int i;
    unsigned char *cp, b0, b1, b2, b3;
    unsigned int image_size;
    unsigned char *output_start, *output_end; 

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
}

void httpd_post() {
}

void clean_buffer(char *buf) {
    char *cp;
    
    for (cp = (buf+0x1FFFF); cp > buf; cp--) {  // sweep buffer from end clearing out garbage characters
        if (*cp == '>')  // final html character
            return;
        else
            *cp = 0;
    }
}


