/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  httpd.c - HTTP GET and POST functions for the SRV-1 / SVS 
 *    Copyright (C) 2005-2009  Surveyor Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <cdefBF537.h>
#include "srv.h"
#include "uart.h"
#include "string.h"
#include "print.h"
#include "malloc.h"
#include "jpeg.h"
#include "debug.h"

static unsigned char base64[64] = {
   'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P',
   'Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f',
   'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v',
   'w','x','y','z','0','1','2','3','4','5','6','7','8','9','+','/'
};

extern void httpd_get();
extern void httpd_post();
extern int  clean_buffer(char *);
extern int  base64_camera_frame(char *, int);
void binary_camera_frame();


#define REQBUF_SIZE         1024
#define INLINEIMGBUF_SIZE   (64 * 1024)


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


static char inlineImgTag[] = "$$camera$$";

static char cgiBody[] = "0\r\n";

#define CONNECTION_HEADER   "Connection: Close\r\n"       // better in IE, Safari



void httpd_get()
{
    int i, ret, t0;
    char ch;
    static char reqBuf[REQBUF_SIZE+1]; 
    static char * inlineImgBuf = 0;
    int inlineImgLength = 0;
    char *method;
    char *path;
    char *protocol;
    char *body;
    int bodyLength = 0, contentLength;
    char * contentType = "text/html";
    int insertInlineImg = FALSE;
    char * inlineTagPtr = 0;


    //
    // Receive and parse the request
    //
    reqBuf[0] = 'G';
    ret = 1;
    t0 = readRTC();
    while (((readRTC()-t0) < 1000) && (ret < REQBUF_SIZE)){
        if (getchar((unsigned char *) &ch))
        {
            char pch = reqBuf[ret - 1];     // ret always >= 1
            reqBuf[ret++] = ch;
                            // Read to the end of the headers: handle any permutation of empty line EOL sequences
                            // Apparently some clients just terminate lines with LF
            if ((ret >= 4  &&  strncmp (reqBuf + ret - 4, "\r\n\r\n", 4) == 0)  ||
                (pch == 0x0d  &&  ch == 0x0d)  ||  (pch == 0x0a  &&  ch == 0x0a))
                break;
        }
    }
    reqBuf[ret] = 0;


    method = strtok(reqBuf, " ");
    path = strtok(0, " ");
    protocol = strtok(0, "\r");
DebugStr ("httpd_get enter: method="); DebugStr (method); DebugStr (" path="); DebugStr (path); DebugStr ("\r\n");
    if (!method || !path || !protocol) 
        goto exit;
    //printf("method: %s   path: %s   protocol: %s\r\n", method, path, protocol);
    if (strcmp(method, "GET") != 0) {
DebugStr ("http_get 501 method=");
DebugStr (method);
DebugStr ("\r\n");
        static char Body501[] = "Method not supported\r\n";
        printf ("HTTP/1.1 501 Method not supported\r\n"
                "Content-Type: text/html\r\n"
                "Content-Length: %d\r\n"
                "Connection: close\r\n"
                "\r\n"
                "%s",
                sizeof (Body501) - 1, Body501);
        goto exit;
    }

    //
    // Camera image binary - robot.jpg
    //
    if (strncmp(path, "/robot.jpg", 10) == 0) {
         binary_camera_frame();
         goto exit;
    }

    //
    // Robot control - robot.cgi
    //
    else if (strncmp(path, "/robot.cgi?", 11) == 0) {
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
                if (!pwm1_init) {
                    initPWM();
                    pwm1_init = 1;
                    pwm1_mode = PWM_PWM;
                    base_speed = 40;
                    lspeed = rspeed = 0;
                }
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

        body = cgiBody;
        contentLength = sizeof (cgiBody) - 1;
        contentType = "text/plain";
    }

    //
    // HTML from flash
    //
    else {
        char * cp;
        if ((strcmp(path, "/") == 0) || (strcmp(path, "/index.html") == 0))
            i = 0;
        else {
            for (i=0; i< sizeof(names) / 4; i++) 
                if (strcmp(path, names[i]) == 0)
                    break;
            if (i == sizeof(names) / 4) {
                static char Body404[] = "File not found\r\n";
                printf ("HTTP/1.1 404 File not found\r\n"
                        "Content-Type: text/html\r\n"
                        "Content-Length: %d\r\n"
                        "Connection: close\r\n"
                        "\r\n"
                        "%s",
                        sizeof (Body404) - 1, Body404);
                goto exit;
            }
        }
        read_double_sector((i*2) + 10, 1);  // set quiet flag
        body = cp = (char *) FLASH_BUFFER;
        bodyLength = contentLength = clean_buffer(cp);  // last character of html file should be '>'.  clean out anything else
    
        //
        // See if the inline image tag is present and generate the image if needed
        //
        insertInlineImg = FALSE;
        while ((*cp != 0) && (cp < (char *)(FLASH_BUFFER+0x00020000))) {
            if ((*cp == '$') && (*(cp+1) == '$')) {
                if (strncmp(cp, inlineImgTag, sizeof (inlineImgTag) - 1) == 0) {
                    insertInlineImg = TRUE;
                    inlineTagPtr = cp;
                    if (inlineImgBuf == 0)
                        inlineImgBuf = malloc (INLINEIMGBUF_SIZE);
                    inlineImgLength = base64_camera_frame (inlineImgBuf, INLINEIMGBUF_SIZE);
                    contentLength += inlineImgLength - sizeof (inlineImgTag) + 1;
                    break;
                }
            }
            ++cp;
        }
    }

    //
    // Send response headers
    //
    printf ("HTTP/1.1 200 OK\r\n"
            "Content-Type: %s\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            CONNECTION_HEADER
            "Content-Length: %d\r\n"
            "\r\n",
            contentType,
            contentLength);

    //
    // Send response body
    //
    if (insertInlineImg) {
        putchars ((unsigned char *) body, inlineTagPtr - body);
        putchars ((unsigned char *) inlineImgBuf, inlineImgLength);
        putchars ((unsigned char *) inlineTagPtr + sizeof (inlineImgTag) - 1, 
                    bodyLength - (inlineTagPtr - body - sizeof (inlineImgTag) + 1));
    }
    else
        putchars ((unsigned char *) body, contentLength);

exit:
DebugStr ("httpd_get exit\r\n");
}



int     base64_camera_frame (
char *  buf,
int     bufSize) {
    int i;
    unsigned char *cp, b0, b1, b2, b3;
    unsigned int image_size;
    unsigned char *output_start, *output_end; 
    int len;

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

    len = 0;
    for (i=0; i<image_size; i+=3) {
        if (len >= bufSize - 4)
            break;
        b0 = ((*cp & 0xFC) >> 2); 
        b1 = ((*cp & 0x03) << 4) | ((*(cp+1) & 0xF0) >> 4); 
        b2 = ((*(cp+1) & 0x0F) << 2) | ((*(cp+2) & 0xC0) >> 6); 
        b3 = *(cp+2) & 0x3F;
        *buf++ = base64[b0];
        *buf++ = base64[b1];
        *buf++ = base64[b2];
        *buf++ = base64[b3];
        cp += 3;
        len += 4;
    }
    return len;
}


void binary_camera_frame() {
     int i;
     unsigned int image_size;
     unsigned char *output_start, *output_end;
     unsigned char *cp;

     grab_frame();
     output_start = (unsigned char *)JPEG_BUF;
     output_end = encode_image((unsigned char *)FRAME_BUF,  output_start, quality,
                                FOUR_TWO_TWO, imgWidth, imgHeight);
     image_size = (unsigned int)(output_end - output_start);

#ifdef _DEBUG
char db[128];
sprintf (db, "Sending JPG %d bytes\r\n", image_size);
DebugStr (db);
#endif

     printf("HTTP/1.1 200 OK\r\n"
            "Content-Type: image/jpeg\r\n"
            "Content-Length: %d\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            CONNECTION_HEADER
            "\r\n",
            image_size);
     led1_on();
     cp = (unsigned char *)JPEG_BUF;
     for (i=0; i<image_size; i++)
         putchar(*cp++);
DebugStr ("Send JPG done\r\n");
}



void httpd_post() {
}



int clean_buffer(char *buf) {
    char *cp;
    
    for (cp = (buf+0x1FFFF); cp > buf; cp--) {  // sweep buffer from end clearing out garbage characters
        if (*cp == '>') { // final html character
            cp[1] = '\r';
            cp[2] = '\n';
            return cp - buf + 3;
        }
        else
            *cp = 0;
    }
    return 0;
}


