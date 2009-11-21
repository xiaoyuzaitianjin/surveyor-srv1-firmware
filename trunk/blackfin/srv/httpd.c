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
#include "stdlib.h"
#include "stm_m25p32.h"

#define REQBUF_SIZE             4096
#define INLINEIMGBUF_SIZE       (64 * 1024)

#define FLASH_SECTOR_SIZE       (64 * 1024)
#define FLASH_SECTORS_FIRST     4
#define FLASH_SECTORS_LAST      62
#define FLASH_SECTORS_UNIT      2

#define BOOT_LOADER_SECTORS     2
#define BOOT_LOADER_MIN_SIZE    (16 * 1024)
#define BOOT_LOADER_MAX_SIZE    (BOOT_LOADER_SECTORS * FLASH_SECTOR_SIZE)

char adminHtml[] =
#include "www/admin.html.c";

static char robotJpg[] = "/robot.jpg";
static char robotCgi[] = "/robot.cgi?";
static char adminPath[] = "/admin";

static char *fileNames[] = {  // index from file name to flash sector:  
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

#define CONNECTION_HEADER   "Connection: Close\r\n"       // better in IE, Safari

static char cgiBody[] = "0\r\n";
static char body404[] = "File not found\r\n";
static char body501[] = "Request format not supported\r\n";
static char resultCode404[] = "HTTP/1.1 404 File not found";
static char resultCode501[] = "HTTP/1.1 501 Request format not supported";

// Get the value of an HTTP header as a string
static  BOOL    getHdrString (              // Returns TRUE if header found
char *          hdrs,                       // String containing all headers
char *          hdrName,                    // Header name to find, including ": " delimiter
char *          value,                      // String buffer containing returned value. Returns an empty string
                                            //  (value[0] = 0) if header not found
int             valMaxChars)                // Size of value buffer in characters. If header value exceeds this size,
                                            //  FALSE is returned
{
    int valChars = 0;
    int hdrNameLen = strlen (hdrName);

    value[0] = 0;   // Return empty string if header not found

    // Scan lines in the headers
    while (*hdrs != 0) {
        // If header name matches, return the value
        if (strncmp (hdrs, hdrName, hdrNameLen) == 0) {
            hdrs += hdrNameLen;
            while (*hdrs != 0  &&  *hdrs != '\r'  &&  *hdrs != '\n') {
                if (valChars == valMaxChars - 1)
                    return FALSE;
                value[valChars++] = *hdrs++;
            }
            value[valChars] = 0;
            return TRUE;        
        }

        // Find the beginning of the next line
        else {
            while (*hdrs != 0  &&  *hdrs != '\r'  &&  *hdrs != '\n')
                ++hdrs;
            if (*hdrs != 0)
                while (*hdrs == '\r'  ||  *hdrs == '\n')
                    ++hdrs;
        }
    }
    return FALSE;
}


/* Get the value of an HTTP header as a decimal int */
static  BOOL    getHdrDecimal (             // Returns TRUE if header found
char *          hdrs,                       // String containing all headers
char *          hdrName,                    // Header name to find, including ": " delimiter
int *           value)                      // Out: returned header value. Unchanged if header not found
{
    char valStr[16];
    if (!getHdrString (hdrs, hdrName, valStr, countof (valStr)))
        return FALSE;
    *value = atoi (valStr);
    return TRUE;
}


/* Get the value of an HTTP "name=value" parameter from a header string or url parameter list
   Handles both header strings like:
      Content-Disposition: form-data; name="file1"; filename="Terrier1.jpg"
   and URL parameters like:
      path?name=file1&filename=Terrier1.jpg
   Removes any quotes surrounding value.*/
static  BOOL    getParam (      // Returns TRUE if paramName found; FALSE if not found or valMaxChars too small
char *          params,         // URL/header parameter string
char *          paramName,      // Name of parameter, including equals sign, as in "name="
char *          value,          // Out: value. Returns empty string if header not found or buffer too small.
int             valMaxChars)    // In: maximum size of value buffer, including null terminator
{
    value[0] = 0;
    char * targ = strstr (params, paramName);
    if (targ == NULL) {
        return FALSE;
    }

    BOOL quoted = FALSE;
    targ += strlen (paramName);
    if (*targ == '"') {
        quoted = TRUE;
        ++targ;
    }

    int valChars = 0;
    while (TRUE)
    {
        char c = *targ++;

        if (valChars >= valMaxChars - 1)
        {
            value[0] = 0;
            return FALSE;
        }
        if (c == 0)
            break;
        if (quoted)
        {
            if (c == '"'  ||  c == '\r'  ||  c == '\n')
                break;
        }
        else
        {
            if (c == ';'  ||  c == ' '  ||  c == '&'  ||  c == '\r'  ||  c == '\n')
                break;
        }
        value[valChars++] = c;
    }
    value[valChars] = 0;
    return TRUE;
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


/* Build the response body for the admin page*/
char *  adminBodyBuilder (      // Returns malloc-ed body string. Caller must free this. NULL if alloc error
char *  statusMsg,              // Status message string to display. NULL if none
char *  statusColor,            // Status message color as HTML color, e.g., "#ff0000". NULL if default
int *   contentLength)          // Out: length of response body
{
    static char statusMsgPat[] =   "$$statusMsg$$";
    static char statusColorPat[] = "$$statusColor$$";
    static char versionPat[] =     "$$version$$";
    static char defStatusColor[] = "#ffffff";

    *contentLength = countof (adminHtml) - 1
                        + (statusMsg == NULL ? 0 : strlen (statusMsg))
                        + (statusColor == NULL ? countof(defStatusColor) - 1 : strlen (statusColor))
                        + strlen ((char *) version_string);
    char * body = malloc (*contentLength + 1);
    memcpy ((unsigned char *) body, (unsigned char *) adminHtml, countof (adminHtml));
    if (body != NULL) {
        strReplace (body, *contentLength, statusMsgPat, statusMsg != NULL ? statusMsg : "");
        strReplace (body, *contentLength, statusColorPat, statusColor != NULL ? statusColor : defStatusColor);
        strReplace (body, *contentLength, versionPat, (char *) version_string);
        *contentLength -= countof (statusMsgPat) - 1 + countof (statusColorPat) - 1 + countof (versionPat) - 1;
    }
    return body;
}


void    httpd_request (char firstChar)
{
    char *reqBuf = (char *)HTTP_BUFFER; 

    int ret, t0, ix, x1, x2;
    char ch;

    // request fields
    char * method;
    char * path;
    char * protocol;
    char * headers;

    // response fields
    char * body = NULL;
    BOOL freeBody = FALSE;
    char * resultCode = "HTTP/1.1 200 OK";
    char * contentType = "text/html";
    int contentLength = 0, flashContentLength = 0;

    BOOL deferredReset = FALSE;

    // Receive the request and headers
    reqBuf[0] = firstChar;

    // GET method
    //if (strcmp (method, "GET") == 0) {
    if (reqBuf[0] == 'G') {
        ret = 1;
        t0 = readRTC();
        while (readRTC() - t0 < 1000  &&  ret < REQBUF_SIZE) {
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
        reqBuf[ret+1] = 0;  // for benefit of serial_out_httpbuffer(), which has to handle nulls inserted by strtok()

        // Parse the request fields
        method = strtok(reqBuf, " ");
        path = strtok(0, " ");
        protocol = strtok(0, "\r\n");
        headers = protocol + strlen (protocol) + 1;
        while (*headers != 0  &&  (*headers == '\r'  ||  *headers == '\n'))
            ++headers;        

        if (!method || !path || !protocol) 
            goto exit;

        // Camera image binary - robot.jpg
        if (strncmp(path, robotJpg, countof (robotJpg) - 1) == 0) {
            grab_frame();
            led1_on();
            body = (char *) JPEG_BUF;
            contentLength = encode_image((unsigned char *)FRAME_BUF, (unsigned char *) body, quality,
                                        FOUR_TWO_TWO, imgWidth, imgHeight)
                            - (unsigned char *) body;
            contentType = "image/jpeg";
        }

        // Robot control - robot.cgi
        else if (strncmp(path, robotCgi, countof(robotCgi) - 1) == 0) {
            char * params = path + countof (robotCgi) - 1;
            char dtype = params[0];  // drive options - m=SRV1, x=SRV-4WD, s=ESC, t=tilt, l=laser
            char cmd1 = params[1];
            char cmd2 = params[2];
            switch (dtype) {
                case '$':  // Blackfin reset
                    if (cmd1 == '!')
                        deferredReset = TRUE;   // defer until after we send the response
                    break;
                case 'l':  // lasers
                    switch (cmd2) {
                        case '1':
                            *pPORTHIO |= 0x0280;
                            break;
                        case '0':
                            *pPORTHIO &= 0xFD7F;
                            break;
                    }
                    break;
                case 'm':  // SRV-1 Robot motor drive (PWM)
                    if ((cmd1<'1') || (cmd1>'9') || (cmd2<'1') || (cmd2>'9'))  // command out of range ?
                        break;
                    if (!pwm1_init) {
                        initPWM();
                        pwm1_init = 1;
                        pwm1_mode = PWM_PWM;
                        base_speed = 40;
                        lspeed = rspeed = 0;
                    }
                    lspeed = ((int)cmd1 - 0x35) * 100;
                    rspeed = ((int)cmd2 - 0x35) * 100;
                    setPPM1(lspeed, rspeed);
                    break;
                case 'x':  // SRV-4WD motor controller
                    if ((cmd1<'1') || (cmd1>'9') || (cmd2<'1') || (cmd2>'9'))  // command out of range ?
                        break;
                    if (xwd_init == 0) {
                        xwd_init = 1;
                        init_uart1(115200);
                        delayMS(10);
                    }
                    x1 = ((int)cmd1 - 0x35) * 31 - 1;
                    x2 = ((int)cmd2 - 0x35) * 31 - 1;
                    uart1SendChar('x');
                    uart1SendChar((char)x1);
                    uart1SendChar((char)x2);
                    break;
                case 's': // Sabertooth or equivalent PPM servo-based motor control
                    if ((cmd1<'1') || (cmd1>'9') || (cmd2<'1') || (cmd2>'9'))  // command out of range ?
                        break;
                    if (!pwm1_init) {
                        initPPM1();
                        pwm1_init = 1;
                        pwm1_mode = PWM_PPM;
                    }
                    x1 = ((int)cmd1 - 0x35) * 24 - 1;
                    x2 = ((int)cmd2 - 0x35) * 24 - 1;
                    setPPM1((char)x1, (char)x2);
                    break;
            }

            body = cgiBody;
            contentLength = countof (cgiBody) - 1;
            contentType = "text/plain";
        }

        // admin - built-in administration page
        else if (strncmp(path, adminPath, countof(adminPath) - 1) == 0) {
            body = adminBodyBuilder (NULL, NULL, &contentLength);
            freeBody = TRUE;
        }

        // Look up HTML from flash
        else {
            int fileIndex = countof(fileNames);     // assume no match

            // Look up the name
            if ((strcmp(path, "/") == 0) || (strcmp(path, "/index.html") == 0))
                fileIndex = 0;
            else {
                for (fileIndex = 0; fileIndex < countof(fileNames); ++fileIndex) 
                    if (strcmp(path, fileNames[fileIndex]) == 0)
                        break;
            }

            // If it's found, pull it out of flash
            if (fileIndex < countof(fileNames)) {
                char * cp;

                read_double_sector ((fileIndex*2) + 10, 1);  // set quiet flag
                body = cp = (char *) FLASH_BUFFER;
                flashContentLength = contentLength = clean_buffer(cp);  // last character of html file should be '>'.  clean out anything else
            }

            // Not found - 404
            else {
                body = body404;
                resultCode = resultCode404;
                contentLength = countof (body404) - 1;
            }
        }
    } // if GET method


    // POST method
    //else if (strcmp (method, "POST") == 0) {
    else if (reqBuf[0] == 'P') {
        ret = 1;
        t0 = readRTC();
        while (readRTC() - t0 < 1000  &&  ret < HTTP_BUFFER_SIZE) {
            if (getchar((unsigned char *) &ch)) {
                reqBuf[ret++] = ch;
                       // POST should end with a boundary terminated by "--"
                //if (ret >= 4  &&  strncmp (reqBuf + ret - 4, "--\r\n", 4) == 0)
                    //break;
                t0 = readRTC();
            }
        }
        reqBuf[ret] = 0;
        reqBuf[ret+1] = 0;  // for benefit of serial_out_httpbuffer(), which has to handle nulls inserted by strtok()

        // Parse the request fields
        method = strtok(reqBuf, " ");
        path = strtok(0, " ");
        protocol = strtok(0, "\r\n");
        headers = protocol + strlen (protocol) + 1;
        while (*headers != 0  &&  (*headers == '\r'  ||  *headers == '\n'))
            ++headers;        

        if (!method || !path || !protocol) 
            goto exit;

        char *reqBody = (char *)HTTP_BUFFER;
        int reqBodyCount = 0;
        int reqContentLength = -1;  // assume no Content-Length header
        static char reqContentType[128];

        // Assume error
        BOOL error = TRUE;
        body = body501;
        contentLength = countof (body501) - 1;
        resultCode = resultCode501;

        // Receive the request body
        getHdrDecimal (headers, "Content-Length: ", &reqContentLength);   // get Content-Length
        getHdrString (headers, "Content-Type: ", reqContentType, countof (reqContentType));

        for (ix=0; ix<ret; ix++) {
            if ((reqBuf[ix] == '\r') &&
              (reqBuf[ix+1] == '\n') &&
              (reqBuf[ix+2] == '\r') &&
              (reqBuf[ix+3] == '\n')) {
                ix += 4;
                while ((ix < ret) && (reqBuf[ix] != '-'))  // search for boundary start
                    ix++;
                reqBody = &reqBuf[ix];
                reqBodyCount = ret - ix;
                break;
            }
        }

        // admin -- Post to administration page
        if (strncmp(path, adminPath, countof(adminPath) - 1) == 0) {
            static char type[64];
            static char boundary[128];
            int boundaryLen;

            // Get the content type and boundary parameter
            type[0] = 0;
            char * typeEnd = strchr (reqContentType, ';');
            if (typeEnd != NULL) {
                strncpy (type, reqContentType, min (typeEnd - reqContentType, countof (type)));
                type[countof (type) - 1] = 0;
            }
            strcpy (boundary, "--");
            getParam (reqContentType, "boundary=", boundary + 2, countof (boundary) - 2);
            boundaryLen = strlen (boundary);

            // If everything is in order for an upload,
            if (reqContentLength  &&  strcmp (type, "multipart/form-data") == 0  &&  
                    boundaryLen > 0  &&  contentLength <= 0x00020000) {   // currently a 128kB limit on transfers

                // Init POST body fields
                char *fileBodyStart = NULL;
                int fileBodyLength = 0;
                enum { undefined, toSectors, toBootLoader } uploadDest = undefined;
                int sectorStart = -1;
                BOOL confirmBootLoader = FALSE;

                // Parse sections of the body
                if (strncmp (reqBody, boundary, boundaryLen) == 0)    // if boundary at start of body
                {
                    char * secStart = NULL;
                    char * secEnd = NULL;
                    char * secBody;
                    int bodyIndex = 0;

                    while (TRUE)
                    {
                        // Find the next boundary. If there was a prev boundary, process this section
                        secEnd = strnstr (reqBody + bodyIndex, boundary, reqBodyCount - bodyIndex);
                        if (secEnd == NULL)
                            break;
                        secEnd -= 2;    // back up to CRLF preceding boundary
                    
                        if (secStart != NULL  &&
                            (secBody = strnstr (secStart, "\r\n\r\n", secEnd - secStart)) != NULL)
                        {
                            static char contDisp[128], nameParam[32];
                            *secBody = 0;   // terminate section headers
                            secBody += 4;

                            // Get Content-Disposition section header and the field name for this section
                            if (getHdrString (secStart, "Content-Disposition: ", contDisp, countof (contDisp)) &&
                                getParam (contDisp, "name=", nameParam, countof (nameParam))) {
                                static char val[32];

                                // fileItem1: the file being uploaded
                                if (strcmp (nameParam, "fileItem1") == 0) {
                                    fileBodyStart = secBody;
                                    fileBodyLength = secEnd - secBody;
                                }

                                // uploadRadios: the upload destination
                                else if (strcmp (nameParam, "uploadRadios") == 0) {
                                    static char toSectorsStr[] = "toSectors", toBootLoaderStr[] = "toBootLoader";
                                    if (strncmp (secBody, toSectorsStr, countof (toSectorsStr) - 1) == 0)
                                        uploadDest = toSectors;
                                    else if (strncmp (secBody, toBootLoaderStr, countof (toBootLoaderStr) - 1) == 0)
                                        uploadDest = toBootLoader;                                           
                                }

                                // sectorStart
                                else if (strcmp (nameParam, "sectorStart") == 0  &&  secEnd - secBody > 0) {
                                    strncpy (val, secBody, min (countof (val), secEnd - secBody));
                                    val[countof (val) - 1] = 0;
                                    sectorStart = atoi (val);
                                    if (sectorStart < FLASH_SECTORS_FIRST  ||  sectorStart > FLASH_SECTORS_LAST)
                                        sectorStart = -1;
                                }

                                // confirmBootLoader
                                else if (strcmp (nameParam, "confirmBootLoader") == 0) {
                                    if (strncmp (secBody, "on", 2) == 0  ||  strncmp (secBody, "1", 1) == 0)
                                        confirmBootLoader = TRUE;
                                }
                            }                                    
                        } // if section to process
                        
                        secStart = secEnd + boundaryLen + 2;   // start of the next section is after end of last

                        // If we're at the end boundary, we're done
                        if (secStart[0] == '-'  &&  secStart[1] == '-')
                            break;
                        secStart += 2; // skip CRLFs after boundary
                        bodyIndex = secStart - reqBody;
                    } // while parsing request body
                } // if first boundary

                 // Validate the upload, then flash the fugger
                static char resultMsg[128];
                resultMsg[0] = 0;
                char * resultColor = "#ffa0a0";     // assume error: red
                if (reqBodyCount != reqContentLength)
                    sprintf (resultMsg, "bodyCount (%d) != Content-Length (%d)", reqBodyCount, reqContentLength);
                else if (fileBodyStart == NULL  ||  fileBodyLength == 0)
                    sprintf (resultMsg, "No file received");
                else if (uploadDest == undefined)
                    sprintf (resultMsg, "No upload destination specified");

                else if (uploadDest == toSectors  &&  sectorStart == -1)
                    sprintf (resultMsg, "Valid starting sector not specified");
                else if (uploadDest == toSectors  &&  fileBodyLength > FLASH_SECTORS_UNIT * FLASH_SECTOR_SIZE)
                    sprintf (resultMsg, "File size (%d) too big for %d sectors", fileBodyLength, FLASH_SECTORS_UNIT);

                else if (uploadDest == toBootLoader  &&  !confirmBootLoader)
                    sprintf (resultMsg, "No boot loader confirmation");
                else if (uploadDest == toBootLoader  &&  
                         (fileBodyLength < BOOT_LOADER_MIN_SIZE  ||  fileBodyLength > BOOT_LOADER_MAX_SIZE))
                    sprintf (resultMsg, "Boot loader file size (%d) too big or too small", fileBodyLength);
                else if (uploadDest == toBootLoader  &&
                         getUnaligned32 (fileBodyStart) != 0xFFA00000  &&  
                         getUnaligned32 (fileBodyStart) != 0xFF800000) {
                    sprintf (resultMsg, "Boot loader first-word signature (0x%08x) wrong", 
                                getUnaligned32 (fileBodyStart));
                }
                else {
                    // Set destination and flash write size based on parameters
                    int flashAddress, bytesToWrite;
                    if (uploadDest == toSectors)
                    {
                        flashAddress = sectorStart * FLASH_SECTOR_SIZE;
                        bytesToWrite = FLASH_SECTORS_UNIT * FLASH_SECTOR_SIZE;
                    }
                    else
                    {
                        flashAddress = BOOT_SECTOR;
                        bytesToWrite = BOOT_LOADER_SECTORS * FLASH_SECTOR_SIZE;
                    }

                    // Move the file down to the beginning of FLASH_BUFFER and pad the sectors with zeroes
                    memmove ((void *) FLASH_BUFFER, fileBodyStart, fileBodyLength);
                    memset ((void *) (FLASH_BUFFER + fileBodyLength), 0, bytesToWrite - fileBodyLength);
#if 1
                    if (spi_write (flashAddress, (unsigned char *) FLASH_BUFFER,
                                    (unsigned char *) FLASH_BUFFER + bytesToWrite, bytesToWrite) != bytesToWrite)
                    {
                        sprintf (resultMsg, "Flash write failed. Error code lost in lower-level code. :)");
                        error = FALSE;
                    }
                    else
#endif
                    {
                        resultColor = "#c0ffc0";        // green
                        if (uploadDest == toSectors)
                            sprintf (resultMsg, "%d bytes uploaded to sector %d", fileBodyLength, sectorStart);
                        else
                            sprintf (resultMsg, "%d bytes uploaded to boot loader", fileBodyLength);
                    }
                }

                // Form our response
                body = adminBodyBuilder (resultMsg, resultColor, &contentLength);
                freeBody = TRUE;

                // Unless we got a valid file, clear the flash buffer
                if (error)
                    memset ((void *) FLASH_BUFFER, 0, FLASH_BUFFER_SIZE);
            } // if starting boundary
        } // if uploadFlash
    } // if POST method


    // Unknown method
    else {
        body = body501;
        contentLength = countof (body501) - 1;
        resultCode = resultCode501;
    }

    // Send response headers
    printf ("%s\r\n"
            "Content-Type: %s\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            CONNECTION_HEADER
            "Content-Length: %d\r\n"
            "\r\n",
            resultCode,
            contentType,
            contentLength);

    // Send response body
#if 0
    if (insertInlineImg) {
        putchars ((unsigned char *) body, inlineTagPtr - body);
        putchars ((unsigned char *) inlineImgBuf, inlineImgLength);
        putchars ((unsigned char *) inlineTagPtr + sizeof (inlineImgTag) - 1, 
                    flashContentLength - (inlineTagPtr - body - sizeof (inlineImgTag) + 1));
    }
    else
#endif
    {
        if (body != NULL)
            putchars ((unsigned char *) body, contentLength);
    }

    // If the response body was malloc-ed, free it
    if (freeBody  &&  body != NULL)
        free (body);

    // If a reset request came in, reset
    if (deferredReset)
    {
        reset_cpu();
    }

exit:
    return;
}

