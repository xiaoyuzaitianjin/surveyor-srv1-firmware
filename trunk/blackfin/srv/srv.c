/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  srv.c - routines to interface with the SRV-1 Blackfin robot.
 *    modified from main.c - main control loop for SRV-1 robot
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
#include "config.h"
#include "uart.h"
#include "i2c.h"
#include "ov9655.h"
#include "ov7725.h"
#include "camera.h"
#include "jpeg.h"
#include "xmodem.h"
#include "stm_m25p32.h"
#include "font8x8.h"
#include "colors.h"
#include "malloc.h"
#include "spi.h"
#include "edit.h"
#include "print.h"
#include "string.h"
#include "neural.h"
#include "float.h"

#include "srv.h"

/* Size of frame */
unsigned int imgWidth, imgHeight;

/* Version */
unsigned char version_string[] = "SRV-1 Blackfin - "  __TIME__ " - " __DATE__ ;

/* Frame count output string */
unsigned char frame[] = "000-deg 000-f 000-d 000-l 000-r";
//unsigned char frame[] = "frame     ";

/* Camera globals */
unsigned int quality, framecount, ix, overlay_flag;
unsigned int segmentation_flag, edge_detect_flag, frame_diff_flag;
unsigned int edge_thresh;
unsigned char *output_start, *output_end; /* Framebuffer addresses */
unsigned int image_size; /* JPEG image size */
char imgHead[11]; /* image frame header for I command */
short hhpel[] = {0, -1, 0, 1, -1, 1, -1, 0, 1};
short vhpel[] = {0, -1, -1, -1, 0, 0, 1, 1, 1};

/* Motor globals */
int lspeed, rspeed, lspeed2, rspeed2, base_speed, base_speed2, err1;
int pwm1_mode, pwm2_mode, pwm1_init, pwm2_init;

/* IMU globals */
int x_acc, x_acc0, x_center;
int y_acc, y_acc0, y_center;

/* Failsafe globals */
int failsafe_mode = 0;
int lfailsafe, rfailsafe;
int failsafe_clock;

/* Sonar globals */
int sonar_data[5];

/* random number generator globals */
unsigned int rand_seed = 0x55555555;

/* General globals */
unsigned char *cp;
unsigned int i, j; // Loop counter.
unsigned int master;  // SVS master or slave ?
unsigned int uart1_flag = 0;
unsigned int thumbnail_flag = 0;

void init_io() {
    *pPORTGIO_DIR = 0x0300;   // LEDs (PG8 and PG9)
    *pPORTH_FER = 0x0000;     // set for GPIO
    *pPORTHIO_DIR |= 0x0040;  // set PORTH6 to output for serial flow control
    *pPORTHIO = 0x0000;       // set output low 
    *pPORTHIO_INEN |= 0x000D; // enable inputs: Matchport RTS0 (H0), battery (H2), master/slave (H3)
    *pPORTHIO_DIR |= 0x0380;  // set up lasers
    if (*pPORTHIO & 0x0008)   // check SVS master/slave bit
        master = 0;
    else
        master = 1;
    pwm1_mode = PWM_OFF;
    pwm2_mode = PWM_OFF;
    pwm1_init = 0;
    pwm2_init = 0;
}

/* reset CPU */
void reset_cpu() {
    asm(
    "p0.l = 0x0100; "
    "p0.h = 0xFFC0; "
    "r0.l = 0x0007; "
    "w[p0] = r0; "
    "ssync; "
    "p0.l = 0x0100; "
    "p0.h = 0xFFC0; "
    "r0.l = 0x0000; "
    "w[p0] = r0; "
    "ssync; "
    "raise 1; ");                        
}

/* clear SDRAM */
void clear_sdram() {
  for (cp=(unsigned char *)0x00100000; cp<(unsigned char *)0x02000000; cp++) {
    *cp = 0;
  }
}

void show_stack_ptr() {
    int x = 0;
    asm("%0 = SP;" : "=r"(x) : "0"(x));
    printf("stack_ptr = 0x%x\n\r", x);
    return;
}

unsigned int stack_remaining() {
    unsigned int x = 0;
    asm("%0 = SP" : "=r"(x) : "0"(x));
    return (x - (unsigned int)STACK_BOTTOM);
}

void show_heap_ptr() {
    printf("heap_ptr  = 0x%x\n\r", (int)heap_ptr);
}

/* SRV-1 Firmware Version Request
   Serial protocol char: V */
void serial_out_version () {
    printf("##Version - %s", version_string);
    if (master)  // check master/slave bit
        printf(" (stereo master)\n\r");     
    else
        printf(" (stereo slave)\n\r");     
}

/* Get current time
   Serial protocol char: t */
void serial_out_time () {
    printf("##time - millisecs:  %d\n\r", readRTC());
}

/* Dump flash buffer to serial
   Serial protocol char: z-d */
void serial_out_flashbuffer () {
    printf("##zdump: \n\r");
    cp = (unsigned char *)FLASH_BUFFER;
    for (i=0; i<0x10000; i++) {
        if (*cp == 0)
            return;
        if (*cp == 0x0A)
            putchar(0x0D);
        putchar(*cp++);
    }
}

/* Turn lasers on
   Serial protocol char: l */
void lasers_on () {
    *pPORTHIO |= 0x0380;
    printf("#l");
}

/* Turn lasers off
   Serial protocol char: L */
void lasers_off () {
    *pPORTHIO &= 0xFC7F;
    printf("#L");
}

/* Show laser range
   Serial protocol char: R */
void show_laser_range(int flag) {
    printf("##Range(cm) = %d\n\r", laser_range(flag));
}

/* Compute laser range 
    turn off lasers
    stop motors
    delay 250ms
    grab reference frame
    do right laser, then left laser
        turn on laser
        delay 250ms
        grab frame
        turn lasers off 
        compute difference
        set color #16
        use adaptive threshold to filter blobs
        range (inches) = imgWidth*2 / vblob(16)
    compare results, return best measurement
*/
unsigned int laser_range(int dflag) {
    unsigned int ix, rrange, lrange, rconf, lconf;
    
    rrange = lrange = 9999;
    rconf = lconf = 0;    // confidence measure
    
    *pPORTHIO &= 0xFC7F;    // lasers off
    if (pwm1_mode == PWM_PWM) setPWM(0, 0);    // motors off
    else setPPM1(50, 50);
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab reference frame
            (unsigned char *)FRAME_BUF2, imgWidth, imgHeight); 
    *pPORTHIO |= 0x0200;      // right laser on
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    compute_frame_diff((unsigned char *)FRAME_BUF,             // compute difference        
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    *pPORTHIO &= 0xFC7F;    // lasers off
    umin[16] = 80; umax[16]=144; vmin[16] = 100; vmax[16]=255; ymax[16]=255;   // set color bin #16
    for(ymin[16]=200; ymin[16]>0; ymin[16]-=10) {
        vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, 16);  // use the brightest blob
        if (dflag)
            printf("right blobs: ymin=%d   %d %d %d %d %d  %d %d %d %d %d  %d %d %d %d %d\n\r",
              ymin[16], 
              blobcnt[0], blobx1[0], blobx2[0], bloby1[0], bloby2[0],
              blobcnt[1], blobx1[1], blobx2[1], bloby1[1], bloby2[1],
              blobcnt[2], blobx1[2], blobx2[2], bloby1[2], bloby2[2]);
        ix = blobcnt[0];
        if (!ix)
            continue;
        if (blobx1[0] < (imgWidth/2)) // make certain that blob is on right
            continue;
        break;
    }
    rrange = (6*imgWidth) / (blobx2[0]+blobx1[0]-imgWidth+1); // right blob
    rconf = (100 * ix) / ((blobx2[0]-blobx1[0]+1) * (bloby2[0]-bloby1[0]+1));

    *pPORTHIO |= 0x0080;      // left laser on
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    compute_frame_diff((unsigned char *)FRAME_BUF,             // compute difference        
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    *pPORTHIO &= 0xFC7F;    // lasers off
    umin[16] = 80; umax[16]=144; vmin[16] = 100; vmax[16]=255; ymax[16]=255;   // set color bin #16
    for(ymin[16]=200; ymin[16]>0; ymin[16]-=10) {
        vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, 16);  // use the brightest blob
        if (dflag)
            printf("left blobs: ymin=%d   %d %d %d %d %d  %d %d %d %d %d  %d %d %d %d %d\n\r",
              ymin[16], 
              blobcnt[0], blobx1[0], blobx2[0], bloby1[0], bloby2[0],
              blobcnt[1], blobx1[1], blobx2[1], bloby1[1], bloby2[1],
              blobcnt[2], blobx1[2], blobx2[2], bloby1[2], bloby2[2]);
        ix = blobcnt[0];
        if (!ix)
            continue;
        if (blobx2[0] > (imgWidth/2)) // make certain that blob is on left
            continue;
        break;
    }
    lrange = (6*imgWidth) / (imgWidth-blobx2[0]-blobx1[0]+ 1); // left blob
    lconf = (100 * ix) / ((blobx2[0]-blobx1[0]+1) * (bloby2[0]-bloby1[0]+1));
    
    if (dflag)
        printf("lconf %d lrange %d rconf %d rrange %d\n\r", lconf, lrange, rconf, rrange);
    if ((lrange==9999) && (rrange==9999))
        return 9999;
    if (lconf > rconf)
        return lrange;
    return rrange;
}

void check_battery() { // 'D' command
    if (*pPORTHIO & 0x0004)
        printf("##D - low battery voltage detected\n\r");
    else
        printf("##D - battery voltage okay\n\r");
}

void led0_on() {
    *pPORTGIO = 0x0100;  // turn on LED0
}

void led1_on() {
    *pPORTGIO = 0x0200;  // turn on LED1
}

/* use GPIO H10 (pin 27), H11 (pin 28), H12 (pin 29), H13 (pin 30) as sonar inputs -
    GPIO H1 (pin 18) is used to trigger the sonar reading (low-to-high transition) */
void init_sonar() {  
    *pPORTHIO_INEN |= 0x3C00;  // enable H27, H28, H29, H30 as inputs
    *pPORTHIO_DIR |= 0x0002;  // set up sonar trigger
    *pPORTHIO &= 0xFFFD;       // force H1 low
    initTMR4();
}

void ping_sonar() {
    printf("##ping %d %d %d %d\n\r", sonar_data[1], sonar_data[2], sonar_data[3], sonar_data[4]);
}

void sonar() {
    int t0, t1, t2, t3, t4, x1, x2, x3, x4, imask;
    
    imask = *pPORTHIO & 0x3C00;
    *pPORTHIO |= 0x0002;       // force H1 high to trigger sonars
    t0 = readRTC();
    t1 = t2 = t3 = t4 = *pTIMER4_COUNTER;
    x1 = x2 = x3 = x4 = 0;
    
    while ((readRTC() - t0) < 40) {
        if ((*pPORTHIO & 0x0400)) 
            x1 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x0800)) 
            x2 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x1000)) 
            x3 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x2000)) 
            x4 = *pTIMER4_COUNTER;
    }

    *pPORTHIO &= 0xFFFD;       // force H1 low to disable sonar
    if (imask & 0x0400)
        x1 = 0;
    else
        x1 = (x1 - t1) / 200;
    if (imask & 0x0800)
        x2 = 0;
    else
        x2 = (x2 - t2) / 200;
    if (imask & 0x1000)
        x3 = 0;
    else
        x3 = (x3 - t3) / 200;
    if (imask & 0x2000)
        x4 = 0;
    else
        x4 = (x4 - t4) / 200;

    sonar_data[0] = sonar_data[1] = x1;  // should fix this - we are counting 1-4, not 0-3
    sonar_data[2] = x2;
    sonar_data[3] = x3;
    sonar_data[4] = x4;
}

void enable_frame_diff() {
    frame_diff_flag = 1;
    grab_reference_frame();
    printf("##g0");
}

void enable_segmentation() {
    segmentation_flag = 1;
    printf("##g1");
}

void enable_edge_detect() {
    edge_detect_flag = 1;
    edge_thresh = 3200;
    printf("##g2");
}

void set_edge_thresh () {
    unsigned char ch;
    ch = getch();
    edge_thresh = (unsigned int)(ch & 0x0f) * 800;
    printf("#T");
}

void disable_frame_diff() {  // disables frame differencing, edge detect and color segmentation
    frame_diff_flag = 0;
    segmentation_flag = 0;
    edge_detect_flag = 0;
    printf("#G");
}

void grab_frame () {
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    if (frame_diff_flag)
        compute_frame_diff((unsigned char *)FRAME_BUF, 
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    if (segmentation_flag)
        color_segment((unsigned char *)FRAME_BUF);
    if (edge_detect_flag) {
        //edge_detect((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, edge_thresh);
        svs_segcode((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh);
        svs_segview((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF);
    }
}


void grab_reference_frame () {
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, 
            (unsigned char *)FRAME_BUF2, imgWidth, imgHeight); 
}

/*  compute frame difference between two frames 
     U and V are computed by U1 + 128 - U2
     Y is computed as abs(Y1 - Y2) 
     fcur is current frame
     fref is reference frame*/
void compute_frame_diff(unsigned char *fcur, unsigned char *fref, int w1, int h1) {
    int ix, ipix;
    
    ipix = w1*h1*2;
    for (ix=0; ix<ipix; ix+=2) {
        fcur[ix] = (unsigned char)((unsigned int)fcur[ix] - (unsigned int)fref[ix] + 128);
        if (fcur[ix+1] < fref[ix+1])
            fcur[ix+1] = fref[ix+1] - fcur[ix+1];
        else
            fcur[ix+1] = fcur[ix+1] - fref[ix+1];
    }
}

/* JPEG compress and send frame captured by grab_frame()
   Serial protocol char: I */
void send_frame () {
    unsigned char i2c_data[2];
    unsigned char ch;
    unsigned int ix;
    
    if (overlay_flag) {
        //frame[9] = (framecount % 10) + 0x30;
        //frame[8] = ((framecount/10)% 10) + 0x30;
        //frame[7] = ((framecount/100)% 10) + 0x30;

        i2c_data[0] = 0x41;  // read compass twice to clear last reading
        i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
        i2c_data[0] = 0x41;
        i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
        ix = ((i2c_data[0] << 8) + i2c_data[1]) / 10;

        frame[2] = (ix % 10) + 0x30;
        frame[1] = ((ix/10)% 10) + 0x30;
        frame[0] = ((ix/100)% 10) + 0x30;

        ping_sonar();
        ix = sonar_data[1] / 100;
        frame[10] = (ix % 10) + 0x30;
        frame[9] = ((ix/10)% 10) + 0x30;
        frame[8] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[2] / 100;
        frame[16] = (ix % 10) + 0x30;
        frame[15] = ((ix/10)% 10) + 0x30;
        frame[14] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[3] / 100;
        frame[22] = (ix % 10) + 0x30;
        frame[21] = ((ix/10)% 10) + 0x30;
        frame[20] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[4] / 100;
        frame[28] = (ix % 10) + 0x30;
        frame[27] = ((ix/10)% 10) + 0x30;
        frame[26] = ((ix/100)% 10) + 0x30;
        
        set_caption(frame, imgWidth);
    }
    output_start = (unsigned char *)JPEG_BUF;
    output_end = encode_image((unsigned char *)FRAME_BUF, output_start, quality, 
            FOUR_TWO_TWO, imgWidth, imgHeight); 
    image_size = (unsigned int)(output_end - output_start);

    led1_on();
    framecount++;

    imgHead[6] = (unsigned char)(image_size & 0x000000FF);
    imgHead[7] = (unsigned char)((image_size & 0x0000FF00) >> 8);
    imgHead[8] = (unsigned char)((image_size & 0x00FF0000) >> 16);
    imgHead[9] = 0x00;
    for (i=0; i<10; i++) {
        while (*pPORTHIO & 0x0001)  // hardware flow control
            continue;
        putchar(imgHead[i]);
    }
    cp = (unsigned char *)JPEG_BUF;
    for (i=0; i<image_size; i++) 
        putchar(*cp++);

    while (getchar(&ch)) {
        // flush input 
        continue;
    }
}

/* Turn image overlay on.
   Serial protocol char: o */
void overlay_on () {
    overlay_flag = 1;
    printf("#o");
}


/* Turn image overlay off.
   Serial protocol char: O */
void overlay_off () {
    overlay_flag = 0;
    printf("#O");
}

/* Camera initial setup */
void camera_setup () {
    int ix;
    
    /* Initialise camera-related globals */
    framecount = 0;
    overlay_flag = 0;
    quality = 4; // Default JPEG quality - range is 1-8 (1 is highest)
    frame_diff_flag = 0;
    segmentation_flag = 0;
    imgWidth = 320;
    imgHeight = 240;
    strcpy(imgHead, "##IMJ5    ");
    
    for (ix=0; ix<3; ix++) {
        delayMS(100);
        i2cwrite(0x21, ov7725_qvga, sizeof(ov7725_qvga)>>1, SCCB_ON);
    }
    for (ix=0; ix<3; ix++) {
        delayMS(100);
        i2cwrite(0x30, ov9655_qvga, sizeof(ov9655_qvga)>>1, SCCB_ON);
    }
    camera_init((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, imgWidth, imgHeight);
    camera_start();
}

void invert_video() {  // flip video for upside-down camera
    i2cwrite(0x30, ov9655_invert, sizeof(ov9655_invert)>>1, SCCB_ON);
    printf("#y");
}

void restore_video() {  // restore normal video orientation
    i2cwrite(0x30, ov9655_restore, sizeof(ov9655_restore)>>1, SCCB_ON);
    printf("#Y");
}

/* Refactored out, code to reset the camera after a frame size change. */
void camera_reset (unsigned int width) {
    if (width == 160) {
        imgWidth = width;
        imgHeight = 120;
        strcpy(imgHead, "##IMJ3    ");
        camera_stop();
        i2cwrite(0x21, ov7725_qqvga, sizeof(ov7725_qqvga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_qqvga, sizeof(ov9655_qqvga)>>1, SCCB_ON);
        printf("#a");
    } else if (width == 320) {
        imgWidth = width;
        imgHeight = 240;
        strcpy(imgHead, "##IMJ5    ");
        camera_stop();
        i2cwrite(0x21, ov7725_qvga, sizeof(ov7725_qvga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_qvga, sizeof(ov9655_qvga)>>1, SCCB_ON);
        printf("#b");
    } else if (width == 640) {
        imgWidth = width;
        imgHeight = 480;
        strcpy(imgHead, "##IMJ7    ");
        camera_stop();
        i2cwrite(0x21, ov7725_vga, sizeof(ov7725_vga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_vga, sizeof(ov9655_vga)>>1, SCCB_ON);
        printf("#c");
    } else if (width == 1280) {
        imgWidth = width;
        imgHeight = 1024;
        strcpy(imgHead, "##IMJ9    ");
        camera_stop();
        i2cwrite(0x30, ov9655_sxga, sizeof(ov9655_sxga)>>1, SCCB_ON);
        printf("#A");
    }
    camera_init((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, imgWidth, imgHeight);
    camera_start();
}

/* Change image quality.
   Serial protocol char: q */
void change_image_quality () {
    unsigned char ch;
    ch = getch();
    quality = (unsigned int)(ch & 0x0f);
    if (quality < 1) {
        quality = 1;
    } else if (quality > 8) {
        quality = 8;
    }
    printf("##quality - %c\n\r", ch);
}

// write caption string of up to 40 characters to frame buffer 
void set_caption(unsigned char *str, unsigned int width) {
    unsigned char *fbuf, *fcur, *str1, cc;
    int len, ix, iy, iz, w2;
    
    w2 = width * 2;
    str1 = str;
    
    for (len=0; len<40 && *str1++; len++);          // find string length
    fbuf = FRAME_BUF + (unsigned char *)((width * 17) - (len * 8));  // point to 1st char
    
    for (ix=0; ix<len; ix++) {
        fcur = fbuf;
        for (iy=0; iy< 8; iy++) {
            cc = font8x8[str[ix]*8 + iy];
            for (iz=0; iz<8; iz++) {
                if (cc & fontmask[iz]) {
                    fcur[0] = 0x80;
                    fcur[1] = 0xff;
                }
                fcur += 2;
            }
            fcur += (width * 2) - 16;          // move to next line
        }    
        fbuf += 16;  // move to next char position
    }
}


void move_image(unsigned char *src1, unsigned char *src2, unsigned char *dst, unsigned int width, unsigned int height) {

    unsigned char *src;
    unsigned short *isrc, *idst;
    unsigned int ix;
        
    if (*pDMA0_CURR_ADDR < (void *)src2)
        src = src2;
    else
        src = src1;
    
    isrc = (unsigned short *)src;
    idst = (unsigned short *)dst;
    for (ix = 0; ix < (width * height); ix++)
        *idst++ = *isrc++;
    return;
}

/* XModem Receive.
   Serial protocol char: X */
void xmodem_receive () {
  err1 = xmodemReceive((unsigned char *)FLASH_BUFFER, 131072);
  if (err1 < 0) {
    printf("##Xmodem receive error: %d\n\r", err1);
  } else {
      printf("##Xmodem success. Count: %d\n\r", err1);
  }
}

void launch_editor() {
    edit((unsigned char *)FLASH_BUFFER);
}

/* Clear flash buffer
   Serial protocol char: z-c */
void clear_flash_buffer () {
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + 0x00020000); ix++)
      *((unsigned char *)ix) = 0;   // clear the read buffer
    printf("##zclear buffer\n\r");
}

/* crc flash buffer using crc16_ccitt()
   Serial protocol char: z-C */
void crc_flash_buffer () {
    unsigned int ix;
    ix = (unsigned int)crc16_ccitt((void *)FLASH_BUFFER, 0x0001fff8);  // don't count last 8 bytes
    printf("##zCRC: 0x%x\n\r", ix);
}

/* Read user flash sector into flash buffer
   Serial protocol char: z-r */
void read_user_flash () {
    int ix;
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + 0x00010000); ix++)
      *((unsigned char *)ix) = 0;   // clear the read buffer
    ix = spi_read(FLASH_SECTOR, (unsigned char *)FLASH_BUFFER, 0x00010000);
    printf("##zread count: %d\n\r", ix);
}

void read_user_sector (int isec) {
    int ix;
    printf("##zRead ");
    if ((isec < 2) || (isec > 63)) {
        printf(" - sector %d not accessible\n\r", isec);
        return;
    }
    ix = spi_read((isec * 0x00010000), (unsigned char *)FLASH_BUFFER, 0x00010000);
    printf (" - loaded %d bytes from flash sector %d\n\r", ix, isec);   
}

/* Write user flash sector from flash buffer
   Serial protocol char: z-w */
void write_user_flash () {
    int ix;
    ix = spi_write(FLASH_SECTOR, (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + 0x00010000), 0x00010000);
    printf("##zwrite count: %d\n\r", ix);
}

void write_user_sector (int isec) {
    int ix;
    printf("##zWrite ");
    if ((isec < 2) || (isec > 63)) {
        printf(" - sector %d not accessible\n\r", isec);
        return;
    }
    ix = spi_write((isec * 0x00010000), (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + 0x00010000), 0x00010000);
    printf (" - saved %d bytes to flash sector %d\n\r", ix, isec);   
}

/* Write boot flash sectors (1-2) from flash buffer
   Serial protocol char: z-Z */
void write_boot_flash () {
    unsigned char *cp;
    int ix;
    cp = (unsigned char *)FLASH_BUFFER;
    if (cp[1] != 0x00 && cp[2] != 0x80 && cp[3] != 0xFF) {
        printf("##zZ boot image - invalid header\n\r");
        return;
    }                        
    ix = spi_write(BOOT_SECTOR, (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + 0x00020000), 0x00020000);
    printf("##zZ boot image write count: %d\n\r", ix);
}

/* Process i2c command:  
        irxy  - i2c read device x, register y, return '##ir value'
        iRxy  - i2c read device x, register y, return 2-byte '##iR value'
        iwxyz - i2c write device x, register y, value z, return '##iw'
   Serial protocol char: i */
void process_i2c() {
    unsigned char i2c_device, i2c_data[2];
    
    switch ((unsigned char)getch()) {
        case 'r':
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2cread(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            printf("##ir%2x %d\n\r", i2c_device, i2c_data[0]);
            break;
        case 'R':
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2cread(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
            printf("##iR%2x %d\n\r",i2c_device, (i2c_data[0] << 8) + i2c_data[1]);
            break;
        case 'w':
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2c_data[1] = (unsigned char)getch();
            i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            printf("##iw%2x\n\r", i2c_device);
            break;
        case 'W':  // multi-write
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2c_data[1] = (unsigned char)getch();
            i2c_data[2] = (unsigned char)getch();
            i2c_data[3] = (unsigned char)getch();
            i2cwrite(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
            printf("##iW%2x", i2c_device);
            break;
        default:
            return;
    }
}



/* Motor command, three character command string follows.
   Serial protocol char: M */
void motor_command() {
    unsigned int mdelay;
    if (!pwm1_init) {
        initPWM();
        pwm1_init = 1;
        pwm1_mode = PWM_PWM;
        base_speed = 40;
        lspeed = rspeed = 0;
    }
    lspeed = (int)((signed char)getch());
    rspeed = (int)((signed char)getch());
    mdelay = (unsigned int)getch();
    setPWM(lspeed, rspeed);
    if (mdelay) {
        delayMS(mdelay * 10);
        setPWM(0, 0);
        lspeed = 0;
        rspeed = 0;
    }
    printf("#M");
}

/* Motor command for 2nd set of timers, three character command string follows.
   Serial protocol char: m */
void motor2_command() {
    unsigned int mdelay;
    if (!pwm2_init) {
        initPWM2();
        pwm2_init = 1;
        pwm2_mode = PWM_PWM;
        base_speed2 = 40;
        lspeed2 = rspeed2 = 0;
    }
    lspeed2 = (int)((signed char)getch());
    rspeed2 = (int)((signed char)getch());
    mdelay = (unsigned int)getch();
    setPWM2(lspeed2, rspeed2);
    if (mdelay) {
        delayMS(mdelay * 10);
        setPWM2(0, 0);
        lspeed2 = 0;
        rspeed2 = 0;
    }
    printf("#m");
}

/* Increase motor base speed
   Serial protocol char: + */
void motor_increase_base_speed() {
    base_speed += 3;
    if (base_speed > 95) {
        base_speed = 95;
    }
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed + 3);
        rspeed = check_bounds_0_100(rspeed + 3);
        setPPM1(lspeed, rspeed);
    }
    printf("#+");
}

/* Decrease motor base speed
   Serial protocol char: - */
void motor_decrease_base_speed() {
    base_speed -= 3;
    if (base_speed < 0) {
        base_speed = 0;
    }
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed - 3);
        rspeed = check_bounds_0_100(rspeed - 3);
        setPPM1(lspeed, rspeed);
    }
    printf("#-");
}

void motor_trim_left() {
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed - 1);
        rspeed = check_bounds_0_100(rspeed + 1);
        setPPM1(lspeed, rspeed);
    }
}

void motor_trim_right() {
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed + 1);
        rspeed = check_bounds_0_100(rspeed - 1);
        setPPM1(lspeed, rspeed);
    }
}

/* Take motor action */
void motor_action(unsigned char ch) {
    motor_set(ch, base_speed, &lspeed, &rspeed);
    printf("#%c", ch);
}

/* General motor control code */
void motor_set(unsigned char cc, int speed, int *ls, int *rs)  {
    int left_speed, right_speed;

    if (pwm1_mode != PWM_PWM) // only run the keypad commands in PWM mode
        return;
        
    left_speed = right_speed = 0;
    switch (cc) {
        case '7':     // drift left
            left_speed = speed-15;
            right_speed = speed+15;
            break;
        case '8':     // forward
            left_speed = speed; 
            right_speed = speed;
            break;
        case '9':     // drift right
            left_speed = speed+15;
            right_speed = speed-15;
            break;
        case '4':     // turn left
            left_speed = speed-30;
            right_speed = speed+30;
            break;
        case '5':        // stop
            left_speed = 0;
            right_speed = 0;
            break;
        case '6':     // turn right
            left_speed = speed+30;
            right_speed = speed-30;
            break;
        case '1':     // back left
            left_speed = -(speed-30);
            right_speed = -(speed+30);
            break;
        case '2':     // back
            left_speed = -speed;
            right_speed = -speed;
            break;
        case '3':     // back right
            left_speed = -(speed+30);
            right_speed = -(speed-30);
            break;
        case '.':     // clockwise turn
            setPWM(70, -70);
            delayMS(200);
            setPWM(0, 0);
            left_speed = 0;
            right_speed = 0;
            break;
        case '0':     // counter clockwise turn
            setPWM(-70, 70);
            delayMS(200);
            setPWM(0, 0);
            left_speed = 0;
            right_speed = 0;
            break;
    }
    setPWM(left_speed, right_speed);
    *ls = left_speed;
    *rs = right_speed;
    return;
}

/* servo command, timers 2 and 3, two character command string follows.
   Serial protocol char: S */
void ppm1_command() {
    if (!pwm1_init) {
        initPPM1();
        pwm1_init = 1;
        pwm1_mode = PWM_PPM;
    }
    lspeed = (int)((signed char)getch());
    rspeed = (int)((signed char)getch());
    setPPM1(lspeed, rspeed);
    printf("#S");
}

/* servo command, timers 6 and 7, two character command string follows.
   Serial protocol char: s */
void ppm2_command() {
    if (!pwm2_init) {
        initPPM2();
        pwm2_init = 1;
        pwm2_mode = PWM_PPM;
    }
    lspeed2 = (int)((signed char)getch());
    rspeed2 = (int)((signed char)getch());
    setPPM2(lspeed2, rspeed2);
    printf("#s");
}

void initPWM() {
    // configure timers 2 and 3 for PWM (H-bridge interface)
    //*pPORT_MUX = 0;  // don't do this - it clobbers timers 6/7
    *pPORTF_FER |= 0x00C0;  // configure PF6 and PF7 as TMR3 and TMR2
    *pTIMER2_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER3_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER2_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER3_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100; 
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100;
    *pTIMER_ENABLE = TIMEN2 | TIMEN3;
    *pPORTHIO_DIR |= 0x0030;  // set PORTH4 and PORTH5 to output for direction control
    *pPORTHIO &= 0xFFCF;      // set output low 
    //*pPORTHIO |= 0x0030;  
}

void initPWM2() {
    // configure timers 6 and 7 for PWM
    *pPORT_MUX |= 0x0010;   // note that this reassigns UART1 signals as timers
    *pPORTF_FER |= 0x000C;  // configure PF2 and PF3 as TMR7 and TMR6
    *pTIMER6_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER7_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER6_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER7_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100; 
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100;
    *pTIMER_ENABLE |= TIMEN6 | TIMEN7;
}

void initTMR4() {
    // configure timer 4
    *pPORTF_FER |= 0x0020;  // configure PF5 TMR4
    *pTIMER4_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER4_PERIOD = PERIPHERAL_CLOCK;  // should be counting a 122MHz rate
    *pTIMER4_WIDTH = PERIPHERAL_CLOCK;
    *pTIMER_ENABLE |= TIMEN4;
}

void initPPM1() {
    // configure timers 2 and 3
    *pPORTF_FER |= 0x00C0;  // configure PF6 and PF7 as TMR3 and TMR2
    *pTIMER2_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER3_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER2_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER3_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 50) * 100) / 2000; // 1.0 millisec pulse
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 50) * 100) / 2000; // 1.0 millisec pulse
    *pTIMER_ENABLE |= TIMEN2 | TIMEN3;
}

void initPPM2() {
    // configure timers 6 and 7
    *pPORT_MUX |= 0x0010;   // note that this reassigns UART1 signals as timers
    *pPORTF_FER |= 0x000C;  // configure PF2 and PF3 as TMR7 and TMR6
    *pTIMER6_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER7_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER6_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER7_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER_ENABLE |= TIMEN6 | TIMEN7;
}

void setPWM (int mleft, int mright) {
    if (mleft < 0) {
        *pPORTHIO = (*pPORTHIO & 0xFFEF);  // clear left direction bit
        mleft = -mleft;
    } else {
        *pPORTHIO = (*pPORTHIO & 0xFFEF) | 0x0010;  // turn on left direction bit
    }
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;

    if (mright < 0) {
        *pPORTHIO = (*pPORTHIO & 0xFFDF);  // clear right direction bit
        mright = -mright;
    } else {
        *pPORTHIO = (*pPORTHIO & 0xFFDF) | 0x0020;  // turn on right direction bit
    }
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mleft) / 100;
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mright) / 100;
}

void setPWM2 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;

    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mleft) / 100;
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mright) / 100;
}

void setPPM1 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mleft)) / 2000;
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mright)) / 2000;
}

void setPPM2 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mleft)) / 2000;
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mright)) / 2000;
}

int check_bounds_0_100(int ix) {
    if (ix > 100)
        ix = 100;
    if (ix < 0)
        ix= 0;
    return ix;
}

/* Initialise the Real-time Clock */
void initRTC() {
    *pRTC_ICTL = 0;  // disable interrupts
    SSYNC;
    *pRTC_PREN = 0;  // disable prescaler - clock counts at 32768 Hz
    SSYNC;
    *pRTC_STAT = 0;  // clear counter
    SSYNC;
}

/* Read the RTC counter, returns number of milliseconds since reset */
int readRTC() {     
    int i1, i2;
    i1 = *pRTC_STAT;
    i2 = (i1 & 0x0000003F) + (((i1 >> 6) & 0x0000003F) * 60) +  
        (((i1 >> 12) & 0x0000001F) * 3600) + (((i1 >> 17) & 0x00007FFF) * 86400);
    return (i2 / 33);  // converts tick count to milliseconds
                       //    32,768 / 32.77 = 1,000
}

/* Clear the RTC counter value */
void clearRTC() {
    *pRTC_STAT = 0;
    SSYNC;
}

void delayMS(int delay) {  // delay up to 100000 millisecs (100 secs)
    int i0;

    if ((delay < 0) || (delay > 100000))
        return;
    i0 = readRTC();
    while (readRTC() < (i0 + delay))
        continue;
}

void delayUS(int delay) {  // delay up to 100000 microseconds (.1 sec)
    // CORE_CLOCK (MASTER_CLOCK * VCO_MULTIPLIER / CCLK_DIVIDER) = 22,118,000 * 22
    // PERIPHERAL_CLOCK  (CORE_CLOCK / SCLK_DIVIDER) =  CORE_CLOCK / 4 = 121,649,000
    // *pTIMER4_PERIOD = PERIPHERAL_CLOCK, so TIMER4 should be counting a 121.649MHz rate
    int target, start;
    
    if ((delay < 0) || (delay > 100000))
        return;
    start = *pTIMER4_COUNTER;
    target = (((PERIPHERAL_CLOCK / 10000) * delay) / 100) + start;
    
    if (target > PERIPHERAL_CLOCK) {  // wait for timer to wrap-around
        target -= PERIPHERAL_CLOCK;
        while (*pTIMER4_COUNTER > target)
            continue;
    }
    while (*pTIMER4_COUNTER < target)
        continue;
}

void delayNS(int delay) {  // delay up to 100000 nanoseconds (.1 millisec)
    // minimum possible delay is approx 10ns
    int target, start;
    
    if ((delay < 10) || (delay > 100000))
        return;
    start = *pTIMER4_COUNTER;
    target = (((PERIPHERAL_CLOCK / 10000) * delay) / 100000) + start;
    
    if (target > PERIPHERAL_CLOCK) {  // wait for timer to wrap-around
        target -= PERIPHERAL_CLOCK;
        while (*pTIMER4_COUNTER > target)
            continue;
    }
    while (*pTIMER4_COUNTER < target)
        continue;
}

/* Enable failsafe - two character motor speeds follow
   Serial protocol char: F */
void enable_failsafe() {
    lfailsafe = (int)((signed char)getch());
        if (lfailsafe == 0)  // minimum PWM power setting is 0x01, not 0x00
            lfailsafe = 1;
    rfailsafe = (int)((signed char)getch());
        if (rfailsafe == 0)  // minimum PWM power setting is 0x01, not 0x00
            rfailsafe = 1;
    failsafe_mode = 1;
    printf("#F");
}

/* Disable failsafe - 
   Serial protocol char: f */
void disable_failsafe() {
    failsafe_mode = 0;
    printf("#f");
}

void reset_failsafe_clock() {
    failsafe_clock = readRTC();
}

void check_failsafe() {
    if (!failsafe_mode)
        return;
    if ((readRTC() - failsafe_clock) < 2000)  // 2 second timeout
        return;
    lspeed = lfailsafe;
    rspeed = rfailsafe;
    if (pwm1_mode == PWM_PWM)
        setPWM(lspeed, rspeed);
    if (pwm1_mode == PWM_PPM)
        setPPM1(lspeed, rspeed);
}

void process_colors() {
    unsigned char ch, ch1, ch2, ch3, ch4;
    unsigned int ix, iy, i1, i2, itot;
    unsigned int ulo[4], uhi[4], vlo[4], vhi[4];
    unsigned char i2c_data[2];
              // vision processing commands
                    //    vc = set colors
                    //    vp = sample individual pixel
                    //    vb = find blobs
                    //    vr = recall colors
                    //    vh = histogram
                    //    vm = mean colors
                    //    vz = zero all color settings
                    //    vd = dump camera registers
    ch = getch();
    switch (ch) {
        case 'c':  //    vc = set colors
            ix = (unsigned int)getch();
            if (ix > '9')
                ix = (ix & 0x0F) + 9;
            else
                ix &= 0x0F;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ymin[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ymax[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            umin[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            umax[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            vmin[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            vmax[ix] = ch1 * 100 + ch2 * 10  + ch3;
            printf("##vc %d\n\r", ix);
            break;
        case 'p':  //    vp = sample individual pixel, print YUV value
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            i1 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            i2 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            grab_frame();
            ix = vpix((unsigned char *)FRAME_BUF, i1, i2);
            printf("##vp %d %d %d\n\r",
                ((ix>>16) & 0x000000FF),  // Y1
                ((ix>>24) & 0x000000FF),  // U
                ((ix>>8) & 0x000000FF));   // V
            break;
        case 'b':  //    vb = find blobs for a given color
            ch1 = getch();
            ch2 = ch1;
            if (ch1 > '9')
                ch1 = (ch1 & 0x0F) + 9;
            else
                ch1 &= 0x0F;
            grab_frame();
            ix = vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, ch1);
            printf("##vb%c\n\r", ch2);
            for (iy=0; iy<ix; iy++) {
                printf(" %d - %d %d %d %d  \n\r", 
                    blobcnt[iy], blobx1[iy], blobx2[iy], bloby1[iy], bloby2[iy]);
            }
            break;
        case 'r':  //    vr = recall colors
            ix = (unsigned int)getch();
            if (ix > '9')
                ix = (ix & 0x0F) + 9;
            else
                ix &= 0x0F;
            printf("##vr %d %d %d %d %d %d %d\n\r",
               ix, ymin[ix], ymax[ix], umin[ix], umax[ix], vmin[ix], vmax[ix]);
            break;
        case 'h':  //    vh = histogram
            grab_frame();
            vhist((unsigned char *)FRAME_BUF);
            printf("##vhist\n\r");
            iy = 0;
            itot = imgWidth * imgHeight / 2;
            printf("      0  16  32  48  64  80  96 112 128 144 160 176 192 208 224 240 (V-axis)\n\r");
            for (i1=0; i1<16; i1++) {
                printf("%d", i1*16);
                for (i2=0; i2<16; i2++) {
                    iy++;
                    ix = i1*16 + i2;
                    if (hist0[ix] > (itot>>2))
                        printf("****");
                    else if (hist0[ix] > (itot>>5))
                        printf(" ***");
                    else if (hist0[ix] > (itot>>8))
                        printf("  **");
                    else if (hist0[ix] > (itot>>11))
                        printf("   *");
                    else {
                        printf("    ");
                        iy--;
                    }
                }
                printf("\n\r");
            }
            printf("(U-axis)             %d regions\n\r", iy);
            break;
        case 'm':  //    vm = mean colors
            grab_frame();
            vmean((unsigned char *)FRAME_BUF);
            printf("##vmean %d %d %d\n\r", mean[0], mean[1], mean[2]);
            break;
        case 'z':  //    vz = clear or segment colors
            ix = (unsigned int)getch() & 0x0F;
            printf("##vzero\n\r");
            switch (ix) {
                case 0:
                    for(ix = 0; ix<MAX_COLORS; ix++) 
                        ymin[ix] = ymax[ix] = umin[ix] = umax[ix] = vmin[ix] = vmax[ix] = 0;
                    break;
                case 1:
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        ymin[ix] = (ix / 4) * 64;
                        ymax[ix] = ymin[ix] + 63;
                        umin[ix] = (ix & 0x02) * 64;
                        umax[ix] = umin[ix] + 127;
                        vmin[ix] = (ix & 0x01) * 128;
                        vmax[ix] = vmin[ix] + 127;
                    }
                    break;
                case 2:
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        ymin[ix] = 0;
                        ymax[ix] = 255;
                        umin[ix] = (ix >> 2) * 64;
                        umax[ix] = umin[ix] + 63;
                        vmin[ix] = (ix & 0x03) * 64;
                        vmax[ix] = vmin[ix] + 63;
                    }
                    break;
                case 3:
                    ulo[0]=0; ulo[1]=96; ulo[2]=128; ulo[3]=160;
                    uhi[0]=ulo[1]-1; uhi[1]=ulo[2]-1; uhi[2]=ulo[3]-1; uhi[3]=255;
                    vlo[0]=0; vlo[1]=96; vlo[2]=128; vlo[3]=160;
                    vhi[0]=vlo[1]-1; vhi[1]=vlo[2]-1; vhi[2]=vlo[3]-1; vhi[3]=255;
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        i1 = ix >> 2;
                        i2 = ix & 0x03;
                        ymin[ix] = 0;
                        ymax[ix] = 255;
                        umin[ix] = ulo[i1];
                        umax[ix] = uhi[i1];
                        vmin[ix] = vlo[i2];
                        vmax[ix] = vhi[i2];
                    }
                    break;
                case 4:
                    for(ix = 0; ix<MAX_COLORS; ix++) {
                        ymin[ix] = ix << 4;
                        ymax[ix] = ymin[ix] + 15;
                        umin[ix] = 0;
                        umax[ix] = 255;
                        vmin[ix] = 0;
                        vmax[ix] = 255;
                    }
                    break;
           }
            break;
        case 'd':  //    vd = dump camera registers
            printf("##vdump\n\r");
            for(ix=0; ix<256; ix++) {
                i2c_data[0] = ix;
                i2cread(0x30, (unsigned char *)i2c_data, 1, SCCB_ON);
                printf("%d %d\n\r", ix, i2c_data[0]);
            }
            break;
    }
}

void process_neuralnet() {
    unsigned char ch;
    unsigned int ix, i1, i2;
              // neural net processing commands
                    //    np = set pattern
                    //    nd = display pattern
                    //    ni = init network
                    //    nt = train for 10000 iterations
                    //    nx = test a pattern
                    //    nb = match blob to patterns
                    //    ng = create pattern from blob
    ch = getch();
    switch (ch) {
        case 'p':  //    np = set pattern
            ix = ctoi(getch());
            if (ix > NUM_NPATTERNS) {
                printf("##np - invalid index\n\r");
                break;
            }
            for (i1=0; i1<8; i1++)
                npattern[ix*8 + i1] = (ctoi(getch()) << 4) + ctoi(getch());
            printf("##np %d\n\r", ix);
            break;
        case 'd':  //    nd = display pattern
            ix = ctoi(getch());
            if (ix > NUM_NPATTERNS) {
                printf("##np - invalid index\n\r");
                break;
            }
            printf("##nd %d\n\r", ix);
            nndisplay(ix);
            break;
        case 'i':  //    ni = init network
            nninit_network();
            printf("##ni - init neural net\n\r");
            break;
        case 't':  //    nt = train network
            nntrain_network(10000);
            printf("##nt - train 10000 iterations\n\r");
            for (ix=0; ix<NUM_NPATTERNS; ix++) {
                nnset_pattern(ix);
                nncalculate_network();
                for (i1=0; i1<NUM_OUTPUT; i1++) 
                    printf(" %3d", N_OUT(i1)/10);
                printf("\n\r");
            }
            break;
        case 'x':  //    nx = test example pattern
            ix = 0;
            for (i1=0; i1<8; i1++) {   /// capture the test pattern and store in N_IN input neurons
                ch = (ctoi(getch()) << 4) + ctoi(getch());
                for (i2=0; i2<8; i2++) {
                    if (ch & nmask[i2])
                        N_IN(ix++) = 1024;
                    else
                        N_IN(ix++) = 0;
                }
            }
            nncalculate_network();
            printf("##nx\n\r");
            for (i1=0; i1<NUM_OUTPUT; i1++) 
                printf(" %3d", N_OUT(i1)/10);
            printf("\n\r");
            break;            
        case 'b':  //    nb = match blob to patterns
            ix = ctoi(getch());    // grab the blob #
            if (!blobcnt[ix]) { 
                printf("##nb - not a valid blob\n\r");
                break;
            }
            /* use data still in blob_buf[] (FRAME_BUF3)
               square the aspect ratio of x1, x2, y1, y2
               then subsample blob pixels to populate N_IN(0:63) with 0:1024 values
               then nncalculate_network() and display the N_OUT() results */
            nnscale8x8((unsigned char *)FRAME_BUF3, blobix[ix], blobx1[ix], blobx2[ix], 
                    bloby1[ix], bloby2[ix], imgWidth, imgHeight);
            nncalculate_network();
            printf("##nb\n\r");
            for (i1=0; i1<NUM_OUTPUT; i1++) 
                printf(" %3d", N_OUT(i1)/10);
            printf("\n\r");
            break;
        case 'g':  //     ng = create pattern from blob
            ix = ctoi(getch());    // grab the new pattern #
            if (!blobcnt[0]) { 
                printf("##ng - no blob to grab\n\r");
                break;
            }
            nnscale8x8((unsigned char *)FRAME_BUF3, blobix[0], blobx1[0], blobx2[0], 
                    bloby1[0], bloby2[0], imgWidth, imgHeight);
            nnpack8x8(ix);
            nndisplay(ix);
            break;
    }
}

/* pseudo-random number generator based on Galois linear feedback shift register
     taps: 32 22 2 1; characteristic polynomial:  x^32 + x^22 + x^2 + x^1 + 1  */
unsigned int rand() { 
    int ix, iy;
    if (rand_seed == 0x55555555) {  // initialize 
        iy = (readRTC() % 1000) + 1000;
        for (ix=0; ix<iy; ix++)
            rand_seed = (rand_seed >> 1) ^ (-(rand_seed & 0x00000001) & 0x80200003); 
    }
    for (ix=0; ix<19; ix++)  // use every 19th result
        rand_seed = (rand_seed >> 1) ^ (-(rand_seed & 0x00000001) & 0x80200003); 
    return (rand_seed);
}

unsigned int isqrt(unsigned int val) {
    unsigned int temp, g=0, b = 0x8000, bshft = 15;
    do {
        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
           g += b;
           val -= temp;
        }
    } while (b >>= 1);
    return g;
}


