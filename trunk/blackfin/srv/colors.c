/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  colors.c - image processing routines for the SRV-1 Blackfin robot.
 *    Copyright (C) 2007-2009  Surveyor Corporation
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

#include "colors.h"
#include "print.h"

extern unsigned int imgWidth, imgHeight;

unsigned int ymax[MAX_COLORS], ymin[MAX_COLORS], umax[MAX_COLORS], umin[MAX_COLORS], vmax[MAX_COLORS], vmin[MAX_COLORS];
unsigned int blobx1[MAX_BLOBS], blobx2[MAX_BLOBS], bloby1[MAX_BLOBS], bloby2[MAX_BLOBS], blobcnt[MAX_BLOBS], blobix[MAX_BLOBS];
unsigned int hist0[256], hist1[256], mean[3];

void init_colors() {
    unsigned int ii;
    
    for(ii = 0; ii<MAX_COLORS; ii++) {
        ymax[ii] = 0;
        ymin[ii] = 0;
        umax[ii] = 0;
        umin[ii] = 0;
        vmax[ii] = 0;
        vmin[ii] = 0;
    }
}

unsigned int vpix(unsigned char *frame_buf, unsigned int xx, unsigned int yy) {
        unsigned int ix;
        ix = index(xx,yy); 
        return    ((unsigned int)frame_buf[ix] << 24) +    // returns UYVY packed into 32-bit word
                        ((unsigned int)frame_buf[ix+1] << 16) +
                        ((unsigned int)frame_buf[ix+2] << 8) +
                        (unsigned int)frame_buf[ix+3];
}

// merge blobs, changing any pixel in blob_buf[] in old_blob to new_blob
void blob_merge(unsigned char *blob_buf, unsigned char old_blob, unsigned char new_blob) {
    int ix;
    for (ix=0; ix<(imgWidth*imgHeight); ix+=2)
        if (blob_buf[ix] == old_blob)
            blob_buf[ix] = new_blob;
}

// return number of blobs found that match the search color
// algorithm derived from "Using a Particle Filter for Gesture Recognition", Alexander Gruenstein
//    http://www.mit.edu/~alexgru/vision/
unsigned int vblob(unsigned char *frame_buf, unsigned char *blob_buf, unsigned int ii) {
    unsigned int ix, iy, xx, yy, y, u, v, tmp;
    unsigned char *bbp, curBlob, vL, vTL, vT, vTR, vME;

    register int y1, y2, u1, u2, v1, v2;
    y1 = ymin[ii];
    y2 = ymax[ii];
    u1 = umin[ii];
    u2 = umax[ii];
    v1 = vmin[ii];
    v2 = vmax[ii];

    for (curBlob=0; curBlob<MAX_BLOBS; curBlob++) {
        blobcnt[curBlob] = 0;
        blobx1[curBlob] = imgWidth;
        blobx2[curBlob] = 0;
        bloby1[curBlob] = imgHeight;
        bloby2[curBlob] = 0;
        blobix[curBlob] = 0;
    }

    bbp = blob_buf;
    for (ix=0; ix<imgWidth*imgHeight; ix++)
        *bbp++ = 0;

    // tag all pixels in blob_buf[]    
    //     matching = 1  
    //     no color match = 0
    // thus all matching pixels will belong to blob #1
    bbp = blob_buf;
    for (ix=0; ix<(imgWidth*imgHeight*2); ix+=4){
        y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
        u = (unsigned int)frame_buf[ix];
        v = (unsigned int)frame_buf[ix+2];

        if ((y > y1) && (y < y2) && (u > u1) && (u < u2) && (v > v1) && (v < v2))
            *bbp = 1;
        bbp += 2;
    }

    curBlob = 2;
    for (yy=1; yy<imgHeight-1; yy++) {   // don't go all the way to the image edge
        for (xx=2; xx<(imgWidth-4); xx+=2) {
            ix = xx + (yy * imgWidth);
            vL = 0; vTL = 0; vT = 0; vTR = 0;
            vME = 0;
            if (blob_buf[ix] == 1) {
                vL =  blob_buf[ix-2];    // left
                vTL = blob_buf[(ix - imgWidth) - 2];   // top left
                vT =  blob_buf[ix - imgWidth];  // top
                vTR = blob_buf[(ix - imgWidth) + 2];   // top right
                
                if (vL)
                    vME = vL;
                if (vTL)
                    vME = vTL;  // guaranteed same as vL by previous iteration
                if (vT) {
                    if ((vL != 0) && (vL != vT))      // we have a U connection
                        blob_merge(blob_buf, vT, vL); // change all vT's to vL's
                    else
                        vME = vT;
                }
                if (vTR) {
                    if ((vTL != 0) && (vTL != vTR))
                        blob_merge(blob_buf, vTR, vTL); // change all vTR's to vTL's
                    else
                        vME = vTR;
                }
                if (vME == 0) {
                    vME = curBlob;
                    curBlob++;
                    if (curBlob >= MAX_BLOBS) { // max blob limit exceeded
                        printf("  vblob #%d: max blob limit exceeded\n\r", ii);
                        return 0;
                    }
                }
                blob_buf[ix] = vME;
            }
        }
    }

    // measure the blobs
    for (yy=0; yy<imgHeight; yy++) {
        for (xx=0; xx<imgWidth; xx+=2) {
            ix = xx + (yy * imgWidth);
            iy = blob_buf[ix];
            if (iy) {
                blobcnt[iy]++;
                if (xx < blobx1[iy])
                    blobx1[iy] = xx;
                if (xx > blobx2[iy])
                    blobx2[iy] = xx;
                if (yy < bloby1[iy])
                    bloby1[iy] = yy;
                if (yy > bloby2[iy])
                    bloby2[iy] = yy;
            }
        }
    }

    // compress the blob array
    for (xx=0; xx<=curBlob; xx++)
        if (blobcnt[xx] < MIN_BLOB_SIZE)
            blobcnt[xx] = 0;
            
    for (xx=0; xx<(curBlob-1); xx++) {
        if (blobcnt[xx] == 0) {
            for (yy=xx+1; yy<=curBlob; yy++) {
                if (blobcnt[yy]) {
                    blobcnt[xx] = blobcnt[yy];
                    blobx1[xx] = blobx1[yy];
                    blobx2[xx] = blobx2[yy];
                    bloby1[xx] = bloby1[yy];
                    bloby2[xx] = bloby2[yy];
                    blobix[xx] = yy;   // this tells us the index of this blob in blob_buf[]
                    blobcnt[yy] = 0;
                    break;
                }
            }
        }
    }
    
    iy = 0;
    for (xx=0; xx<=curBlob; xx++) {
        if (blobcnt[xx])
            iy++;
        else
            break;
    }
    curBlob = iy;

    // sort blobs by size, largest to smallest pixel count
    for (xx=0; xx<=curBlob; xx++) {
        if (blobcnt[xx] == 0)  // no more blobs
            break;
        for (yy=xx+1; yy<=curBlob; yy++) {
            if (blobcnt[yy] == 0)
                break;
            if (blobcnt[xx] < blobcnt[yy]) {
                tmp = blobcnt[xx];
                blobcnt[xx] = blobcnt[yy];
                blobcnt[yy] = tmp;
                tmp = blobx1[xx];
                blobx1[xx] = blobx1[yy];
                blobx1[yy] = tmp;
                tmp = blobx2[xx];
                blobx2[xx] = blobx2[yy];
                blobx2[yy] = tmp;
                tmp = bloby1[xx];
                bloby1[xx] = bloby1[yy];
                bloby1[yy] = tmp;
                tmp = bloby2[xx];
                bloby2[xx] = bloby2[yy];
                bloby2[yy] = tmp;
                tmp = blobix[xx];
                blobix[xx] = blobix[yy];
                blobix[yy] = tmp;
            }
        }
    }
    return curBlob;
}

// histogram function - 
//  hist0[] holds frequency of u|v combination  
//  hist1[] holds average luminance corresponding each u|v combination

void vhist(unsigned char *frame_buf) {
    unsigned int ix, iy, xx, yy, y1, u1, v1;

    for (ix=0; ix<256; ix++) {          hist0[ix] = 0;  // accumulator 
        hist1[ix] = 0;  
    }
    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);  
            y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            u1 = ((unsigned int)frame_buf[ix]);
            v1 = ((unsigned int)frame_buf[ix+2]);
            iy = (u1 & 0xF0) + (v1 >> 4);
            hist0[iy]++;
            hist1[iy] += y1;
        }
    }
    for (ix=0; ix<256; ix++)
        if (hist1[ix])
            hist1[ix] /= hist0[ix];  // normalize by number of hits
}

/* mean color function - computes mean value for Y, U and V
   mean[0] = Y mean, mean[1] = U mean, mean[2] = V mean */
void vmean(unsigned char *frame_buf) {
    unsigned int ix, xx, yy, y1, u1, v1;
    unsigned int my, mu, mv;
    my = mu = mv = 0;

    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);  // yx, uv, vx range from 0-63 (yuv value divided by 4)
            y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            u1 = ((unsigned int)frame_buf[ix]);
            v1 = ((unsigned int)frame_buf[ix+2]);
            my += y1;
            mu += u1;
            mv += v1;
        }
    }
    mean[0] = ((my*2) / imgWidth) / imgHeight;
    mean[1] = ((mu*2) / imgWidth) / imgHeight;
    mean[2] = ((mv*2) / imgWidth) / imgHeight;
}

void color_segment(unsigned char *frame_buf) {
    unsigned int ix, xx, yy, y, u, v, clr;
    unsigned int ymid[MAX_COLORS], umid[MAX_COLORS], vmid[MAX_COLORS];
    
    for (ix=0; ix<MAX_COLORS; ix++) {
        ymid[ix] = (ymax[ix] + ymin[ix]) >> 1;
        umid[ix] = (umax[ix] + umin[ix]) >> 1;
        vmid[ix] = (vmax[ix] + vmin[ix]) >> 1;
    }
    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);
            y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            //y = (unsigned int)frame_buf[ix+1];
            u = (unsigned int)frame_buf[ix];
            v = (unsigned int)frame_buf[ix+2];
            for (clr=0; clr<MAX_COLORS; clr++) {
                if (ymax[clr] == 0)    // skip this color if not defined
                    continue;
                if ((y >= ymin[clr])
                  && (y <= ymax[clr]) 
                  && (u >= umin[clr]) 
                  && (u <= umax[clr]) 
                  && (v >= vmin[clr]) 
                  && (v <= vmax[clr])) {
                    frame_buf[ix+1] = frame_buf[ix+3] = ymid[clr];
                    frame_buf[ix] = umid[clr];
                    frame_buf[ix+2] = vmid[clr];
                    break;
                }
            }
            if (clr == MAX_COLORS) {  // if no match, black out the pixel
                frame_buf[ix+1] = frame_buf[ix+3] = 0;
                frame_buf[ix] = frame_buf[ix+2] = 128;
            }
        }
    }
}

/* count number of pixels matching 'clr' bin in range [x1,y1] to [x2,y2] */
unsigned int vfind(unsigned char *frame_buf, unsigned int clr, 
                   unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2) {
    unsigned int ix, xx, yy, y, u, v, count;
    
    count = 0;
    for (xx=x1; xx<x2; xx+=2) {   
        for (yy=y1; yy<y2; yy++) {
            ix = index(xx,yy);
            y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            //y = (unsigned int)frame_buf[ix+1];
            u = (unsigned int)frame_buf[ix];
            v = (unsigned int)frame_buf[ix+2];
            if ((y >= ymin[clr])
              && (y <= ymax[clr]) 
              && (u >= umin[clr]) 
              && (u <= umax[clr]) 
              && (v >= vmin[clr]) 
              && (v <= vmax[clr])) {
                count++;
            }
        }
    }
    return count;
}

void edge_detect(unsigned char *inbuf, unsigned char *outbuf, int thresh) {
    unsigned int ix, xx, yy, y2, u2, v2, skip;
    unsigned char *ip, *op;
    unsigned int *ip1, *op1;
    unsigned int gx, gy;
    
    skip = imgWidth*2;
    for (xx=2; xx<imgWidth-2; xx+=2) {   
        for (yy=1; yy<imgHeight-1; yy++) {
            gx = gy = 0;
            ix = index(xx, yy);
            ip = inbuf + ix;
            op = outbuf + ix;

            y2 = *(ip+5) + *(ip+7) - *(ip-3) - *(ip-1);
            u2 = *(ip+4) - *(ip-4);
            v2 = *(ip+6) - *(ip-2);
            gy = ((y2*y2)>>2) + u2*u2 + v2*v2;
            
            y2 = *(ip+1+skip) + *(ip+3+skip) - *(ip+1-skip) - *(ip+3-skip);
            u2 = *(ip+skip) - *(ip-skip); 
            v2 = *(ip+2+skip) - *(ip+2-skip); 
            gx = ((y2*y2)>>2) + u2*u2 + v2*v2;
 
            if ((gx > thresh) || (gy > thresh)) {
                *op = 128;
                *(op+2) = 128;
                *(op+1) = 255;
                *(op+3) = 255;
            } else {
                *op = (*(ip) & 0x80) | 0x40;
                *(op+2) = (*(ip+2) & 0x80) | 0x40;
                *(op+1) = (*(ip+1) & 0xC0) | 0x20;
                *(op+3) = (*(ip+3) & 0xC0) | 0x20;
            }
        }
    }

    op1 = (unsigned int *)outbuf;
    ip1 = (unsigned int *)inbuf;
    for (ix=0; ix<(imgWidth*imgHeight>>1); ix++)
        ip1[ix] = op1[ix];
}

unsigned int vscan(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           unsigned int columns, unsigned int *outvect) {
    unsigned int ix, x, y, hits;
    unsigned char *pp;
    
    svs_segcode(outbuf, inbuf, thresh);  // find edge pixels

    hits = 0;
    pp = outbuf;
    for (ix=0; ix<columns; ix++)    // initialize output vector
        outvect[ix] = imgHeight;
        
    for (y=0; y<imgHeight; y++) {  // now search from top to bottom, noting each hit in the appropriate column
        for (x=0; x<imgWidth; x+=2) {  // note that edge detect used full UYVY per pixel position
            if (*pp & 0xC0) {   // look for edge hit 
                outvect[((x * columns) / imgWidth)] = imgHeight - y;
                hits++;
            }
            pp++;
        }
    }
    return hits;
}

void svs_segcode(unsigned char *outbuf, unsigned char *inbuf, int thresh) {
    unsigned int ix, xx, yy, y2, u2, v2, skip;
    unsigned char *ip, *op, cc;
    unsigned int gx, gy;
    
    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    
    skip = imgWidth*2;
    op = outbuf;
    for (yy=0; yy<imgHeight; yy++) {
        for (xx=0; xx<imgWidth; xx+=2) {   
            if ((xx < 2) || (xx >= imgWidth-2) || (yy < 1) || (yy >= imgHeight-1)) {
                *op++ = 0;
                continue;
            }
            gx = gy = 0;
            ix = index(xx, yy);
            ip = inbuf + ix;

            y2 = *(ip+5) + *(ip+7) - *(ip-3) - *(ip-1);
            u2 = *(ip+4) - *(ip-4);
            v2 = *(ip+6) - *(ip-2);
            gy = ((y2*y2)>>2) + u2*u2 + v2*v2;
            
            y2 = *(ip+1+skip) + *(ip+3+skip) - *(ip+1-skip) - *(ip+3-skip);
            u2 = *(ip+skip) - *(ip-skip); 
            v2 = *(ip+2+skip) - *(ip+2-skip); 
            gx = ((y2*y2)>>2) + u2*u2 + v2*v2;
 
            cc = ((*ip >> 2) & 0x38)       // threshold U to 0x38 position
               + ((*(ip+2) >> 5) & 0x07);  // threshold V to 0x07 position
            if (gx > thresh)  
                cc |= 0x80;               // add 0x80 flag if this is edge pixel
            if (gy > thresh)  
                cc |= 0x40;               // add 0x40 flag if this is edge pixel
            *op++ = cc;
        }
    }
}

void svs_segview(unsigned char *inbuf, unsigned char *outbuf) {
    unsigned int ix;
    unsigned char *ip, *op;

    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    
    ip = inbuf;
    op = outbuf;
    for (ix=0; ix<imgWidth*imgHeight; ix+=2) {   
        if (*ip & 0xC0) {      // is this an edge pixel ?
            *(op+1) = *(op+3) = 0xFF;
            *op = *(op+2) = 0x80;
        } else {
            *(op+1) = *(op+3) = 0xA0;
            *op = ((*ip & 0x38) << 2) + 0x10;
            *(op+2) = ((*ip & 0x07) << 5) + 0x10;
        }
        op += 4;
        ip++;
    }
}

