/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  stereo.c - SVS (stereo vision system) functions
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

#include "stereo.h"
#include <cdefBF537.h>
#include "config.h"
#include "uart.h"
#include "print.h"
#include "xmodem.h"
#include "srv.h"
#include "malloc.h"
#include "string.h"

/* -----------------------------------------------------------------------------------------------------*/
/* stereo correspondence */
/* -----------------------------------------------------------------------------------------------------*/

void svs_master(unsigned short *outbuf16, unsigned short *inbuf16, int bufsize)
{
#ifdef SVS_PROFILE
    int t[6];
#endif
    int remainingBytes;
    unsigned short ix, dummy;
    
#ifdef SVS_PROFILE
    t[0] = readRTC();
#endif

    *pPORTF_FER |= (PF14|PF13|PF12|PF11|PF10);  /* SLCK, MISO, MOSI perhipheral */
    *pPORT_MUX |= PJSE;  /* Enable PJ10 (SSEL2/3) PORT_MUX PJSE=1 */
    *pSPI_BAUD = 8;      /* 4, 8, 16 */
    *pSPI_FLG = FLS3;    /* set FLS3 (FLG3 not required with CHPA=0 */
    *pSPI_CTL = SPE|MSTR|CPOL|SIZE|EMISO|0x00;  /* we use CPHA=0 hardware control */
    SSYNC;
    
#ifdef SVS_PROFILE
    t[1] = readRTC();
#endif

    remainingBytes = bufsize;

    ix = (unsigned int)crc16_ccitt((void *)outbuf16, bufsize-8);
    outbuf16[(bufsize/2)-2] = ix; /* write CRC into buffer */
    
#ifdef SVS_PROFILE
    t[2] = readRTC();    
#endif

    *pSPI_TDBR = *outbuf16++;
    dummy = *pSPI_RDBR;  /* slave will not have written yet */
    remainingBytes -= 2;
    SSYNC;
    
#ifdef SVS_PROFILE
    t[3] = readRTC();    
#endif

    while(remainingBytes)
    {
        while((*pSPI_STAT&SPIF) ==0 )
            ; /* ensure spi tranfer complete */
        while((*pSPI_STAT&TXS) >0 )
            ;  /* ensure tx buffer empty */
        while((*pSPI_STAT&RXS) ==0 )
            ; /* ensure rx buffer full */
        *pSPI_TDBR = *outbuf16++;
        SSYNC;
        if (remainingBytes == (bufsize-2))  /* first data is junk */
            dummy = *pSPI_RDBR;
        else
            *inbuf16++ = *pSPI_RDBR; /* read data from slave processor */
        SSYNC;
        remainingBytes -= 2;
    }

#ifdef SVS_PROFILE
    t[4] = readRTC();
#endif
    
    printf("##$X SPI Master - Ack = 0x%x\n\r", (unsigned int)dummy);
    *pSPI_CTL = 0x400;
    *pSPI_FLG = 0x0;
    *pSPI_BAUD = 0x0;
    SSYNC;
    
#ifdef SVS_PROFILE
    t[5] = readRTC();
#endif
    
#ifdef SVS_PROFILE
    printf("svs_master:\n\r");
    printf("    step 0  %d\n\r", t[1]-t[0]);
    printf("    step 1  %d\n\r", t[2]-t[1]);
    printf("    step 2  %d\n\r", t[3]-t[2]);
    printf("    step 3  %d\n\r", t[4]-t[3]);
    printf("    step 4  %d\n\r", t[5]-t[4]);
#endif
}

void svs_slave(
    unsigned short *inbuf16,
    unsigned short *outbuf16,
    int bufsize)
{
    int remainingBytes;
    unsigned short *bufsave;

    bufsave = inbuf16;
    *pPORTF_FER    |= (PF14|PF13|PF12|PF11|PF10);    /* SPISS select PF14 input as slave, */
    /* MOSI PF11 enabled (note shouldn't need PF10 as that's the flash memory) */
    *pPORT_MUX    |= PJSE;     /* Enable PJ10 SSEL2 & 3 PORT_MUX PJSE=1  not required as we're slave.. */
    *pSPI_BAUD    = 8;         /* 4, 8, 16 */
    *pSPI_CTL     = SPE|CPOL|SIZE|EMISO|0x00; /* tc on read */
    SSYNC;

    remainingBytes = bufsize;

    //ix = (unsigned int)crc16_ccitt((void *)buf16, bufsize-8);
    //buf16[(bufsize/2)-2] = ix; /* write CRC into buffer */

    while(remainingBytes)
    {
        while( (*pSPI_STAT&SPIF) == 0 )
            ; /* ensure spif transfer complete */
        while( (*pSPI_STAT&RXS) == 0  )
            ; /* ensure rx buffer full */

        *pSPI_TDBR = *outbuf16++;
        SSYNC;
        *inbuf16++ = *pSPI_RDBR; /* read full 16 bits in one */
        remainingBytes -= 2;
        while( (remainingBytes>0) && ((*pSPI_STAT&TXS) > 0)  )
            ; /* ensure tx buffer empty */
    };

    *pSPI_CTL = 0x400;
    *pSPI_FLG = 0x0;
    *pSPI_BAUD = 0x0;
    SSYNC;
}

/* structure storing data for this camera */
svs_data_struct *svs_data;

/* structure storing data for this camera on the previous frame*/
svs_data_struct *svs_data_previous;

/* structure storing data received from the opposite camera */
svs_data_struct *svs_data_received;

/* buffer which stores sliding sum */
int *row_sum;

/* buffer used to find peaks in edge space */
unsigned int *row_peaks;

/* array stores matching probabilities (prob,x,y,disp) */
unsigned int *svs_matches;

/* used during filtering */
unsigned char *valid_quadrants;

/* array used to store a disparity histogram */
int* disparity_histogram_plane;
int* disparity_plane_fit;

/* maps raw image pixels to rectified pixels */
int *calibration_map;

/* priors used when processing a video stream */
int* disparity_priors;

/* array storing the number of features detected on each column */
unsigned short int* features_per_col;

/* array storing y coordinates of detected features */
short int* feature_y;

/* used during segmentation */
int* curr_ID;
int* curr_ID_hits;
    
extern unsigned int imgWidth, imgHeight;
extern int svs_calibration_offset_x, svs_calibration_offset_y;
extern int svs_centre_of_disortion_x, svs_centre_of_disortion_y;
extern int svs_scale_num, svs_scale_denom;
extern int svs_coeff_degree;
extern long* svs_coeff;
extern int svs_width, svs_height;

/* previous image width.  This is used to detect resolution changes */
int svs_prev_width;

/* array used to estimate footline of objects on the ground plane */
unsigned short int* footline;

/* distances to the footline in mm */
unsigned short int* footline_dist_mm;

/* whether mapping was previously enabled */
int prev_svs_enable_mapping;

/* offsets of pixels to be compared within the patch region
 * arranged into a rectangular structure */
const int pixel_offsets[] =
{
    -2,-4,  -1,-4,         1,-4,  2,-4,
    -5,-2,  -4,-2,  -3,-2,  -2,-2,  -1,-2,  0,-2,  1,-2,  2,-2,  3,-2,  4,-2,  5,-2,
    -5, 2,  -4, 2,  -3, 2,  -2, 2,  -1, 2,  0, 2,  1, 2,  2, 2,  3, 2,  4, 2,  5, 2,
    -2, 4,  -1, 4,         1, 4,  2, 4
};


/* lookup table used for counting the number of set bits */
const unsigned char BitsSetTable256[] =
{
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

/* Updates sliding sums and edge response values along a single row or column
 * Returns the mean luminance along the row or column */
int svs_update_sums(
int cols, /* if non-zero we're dealing with columns not rows */
int i, /* row or column index */
unsigned char* rectified_frame_buf, /* image data */
int segment) { /* if non zero update low contrast areas used for segmentation */

    int j, x, y, idx, max, sum = 0, mean = 0;

    if (cols == 0) {
	/* compute sums along the row */
	y = i;
	idx = (int)imgWidth * y;
	max = (int)imgWidth;

        sum = rectified_frame_buf[idx];
	row_sum[0] = sum;
	for (x = 1; x < max; x++, idx++) {
	    sum += rectified_frame_buf[idx];
	    row_sum[x] = sum;
	}
    } else {
	/* compute sums along the column */
	idx = i;
	x = i;
	max = (int) imgHeight;

        sum = rectified_frame_buf[idx];
	row_sum[0] = sum;
	for (y = 1; y < max; y++, idx+=(int)imgWidth) {
	    sum += rectified_frame_buf[idx];
	    row_sum[y] = sum;
	}
    }

    /* row mean luminance */
    mean = row_sum[max - 1] / (max * 2);

    /* compute peaks */
    int p0, p1;
    for (j = 4; j < max - 4; j++) {
	sum = row_sum[j];
	/* edge using 1 pixel radius */
	p0 = (sum - row_sum[j - 1]) - (row_sum[j + 1] - sum);
	if (p0 < 0) p0 = -p0;

	/* edge using 2 pixel radius */
	p1 = (sum - row_sum[j - 2]) - (row_sum[j + 2] - sum);
	if (p1 < 0) p1 = -p1;

	/* overall edge response */
	row_peaks[j] = (p0 + p1) * 32;
    }
    
    return (mean);
}


/* performs non-maximal suppression on the given row or column */
void svs_non_max(
int cols, /* if non-zero we're dealing with columns not rows */
int inhibition_radius, /* radius for non-maximal suppression */
unsigned int min_response) { /* minimum threshold as a percent in the range 0-200 */

    int i, r, max, max2;
    unsigned int v;

    /* average response */
    unsigned int av_peaks = 0;
    max = (int)imgWidth;
    if (cols != 0) max = (int)imgHeight;
    max2 = max - inhibition_radius;
    max -= 4;
    for (i = 4; i < max; i++) {
	av_peaks += row_peaks[i];
    }

    /* adjust the threshold */
    av_peaks = av_peaks * min_response / (100 * (max - 4));
    
    for (i = 4; i < max2; i++) {
	if (row_peaks[i] > av_peaks) {			
	    v = row_peaks[i];
	    if (v > 0) {
		for (r = 1; r < inhibition_radius; r++) {
		    if (row_peaks[i + r] < v) {
  			row_peaks[i + r] = 0;
		    } else {
			row_peaks[i] = 0;
			break;
		    }
		}
            }
        } else {
  	    row_peaks[i] = 0;	    
  	}
    }
}

/* creates a binary descriptor for a feature at the given coordinate
   which can subsequently be used for matching */
int svs_compute_descriptor(
    int px,
    int py,
    unsigned char* rectified_frame_buf,
    int no_of_features,
    int row_mean)
{

    unsigned char bit_count = 0;
    int pixel_offset_idx, ix, bit;
    int meanval = 0;
    unsigned int desc = 0;

    /* find the mean luminance for the patch */
    for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS*2; pixel_offset_idx += 2)
    {
        ix = rectified_frame_buf[pixindex2((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
        meanval += rectified_frame_buf[ix];
    }
    meanval /= SVS_DESCRIPTOR_PIXELS;

    /* binarise */
    bit = 1;
    for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS*2; pixel_offset_idx += 2, bit *= 2)
    {
        ix = rectified_frame_buf[pixindex2((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
        if (rectified_frame_buf[ix] > meanval)
        {
            desc |= bit;
            bit_count++;
        }
    }

    if ((bit_count > 3) &&
            (bit_count < SVS_DESCRIPTOR_PIXELS-3))
    {
        /* adjust the patch luminance relative to the mean
         * luminance for the entire row.  This helps to ensure
         * that comparisons between left and right images are
         * fair even if there are exposure/illumination differences. */
        meanval = meanval - row_mean + 127;
        if (meanval < 0)
            meanval = 0;
        if (meanval > 255)
            meanval = 255;

        svs_data->mean[no_of_features] = (unsigned char)(meanval/3);
        svs_data->descriptor[no_of_features] = desc;
        return(0);
    }
    else
    {
        /* probably just noise */
        return(-1);
    }
}

/* returns a set of vertically oriented features suitable for stereo matching */
int svs_get_features_vertical(
    unsigned char* rectified_frame_buf,  /* image data */
    int inhibition_radius,               /* radius for non-maximal supression */
    unsigned int minimum_response,       /* minimum threshold */
    int calibration_offset_x,            /* calibration x offset in pixels */
    int calibration_offset_y,            /* calibration y offset in pixels */
    int segment)                         /* if non zero update low contrast areas used for segmentation */
{

    unsigned short int no_of_feats;
    int x, y, row_mean, start_x;
    int no_of_features = 0;
    int row_idx = 0;

    memset((void*)(svs_data->features_per_row), '\0', imgHeight/SVS_VERTICAL_SAMPLING * sizeof(unsigned short));

    start_x = imgWidth - 15;
    if ((int)imgWidth - inhibition_radius - 1 < start_x)
        start_x = (int)imgWidth - inhibition_radius - 1;

    for (y = 4 + calibration_offset_y; y < (int)imgHeight - 4; y += SVS_VERTICAL_SAMPLING)
    {

        /* reset number of features on the row */
        no_of_feats = 0;

        if ((y >= 4) && (y <= (int)imgHeight - 4))
        {

            row_mean = svs_update_sums(0, y, rectified_frame_buf, segment);
            svs_non_max(0, inhibition_radius, minimum_response);

            /* store the features */
            for (x = start_x; x > 15; x--)
            {
                if (row_peaks[x] > 0)
                {

                    if (svs_compute_descriptor(
                                x, y, rectified_frame_buf, no_of_features, row_mean) == 0)
                    {

                        svs_data->feature_x[no_of_features++] = (short int)(x + calibration_offset_x);
                        no_of_feats++;
                        if (no_of_features == SVS_MAX_FEATURES)
                        {
                            y = imgHeight;
                            printf("stereo feature buffer full\n");
                            break;
                        }
                    }
                }
            }
        }

        svs_data->features_per_row[row_idx++] = no_of_feats;
    }
    
    svs_data->no_of_features = no_of_features;
    
#ifdef SVS_VERBOSE 
    printf("%d vertically oriented edge features located\n", no_of_features);
#endif
        
    return(no_of_features);
}

/* returns a set of horizontally oriented features */
int svs_get_features_horizontal(unsigned char* rectified_frame_buf, /* image data */
int inhibition_radius, /* radius for non-maximal supression */
unsigned int minimum_response, /* minimum threshold */
int calibration_offset_x, int calibration_offset_y,
int segment) /* if non zero update low contrast areas used for segmentation */
{
    unsigned short int no_of_feats;
    int x, y, row_mean, start_y;
    int no_of_features = 0;
    int col_idx = 0;

    /* create arrays */
    if (features_per_col == 0) {
        features_per_col = (unsigned short int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING*sizeof(unsigned short int));
        feature_y = (short int*)malloc(SVS_MAX_FEATURES*sizeof(short int));
        footline = (unsigned short int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short int));
    }
    
    int ground_y=(int)imgHeight;
    if (svs_enable_ground_priors) {
        ground_y = (int)imgHeight - 1 - (svs_ground_y_percent*(int)imgHeight/100);        
        memset((void*)footline, '\0', SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING
            * sizeof(unsigned short));
    }

    memset((void*)features_per_col, '\0', SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING
        * sizeof(unsigned short));

    start_y = (int)imgHeight - 15;
    if ((int)imgHeight - inhibition_radius - 1 < start_y)
        start_y = (int)imgHeight - inhibition_radius - 1;

    for (x = 4 + calibration_offset_x; x < (int)imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING) {

        /* reset number of features on the row */
        no_of_feats = 0;

        if ((x >= 4) && (x <= (int)imgWidth - 4)) {

	    row_mean = svs_update_sums(1, x, rectified_frame_buf, segment);
  	    svs_non_max(1, inhibition_radius, minimum_response);

	    /* store the features */
	    for (y = start_y; y > 15; y--) {
	        if (row_peaks[y] > 0) {
	        
	            if (svs_enable_ground_priors) {
	                if (y >= ground_y) {
                            footline[x / SVS_HORIZONTAL_SAMPLING] = y;
	                }
	            }

		    if (svs_compute_descriptor(x, y, rectified_frame_buf, no_of_features, row_mean) == 0) {
		        feature_y[no_of_features++] = (short int) (y + calibration_offset_y);
		        no_of_feats++;
		        if (no_of_features == SVS_MAX_FEATURES) {
			    x = (int)imgWidth;
			    printf("stereo feature buffer full\n");
			    break;
		        }
		    }
	        }
	    }
        }

        features_per_col[col_idx++] = no_of_feats;
    }
    
#ifdef SVS_VERBOSE 
    printf("%d horizontally oriented edge features located\n", no_of_features);
#endif

    return (no_of_features);
}

/* sends feature data from the right camera board to the left */
void svs_send_features()
{
    /* create checksum */
    svs_data->crc = 0;
    svs_data->received_bit = 1;
    svs_data->crc = (unsigned short)crc16_ccitt((void *)svs_data, sizeof(svs_data));

    /* copy data into the send buffer */
    memcpy(
        (void*)(FLASH_BUFFER+16384), 
        (void*)svs_data,
        16384);
    
#ifdef SVS_VERBOSE
    printf("svs slave waiting to send features\n");
#endif
    /* send it */
    svs_slave(
        (unsigned short *)FLASH_BUFFER,
        (unsigned short *)(FLASH_BUFFER+16384), 
        16384);

#ifdef SVS_VERBOSE
    printf("%d features sent\n", svs_data->no_of_features);
#endif
}

/* left camera board receives feature data from the right camera board */
int svs_receive_features()
{
#ifdef SVS_PROFILE
    int t[3];
#endif
    unsigned short crc, crc_received;
    int features_received = 0;
    
#ifdef SVS_PROFILE
    t[0] = readRTC();
#endif
    svs_master(
        (unsigned short *)FLASH_BUFFER,
        (unsigned short *)(FLASH_BUFFER+16384), 
        16384);
#ifdef SVS_PROFILE
    t[1] = readRTC();
#endif
    svs_data_received = (svs_data_struct*)(FLASH_BUFFER+16384);
                
    if (svs_data_received->received_bit != 0) {

        /* run checksum */        
        crc_received = svs_data_received->crc;
        svs_data_received->crc = 0;
        crc = (unsigned short)crc16_ccitt((void *)svs_data_received, sizeof(svs_data_received));
        
#ifdef SVS_PROFILE
        t[2] = readRTC();
#endif    
        if (crc == crc_received) {
            features_received = svs_data_received->no_of_features;
        }
        else {
            printf("Checksum invalid\n");
            features_received = -2;
        }
    }
    else {
#ifdef SVS_PROFILE
        t[2] = 0;
#endif
        printf("Received bit not set\n");
        features_received = -1;
    }
        
#ifdef SVS_VERBOSE
    printf("%d features received\n", svs_data_received->no_of_features);
#endif
#ifdef SVS_PROFILE
    printf("svs_receive_features:\n\r");
    printf("    svs_master  %d\n\r", t[1]-t[0]);
    printf("    crc16_ccitt %d\n\r", t[2]-t[1]);
#endif
        
    return(features_received);
}

/* reads camera calibration parameters from flash sector 03 */
int svs_read_calib_params()
{
    int coeff, is_valid=1;
    read_user_sector(SVS_SECTOR);
    int* buf = (int *)FLASH_BUFFER;
    svs_calibration_offset_x = buf[0];
    svs_calibration_offset_y = buf[1];
    svs_centre_of_disortion_x = imgWidth / 2;
    svs_centre_of_disortion_y = (int)imgHeight / 2;
    if ((buf[2] > 0) && (buf[3] > 0) &&
        (buf[2] < 3000) && (buf[3] < 3000)) {
        svs_centre_of_disortion_x = buf[2];
        svs_centre_of_disortion_y = buf[3];        
    }
    else {
        is_valid = 0;
    }
    svs_scale_num = 1;
    svs_scale_denom = 1;
    svs_coeff[0] = 0;
    svs_width = 320;
    svs_height = 240;
    if ((buf[4] > 0) && (buf[5] > 0)) {
        svs_scale_num = buf[4];
        svs_scale_denom = buf[5];
        svs_coeff_degree = buf[6];
        if ((svs_coeff_degree > 5) || (svs_coeff_degree < 3)) svs_coeff_degree = 3;
        for (coeff = 1; coeff <= svs_coeff_degree; coeff++) {
            svs_coeff[coeff] = buf[6+coeff];
        }
        if ((buf[7+svs_coeff_degree] > 100) && 
            (buf[7+svs_coeff_degree] <= SVS_MAX_IMAGE_WIDTH)) {
            svs_width = buf[7+svs_coeff_degree];
            svs_height = buf[8+svs_coeff_degree];
        }
    }
    else {
        is_valid = 0;
        svs_coeff_degree = 3;
        svs_coeff[1] = 10954961;
        svs_coeff[2] = -10710;
        svs_coeff[3] = -6;
    }
            
#ifdef SVS_VERBOSE
    if (is_valid != 0) {
        printf("offset: %d %d\n", svs_calibration_offset_x, svs_calibration_offset_y);
        printf("centre: %d %d\n", svs_centre_of_disortion_x, svs_centre_of_disortion_y);
        printf("scale:  %d/%d\n", svs_scale_num, svs_scale_denom);
        printf("coefficients:  ");        
        for (coeff = 1; coeff <= svs_coeff_degree; coeff++) {
            printf("%d ", svs_coeff[coeff]);
        }
        printf("\n");
    }
    else {
        printf("Calibration parameters were not found\n");
        int i;
        for (i = 0; i < 9; i++) {
            printf("buffer %d: %d\n", i, buf[i]);
        }
    }
#endif

    /* create the calibration map using the parameters */
    svs_make_map(
        (long)svs_centre_of_disortion_x, 
        (long)svs_centre_of_disortion_y, 
        svs_coeff, 
        svs_coeff_degree,
        svs_scale_num, svs_scale_denom);

    return(is_valid);
}


/* updates a set of features suitable for stereo matching */
int svs_grab(
    int calibration_offset_x,  /* calibration x offset in pixels */
    int calibration_offset_y,  /* calibration y offset in pixels */
    int left_camera,           /* if non-zero this is the left camera */
    int show_feats)            /* if non-zero then show features */
{
#ifdef SVS_PROFILE
    int t[8];
#endif
    int no_of_feats;
    /* some default values */
    const int inhibition_radius = 16;
    const unsigned int minimum_response = 300;
    
#ifdef SVS_PROFILE
    t[0] = readRTC();    
#endif
    
    /* clear received data bit */
    svs_data->received_bit = 0;

    /* grab new frame */
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, (unsigned char *)FRAME_BUF, imgWidth, imgHeight);

#ifdef SVS_PROFILE
    t[1] = readRTC();    
#endif

    /* if the image resolution has changed recompute the calibration map */
    if (svs_prev_width != svs_width) {
        svs_read_calib_params();
        svs_prev_width = svs_width;
    }

    /* rectify the image */
    svs_rectify((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3);

#ifdef SVS_PROFILE
    t[2] = readRTC();    
#endif

    /* copy rectified back to the original */
    memcpy((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, imgWidth * imgHeight * 4);

#ifdef SVS_PROFILE
    t[3] = readRTC();    
#endif

    /* convert to YUV */
    move_yuv422_to_planar((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, imgWidth, imgHeight);

#ifdef SVS_PROFILE
    t[4] = readRTC();    
#endif

    /* compute edge features */
    no_of_feats = svs_get_features_vertical((unsigned char *)FRAME_BUF3, inhibition_radius, minimum_response, calibration_offset_x, calibration_offset_y, left_camera);

#ifdef SVS_PROFILE
    t[5] = readRTC();    
    t[6] = 0;
#endif
    
    if ((left_camera != 0) && (svs_enable_horizontal)) {
        svs_get_features_horizontal((unsigned char *)FRAME_BUF3, inhibition_radius, minimum_response,     calibration_offset_x, calibration_offset_y, left_camera);        
#ifdef SVS_PROFILE
        t[6] = readRTC();        
#endif
    }
    
#ifdef SVS_PROFILE
    t[7] = 0;
#endif
    if (show_feats != 0) {
        /* display the features */   
        svs_show_features((unsigned char *)FRAME_BUF, no_of_feats);
#ifdef SVS_PROFILE
        t[7] = readRTC();        
#endif
    }
    
#ifdef SVS_PROFILE
    printf("svs_grab:\n\r");
    printf("    move_image                  %d\n\r", t[1]-t[0]);
    printf("    svs_rectify                 %d\n\r", t[2]-t[1]);
    printf("    memcpy                      %d\n\r", t[3]-t[2]);
    printf("    move_yuv422_to_planar       %d\n\r", t[4]-t[3]);
    printf("    svs_get_features_vertical   %d\n\r", t[5]-t[4]);
    printf("    svs_get_features_horizontal %d\n\r", t[6]-t[5]);
    if (show_feats != 0) printf("    svs_show_features           %d\n\r", t[7]-t[6]);
#endif    
    return(no_of_feats);
}

/* Match features from this camera with features from the opposite one.
 * It is assumed that matching is performed on the left camera CPU */
int svs_match(
    int ideal_no_of_matches,          /* ideal number of matches to be returned */
    int max_disparity_percent,        /* max disparity as a percent of image width */
    int descriptor_match_threshold,   /* minimum no of descriptor bits to be matched, in the range 1 - SVS_DESCRIPTOR_PIXELS */
    int learnDesc,                    /* descriptor match weight */
    int learnLuma,                    /* luminance match weight */
    int learnDisp,                    /* disparity weight */
    int learnPrior,                   /* prior weight */
    int groundPrior,                  /* prior weight for ground plane */
    int use_priors)                   /* if non-zero then use priors, assuming time between frames is small */ 
{
    int x, xL=0, xR, L, R, y, no_of_feats=0, no_of_feats_left;
    int no_of_feats_right, row, col, bit, disp_diff=0;
    int luma_diff, min_disp=0, max_disp=0, disp_prior=0, max_disp_pixels;
    int meanL, meanR, disp, fL=0, fR=0, bestR=0;
    unsigned int descL, descLanti, descR, desc_match;
    unsigned int correlation, anticorrelation, total, n;
    unsigned int match_prob, best_prob;
    int max, curr_idx, search_idx, winner_idx=0;
    int no_of_possible_matches = 0, matches = 0;
    int itt, idx, prev_matches, row_offset, col_offset;

    unsigned int meandescL, meandescR;
    short meandesc[SVS_DESCRIPTOR_PIXELS];
    
    /* initialise arrays used for matching
     * note that we only need to do this on the left camera */
    if (svs_matches == 0) init_svs_matching_data();

    /* convert max disparity from percent to pixels */
    max_disp_pixels = max_disparity_percent * imgWidth / 100;
    min_disp = -10;
    max_disp = max_disp_pixels;
    
    /* find ground plane */
    int ground_prior=0;
    int ground_y = (int)imgHeight * svs_ground_y_percent/100;    
    int footline_y=0;
    int ground_y_sloped=0, ground_height_sloped=0;
    int obstaclePrior = 20;
    int half_width = (int)imgWidth/2;
    svs_ground_plane();
    
    row = 0;
    for (y = 4; y < (int)imgHeight - 4; y += SVS_VERTICAL_SAMPLING, row++)
    {

        /* number of features on left and right rows */
        no_of_feats_left = svs_data->features_per_row[row];
        no_of_feats_right = svs_data_received->features_per_row[row];

        /* compute mean descriptor for the left row
         * this will be used to create eigendescriptors */
        meandescL = 0;
        memset((void *)&meandesc, '\0', (SVS_DESCRIPTOR_PIXELS)* sizeof(short));
        for (L = 0; L < no_of_feats_left; L++)
        {
            descL = svs_data->descriptor[fL + L];
            n = 1;
            for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
            {
                if (descL & n)
                    meandesc[bit]++;
                else
                    meandesc[bit]--;
            }
        }
        n = 1;
        for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
        {
            if (meandesc[bit] >= 0)
                meandescL |= n;
        }

        /* compute mean descriptor for the right row
         * this will be used to create eigendescriptors */
        meandescR = 0;
        memset((void *)&meandesc, '\0', (SVS_DESCRIPTOR_PIXELS)* sizeof(short));
        for (R = 0; R < no_of_feats_right; R++)
        {
            descR = svs_data_received->descriptor[fR + R];
            n = 1;
            for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
            {
                if (descR & n)
                    meandesc[bit]++;
                else
                    meandesc[bit]--;
            }
        }
        n = 1;
        for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
        {
            if (meandesc[bit] > 0)
                meandescR |= n;
        }

        /* features along the row in the left camera */
        for (L = 0; L < no_of_feats_left; L++)
        {

            /* x coordinate of the feature in the left camera */
            xL = svs_data->feature_x[fL + L];

	    if (use_priors != 0) {
		disp_prior = disparity_priors[(row * imgWidth + xL) / 16];
		if (svs_enable_ground_priors) {
		    /* Here svs_ground_slope_percent is a simple way of representing the
		       roll angle of the robot, without using any floating point maths.
		       Positive slope corresponds to a positive (clockwise) roll angle.
		       The slope is the number of pixels of vertical displacement of
		       the horizon at the far right of the image, relative to the centre 
		       of the image, and is expressed as a percentage of the image height */
                    ground_y_sloped = ground_y + ((half_width - xL) * svs_ground_slope_percent / 100);
		    ground_height_sloped = (int)imgHeight - 1 - ground_y_sloped;
		    footline_y = footline[xL / SVS_HORIZONTAL_SAMPLING];
		    ground_prior = (footline_y - ground_y_sloped) * max_disp_pixels / ground_height_sloped;		
		}
	    }

            /* mean luminance and eigendescriptor for the left camera feature */
            meanL = svs_data->mean[fL + L];
            descL = svs_data->descriptor[fL + L] & meandescL;

            /* invert bits of the descriptor for anti-correlation matching */
            n = descL;
            descLanti = 0;
            for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++)
            {
                /* Shift result vector to higher significance. */
                descLanti <<= 1;
                /* Get least significant input bit. */
                descLanti |= n & 1;
                /* Shift input vector to lower significance. */
                n >>= 1;
            }

            total = 0;

            /* features along the row in the right camera */
            for (R = 0; R < no_of_feats_right; R++)
            {

                /* set matching score to zero */
                row_peaks[R] = 0;

                /* x coordinate of the feature in the right camera */
                xR = svs_data_received->feature_x[fR + R];

                /* compute disparity */
                disp = xL - xR;

                /* is the disparity within range? */
                if ((disp >= min_disp) && (disp < max_disp))
                {
                    if (disp < 0)
                        disp = 0;


                    /* mean luminance for the right camera feature */
                    meanR = svs_data_received->mean[fR + R];

                    /* is the mean luminance similar? */
                    luma_diff = meanR - meanL;

                    /* right camera feature eigendescriptor */
                    descR = svs_data_received->descriptor[fR + R] & meandescR;

                    /* bitwise descriptor correlation match */
                    desc_match = descL & descR;

                    /* count the number of correlation bits */
                    correlation =
                        BitsSetTable256[desc_match & 0xff] +
                        BitsSetTable256[(desc_match >> 8) & 0xff] +
                        BitsSetTable256[(desc_match >> 16) & 0xff] +
                        BitsSetTable256[desc_match >> 24];

                    /* were enough bits matched ? */
                    if ((int)correlation >= descriptor_match_threshold)
                    {
                        /* bitwise descriptor anti-correlation match */
                        desc_match = descLanti & descR;

                        /* count the number of anti-correlation bits */
                        anticorrelation =
                            BitsSetTable256[desc_match & 0xff] +
                            BitsSetTable256[(desc_match >> 8) & 0xff] +
                            BitsSetTable256[(desc_match >> 16) & 0xff] +
                            BitsSetTable256[desc_match >> 24];

                        if (luma_diff < 0)
                            luma_diff = -luma_diff;
                        int score =
                            10000 +
                            (max_disp * learnDisp) +
                            (((int)correlation + (int)(SVS_DESCRIPTOR_PIXELS - anticorrelation)) * learnDesc) -
                            (luma_diff * learnLuma) -
                            (disp * learnDisp);

			if (use_priors) {
			    disp_diff = disp - disp_prior;
			    if (disp_diff < 0)
				disp_diff = -disp_diff;
			    score -= disp_diff * learnPrior;
			    
			    /* bias for ground_plane */
			    if (svs_enable_ground_priors) {
			        if (y > footline_y) {
			            /* below the footline - bias towards ground plane */
  			            disp_diff = disp - ((y - ground_y_sloped) * max_disp_pixels / ground_height_sloped);
			            if (disp_diff < 0) disp_diff = -disp_diff;
			            score -= disp_diff * groundPrior;
			        }
			        else {
			            if (ground_prior > 0) {
			                /* above the footline - bias towards obstacle*/
			                disp_diff = disp - ground_prior;
			                if (disp_diff < 0) disp_diff = -disp_diff;
			                score -= disp_diff * obstaclePrior;
			            }
			        }
			    }
			}

                        if (score < 0)
                            score = 0;

                        /* store overall matching score */
                        row_peaks[R] = (unsigned int)score;
                        total += row_peaks[R];
                    }
                }
                else
                {
                    if ((disp < min_disp) && (disp > -max_disp))
                    {
                        row_peaks[R] = (unsigned int)((max_disp - disp) * learnDisp);
                        total += row_peaks[R];
                    }
                }
            }

            /* non-zero total matching score */
            if (total > 0)
            {

                /* convert matching scores to probabilities */
                best_prob = 0;
                for (R = 0; R < no_of_feats_right; R++)
                {
                    if (row_peaks[R] > 0)
                    {
                        match_prob = row_peaks[R] * 1000 / total;
                        if (match_prob > best_prob)
                        {
                            best_prob = match_prob;
                            bestR = R;
                        }
                    }
                }

                if ((best_prob > 0) &&
                        (best_prob < 1000) &&
                        (no_of_possible_matches < SVS_MAX_FEATURES))
                {

                    /* x coordinate of the feature in the right camera */
                    xR = svs_data_received->feature_x[fR + bestR];

                    /* possible disparity */
                    disp = xL - xR;

                    if (disp >= min_disp)
                    {
                        if (disp < 0)
                            disp = 0;
                        /* add the best result to the list of possible matches */
                        svs_matches[no_of_possible_matches*4] = best_prob;
                        svs_matches[no_of_possible_matches*4 + 1] = (unsigned int)xL;
                        svs_matches[no_of_possible_matches*4 + 2] = (unsigned int)y;
                        svs_matches[no_of_possible_matches*4 + 3] = (unsigned int)disp;
                        no_of_possible_matches++;
                    }
                }
            }
        }

        /* increment feature indexes */
        fL += no_of_feats_left;
        fR += no_of_feats_right;
    }

    // clear priors
    int priors_length = (int)(imgWidth * imgHeight) / (16*SVS_VERTICAL_SAMPLING);
    
    if (no_of_possible_matches > 20)
    {    
        memset((void*)disparity_priors, '\0', priors_length*sizeof(int));
        
        /* filter the results */
        svs_filter_plane(no_of_possible_matches, max_disp);

        /* sort matches in descending order of probability */
        if (no_of_possible_matches < ideal_no_of_matches)
        {
            ideal_no_of_matches = no_of_possible_matches;
        }
        curr_idx = 0;
        search_idx = 0;
        for (matches = 0; matches < ideal_no_of_matches; matches++, curr_idx += 4)
        {

            match_prob =  svs_matches[curr_idx];
            winner_idx = -1;

            search_idx = curr_idx + 4;
            max = no_of_possible_matches * 4;
            while (search_idx < max)
            {
                if (svs_matches[search_idx] > match_prob)
                {
                    match_prob = svs_matches[search_idx];
                    winner_idx = search_idx;
                }
                search_idx += 4;
            }
            if (winner_idx > -1)
            {

                /* swap */
                best_prob = svs_matches[winner_idx];
                xL = svs_matches[winner_idx + 1];
                y = svs_matches[winner_idx + 2];
                disp = svs_matches[winner_idx + 3];

                svs_matches[winner_idx] = svs_matches[curr_idx];
                svs_matches[winner_idx + 1] = svs_matches[curr_idx + 1];
                svs_matches[winner_idx + 2] = svs_matches[curr_idx + 2];
                svs_matches[winner_idx + 3] = svs_matches[curr_idx + 3];

                svs_matches[curr_idx] = best_prob;
                svs_matches[curr_idx + 1] = xL;
                svs_matches[curr_idx + 2] = y;
                svs_matches[curr_idx + 3] = disp;
                
		/* update your priors */
		row = y / SVS_VERTICAL_SAMPLING;
		for (row_offset = -3; row_offset <= 3; row_offset++) {
			for (col_offset = -1; col_offset <= 1; col_offset++) {
				idx = (((row + row_offset) * (int)imgWidth + xL) / 16) + col_offset;
				if ((idx > -1) && (idx < priors_length)) {
			            if (disparity_priors[idx] == 0)
				        disparity_priors[idx] = disp;
  				    else
				        disparity_priors[idx] = (disp+disparity_priors[idx])/2;
				}
			}
		}
            }

            if (svs_matches[curr_idx] == 0)
            {
                break;
            }
        }

	/* attempt to assign disparities to horizontally oriented features */
	if ((svs_enable_horizontal != 0) &&
	    (features_per_col != 0)) {
 	    memset((void*)valid_quadrants, '\0', SVS_MAX_MATCHES * sizeof(unsigned char));
	    itt = 0;
	    prev_matches = matches;
	    for (itt = 0; itt < 10; itt++) {
		fL = 0;
		col = 0;
		for (x = 4; x < (int)imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING, col++) {

			no_of_feats = features_per_col[col];

			// features along the row in the left camera
			for (L = 0; L < no_of_feats; L++) {

				if (valid_quadrants[fL + L] == 0) {
					// y coordinate of the feature in the left camera 
					y = feature_y[fL + L];

					// lookup disparity from priors

					row = y / SVS_VERTICAL_SAMPLING;
					disp_prior = disparity_priors[(row * (int)imgWidth + x) / 16];

					if ((disp_prior > 0) &&
						(matches < SVS_MAX_MATCHES)) {
						curr_idx = matches * 4;
						svs_matches[curr_idx] = 1000;
						svs_matches[curr_idx + 1] = x;
						svs_matches[curr_idx + 2] = y;
						svs_matches[curr_idx + 3] = disp_prior;
						matches++;
						
						//printf("horizontal match\n");
						
						// update your priors 
						for (row_offset = -3; row_offset <= 3; row_offset++) {
							for (col_offset = -1; col_offset <= 1; col_offset++) {
								idx = (((row + row_offset) * (int)imgWidth + x) / 16) + col_offset;
								if ((idx > -1) && (idx < priors_length)) {
								    if (disparity_priors[idx] == 0) {
									disparity_priors[idx] = disp_prior;
								    }
								}
							}
						}

						valid_quadrants[fL + L] = 1;
					}
				}
			}
			fL += no_of_feats;
	        }
	        if (prev_matches == matches) break;
	        prev_matches = matches;
	    }
	}
    }        
        
#ifdef SVS_VERBOSE
    printf("%d stereo matches\n", matches);
#endif

    return(matches);
}

/* filtering function removes noise by searching for a peak in the disparity histogram */
void svs_filter_plane(int no_of_possible_matches, /* the number of stereo matches */
int max_disparity_pixels) /*maximum disparity in pixels */
{
	int i, hf, hist_max, w = SVS_FILTER_SAMPLING, w2, n, horizontal = 0;
	unsigned int x, y, disp, tx = 0, ty = 0, bx = 0, by = 0;
	int hist_thresh, hist_mean, hist_mean_hits, mass, disp2;
	int min_ww, max_ww, m, ww, d;
	int ww0, ww1, disp0, disp1, cww, dww, ddisp;

	/* clear quadrants */
	memset(valid_quadrants, 0, no_of_possible_matches * sizeof(unsigned char));

	/* create disparity histograms within different
	 * zones of the image */
	for (hf = 0; hf < 11; hf++) {

		switch (hf) {

		// overall horizontal
		case 0: {
			tx = 0;
			ty = 0;
			bx = imgWidth;
			by = imgHeight;
			horizontal = 1;
			w = bx;
			break;
		}
			// overall vertical
		case 1: {
			tx = 0;
			ty = 0;
			bx = imgWidth;
			by = imgHeight;
			horizontal = 0;
			w = by;
			break;
		}
			// left hemifield 1
		case 2: {
			tx = 0;
			ty = 0;
			bx = imgWidth / 3;
			by = imgHeight;
			horizontal = 1;
			w = bx;
			break;
		}
			// left hemifield 2
		case 3: {
			tx = 0;
			ty = 0;
			bx = imgWidth / 2;
			by = imgHeight;
			horizontal = 1;
			w = bx;
			break;
		}

			// centre hemifield (vertical)
		case 4: {
			tx = imgWidth / 3;
			bx = imgWidth * 2 / 3;
			w = imgHeight;
			horizontal = 0;
			break;
		}
			// centre above hemifield
		case 5: {
			tx = imgWidth / 3;
			ty = 0;
			bx = imgWidth * 2 / 3;
			by = imgHeight / 2;
			w = by;
			horizontal = 0;
			break;
		}
			// centre below hemifield
		case 6: {
			tx = imgWidth / 3;
			ty = imgHeight / 2;
			bx = imgWidth * 2 / 3;
			by = imgHeight;
			w = ty;
			horizontal = 0;
			break;
		}
			// right hemifield 0
		case 7: {
			tx = imgWidth * 2 / 3;
			bx = imgWidth;
			horizontal = 1;
			break;
		}
			// right hemifield 1
		case 8: {
			tx = imgWidth / 2;
			bx = imgWidth;
			w = tx;
			break;
		}
			// upper hemifield
		case 9: {
			tx = 0;
			ty = 0;
			bx = imgWidth;
			by = imgHeight / 2;
			horizontal = 0;
			w = by;
			break;
		}
			// lower hemifield
		case 10: {
			ty = by;
			by = imgHeight;
			horizontal = 0;
			break;
		}
		}

		/* clear the histogram */
		w2 = w / SVS_FILTER_SAMPLING;
		if (w2 < 1)
			w2 = 1;
		memset((void*) disparity_histogram_plane, '\0', w2
				* max_disparity_pixels * sizeof(int));
		memset((void*) disparity_plane_fit, '\0', w2 * sizeof(int));
		hist_max = 0;

		/* update the disparity histogram */
		n = 0;
		for (i = 0; i < no_of_possible_matches; i++) {
			x = svs_matches[i * 4 + 1];
			if ((x > tx) && (x < bx)) {
				y = svs_matches[i * 4 + 2];
				if ((y > ty) && (y < by)) {
					disp = svs_matches[i * 4 + 3];
					if ((int) disp < max_disparity_pixels) {
						if (horizontal != 0) {
							n = (((x - tx) / SVS_FILTER_SAMPLING)
									* max_disparity_pixels) + disp;
						} else {
							n = (((y - ty) / SVS_FILTER_SAMPLING)
									* max_disparity_pixels) + disp;
						}
						disparity_histogram_plane[n]++;
						if (disparity_histogram_plane[n] > hist_max)
							hist_max = disparity_histogram_plane[n];
					}
				}
			}
		}

		/* find peak disparities along a range of positions */
		hist_thresh = hist_max / 4;
		hist_mean = 0;
		hist_mean_hits = 0;
		disp2 = 0;
		min_ww = w2;
		max_ww = 0;
		for (ww = 0; ww < (int) w2; ww++) {
			mass = 0;
			disp2 = 0;
			for (d = 1; d < max_disparity_pixels - 1; d++) {
				n = ww * max_disparity_pixels + d;
				if (disparity_histogram_plane[n] > hist_thresh) {
					m = disparity_histogram_plane[n]
							+ disparity_histogram_plane[n - 1]
							+ disparity_histogram_plane[n + 1];
					mass += m;
					disp2 += m * d;
				}
				if (disparity_histogram_plane[n] > 0) {
					hist_mean += disparity_histogram_plane[n];
					hist_mean_hits++;
				}
			}
			if (mass > 0) {
				// peak disparity at this position
				disparity_plane_fit[ww] = disp2 / mass;
				if (min_ww == (int) w2)
					min_ww = ww;
				if (ww > max_ww)
					max_ww = ww;
			}
		}
		hist_mean /= hist_mean_hits;

		/* fit a line to the disparity values */
		ww0 = 0;
		ww1 = 0;
		disp0 = 0;
		disp1 = 0;
		int hits0,hits1;
		if (max_ww >= min_ww) {
			cww = min_ww + ((max_ww - min_ww) / 2);
			hits0 = 0;
			hits1 = 0;
			for (ww = min_ww; ww <= max_ww; ww++) {
				if (ww < cww) {
					disp0 += disparity_plane_fit[ww];
					ww0 += ww;
					hits0++;
				} else {
					disp1 += disparity_plane_fit[ww];
					ww1 += ww;
					hits1++;
				}
			}
			if (hits0 > 0) {
				disp0 /= hits0;
				ww0 /= hits0;
			}
			if (hits1 > 0) {
				disp1 /= hits1;
				ww1 /= hits1;
			}
		}
		dww = ww1 - ww0;
		ddisp = disp1 - disp0;

		/* remove matches too far away from the peak by setting
		 * their probabilities to zero */
		for (i = 0; i < no_of_possible_matches; i++) {
			x = svs_matches[i * 4 + 1];
			if ((x > tx) && (x < bx)) {
				y = svs_matches[i * 4 + 2];
				if ((y > ty) && (y < by)) {
					disp = svs_matches[i * 4 + 3];

					if (horizontal != 0) {
						ww = (x - tx) / SVS_FILTER_SAMPLING;
						n = ww * max_disparity_pixels + disp;
					} else {
						ww = (y - ty) / SVS_FILTER_SAMPLING;
						n = ww * max_disparity_pixels + disp;
					}

					if (dww > 0) {
						disp2 = disp0 + ((ww - ww0) * ddisp / dww);
					}
					else {
						disp2 = disp0;
					}

					if (((int)disp >= disp2-2) &&
					    ((int)disp <= disp2+2) &&
					    ((int)disp < max_disparity_pixels)) {
					    /* this disparity lies along the plane */
					    valid_quadrants[i]++;
					}
				}
			}
		}
	}

	for (i = 0; i < no_of_possible_matches; i++) {
		if (valid_quadrants[i] == 0) {
			/* set probability to zero */
			svs_matches[i * 4] = 0;
		}
	}
}

/* takes the raw image and returns a rectified version */
void svs_rectify(
    unsigned char* raw_image,     /* raw image grabbed from camera (4 bytes per pixel) */
    unsigned char* rectified_frame_buf) /* returned rectified image (4 bytes per pixel) */
{
    int i, max, idx0, idx1;

    if (calibration_map != 0) {

        max = (int)(imgWidth * imgHeight * 2)-2;
        for (i = max; i >= 0; i-=2) {
            idx0 = i & 0xFFFFFFFC;
            idx1 = calibration_map[idx0];
            rectified_frame_buf[idx0++] = raw_image[idx1++];
            rectified_frame_buf[idx0++] = raw_image[idx1++];
            rectified_frame_buf[idx0++] = raw_image[idx1++];
            rectified_frame_buf[idx0] = raw_image[idx1];
        }
        
    }
    else {
        printf("No calibration map has been defined\n");
    }
}

/* initialises arrays */
void init_svs() {
    calibration_map = (int *)malloc(SVS_MAX_IMAGE_WIDTH*SVS_MAX_IMAGE_HEIGHT*4);
    row_peaks = (unsigned int *)malloc(SVS_MAX_IMAGE_WIDTH*4);
    row_sum = (int*)malloc(SVS_MAX_IMAGE_WIDTH*4);
    svs_data = (svs_data_struct *)malloc(sizeof(svs_data_struct));
    svs_matches = 0;
    features_per_col = 0;
    feature_y = 0;
    svs_enable_horizontal = 1;
    
    svs_coeff = (long*)malloc(8 * sizeof(long));
    svs_read_calib_params();
    svs_prev_width = svs_width;
}

/* initialises arrays associated with matching
 * This only needs to be done on the left camera */
void init_svs_matching_data() {
    /* size of the image on the CCD/CMOS sensor 
       OV9655 4.17mm 
       OV7725 (low light sensor) 3.98mm */
    svs_sensor_width_mmx100 = 417; // TODO possible autodetect
    svs_ground_y_percent = 50; // ground plane as a percent of the image height
    svs_enable_ground_priors = 1;
    svs_ground_slope_percent = 0;
    svs_right_turn_percent = 0;
    svs_turn_tollerance_percent = 10;
#ifdef SVS_MAPPING_BY_DEFAULT    
    svs_enable_mapping = 1;
#else
    svs_enable_mapping = 0;
#endif
    prev_svs_enable_mapping = 0;
            
    svs_data_previous = (svs_data_struct *)malloc(sizeof(svs_data_struct));
    memset((void*)svs_data_previous->features_per_row, '\0', SVS_MAX_IMAGE_HEIGHT/SVS_VERTICAL_SAMPLING*sizeof(unsigned short int));
    
    footline_dist_mm = (unsigned short int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short int));   
    memset((void*)footline_dist_mm, '\0', SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short int));
    
    svs_matches = (unsigned int *)malloc(SVS_MAX_MATCHES*16);
    disparity_histogram_plane = (int*)malloc((SVS_MAX_IMAGE_WIDTH/SVS_FILTER_SAMPLING)*(SVS_MAX_IMAGE_WIDTH / 2)*sizeof(int));
    disparity_plane_fit = (int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_FILTER_SAMPLING * sizeof(int));
    valid_quadrants = (unsigned char*)malloc(SVS_MAX_MATCHES);
    disparity_priors = (int*)malloc((SVS_MAX_IMAGE_WIDTH*SVS_MAX_IMAGE_HEIGHT/(16*SVS_VERTICAL_SAMPLING))*4);
    
}

/* shows edge features */
void svs_show_features(
unsigned char *outbuf,  /* output image.  Note that this is YUVY */
int no_of_feats) {      /* number of detected features */

    int n, f, x, y, row = 0;
    int feats_remaining = svs_data->features_per_row[row];
    
    for (f = 0; f < no_of_feats; f++, feats_remaining--) {

        x = (int)svs_data->feature_x[f] - svs_calibration_offset_x;
	y = 4 + (row * SVS_VERTICAL_SAMPLING) + svs_calibration_offset_y;

        if ((y > 0) && 
            (y < (int)imgHeight-1)) {

            /* draw edge */
            n = pixindex(x, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y-1);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y-2);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y+1);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y+2);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x-1, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x+1, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x-2, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x+2, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
        }
        
        /* move to the next row */
        if (feats_remaining <= 0) {
            row++;
            feats_remaining = svs_data->features_per_row[row];
        }
    }
}

/* calculates a distance for each point on the footline */
void svs_footline_update(
    int max_disparity_percent)
{
    const int range_const = SVS_FOCAL_LENGTH_MMx100 * SVS_BASELINE_MM;
    int max = (int)imgWidth / (int)SVS_HORIZONTAL_SAMPLING;
    int i, x, y, disp, disp_mmx100, y_mm;
    int ground_y = (int)imgHeight * svs_ground_y_percent/100;
    int ground_y_sloped=0, ground_height_sloped=0;    
    int half_width = (int)imgWidth/2;
    int max_disp_pixels = max_disparity_percent * (int)imgWidth / 100;    
    int forward_movement_mm = 0;
    int forward_movement_hits = 0;
        
    for (i = 0; i < max; i++) {
        x = i * SVS_HORIZONTAL_SAMPLING;
        y = footline[i];
        ground_y_sloped = ground_y + ((half_width - x) * svs_ground_slope_percent / 100);
        ground_height_sloped = (int)imgHeight - 1 - ground_y_sloped;
        disp = (y - ground_y_sloped) * max_disp_pixels / ground_height_sloped;
        disp_mmx100 = disp * svs_sensor_width_mmx100 / (int)imgWidth;
        if (disp_mmx100 > 0) {
            // get position of the feature in space 
            y_mm = range_const / disp_mmx100;

            if (footline_dist_mm[i] != 0) {
                forward_movement_mm += y_mm - footline_dist_mm[i];
                forward_movement_hits++;
            }
            footline_dist_mm[i] = y_mm;
        }
        else {
            footline_dist_mm[i] = 0;
        }
    }    
    if (forward_movement_hits > 0) {
        forward_movement_mm /= forward_movement_hits;
        //printf("forward movement %d mm\n", forward_movement_mm);
    }
}

/* finds the ground plane */
void svs_ground_plane()
{
#ifdef SVS_PROFILE
    int t = readRTC();
#endif

    if (svs_enable_ground_priors) {
        int i, j, x, y, diff, prev_y = 0, prev_i=0;
        
        /* Remove points which have a large vertical difference.
           Successive points with small vertical difference are likely
           to correspond to real borders rather than carpet patterns, etc*/
        int max_diff = (int)imgHeight / 30;        
        for (i = 0; i < (int)imgWidth / SVS_HORIZONTAL_SAMPLING; i++) {
            x = i * SVS_HORIZONTAL_SAMPLING;
            y = footline[i];
            if (y != 0) {
                if (prev_y != 0) {
                    diff = y - prev_y;
                    if (diff < 0) diff = -diff;
                    if (diff > max_diff) {
                        if (y < prev_y) {
                            footline[prev_i] = 0;
                        }
                        else {
                            footline[i] = 0;
                        }
                    }
                }
                prev_i = i;
                prev_y = y;
            }
        }
        
        /* fill in missing data to create a complete ground plane */
        prev_i = 0;
        prev_y = 0;
        int max = (int)imgWidth / SVS_HORIZONTAL_SAMPLING;
        for (i = 0; i < max; i++) {
            x = i * SVS_HORIZONTAL_SAMPLING;
            y = footline[i];
            if (y != 0) {
                if (prev_y == 0) prev_y = y;
                for (j = prev_i; j < i; j++) {
                    footline[j] = prev_y + ((j - prev_i) * (y - prev_y) / (i - prev_i));
                }
                prev_y = y;
                prev_i = i;
            }
        }
        for (j = prev_i; j < max; j++) {
            footline[j] = prev_y;
        }
    }
#ifdef SVS_PROFILE
    printf("svs_ground_plane %d mS\n", readRTC() - t);
#endif    
}

/* shows footline */
void svs_show_footline(
unsigned char *outbuf)  /* output image.  Note that this is YUVY */
{
    int i, x, y, n;
    
    for (i = 0; i < (int)imgWidth / SVS_HORIZONTAL_SAMPLING; i++) {
        x = i * SVS_HORIZONTAL_SAMPLING;
        y = footline[i];
        if (y > 0) {
            /* draw edge */
            n = pixindex(x, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y-1);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y-2);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y+1);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x, y+2);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x-1, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x+1, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x-2, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
            n = pixindex(x+2, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;        
        }
    }
}

/* shows stereo matches as blended spots */
void svs_show_matches(
unsigned char *outbuf,  /* output image.  Note that this is YUVY */
int no_of_matches) {    /* number of stereo matches */

    int x, y, xx, yy, i, dx, dy, dist_sqr, n;
    int radius, radius_sqr;
   
    for (i = 0; i < no_of_matches*4; i += 4) {
        if (svs_matches[i] > 0) {
            x = svs_matches[i + 1];
            y = svs_matches[i + 2];
            radius = 1 + (svs_matches[i + 3] / 6);
                
            if ((x - radius >= 0) &&
                (x + radius < (int)imgWidth) &&
                (y - radius >= 0) &&
                (y + radius < (int)imgHeight)) {
            
                radius_sqr = radius*radius;

                for (yy = y - radius; yy <= y + radius; yy++) {                
                    dy = yy - y;
                    for (xx = x - radius; xx <= x + radius; xx++, n += 3) {
                        dx = xx - x;
                        dist_sqr = dx*dx + dy*dy;
                        if (dist_sqr <= radius_sqr) {
                            n = pixindex(xx, yy);
                            outbuf[n] = (unsigned char)((outbuf[n] + 84) / 2);
                            outbuf[n+1] = (unsigned char)((outbuf[n + 1] + 72) / 2);
                            outbuf[n+2] = (unsigned char)((outbuf[n + 2] + 255) / 2);
                        }
                    }
                }
            }
        }
    }
    
#ifdef SVS_SHOW_STEERING
    int steer_y = (int)imgHeight*9/10;
    int steer_width = 10;
    int steer_length = 50;
    int half_width = (int)imgWidth/2;
    int w;
    switch(svs_steer) 
    {
        case -1: { /* left */
            for (x = half_width-steer_length; x < half_width; x++) {
                w = (x - (half_width-steer_length)) * steer_width / steer_length;
                for (y = steer_y - w; y < steer_y + w; y++) {                
                    if (y < (int)imgHeight) {
                        n = pixindex(x, y);
                        outbuf[n] = (unsigned char)84;
                        outbuf[n+1] = (unsigned char)72;
                        outbuf[n+2] = (unsigned char)255;
                    }
                }
            }
            break;
        }
        case 0: { /* ahead */
            for (y = steer_y - steer_length; y < steer_y; y++) {                
                w = (y - (steer_y - steer_length)) * steer_width / steer_length;
                for (x = half_width-w; x < half_width+w; x++) {
                    n = pixindex(x, y);
                    outbuf[n] = (unsigned char)84;
                    outbuf[n+1] = (unsigned char)72;
                    outbuf[n+2] = (unsigned char)255;
                }
            }
            break;
        }
        case 1: { /* right */
            for (x = half_width; x < half_width+steer_length; x++) {
                w = steer_width - ((x - half_width) * steer_width / steer_length);
                for (y = steer_y - w; y < steer_y + w; y++) {                
                    if (y < (int)imgHeight) {
                        n = pixindex(x, y);
                        outbuf[n] = (unsigned char)84;
                        outbuf[n+1] = (unsigned char)72;
                        outbuf[n+2] = (unsigned char)255;
                    }
                }
            }
            break;
        }
    }
#endif    
}

/* computes the calibration map */
void svs_make_map(
long centre_of_distortion_x, /* centre of distortion x coordinate in pixels xSVS_MULT */
long centre_of_distortion_y, /* centre of distortion y coordinate in pixels xSVS_MULT */
long* coeff,                 /* three lens distortion polynomial coefficients xSVS_MULT_COEFF */
int degree,                  /* number of polynomial coefficients */
long scale_num,              /* scaling numerator */
long scale_denom) {          /* scaling denominator */

#ifdef SVS_VERBOSE
    printf("Computing calibration map...");
#endif

    long v, powr, radial_dist_rectified, radial_dist_original;
    long i, x, y, dx, dy;
    long n, n2, x2, y2;
    long ww = svs_width;
    long hh = svs_height;
    long half_width = ww / 2;
    long half_height = hh / 2;
    scale_denom *= SVS_MULT;
    for (x = 0; x < ww; x++) {

        dx = (x*SVS_MULT) - centre_of_distortion_x;
        
        for (y = 0; y < hh; y++) {

            dy = (y*SVS_MULT) - centre_of_distortion_y;

            v = dx*dx + dy*dy;
            /* integer square root */
            for (radial_dist_rectified=0; 
                 v >= (2*radial_dist_rectified)+1;
                 v -= (2*radial_dist_rectified++)+1);
                        
            if (radial_dist_rectified >= 0) {                

                /* for each polynomial coefficient */
                radial_dist_original = 0;
                powr = 1;
                for (i = 0; i <= degree; i++, powr *= radial_dist_rectified) {
                    radial_dist_original += coeff[i] * powr;
                }
                
		if (radial_dist_original > 0) {
		
  		    radial_dist_original /= SVS_MULT_COEFF;
		
		    x2 = centre_of_distortion_x + (dx * radial_dist_original / radial_dist_rectified);
		    x2 = (x2 - (half_width*SVS_MULT)) * scale_num / scale_denom;
                    y2 = centre_of_distortion_y + (dy * radial_dist_original / radial_dist_rectified);
                    y2 = (y2 - (half_height*SVS_MULT)) * scale_num / scale_denom;

                    x2 += half_width;
                    y2 += half_height;

                    if ((x2 > -1) && (x2 < ww) &&
                        (y2 > -1) && (y2 < hh)) {

                        if (ww != imgWidth) {
                            /* if the resolution at which the cameras were calibrated
                             * is different from the current resolution */
                            x = x*imgWidth / ww;
                            y = y*imgHeight / hh;
                            x2 = x2*imgWidth / ww;
                            y2 = y2*imgHeight / hh;
                        }
                        n = pixindex(x, y);
                        n2 = pixindex(x2, y2);

                        calibration_map[(int)n] = (int)n2;
                    }
                }
            }
        }
    }
#ifdef SVS_VERBOSE
    printf("Done\n");
#endif
}

/* send the disparity data */
void svs_send_disparities (
    int no_of_matches) {
    unsigned char ch, *cp;
    unsigned short *disp;
    unsigned int i,n;
        
    if (no_of_matches > 0) {
        int disp_data_size = 3*sizeof(unsigned short);
        int data_size = disp_data_size * no_of_matches;
        char* dispHead = (char*)DISP_BUF;
        strcpy(dispHead, "##DSP");
        
        /* write header */
        for (i=0; i<5; i++) {
            while (*pPORTHIO & 0x0001)  // hardware flow control
                continue;
            putchar(dispHead[i]);
        }
        
        sprintf(dispHead, "%d    ", no_of_matches);
        
        /* write number of matches */
        for (i=0; i<4; i++) {
            while (*pPORTHIO & 0x0001)  // hardware flow control
                continue;
            putchar(dispHead[i]);
        }
        
        /* write the data */
        disp = (unsigned short *)DISP_BUF;
        n=0;
        for (i=0;i<no_of_matches;i++) {
            disp[n++] = (unsigned short)svs_matches[i*4+1];
            disp[n++] = (unsigned short)svs_matches[i*4+2];
            disp[n++] = (unsigned short)svs_matches[i*4+3];
        }

        cp = (unsigned char *)DISP_BUF;
        for (i=0; i<data_size; i++) 
            putchar(*cp++);

        while (getchar(&ch)) {
            // flush input 
            continue;
        }
    }
}

/* main stereo update */
void svs_stereo(int send_disparities) 
{
#ifdef SVS_PROFILE
    int t[4];
#endif
    int matches = 0;
    *pPORTHIO |= 0x0100;  // set stereo sync flag high

#ifdef SVS_PROFILE
    t[0] = readRTC();
#endif
    svs_grab(svs_calibration_offset_x, svs_calibration_offset_y, master, 0);
    if (master) {       
#ifdef SVS_PROFILE
        t[1] = readRTC();
#endif
        if (svs_receive_features() > -1) {
            int ideal_no_of_matches = 200;
            int max_disparity_percent = 40;
            /* minimum no of descriptor bits to be matched, in the range 1 - SVS_DESCRIPTOR_PIXELS */
            int descriptor_match_threshold = 0;
            /* descriptor match weight */
            int learnDesc = 90;
            /* luminance match weight */
            int learnLuma = 35;
            /* disparity weight */
            int learnDisp = 1;
            /* priors*/
            int use_priors = 1;
            int learnPrior = 4;
            int groundPrior = 200;
        
#ifdef SVS_PROFILE
            t[2] = readRTC();
#endif
            matches = svs_match(
                ideal_no_of_matches, 
                max_disparity_percent, 
                descriptor_match_threshold, 
                learnDesc, 
                learnLuma, 
                learnDisp, 
                learnPrior, 
                groundPrior,
                use_priors);
                
#ifdef SVS_PROFILE
            t[3] = readRTC();
            printf("svs_grab             %d\n\r", t[1]-t[0]);
            printf("svs_receive_features %d\n\r", t[2]-t[1]);
            printf("svs_match            %d\n\r", t[3]-t[2]);
#endif
            if (send_disparities != 0) svs_send_disparities(matches);
            
            /* mapping */
            if (svs_enable_mapping != 0) {
                if (prev_svs_enable_mapping == 0) init_map();
                svs_footline_update(max_disparity_percent);
                update_pose(svs_data, svs_data_previous);
                if (robot_moving == 0) {
                    map_recenter();
                    map_update(matches, max_disparity_percent, svs_matches, footline);
                }
            }
        }

        // swap data buffers to enable visual odometry between successive frames */
        svs_data_struct *temp = svs_data;
        svs_data = svs_data_previous;
        svs_data_previous = temp;

        svs_show_matches((unsigned char *)FRAME_BUF, matches);        
        //svs_show_footline((unsigned char *)FRAME_BUF);
        if (svs_enable_mapping != 0) {
            if (prev_svs_enable_mapping == 0) init_map();
            show_map((unsigned char *)FRAME_BUF);
        }
        prev_svs_enable_mapping = svs_enable_mapping;
        
        *pPORTHIO &= 0xFEFF;  // set stereo sync flag low            
    }    
}


/* -----------------------------------------------------------------------------------------------------*/
/* mapping */
/* -----------------------------------------------------------------------------------------------------*/


/* grid map */
unsigned char* map_occupancy;

/* cartesian coordinates of observed features for each map position */
int* map_coords;

/* robot position and speed */
int robot_x_mm, robot_y_mm;
int robot_speed_mmsec, robot_orientation_degrees;

/* center position of the map */
int map_x_mm, map_y_mm;

/* distance moved from the previous position */
int dist_moved_mm, prev_dist_moved_mm;

int SinLookup[] = {
0,17,34,52,69,87,104,121,139,156,173,190,207,224,241,258,275,292,309,325,342,358,374,390,406,422,438,453,469,484,500,515,529,544,559,573,587,601,615,629,642,656,669,681,694,707,
719,731,743,754,766,777,788,798,809,819,829,838,848,857,866,874,882,891,898,906,913,920,927,933,939,945,951,956,961,965,970,974,978,981,984,987,990,992,994,996,997,998,999,999,999,
999,999,998,997,996,994,992,990,987,984,981,978,974,970,965,961,956,951,945,939,933,927,920,913,906,898,891,882,874,866,857,848,838,829,819,809,798,788,777,766,754,743,731,719,707,
694,681,669,656,642,629,615,601,587,573,559,544,529,515,500,484,469,453,438,422,406,390,374,358,342,325,309,292,275,258,241,224,207,190,173,156,139,121,104,87,69,52,34,17,0,
-17,-34,-52,-69,-87,-104,-121,-139,-156,-173,-190,-207,-224,-241,-258,-275,-292,-309,-325,-342,-358,-374,-390,-406,-422,-438,-453,-469,-484,-500,-515,-529,-544,-559,-573,-587,-601,-615,-629,-642,-656,-669,-681,-694,-707,
-719,-731,-743,-754,-766,-777,-788,-798,-809,-819,-829,-838,-848,-857,-866,-874,-882,-891,-898,-906,-913,-920,-927,-933,-939,-945,-951,-956,-961,-965,-970,-974,-978,-981,-984,-987,-990,-992,-994,-996,-997,-998,-999,-999,-999,
-999,-999,-998,-997,-996,-994,-992,-990,-987,-984,-981,-978,-974,-970,-965,-961,-956,-951,-945,-939,-933,-927,-920,-913,-906,-898,-891,-882,-874,-866,-857,-848,-838,-829,-819,-809,-798,-788,-777,-766,-754,-743,-731,-719,-707,
-694,-681,-669,-656,-642,-629,-615,-601,-587,-573,-559,-544,-529,-515,-499,-484,-469,-453,-438,-422,-406,-390,-374,-358,-342,-325,-309,-292,-275,-258,-241,-224,-207,-190,-173,-156,-139,-121,-104,-87,-69,-52,-34,-17,
};

/* initialises arrays associated with mapping */
void init_map() {
    int i;
                    
    map_occupancy = (unsigned char*)malloc(MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 2);
    map_coords = (int*)malloc(MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 2 * sizeof(int));
    memset((void*)map_occupancy, '\0', MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 2);
    for (i = (MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 2)-2; i >= 0; i-=2) map_coords[i] = EMPTY_CELL;
    robot_x_mm = 0;
    robot_y_mm = 0;
    map_x_mm = 0;
    map_y_mm = 0;
    robot_speed_mmsec = 0;
    robot_orientation_degrees = 0;
    robot_moving = 0;
    dist_moved_mm = 0;
    prev_dist_moved_mm = 0;    
}

/* estimates the robots pose */
void visual_odometry(
    int right_turn_pixels,
    int tollerance,
    svs_data_struct *svs_data,
    svs_data_struct *svs_data_previous)
{
#ifdef SVS_PROFILE
    int t = readRTC();
#endif

    int i, j, best_dx, x, expected_x, x_prev, y, no_of_feats, no_of_feats_prev, n, f, f_prev;
    int dx, dx2, min_dx, state, av_dx=0, hits=0;
    n = 0;
    f = 0;    
    f_prev = 0;
    for (y = 4; y < (int)imgHeight - 4; y += SVS_VERTICAL_SAMPLING, n++)
    {        
        no_of_feats = svs_data->features_per_row[n];
        no_of_feats_prev = svs_data_previous->features_per_row[n];
        // current edge features         
        for (i = 0; i < no_of_feats; i++) {
            x = svs_data_previous->feature_x[f + i];
            min_dx = 9999;
            best_dx = 9999;
            state = 0;
            expected_x = x - right_turn_pixels;
            // previous edge features 
            for (j = 0; j < no_of_feats_prev; j++) {
                x_prev = svs_data_previous->feature_x[f_prev + j];
                dx = x_prev - expected_x;
                if ((dx > -tollerance) && (dx < tollerance)) {
                    // within tollerance
                    state = 1;
                    if (dx >= 0)
                        dx2 = dx;
                    else
                        dx2 = -dx;
                    if (dx2 < min_dx) {
                        min_dx = dx2;
                        best_dx = x - x_prev;
                        if (min_dx < 4) break;
                    }
                }
                else {
                    // outside of tollerance 
                    if (state == 1) break;
                }
            }
            if (best_dx != 9999) {
                av_dx += best_dx;
                hits++;
            }
            
        }        
        f += no_of_feats;
        f_prev += no_of_feats_prev;
    }
    
    if (hits > 0) {
        av_dx /= hits;
    }
    else {
        av_dx = right_turn_pixels;
    }
    
    // update orientation of the robot 
    robot_orientation_degrees += av_dx * SVS_FOV_DEGREES / ((int)imgWidth*2);
    if (robot_orientation_degrees < 0) robot_orientation_degrees += 360;
    if (robot_orientation_degrees >= 360) robot_orientation_degrees -= 360;
    
#ifdef SVS_PROFILE
    printf("svs_visual_odometry %d mS\n", readRTC() - t);
#endif
}

/* re-centers the grid map */
void map_recenter()
{
#ifdef SVS_PROFILE
    int t;
#endif

    /* compute change in translation */
    int dx = robot_x_mm - map_x_mm;
    int dy = robot_y_mm - map_y_mm;
    
    if ((dx < -MAP_CELL_SIZE_MM) || (dx > MAP_CELL_SIZE_MM) ||
        (dy < -MAP_CELL_SIZE_MM) || (dy > MAP_CELL_SIZE_MM)) {

#ifdef SVS_PROFILE
        t = readRTC();
#endif

        int n, i, offset, cell_x_mm, cell_y_mm, cell_x, cell_y;
        unsigned char* buffer_occupancy = (unsigned char*)FRAME_BUF3;
        int* buffer_coords = (int*)FRAME_BUF4;

        /* clear the buffer */
        const int array_length = MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 2;
        for (i = array_length-2; i >= 0; i-=2) buffer_coords[i] = EMPTY_CELL;
        memset((void*)buffer_occupancy, '\0', array_length);

        /* update map cells */
        offset = MAP_WIDTH_CELLS/2;    
        for (i = array_length-2; i >= 0; i -= 2) {
            if (map_coords[i] != EMPTY_CELL) {
                    
                /* update cell position */
                cell_x_mm = map_coords[i];
                cell_y_mm = map_coords[i + 1];

                cell_x = offset + ((cell_x_mm - map_x_mm - dx) / MAP_CELL_SIZE_MM);
                cell_y = offset + ((cell_y_mm - map_y_mm - dy) / MAP_CELL_SIZE_MM);
                n = (cell_y * MAP_WIDTH_CELLS + cell_x) * 2;
                if ((n >= 0) && (n < array_length)) {
                            
                    if (buffer_coords[n] == EMPTY_CELL) {
                        /* new cell */
                        buffer_occupancy[n] = map_occupancy[i];
                        buffer_occupancy[n+1] = map_occupancy[i+1];
                        buffer_coords[n] = cell_x_mm;
                        buffer_coords[n+1] = cell_y_mm;
                    }
                    else {
                        /* merge cells */
                        buffer_occupancy[n] += (unsigned char)((map_occupancy[i] - buffer_occupancy[n])/2);
                        buffer_occupancy[n+1] += (unsigned char)((map_occupancy[i+1] - buffer_occupancy[n+1])/2);
                        buffer_coords[n] += (cell_x_mm - buffer_coords[n])/2;
                        buffer_coords[n+1] += (cell_y_mm - buffer_coords[n+1])/2;
                    }
                }
            }
        }
    
        /* copy buffer back to the map */
        memcpy((void*)map_coords, (void*)buffer_coords, array_length*sizeof(int));
        memcpy((void*)map_occupancy, (void*)buffer_occupancy, array_length);
    
        /* set the new map centre position */
        map_x_mm = robot_x_mm;
        map_y_mm = robot_y_mm;
        
        prev_dist_moved_mm = dist_moved_mm;
        
#ifdef SVS_PROFILE
        printf("svs_map_recenter %d mS\n", readRTC() - t);
#endif
    }
}

/* update the estimated pose of the robot */
void update_pose(
svs_data_struct *svs_data, 
svs_data_struct *svs_data_previous) 
{    
    /* estimate the change in orientation of the robot */
    if (svs_right_turn_percent != 0) {
        visual_odometry(
            svs_right_turn_percent * (int)imgWidth / 100, 
            svs_turn_tollerance_percent * (int)imgWidth / 100,
            svs_data, svs_data_previous);    
        svs_right_turn_percent = 0;
    }
    
    /* update the position/pose estimate */
    if (robot_moving != 0) {
        int pwm = (lspeed + rspeed)/2;
        if (pwm < 0) pwm = -pwm;
        const int reference_variance0 = 7;
        const int reference_speed0 = 95;
        const int reference_pwm0 = 19;
        const int reference_variance1 = 22;
        const int reference_speed1 = 265;
        const int reference_pwm1 = 31;
        int robot_speed_mmsec = 
            reference_speed0 + 
            ((pwm - reference_pwm0) * 
             (reference_speed1 - reference_speed0) / 
             (reference_pwm1 - reference_pwm0));
        int speed_variance_mmsec = 
            reference_variance0 + 
            ((pwm - reference_pwm0) * 
             (reference_variance1 - reference_variance0) / 
             (reference_pwm1 - reference_pwm0));
        int time_elapsed_mS = readRTC() - move_start_time;
        int dist_mm_min = time_elapsed_mS * (robot_speed_mmsec - speed_variance_mmsec) / 1000;
        int dist_mm_max = time_elapsed_mS * (robot_speed_mmsec + speed_variance_mmsec) / 1000;
        if (lspeed + rspeed < 0) {
            dist_mm_min = -dist_mm_min;
            dist_mm_max = -dist_mm_max;
        }
        
        dist_moved_mm = ((dist_mm_min + dist_mm_max)/2) - prev_dist_moved_mm;
        
        robot_x_mm = map_x_mm + (dist_moved_mm*SinLookup[robot_orientation_degrees]/10000);
        int CosVal = 90 - robot_orientation_degrees;
        if (CosVal < 0) CosVal += 360;
        robot_y_mm = map_y_mm + (dist_moved_mm*SinLookup[CosVal]/1000);
    }
    else {
        prev_dist_moved_mm = 0;
    }
}

/* updates a grid map */
void map_update(
int no_of_matches,
int max_disparity_percent,
unsigned int* svs_matches,
unsigned short int* footline)
{
#ifdef SVS_PROFILE
    int t = readRTC();
#endif

    int i, rot, x, y, xx, yy, disp, x_offset, y_offset;
    int x_mm, y_mm, curr_y_mm, x_rotated_mm, y_rotated_mm, disp_mmx100;
    int n, SinVal, CosVal;
    const int range_const = SVS_FOCAL_LENGTH_MMx100 * SVS_BASELINE_MM * 10;
    int half_width = (int)imgWidth/2;
    int dx = robot_x_mm - map_x_mm;
    int dy = robot_y_mm - map_y_mm;
    const int centre_cell = MAP_WIDTH_CELLS/2;
    const int update_radius = 0;
        
    int on_ground_plane;
    for (i = 0; i < no_of_matches*4; i += 4) {           
        if (svs_matches[i] > 0) {  // match probability > 0
            x = svs_matches[i + 1];
            y = svs_matches[i + 2];
            if (y < footline[x / (int)SVS_HORIZONTAL_SAMPLING])
                on_ground_plane = 0;
            else
                on_ground_plane = 1;
            
            disp = svs_matches[i + 3];
            disp_mmx100 = disp * svs_sensor_width_mmx100 / (int)imgWidth;
            if (disp_mmx100 > 0) {
                // get position of the feature in space
                int range_mm = range_const / disp_mmx100;
                
                curr_y_mm = range_mm;
                int rot_off;
                int max_rot_off = (int)SVS_FOV_DEGREES/2;
                                
                rot_off = (x - half_width) * (int)SVS_FOV_DEGREES / (int)imgWidth;
                if ((rot_off > -max_rot_off) && (rot_off < max_rot_off)) {
                    rot = robot_orientation_degrees + rot_off;

                    if (rot >= 360) rot -= 360;
                    if (rot < 0) rot += 360;

                    SinVal = SinLookup[rot];
                    CosVal = 90 - rot;
                    if (CosVal < 0) CosVal += 360;
                    CosVal = SinLookup[CosVal];                            
                    
                    // rotate by the orientation of the robot 
                    x_rotated_mm = SinVal * curr_y_mm / (int)10000;
                    y_rotated_mm = CosVal * curr_y_mm / (int)10000;
                    
                    int x_cell = (x_rotated_mm + dx) / (int)MAP_CELL_SIZE_MM;
                    int y_cell = (y_rotated_mm + dy) / (int)MAP_CELL_SIZE_MM;

                    if ((x_cell > -centre_cell+1) && (x_cell < centre_cell-1) &&
                        (y_cell > -centre_cell+1) && (y_cell < centre_cell-1)) {

                        int abs_x_cell = x_cell;
                        if (abs_x_cell < 0) abs_x_cell = -abs_x_cell;
                        int abs_y_cell = y_cell;
                        if (abs_y_cell < 0) abs_y_cell = -abs_y_cell;

                        /* vacancy */
                        int prob = 20;
                        if (abs_x_cell > abs_y_cell) {
                            for (x = 0; x <= abs_x_cell; x++) {
                                y = x * y_cell / abs_x_cell;
                                xx = x;
                                if (x_cell < 0) xx = -x;

                                x_mm = robot_x_mm + (x*x_rotated_mm/abs_x_cell);
                                y_mm = robot_y_mm + (x*y_rotated_mm/abs_x_cell);
                                
                                for (x_offset = -update_radius; x_offset <= update_radius; x_offset++) {
                                    for (y_offset = -update_radius; y_offset <= update_radius; y_offset++) {
                                        n = (((y + centre_cell + y_offset) * MAP_WIDTH_CELLS) + (xx + centre_cell + x_offset)) * 2;
                                        if (map_coords[n] == EMPTY_CELL) {
                                            map_coords[n] = x_mm + (x_offset*MAP_CELL_SIZE_MM);
                                            map_coords[n+1] = y_mm + (y_offset*MAP_CELL_SIZE_MM);
                                        }
                                        else {
                                            map_coords[n] += (x_mm + (x_offset*MAP_CELL_SIZE_MM) - map_coords[n])/2;
                                            map_coords[n+1] += (y_mm + (y_offset*MAP_CELL_SIZE_MM) - map_coords[n+1])/2;
                                        }
                                    
                                        if (map_occupancy[n] < 235) {
                                            // increment vacancy                                     
                                            map_occupancy[n] += (unsigned char)prob;
                                        }
                                        else {
                                            // decrement occupancy 
                                            if (map_occupancy[n+1] > 0) map_occupancy[n+1]--;
                                        }                                                                            
                                    }
                                }
                                                                
                                prob--;
                                if (prob < 1) prob = 1;
                            }                                    
                        }
                        else {
                            for (y = 0; y < abs_y_cell; y++) {
                                x = y * x_cell / abs_y_cell;
                                yy = y;
                                if (y_cell < 0) yy = -y;

                                x_mm = robot_x_mm + (y*x_rotated_mm/abs_y_cell);
                                y_mm = robot_y_mm + (y*y_rotated_mm/abs_y_cell);

                                for (x_offset = -update_radius; x_offset <= update_radius; x_offset++) {
                                    for (y_offset = -update_radius; y_offset <= update_radius; y_offset++) {
                                        n = (((yy + centre_cell + y_offset) * MAP_WIDTH_CELLS) + (x + centre_cell + x_offset)) * 2;
                                        if (map_coords[n] == EMPTY_CELL) {
                                            map_coords[n] = x_mm + (x_offset*MAP_CELL_SIZE_MM);
                                            map_coords[n+1] = y_mm + (y_offset*MAP_CELL_SIZE_MM);                                                                                        
                                        }
                                        else {
                                            map_coords[n] += (x_mm + (x_offset*MAP_CELL_SIZE_MM) - map_coords[n])/2;
                                            map_coords[n+1] += (y_mm + (y_offset*MAP_CELL_SIZE_MM) - map_coords[n+1])/2;
                                        }
                                        if (map_occupancy[n] < 235) {
                                            // increment vacancy                                     
                                            map_occupancy[n] += (unsigned char)prob;
                                        }
                                        else {
                                            // decrement occupancy 
                                            if (map_occupancy[n+1] > 0) map_occupancy[n+1]--;
                                        }                                        
                                    }
                                }
                                prob--;
                                if (prob < 1) prob = 1;                                
                            }                                    
                        }
                        
                        if ((on_ground_plane == 0) && 
                            (range_mm > MAP_MIN_RANGE_MM)) {
                            // occupancy
                            prob = 20;
                            int tail_length = range_mm / ((int)MAP_CELL_SIZE_MM*30);
                            if (tail_length < 2) tail_length=2;

                            if (abs_x_cell > abs_y_cell) {
                                for (x = abs_x_cell; x <= abs_x_cell+tail_length; x++) {
                                    y = x * y_cell / abs_x_cell;
                                    xx = x;
                                    if (x_cell < 0) xx = -x;

                                    x_mm = robot_x_mm + (x*x_rotated_mm/abs_x_cell);
                                    y_mm = robot_y_mm + (x*y_rotated_mm/abs_x_cell);
                                    
                                    for (x_offset = -update_radius; x_offset <= update_radius; x_offset++) {
                                        for (y_offset = -update_radius; y_offset <= update_radius; y_offset++) {
                                            n = (((y + centre_cell + y_offset) * MAP_WIDTH_CELLS) + (xx + centre_cell + x_offset)) * 2;
                                            //n = (((y + centre_cell) * MAP_WIDTH_CELLS) + (xx + centre_cell)) * 2;                                            
                                            if ((y + centre_cell >= 0) && (xx + centre_cell >= 0) &&
                                                (y + centre_cell < MAP_WIDTH_CELLS) && (xx + centre_cell < MAP_WIDTH_CELLS)) {
                                                if (map_coords[n] == EMPTY_CELL) {
                                                    map_coords[n] = x_mm + (x_offset*MAP_CELL_SIZE_MM);
                                                    map_coords[n+1] = y_mm + (y_offset*MAP_CELL_SIZE_MM);
                                                }
                                                else {
                                                    map_coords[n] += (x_mm + (x_offset*MAP_CELL_SIZE_MM) - map_coords[n])/2;
                                                    map_coords[n+1] += (y_mm + (y_offset*MAP_CELL_SIZE_MM) - map_coords[n+1])/2;
                                                }
                                                if (map_occupancy[n+1] < 235) {
                                                    // increment occupancy (very crude sensor model)
                                                    map_occupancy[n+1] += (unsigned char)prob;
                                                }
                                                else {
                                                    // decrement vacancy 
                                                    if (map_occupancy[n] > 0) 
                                                        map_occupancy[n]--;
                                                }                                                
                                            }
                                        }
                                    }
                                    
                                    prob--;
                                    if (prob < 1) prob = 1;                                                                        
                                }
                            }
                            else {
                                for (y = abs_y_cell; y <= abs_y_cell+tail_length; y++) {
                                    x = y * x_cell / abs_y_cell;
                                    yy = y;
                                    if (y_cell < 0) yy = -y;

                                    x_mm = robot_x_mm + (y*x_rotated_mm/abs_y_cell);
                                    y_mm = robot_y_mm + (y*y_rotated_mm/abs_y_cell);
                                    
                                    for (x_offset = -update_radius; x_offset <= update_radius; x_offset++) {
                                        for (y_offset = -update_radius; y_offset <= update_radius; y_offset++) {
                                            n = (((yy + centre_cell + y_offset) * MAP_WIDTH_CELLS) + (x + centre_cell + x_offset)) * 2;
                                            //n = (((yy + centre_cell) * MAP_WIDTH_CELLS) + (x + centre_cell)) * 2;
                                            if ((yy + centre_cell >= 0) && (x + centre_cell >= 0) &&
                                                (yy + centre_cell < MAP_WIDTH_CELLS) && (x + centre_cell < MAP_WIDTH_CELLS)) {
                                                if (map_coords[n] == EMPTY_CELL) {
                                                    map_coords[n] = x_mm + (x_offset*MAP_CELL_SIZE_MM);
                                                    map_coords[n+1] = y_mm + (y_offset*MAP_CELL_SIZE_MM);
                                                }
                                                else {
                                                    map_coords[n] += (x_mm + (x_offset*MAP_CELL_SIZE_MM) - map_coords[n])/2;
                                                    map_coords[n+1] += (y_mm + (y_offset*MAP_CELL_SIZE_MM) - map_coords[n+1])/2;
                                                }
                                                if (map_occupancy[n+1] < 235) {
                                                    // increment occupancy (very crude sensor model)
                                                    map_occupancy[n+1] += (unsigned char)prob;
                                                }
                                                else {
                                                    // decrement vacancy 
                                                    if (map_occupancy[n] > 0) 
                                                        map_occupancy[n]--;
                                                }                                                
                                            }

                                    
                                        }
                                    }

                                    prob--;
                                    if (prob < 1) prob = 1;
                                }
                            }

                        }
                    }
                    
                }
            }            
        }
    }
        
#ifdef SVS_PROFILE
    printf("svs_map_update %d mS\n", readRTC() - t);
#endif    
}

/* shows a map oriented with the robot */
void show_map(
unsigned char* outbuf)
{
    int x,y,xx,yy,n,n2;
    const int divisor = 3;
    int ww = (int)imgWidth/divisor;
    int hh = (int)imgHeight/divisor;
            
    for (y = 0; y < hh; y++) {
        yy = MAP_WIDTH_CELLS - 1 - (y * MAP_WIDTH_CELLS / hh);
        for (x = 0; x < ww; x++) {
            xx = x * MAP_WIDTH_CELLS / ww;
            n = pixindex(x, y);
            n2 = ((yy * MAP_WIDTH_CELLS) + xx) * 2;
                    
            if (map_coords[n2] != EMPTY_CELL) {
                if (map_occupancy[n2] >= map_occupancy[n2+1]) {
                    /* vacant */
                    outbuf[n] = 255;
                    outbuf[n+1] = 44;
                    outbuf[n+2] = 84;
                    outbuf[n+3] = 21;
                }
                else {
                    /* occupied */
                    outbuf[n] = 84;
                    outbuf[n+1] = 72;
                    outbuf[n+2] = 255;
                    outbuf[n+3] = 0;
                }
            }
            else {
                /* black */                    
                outbuf[n] = 128;
                outbuf[n+1] = 0;
                outbuf[n+2] = 128;
                outbuf[n+3] = 0;
            }
        }
    }            
    
}


