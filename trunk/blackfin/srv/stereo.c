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
    unsigned short ix, *bufsave;

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

/* a brief history of histogram peaks, used for filtering */
unsigned short* peaks_history;
int peaks_history_index;
int enable_peaks_filter;

/* array used to store a disparity histogram */
int *disparity_histogram;

/* maps raw image pixels to rectified pixels */
int *calibration_map;

/* priors used when processing a video stream */
int* disparity_priors;

/* array storing the number of features detected on each column */
unsigned short int* features_per_col;

/* array storing y coordinates of detected features */
short int* feature_y;
    
extern unsigned int imgWidth, imgHeight;
extern int svs_calibration_offset_x, svs_calibration_offset_y;
extern int svs_centre_of_disortion_x, svs_centre_of_disortion_y;
extern int svs_scale_num, svs_scale_denom;
extern int svs_coeff_degree;
extern long* svs_coeff;
extern int svs_width, svs_height;

/* previous image width.  This is used to detect resolution changes */
int svs_prev_width;

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
int svs_update_sums(int cols, /* if non-zero we're dealing with columns not rows */
int i, /* row or column index */
unsigned char* rectified_frame_buf) { /* image data */

	int j, x, y, idx, max, sum = 0, mean = 0;

	if (cols == 0) {
		/* compute sums along the row */
		y = i;
		idx = imgWidth * y;
		max = (int) imgWidth;

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
		for (y = 1; y < max; y++, idx+=imgWidth) {
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
		/* edge using 2 pixel radius */
		p0 = (sum - row_sum[j - 2]) - (row_sum[j + 2] - sum);
		if (p0 < 0) p0 = -p0;

		/* edge using 4 pixel radius */
		p1 = (sum - row_sum[j - 4]) - (row_sum[j + 4] - sum);
		if (p1 < 0) p1 = -p1;

		/* overall edge response */
		row_peaks[j] = p0 + p1;
	}

	return (mean);
}


/* performs non-maximal suppression on the given row or column */
void svs_non_max(int cols, /* if non-zero we're dealing with columns not rows */
int inhibition_radius, /* radius for non-maximal suppression */
unsigned int min_response) { /* minimum threshold as a percent in the range 0-200 */

	int i, r, max, max2;
	unsigned int v;

	/* average response */
	unsigned int av_peaks = 0;
	max = (int) imgWidth;
	if (cols != 0)
		max = (int) imgHeight;
	for (i = 4; i < max - 4; i++) {
		av_peaks += row_peaks[i];
	}
	av_peaks /= (max - 8);

	/* adjust the threshold */
	av_peaks = av_peaks * min_response / 100;

        max2 = max - inhibition_radius;
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
    int calibration_offset_y)            /* calibration y offset in pixels */
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

            row_mean = svs_update_sums(0, y, rectified_frame_buf);
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
int calibration_offset_x, int calibration_offset_y) {

    unsigned short int no_of_feats;
    int x, y, row_mean, start_y;
    int no_of_features = 0;
    int col_idx = 0;

    /* create arrays */
    if (features_per_col == 0) {
        features_per_col = (unsigned short int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING*sizeof(unsigned short int));
        feature_y = (short int*)malloc(SVS_MAX_FEATURES*sizeof(short int));
    }

    memset((void*)features_per_col, '\0', SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING
        * sizeof(unsigned short));

    start_y = imgHeight - 15;
    if ((int) imgHeight - inhibition_radius - 1 < start_y)
        start_y = (int) imgHeight - inhibition_radius - 1;

    for (x = 4 + calibration_offset_x; x < (int) imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING) {

        /* reset number of features on the row */
        no_of_feats = 0;

        if ((x >= 4) && (x <= (int) imgWidth - 4)) {

	    row_mean = svs_update_sums(1, x, rectified_frame_buf);
  	    svs_non_max(1, inhibition_radius, minimum_response);

	    /* store the features */
	    for (y = start_y; y > 15; y--) {
	        if (row_peaks[y] > 0) {

		    if (svs_compute_descriptor(x, y, rectified_frame_buf, no_of_features, row_mean) == 0) {
		        feature_y[no_of_features++] = (short int) (y + calibration_offset_y);
		        no_of_feats++;
		        if (no_of_features == SVS_MAX_FEATURES) {
			    x = imgWidth;
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
    svs_centre_of_disortion_y = imgHeight / 2;
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
    no_of_feats = svs_get_features_vertical((unsigned char *)FRAME_BUF3, inhibition_radius, minimum_response, calibration_offset_x, calibration_offset_y);

#ifdef SVS_PROFILE
    t[5] = readRTC();    
    t[6] = 0;
#endif
    
    if ((left_camera != 0) && (svs_enable_horizontal)) {
        svs_get_features_horizontal((unsigned char *)FRAME_BUF3, inhibition_radius, minimum_response,     calibration_offset_x, calibration_offset_y);        
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
    int use_priors,                   /* if non-zero then use priors, assuming time between frames is small */ 
    int show_matches)                 /* if non-zero show matches */  
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
    int priors_length = imgWidth * imgHeight / (16*SVS_VERTICAL_SAMPLING);
    
    if (no_of_possible_matches > 20)
    {    
        memset((void*)disparity_priors, '\0', priors_length*sizeof(int));
        
        /* filter the results */
        svs_filter(no_of_possible_matches, max_disp, 3, use_priors);

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
				idx = (((row + row_offset) * imgWidth + xL) / 16) + col_offset;
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
		for (x = 4; x < (int) imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING, col++) {

			no_of_feats = features_per_col[col];

			// features along the row in the left camera
			for (L = 0; L < no_of_feats; L++) {

				if (valid_quadrants[fL + L] == 0) {
					// y coordinate of the feature in the left camera 
					y = feature_y[fL + L];

					// lookup disparity from priors

					row = y / SVS_VERTICAL_SAMPLING;
					disp_prior = disparity_priors[(row * imgWidth + x) / 16];

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
								idx = (((row + row_offset) * imgWidth + x) / 16) + col_offset;
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

    if (show_matches != 0) {
        /* display the matches */   
        svs_show_matches((unsigned char *)FRAME_BUF, matches);
    }    
        
    return(matches);
}


/* filtering function removes noise by searching for a peak in the disparity histogram */
void svs_filter(
    int no_of_possible_matches, /* the number of stereo matches */
    int max_disparity_pixels,   /* maximum disparity in pixels */
    int tolerance,              /* tolerance around the peak in pixels of disparity */
    int enable_secondary)       /* enable secondary filtering (over time) */
{

    int i, hf;
    unsigned int x, y, tx=0, ty=0, bx=0, by=0;
    int min, max, maxval, diff, p, disp3, disp2_hits, min_disp, max_disp;
    svs_steer = 0;

    /* clear quadrants */
    memset((void*)valid_quadrants, '\0', no_of_possible_matches * sizeof(unsigned char));

    /* create disparity histograms within different
     * zones of the image */
    for (hf = 0; hf < 4; hf++)
    {

        switch(hf)
        {
            /* left hemifield */
        case 0:
            {
                tx = 0;
                ty = 0;
                bx = imgWidth/2;
                by = imgHeight;
                break;
            }
            /* right hemifield */
        case 1:
            {
                tx = bx;
                bx = imgWidth;
                break;
            }
            /* upper hemifield */
        case 2:
            {
                tx = 0;
                ty = 0;
                bx = imgWidth;
                by = imgHeight/2;
                break;
            }
            /* lower hemifield */
        case 3:
            {
                ty = by;
                by = imgHeight;
                break;
            }
        }

        /* clear the histogram */
        memset((void *)disparity_histogram, '\0', max_disparity_pixels * sizeof(int));
        int hist_max = 0;

        /* update the disparity histogram */
        for (i = 0; i < no_of_possible_matches; i++)
        {
            x = svs_matches[i*4 + 1];
            if ((x > tx) && (x < bx))
            {
                y = svs_matches[i*4 + 2];
                if ((y > ty) && (y < by))
                {
                    int disp = svs_matches[i*4 + 3];
                    disparity_histogram[disp]++;
                    if (disparity_histogram[disp] > hist_max)
                        hist_max = disparity_histogram[disp];
                }
            }
        }

        /* locate the histogram peak */
        int mass = 0;
        unsigned int disp;
        int disp2 = 0;
        int hist_thresh = hist_max/4;
        int hist_mean = 0;
        int hist_mean_hits = 0;
        int d;
        for (d = 3; d < max_disparity_pixels-1; d++)
        {
            if (disparity_histogram[d] > hist_thresh)
            {
                int m = disparity_histogram[d] + disparity_histogram[d-1] + disparity_histogram[d+1];
                mass += m;
                disp2 += m * d;
            }
            if (disparity_histogram[d] > 0)
            {
                hist_mean += disparity_histogram[d];
                hist_mean_hits++;
            }
        }
        if (mass > 0)
        {
            disp2 /= mass;
            hist_mean /= hist_mean_hits;
        }

        /* simple near/far classification adjusts
         * the peak disparity that we're interested in */
        int near = 1;
        if (hist_mean*4 > disparity_histogram[0])
        {
            near = 0;
        }
                
        if (enable_secondary) {
        
	    // store history 
	    peaks_history[hf * SVS_PEAKS_HISTORY + peaks_history_index] = (unsigned short) disp2;

 	    // secondary filtering over time
	    // this is only useful if the frame rate is reasonably high
	    if (enable_peaks_filter != 0) {
	        // clear the histogram
		memset((void*)disparity_histogram, '\0', max_disparity_pixels * sizeof(int));
		min = max_disparity_pixels;
		max = 0;
		// create a histogram of peak disparities over the last few frames
		for (p = 0; p < SVS_PEAKS_HISTORY; p++) {
		    disp2 = peaks_history[hf * SVS_PEAKS_HISTORY + p] / 5;
		    disparity_histogram[disp2]++;
		    if (disp2 < min) min = disp2;
		    if (disp2 > max) max = disp2;
	        }
		// find the peak
		maxval = -1;
		disp3 = 0;
		for (p = max; p >= min; p--) {
		    if (disparity_histogram[p] > maxval) {
		        maxval = disparity_histogram[p];
			disp3 = p;
		    }
		}
		// average the disparity values
		disp2 = 0;
		disp2_hits = 0;
		for (p = 0; p < SVS_PEAKS_HISTORY; p++) {
		    if (peaks_history[hf * SVS_PEAKS_HISTORY + p] / 5 == disp3) {
			disp2 += peaks_history[hf * SVS_PEAKS_HISTORY + p];
			disp2_hits++;
		    }
		}
		if (disp2_hits > 0) disp2 /= disp2_hits;
	    }
	}	
	
	/* store peak disparity values for the left and right hemifields */
	if (hf == 0) svs_disp_left = disp2;
	if (hf == 1)
	{
	    svs_disp_right = disp2;
	    
	    /* update steering signal for obstacle avoidance */
	    diff = svs_disp_right - svs_disp_left;	    
	    if (diff > SVS_STEER_DEADBAND) {
	        svs_steer = -1;
	    }
	    if (diff < -SVS_STEER_DEADBAND) {
  	        svs_steer = 1;
	    }
	}
        
        /* remove matches too far away from the peak by setting
         * their probabilities to zero */
        min_disp = disp2 - tolerance;
        max_disp = disp2 + tolerance;
        for (i = 0; i < no_of_possible_matches; i++)
        {
            x = svs_matches[i*4 + 1];
            if ((x > tx) && (x < bx))
            {
                y = svs_matches[i*4 + 2];
                if ((y > ty) && (y < by))
                {
                    disp = svs_matches[i*4 + 3];
                    if (near == 1)
                    {
                        if (!((disp < min_disp) || (disp > max_disp)))
                        {
                            /* near - within stereo ranging resolution */
                            valid_quadrants[i]++;
                        }
                    }
                    else
                    {
                        if (disp <= 2)
                        {
                            /* far out man */
                            valid_quadrants[i]++;
                        }
                    }
                }
            }
        }
    }

    for (i = 0; i < no_of_possible_matches; i++)
    {
        if (valid_quadrants[i] == 0)
        {
            /* set probability to zero */
            svs_matches[i*4] = 0;
        }
    }

    if (enable_secondary) {
	/* increment history counter */
	peaks_history_index++;
	if (peaks_history_index >= SVS_PEAKS_HISTORY) {
	    peaks_history_index = 0;
	    enable_peaks_filter = 1;
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

        max = (imgWidth * imgHeight * 2)-2;
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
    svs_matches = (unsigned int *)malloc(SVS_MAX_MATCHES*16);
    disparity_histogram = (int*)malloc(SVS_MAX_IMAGE_WIDTH*2); /* more than half max width is overkill*/
    valid_quadrants = (unsigned char*)malloc(SVS_MAX_MATCHES);
    disparity_priors = (int*)malloc((SVS_MAX_IMAGE_WIDTH*SVS_MAX_IMAGE_HEIGHT/(16*SVS_VERTICAL_SAMPLING))*4);    
    peaks_history = 0;
    peaks_history_index = 0;
    enable_peaks_filter = 0;
    peaks_history = (unsigned short*)malloc(4*SVS_PEAKS_HISTORY*sizeof(unsigned short));
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
            (y < imgHeight-1)) {

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

/* shows stereo matches as blended spots */
void svs_show_matches(
unsigned char *outbuf,  /* output image.  Note that this is YUVY */
int no_of_matches) {    /* number of stereo matches */

    int x, y, xx, yy, i, dx, dy, dist_sqr, n;
    int radius, radius_sqr;
   
    for (i = 0; i < no_of_matches*4; i += 4) {
   
        x = svs_matches[i + 1];
        y = svs_matches[i + 2];
        radius = 1 + (svs_matches[i + 3] / 6);
                
        if ((x - radius >= 0) &&
            (x + radius < imgWidth) &&
            (y - radius >= 0) &&
            (y + radius < imgHeight)) {
            
            radius_sqr = radius*radius;

            for (yy = y - radius; yy <= y + radius; yy++) {                
                dy = yy - y;
                for (xx = x - radius; xx <= x + radius; xx++, n += 3) {
                    dx = xx - x;
                    dist_sqr = dx*dx + dy*dy;
                    if (dist_sqr <= radius_sqr) {
                        n = pixindex(xx, yy);
                        outbuf[n] = (unsigned char)((outbuf[n] + 145) / 2);
                        outbuf[n+1] = (unsigned char)((outbuf[n + 1] + 54) / 2);
                        outbuf[n+2] = (unsigned char)((outbuf[n + 2] + 34) / 2);
                    }
                }
            }
        }
    }
    
#ifdef SVS_SHOW_STEERING
    int steer_y = imgHeight*9/10;
    int steer_width = 10;
    int steer_length = 50;
    int half_width = imgWidth/2;
    int w;
    switch(svs_steer) 
    {
        case -1: { /* left */
            for (x = half_width-steer_length; x < half_width; x++) {
                w = (x - (half_width-steer_length)) * steer_width / steer_length;
                for (y = steer_y - w; y < steer_y + w; y++) {                
                    if (y < imgHeight) {
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
                    if (y < imgHeight) {
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
                use_priors, 1);
#ifdef SVS_PROFILE
            t[3] = readRTC();
            printf("svs_grab             %d\n\r", t[1]-t[0]);
            printf("svs_receive_features %d\n\r", t[2]-t[1]);
            printf("svs_match            %d\n\r", t[3]-t[2]);
#endif
            if (send_disparities != 0) svs_send_disparities(matches);
        }
        *pPORTHIO &= 0xFEFF;  // set stereo sync flag low            
    }
}



