#include "gps.h"
#include "print.h"
#include "malloc.h"
#include "string.h"
#include "srv.h"
#include "uart.h"

/* typedef struct gps_data {
    int latdeg;
    int latmin;
    int londeg;
    int lonmin;
    int alt;
    int fix;
    int sat;
    int utc;
}; 

$GPGGA,173415.400,3514.5974,N,12037.2028,W,1,8,1.18,84.6,M,-30.8,M,,*50
$GPGAA,hhmmss.sss,ddmm.mmmm,n,dddmm.mmmm,e,q,ss,y.y,a.a,z,g.g,z,t.t,iii*CC
    where:
        GPGAA        : GPS fixed data identifier
        hhmmss.ss    : Coordinated Universal Time (UTC), also known as GMT
        ddmm.mmmm,n  : Latitude in degrees, minutes and cardinal sign
        dddmm.mmmm,e : Longitude in degrees, minutes and cardinal sign
        q            : Quality of fix.  1 = there is a fix
        ss           : Number of satellites used
        y.y          : Horizontal precision
        a.a,M        : GPS antenna altitude in meters
        g.g,M        : geoidal separation in meters
        t.t          : age of the differential correction data
        iiii         : Differential station's ID
        *CC          : Checksum 
*/

extern unsigned int uart1_flag;
struct gps_data gps_gga;

void gps_show() {
    printf("##gps\r\n");
    if (!gps_parse())
        printf("no response from gps\n\r");
    printf("lat deg: %d\n\r", gps_gga.latdeg);
    printf("lat min: %d\n\r", gps_gga.latmin);
    printf("lon deg: %d\n\r", gps_gga.londeg);
    printf("lon min: %d\n\r", gps_gga.lonmin);
    printf("gps alt: %d\n\r", gps_gga.alt);
    printf("gps fix: %d\n\r", gps_gga.fix);
    printf("gps sat: %d\n\r", gps_gga.sat);
    printf("gps utc: %d\n\r", gps_gga.utc);
}

int gps_parse() {
    unsigned char buf[100];
    unsigned char ch;
    unsigned int t0, sum, pow10;
    int i1, i2, ilast, ix;
    
    i1 = i2 = ilast = 0;  // to get rid of compiler warnings
    
    if (!uart1_flag) {
        init_uart1();
        uart1_flag = 1;
    }
    t0 = readRTC();  // set up for 1-second timeout
    
    while (1) {
        if ((readRTC() - t0) > 1000) { // check for timeout
            gps_gga.fix = 0;
            gps_gga.sat = 0;
            return 0;
        }
        if (!uart1GetChar(&ch)) 
            continue;
        if (ch != '$')   // look for "$GPGGA," header
            continue;
        for (i1=0; i1<6; i1++)     
            buf[i1] = uart1GetCh();
        if ((buf[2] != 'G') || (buf[3] != 'G') || (buf[4] != 'A'))
            continue;
        for (i1=0; i1<100; i1++) {
            buf[i1] = uart1GetCh();  // read 100 chars into data buffer
            if (buf[i1] == '\r') {
                buf[i1] = 0;
                ilast = i1;
                break;
            }
        }
        printf("$GPGGA,%s\n\r", buf);

        // i1 = start of search, i2 = end of search (comma), ilast = end of buffer

        // parse utc
        i1 = 0;
        sum = 0;
        pow10 = 1;
        for (ix=0; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.')
                continue;
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
        }
        gps_gga.utc = sum / 1000;
        
        // parse lat
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.')
                continue;
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
        }
        gps_gga.latdeg = sum / 1000000;
        gps_gga.latmin = sum - (gps_gga.latdeg * 1000000);
        
        // skip N/S field
        i1 = i2+1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }

        // parse lon
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.')
                continue;
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
        }
        gps_gga.londeg = sum / 1000000;
        gps_gga.lonmin = sum - (gps_gga.londeg * 1000000);
        
        // skip E/W field
        i1 = i2+1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }

        // parse fix
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        gps_gga.fix = buf[i2-1] & 0x0F;
        
        // parse satellites
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
        }
        gps_gga.sat = sum;
        
        // skip horz-precision field
        i1 = i2+1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }

        // parse alt
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.')
                continue;
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
        }
        gps_gga.alt = sum / 10;
        
        return 1;
    }
}

