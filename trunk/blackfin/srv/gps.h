void gps_show();
int gps_parse();

typedef struct gps_data {
    int latdeg;
    int latmin;
    int londeg;
    int lonmin;
    int alt;
    int fix;
    int sat;
    int utc;
} gps_data;

extern struct gps_data gps_gga;

