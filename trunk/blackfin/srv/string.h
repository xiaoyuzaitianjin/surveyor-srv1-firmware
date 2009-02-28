extern int  strcmp(char *, char *);
int strncmp(char *, char *, int);
extern char *strchr(char *, char);
extern void strcpy(char *, char *);
char *strncpy(char *, char *, int);
extern char *strdup(char *);
extern int strlen(char *);
extern int isdigit(char);
extern int  atoi(char *);
extern void itoa(int, char *);
extern void reverse(char *);
extern void memcpy(unsigned char *, unsigned char *, int);
extern void memset(unsigned char*, unsigned char, int);
#define xmemset memset
#define xmemcpy memcpy

