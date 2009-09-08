extern int  strcmp(char *, char *);
extern int strncmp(char *, char *, int);
extern char *strchr(char *, char);
extern void strcpy(char *, char *);
extern char *strncpy(char *, char *, int);
extern char *strdup(char *);
extern int strlen(char *);
extern int isdigit(char);
extern int  atoi(char *);
extern void itoa(int, char *);
extern void reverse(char *);
extern void memcpy(unsigned char *, unsigned char *, int);
extern void memset(unsigned char*, unsigned char, int);
extern unsigned int ctoi(unsigned char);
#define xmemset memset
#define xmemcpy memcpy
extern unsigned int atoi_b16(char *s);
extern char *strpbrk(const char *str, const char *set);
extern char *strtok(char *s, const char *delim);

