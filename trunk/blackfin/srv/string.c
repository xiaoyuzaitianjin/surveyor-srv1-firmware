/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  string.c - string library for the SRV-1 Blackfin robot.
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
#include "malloc.h"

int strcmp(char *s1, char *s2)
{
  while (*s1 == *s2++)
    if (*s1++ == 0)
        return (0);
  return (*s1 - *--s2);
}

int strncmp(char *s1, char *s2, int n)
{
  unsigned char u1, u2;
  while (n-- > 0)
    {
      u1 = (unsigned char) *s1++;
      u2 = (unsigned char) *s2++;
      if (u1 != u2)
    return u1 - u2;
      if (u1 == '\0')
    return 0;
  }
  return 0;
}

char *strchr(char *s, char c)
{
        for (;;)
        {
             if (*s == c) 
                    return s;
             if (!*s++) 
                    return 0;
        }
}

void strcpy(char *pDst, char *pSrc)
{
        while ((*pDst++ = *pSrc++) != '\0')
                continue;
}

char *strncpy(char *dst, const char *src, int n)
{
  if (n != 0) {
    char *d = dst;
    char *s = src;
    do {
      if ((*d++ = *s++) == 0) {
        /* NUL pad the remaining n-1 bytes */
        while (--n != 0)
          *d++ = 0;
        break;
      }
    } while (--n != 0);
  }
  return (dst);
}

int atoi(char *p)
{
    int n, f;

    n = 0;
    f = 0;
    for(;;p++) {
        switch(*p) {
        case ' ':
        case '\t':
            continue;
        case '-':
            f++;
        case '+':
            p++;
        }
        break;
    }
    while(*p >= '0' && *p <= '9')
        n = n*10 + *p++ - '0';
    return(f? -n: n);
}

int strlen(char *pStr)
{
    char *pEnd;

    for (pEnd = pStr; *pEnd != 0; pEnd++)
        continue;

    return pEnd - pStr;
}

char *strdup(char *s)
{
    char *result = (char*)malloc(strlen(s) + 1);
    if (result == (char*)0)
        return (char*)0;
    strcpy(result, s);
        return result;
}

#define SWAP_CHAR( x, y ) {char c; c = x; x = y; y = c;}

void reverse(char *t)
{
  int i,j;
  for(i = 0, j = strlen(t)-1; i < j; i++, j--)
    SWAP_CHAR(t[i], t[j]);
}

void itoa( int n, char *s )
{
    int i, sign;
    if ( ( sign = n ) < 0 )
        n = -n;
    i = 0;
    do {
        s[ i++ ] = (unsigned int)n % 10 + '0';
    } while (( n /= 10 ) > 0 );
    if ( sign < 0 )
        s[ i++ ] = '-';
    s[ i ] = '\0';
    reverse( s );
}

int isdigit(c)
 char c;
{
  if ((c>='0') && (c<='9'))
    return 1;
  else
    return 0;  
}

void memcpy (char *dst, char *src, int count)
{
    while (count--)
        *dst++ = *src++;
}

void memset (char *dst, char ch, int count)
{
    while (count--)
        *dst++ = ch;
}

unsigned int ctoi(unsigned char c) {
    if (c > '9')
        return (unsigned int)(c & 0x0F) + 9;
    else
        return (unsigned int)(c & 0x0F);
}

unsigned int atoi_b16(char *s) {
	// Convert two hex characters to a int8
	unsigned int result = 0;
	int i;
	for(i = 0; i < 2; i++,s++)  {
		if (*s >= 'A')
			result = 16 * result + (*s) - 'A' + 10;
		else
			result = 16 * result + (*s) - '0';
	}
	return result;
}

char *strpbrk(const char *str, const char *set) {
	while (*str != '\0')
		if (strchr(set, *str) == 0)
			++str;
		else
			return (char *) str;

	return 0;
}

int strspn(const char *s, const char *accept) {
	const char *p;
	const char *a;
	int count = 0;
	for (p = s; *p != '\0'; ++p) {
		for (a = accept; *a != '\0'; ++a)
			if (*p == *a)
				break;
		if (*a == '\0')
			return count;
		else
			++count;
	}
	return count;
}

char *strtok_r(char *s, const char *delim, char **save_ptr) {
	char *token;
	token = 0;					/* Initialize to no token. */
	if (s == 0) {				/* If not first time called... */
		s = *save_ptr;		/* restart from where we left off. */
	}
	if (s != 0) {				/* If not finished... */
		*save_ptr = 0;
		s += strspn(s, delim);	/* Skip past any leading delimiters. */
		if (*s != '\0') {		/* We have a token. */
			token = s;
			*save_ptr = strpbrk(token, delim); /* Find token's end. */
			if (*save_ptr != 0) {
				/* Terminate the token and make SAVE_PTR point past it.  */
				*(*save_ptr)++ = '\0';
			}
		}
	}
	return token;
}

