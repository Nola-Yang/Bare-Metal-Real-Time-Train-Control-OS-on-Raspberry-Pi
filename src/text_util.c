#include "text_util.h"


int strcmp(const char *s1, const char *s2) {
    while (*s1 && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

void strncpy(char *dest, const char *src, int n) {
    int i = 0;
    while (i < n - 1 && src[i]) {
        dest[i] = src[i];
        i++;
    }
    dest[i] = '\0';
}