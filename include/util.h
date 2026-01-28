#ifndef _util_h_
#define _util_h_ 1

#include <stdbool.h>


int a2d(char ch);
char a2ui(char ch, char **src, unsigned int base, unsigned int *nump);
void ui2a(unsigned int num, unsigned int base, char *bf);
void i2a(int num, char *bf);

// toggle_caches: Toggles the data/instruction caches according to the build arguments
void toggle_caches(bool dcache_on, bool icache_on);


#if !defined(MMU)
#include <stddef.h>

// define our own memset to avoid SIMD instructions emitted from the compiler
void* memset(void *s, int c, size_t n);

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void* memcpy(void* dest, const void* src, size_t n);
#endif


#endif /* util.h */
