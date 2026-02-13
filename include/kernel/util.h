#ifndef _util_h_
#define _util_h_ 1

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


int a2d(char ch);
char a2ui(char ch, char **src, unsigned int base, unsigned int *nump);
void ui2a(unsigned int num, unsigned int base, char *bf);
void i2a(int num, char *bf);

// String to integer conversion
int str2int(const char *str);

// padded_ui2a: Convers a number to a string with some padding prefix
void padded_ui2a(unsigned int num, unsigned int base, char *buf, unsigned int length, char pad_char);

// max_uint(a, b): Calculates the max of 2 integers
uint32_t max_uint(uint32_t a, uint32_t b);

// min_uint(a, b): Calulcates the min of 2 integers
uint32_t min_uint(uint32_t a, uint32_t b);

// Clock rendering
void clock_render(uint64_t elapsed_us, char *buf);

// toggle_caches: Toggles the data/instruction caches according to the build arguments
void toggle_caches(bool dcache_on, bool icache_on);

// swap: Swaps the values from 2 variables
void swap(void *a, void *b, size_t size);

// max_uint(a, b): Calculates the max of 2 integers
uint32_t max_uint(uint32_t a, uint32_t b);

// min_uint(a, b): Calulcates the min of 2 integers
uint32_t min_uint(uint32_t a, uint32_t b);

#if !defined(MMU)

// define our own memset to avoid SIMD instructions emitted from the compiler
void* memset(void *s, int c, size_t n);

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void* memcpy(void* dest, const void* src, size_t n);
#endif


#endif /* util.h */
