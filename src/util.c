#include "util.h"
#include <stdint.h>


// ascii digit to integer
int a2d(char ch) {
	if (ch >= '0' && ch <= '9') return ch - '0';
	if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
	if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
	return -1;
}

// ascii string to unsigned int, with base
char a2ui(char ch, char **src, unsigned int base, unsigned int *nump) {
	unsigned int num;
	int digit;
	char *p;

	p = *src; num = 0;
	while ((digit = a2d(ch)) >= 0) {
		if ((unsigned int)digit > base) break;
		num = num * base + digit;
		ch = *p++;
	}
	*src = p; *nump = num;
	return ch;
}

// unsigned int to ascii string, with base
void ui2a(unsigned int num, unsigned int base, char *buf) {
	unsigned int n = 0;
	unsigned int d = 1;

	while ((num / d) >= base) d *= base;
	while (d != 0) {
		unsigned int dgt = num / d;
		num %= d;
		d /= base;
		if (n || dgt > 0 || d == 0) {
			*buf++ = dgt + (dgt < 10 ? '0' : 'A' - 10);
			++n;
		}
	}
	*buf = 0;
}

// signed int to ascii string
void i2a(int num, char *buf) {
	if (num < 0) {
		num = -num;
		*buf++ = '-';
	}
	ui2a(num, 10, buf);
}

void toggle_caches() {
	uint64_t sctlr_el1;

	__asm__ volatile("mrs %0, sctlr_el1" : "=r" (sctlr_el1));

	#ifdef DCACHE
	sctlr_el1 |= (1 << 2);
	#else
	sctlr_el1 &= ~(1 << 2);
	#endif

	#ifdef ICACHE
	sctlr_el1 |= (1 << 12);
	#else
	sctlr_el1 |= ~(1 << 12);
	#endif

	// Write back and synchronize to barriers
    asm volatile("msr sctlr_el1, %0" : : "r" (sctlr_el1));
    asm volatile("isb" : : : "memory");
    asm volatile("dsb sy" : : : "memory");
}


#if !defined(MMU)
#include <stddef.h>

// define our own memset to avoid SIMD instructions emitted from the compiler
void* memset(void *s, int c, size_t n) {
	for (char* it = (char*)s; n > 0; --n) *it++ = c;
	return s;
}

// define our own memcpy to avoid SIMD instructions emitted from the compiler
void* memcpy(void* dest, const void* src, size_t n) {
	char* sit = (char*)src;
	char* cdest = (char*)dest;
	for (size_t i = 0; i < n; ++i) *cdest++ = *sit++;
	return dest;
}
#endif
