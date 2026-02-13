#include "util.h"
#include <stdint.h>
#include <stdbool.h>


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

void padded_ui2a(unsigned int num, unsigned int base, char *buf, unsigned int length, char pad_char) {
	unsigned int divide_result = num;
	unsigned int num_of_digits = 0;

	while (divide_result) {
		divide_result /= base;
		num_of_digits++;
	}

	unsigned int zero_pad_count = length - num_of_digits;
	while (zero_pad_count > 0) {
		*buf = pad_char;
		buf++;
		zero_pad_count--;
	}

	ui2a(num, base, buf);
}

void toggle_caches(bool dcache_on, bool icache_on) {
	uint64_t sctlr_el1;

	__asm__ volatile("mrs %0, sctlr_el1" : "=r" (sctlr_el1));

	if (dcache_on) {
		sctlr_el1 |= (1 << 2);
	} else {
		sctlr_el1 &= ~(1 << 2);
	}

	if (icache_on) {
		sctlr_el1 |= (1 << 12);
	} else {
		sctlr_el1 &= ~(1 << 12);
	}

	// Write back and synchronize to barriers
    asm volatile("msr sctlr_el1, %0" : : "r" (sctlr_el1));
    asm volatile("isb" : : : "memory");
    asm volatile("dsb sy" : : : "memory");
}

void swap(void *a, void *b, size_t size) {
	char temp[size];
    memcpy(temp, a, size);
    memcpy(a, b, size);
    memcpy(b, temp, size);
}

uint32_t max_uint(uint32_t a, uint32_t b) {
    return (a > b) ? a : b;
}

uint32_t min_uint(uint32_t a, uint32_t b) {
	return (a < b) ? a : b;
}

// String to integer conversion
int str2int(const char *str) {
    int result = 0;
    int sign = 1;

    if (*str == '-') {
        sign = -1;
        str++;
    }

    while (*str >= '0' && *str <= '9') {
        result = result * 10 + (*str - '0');
        str++;
    }

    return result * sign;
}

// Render elapsed microseconds as mm:ss.t string
void clock_render(uint64_t elapsed_us, char *buf) {
    uint64_t elapsed_tenths = elapsed_us / 100000;
    // Saturate at 99:59.9 to avoid wrap-around.
    if (elapsed_tenths > 59999) {
        elapsed_tenths = 59999;
    }
    uint32_t minutes = (elapsed_tenths / 600);
    uint32_t seconds = (elapsed_tenths / 10) % 60;
    uint32_t tenths = elapsed_tenths % 10;

    buf[0] = '0' + (minutes / 10);
    buf[1] = '0' + (minutes % 10);
    buf[2] = ':';
    buf[3] = '0' + (seconds / 10);
    buf[4] = '0' + (seconds % 10);
    buf[5] = '.';
    buf[6] = '0' + tenths;
    buf[7] = '\0';
}


#if !defined(MMU)

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
