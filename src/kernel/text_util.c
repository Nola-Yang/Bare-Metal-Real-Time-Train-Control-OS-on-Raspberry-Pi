#include "text_util.h"

#define TEMP_BUF_SIZE 768
static char temp_buf[TEMP_BUF_SIZE];


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

char* str_buf_append(char *p, const char *str) {
    while (*str && (p - temp_buf) < TEMP_BUF_SIZE - 1) {
        *p++ = *str++;
    }
    return p;
}

char* str_buf_append_char(char *p, char c) {
    if ((p - temp_buf) < TEMP_BUF_SIZE - 1) {
        *p++ = c;
    }
    return p;
}

char* str_buf_append_int(char *p, int value) {
    char num_buf[12];
    i2a(value, num_buf);
    return str_buf_append(p, num_buf);
}

char* str_buf_append_uint(char *p, unsigned int value) {
    char num_buf[12];
    ui2a(value, 10, num_buf);
    return str_buf_append(p, num_buf);
}

char* str_buf_clear_to_line_end(char *p) {
    return str_buf_append(p, "\033[K");
}

char* str_buf_move_cursor(char *p, uint32_t line_pos, uint32_t cursor_pos) {
    p = str_buf_append(p, "\033[7;1H");
    p = str_buf_append_uint(p, line_pos);
    p = str_buf_append_char(p, ';');
    p = str_buf_append_uint(p, cursor_pos);
    return str_buf_append_char(p, 'H');
}

char *str_buf_save_cursor(char *p) {
    return str_buf_append(p, "\033[s");
}

char *str_buf_previous_cursor(char *p) {
    return str_buf_append(p, "\033[u");
}

char* str_buf_get_temp(void) {
    return temp_buf;
}