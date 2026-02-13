#ifndef TEXT_UTIL_H
#define TEXT_UTIL_H

#include <stdint.h>


// strcmp: Compares 2 strings
int strcmp(const char *s1, const char *s2);

// strcpy: Copies a string of size 'n' (includes the end character)
void strncpy(char *dest, const char *src, int n);

// str_buf_append: Puts a string into the string buffer
char* str_buf_append(char *p, const char *str);

// str_buf_append: Puts a character int the string buffer
char* str_buf_append_char(char *p, char c);

// str_buf_append_int: Puts an integer into the string buffer
char* str_buf_append_int(char *p, int value);

// str_buf_append_uint: Puts an unsinged integer into the string buffer
char* str_buf_append_uint(char *p, unsigned int value);

// str_buf_clear_to_line_end: Puts the ANSI code for clearing from the cursor
//  to the end of the line
char* str_buf_clear_to_line_end(char *p);

// str_buf_move_cursor(p): Buffers moving the cursor
char* str_buf_move_cursor(char *p, uint32_t line_pos, uint32_t cursor_pos);

// str_buf_save_cursor: Buffer the saving of the cursor
char *str_buf_save_cursor(char *p);

// str_buf_previous_cursor: Buffers retreiving the previous cursor position
char *str_buf_previous_cursor(char *p);

// str_buf_get_temp: Retrieves a temporary buffer
char* str_buf_get_temp(void);


#endif