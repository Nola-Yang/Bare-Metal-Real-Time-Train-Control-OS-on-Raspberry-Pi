#ifndef _ui_h_
#define _ui_h_ 1

#include <stdint.h>

// Initialize UI with terminal server TID
void ui_init(int term_tid);
void ui_prepare_cmd(void);
void ui_scroll_cmd(void);
void ui_cmd_newprompt(void);
void ui_cmd_backspace(void);
void ui_cmd_putc(char c);

// Draw components
void ui_switches(void);
void ui_draw_sensors(uint64_t start_us);
void ui_update_clock(uint64_t start_us, uint64_t now);
void ui_update_idle(int percent);

// Dirty flag management
int ui_is_switches_dirty(void);
int ui_is_sensors_dirty(void);
void ui_mark_switches_clean(void);
void ui_mark_sensors_clean(void);
void ui_mark_switches_dirty(void);
void ui_mark_sensors_dirty(void);

// ui_puts: send string via terminal server
void ui_puts(const char *str);

// ui_clear_line: Clears a line
void ui_clear_line();

#ifdef MEASURE
// ui_print_sensor_time: Prints the starting and ending time a train
//  passes over 2 sensors in a CSV format
void ui_print_sensor_time(uint64_t start, uint64_t end);
#endif

#endif /* _ui_h_ */
