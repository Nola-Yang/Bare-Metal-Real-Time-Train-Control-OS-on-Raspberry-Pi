#ifndef _train_control_h_
#define _train_control_h_ 1

// Main train control task
void train_control_task(void);

// Command input task
void command_input_task(void);

// Periodic UI refresh task
void ui_tick_task(void);

#endif /* _train_control_h_ */
