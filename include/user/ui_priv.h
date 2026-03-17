#ifndef _ui_priv_h_
#define _ui_priv_h_ 1

enum {
    UI_RESERVATION_TRAIN_COUNT = 6,
    UI_RESERVATION_BLOCK_ROWS  = 4,
    UI_RESERVATION_BLOCK_COLS  = 2,
    UI_RESERVATION_START_ROW   = 35,
    UI_RESERVATION_GROUP_ROWS  = 3,
    UI_RESERVATION_END_ROW     = UI_RESERVATION_START_ROW +
                                 UI_RESERVATION_GROUP_ROWS * UI_RESERVATION_BLOCK_ROWS - 1,
    UI_RESERVATION_COL_WIDTH   = 58,
    UI_RESERVATION_COL_GAP     = 4,
    UI_RESERVATION_LINE_CHARS  = UI_RESERVATION_COL_WIDTH - 2,
    UI_CMD_SCROLL_TOP          = UI_RESERVATION_END_ROW + 2,
    UI_CMD_SCROLL_BOTTOM       = 120,
};

/* Row 10 switch-change log is drawn during init and on every switch update. */
void ui_draw_switch_log(void);

#endif /* _ui_priv_h_ */
