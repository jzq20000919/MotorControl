#ifndef MOTOR_UI_H
#define MOTOR_UI_H

#include "lvgl.h"

void motor_ui_create(lv_display_t *display);
void motor_ui_attach_input(lv_indev_t *indev);

#endif /* MOTOR_UI_H */
