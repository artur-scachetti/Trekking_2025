#ifndef UART_COMMUNICATION_H
#define UART_COMMUNICATION_H

#include "esp_err.h"
#include "types.h"

esp_err_t uart_init();
esp_err_t uart_send_frame(data_t *cmd);
void uart_send(float *angular_speed_left, float *angular_speed_right, float *servo_angle);
void uart_read();

#endif