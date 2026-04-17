#include "uart.h"
#include "global_variables.h"
#include "sycronization.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_timer.h"

typedef enum {

    STATE_WAIT_SOF,         //Start of Frame
    STATE_WAIT_PAYLOAD,
    STATE_WAIT_CHECKSUM,
    STATE_WAIT_EOF          //End of Frame

} parser_state_t;

static const char *TAG = "UART";

static parser_state_t s_current_state = STATE_WAIT_SOF;
static uint8_t s_data_index = 0;

static union {

    uint8_t bytes[PAYLOAD_SIZE_RX];
    data_t data;

} s_payload_buffer;

static uint8_t calculate_checksum() 
{
    uint8_t chk = 0;
    for (int i = 0; i < PAYLOAD_SIZE_RX; i++)
    {
        chk += s_payload_buffer.bytes[i];
    }
    
    return chk & 0xFF;
}

static void process_received_byte(uint8_t byte)
{
    switch(s_current_state)
    {
        case STATE_WAIT_SOF:

            if(byte == FRAME_SOF)
            {
                s_current_state = STATE_WAIT_PAYLOAD;
                s_data_index = 0;
            }

            break;

        case STATE_WAIT_PAYLOAD:

            s_payload_buffer.bytes[s_data_index] = byte;
            s_data_index++;

            if(s_data_index >= PAYLOAD_SIZE_RX)
            {
                s_current_state = STATE_WAIT_CHECKSUM;
            }

            break;

        case STATE_WAIT_CHECKSUM:
            {
                uint8_t calculated_chk = calculate_checksum();

                if(byte == calculated_chk)
                {
                    s_current_state = STATE_WAIT_EOF;
                }

                else
                {
                    ESP_LOGW(TAG, "Checksum falhou. Byte esperado: 0x%02X, Byte recebido: 0x%02X", calculated_chk, byte);
                    s_current_state = STATE_WAIT_SOF;
                }
            
            }

            break;

        case STATE_WAIT_EOF:
            
            if(byte == FRAME_EOF)
            {
                memcpy(&global_ros_angular_speed_left, &s_payload_buffer.data.angular_speed_left, 4);
                memcpy(&global_ros_angular_speed_right, &s_payload_buffer.data.angular_speed_right, 4);
                memcpy(&global_ros_servo_angle, &s_payload_buffer.data.servo_angle, 4);

                ESP_LOGI(TAG, "Frame válido! V Angular E: %.2f | V Angular D: %.2f | Angulo Servo: %.2f", 
                         global_ros_angular_speed_left, global_ros_angular_speed_right, global_ros_servo_angle);
            }

            else
            {
                ESP_LOGW(TAG, "EOF falhou! Esperado: 0x%02X, Recebido: 0x%02X", FRAME_EOF, byte);
            }

            s_current_state = STATE_WAIT_SOF;
            break;

    }
}

esp_err_t uart_init()
{ 
    const uart_config_t uart_config = {

        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    
    ESP_LOGI(TAG, "Driver UART0 (USB) inicializado a 115200 bps.");
    return ESP_OK;

}


esp_err_t uart_send_frame(data_to_send_t *cmd, size_t size)
{

    const int frame_size = 1 + size + 1 + 1; // SOF + Payload + Checksum + EOF

    uint8_t frame[frame_size];
    int index = 0;

    frame[index++] = FRAME_SOF;

    memcpy(&frame[index], cmd, size);
    index += size;

    uint8_t chk = 0;

    uint8_t *cmd_bytes = (uint8_t *)cmd;

    for (int i = 0; i < size; i++) {
        chk += cmd_bytes[i];
    }
    frame[index++] = chk & 0xFF;

    frame[index++] = FRAME_EOF;

    int bytes_sent = uart_write_bytes(UART_PORT_NUM, frame, frame_size);

    if (bytes_sent == frame_size) {
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "Falha ao enviar frame, bytes_sent=%d (esperado %d)", bytes_sent, frame_size);
        return ESP_FAIL;
    }

}


void uart_send(double *total_x_displacement, double *total_y_displacement, double *total_angular_displacement)
{
    xSemaphoreTake(xSemaphore_getSpeed,portMAX_DELAY);

    data_to_send_t data;

    static int total_x_micrometers;
    static int total_y_micrometers;
    static int total_theta_mmrad;

    uint32_t timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    
    total_x_micrometers = (int)((*total_x_displacement)*1000);
    total_y_micrometers = (int)((*total_y_displacement)*1000);
    total_theta_mmrad = (int)((*total_angular_displacement)*1000);

    memcpy(data,&total_x_micrometers,4);
    memcpy(data+4,&total_y_micrometers,4);
    memcpy(data+8,&total_theta_mmrad,4);
    memcpy(data+12,&timestamp,4);

    esp_err_t ret = uart_send_frame(&data, PAYLOAD_SIZE_TX);

    xSemaphore_Give(xSemaphore_getSpeed);

}


void uart_read()
{
    xSemaphore_Take(xSemaphore_getRosSpeed, portMAX_DELAY);

    static uint8_t data[RD_BUF_SIZE];

    int len = uart_read_bytes(UART_PORT_NUM, data, RD_BUF_SIZE, pdMS_TO_TICKS(20));

    if (len > 0) {
        
        for (int i = 0; i < len; i++) {
            process_received_byte(data[i]);
        }
    }

    xSemaphore_Give(xSemaphore_getRosSpeed);
}