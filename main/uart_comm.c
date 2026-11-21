#include "uart_comm.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <string.h>
#include "app_queue.h"
#include "app_mqtt.h"
#include "mqtt_config.h"


#define UART_TASK_STACK_SIZE 4096
#define UART_TASK_PRIORITY   5

static const char *TAG = "uart_comm";

#define CMD_SET_USERNAME     0x0001
#define CMD_SET_PASSWORD     0x0002
#define CMD_SAVE_CONFIG      0x0003
#define CMD_SET_BROKER_URI   0x0004

#define CMD_ACK_USERNAME     0x0081
#define CMD_ACK_PASSWORD     0x0082
#define CMD_ACK_SAVE         0x0083
#define CMD_ACK_BROKER_URI   0x0084

#define ACK_SUCCESS          0x00
#define ACK_INVALID_DATA     0x01
#define ACK_CHECKSUM_ERROR   0x02
#define ACK_CONFIG_MISSING   0x03
#define ACK_NVS_ERROR        0x04

#define UART_FRAME_MAX_LEN   UART_RX_MAX_LEN

static uint8_t s_frame_buffer[UART_FRAME_MAX_LEN];
static size_t s_frame_length = 0;
static size_t s_frame_expected = 0;
static bool s_frame_active = false;
static bool s_header_first_byte = false;

static uart_rx_callback_t s_rx_callback = NULL;
static bool s_echo_enabled = true; // Mặc định bật tự động echo dữ liệu < 128 byte
static QueueHandle_t s_uart_queue = NULL; // Hàng đợi sự kiện UART ngắt phần cứng

#define UART_CONFIG_SAVED_BIT       BIT0
#define UART_CFG_USERNAME_RECEIVED  BIT0
#define UART_CFG_PASSWORD_RECEIVED  BIT1
#define UART_CFG_BROKER_RECEIVED    BIT2
#define UART_CFG_ALL_RECEIVED       (UART_CFG_USERNAME_RECEIVED | UART_CFG_PASSWORD_RECEIVED | UART_CFG_BROKER_RECEIVED)

static EventGroupHandle_t s_uart_config_event_group = NULL;
static uint8_t s_config_received_mask = 0;


static uint8_t uart_calculate_xor(
    const uint8_t *data,
    size_t length)
{
    uint8_t checksum = 0;

    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }

    return checksum;
}

static void uart_send_config_ack(
    uint16_t command,
    uint8_t status)
{
    uint8_t frame[9];

 	/* Header */
    frame[0] = 0xAA;
    frame[1] = 0xAA;
	/* Command - Big Endian */
    frame[2] = (uint8_t)(command >> 8);
    frame[3] = (uint8_t)(command & 0xFF);
	/* Length = DataLength + 4 */
    frame[4] = 0x05;
 	/* Data: ACK status */    
    frame[5] = status;
	/* Checksum: XOR Command_H đến Data */
    frame[6] = uart_calculate_xor(&frame[2], 4);

	/* Tail */
    frame[7] = 0x55;
    frame[8] = 0x55;
	/* Gửi 9 byte nhị phân qua UART1 */
    uart_comm_send_bytes(frame, sizeof(frame));

    /* Hiển thị ACK dưới dạng HEX để debug */
    ESP_LOG_BUFFER_HEX("UART_ACK", frame, sizeof(frame));
    
    ESP_LOGI(TAG,
             "Config ACK: command=0x%04X, status=0x%02X",
             command, status);
}

static bool uart_is_config_command(uint16_t command)
{
    return (command == CMD_SET_USERNAME ||
            command == CMD_SET_PASSWORD ||
            command == CMD_SET_BROKER_URI ||
            command == CMD_SAVE_CONFIG);
}
static void uart_process_config_frame(
    const uint8_t *frame,
    size_t total_length)
{
    uint16_t command =
        ((uint16_t)frame[2] << 8) | frame[3];

    // Đọc Length trước khi sử dụng
    uint8_t length_field = frame[4];

    // Kiểm tra độ dài frame
    if (length_field < 4 ||
        total_length != (size_t)length_field + 4) {

        ESP_LOGW(TAG,
                 "Invalid config frame length: Length=%u, Total=%u",
                 length_field,
                 (unsigned int)total_length);
        return;
    }

    size_t data_length = length_field - 4;

    uint8_t received_checksum =
        frame[total_length - 3];

 	// Tính XOR từ Command đến byte Data cuối cùng
	uint8_t calculated_checksum =
    uart_calculate_xor(
        &frame[2],
        length_field - 1);

    ESP_LOGI(TAG,
             "Config checksum: CMD=0x%04X, "
             "Calculated=0x%02X, Received=0x%02X",
             command,
             calculated_checksum,
             received_checksum);
             
    uint16_t ack_command = 0;
    uint8_t status = ACK_SUCCESS;
    bool config_saved = false;

    switch (command) {

    case CMD_SET_USERNAME:
        ack_command = CMD_ACK_USERNAME;

        if (received_checksum != calculated_checksum) {
            status = ACK_CHECKSUM_ERROR;
        } else if (data_length == 0) {
            status = ACK_INVALID_DATA;
        } else {
            esp_err_t err =
                mqtt_config_set_username(
                    &frame[5],
                    data_length);

            if (err != ESP_OK) {
                status = ACK_INVALID_DATA;
            } else {
                s_config_received_mask |= UART_CFG_USERNAME_RECEIVED;
            }
        }
        break;

    case CMD_SET_PASSWORD:
        ack_command = CMD_ACK_PASSWORD;

        if (received_checksum != calculated_checksum) {
            status = ACK_CHECKSUM_ERROR;
        } else if (data_length == 0) {
            status = ACK_INVALID_DATA;
        } else {
            esp_err_t err =
                mqtt_config_set_password(
                    &frame[5],
                    data_length);

            if (err != ESP_OK) {
                status = ACK_INVALID_DATA;
            } else {
                s_config_received_mask |= UART_CFG_PASSWORD_RECEIVED;
            }
        }
        break;
	case CMD_SET_BROKER_URI:
	    ack_command = CMD_ACK_BROKER_URI;
	
	    if (received_checksum != calculated_checksum) {
	        status = ACK_CHECKSUM_ERROR;
	    } else if (data_length == 0) {
	        status = ACK_INVALID_DATA;
	    } else {
	        esp_err_t err = mqtt_config_set_broker_uri(
	            &frame[5],
	            data_length
	        );
	
	        if (err != ESP_OK) {
	            status = ACK_INVALID_DATA;
	        } else {
            s_config_received_mask |= UART_CFG_BROKER_RECEIVED;
        }
	    }
	    break;
    case CMD_SAVE_CONFIG:
        ack_command = CMD_ACK_SAVE;

        if (received_checksum != calculated_checksum) {
            status = ACK_CHECKSUM_ERROR;
        } else if (data_length != 0) {
            status = ACK_INVALID_DATA;
        } else if (s_config_received_mask != UART_CFG_ALL_RECEIVED) {
            ESP_LOGW(TAG,
                     "SAVE_CONFIG rejected: MCU must send Username, Password and Broker URI first (mask=0x%02X)",
                     s_config_received_mask);
            status = ACK_CONFIG_MISSING;
        } else {
	 		
	 		/* Bước 1: Lưu cấu hình MQTT vào NVS */
	        esp_err_t err = mqtt_config_save();
	
	        if (err == ESP_ERR_INVALID_STATE) {
	
	            status = ACK_CONFIG_MISSING;
	
	        } else if (err != ESP_OK) {
	
	            status = ACK_NVS_ERROR;
	
	        } else {
	
				/*
				 * Bước 2: NVS đã lưu thành công.
				 * Yêu cầu MQTT Supervisor cập nhật cấu hình.
				 */
				ESP_LOGI(TAG, "MQTT configuration saved successfully");
                config_saved = true;
				
				err = app_mqtt_request_reconfigure();
				
				if (err != ESP_OK) {
				
				    ESP_LOGW(
				        TAG,
				        "MQTT reconfiguration request failed: %s",
				        esp_err_to_name(err)
				    );
				
				} else {
				
				    ESP_LOGI(
				        TAG,
				        "MQTT reconfiguration request accepted"
				    );
				}
            }
        }
        break;

    default:
        return;
    }

    uart_send_config_ack(ack_command, status);

    if (config_saved && status == ACK_SUCCESS &&
        s_uart_config_event_group != NULL) {
        xEventGroupSetBits(
            s_uart_config_event_group,
            UART_CONFIG_SAVED_BIT
        );
    }
}

static void uart_forward_normal_frame(
    const uint8_t *frame,
    size_t length)
{
    uart_rx_data_t uart_data = {0};

    if (length > UART_RX_MAX_LEN) {
        ESP_LOGW(TAG, "UART frame exceeds queue buffer");
        return;
    }

    uart_data.length = (uint16_t)length;
    memcpy(uart_data.data, frame, length);

    if (xQueueSend(uart_to_mqtt_queue,
                   &uart_data,
                   0) != pdPASS) {
        ESP_LOGW(TAG, "UART -> MQTT Queue full");
    }
}


static void uart_process_frame(
    const uint8_t *frame,
    size_t total_length)
{
    if (total_length < 8 || total_length > UART_FRAME_MAX_LEN) {
        ESP_LOGW(TAG, "Invalid frame length: %u",
                 (unsigned int)total_length);
        return;
    }

    // Check Header và Tail
    if (frame[0] != 0xAA ||
        frame[1] != 0xAA ||
        frame[total_length - 2] != 0x55 ||
        frame[total_length - 1] != 0x55) {

        ESP_LOGW(TAG, "Invalid UART frame format");
        return;
    }

    uint16_t command =
        ((uint16_t)frame[2] << 8) | frame[3];

    uint8_t length_field = frame[4];

    // Kiểm tra Length
    if (length_field < 4 ||
        total_length != (size_t)length_field + 4) {

        ESP_LOGW(TAG,
                 "Invalid length: field=%u, total=%u",
                 length_field,
                 (unsigned int)total_length);
        return;
    }

    // Lấy Checksum
    uint8_t received_cs = frame[total_length - 3];

    // Tính XOR từ Command_H đến Data cuối cùng
    uint8_t calculated_cs =
        uart_calculate_xor(&frame[2], length_field - 1);

    ESP_LOGI(TAG,
             "CHECKSUM: CMD=0x%04X, Length=%u, "
             "Calculated=0x%02X, Received=0x%02X",
             command,
             length_field,
             calculated_cs,
             received_cs);

    // Nếu là lệnh cấu hình MQTT
    if (uart_is_config_command(command)) {

        // Hàm này tự kiểm tra Checksum và trả ACK
        uart_process_config_frame(frame, total_length);
        return;
    }

    // Frame thông thường
    if (received_cs != calculated_cs) {

        ESP_LOGW(TAG,
                 "Normal frame checksum error: CMD=0x%04X",
                 command);
        return;
    }

    uart_forward_normal_frame(frame, total_length);
}


static void uart_parser_reset(void)
{
    s_frame_length = 0;
    s_frame_expected = 0;
    s_frame_active = false;
    s_header_first_byte = false;
}

static void uart_parser_feed_byte(uint8_t byte)
{
    // Tìm header AA AA
    if (s_frame_length == 0) {
        if (byte == 0xAA) {
            s_frame_buffer[0] = byte;
            s_frame_length = 1;
        }
        return;
    }

    if (s_frame_length == 1) {
        if (byte == 0xAA) {
            s_frame_buffer[1] = byte;
            s_frame_length = 2;
        } else {
            s_frame_length = 0;
        }
        return;
    }

    if (s_frame_length >= UART_FRAME_MAX_LEN) {
        uart_parser_reset();
        return;
    }

    s_frame_buffer[s_frame_length++] = byte;

    // Đã nhận đủ Header + Command + Length
    if (s_frame_length == 5) {

        uint8_t length_field = s_frame_buffer[4];

        if (length_field < 4) {
            uart_parser_reset();
            return;
        }

        s_frame_expected = (size_t)length_field + 4;

        if (s_frame_expected > UART_FRAME_MAX_LEN) {
            uart_parser_reset();
            return;
        }
    }

    // Đã nhận đủ frame
    if (s_frame_expected > 0 &&
        s_frame_length == s_frame_expected) {

        uart_process_frame(
            s_frame_buffer,
            s_frame_length);

        uart_parser_reset();
    }
}

esp_err_t uart_comm_init(void) {
    if (s_uart_config_event_group == NULL) {
        s_uart_config_event_group = xEventGroupCreate();
        if (s_uart_config_event_group == NULL) {
            ESP_LOGE(TAG, "Cannot create UART config EventGroup");
            return ESP_ERR_NO_MEM;
        }
    }
    s_config_received_mask = 0;

	const uart_config_t uart_config = {
		.baud_rate = UART_COMM_BAUD_RATE,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	// 1. Cấu hình thông số UART trước
	esp_err_t err = uart_param_config(UART_COMM_PORT_NUM, &uart_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config error: %s", esp_err_to_name(err));
		return err;
	}

	// 2. Cài đặt chân TX, RX và kích hoạt điện trở kéo lên Pull-up nội cho chân RX (GPIO 5)
	err = uart_set_pin(UART_COMM_PORT_NUM, UART_COMM_TX_PIN, UART_COMM_RX_PIN,
					   UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_set_pin error: %s", esp_err_to_name(err));
		return err;
	}
	gpio_set_pull_mode(UART_COMM_RX_PIN, GPIO_PULLUP_ONLY);

	// 3. Cài đặt driver sau cùng với RX/TX buffer và Event Queue (20 sự kiện)
	// Để driver sử dụng cấu hình ngắt mặc định chuẩn xác của ESP-IDF
	err = uart_driver_install(UART_COMM_PORT_NUM, UART_COMM_BUF_SIZE * 2,
							UART_COMM_BUF_SIZE * 2, 20, &s_uart_queue, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_driver_install error: %s", esp_err_to_name(err));
		return err;
	}

	ESP_LOGI(
		TAG,
		"UART initialized successfully. Port=%d, Baud=%d, TX=GPIO%d, RX=GPIO%d",
		UART_COMM_PORT_NUM, UART_COMM_BAUD_RATE, UART_COMM_TX_PIN,
		UART_COMM_RX_PIN);
	return ESP_OK;
}

int uart_comm_send_bytes(const uint8_t *data, size_t len) {
	if (data == NULL || len == 0) {
		return 0;
	}
	return uart_write_bytes(UART_COMM_PORT_NUM, (const char *)data, len);
}

int uart_comm_send_string(const char *str) {
	if (str == NULL) {
		return 0;
	}
	return uart_comm_send_bytes((const uint8_t *)str, strlen(str));
}

int uart_comm_receive_bytes(uint8_t *buf, size_t max_len, uint32_t timeout_ms) {
	if (buf == NULL || max_len == 0) {
		return 0;
	}
	TickType_t ticks = (timeout_ms == portMAX_DELAY)
						   ? portMAX_DELAY
						   : pdMS_TO_TICKS(timeout_ms);
	return uart_read_bytes(UART_COMM_PORT_NUM, buf, max_len, ticks);
}

void uart_comm_register_rx_callback(uart_rx_callback_t cb) {
	s_rx_callback = cb;
}

void uart_comm_set_echo(bool enable) {
	s_echo_enabled = enable;
	ESP_LOGI(TAG, "UART Auto-Echo set to %s (limit < %d bytes)",
			 enable ? "ENABLED" : "DISABLED", UART_COMM_MAX_ECHO_LEN);
}
/*
static void uart_comm_rx_task_entry(void *pvParameters) {
	uint8_t *rx_buf = (uint8_t *)malloc(UART_COMM_BUF_SIZE);
	if (rx_buf == NULL) {
		ESP_LOGE(TAG, "Failed to allocate memory for UART RX buffer");
		vTaskDelete(NULL);
		return;
	}

	uart_event_t event;
	ESP_LOGI(
		TAG,
		"UART RX Event Task started (Listening on GPIO%d RX, Echo < %d bytes)...",
		UART_COMM_RX_PIN, UART_COMM_MAX_ECHO_LEN);

	while (1) {
		// Đợi sự kiện từ hàng đợi s_uart_queue
		if (xQueueReceive(s_uart_queue, (void *)&event, pdMS_TO_TICKS(100))) {
			switch (event.type) {
			case UART_DATA:
				if (event.size > 0) {
					size_t to_read = (event.size < (UART_COMM_BUF_SIZE - 1))
										 ? event.size
										 : (UART_COMM_BUF_SIZE - 1);
					int len = uart_read_bytes(UART_COMM_PORT_NUM, rx_buf,
											  to_read, pdMS_TO_TICKS(100));
					if (len > 0) {
						rx_buf[len] = '\0'; // Đảm bảo null-terminated

						// Phản hồi echo nếu chuỗi ít hơn 128 byte
						if (s_echo_enabled) {
							if (len < UART_COMM_MAX_ECHO_LEN) {
								uart_comm_send_bytes(rx_buf, (size_t)len);
								ESP_LOGI(TAG, "UART Echo [%d bytes]: %.*s", len,
										 len, (char *)rx_buf);
							} else {
								ESP_LOGW(TAG,
										 "UART Received [%d bytes] >= %d bytes limit, skipped echo",
										 len, UART_COMM_MAX_ECHO_LEN);
							}
						}

						// Gọi callback nếu có đăng ký
						if (s_rx_callback != NULL) {
							s_rx_callback(rx_buf, (size_t)len);
						}
					}
				}
				break;

			case UART_FIFO_OVF:
				ESP_LOGW(TAG, "UART HW FIFO Overflow! Resetting buffer...");
				uart_flush_input(UART_COMM_PORT_NUM);
				xQueueReset(s_uart_queue);
				break;

			case UART_BUFFER_FULL:
				ESP_LOGW(TAG, "UART Ring Buffer Full! Resetting buffer...");
				uart_flush_input(UART_COMM_PORT_NUM);
				xQueueReset(s_uart_queue);
				break;

			case UART_FRAME_ERR:
				ESP_LOGE(TAG, "UART Frame Error! Vui lòng kiểm tra Baudrate và chân cắm RX/TX.");
				break;

			case UART_PARITY_ERR:
				ESP_LOGE(TAG, "UART Parity Error!");
				break;

			case UART_BREAK:
				ESP_LOGW(TAG, "UART RX Break Detected!");
				break;

			default:
				ESP_LOGI(TAG, "UART unhandled event type: %d", event.type);
				break;
			}
		}
		else {
				//ESP_LOGI(TAG, "UART unhandled event type: %d", event.type);
			}
	}

	free(rx_buf);
	vTaskDelete(NULL);
}

esp_err_t uart_comm_start_rx_task(uint32_t stack_size, UBaseType_t priority) {
	BaseType_t ret = xTaskCreate(uart_comm_rx_task_entry, "uart_rx_task",
								 (stack_size > 0) ? stack_size : 3072, NULL,
								 (priority > 0) ? priority : 10, NULL);

	return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}
*/
static void uart_comm_task(void *arg)
{
    uart_event_t event;
    //uint8_t rx_buffer[256];

    ESP_LOGI(TAG,
             "UART RX Task started. Listening on GPIO%d",
             UART_COMM_RX_PIN);

    while (1)
    {
        // 1. Chờ sự kiện UART từ UART Driver
        if (xQueueReceive(s_uart_queue,
                          &event,
                          portMAX_DELAY) == pdTRUE)
        {
            switch (event.type)
            {
				/*
                case UART_DATA:
                {
                    // 2. Đọc dữ liệu từ RX Ring Buffer
                    int len = uart_read_bytes(
                        UART_COMM_PORT_NUM,
                        rx_buffer,
                        sizeof(rx_buffer),
                        0
                    );

                    if (len > 0)
                    {
						if (len > UART_RX_MAX_LEN) {
							ESP_LOGW(TAG, "lenght data > UART_RX_MAX_LEN");
							break;
						}
                        // 3. Hiển thị dữ liệu UART nhận được
                        ESP_LOG_BUFFER_HEX(
                            "UART_RX",
                            rx_buffer,
                            len
                        );
                        // Echo dữ liệu UART1 RX sang UART1 TX
						uart_write_bytes(
						    UART_COMM_PORT_NUM,
						    (const char *)rx_buffer,
						    len
						);
						// Tạo packet chứa dữ liệu UART thô
						    uart_rx_data_t uart_data = {0};
						
						    uart_data.length = (uint16_t)len;
						
						    memcpy(
						        uart_data.data,
						        rx_buffer,
						        len
						    );
						
						    // Đưa dữ liệu vào UART -> MQTT Queue
						    if (xQueueSend(
						            uart_to_mqtt_queue,
						            &uart_data,
						            0
						        ) != pdPASS)
						    {
						        ESP_LOGW(TAG, "UART -> MQTT Queue full");
						    }
						    else
						    {
						        ESP_LOGI(TAG, "UART data queued: %d bytes", len);
						    }						
						
                        // TODO:
                        // 4. Đưa dữ liệu vào bộ phân tích frame
                    }
                    break;
                }
				*/
				case UART_DATA:
				{
				    uint8_t rx_buffer[256];
				    size_t buffered_length = 0;
				
				    // Đọc hết dữ liệu đang nằm trong UART RX Ring Buffer
				    do {
				        if (uart_get_buffered_data_len(
				                UART_COMM_PORT_NUM,
				                &buffered_length) != ESP_OK) {
				            break;
				        }
				
				        if (buffered_length == 0) {
				            break;
				        }
				
				        size_t read_size = buffered_length;
				
				        if (read_size > sizeof(rx_buffer)) {
				            read_size = sizeof(rx_buffer);
				        }
				
				        int len = uart_read_bytes(
				            UART_COMM_PORT_NUM,
				            rx_buffer,
				            read_size,
				            0
				        );
				
				        if (len <= 0) {
				            break;
				        }
						// Hiển thị toàn bộ dữ liệu nhận từ UART1
						ESP_LOGI(TAG, "UART1 RX: %d bytes", len);
						ESP_LOG_BUFFER_HEX("UART_RX", rx_buffer, len);				
				        // Đưa từng byte vào parser
				        for (int i = 0; i < len; i++) {
				            uart_parser_feed_byte(rx_buffer[i]);
				        }
				
				    } while (buffered_length > 0);
				
				    break;
				}
                case UART_FIFO_OVF:
                {
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(UART_COMM_PORT_NUM);
                    xQueueReset(s_uart_queue);
                    break;
                }

                case UART_BUFFER_FULL:
                {
                    ESP_LOGW(TAG, "UART RX buffer full");
                    uart_flush_input(UART_COMM_PORT_NUM);
                    xQueueReset(s_uart_queue);
                    break;
                }

                case UART_FRAME_ERR:
                    ESP_LOGW(TAG, "UART frame error");
                    break;

                case UART_PARITY_ERR:
                    ESP_LOGW(TAG, "UART parity error");
                    break;

                default:
                    ESP_LOGD(TAG,
                             "UART event type: %d",
                             event.type);
                    break;
            }
        }
    }
}

esp_err_t uart_comm_start_task(void)
{
    BaseType_t result = xTaskCreate(
        uart_comm_task,
        "uart_comm_task",
        UART_TASK_STACK_SIZE,
        NULL,
        UART_TASK_PRIORITY,
        NULL
    );

    if (result != pdPASS)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t uart_comm_wait_config_saved(TickType_t timeout)
{
    if (s_uart_config_event_group == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    EventBits_t bits = xEventGroupWaitBits(
        s_uart_config_event_group,
        UART_CONFIG_SAVED_BIT,
        pdTRUE,
        pdFALSE,
        timeout
    );

    if ((bits & UART_CONFIG_SAVED_BIT) != 0) {
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}
