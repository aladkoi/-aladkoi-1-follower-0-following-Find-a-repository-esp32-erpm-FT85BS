/* UART asynchronous example with state structure
   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <math.h>
#include "esp_intr_alloc.h"
#include "esp_pm.h"
#include "iot_button.h"
#include "esp_sleep.h"
#include <stdio.h>
#include <inttypes.h>
#include "button_gpio.h"
#include "crc.h"  // Подсчет по duty
#include "led_break.h"

#define TAG "BUTTON"

#define BUTTON_NUM1 23 // Левая кнопка
#define BUTTON_NUM2 22 // Правая кнопка
#define BUTTON_ACTIVE_LEVEL 0
#define BLUE_LED_PIN GPIO_NUM_2
#define POWER_PIN 4
#define BREAK_PIN 36 // Вход стоп-сигнала
#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define RX_TIMEOUT (1000 / portTICK_PERIOD_MS)

// Структура состояния системы
typedef struct {
    char name_cont[8];         // Имя контроллера (с завершающим нулем)
    float Vbatt;               // Напряжение батареи
    int rpm_controller;        // Текущие обороты
    int volt_bl;               // Уровень для передачи в контроллер
    int break_volt_bl;         // Уровень сигнала от мобильного при торможении
    int start_level;           // Уровень начального старта
    int current_rpm;           // Текущий уровень RPM для сохранения
    int current_level;         // Текущий уровень газа
    int current_level_speed;   // Уровень газа для возобновления скорости
    int current_level_rpm;     // Уровень RPM для возобновления скорости
    int croiuse_level;         // Текущий уровень круиза
    int break_croiuse_level;   // Уровень круиза при торможении
    bool controllerBrake;      // Нажата ручка тормоза
    int currentSpeed;          // Скорость для круиза
    int cr;                    // Текущий индекс круиза
    int current_amper;         // Текущий индекс шага скорости
    int target_erpm;           // Целевая скорость (ERPM)
    bool crouise_on;           // Включен круиз
    bool isn;                  // Умный тормоз
    int count_telemtr;         // Количество телеметрии
    int numberCrouise;         // Номер круиза
    bool stop_from_mobile;     // Стоп с мобильного
    bool break_long;           // Долгое нажатие тормоза
    bool speed_up;             // Скорость нарастает
    bool addspeed;             // Увеличение скорости с 0
    int break_level;           // Уровень тормоза (1 - не нажат, 0 - нажат)
    int operation;             // Номер операции (0 - соединение, 1 - телеметрия)
} ControllerState_t;

// Инициализация состояния
static ControllerState_t state = {
    .name_cont = "",
    .Vbatt = 0.0f,
    .rpm_controller = 0,
    .volt_bl = 0,
    .break_volt_bl = 0,
    .start_level = 0,
    .current_rpm = 0,
    .current_level = 0,
    .current_level_speed = 0,
    .current_level_rpm = 0,
    .croiuse_level = -1,
    .break_croiuse_level = -1,
    .controllerBrake = false,
    .currentSpeed = 0,
    .cr = -1,
    .current_amper = 3, // Начальный индекс шага скорости
    .target_erpm = 0,
    .crouise_on = false,
    .isn = true,
    .count_telemtr = 0,
    .numberCrouise = 255,
    .stop_from_mobile = false,
    .break_long = false,
    .speed_up = false,
    .addspeed = false,
    .break_level = 1,
    .operation = 0
};

// Мьютекс для синхронизации доступа к состоянию
static SemaphoreHandle_t state_mutex;

// Очередь UART
static QueueHandle_t uart_queue;


// Массивы круиза
volatile float level_crouise[6] = {0, 21.4, 33.2, 46.0, 58.4, 95.0};
volatile int rpm_crouise[6] = {2611, 4700, 7330, 10115, 12870, 20800};
const int len_crouise = 5; // Размер массива круиза минус 1

// Названия событий кнопок
static const char *button_event_names[] = {
    [BUTTON_PRESS_DOWN] = "BUTTON_PRESS_DOWN",
    [BUTTON_PRESS_UP] = "BUTTON_PRESS_UP",
    [BUTTON_PRESS_REPEAT] = "BUTTON_PRESS_REPEAT",
    [BUTTON_SINGLE_CLICK] = "BUTTON_SINGLE_CLICK",
    [BUTTON_DOUBLE_CLICK] = "BUTTON_DOUBLE_CLICK",
    [BUTTON_LONG_PRESS_START] = "BUTTON_LONG_PRESS_START",
    [BUTTON_LONG_PRESS_HOLD] = "BUTTON_LONG_PRESS_HOLD",
    [BUTTON_LONG_PRESS_UP] = "BUTTON_LONG_PRESS_UP",
};

// Функции управления состоянием
static void stop_Speed(bool status) {
        if (status && !state.controllerBrake) {
            state.controllerBrake = true;
            if (!state.isn) { // Умный тормоз
                state.current_level = 0;
                state.current_rpm = 0;
            }
            state.volt_bl = 0;
            state.numberCrouise = 255;
            state.crouise_on = false;
            state.croiuse_level = -1;
            state.cr = -1;
            getDuty(0.0);
    }
}

static void setCurrentLevel(void) {
        //printf("setCurrentLevel\n");
        state.current_level_speed = state.volt_bl;
        state.current_level = state.volt_bl;
        state.current_level_rpm = state.rpm_controller;
}

static int get_level(bool forward) {
    int cr = -1;
        if (forward && state.volt_bl >= rpm_crouise[len_crouise]) {
            cr = len_crouise;
        } else if (forward) {
            for (int i = 0; i <= len_crouise; i++) {
                if (rpm_crouise[i] > state.volt_bl) {
                    cr = i;
                    break;
                }
            }
        } else {
            for (int i = len_crouise; i >= 0; i--) {
                if (state.volt_bl > rpm_crouise[i]) {
                    cr = i;
                    break;
                }
            }
        }
        if (!forward && cr == -1) cr = -1;
        else if (cr == -1) cr = 0;
    return cr;
}

static void do_add_speed(void* parameter) {
    const uint32_t DELAY_MS = 450;
    //const uint32_t CHECK_INTERVAL_MS = 450;
    //TickType_t last_check_time;
    int last_index = 0;

    while (1) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            if (state.rpm_controller > 0) {
                if (state.rpm_controller > state.volt_bl)
                    state.current_amper = getCurrentDecIndexSpeed(state.volt_bl);
                else
                    state.current_amper = getCurrentIndexSpeed(state.rpm_controller);
            } else {
                state.current_amper = 3;
            }
            bool local_addspeed = state.addspeed;
            xSemaphoreGive(state_mutex);
            
            if (local_addspeed) {
                //last_check_time = xTaskGetTickCount();
                if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                    last_index = find_index_linear(state.volt_bl);
                    if (last_index < 0) last_index = 0;
                    state.target_erpm = erpm_step[state.current_amper];
                    xSemaphoreGive(state_mutex);
                }

                while (state.break_level == 1 && local_addspeed && state.current_amper <= last_index) {
                    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                        state.target_erpm = erpm_step[state.current_amper];
                        state.current_amper++;
                        local_addspeed = state.addspeed;
                        xSemaphoreGive(state_mutex);
                    }
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

                if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                    state.addspeed = false;
                    local_addspeed = false;
                    xSemaphoreGive(state_mutex);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    vTaskDelete(NULL);
}

static void select_level(int crouise) {
          if (state.volt_bl == 0 || (crouise > -1 && crouise < (len_crouise + 1))) {
            if (!state.speed_up) {
                state.volt_bl = rpm_crouise[crouise];
                state.currentSpeed = rpm_crouise[crouise];
            } else {
                state.volt_bl = rpm_crouise[crouise];
                if (state.break_level == 1 && crouise > 0) {
                    state.addspeed = true;
                }
            }
            state.controllerBrake = false;
        }  
}

static float start_crouise(void) {
    float result = 0.0f;
        state.croiuse_level = 0;
        select_level(state.croiuse_level);
        if (state.volt_bl <= state.start_level) state.volt_bl = rpm_crouise[0];
        if (state.volt_bl > 20800) state.volt_bl = 20800;
        getDuty(state.volt_bl);
        state.current_level = state.volt_bl;
        result = state.volt_bl;
    return result;
}

static void AddSpeed(void) {
        if (state.break_level == 1 && state.volt_bl < rpm_crouise[len_crouise]) {
            if (state.volt_bl <= state.start_level || state.volt_bl == 0) {
                state.volt_bl = start_crouise();
            } else {
                state.volt_bl = getStep(true, state.volt_bl);
                if (state.volt_bl > rpm_crouise[len_crouise]) state.volt_bl = rpm_crouise[len_crouise];
            }
        }
        printf("AddSpeed1\n");
        setCurrentLevel();
}

static void setCrouise(int crouise) {
       if (state.cr == crouise) {            
            return;
        }
        state.crouise_on = true;
        state.cr = crouise;
        state.numberCrouise = crouise;
        select_level(crouise);
        printf("setCrouise1\n");
        setCurrentLevel();
}

// Обработчики событий кнопок
static void button_event_break(void *arg, void *data) {
    printf("button_event_break\n");
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        printf("do button_event_break\n");
        state.break_long = false;
        state.break_level = 0;
        gpio_set_level(BLUE_LED_PIN, 1);
        DoLight(128);
        state.controllerBrake = false;
        state.break_volt_bl = state.volt_bl;
        state.break_croiuse_level = state.croiuse_level;
        state.addspeed = false;
        stop_Speed(true);
        setCurrentLevel();
        xSemaphoreGive(state_mutex);
    }
    printf("button_event_break_end--------\n");
}

static void button_event_break_long(void *arg, void *data) {
    printf("button_event_break_long\n");
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        state.break_long = true;
        state.addspeed = false;
        state.current_amper = 3; // Сбрасываем в исходное состояние
        xSemaphoreGive(state_mutex);
    }
    printf("button_event_break_long_end--------\n");
}

static void button_event_break_end(void *arg, void *data) {
    printf("button_event_break_end\n");
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        gpio_set_level(BLUE_LED_PIN, 0);
        state.rpm_controller = 0;
        if (!state.break_long) {
            state.volt_bl = state.break_volt_bl;
            if (state.volt_bl > 0) {
                state.croiuse_level = get_level(true) - 1;
                if (state.croiuse_level < 1) state.croiuse_level = 0;
                state.cr = -1;
                state.crouise_on = true;
                state.speed_up = true;
                state.break_level = 1;
                state.volt_bl = 0;
                setCrouise(state.croiuse_level);
            }
        }
        state.break_volt_bl = 0;
        state.break_long = false;
        state.break_level = 1;
        DoLight(0);
        xSemaphoreGive(state_mutex);
    }
     printf("button_event_break_end_end--------\n");
}

static void button_event_cb1(void *arg, void *data) {
    //printf("button_event_cb1\n");
    button_handle_t btn_handle = (button_handle_t)arg;
    button_event_t event = iot_button_get_event(btn_handle);
    if (event >= sizeof(button_event_names) / sizeof(button_event_names[0]) || !button_event_names[event]) return;

    const char *event_name = button_event_names[event];
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        if (strcmp(state.name_cont, "FT85BS") != 0) {
            xSemaphoreGive(state_mutex);
            return;
        }
          
        if (strcmp(event_name, "BUTTON_SINGLE_CLICK") == 0) {
            //printf("button_event_single\n");
            //printf("state.croiuse_level=%d\n",state.croiuse_level);
            //printf("state.rpm_controller%d\n",state.rpm_controller);
            if (state.croiuse_level == -1 && state.rpm_controller == 0) {
                state.croiuse_level = 0;
                state.cr=-1;
                state.crouise_on = true;
                state.speed_up = true;
                setCrouise(state.croiuse_level);
            } else {
                AddSpeed();
                state.currentSpeed = 0;
            }
        } else if (strcmp(event_name, "BUTTON_PRESS_REPEAT") == 0) {
            if (state.croiuse_level < len_crouise) {
                state.croiuse_level = get_level(true);
                if (state.croiuse_level < 1) state.croiuse_level = 0;
                if (state.croiuse_level == 0) state.croiuse_level = 1;
                state.crouise_on = true;
                state.speed_up = true;
                setCrouise(state.croiuse_level);
            }
        } else if (strcmp(event_name, "BUTTON_LONG_PRESS_HOLD") == 0) {
            AddSpeed();
        }
        xSemaphoreGive(state_mutex);
    }
    printf("button_event_cb1_end\n");
}

static void button_event_cb2(void *arg, void *data) {
    printf("button_event_cb2\n");
    button_handle_t btn_handle = (button_handle_t)arg;
    button_event_t event = iot_button_get_event(btn_handle);
    if (event >= sizeof(button_event_names) / sizeof(button_event_names[0]) || !button_event_names[event]) return;

    const char *event_name = button_event_names[event];
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
        state.addspeed = false;
        if (strcmp(event_name, "BUTTON_SINGLE_CLICK") == 0) {
            state.crouise_on = false;
            if (state.break_level == 1 && state.volt_bl >= state.start_level)
                state.volt_bl = getStep(false, state.volt_bl);
            if (state.volt_bl < 0) state.volt_bl = 0;
            setCurrentLevel();
        } else if (strcmp(event_name, "BUTTON_PRESS_REPEAT") == 0) {
            state.croiuse_level = get_level(false);
            if (state.croiuse_level < 0) {
                state.volt_bl = 0;
                setCurrentLevel();
            }
            if (state.croiuse_level > len_crouise) state.croiuse_level = len_crouise;
            if (state.croiuse_level != -1) {
                state.crouise_on = true;
                state.speed_up = false;
                setCrouise(state.croiuse_level);
            } else {
                state.stop_from_mobile = true;
            }
        } else if (strcmp(event_name, "BUTTON_LONG_PRESS_HOLD") == 0) {
            state.crouise_on = false;
            if (state.break_level == 1 && state.volt_bl >= state.start_level)
                state.volt_bl = getStep(false, state.volt_bl);
            if (state.volt_bl < 0) state.volt_bl = 0;
            setCurrentLevel();
        }
        xSemaphoreGive(state_mutex);
    }
    printf("button_event_cb2_end\n");
}

// Инициализация кнопок
static void button_init_break(uint32_t button_num) {
    button_config_t btn_cfg = {
        .long_press_time = 3000,
        .short_press_time = 50,
    };
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_DOWN, NULL, button_event_break, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_START, NULL, button_event_break_long, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_END, NULL, button_event_break_end, NULL);
    ESP_ERROR_CHECK(ret);
}

static void button_init1(uint32_t button_num) {
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT, NULL, button_event_cb1, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb1, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, NULL, button_event_cb1, NULL);
    ESP_ERROR_CHECK(ret);
}

static void button_init2(uint32_t button_num) {
    button_config_t btn_cfg = {0};
    button_gpio_config_t gpio_cfg = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
    };
    button_handle_t btn;
    esp_err_t ret = iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn);
    assert(ret == ESP_OK);
    ret |= iot_button_register_cb(btn, BUTTON_PRESS_REPEAT, NULL, button_event_cb2, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, NULL, button_event_cb2, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, NULL, button_event_cb2, NULL);
    ESP_ERROR_CHECK(ret);
}

static void init_button(void) {
    button_init1(BUTTON_NUM1);
    button_init2(BUTTON_NUM2);
    button_init_break(BREAK_PIN);
}

// Обработка данных UART
static uint8_t* processData(uint8_t* target_bytes, uint8_t* read_buff, int buff_size, int offset) {
    int index = -1;
    for (int i = 0; i < buff_size - 1; i++) {
        if (read_buff[i] == target_bytes[0] && read_buff[i + 1] == target_bytes[1]) {
            index = i;
            break;
        }
    }
    if (index != -1) {
        uint8_t* new_buff = (uint8_t*) malloc(buff_size - offset);
        if (new_buff) {
            memcpy(new_buff, read_buff, index);
            memcpy(new_buff + index, read_buff + index + offset, buff_size - index - offset);
            memcpy(read_buff, new_buff, buff_size - offset);
            free(new_buff);
        }
    }
    return read_buff;
}

static void sendBufferToController(uint8_t *buffer, uint8_t bufferSize) {
    uart_write_bytes(UART_NUM, (const char*)buffer, bufferSize);
    uart_wait_tx_done(UART_NUM, 2000 / portTICK_PERIOD_MS);
}

static void uart_task(void *arg) {
    uart_event_t event;
    uint8_t *rx_data = (uint8_t *)malloc(BUF_SIZE);
    size_t rx_length = 0;
    uint8_t start_bytes[] = {170, 12};

    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    rx_length = uart_read_bytes(UART_NUM, rx_data, event.size, RX_TIMEOUT);
                    if (rx_length > 0) {
                        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
                            if (state.operation == 0) {
                                processData(start_bytes, rx_data, rx_length, 6);
                                strncpy(state.name_cont, (char*)rx_data, 6);
                                state.name_cont[6] = '\0';
                                if (strcmp(state.name_cont, "FT85BS") == 0) {
                                    ESP_LOGI("UART", "Connected to controller");
                                    gpio_set_level(POWER_PIN, 1);
                                    state.operation = 1;
                                    state.count_telemtr = 0;
                                    xTaskCreatePinnedToCore(do_add_speed, "Do add speed", 10000, NULL, 10, NULL, 0);
                                }
                            } else if (state.operation == 1) {
                                if (rx_length == 34 && rx_data[0] == 170 && rx_data[1] == 29 && rx_data[33] == 221) {
                                    state.Vbatt = (rx_data[5] << 8) | rx_data[6];
                                    uint16_t rpm = (rx_data[17] << 8) | rx_data[18];
                                    if (rpm < 30000) state.rpm_controller = rpm;
                                    if (state.Vbatt < 3000) {
                                        memset(state.name_cont, 0, sizeof(state.name_cont));
                                        gpio_set_level(POWER_PIN, 0);
                                    }
                                }
                            }
                            xSemaphoreGive(state_mutex);
                        }
                    }
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGE("UART", "FIFO Overflow");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                case UART_BUFFER_FULL:
                    ESP_LOGE("UART", "Buffer Full");
                    uart_flush_input(UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                default:
                    break;
            }
        }
    }
    free(rx_data);
    vTaskDelete(NULL);
}

static void loop_controller(void* parameter) {
    printf("Start loop_controller\n");
    int my_voltbl = 0;
    uint8_t start_buffer[] = {170, 1, 17, 76, 127, 221};
    uint8_t telemetr_data[] = {0xaa, 0x04, 0x1a, 0x01, 0x00, 0x64, 0x17, 0x57, 0xdd};
    uint8_t stop_data[] = {170, 5, 4, 0, 0, 0, 0, 192, 213, 221};
    TickType_t wait_time = 2000 / portTICK_PERIOD_MS;

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    while (1) {
        if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
            if (strcmp(state.name_cont, "FT85BS") != 0) {
                wait_time = 2000 / portTICK_PERIOD_MS;
                state.operation = 0;
                sendBufferToController(start_buffer, sizeof(start_buffer));
            } else {
                wait_time = 50 / portTICK_PERIOD_MS;
                if (state.operation == 1) {
                    if (state.count_telemtr == 0 || state.count_telemtr == 4 || state.count_telemtr == 8 ||
                        state.count_telemtr == 12 || state.count_telemtr == 16) {
                        sendBufferToController(telemetr_data, sizeof(telemetr_data));
                    } else {
                        if (state.break_level == 0) {
                            my_voltbl = 0;
                            sendBufferToController(stop_data, sizeof(stop_data));
                        } else {
                            if (state.volt_bl == 0 && state.current_level == 0) getDuty(0.0);
                            else if (state.addspeed) {
                                getSpeed(state.target_erpm);
                                sendBufferToController(speed_data, sizeof(speed_data));
                                state.croiuse_level = get_level(true) - 1;
                                if (state.croiuse_level < 1) state.croiuse_level = 0;
                            } else {
                                if (state.volt_bl == 0 && state.current_level > 0) state.volt_bl = state.current_level;
                                if (state.volt_bl > 20800) state.volt_bl = 20800;
                                //printf("setCurrentLevel() from loop_controller\n");
                                //setCurrentLevel();
                                if (my_voltbl != state.volt_bl) {
                                    my_voltbl = state.volt_bl;
                                    getSpeed(state.volt_bl);
                                    setCurrentLevel();
                                    //sendBufferToController(speed_data, sizeof(speed_data));
                                }
                                    sendBufferToController(speed_data, sizeof(speed_data));
                            }
                        }
                    }
                    state.count_telemtr++;
                    if (state.count_telemtr > 19) state.count_telemtr = 0;
                }
            }
            xSemaphoreGive(state_mutex);
        }
        vTaskDelay(wait_time);
    }
}

static void configure_led(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BLUE_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void app_main(void) {
    state_mutex = xSemaphoreCreateMutex();

    init_button();
    configure_led();
    getDuty(0.0);
    initBreakLight();

    gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(POWER_PIN, 0);

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM));

    printf("Start prog\n");
    xTaskCreatePinnedToCore(loop_controller, "Loop_controller", 4096, NULL, 10, NULL, 1);
}
