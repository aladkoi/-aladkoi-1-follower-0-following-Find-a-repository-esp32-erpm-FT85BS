/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <math.h>
#include <hal/gpio_types.h>
#include "esp_intr_alloc.h"
#include "esp_pm.h"
#include "iot_button.h"
#include "esp_sleep.h"
#include <stdio.h>
#include <inttypes.h>

#include "esp_pm.h"
#include "iot_button.h"
#include "esp_sleep.h"
#include "button_gpio.h"

#include "esp_log.h"
#define TAG "BUTTON"

#include <stdio.h>
#include <inttypes.h>
#include <crc.h>   /// это подсчет по duty
#include <led_break.h>
#include "driver/uart_vfs.h"
#include "freertos/semphr.h"


#define BUTTON_NUM1 23 // левая
#define BUTTON_NUM2 22 //правая
#define BUTTON_ACTIVE_LEVEL     0

#define BLUE_LED_PIN GPIO_NUM_2
const uart_port_t uart_num = UART_NUM_1; // Используем UART1
 #define POWER_PIN 4 
//#include <HardwareSerial.h>
//#include "esp_log.h"
// #include <ctype.h>
// #include <stdlib.h>
//#define BUF_SIZE (2048)
#define BUF_SIZE (1024)              // Размер буфера
#define RX_TIMEOUT (1000 / portTICK_PERIOD_MS) // Таймаут 1 сек
uint8_t read_buff[BUF_SIZE];
//uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

TaskHandle_t Task11=0;
TaskHandle_t TaskSpeed=0;
TaskHandle_t Task3=0;
uint8_t stop_data[]={170,5,4,0,0,0,0,192,213,221}; 
char name_cont[7] = ""; 
//char Vbatt[5];
float Vbatt = 0.0f; /// напряжение батареи
int rpm_controller=0;
int count_summ=0;
bool power=true; /// по умолчанию нет мобильного
bool mobile=false;  /// управление с мобильного
volatile int operation=0;   /// номер операции
                            /// 0 - это соединение с контроллером
                            /// 1 - получение телеметрии
volatile float speed=0;
volatile int volt_bl=0;   /// уровень для передачи в контрорллер
volatile int break_volt_bl=0;   /// уровень сигнала от мобильного
volatile int start_level=0;//  уровень начального старта, передается с мобильного
volatile int current_rpm=0;  /// это текущий уровень ручки газа для сохранения по нажатию ручки тормоза
volatile int current_level=0;  /// это текущий уровень ручки газа для сохранения по нажатию ручки тормоза
volatile int current_level_speed=0;  /// это текущий уровень ручки газа для сохранения по возобновлению скорости
volatile int current_level_rpm=0;  /// это текущий уровень RPM по возобновлению скорости
volatile int croiuse_level=-1;
volatile int break_croiuse_level=-1;
volatile bool controllerBrake=false;   /// нажата ручка тормоза
volatile int currentSpeed=0;   /// это скорость для круиза
volatile int cr=-1;
//const int start_amper=14;   /// начальный ток
const int start_amper;   /// начальный erpm
volatile int current_amper=14;  /// текущий ток
int target_erpm=0; // текущий ERPM  для  шага скорости
volatile bool crouise_on=false; /// включен круиз
volatile bool isn=true; /// умный тормоз
volatile int count_telemtr=0; /// количество телеметрии

int numberCrouise=255;  
volatile bool stop_from_mobile=false;   /// нажата ручка тормоза
volatile bool break_long=false; /// статус долгого нажатия break

volatile int rpm = 0;
TickType_t wait_time = 2000 / portTICK_PERIOD_MS;
volatile float level_crouise[6] = {0,21.4,33.2,46.0,58.4,95.0};
volatile int rpm_crouise[6] = {2611,4700,7330,10115,12870,20800};
volatile int len_crouise=5; /// размер массива level_crouise минус 1
volatile bool speed_up=false;   /// Скорость нарастает
volatile bool addspeed=false;   /// это для передачи увеличения скорости с 0



#define BREAK_PIN 36 /// Это вход стоп сигнала
volatile int break_level=1;  ///не нажат тормоз

//SemaphoreHandle_t myMutexSpeed;
//static SemaphoreHandle_t task_semaphore = NULL;
static QueueHandle_t uart_queue;
//SemaphoreHandle_t addspeed_mutex;



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

void stop_Speed(bool status){                 
      if (status==true){  
        //printf("stop_Speed\n");    
        if (controllerBrake)return;
        controllerBrake=true;
        if (!isn){ /// это умный тормоз
        current_level=0;   /// по нажатию сохраняем текущий уровень и сбрасывает уровень газа
        current_rpm=0;        
        }
        //Serial.printf("current_level= %d\n", current_level);
        //Serial.printf("volt_bl = %d\n",volt_bl);   
        //Serial.println("");
        volt_bl=0;
        //key_trottle=0;
        numberCrouise=255;
        crouise_on=false;
        croiuse_level=-1;
        cr=-1;
        //if (current_level==0)cr=-1;
        getDuty(0.0);
        //dacWrite(analogOutPin, 0);
        //digitalWrite(BREAK_LIGTH, HIGH);   
       }
       //else controllerBrake=false;
}


void setCurrentLevel(){
    current_level_speed=volt_bl;
    current_level=volt_bl;
    current_level_rpm=rpm;   
}





// float getStep(bool forward){
//     //Serial.print("getStep()=");
//     float step = exp((rpm_crouise[len_crouise] - volt_bl) / 50.0); // Экспоненциальная функция    
//     //Serial.println(step);
//     if (step < 1) step = 1;
//     step=roundf(step * 10) / 10;
//     return step;
//  }


 /// получить круиз по текущим оборотам
 int get_level(bool forward){  
    cr=-1;
    //printf("get_level\n");
    //printf("numbercroise = %d\n",len_crouise); 
    if (forward && volt_bl>=rpm_crouise[len_crouise])return len_crouise;
    if (forward)for (int i = 0; i < len_crouise+1; i++  ){
      if (rpm_crouise[i]>volt_bl)return i; /// получили круиз с большими оборотами                                    }
    }
    else for(int i = len_crouise; i > -1; i--){
         if(volt_bl>rpm_crouise[i])return i;
    }  
    if (!forward)return -1;
    return 0;   /// ничего не нашло
}

/// это алгоритм управления по току
// void do_add_speed(void* parameter) {
//     const int MAX_CURRENT = 21;
//     const uint32_t DELAY_MS = 450;
//     const uint32_t CHECK_INTERVAL_MS = 450; // Интервал проверки
//     const uint32_t MAX_ATTEMPTS = 4;        // Количество интервалов
//     uint32_t attempt_count;
//     TickType_t last_check_time;

//     while (1) {
//         current_amper = start_amper;

//         bool local_addspeed;
//         if (xSemaphoreTake(addspeed_mutex, portMAX_DELAY) == pdTRUE) {
//             local_addspeed = addspeed; // Читаем addspeed безопасно
//             xSemaphoreGive(addspeed_mutex);
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
//             continue; // Пропускаем итерацию, если мьютекс недоступен
//         }

//         while (local_addspeed && current_amper <= MAX_CURRENT) {
//             attempt_count = 0;
//             last_check_time = xTaskGetTickCount();

//             //printf("do_add_speed\nvolt_bl=%d erpm=%d\n", 
//              //      volt_bl, rpm_controller);

//             while (break_level == 1 && 
//                    rpm_controller < volt_bl && 
//                    local_addspeed) {
                
//                 //printf("current_amper=%d\n", current_amper);

//                 TickType_t current_time = xTaskGetTickCount();
//                 if ((current_time - last_check_time) * portTICK_PERIOD_MS >= CHECK_INTERVAL_MS) {
//                     attempt_count++;
//                     last_check_time = current_time;

//                     if (attempt_count >= MAX_ATTEMPTS) {
//                         attempt_count = 0;
//                         current_amper++;
//                         //printf("volt_bl=%d\n", volt_bl);

//                         if (current_amper > MAX_CURRENT) {
//                             current_amper = start_amper;
//                             volt_bl = 0;
//                             local_addspeed = false;
//                             //printf("not_speed\nvolt_bl=%d\n", volt_bl);
//                             break;
//                         }
//                     }
//                 }
//                 vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
//             }

//             //printf("%s\n", local_addspeed ? "end_add_speed" : "addspeed=false");
//             if (local_addspeed) {
//                 //printf("break_level=%d erpm=%d\n",break_level, rpm_controller);
//             }

//             // Обновляем addspeed в глобальной области
//             if (xSemaphoreTake(addspeed_mutex, portMAX_DELAY) == pdTRUE) {
//                 addspeed = false; // Сбрасываем addspeed
//                 local_addspeed = addspeed; // Обновляем локальную копию для следующей проверки
//                 xSemaphoreGive(addspeed_mutex);
//             }

//             current_amper = start_amper; // Подготовка к следующей итерации
//         }
//         vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
//     }
//     vTaskDelete(NULL);
// }

// /// это алгоритм управления по  нарастанию оборотов
// void do_add_speed(void* parameter) {
//     const int MAX_CURRENT = 49;             // Максимальный индекс массива (0-49 = 50 значений)
//     const uint32_t DELAY_MS = 450;          // Задержка между итерациями
//     const uint32_t CHECK_INTERVAL_MS = 450; // Интервал проверки времени
//     const uint32_t MAX_ATTEMPTS = 4;        // Количество интервалов для шага
//     uint32_t attempt_count;                 // Счетчик неудачных попыток
//     TickType_t last_check_time;             // Время последней проверки

//     while (1) {
//         current_amper = 3;  // Начальный индекс массива (erpm_step[3] = 1248)

//         bool local_addspeed;
//         if (xSemaphoreTake(addspeed_mutex, portMAX_DELAY) == pdTRUE) {
//             local_addspeed = addspeed; // Безопасно читаем addspeed
//             xSemaphoreGive(addspeed_mutex);
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
//             continue; // Пропускаем итерацию, если мьютекс недоступен
//         }

//         while (local_addspeed && current_amper <= MAX_CURRENT) {
//             attempt_count = 0;
//             last_check_time = xTaskGetTickCount();

//             // Устанавливаем целевую скорость из массива по текущему индексу
//             target_erpm = erpm_step[current_amper]; // Начинаем с индекса 3

//             while (break_level == 1 && rpm_controller < volt_bl && local_addspeed) {
                
//                 // Проверяем, не достигла ли текущая скорость целевой отметки
//                 printf("rpm_controller=%d ", rpm_controller);
//                 printf("target_erpm=%d\n", target_erpm);
//                 if (rpm_controller < target_erpm) {
//                     attempt_count++; // Увеличиваем счетчик, если скорость не достигнута                    
//                 } else {
//                     attempt_count = 0; // Сбрасываем счетчик, если скорость достигнута и берем следующий шаг
//                     current_amper++;
//                     current_amper++;                    
//                     if (current_amper > MAX_CURRENT)current_amper=MAX_CURRENT;
//                     if (current_amper > MAX_CURRENT) {
//                         current_amper = 3; // Сбрасываем на начальный индекс 3
//                         volt_bl = 0;
//                         local_addspeed = false;
//                         break;
//                     }
//                     if (current_amper > MAX_CURRENT)current_amper=MAX_CURRENT;
//                     target_erpm = erpm_step[current_amper];
//                     if (target_erpm >= volt_bl)break;
//                 }
//                 TickType_t current_time = xTaskGetTickCount();
//                 if ((current_time - last_check_time) * portTICK_PERIOD_MS >= CHECK_INTERVAL_MS) {
//                     last_check_time = current_time;
//                     if (attempt_count >= MAX_ATTEMPTS) {
//                         attempt_count = 0;
//                         current_amper++; // Переходим к следующему шагу массива
//                         current_amper++;

//                         // Проверяем, превышает ли новая цель volt_bl
//                         if (current_amper > MAX_CURRENT) {
//                             current_amper = 3; // Сбрасываем на начальный индекс 3
//                             volt_bl = 0;
//                             local_addspeed = false;
//                             break;
//                         }
//                         target_erpm = erpm_step[current_amper]; // Обновляем целевую скорость
//                     }
//                 }
//                 vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
//             }

//             // Обновляем addspeed в глобальной области
//             if (xSemaphoreTake(addspeed_mutex, portMAX_DELAY) == pdTRUE) {
//                 addspeed = false; // Сбрасываем addspeed
//                 local_addspeed = addspeed; // Обновляем локальную копию
//                 xSemaphoreGive(addspeed_mutex);
//             }

//             //current_amper = 3; // Сбрасываем индекс для следующей итерации на 3
//         }
//         vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
//     }
//     vTaskDelete(NULL);
// }

// void select_level(int crouise){
//         if (volt_bl==0 || (crouise>-1 && crouise<(len_crouise+1))){
//             volt_bl=rpm_crouise[crouise];
//             currentSpeed=rpm_crouise[crouise];
//         }
//         controllerBrake=false;
// }


void select_level(int crouise){
    if (volt_bl==0 || (crouise>-1 && crouise<(len_crouise+1))){
        if (!speed_up) { /// скорость вниз
            //volt_bl=level_crouise[crouise];
            volt_bl=rpm_crouise[crouise];
            currentSpeed=rpm_crouise[crouise];
                       }
        else {  //// скорость вверх, делаем плавное нарастание
            //            volt_bl=rpm_crouise[crouise];
            //currentSpeed=rpm_crouise[crouise];
            volt_bl=rpm_crouise[crouise];
            // if (break_level==1){
            //     if (xSemaphoreTake(addspeed_mutex, portMAX_DELAY) == pdTRUE) {
            //           addspeed=true;  /// запускаем токовое нарастание скорости   //xTaskCreate(do_add_speed,"Do add speed", 10000, (void *)crouise, 1, NULL); 
            //           xSemaphoreGive(addspeed_mutex);
            //                                                                  }
            //                    }
        }               
    }
    controllerBrake=false;
}






float start_crouise(){  /// выставляет скорость по начальному круизу
       //Serial.println("start_crouise()");
        //Serial.println("rpm==0");
        croiuse_level=0;
        //crouise_on=true;
        select_level(croiuse_level);
        if (volt_bl<=start_level)volt_bl=rpm_crouise[0];
        //Serial.println(volt_bl);    
        //volt_bl=constrain(volt_bl, 0, 95);
        if (volt_bl>20800)volt_bl=20800.0;
        //dacWrite(analogOutPin, volt_bl);        
         getDuty(volt_bl);
        //Serial.println(volt_bl);     
        current_level=volt_bl;
        return volt_bl;
}


void AddSpeed(){
    //printf("AddSpeed\n");    
    //printf("volt_bl3=%d ",volt_bl);    
    //printf("break_level=%d ",break_level);    
    //printf("current_level_speed=%d ",current_level_speed); 
    //printf("start_level=%d\n",start_level);    
     if (break_level==1 && volt_bl<rpm_crouise[len_crouise]){
        //printf("break_level====");    
     if (volt_bl<=start_level || volt_bl==0) {
        volt_bl=start_crouise(); /// выставляет по начальному круизу скорость
     }    
     else {     
     //printf("volt_bl1=%d\n",volt_bl);    
     //printf("rpm2==\n");      
     volt_bl=getStep(true,volt_bl);
     if (volt_bl>rpm_crouise[len_crouise])volt_bl=rpm_crouise[len_crouise];
          }
    }     
    // if (croiuse_level==0 && volt_bl<rpm_crouise[croiuse_level])volt_bl=rpm_crouise[croiuse_level];
         //printf("setCurrentLevel==\n");      
         setCurrentLevel();  
}


void setCrouise(int crouise) {   /// устанавливаем сохране6ннывй в памяти контроллера круиз
         //printf("----------------  setCrouise --------------------------\n");  
        if (cr==crouise){
          //printf("----------------  Crouise  allready set --------------------------\n");            
          return;
        }
          crouise_on=true;
          cr=crouise;
          //printf("setCrouise\n");        
          numberCrouise=cr;
        //printf("numbercroise = %d\n",crouise);        
        //printf("numberCrouise = %d\n",numberCrouise);        
        select_level(crouise);
        setCurrentLevel();
}     





/// зажат тормоз
static void button_event_break(void *arg, void *data)
{
    //printf("button_event_break\n");
    break_long=false;
    break_level=0;
    gpio_set_level(BLUE_LED_PIN, 1);
    DoLight(128);
    controllerBrake=false;
    break_volt_bl=volt_bl;
    break_croiuse_level=croiuse_level;   
    addspeed=false;
    stop_Speed(true);
    setCurrentLevel();
}

/// break 3 сек
static void button_event_break_long(void *arg, void *data)
{
    //printf("=============================button_event_break_long======================================================================\n");
    break_long=true;
    addspeed=false;
    current_amper=start_amper;  /// сбрасываем в исходное состояние
}

/// отпущен тормоз
static void button_event_break_end(void *arg, void *data)
{
    //printf("=============================button_event_break_end======================================================================\n");
    gpio_set_level(BLUE_LED_PIN, 0);
    rpm_controller=0;
    if (!break_long){ /// не сработало долгое нажатие, 
         //printf("=============================not button_event_break_long_end======================================================================\n");    
         volt_bl=break_volt_bl;
         if (volt_bl>0){
         croiuse_level=get_level(true)-1;
         if (croiuse_level<1)croiuse_level=0;
         cr=-1;
         crouise_on=true;
         speed_up=true; /// скорость вверх
         //printf("croiuse_level1=%d\n",croiuse_level);
         break_level=1;
         volt_bl=0;                  
         setCrouise(croiuse_level);
         }
     }
     break_volt_bl=0;
     break_long=false;
     break_level=1;
     DoLight(0);
}

/// левая кнопка
static void button_event_cb1(void *arg, void *data)
{
    button_handle_t btn_handle = (button_handle_t)arg; // Приведение arg к button_handle_t
    button_event_t event = iot_button_get_event(btn_handle); // Получение типа события
    if (event < sizeof(button_event_names) / sizeof(button_event_names[0]) && button_event_names[event] != NULL) {

        const char *event_name =button_event_names[event];
        //ESP_LOGI(TAG, "Event: %s", event_name);

    if (strcmp(name_cont, "FT85BS") != 0)return;

    if (strcmp(event_name, "BUTTON_SINGLE_CLICK") == 0){
        //printf("Button1 single\n");
        if (croiuse_level==-1 && rpm==0){
            croiuse_level=0;
            crouise_on=true; 
            speed_up=true; /// скорость вверх
            setCrouise(croiuse_level);
        }
        else {
        AddSpeed();        
        currentSpeed=0;  /// не нужна работа
        }

    }    
    else if (strcmp(event_name,"BUTTON_PRESS_REPEAT")== 0){
        //printf("Button1 double\n");
        //printf("croiuse_level=%d\n",croiuse_level);
         if (croiuse_level<len_crouise){
            //printf("get_level\n");
             croiuse_level=get_level(true);  /// выставляем уровень круиза для текущего уровня набора скорости по одиночному или плавному нажатию кнопки
             if (croiuse_level<1)croiuse_level=0;
             if (croiuse_level==0)croiuse_level=1;
             crouise_on=true;
             speed_up=true; /// скорость вверх
             //printf("croiuse_level1=%d\n",croiuse_level);
             setCrouise(croiuse_level);
        }
       }    
    else if (strcmp(event_name,"BUTTON_LONG_PRESS_HOLD")== 0){
        //printf("Button1 LONG\n");
         AddSpeed();
    }    
    //else printf("None\n");
}
}

/// правая кнопка
static void button_event_cb2(void *arg, void *data)
{
    button_handle_t btn_handle = (button_handle_t)arg; // Приведение arg к button_handle_t
    button_event_t event = iot_button_get_event(btn_handle); // Получение типа события
    if (event < sizeof(button_event_names) / sizeof(button_event_names[0]) && button_event_names[event] != NULL) {
        const char *event_name =button_event_names[event];
        //ESP_LOGI(TAG, "Event: %s", event_name);   
    addspeed=false;   
    if (strcmp(event_name, "BUTTON_SINGLE_CLICK") == 0){
        //printf("Button2 single\n");
             crouise_on=false;
        /// уменьшаем уровень  
    if (break_level==1 && volt_bl>=start_level)volt_bl=getStep(false,volt_bl);;
      if (volt_bl<0)volt_bl=0;
    setCurrentLevel(); 
    }    
    else if (strcmp(event_name,"BUTTON_PRESS_REPEAT")== 0){
        //printf("Button2 double\n");         
         croiuse_level=get_level(false);
         if (croiuse_level<0){
            volt_bl=0;         
            setCurrentLevel();
         }   
         //printf("crouise_level=%d\n",croiuse_level);
    if (croiuse_level>len_crouise)croiuse_level=len_crouise;
     if (croiuse_level!=-1){
        crouise_on=true;
        speed_up=false;
        setCrouise(croiuse_level);

                            }   
        else stop_from_mobile=true;      
    }    
    else if (strcmp(event_name,"BUTTON_LONG_PRESS_HOLD")== 0){
        //printf("Button2 LONG\n");
         crouise_on=false;   
         if (break_level==1 && volt_bl>=start_level)volt_bl=getStep(false,volt_bl);;
         if (volt_bl<0)volt_bl=0;
            setCurrentLevel(); 
    }    
    //else printf("None\n");
}
    //printf("Button2 event %s\n", button_event_table[(button_event_t)data]);
    //esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
}


void button_init_break(uint32_t button_num)
{
    button_config_t btn_cfg = {
        .long_press_time = 3000,  // Задержка 3 секунды для long press
        .short_press_time = 50,   // Короткое нажатие (не используем, но задаём)
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



void button_init1(uint32_t button_num)
{
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
    //ret |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, NULL, button_event_cb1, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, NULL, button_event_cb1, NULL);
    ESP_ERROR_CHECK(ret);
}



void button_init2(uint32_t button_num)
{
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
    //ret |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, NULL, button_event_cb2, NULL);
    ret |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_HOLD, NULL, button_event_cb2, NULL);
    ESP_ERROR_CHECK(ret);
}


void init_button(){
    button_init1(BUTTON_NUM1);
    button_init2(BUTTON_NUM2);
    button_init_break(BREAK_PIN);
}



uint8_t* processData(uint8_t* target_bytes, uint8_t* read_buff, int buff_size, int offset) {
    //printf("processData");
    int index = -1;
    for (int i = 0; i < buff_size; i++) {
        if (read_buff[i] == target_bytes[0] && read_buff[i+1] == target_bytes[1]) {
            index = i;
            break;
        }
    }
    //printf("index=%d\n", index);
    if (index != -1) {
        uint8_t* new_buff = (uint8_t*) malloc(buff_size - offset);
        memcpy(new_buff, read_buff, index);
        memcpy(new_buff + index, read_buff + index + offset, buff_size - index - offset);
        for (int i = 0; i < buff_size - offset; i++) {
            //printf("%d\n",new_buff[i])              
            read_buff[i] = new_buff[i];
        }
        free(new_buff);  // Освобождаем new_buff
    }
    
    return read_buff;  // Возвращаем обновленный read_buff 
} 


void printBuffer(uint8_t *buffer, uint8_t bufferSize) {
                printf("Прочитанные данные: ");
                printf("bytes_available=%d\n", bufferSize);
                 for (int i = 0; i < bufferSize; ++i) {
                      printf("%d,", buffer[i]); // Преобразование uint8_t в символ
                     }
                      printf("\n");
}    


void mybuffer_to_hex(uint8_t *buffer, uint8_t bufferSize) {
    // Выделяем память под строку: 2 символа на байт + 1 для '\0'
    char *hex_str = (char *)malloc((bufferSize * 2) + 1);
    if (hex_str == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for hex string");
    }

    // Преобразование каждого байта в два hex-символа
    for (uint8_t i = 0; i < bufferSize; i++) {
        sprintf(hex_str + (i * 2), "%02X ", buffer[i]);
    }

    // Добавляем завершающий нулевой символ
    hex_str[bufferSize * 2] = '\0';

    printf(hex_str);
    printf("\n");
}

void sendBufferToController(uint8_t *buffer, uint8_t bufferSize) {
    //mybuffer_to_hex(buffer, bufferSize);

    uart_write_bytes(uart_num, (const char*)buffer, bufferSize);
    uart_wait_tx_done(uart_num, wait_time);
}


// int readBufferfromController() {
//     //printf("Читаю данные\n");
//     int bytes_available=0;
//      while (1) {
//         bytes_available = uart_read_bytes(uart_num, read_buff, BUF_SIZE-1, wait_time);
//         if (bytes_available > 0) {
//             //printBuffer(read_buff,bytes_available);          
//             break; // Выход из цикла, если данные получены

//         }
//         vTaskDelay(wait_time); // Задержка перед следующей попыткой чтения
//     }   
//     // printf("возвращаю данные=%d\n", bytes_available);
//     //                 for (int i = 0; i < bytes_available-1; ++i) {
//     //                  printf(" %d",read_buff[i]); // Преобразование uint8_t в символ
//     //                 }
//     //                   printf("\n");
//     return bytes_available;
// }

                  

// void process_data(const uint8_t *data, int length) {
//     // Ваш код для обработки данных
//     // Например, вывод данных или другие действия
//     int volt=0;
//     int rp=0;
//     bool find=false;
//     //printf("Данные: ");
//     for (int i = 0; i < length; ++i) {
//         //printf("%02X ", data[i]);
//         if (i==3){  /// напруга  
//            //printf("%d ", volt);
//            if (volt<5500){
//            find=true; 
//            count_summ+=1;
//            Vbatt+=volt;
//            }
//         }       
//         if (i==17 && find){ /// обороты
//             rp+=(read_buff[i] << 8 | read_buff[i+1]);
//             //printf("%d\n", rp);
//             rpm_controller+=rp;

//         }
//         //printf("%d ", data[i]);
//     }
//     //printf("\n");
// }

// void find_data_strings(const uint8_t *read_buff, int buff_size) {
//     int start_index = 0;
//     while (start_index < buff_size) {
//         // Поиск начала пакета данных (байт=170)
//         while (start_index < buff_size && read_buff[start_index] != 170) {
//             ++start_index;
//         }

//         if (start_index >= buff_size) {
//             break; // Начало пакета данных не найдено
//         }

//         // Выделение длины данных
//         int data_length = read_buff[start_index + 1];

//         // Проверка, что данные полностью в буфере
//         if (start_index + 2 + data_length <= buff_size) {
//             process_data(&read_buff[start_index + 2], data_length);
//         }

//         // Переход к следующему пакету данных
//         start_index += 2 + data_length;
//     }
// }


// Задача обработки UART
void uart_task(void *arg) {
    uart_event_t event;
    uint8_t *rx_data = (uint8_t *)malloc(BUF_SIZE);
    size_t rx_length = 0;
    uint8_t start_bytes[] = {170, 12};

    // Отправляем начальный буфер
    //uart_write_bytes(uart_num, (const char *)start_buffer, sizeof(start_buffer));
    //ESP_LOGI(TAG, "Sent start_buffer: %d bytes", sizeof(start_buffer));

    while (1) {
        // Ожидаем события UART
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    // Читаем принятые данные
                    rx_length = uart_read_bytes(uart_num, rx_data, event.size, RX_TIMEOUT);
                    if (rx_length > 0) {
                        //ESP_LOGI("UART", "Received %d bytes:", rx_length); 
                        //printBuffer(rx_data,rx_length);                        
                        if (operation==0){                            
                        processData(start_bytes, rx_data, rx_length, 6);                        
                        //printBuffer(read_buff,sizeof(read_buff));
                        for (int i = 0; i < 6; ++i)name_cont[i] = (char)rx_data[i];                     
                        name_cont[7] = '\0'; // Установка завершающего нулевого символа
                          if (strcmp(name_cont, "FT85BS") == 0) {
                               ESP_LOGI("UART","Connected to controller\n");
                               gpio_set_level(POWER_PIN, 1);                        
                               operation=1;
                               wait_time = 50 / portTICK_PERIOD_MS;
                               count_telemtr=0;
                                        //xTaskCreatePinnedToCore(do_add_speed,"Do add speed",10000  ,NULL,10,&TaskSpeed,0);
                                                                }
                                        }
                        else if (operation==1) {  /// чтение телеметрии
                            //printBuffer(rx_data,rx_length);
                            if (rx_length==34){
                                if (rx_data[0]==170 && rx_data[1]==29 && rx_data[33]==221){ /// проверка на корректные данные 
                                          /// 5,6 напряжение                                          
                                          uint16_t result = (rx_data[5] << 8) | rx_data[6];
                                          Vbatt=result;
                                          //printf("Volt=%d\n",res);
                                          result = (rx_data[17] << 8) | rx_data[18];
                                          if ((int)result<30000)rpm_controller=result;
                                          //printf("=================ERPM===========%d\n",rpm_controller);                                         
                                          if (Vbatt<3000){ /// батарея выключена
                                            printf("=================Vbatt===========%.2f\n",Vbatt);
                                            for (int i = 0; i < 6; ++i)name_cont[i] =0;
                                            gpio_set_level(POWER_PIN, 0);
                                          }
                                          //17 - 18 erpm
                                }
                               }
                        }                
                    }
                    break;

                case UART_FIFO_OVF:
                    ESP_LOGE("UART", "FIFO Overflow");
                    uart_flush_input(uart_num);
                    xQueueReset(uart_queue);
                    break;

                case UART_BUFFER_FULL:
                    ESP_LOGE("UART", "Buffer Full");
                    uart_flush_input(uart_num);
                    xQueueReset(uart_queue);
                    break;

                case UART_BREAK:
                    ESP_LOGI("UART", "Break detected");
                    break;

                default:
                    ESP_LOGI("UART", "Other event type: %d", event.type);
                    break;
            }
        }
    }
    //ESP_LOGI("UART", "exit uart_task");
    free(rx_data);
    vTaskDelete(NULL);
}




// Задача для отправки и получения данных
void loop_controller(void* parameter) {
    printf("Start loop_controller\n");
    int my_voltbl=0;
    uint8_t start_buffer[] = {170, 1, 17, 76, 127, 221};
    //uint8_t data4[] = {170, 4, 26, 1, 0, 100, 23, 87, 221};
    uint8_t telemetr_data[]={0xaa,0x04,0x1a,0x01,0x00,0x64,0x17,0x57,0xdd};
    uint8_t stop_data[] = {170, 5, 4, 0, 0, 0, 0, 192, 213, 221};
    //uint8_t start_target_bytes[] = {170, 8};
    //uint8_t start_bytes[] = {170, 12};
    //int bytes_available = 0;
    //uint8_t start_amper_data[] ={170, 5, 4, 0, 0,7,208,108,214,221};  /// это старт токовым управлением на 2А
    //uint8_t read_buff_tmp[512];
   
    
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    while(1) {
        //printf("current_level=%d ",current_level); 
        //printf("volt_bl=%.2f\n",volt_bl); 
       if (strcmp(name_cont, "FT85BS") != 0) {
            printf("Start data\n");
            wait_time = 2000 / portTICK_PERIOD_MS;
            operation=0;
            sendBufferToController(start_buffer, sizeof(start_buffer));
        } else {  //// соединение создано
            wait_time = 50 / portTICK_PERIOD_MS;
        //printf("my_voltbl=%.2f ",my_voltbl); 
        //printf("volt_bl=%.2f\n",volt_bl);              
             if (operation==1){ /// это телеметрия
                //ESP_LOGI("UART", "================================get telemetria=============================");
                if (count_telemtr==0 || count_telemtr==4 || count_telemtr==8 || count_telemtr==12 || count_telemtr==16)sendBufferToController(telemetr_data, sizeof(telemetr_data));
                else {                  
             if (break_level==0){                
                my_voltbl=0;
                sendBufferToController(stop_data, sizeof(stop_data));
             }   
             else {
                if(volt_bl==0 && current_level==0) getDuty(0.0);  /// это нажат тормоз                
                else if (addspeed==true){   /// начальное токовое управление
                    //ESP_LOGI("UART", "===== amper ====%d\n",current_amper);  
                    //getAmper(current_amper);
                    getSpeed(target_erpm); 
                    my_voltbl=0;



                    sendBufferToController(speed_data, sizeof(speed_data)); 


                    
                    croiuse_level=get_level(true)-1;
                    if (croiuse_level<1)croiuse_level=0;
                }
                else {   /// это не токовое управление
                       //ESP_LOGI("UART", "----volt_bl==0 && current_level>0------\n");    
                       //ESP_LOGI("UART", "volt_bl=%d ",volt_bl);
                       //ESP_LOGI("UART", "current_level=%d\n",current_level);    
                       //ESP_LOGI("UART", "my_voltbl=%d\n",my_voltbl);  
                     if(volt_bl==0 && current_level>0)volt_bl=current_level;
                         if (volt_bl>20800)volt_bl=20800;
                             setCurrentLevel();                                
                     
               if (my_voltbl!=volt_bl) {            
                   my_voltbl=volt_bl;
                   //ESP_LOGI("UART", "++++++++++++++++++++++++++++++++++++++++++++++-------------getSpeed--------+++++++++++++++++++++++++++++\n");   
                   //ESP_LOGI("UART", "volt_bl=%d\n",volt_bl);   
                   getSpeed(volt_bl);      
                  }    
                //ESP_LOGI("UART", "++++++++++++++++++++++++++++++++++++++++++++++ sendBufferToController  -+++++++++++++++++++++++++++++\n");      
                //mybuffer_to_hex(speed_data, sizeof(speed_data));



                sendBufferToController(speed_data, sizeof(speed_data));        




                }
             }     
            }
                      }
                      count_telemtr++;
                      if (count_telemtr>19){count_telemtr=0;
                        //ESP_LOGI("UART", "----------------------------------------------------------------------\n");                      
                      }
        }
        vTaskDelay(wait_time); 
        //vTaskDelay(1000); // Задержка в 300 мс
    }
}


// void delete_speed(){
//      controllerBrake=false;
//      volt_bl=0;
//      numberCrouise=255;
//      cr=-1;
//      getDuty(0.0);
// }

static void configure_led(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BLUE_LED_PIN),  // Маска для указания пина
        .mode = GPIO_MODE_OUTPUT,                // Режим вывода
        .pull_up_en = GPIO_PULLUP_DISABLE,       // Отключение подтяжки вверх
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   // Отключение подтяжки вниз
        .intr_type = GPIO_INTR_DISABLE           // Отключение прерываний
    };
    gpio_config(&io_conf);
}



void app_main(void)
{
    //addspeed_mutex = xSemaphoreCreateMutex();
    init_button(); /// инициализируем кнопки
    //myMutexSpeed = xSemaphoreCreateMutex();
    //task_semaphore = xSemaphoreCreateBinary();

    configure_led();
    getDuty(0.0);
      
    initBreakLight();
    //DoLight(128);

   // power_save_init();
    // Настройка параметров UART

    // Настройка POWER_PIN как выход
    gpio_set_direction(POWER_PIN, GPIO_MODE_OUTPUT);

    // Установка POWER_PIN в низкий уровень
    gpio_set_level(POWER_PIN, 0);
    //gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);


    uart_config_t uart_config = {
        .baud_rate = 115200, // Скорость передачи данных
        .data_bits = UART_DATA_8_BITS, // Количество бит данных
        .parity = UART_PARITY_DISABLE, // Без контроля четности
        .stop_bits = UART_STOP_BITS_1, // Один стоп-бит
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Без аппаратного управления потоком
        .rx_flow_ctrl_thresh = 122,
    };

//  int intr_alloc_flags = 0;

// #if CONFIG_UART_ISR_IN_IRAM
//     intr_alloc_flags = ESP_INTR_FLAG_IRAM;
// #endif

    // Конфигурация UART с заданными параметрами
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Установка пинов UART
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Установка драйвера UART
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart_queue, 0));

    ESP_ERROR_CHECK(uart_enable_rx_intr(uart_num));
    // Использование драйвера UART
    //uart_vfs_dev_use_driver(uart_num);
    
  
    printf("Start prog\n");
    xTaskCreatePinnedToCore(loop_controller,"Loop_controller",4096  ,NULL,10,&Task11,1);
    //xTaskCreatePinnedToCore(loop_controller,"Loop_controller",2048  ,NULL,10,&Task11,1);


}


 
