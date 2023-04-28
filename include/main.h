#include <Arduino.h>
#include "WiFi.h"
#include <lvgl.h>

#define WIFI_SSID "Neurotoxin2"
#define WIFI_PASSWORD "Mxbb2Col"

#define SCREEN_WIDTH 160
#define SCREEN_HEIGHT 80

#define LED GPIO_NUM_12

#define GPIO_UP_PIN 8
#define GPIO_RT_PIN 9
#define GPIO_DN_PIN 13
#define GPIO_LT_PIN 5
#define GPIO_CR_PIN 4

extern HardwareSerial Serial;

void WiFiEvent(WiFiEvent_t event);
void connectToWifi();
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static void indev_keypad_read(lv_indev_drv_t *drv, lv_indev_data_t *data);
void IRAM_ATTR onTick();
static void init_gpio();