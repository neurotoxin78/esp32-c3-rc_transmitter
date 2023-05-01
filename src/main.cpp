#include "main.h"
#include "sntp.h"
#include "tasks.h"
#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <lvgl.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include "ui/ui.h"
#include "network.h"

static const char *TAG = "rc_transmitter";
static const char *ssid = WIFI_SSID;
static const char *password = WIFI_PASSWORD;
String hostname = "rc-controller";
TimerHandle_t wifiReconnectTimer;

static const uint16_t screenWidth = SCREEN_WIDTH;
static const uint16_t screenHeight = SCREEN_HEIGHT;
static hw_timer_t *lv_tick_timer = NULL;
static lv_group_t *main_group = NULL;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];
static lv_indev_t *indev_keypad;

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */


void setup()
{
  Serial.begin(115200);
  init_gpio();
  // Wi-Fi
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  connectToWifi();
  delay(500);

  // LVGL init
  lv_init();

  lv_tick_timer = timerBegin(0, 80, true);

  timerAttachInterrupt(lv_tick_timer, &onTick, true);
  timerAlarmWrite(lv_tick_timer, 5000, true);
  timerAlarmEnable(lv_tick_timer);

  tft.begin();        /* TFT init */
  tft.setRotation(3); /* Landscape orientation, flipped */

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);
  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  // Create the custom indev driver
  static lv_indev_drv_t indev_drv_keypad;
  lv_indev_drv_init(&indev_drv_keypad);
  indev_drv_keypad.type = LV_INDEV_TYPE_KEYPAD;
  indev_drv_keypad.read_cb = indev_keypad_read;

  // Register the custom indev driver
  indev_keypad = lv_indev_drv_register(&indev_drv_keypad);
  ui_init();
  main_group = lv_group_create();
  lv_group_add_obj(main_group, ui_MainScreen);
  lv_indev_set_group(indev_keypad, main_group);
  lv_group_set_default(main_group);
  lv_indev_enable(indev_keypad, true);

  createTasks();
  if (WiFi.status() != WL_CONNECTED)
  {
    udpConnect(addr, port);
  }
  // Start UDP Listener
  udp.listen(port);
  udp.onPacket(parsePacket);
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
}

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection, reconnect...");
    lv_img_set_src(ui_netImage, &ui_img_net_d_png);
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void connectToWifi()
{
  WiFi.setHostname(hostname.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

static void init_gpio()
{
  pinMode(GPIO_UP_PIN, INPUT_PULLUP);
  // pinMode(GPIO_RT_PIN, INPUT_PULLDOWN);
  pinMode(GPIO_DN_PIN, INPUT_PULLUP);
  pinMode(GPIO_LT_PIN, INPUT_PULLUP);
  pinMode(GPIO_CR_PIN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(BTNL_PIN, INPUT_PULLDOWN);
  pinMode(BTNR_PIN, INPUT_PULLDOWN);
}

static void indev_keypad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  // Read input from the GPIO keys
  int up_state = digitalRead(GPIO_UP_PIN);
  int dn_state = digitalRead(GPIO_DN_PIN);
  int lt_state = digitalRead(GPIO_LT_PIN);
  // int rt_state = digitalRead(GPIO_RT_PIN);
  int cr_state = digitalRead(GPIO_CR_PIN);

  // Create an LVGL input data structure
  data->state = LV_INDEV_STATE_REL;
  if (dn_state == LOW)
  {
    data->key = LV_KEY_PREV;
    data->state = LV_INDEV_STATE_PR;
    // Serial.println("DOWN");
  }
  else if (up_state == LOW)
  {
    data->key = LV_KEY_NEXT;
    data->state = LV_INDEV_STATE_PR;
    // Serial.println("UP");
  }
  else if (cr_state == LOW)
  {
    data->key = LV_KEY_ENTER;
    data->state = LV_INDEV_STATE_PR;
    // Serial.println("ENTER");
  }
  else if (lt_state == LOW)
  {
    data->key = LV_KEY_ESC;
    data->state = LV_INDEV_STATE_PR;
    // Serial.println("LEFT");
  } // else if(rt_state == HIGH) {
    // data->key = LV_KEY_RIGHT;
    // data->state = LV_INDEV_STATE_PR;
    // Serial.println("RIGHT");
  //}
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

void IRAM_ATTR onTick()
{
  lv_tick_inc(5);
}