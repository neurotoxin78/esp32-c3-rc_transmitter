#include "main.h"
#include <Arduino.h>
#include <WiFi.h>
#include "tasks.h"
#include <lvgl.h>
#include "ui/ui.h"
#include <Joystick.h>
#include <AxisJoystick.h>
#include "network.h"

Joystick *joystic;

AsyncUDPMessage udp_command;
// lv_img_set_src(ui_JoyImage, &ui_img_joy_normal_png);

TaskHandle_t heartbeat_Handler;
TaskHandle_t joystic_Handler;
TaskHandle_t buttons_Handler;
TaskHandle_t con_strength_Handler;
TaskHandle_t con_info_Handler;

void CheckJoy(const Joystick::Move move)
{
  switch (move)
  {
  // case Joystick::Move::NOT:
  //   Serial.println("NOT");
  case Joystick::Move::PRESS:
    sendPacket("Press", addr, port);
    break;
  case Joystick::Move::UP:
    lv_img_set_src(ui_netImage, &ui_img_net_send_png);
    sendPacket("UP", addr, port);
    break;
  case Joystick::Move::DOWN:
    lv_img_set_src(ui_netImage, &ui_img_net_send_png);
    sendPacket("DOWN", addr, port);
    break;
  case Joystick::Move::RIGHT:
    lv_img_set_src(ui_netImage, &ui_img_net_send_png);
    sendPacket("RIGHT", addr, port);
    break;
  case Joystick::Move::LEFT:
    lv_img_set_src(ui_netImage, &ui_img_net_send_png);
    sendPacket("LEFT", addr, port);
    break;
  default:
    lv_img_set_src(ui_netImage, &ui_img_net_none_png);
    // Serial.print(".");
  }
}

void heartbeat_process(void *parameter)
{

  for (;;)
  {
    digitalWrite(LED, HIGH);
    lv_img_set_src(ui_heartImage, &ui_img_heart_white_png);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    lv_img_set_src(ui_heartImage, &ui_img_heart_black_png);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, HIGH);
    lv_img_set_src(ui_heartImage, &ui_img_heart_white_png);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    lv_img_set_src(ui_heartImage, &ui_img_heart_black_png);
    vTaskDelay(2900 / portTICK_PERIOD_MS);
  }
}

void buttons_Task(void *parameter)
{
  int btnl_state;
  int btnr_state;
  for (;;)
  {
    btnl_state = digitalRead(BTNL_PIN);
    btnr_state = digitalRead(BTNR_PIN);
    if (btnl_state == HIGH)
    {
      //Serial.println("BTNL");
      sendPacket("BTNL", addr, port);
    }
    if (btnr_state == LOW)
    {
      Serial.println("BTNR");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void joystic_Task(void *parameter)
{
  joystic = new AxisJoystick(SW_PIN, VRX_PIN, VRY_PIN);
  joystic->calibrate(ESP32_ADC_MIN, ESP32_ADC_MAX, AXES_DEVIATION);

  for (;;)
  {
    CheckJoy(joystic->multipleRead());

    /* Serial.print("| SingleRead: " + moveTitle(joystic->singleRead()));
    Serial.print(" | MultipleRead: " + moveTitle(joystic->multipleRead()));
    Serial.print(" | Press: " + String(joystic->isPress()));
    Serial.print(" | Up: " + String(joystic->isUp()));
    Serial.print(" | Down: " + String(joystic->isDown()));
    Serial.print(" | Right: " + String(joystic->isRight()));
    Serial.print(" | Left: " + String(joystic->isLeft()));
    Serial.print(" | VRx: " + String(joystic->readVRx()));
    Serial.print(" | VRy: " + String(joystic->readVRy()));
    Serial.print(" | SW: " + String(joystic->readSW()) + " |\r");
    */
    vTaskDelay(10);
  }
}

void conInfo_Task(void *parameter)
{
  char buffer[100];
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      sprintf(buffer, "%s RSSI: %d dBm", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      lv_label_set_text(ui_StatusBar, buffer);
    }
    else
    {
      lv_label_set_text(ui_StatusBar, "Disconnected");
    }
    vTaskDelay(1000);
  }
}

int getStrength()
{
  long rssi = WiFi.RSSI();
  long averageRSSI = 0;

  averageRSSI = 2 * (rssi + 100);
  return averageRSSI;
}

void conStrength_Task(void *parameter)
{
  char buffer[100];
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      lv_bar_set_value(ui_SignalStrength, getStrength(), LV_ANIM_OFF);
    }
    vTaskDelay(250);
  }
}

void createTasks(void)
{
  xTaskCreatePinnedToCore(heartbeat_process, "HEARTBEAT", 2000, NULL, 1, &heartbeat_Handler, 0);
  xTaskCreatePinnedToCore(joystic_Task, "Joystic_Task", 2000, NULL, 1, &joystic_Handler, 0);
  xTaskCreatePinnedToCore(buttons_Task, "Buttond_Task", 2000, NULL, 1, &buttons_Handler, 0);
  xTaskCreatePinnedToCore(conInfo_Task, "Connection_Info", 2000, NULL, 1, &con_info_Handler, 0);
  xTaskCreatePinnedToCore(conStrength_Task, "Connection_Info", 2000, NULL, 1, &con_strength_Handler, 0);
}
