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

uint64_t chipid;

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

void getInfo(){
    chipid = ESP.getEfuseMac();
    Serial.printf("ESP32 Chip ID = %04X", (uint16_t)(chipid >> 32));
    Serial.printf("%08X\n", (uint32_t)chipid);
    Serial.printf("ChipRevision %d, Cpu Freq %d, SDK Version %s\n", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getSdkVersion());
    Serial.println();
    Serial.printf("Flash Size %d, Flash Speed %d\n", ESP.getFlashChipSize(), ESP.getFlashChipSpeed());
    Serial.println();
    Serial.print("Free heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" - Max block: ");
    Serial.print(ESP.getMaxAllocHeap());
}

void heartbeat_process(void *parameter)
{
  uint32_t freeHeap;
  char buffer[100];
  for (;;)
  {
    digitalWrite(LED, HIGH);
    lv_img_set_src(ui_heartImage, &ui_img_heart_orange_png);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    lv_img_set_src(ui_heartImage, &ui_img_16x16_blank_png);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, HIGH);
    lv_img_set_src(ui_heartImage, &ui_img_heart_orange_png);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(LED, LOW);
    lv_img_set_src(ui_heartImage, &ui_img_16x16_blank_png);
    vTaskDelay(1900 / portTICK_PERIOD_MS);
    freeHeap = ESP.getFreeHeap() / 1024;
    snprintf(buffer, sizeof(buffer), "%dkB", freeHeap);
    lv_label_set_text(ui_memLabel, buffer);

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
      // Serial.println("BTNL");
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
