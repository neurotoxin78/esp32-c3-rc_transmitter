// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.2.3
// LVGL version: 8.3.4
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_events.h"
extern lv_obj_t * ui_MainScreen;
extern lv_obj_t * ui_StatusBar;
extern lv_obj_t * ui_SignalStrength;
extern lv_obj_t * ui_netImage;
extern lv_obj_t * ui_heartImage;
extern lv_obj_t * ui_memLabel;
extern lv_obj_t * ui_servoAngle;
extern lv_obj_t * ui_speedLabel;
extern lv_obj_t * ui____initial_actions0;


LV_IMG_DECLARE(ui_img_net_d_png);    // assets/net_d.png
LV_IMG_DECLARE(ui_img_heart_orange_png);    // assets/heart_orange.png
LV_IMG_DECLARE(ui_img_heart_black_png);    // assets/heart_black.png
LV_IMG_DECLARE(ui_img_heart_white_png);    // assets/heart_white.png
LV_IMG_DECLARE(ui_img_net_none_png);    // assets/net_none.png
LV_IMG_DECLARE(ui_img_net_receive_png);    // assets/net_receive.png
LV_IMG_DECLARE(ui_img_net_send_png);    // assets/net_send.png
LV_IMG_DECLARE(ui_img_16x16_blank_png);    // assets/16x16_blank.png




void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
