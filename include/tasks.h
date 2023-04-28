#include "Arduino.h"

#define VRX_PIN GPIO_NUM_0 // ESP32 pin GIOP36 (ADC0) connected to VRX pin
#define VRY_PIN GPIO_NUM_1 // ESP32 pin GIOP39 (ADC0) connected to VRY pin
#define SW_PIN GPIO_NUM_12
#define ESP32_ADC_MIN 0
#define ESP32_ADC_MAX 4095
#define AXES_DEVIATION 100

void createTasks(void);
void heartbeat_process(void *parameter);
void joystic_Task(void *parameter);
void conInfo_Task(void *parameter);
void conStrength_Task(void *parameter);
int getStrength();

