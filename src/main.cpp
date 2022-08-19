#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <Stepper.h>
#include "queue.h"

#define LED_1               8
#define LED_2               9
#define ANALOG              A0

#define MOTOR_PIN_1         2
#define MOTOR_PIN_2         3
#define MOTOR_PIN_3         4
#define MOTOR_PIN_4         5
#define STEPS               200
#define SPEED               60

#define TIME_1              300
#define TIME_2              207

#define BAUD                115200

#define VAL_TOTAL_DEGREE    2040UL
#define TOTAL_DEGREE        360UL

Stepper motor (STEPS, MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4);
QueueHandle_t readQueue;

void configPins();
void enablePins(int otpt);
void disablePins(int otpt);

uint32_t calcDegree(uint16_t degree);
inline void runTask();
void ledTask_1(void * pvParameters);
void ledTask_2(void * pvParameters);
void stepper_controller(void * pvParameters);
void sendDegree(void * pvParameters);

void setup() {
  readQueue = xQueueCreate(1, sizeof(uint32_t));
  Serial.begin(BAUD);
  configPins();
  motor.setSpeed(SPEED);
  runTask();
  Serial.println("runTask()");

}

void loop() {
}

void enablePins(int otpt) {
  digitalWrite(otpt, HIGH);
}

void disablePins(int otpt) {
  digitalWrite(otpt, LOW);
}

void configPins() {
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
}

inline void runTask() {
  xTaskCreate(ledTask_2, "led_2_task", 64, NULL, 1, NULL );
  xTaskCreate(ledTask_1, "led_1_task", 64, NULL, 1, NULL );
  xTaskCreate(stepper_controller, "stepper_task", 128, NULL, 1, NULL );
  xTaskCreate(sendDegree, "analog read", 128, NULL, 1 , NULL);
}

void ledTask_1(void * pvParameters) {
  while (1) {
    enablePins(LED_1);
    vTaskDelay( TIME_2 / portTICK_PERIOD_MS );
    disablePins(LED_1);
    vTaskDelay( TIME_2 / portTICK_PERIOD_MS );
  }
}

void ledTask_2(void * pvParameters) {
  while (1) {
    enablePins(LED_2);
    vTaskDelay( TIME_1 / portTICK_PERIOD_MS );
    disablePins(LED_2);
    vTaskDelay( TIME_1 / portTICK_PERIOD_MS );
  }
}

void stepper_controller(void * pvParameters) {
  uint32_t receive_value = 0;
  while (1) {
    if (xQueueReceive(readQueue, &receive_value, 0) == pdPASS) {
      motor.step(receive_value);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void sendDegree(void * pvParameters) {
  uint32_t read_val;
  static uint32_t value_to_send;
  // Serial.println("Entrou na sendDegree");
  while (1) {
    if (Serial.available() > 0) {
      read_val = Serial.read();
      value_to_send = calcDegree(read_val);
      xQueueSend(readQueue, &value_to_send, portMAX_DELAY);
      Serial.println(value_to_send);
      Serial.flush();
      read_val=0;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

uint32_t calcDegree(uint16_t degree) {
  return ((VAL_TOTAL_DEGREE * degree) / TOTAL_DEGREE );
}