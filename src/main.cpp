#include <Arduino.h>
#include "arduino_freertos.h"
#include "avr/pgmspace.h"

#include "queue.h"
#include "semphr.h"

QueueHandle_t xQueue;
SemaphoreHandle_t uart_mutex;


#define WDT 33
#define EBS_PRESSURE_PIN A13
#define IGN_PIN 34
#define R2D_PIN 35



void Task1()
{
  int i = 0;
  while (1)
  {
    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE)
      Serial.println("Task1");
    xSemaphoreGive(uart_mutex);
    xQueueSend(xQueue, &i, portMAX_DELAY);
    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE)
    {
      i++;
      Serial.print("Current i: ");
      Serial.println(i);
      xSemaphoreGive(uart_mutex);
    }
     vTaskDelay(200);
  }
}

void Task2(void *pvParameters)
{

  while (1)
  {
    digitalWrite(WDT, arduino::HIGH);
    vTaskDelay(500);
    digitalWrite(WDT, arduino::LOW);
    vTaskDelay(500);
  }
}

void Task3(void *pvParameters)
{
  int i = 0;
  while (1)
  {
    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE)
      Serial.println("Task3");
    xSemaphoreGive(uart_mutex);
    xQueueReceive(xQueue, &i, portMAX_DELAY);
    if (xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE)
      Serial.println(i);
    xSemaphoreGive(uart_mutex);

    vTaskDelay(200);
  }
}

void task4(void *pvParameters)
{
  pinMode(34, arduino::OUTPUT);
  while (1)
  {
    digitalWrite(34, arduino::HIGH);
    vTaskDelay(50);
    digitalWrite(34, arduino::LOW);
    vTaskDelay(50);
  }
}

void setup()
{
   Serial.begin(9600);
  pinMode(36, arduino::OUTPUT);
  pinMode(WDT, arduino::OUTPUT);
  pinMode(IGN_PIN, arduino::INPUT);
  pinMode(R2D_PIN, arduino::INPUT);

  /***** Start of initial sequence *****/
  unsigned long test_time = millis();
  for(int i = 0; i < 4; i++)
  {
  digitalWrite(WDT, arduino::HIGH);
  test_time = millis();
  while (millis() - test_time < 500);
  digitalWrite(WDT, arduino::LOW);
  test_time = millis();
  while (millis() - test_time < 500);
  }

  while (millis() - test_time < 2000);
  

  //later check if sdc opened
  //if it didnt open send failure to can and stop here
  int digital_tank_pressure = analogRead(EBS_PRESSURE_PIN);
  Serial.println(digital_tank_pressure);
  //check if pressure is in range

  while (R2D_PIN == 0);
  test_time = millis();
  while(millis() - test_time < 2500);
  /***** End of initial sequence *****/
  

  xQueue = xQueueCreate(1, sizeof(int));
  uart_mutex = xSemaphoreCreateMutex();
 
  Serial.println("Setup done");
  xTaskCreate(Task1, "Task1", 100, NULL, 3, NULL);
  xTaskCreate(Task2, "Task2", 100, NULL, 0, NULL);
  xTaskCreate(Task3, "Task3", 100, NULL, 1, NULL);
  xTaskCreate(task4, "Task4", 100, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop()
{
  vTaskDelete(NULL);
}
