#include <Arduino.h>
#include <avr/io.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#define STACK_SIZE 200
#define PIN_LED6 6
#define PIN_LED7 7
#define PIN_LED8 8
#define PIN_LED9 9
#define PIN_PTTM 0
#define PIN_0 0
#define PIN_1 1
#define PIN_SPKR 10
#define PIN_MTR 11

// TONES  ==========================================
#define  lvl0     100
#define  lvl1     600
#define  lvl2     1100
#define  lvl3     1600
// Define a special note, 'R', to represent a rest
#define  lvl0     0

// SPEEDS  =========================================
#define  splvl0     130
#define  splvl1     180
#define  splvl2     210
#define  splvl3     255


struct values {
	int desired;
	int current;
	int distance;
};

QueueHandle_t xQueueSpeed = xQueueCreate(5, sizeof(int));
QueueHandle_t xQueueOutput = xQueueCreate(5, sizeof(int));
unsigned long debounce_time = 150;
volatile unsigned long last_millisDeBounce0;
volatile unsigned long last_millisDeBounce1;
int speed_setting = 3;
int current_speed = 0;
int current_distance;

int up = 1;
int down = 2;
int note = lvl0;

SemaphoreHandle_t xSemaphore = xSemaphoreCreateMutex();

int minimum(int a, int b) {
	if (a > b) {
		return b;
	} else {
		return a;
	}
}

void setLEDs(int speed, int speedsetting, int currentdistance) {
	if (speed == 0) {
		digitalWrite(PIN_LED6, LOW);
		digitalWrite(PIN_LED7, LOW);
		digitalWrite(PIN_LED8, LOW);
		tone(PIN_SPKR, lvl0);
		analogWrite(PIN_MTR, splvl0);
		//Serial.println("0 OK");
	} else if (speed == 1) {
		digitalWrite(PIN_LED6, HIGH);
		digitalWrite(PIN_LED7, LOW);
		digitalWrite(PIN_LED8, LOW);
		tone(PIN_SPKR, lvl1);
		analogWrite(PIN_MTR, splvl1);
	} else if (speed == 2) {
		digitalWrite(PIN_LED6, HIGH);
		digitalWrite(PIN_LED7, HIGH);
		digitalWrite(PIN_LED8, LOW);
		tone(PIN_SPKR, lvl2);
		analogWrite(PIN_MTR, splvl2);
	} else if (speed == 3) {
		digitalWrite(PIN_LED6, HIGH);
		digitalWrite(PIN_LED7, HIGH);
		digitalWrite(PIN_LED8, HIGH);
		tone(PIN_SPKR, lvl3);
		analogWrite(PIN_MTR, splvl3);
	}
	if (speedsetting > currentdistance) {
		digitalWrite(PIN_LED9, HIGH);
	} else {
		digitalWrite(PIN_LED9, LOW);
	}
}

void taskReadDistance(void *p) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 500;

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE) {
			int distance = analogRead(PIN_PTTM);
			if (distance < 200) {
				current_distance = 0;
			} else if (distance < 400) {
				current_distance = 1;
			} else if (distance < 600) {
				current_distance = 2;
			} else {
				current_distance = 3;
			}
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}
	}
}

void taskSetSpeed(void *p) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE) {
			current_speed = minimum(speed_setting, current_distance);
			setLEDs(current_speed, speed_setting, current_distance);
			xQueueSendToBack(xQueueOutput, (void * ) &current_speed,
					(TickType_t ) 10);
			xQueueSendToBack(xQueueOutput, (void * ) &speed_setting,
					(TickType_t ) 10);
			xQueueSendToBack(xQueueOutput, (void * ) &current_distance,
					(TickType_t ) 10);
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}
	}
}

void taskPrint(void *p) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();
	while (1) {
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE) {
			void *current;
			void *desired;
			void *distance;
			if (xQueueOutput != 0) {
				if (xQueueReceive(xQueueOutput, &(current), (TickType_t ) 10)) {
					if (xQueueReceive(xQueueOutput, &(desired),
							(TickType_t ) 10)) {
						if (xQueueReceive(xQueueOutput, &(distance),
								(TickType_t ) 10)) {
							Serial.print("C: ");
							Serial.println((int) current);
							Serial.print("De: ");
							Serial.println((int) desired);
							Serial.print("DI: ");
							Serial.println((int) distance);
						}
					}
				}
			}
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}
	}
}

void taskUpdateSpeedSetting(void *p) {
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000;

	xLastWakeTime = xTaskGetTickCount();
	while (1) {
		if ( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE) {
			int *change;
			if (xQueueSpeed != 0) {
				if (xQueueReceive(xQueueSpeed, &(change), (TickType_t ) 10)) {

					if ((int) change == 1 && speed_setting < 3) {
						speed_setting++;
					} else if ((int) change == 2 && speed_setting > 0) {
						speed_setting--;
					}
				}
			}
			xSemaphoreGive(xSemaphore);
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}
	}
}

void int0ISR() {
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if ((unsigned long) (millis() - last_millisDeBounce0) >= debounce_time) {
		xQueueSendToBackFromISR(xQueueSpeed, &up, &xHigherPriorityTaskWoken);
		last_millisDeBounce0 = millis();
	}
	if (xHigherPriorityTaskWoken == pdTRUE) {
		taskYIELD();
	}
}

void int1ISR() {
	static BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if ((unsigned long) (millis() - last_millisDeBounce0) >= debounce_time) {
		xQueueSendToBackFromISR(xQueueSpeed, &down, &xHigherPriorityTaskWoken);
		last_millisDeBounce0 = millis();
	}
	if (xHigherPriorityTaskWoken == pdTRUE) {
		taskYIELD();
	}
}

void setup() {
	pinMode(PIN_LED6, OUTPUT);
	pinMode(PIN_LED7, OUTPUT);
	pinMode(PIN_LED8, OUTPUT);
	pinMode(PIN_LED9, OUTPUT);
	pinMode(PIN_SPKR, OUTPUT);
	pinMode(PIN_MTR, OUTPUT);
	attachInterrupt(PIN_0, int0ISR, FALLING);
	attachInterrupt(PIN_1, int1ISR, FALLING);
	Serial.begin(115200);
	xSemaphoreGive(xSemaphore);
}
void loop() {
	/* create two tasks one with higher priority than the other */
	xTaskCreate(taskReadDistance, "TaskReadDistance", STACK_SIZE,
			(void * ) NULL, 4, NULL);
	xTaskCreate(taskSetSpeed, "TaskSetSpeed", STACK_SIZE, (void * ) NULL, 2,
			NULL);
	xTaskCreate(taskUpdateSpeedSetting, "TaskUpdateSpeedSetting", STACK_SIZE,
			(void * ) NULL, 3, NULL);
	xTaskCreate(taskPrint, "TaskPrint", STACK_SIZE, (void * ) NULL, 1, NULL);
	/* start scheduler */
	vTaskStartScheduler();
}
