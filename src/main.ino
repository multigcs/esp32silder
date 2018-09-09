
#include <Wire.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <RotaryEncoder.h>
#include "BluetoothSerial.h"
#include "SSD1306Wire.h"

#define X_MAX 4000

#define PIN_DIR_X 23
#define PIN_STEP_X 16
#define PIN_MS1_X 19
#define PIN_MS2_X 34
#define PIN_EN_X 18

#define PIN_DIR_Y 35
#define PIN_STEP_Y 32
#define PIN_SW_X 17
#define PIN_SW_Y 5

#define PIN_SERVO_Z 33
#define PIN_SERVO_F 15

#define PIN_SHUTTER 2

#define PIN_BATT A0

#define PIN_ENC_A 26
#define PIN_ENC_B 25
#define PIN_ENC_SW 27

hw_timer_t * timer = NULL;
TaskHandle_t xTask1;
TaskHandle_t xTask2;

RotaryEncoder encoder(PIN_ENC_A, PIN_ENC_B);
AccelStepper stepperX(1, PIN_STEP_X, PIN_DIR_X);
AccelStepper stepperY(1, PIN_STEP_Y, PIN_DIR_Y);
Servo servoZ;
Servo servoF;
BluetoothSerial SerialBT;
SSD1306Wire display(0x3c, 21, 22);

int set_posX = 0;
int set_speedX = 3000;
int set_accelX = 500;
int set_posY = 0;
int set_speedY = 3000;
int set_accelY = 500;
int set_posZ = 0;
int set_posF = 0;
int lockX = 0;
int lockY = 0;

int batt = 0;
int diff = 20;
int sdelay = 2000;
int start_pos = 0;
int end_pos= X_MAX;
int dorun = 0;
int dozero = 1;
int part = 0;
int encoder_click = 0;

// menu
enum {
	MENU_INT,
	MENU_UINT,
	MENU_BOOL,
};
typedef struct {
	char text[20];
	uint8_t type;
	uint8_t steps;
	uint8_t updated;
	void *value;
	int last;
	int min;
	int max;
} MENU;
#define MENU_N 12
#define MENU_DE 3

volatile int menu_s = 0;
volatile int menu_n = 1;
MENU menu[MENU_N] = {
	{"RUN", MENU_BOOL, 1, 1, &dorun, 0, 0, 1},
	{"Difference", MENU_INT, 10, 1, &diff, 0, 1, X_MAX},
	{"Delay", MENU_INT, 10, 1, &sdelay, 0, 0, 30000},
	{"Start-Position", MENU_INT, 100, 1, &start_pos, 0, 0, X_MAX},
	{"End-Position", MENU_INT, 100, 1, &end_pos, 0, 0, X_MAX},
	{"Speed", MENU_INT, 100, 1, &set_speedX, 0, 1, 4000},
	{"Acceleration", MENU_INT, 10, 1, &set_accelX, 0, 1, 400},
	{"Position-Lock", MENU_BOOL, 1, 1, &lockX, 0, 0, 1},
	{"Servo 1", MENU_INT, 1, 1, &set_posZ, 0, 0, 180},
	{"Servo 2", MENU_INT, 1, 1, &set_posF, 0, 0, 180},
	{"X-Position", MENU_UINT, 1, 1, &set_posX, 0, -18, 18},
	{"Zero all Axis", MENU_BOOL, 1, 1, &dozero, 0, 0, 1},
};


void IRAM_ATTR isr_encoder() {
	encoder.tick();
}


void draw_menu() {
	int n = 0;
	int m_y = 25;
	char tmpstr[10];
	int ms = 0;
	if (menu_s == 0) {
		menu_n = encoder.getPosition();
		if (menu_n >= MENU_N) {
			menu_n = 0;
			encoder.setPosition(menu_n);
		} else if (menu_n < 0) {
			menu_n = MENU_N - 1;
			encoder.setPosition(menu_n);
		}
	} else {
		int *value = (int*)menu[menu_n].value;
		*value = encoder.getPosition() * menu[menu_n].steps;
		if (*value < menu[menu_n].min) {
			*value = menu[menu_n].min;
		} else if (*value > menu[menu_n].max) {
			*value = menu[menu_n].max;
		}
		if (menu[menu_n].last != *value) {
			menu[menu_n].updated = 1;
			menu[menu_n].last = *value;
		}
	}
	if (digitalRead(PIN_ENC_SW) == 0) {
		encoder_click = 1;
	}
	if (encoder_click == 1) {
		if (menu_s == 0) {
			int *value = (int*)menu[menu_n].value;
			if (menu[menu_n].type == MENU_BOOL) {
				*value = 1 - *value;
				menu[menu_n].updated = 1;
				menu[menu_n].last = *value;
			} else {
				menu_s = 1;
				encoder.setPosition(*value / menu[menu_n].steps);
			}
		} else {
			menu_s = 0;
			encoder.setPosition(menu_n);
			// on exit menu
			if (menu_n == 10) {
				set_posX = 0;
			}
		}
		delay(300);
		encoder_click = 0;
	}
	if (menu_n < (MENU_DE - 1) / 2) {
		ms = 0;
	} else if (menu_n > MENU_N - (MENU_DE + 1) / 2) {
		ms = MENU_N - MENU_DE;
	} else {
		ms = menu_n - (MENU_DE - 1) / 2;
	}
	display.clear();
	if (dorun == 1) {
		display.setFont(ArialMT_Plain_16);
		display.drawString(0, 0, "RUNNING");
	} else if (dozero == 1) {
		display.setFont(ArialMT_Plain_16);
		display.drawString(0, 0, "ZERO AXIS");
	} else {
		display.setFont(ArialMT_Plain_16);
		display.drawString(0, 0, "mXm-Slider");
	}
	// draw position-bar
	int cpos = stepperX.currentPosition();
	int lpos = cpos * 128 / X_MAX;
	display.drawHorizontalLine(0, 20, 128);
	display.drawVerticalLine(lpos - 1, 19, 3);
	display.drawVerticalLine(lpos, 19, 3);
	display.drawVerticalLine(lpos + 1, 19, 3);
	display.setFont(ArialMT_Plain_10);
	sprintf(tmpstr, "%0.1fV", (float)batt / 10.0);
	display.drawString(105, 3, tmpstr);
	for (n = ms; n < MENU_N; n++) {
		int *value = (int *)menu[n].value;
		switch(menu[n].type) {
			case MENU_BOOL: {
				if ((int)*value == 1) {
					sprintf(tmpstr, "YES\n");
				} else {
					sprintf(tmpstr, "NO\n");
				}
				break;
			};
			default: {
				sprintf(tmpstr, "%i\n", (int)*value);
			};
		}
		if (menu_s == 0) {
			display.setFont(ArialMT_Plain_10);
			if (menu_n == n) {
				display.drawString(0, m_y, ">");
			}
			display.drawString(15, m_y, menu[n].text);
			display.drawString(90, m_y, tmpstr);
			m_y += 12;
		} else if (menu_n == n) {
			display.setFont(ArialMT_Plain_16);
			display.drawString(10, 23, menu[n].text);
			display.setFont(ArialMT_Plain_24);
			display.drawString(70, 40, tmpstr);
		}
	}
	stepperX.setMaxSpeed(set_speedX);
	stepperX.setAcceleration(set_accelX);
	stepperY.setMaxSpeed(set_speedY);
	stepperY.setAcceleration(set_accelY);
	servoZ.write(set_posZ);
	servoF.write(set_posF);
	display.display();
}


void gotozeroX() {
	part = 0;
	dorun = 0;
	// force move out of sw
	digitalWrite(PIN_DIR_X, HIGH);
	while (digitalRead(PIN_SW_X) == 0) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(4000);
	}
	// move to sw
	digitalWrite(PIN_DIR_X, LOW);
	while (digitalRead(PIN_SW_X) == 1) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(4000);
	}
	// move slowly out of sw
	digitalWrite(PIN_DIR_X, HIGH);
	while (digitalRead(PIN_SW_X) == 0) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(20000);
	}
	// move slowly to zero pos
	int n = 0;
	for (n = 0; n < 20; n++) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(20000);
	}
	if (lockX == 0) {
		digitalWrite(PIN_EN_X, HIGH);
	}
	stepperX.setCurrentPosition(0);
}


void gotozeroY() {
	return;
	part = 0;
	dorun = 0;
	digitalWrite(PIN_EN_X, LOW);
	// force move out of sw
	digitalWrite(PIN_DIR_Y, HIGH);
	while (digitalRead(PIN_SW_Y) == 0) {
		digitalWrite(PIN_STEP_Y, HIGH);
		digitalWrite(PIN_STEP_Y, LOW);
		delayMicroseconds(4000);
	}
	// move to sw
	digitalWrite(PIN_DIR_Y, LOW);
	while (digitalRead(PIN_SW_Y) == 1) {
		digitalWrite(PIN_STEP_Y, HIGH);
		digitalWrite(PIN_STEP_Y, LOW);
		delayMicroseconds(4000);
	}
	// move slowly out of sw
	digitalWrite(PIN_DIR_Y, HIGH);
	while (digitalRead(PIN_SW_Y) == 0) {
		digitalWrite(PIN_STEP_Y, HIGH);
		digitalWrite(PIN_STEP_Y, LOW);
		delayMicroseconds(20000);
	}
	// move slowly to zero pos
	int n = 0;
	for (n = 0; n < 20; n++) {
		digitalWrite(PIN_STEP_Y, HIGH);
		digitalWrite(PIN_STEP_Y, LOW);
		delayMicroseconds(20000);
	}
	stepperY.setCurrentPosition(0);
}


void menuTask( void * parameter ) {
	int mn = 0;
	while (1) {
		int n = 0;
		int d = 300;
		char tmp_str[100];
		for (n = 0; n < MENU_N; n++) {
			if (menu[n].updated != 0) {
				menu[n].updated = 0;
				mn = n;
				d = 100;
				if (n == 0) {
					// start run
					if (dorun == 1) {
						part = 0;
						digitalWrite(PIN_EN_X, LOW);
						stepperX.moveTo(start_pos + (part * diff));
						if (start_pos + (part * diff) > end_pos) {
							dorun = 0;
						}
					}
				}
				if (n == 7) {
					// change lock
					if (lockX == 1) {
						digitalWrite(PIN_EN_X, LOW);
					} else {
						digitalWrite(PIN_EN_X, HIGH);
					}
				}
			}
		}
		int *value = (int *)menu[mn].value;
		sprintf(tmp_str, "%i;%i;%i;%s;%i\n", mn, MENU_N, menu[mn].type, menu[mn].text, (int)*value);
		SerialBT.print(tmp_str);
		mn++;
		if (mn >= MENU_N) {
			mn = 0;
			delay(1000);
		}
		delay(d);
	}
}


void setup() {
	// serial & bluetooth
	Serial.begin(115200);
	Serial.println("welcome to the mXm-Slider");
	SerialBT.begin("mXm-Slider");

	// display
	display.init();
	display.flipScreenVertically();
	display.clear();
	display.setFont(ArialMT_Plain_24);
	display.drawString(0, 0, "mXm-Slider");
	display.display();
	display.setFont(ArialMT_Plain_16);

	// pins
	pinMode(PIN_SHUTTER, OUTPUT);
	digitalWrite(PIN_SHUTTER, LOW);
	pinMode(PIN_MS1_X, OUTPUT);
	digitalWrite(PIN_MS1_X, HIGH);
	pinMode(PIN_MS2_X, OUTPUT);
	digitalWrite(PIN_MS2_X, LOW);
	pinMode(PIN_EN_X, OUTPUT);
	digitalWrite(PIN_EN_X, HIGH);
	pinMode(PIN_SW_X, INPUT_PULLUP);

	// encoder
	attachInterrupt(PIN_ENC_A, isr_encoder, CHANGE);
	attachInterrupt(PIN_ENC_B, isr_encoder, CHANGE);
	pinMode(PIN_ENC_SW, INPUT_PULLUP);

	// stepper & servo
	stepperX.setMaxSpeed(set_speedX);
	stepperX.setAcceleration(set_accelX);
	stepperY.setMaxSpeed(set_speedY);
	stepperY.setAcceleration(set_accelY);
	servoZ.attach(PIN_SERVO_Z);
	servoF.attach(PIN_SERVO_F);

	xTaskCreatePinnedToCore(
		menuTask,           /* Task function. */
		"menuTask",        /* name of task. */
		10000,                    /* Stack size of task */
		NULL,                     /* parameter of the task */
		1,                        /* priority of the task */
		&xTask1,                /* Task handle to keep track of created task */
	1);                    /* pin task to core 0 */
	xTaskCreatePinnedToCore(
		stepperTask,           /* Task function. */
		"stepperTask",        /* name of task. */
		10000,                    /* Stack size of task */
		NULL,                     /* parameter of the task */
		1,                        /* priority of the task */
		&xTask2,            /* Task handle to keep track of created task */
	0);                 /* pin task to core 1 */


	timer = timerBegin(0, 80, true);
	timerAttachInterrupt(timer, &onTimer, true);
	timerAlarmWrite(timer, 1000, true);
	timerAlarmEnable(timer);
}


void IRAM_ATTR onTimer() {
	if (menu_n == 10 && menu_s == 1) {
		digitalWrite(PIN_EN_X, LOW);
		if (set_posX < 0) {
			digitalWrite(PIN_DIR_X, HIGH);
			digitalWrite(PIN_STEP_X, HIGH);
			digitalWrite(PIN_STEP_X, LOW);
			int newpos = stepperX.currentPosition() + 1;
			stepperX.setCurrentPosition(newpos);
			stepperX.moveTo(newpos);           
			if (newpos >= X_MAX) {
				set_posX = 0;
				encoder.setPosition(set_posX);
			} else if (X_MAX - newpos < set_posX * 2) {
				set_posX = (X_MAX - newpos) / 2;
				encoder.setPosition(set_posX);
			}
		} else if (set_posX > 0) {
			digitalWrite(PIN_DIR_X, LOW);
			digitalWrite(PIN_STEP_X, HIGH);
			digitalWrite(PIN_STEP_X, LOW);
			int newpos = stepperX.currentPosition() - 1;
			stepperX.setCurrentPosition(newpos);
			stepperX.moveTo(newpos);
			if (newpos <= 0) {
				set_posX = 0;
				encoder.setPosition(set_posX);
			} else if (newpos < set_posX * 2) {
				set_posX = newpos / 2;
				encoder.setPosition(set_posX);
			}
		}
		timerAlarmWrite(timer, 20000 - abs(set_posX) * 1000, true);
	} else {
		timerAlarmWrite(timer, 1000, true);
	}
}


void stepperTask( void * parameter ) {
	while (1) {
		if (dozero == 1) {
			set_posZ = 0;
			set_posF = 0;
			servoZ.write(set_posZ);
			servoF.write(set_posF);
			gotozeroX();
			gotozeroY();
			dorun = 0;
			dozero = 0;
		}
		if (dorun == 1) {
			if (stepperX.distanceToGo() == 0) {
				if (lockX == 0) {
					digitalWrite(PIN_EN_X, HIGH);
				}
				delay(sdelay / 2);
				digitalWrite(PIN_SHUTTER, HIGH);
				delay(100);
				digitalWrite(PIN_SHUTTER, LOW);
				delay(sdelay / 2 - 100);
				part++;
				digitalWrite(PIN_EN_X, LOW);
				stepperX.moveTo(start_pos + (part * diff));
				if (start_pos + (part * diff) > end_pos) {
					dorun = 0;
				}
			}
			stepperX.run();
		} else {
			if (menu_n == 10 && menu_s == 1) {
			} else {
				stepperX.run();
				if (stepperX.distanceToGo() == 0) {
					if (lockX == 0) {
						digitalWrite(PIN_EN_X, HIGH);
					}
				}
			}
		}
	}
}



void loop() {
	batt = ((analogRead(PIN_BATT) * 33 * 3.3 / 4096) + batt) / 2;
	// user-interface
	char c = 0;
	if (SerialBT.available()) {
		c = SerialBT.read();
		if (c == 's') {
			int id = SerialBT.parseInt();
			int val = SerialBT.parseInt();
			if (id >= 0 && id < MENU_N) {
				int *value = (int*)menu[id].value;
				*value = encoder.getPosition() * menu[id].steps;
				*value = val;
				if (menu[id].last != *value) {
					menu[id].updated = 1;
					menu[id].last = *value;
				}
			}
		}
	}
	draw_menu();
}


