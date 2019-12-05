
#include <Wire.h>
#include <RotaryEncoder.h>
#include <SSD1306Wire.h>
/*
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
*/

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

#define PIN_SHUTTER 2

#define PIN_BATT A0

#define PIN_ENC_A 26
#define PIN_ENC_B 25
#define PIN_ENC_SW 27

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTER_UUID_SETUP "beb5483e-36e1-4688-b7f5-ea07361b2601"
#define CHARACTER_UUID_DATA  "beb5483e-36e1-4688-b7f5-ea07361b2601"
#define CHARACTER_UUID_CTRL  "beb5483e-36e1-4688-b7f5-ea07361b2602"

/*
bool BLEdeviceConnected = false;
BLECharacteristic *pCharacteristic_setup;
BLECharacteristic *pCharacteristic_data;
BLECharacteristic *pCharacteristic_ctrl;
*/

hw_timer_t * timer = NULL;
TaskHandle_t xTask1;
TaskHandle_t xTask2;

RotaryEncoder encoder(PIN_ENC_A, PIN_ENC_B);
SSD1306Wire display(0x3c, 21, 22);

int batt = 0;
int part = 0;
int encoder_click = 0;

int set_accelX = 500;
int set_posY = 0;
int set_accelY = 500;

volatile int stage = 0;
volatile int position = 0;
volatile int gotopos = 0;

enum {
	STAGE_GOTO,
	STAGE_WAIT,
	STAGE_EXPOSE,
	STAGE_DELAY,
	STAGE_LAST
};

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
	int16_t value;
	int16_t last;
	int16_t min;
	int16_t max;
} MENU;
#define MENU_DE 3

enum {
	M_RUN,
	M_DIFF,
	M_DELAY,
	M_WAIT,
	M_EXPOS,
	M_START,
	M_END,
	M_SPEED,
	M_ACCEL,
	M_POSLOCK,
	M_XPOS,
	M_ZERO,
	M_LAST
};

volatile int menu_s = 0;
volatile int menu_n = 1;
volatile MENU menu[M_LAST] = {
	{"RUN", MENU_BOOL, 1, 1, 0, 0, 0, 1},
	{"Difference", MENU_INT, 10, 1, 20, 0, 1, X_MAX},
	{"Delay", MENU_INT, 100, 1, 1000, 0, 0, 30000},
	{"Wait", MENU_INT, 10, 0, 300, 0, 0, 1000},
	{"Exposure", MENU_INT, 100, 0, 100, 0, 10, 30000},
	{"Start-Position", MENU_INT, 100, 1, 0, 0, 0, X_MAX},
	{"End-Position", MENU_INT, 100, 1, X_MAX, 0, 0, X_MAX},
	{"Speed", MENU_INT, 5, 1, 1, 0, 1, 100},
	{"Acceleration", MENU_INT, 10, 1, 500, 0, 1, 400},
	{"Position-Lock", MENU_BOOL, 1, 1, 0, 0, 0, 1},
	{"X-Position", MENU_UINT, 1, 1, 0, 0, -18, 18},
	{"Zero all Axis", MENU_BOOL, 1, 1, 1, 0, 0, 1},
};

/*
class MyServerCallbacks: public BLEServerCallbacks {
	void onConnect(BLEServer* pServer) {
		BLEdeviceConnected = true;
	};
	void onDisconnect(BLEServer* pServer) {
		BLEdeviceConnected = false;
	}
};

class MyCallbacksSetup: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
//		std::string value = pCharacteristic->getValue();
	}
};

class MyCallbacksCtrl: public BLECharacteristicCallbacks {
	void onWrite(BLECharacteristic *pCharacteristic) {
//		std::string value = pCharacteristic->getValue();
		menu[M_RUN].value = 1 - menu[M_RUN].value;
	}
};
*/

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
		if (menu_n >= M_LAST) {
			menu_n = 0;
			encoder.setPosition(menu_n);
		} else if (menu_n < 0) {
			menu_n = M_LAST - 1;
			encoder.setPosition(menu_n);
		}
	} else {
		menu[menu_n].value = encoder.getPosition() * menu[menu_n].steps;
		if (menu[menu_n].value < menu[menu_n].min) {
			menu[menu_n].value = menu[menu_n].min;
			encoder.setPosition(menu[menu_n].value / menu[menu_n].steps);
		} else if (menu[menu_n].value > menu[menu_n].max) {
			menu[menu_n].value = menu[menu_n].max;
			encoder.setPosition(menu[menu_n].value / menu[menu_n].steps);
		}
		if (menu[menu_n].last != menu[menu_n].value) {
			menu[menu_n].updated = 1;
			menu[menu_n].last = menu[menu_n].value;
			if (menu[M_DELAY].value - menu[M_WAIT].value - menu[M_EXPOS].value < 0) {
				menu[M_DELAY].value = menu[M_WAIT].value + menu[M_EXPOS].value;
				encoder.setPosition(menu[menu_n].value / menu[menu_n].steps);
			}
		}
	}
	if (digitalRead(PIN_ENC_SW) == 0) {
		encoder_click = 1;
	}
	if (encoder_click == 1) {
		if (menu_s == 0) {
			if (menu[menu_n].type == MENU_BOOL) {
				menu[menu_n].value = 1 - menu[menu_n].value;
				menu[menu_n].updated = 1;
				menu[menu_n].last = menu[menu_n].value;
			} else {
				menu_s = 1;
				encoder.setPosition(menu[menu_n].value / menu[menu_n].steps);
			}
		} else {
			menu_s = 0;
			encoder.setPosition(menu_n);
			menu[M_XPOS].value = 0;
		}
		delay(300);
		encoder_click = 0;
	}
	if (menu_n < (MENU_DE - 1) / 2) {
		ms = 0;
	} else if (menu_n > M_LAST - (MENU_DE + 1) / 2) {
		ms = M_LAST - MENU_DE;
	} else {
		ms = menu_n - (MENU_DE - 1) / 2;
	}
	display.clear();
	if (menu[M_RUN].value == 1) {
		display.setFont(ArialMT_Plain_16);
		display.drawString(0, 0, "RUNNING");
	} else if (menu[M_ZERO].value == 1) {
		display.setFont(ArialMT_Plain_16);
		display.drawString(0, 0, "ZERO AXIS");
	} else {
		display.setFont(ArialMT_Plain_16);
		display.drawString(0, 0, "mXm-Slider");
	}
	// draw position-bar
	int cpos = position;
	int lpos = cpos * 128 / X_MAX;
	display.drawHorizontalLine(0, 20, 128);
	display.drawVerticalLine(lpos - 1, 19, 3);
	display.drawVerticalLine(lpos, 19, 3);
	display.drawVerticalLine(lpos + 1, 19, 3);
	display.setFont(ArialMT_Plain_10);
	sprintf(tmpstr, "%0.1fV", (float)batt / 10.0);
	display.drawString(105, 3, tmpstr);
	for (n = ms; n < M_LAST; n++) {
		switch(menu[n].type) {
			case MENU_BOOL: {
				if (menu[n].value == 1) {
					sprintf(tmpstr, "YES\n");
				} else {
					sprintf(tmpstr, "NO\n");
				}
				break;
			};
			default: {
				sprintf(tmpstr, "%i\n", menu[n].value);
			};
		}
		if (menu_s == 0) {
			display.setFont(ArialMT_Plain_10);
			if (menu_n == n) {
				display.drawString(0, m_y, ">");
			}
			display.drawString(15, m_y, (char *)menu[n].text);
			display.drawString(90, m_y, tmpstr);
			m_y += 12;
		} else if (menu_n == n) {
			display.setFont(ArialMT_Plain_16);
			display.drawString(10, 23, (char *)menu[n].text);
			display.setFont(ArialMT_Plain_24);
			display.drawString(70, 40, tmpstr);
		}
	}
	display.display();
}


void gotozeroX() {
	part = 0;
	menu[M_RUN].value = 0;
	// force move out of sw
	digitalWrite(PIN_DIR_X, HIGH);
	while (digitalRead(PIN_SW_X) == 0) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		delayMicroseconds(100);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(4000);
	}
	// move to sw
	digitalWrite(PIN_DIR_X, LOW);
	while (digitalRead(PIN_SW_X) == 1) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		delayMicroseconds(100);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(4000);
	}
	// move slowly out of sw
	digitalWrite(PIN_DIR_X, HIGH);
	while (digitalRead(PIN_SW_X) == 0) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		delayMicroseconds(100);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(20000);
	}
	// move slowly to zero pos
	int n = 0;
	for (n = 0; n < 20; n++) {
		digitalWrite(PIN_EN_X, LOW);
		digitalWrite(PIN_STEP_X, HIGH);
		delayMicroseconds(100);
		digitalWrite(PIN_STEP_X, LOW);
		delayMicroseconds(20000);
	}
	if (menu[M_POSLOCK].value == 0) {
		digitalWrite(PIN_EN_X, HIGH);
	}
	position = 0;
}



void IRAM_ATTR onTimer() {
	if (menu_n == M_XPOS && menu_s == 1) {
		timerAlarmWrite(timer, 20000 - abs(menu[M_XPOS].value) * 1000, true);
		digitalWrite(PIN_EN_X, LOW);
		if (menu[M_XPOS].value < 0) {
			digitalWrite(PIN_DIR_X, HIGH);
			digitalWrite(PIN_STEP_X, HIGH);
			digitalWrite(PIN_STEP_X, LOW);
			position++;
			if (position >= X_MAX) {
				menu[M_XPOS].value = 0;
				encoder.setPosition(menu[M_XPOS].value);
			} else if (X_MAX - position < menu[M_XPOS].value * 2) {
				menu[M_XPOS].value = (X_MAX - position) / 2;
				encoder.setPosition(menu[M_XPOS].value);
			}
		} else if (menu[M_XPOS].value > 0) {
			digitalWrite(PIN_DIR_X, LOW);
			digitalWrite(PIN_STEP_X, HIGH);
			digitalWrite(PIN_STEP_X, LOW);
			position--;
			if (position <= 0) {
				menu[M_XPOS].value = 0;
				encoder.setPosition(menu[M_XPOS].value);
			} else if (position < menu[M_XPOS].value * 2) {
				menu[M_XPOS].value = position / 2;
				encoder.setPosition(menu[M_XPOS].value);
			}
		}
	} else if (menu[M_RUN].value == 1) {
			if (stage == STAGE_GOTO) {
				if (position < gotopos) {
					timerAlarmWrite(timer, 10000 - menu[M_SPEED].value * 8000 / 100, true);
					digitalWrite(PIN_DIR_X, HIGH);
					digitalWrite(PIN_STEP_X, HIGH);
					digitalWrite(PIN_STEP_X, LOW);
					position++;
				} else if (position > gotopos) {
					timerAlarmWrite(timer, 10000 - menu[M_SPEED].value * 8000 / 100, true);
					digitalWrite(PIN_DIR_X, LOW);
					digitalWrite(PIN_STEP_X, HIGH);
					digitalWrite(PIN_STEP_X, LOW);
					position--;
				} else {
					timerAlarmWrite(timer, menu[M_WAIT].value * 1000, true);
					stage = STAGE_WAIT;
				}
			} else if (stage == STAGE_WAIT) {
				timerAlarmWrite(timer, menu[M_EXPOS].value * 1000, true);
				digitalWrite(PIN_SHUTTER, HIGH);
				stage = STAGE_EXPOSE;
			} else if (stage == STAGE_EXPOSE) {
				timerAlarmWrite(timer, (menu[M_DELAY].value - menu[M_WAIT].value - menu[M_EXPOS].value) * 1000, true);
				digitalWrite(PIN_SHUTTER, LOW);
				gotopos = menu[M_START].value + (part * menu[M_DIFF].value);
				part++;
				digitalWrite(PIN_EN_X, LOW);
				stage = STAGE_GOTO;
			}
	} else{
		timerAlarmWrite(timer, 100000, true);
	}
}




void setup() {
	Serial.begin(115200);
	Serial.println("welcome to the mXm-Slider");


/*
	// bluetooth
	BLEDevice::init("mXm-Slider");
	BLEServer *pServer = BLEDevice::createServer();
	pServer->setCallbacks(new MyServerCallbacks());
	BLEService *pService = pServer->createService(SERVICE_UUID);

	pCharacteristic_setup = pService->createCharacteristic(CHARACTER_UUID_SETUP, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_NOTIFY);
	pCharacteristic_setup->addDescriptor(new BLE2902());
	pCharacteristic_setup->setCallbacks(new MyCallbacksSetup());

	pCharacteristic_data = pService->createCharacteristic(CHARACTER_UUID_DATA, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_NOTIFY);
	pCharacteristic_data->addDescriptor(new BLE2902());

	pCharacteristic_ctrl = pService->createCharacteristic(CHARACTER_UUID_CTRL, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE | BLECharacteristic::PROPERTY_NOTIFY);
	pCharacteristic_ctrl->addDescriptor(new BLE2902());
	pCharacteristic_ctrl->setCallbacks(new MyCallbacksCtrl());

	pService->start();

	uint8_t value[1];
	value[0] = 0;
	pCharacteristic_setup->setValue(value, 0);
	pCharacteristic_data->setValue(value, 0);
	pCharacteristic_ctrl->setValue(value, 0);
	pServer->getAdvertising()->start();
*/

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
	pinMode(PIN_DIR_X, OUTPUT);
	digitalWrite(PIN_DIR_X, HIGH);
	pinMode(PIN_STEP_X, OUTPUT);
	digitalWrite(PIN_STEP_X, HIGH);
	pinMode(PIN_SW_X, INPUT_PULLUP);

	// encoder
	attachInterrupt(PIN_ENC_A, isr_encoder, CHANGE);
	attachInterrupt(PIN_ENC_B, isr_encoder, CHANGE);
	pinMode(PIN_ENC_SW, INPUT_PULLUP);


	timer = timerBegin(0, 80, true);
	timerAttachInterrupt(timer, &onTimer, true);
	timerAlarmWrite(timer, 1000, true);
	timerAlarmEnable(timer);
}



void loop() {


	static int mn = 0;
	batt = ((analogRead(PIN_BATT) * 33 * 3.3 / 4096) + batt) / 2;
	draw_menu();
	int n = 0;
	int d = 300;
	char tmp_str[100];
	for (n = 0; n < M_LAST; n++) {
		if (menu[n].updated != 0) {
			menu[n].updated = 0;
//			mn = n;
			d = 100;
			if (n == M_RUN) {
				// start run
				if (menu[M_RUN].value == 1) {
					part = 0;
					digitalWrite(PIN_EN_X, LOW);
					gotopos = menu[M_START].value + (part * menu[M_DIFF].value);
					if (menu[M_START].value + (part * menu[M_DIFF].value) > menu[M_END].value) {
						menu[M_RUN].value = 0;
					}
				}
			}
			if (n == M_POSLOCK) {
				// change lock
				if (menu[M_POSLOCK].value == 1) {
					digitalWrite(PIN_EN_X, LOW);
				} else {
					digitalWrite(PIN_EN_X, HIGH);
				}
			}
		}
	}

	if (menu[M_ZERO].value == 1) {
		gotozeroX();
		menu[M_ZERO].value = 0;
	}
/*
	static int nn = 0;
	nn++;
	if (nn >= 50) {
		nn = 0;
		mn++;
		if (mn >= M_LAST) {
			mn = 0;
		}
		sprintf(tmp_str, "%i;%i;%i;%s;%i", mn, M_LAST, menu[mn].type, menu[mn].text, menu[mn].value);
		pCharacteristic_setup->setValue((uint8_t *)tmp_str, strlen(tmp_str));
		pCharacteristic_setup->notify();
		Serial.println(tmp_str);
	}
*/
}


