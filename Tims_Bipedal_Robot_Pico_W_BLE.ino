

/*

	Tims_Bipedal_Robot_Pico_W_BLE.ino
		Version 29/01/2025

	This sketch is by Tim Jackson.1960.
		https://www.timsnet.co.uk/

		It is for the Bipedal Robot by Freenove.
		Using Raspberry Pi Pico (W) BLE.

		Creadit where due:

		It is made from the Freenove libraries and examples.
		https://freenove.com/

		Audio Libraries:
		https://github.com/earlephilhower/ESP8266Audio/tree/master

		Notes!
			Using Arduino IDE make sure the following are set:
				Board:				Raspberry Pi Pico (W).
				Flash Size:			Sketch-1M, FS-1M.
				IP/Bluetooth Stack:	IPv4 + Bluetooth.

	Sketch uses 655008 bytes (62%) of program storage space. Maximum is 1044480 bytes.
	Global variables use 123904 bytes (47%) of dynamic memory, leaving 138240 bytes for local variables. Maximum is 262144 bytes.


*/

#include <BTstackLib.h>
#include <SPI.h>
#include "btstack.h"

#include <Arduino.h>
#include <EEPROM.h>
#include "BLE_DataCommand.h"
#include "AudioFileSourceLittleFS.h"
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2SNoDAC.h"
#include "Bipedal_Robot_sounds.h"
#include "Freenove_Robot_For_Pico_W.h"
#include "Freenove_Robot_Emotion.h"
#include "Freenove_Robot_WS2812.h"
#include "Bipedal_Robot.h"

//#define DEBUG_COMS
//#define DEBUG_SOUND
//#define DEBUG_ACTION


#define MAX_LENGTH 256
static char characteristic_data[MAX_LENGTH] = "Pico W Bluetooth";

/*
	Servos

	Calibration:
			   ---     ---
		 ---------------
		|     O   O     |
		|---------------|
YR 12==>|               | <== YL 10
		 ---------------
			||     ||
			||     ||
RR 13==>  -----   -----  <== RL 11
		 |-----   -----|

		{ int YL, int YR, int RL, int RR }

		Values in radians
*/
int YL = -10;
int YR = 7;
int RL = -3;
int RR = 25;
int saveSettings = 0;
int calibratePosition[4] = { YL, YR, RL, RR };

/*	==	Robot	===	*/
#define LeftLegPin 10
#define RightLegPin 12
#define LeftFootPin 11
#define RightFootPin 13
Bipedal_Robot Bipedal_Robot;
int PresetTask = 0;
int playstartingmusic = 0;
int T = 1000;
int moveId = 0;
int dance_steps = 0;
int Ultrasonic_Avoid_steps = 0;
int moveSize = 15;
int Resting = 0;
float Distance = 0;
bool isServoResting = true;

/*	===	Timeing	===	*/
#define UPLOAD_VOL_TIME 2000
unsigned long lastUploadVoltageTime;
#define UPLOAD_DISTANCE_TIME 200
unsigned long lastUploadDistanceTime;

/*	===	BLE	===	*/
bool isSubscribed = false;
hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
uint16_t characteristic_handle = 0;
#define CCCD_HANDLE (characteristic_handle + 1)
BLE_DataCommand BLECmd;

/*	===	Audio	===	*/
//#define BUZZER_PIN 2 /*	Sorted in "Freenove_Robot_For_Pico_W.h"	*/
#define SPEAKER_PIN 6
AudioFileSourceLittleFS* file;
AudioOutputI2SNoDAC* out;
AudioGeneratorMP3* mp3;
int repeatNumber = 1;
int trackNumber = 0;
/*
	Volume is a multiplier
	example:
		out->SetGain(2.0);  // Double the volume
		out->SetGain(0.5);  // Halve the volume

*/
int playCount = 0;
float PlaybackVolume = 2.0;
bool isPlayingMP3 = false;
bool isPlayingSong = false;
bool mp3Initialized = false;


/*	Setup Raspberry Pi Pico Onboard BLE	*/
void setupBLE(const char* BLEName) {
	BTstack.setBLEDeviceConnectedCallback(deviceConnectedCallback);
	BTstack.setBLEDeviceDisconnectedCallback(deviceDisconnectedCallback);
	BTstack.setGATTCharacteristicRead(gattReadCallback);
	BTstack.setGATTCharacteristicWrite(gattWriteCallback);

	BTstack.addGATTService(new UUID("B8E06067-62AD-41BA-9231-206AE80AB551"));
	characteristic_handle = BTstack.addGATTCharacteristicDynamic(
		new UUID("f897177b-aee8-4767-8ecc-cc694fd5fce0"),
		ATT_PROPERTY_READ | ATT_PROPERTY_WRITE | ATT_PROPERTY_NOTIFY,
		0);

	BTstack.setup(BLEName);
	BTstack.startAdvertising();

#ifdef DEBUG_COMS
	Serial.println("Waiting for a client connection to notify...");
#endif /*	DEBUG_COMS	*/

}
/*	Setups	*/
void setup(void) {
	Serial.begin(115200);
	delay(1000);
	setupBLE("PicoW_BLE");
	delay(1000);

	EEPROM.begin(512);																/*	Initialize the ultrasonic module	*/

	Bipedal_Robot.init(LeftLegPin, RightLegPin, LeftFootPin, RightFootPin, true);	/*	Set the Servo pins	*/
	Bipedal_Robot.home();

	delay(50);
	//Buzzer_Setup();	/*	Do not use, done by "Bipedal_Robot.Buzzer_init(BUZZER_PIN)"	*/
	WS2812_Setup();
	Emotion_Setup();
	Emotion_Detection();

	/*  Register BLE Commands   */
	BLECmd.addCommand("A", receiveMovement);
	BLECmd.addCommand("B", receiveEmotion);
	BLECmd.addCommand("C", receiveLED);
	BLECmd.addCommand("D", recieveBuzzer);
	BLECmd.addCommand("G", recieveCalibration);
	BLECmd.addCommand("H", receivePresetTask);
	BLECmd.addCommand("L", receiveMP3Play);
	BLECmd.addCommand("M", receiveAction);
	BLECmd.addCommand("S", receiveStop);
	BLECmd.addCommand("O", reciveSing);
	BLECmd.addDefaultHandler(receiveStop);
}
void setup1() {

	pinMode(SPEAKER_PIN, OUTPUT);			/*	Sets the Speaker pin	*/
	digitalWrite(SPEAKER_PIN, LOW);

	Bipedal_Robot.Buzzer_init(BUZZER_PIN);	/*	Sets the Buzzer pin		*/

	Ultrasonic_Setup();

}
/*	Loops	*/
void loop() {

	BTstack.loop();	/*	Check BLE	*/

	WS2812_Show(ws2812_task_mode);	/*	RGB LEDs	*/
	/*	Battery Voltage	*/
	if (millis() - lastUploadVoltageTime > UPLOAD_VOL_TIME) {
		upLoadVoltageToApp();
		lastUploadVoltageTime = millis();
	}
	/*	Matrix Expesion	*/
	if (Check_Module_value == MATRIX_IS_EXIST) {
		Emotion_Show(emotion_task_mode);	/*	LED Mmatrix Display.	*/
	}

	move(moveId);	/*	Movement	*/


	/*	===	Feedback, Debugging and Throughput	===	*/
	if (Serial.available() > 0) {
		String input = Serial.readStringUntil('\n');
		input.trim();

		size_t input_length = input.length();

		memcpy(characteristic_data, input.c_str(), input_length);
		characteristic_data[input_length] = '\n';
		characteristic_data[input_length + 1] = '\0';

		Serial.print("input data: ");
		Serial.print(characteristic_data);

		sendNotificationToSubscribers();
		delay(5);
	}
}
void loop1() {
	/*	MP3 Play	*/
	handleMP3Playback();

	/*	Sonar Distance	*/
	if (millis() - lastUploadDistanceTime > UPLOAD_DISTANCE_TIME) {
		if (PresetTask == 1) {
			upLoadDistanceToApp();
		}
		lastUploadDistanceTime = millis();
	}

}

/*	===	Callbacks		===	*/
void deviceConnectedCallback(BLEStatus status, BLEDevice* device) {
	if (status == BLE_STATUS_OK) {
		Serial.println("Device connected!");
		connection_handle = device->getHandle();
	}
}
void deviceDisconnectedCallback(BLEDevice* device) {
	Serial.println("Disconnected.");
	connection_handle = HCI_CON_HANDLE_INVALID;
	isSubscribed = false;
}
uint16_t gattReadCallback(uint16_t value_handle, uint8_t* buffer, uint16_t buffer_size) {
	if (buffer && buffer_size > 0) {
		Serial.print("Read data: ");
		Serial.println(characteristic_data);

		size_t data_length = strlen(characteristic_data);
		if (data_length > buffer_size) {
			data_length = buffer_size;
		}

		memcpy(buffer, characteristic_data, data_length);
		return data_length;
	}
	return 0;
}
int gattWriteCallback(uint16_t value_handle, uint8_t* buffer, uint16_t size) {
	if (value_handle == CCCD_HANDLE) {
		if (size >= 2) {
			uint16_t cccd_value = buffer[0] | (buffer[1] << 8);
			isSubscribed = cccd_value ? true : false;
		}
	}
	else {
		size_t copy_size = (size < (MAX_LENGTH - 1)) ? size : (MAX_LENGTH - 1);
		memcpy(characteristic_data, buffer, copy_size);
		characteristic_data[copy_size] = '\0';

		Serial.print("Received data: ");
		Serial.println(characteristic_data);

		BLECmd.processBLEData(buffer, size);	/*	Call BLE data processing function	*/
	}
	return 0;
}
/*	===	Notification	===	*/
void sendNotificationToSubscribers() {
	if (isSubscribed && connection_handle != HCI_CON_HANDLE_INVALID) {
		att_server_notify(connection_handle, characteristic_handle,
			(uint8_t*)characteristic_data, strlen(characteristic_data));
	}
}


/*	Movement	*/
void move(int movemode) {
	// bool manualMode = false;
	switch (movemode) {
	case 0:
		Bipedal_Robot.home();
		if (Resting == 1) {
			Bipedal_Robot.setRestState(false);
			Bipedal_Robot.home();
			Resting = 0;
		}
		break;
	case 1:
		staticEmtions(21);
		Bipedal_Robot.walk(1, T, 1);
		break;
	case 2:
		staticEmtions(21);
		Bipedal_Robot.walk(1, T, -1);
		break;
	case 3:
		staticEmtions(21);
		Bipedal_Robot.turn(1, T, 1);
		break;
	case 4:
		staticEmtions(21);
		Bipedal_Robot.turn(1, T, -1);
		break;
	case 5:
		Bipedal_Robot.updown(1, 2000, 30);
		break;
	case 6: Bipedal_Robot.moonwalker(1, T, moveSize, 1); break;
	case 7: Bipedal_Robot.moonwalker(1, T, moveSize, -1); break;
	case 8: Bipedal_Robot.swing(1, T, moveSize); break;
	case 9: Bipedal_Robot.crusaito(1, T, moveSize, 1); break;
	case 10: Bipedal_Robot.crusaito(1, T, moveSize, -1); break;
	case 11: Bipedal_Robot.jump(1, T); break;
	case 12: Bipedal_Robot.flapping(1, T, moveSize, 1); break;
	case 13: Bipedal_Robot.flapping(1, T, moveSize, -1); break;
	case 14: Bipedal_Robot.tiptoeSwing(1, T, moveSize); break;
	case 15: Bipedal_Robot.bend(1, T, 1); break;
	case 16: Bipedal_Robot.bend(1, T, -1); break;
	case 17: Bipedal_Robot.shakeLeg(1, T, 1); break;
	case 18:/*	Hello.mp3			*/
		staticEmtions(21);
		Bipedal_Robot.swing(1, 1100, 50);
		Bipedal_Robot.home();
		Resting = 1;
		moveId = 0;
		clearEmtions();
		break;
	case 19:/*	Nicetomeetyou.mp3	*/
		staticEmtions(22);
		Bipedal_Robot.moonwalker(1, 1550, 50, 1);
		Bipedal_Robot.home();
		Resting = 1;
		moveId = 0;
		clearEmtions();
		break;
	case 20:/*	goodbye.mp3			*/
		staticEmtions(23);
		Bipedal_Robot.ascendingTurn(2, 700, 12);
		clearEmtions();
		delay(100);
		Bipedal_Robot.home();
		Resting = 1;
		moveId = 0;
		break;
	case 21:
		Ultrasonic_Avoid();
		break;
	case 22:
		dance();
		break;
	default:
		break;
	}
}
/*  Ultrasonic Avoidance    */
void Ultrasonic_Avoid() {
	if (Distance <= 15) {
		Bipedal_Robot.walk(2, 1000, -1);	/*	BACKWARD x2	*/
		Bipedal_Robot.turn(3, 1000, -1);	/*	LEFT x3		*/
	}
	Bipedal_Robot.walk(1, 1500, 1);			/*	FORWARD x1	*/
}
/*  Dance Routeen    */
void dance() {

	dance_steps = dance_steps + 1;
	staticEmtions(21);
	if (moveId > 0) {
		switch (dance_steps) {
		case 0:
			dance_steps = 0;
			break;
		case 1:
			Bipedal_Robot.jitter(1, 750, 20);
			break;
		case 2:
			Bipedal_Robot.jitter(1, 750, 20);
			break;
		case 3:
			Bipedal_Robot.crusaito(1, 800, 30, 1);
			break;
		case 4:
			Bipedal_Robot.crusaito(1, 800, 30, -1);
			break;
		case 5:
			Bipedal_Robot.crusaito(1, 800, 30, 1);
			Bipedal_Robot.home();
			delay(300);
			break;
		case 6:
			Bipedal_Robot.walk(1, 1500, -1);
			break;
		case 7:
			Bipedal_Robot.walk(1, 1000, 1);
			break;
		case 8:
			Bipedal_Robot.walk(1, 1000, 1);
			Bipedal_Robot.home();
			break;
		case 9:
			Bipedal_Robot.moonwalker(1, 600, 30, 1);
			break;
		case 10:
			Bipedal_Robot.moonwalker(1, 600, 30, -1);
			break;
		case 11:
			Bipedal_Robot.moonwalker(1, 600, 30, 1);
			break;
		case 12:
			Bipedal_Robot.moonwalker(1, 600, 30, -1);
			Bipedal_Robot.home();
			delay(100);
			break;
		case 13:
			Bipedal_Robot.walk(1, 1500, -1);
			Bipedal_Robot.home();
			delay(100);
			break;
		}
		if (dance_steps > 14) {
			dance_steps = 0;
		}
	}
	else {
		Resting = 1;
		moveId = 0;
		dance_steps = 0;
	}
}

/*
	Set Expressions

		Identifier = B.
		Perameters = 1. (Mode)

	Example: B#1
	Range: 1 to 6 (7 or Greater = Random)

*/
void receiveEmotion() {

	sendAck();

	Emotion_Setup();
	moveId = 0;
	//trackNumber = 0;
	char* arg;
	arg = BLECmd.next();                        /*  Get next argument.                  */

	if (arg != NULL) {
		String stringOne = String(arg);         /*  Convert constant char to String.    */
		emotion_task_mode = stringOne.toInt();
		Emotion_SetMode(emotion_task_mode);     /*  Show static expressions.            */
	}

	sendFinalAck();

#ifdef DEBUG_ACTION
	Serial.print("receiveEmotion arg: ");
	Serial.println(arg);
#endif /*	DEBUG_ACTION	*/

}
/*
	Set LEDs

		Identifier = C.
		Perameters = 4. (Mode, Red, Green, Blue)

	Example: C#2#100#150#200
	Range:	Mode	0 to 5
			Red		0 to 255
			Green	0 to 255
			Blue	0 to 255

*/
void receiveLED() {

	sendAck();

	moveId = 0;
	//trackNumber = 0;
	char* arg;
	unsigned char paramters[3];
	char* endstr;
	arg = BLECmd.next();                        /*  Get next argument.      */
	if (arg != NULL) {
		String stringOne = String(arg);			/*	convert into String		*/
		ws2812_task_mode = stringOne.toInt();	/*	convert into Interger	*/
	}
	else {
	}
	arg = BLECmd.next();                        /*  Get next argument.      */
	String stringOne1 = String(arg);			/*	convert into String		*/
	paramters[0] = stringOne1.toInt();			/*	convert into Interger	*/

	arg = BLECmd.next();                        /*  Get next argument.      */
	String stringOne2 = String(arg);			/*	convert into String		*/
	paramters[1] = stringOne2.toInt();			/*	convert into Interger	*/

	arg = BLECmd.next();                        /*  Get next argument.      */
	String stringOne3 = String(arg);			/*	convert into String		*/
	paramters[2] = stringOne3.toInt();			/*	convert into Interger	*/

	WS2812_Set_Color_1(15, paramters[0], paramters[1], paramters[2]);

	sendFinalAck();
}
/*
	Set Buzzer

		Identifier = D.
		Perameters = 1. (Frequency)

	Example: D#2000
	Range: What range can you hear?

*/
void recieveBuzzer() {

	sendAck();

	moveId = 0;
	//trackNumber = 0;
	bool error = false;
	int frequency;
	char* arg;
	arg = BLECmd.next();                        /*  Get next argument.      */
	if (arg != NULL) frequency = atoi(arg);

	if (frequency) {

		tone(BUZZER_PIN, frequency);
	}
	else {

		noTone(BUZZER_PIN);
	}


	sendFinalAck();
}
/*
	Calibration

		YL = Left Leg.
		YR = Right Leg.
		RL = Left Foot.
		RR = Right Foot.

		calibratePosition[4] = { YL, YR, RL, RR };

		Identifier = G.
		Perameters = 5. (Left Leg, Right Leg, Left Foot, Right Foot, Save to EEPROM)

	Example: G#-10#7#-3#25#0
	Adjustments are +/- from 90 degrees.

*/
void calib_homePos() {
	int servoPos[4];
	servoPos[0] = 90;
	servoPos[1] = 90;
	servoPos[2] = 90;
	servoPos[3] = 90;
	Bipedal_Robot._moveServos(500, servoPos);
}
void recieveCalibration() {

	sendAck();

	char* arg;
	arg = BLECmd.next();					/*  Get next argument.      */
	if (arg != NULL) {
		YL = String(arg).toInt();			/*	convert into Interger	*/
	}
	arg = BLECmd.next();					/*  Get next argument.      */
	if (arg != NULL) {
		YR = String(arg).toInt();			/*	convert into Interger	*/
	}
	arg = BLECmd.next();					/*  Get next argument.      */
	if (arg != NULL) {
		RL = String(arg).toInt();			/*	convert into Interger	*/
	}
	arg = BLECmd.next();					/*  Get next argument.      */
	if (arg != NULL) {
		RR = String(arg).toInt();			/*	convert into Interger	*/
	}
	arg = BLECmd.next();					/*  Get next argument.      */
	if (arg != NULL) {
		saveSettings = String(arg).toInt();	/*	convert into Interger	*/
	}

	sendFinalAck();

	YL = constrain(YL - 90, -40, 50);
	YR = constrain(YR - 90, -40, 50);
	RL = constrain(RL - 90, -40, 50);
	RR = constrain(RR - 90, -40, 50);
	Bipedal_Robot.setTrims(YL, YR, RL, RR);
	calib_homePos();
	if (saveSettings == 1) {
		Bipedal_Robot.setTrims(YL, YR, RL, RR);
		calib_homePos();
		Bipedal_Robot.saveTrimsOnEEPROM();
	}


}
/*
	Move

		Identifier = A.
		Perameters = 2. (Mode, Speed)

	Example: A#6#1000
	Range:	Mode	0 to 22 (21 = Sonar, 22 = Dance)
			Speed	The higher the value the slower it moves. (There are presets 0, 1 and 2)

*/
void receiveMovement() {

	sendAck();

	ws2812_task_mode = 0;
	emotion_task_mode = 0;
	if (Bipedal_Robot.getRestState() == true) Bipedal_Robot.setRestState(false);
	isServoResting = Bipedal_Robot.getRestState();
	char* arg;
	arg = BLECmd.next();                        /*  Get next argument.      */
	if (arg != NULL) {
		moveId = atoi(arg);
		if (moveId == 0) {
			Resting = 1;
		}
	}
	else {
		moveId = 0;
	}
	arg = BLECmd.next();                        /*  Get next argument.      */
	if (arg != NULL) {
		T = atoi(arg);
		if (T == 0) {
			T = 1000;
		}
		else if (T == 1) {
			T = 1500;
		}
		else if (T == 2) {
			T = 2000;
		}
	}
	else T = 2000;

#ifdef DEBUG
	Serial.print("T is:");
	Serial.println(T);
#endif /*	DEBUG	*/

	arg = BLECmd.next();                        /*  Get next argument.      */
	if (arg != NULL) moveSize = atoi(arg);
	else moveSize = 15;

	sendFinalAck();

}
/*
	Sonar or Dance Mode

		Identifier = H.
		Perameters = 1. (Off/Sonar/Dance)

	Example: H#1
	Range:	0 Off.
			1 Activate Sonar (Puts it in move mode 21).
			2 Activate Dance (Puts it in move mode 22).

*/
void receivePresetTask() {

	sendAck();

	ws2812_task_mode = 0;
	emotion_task_mode = 0;
	if (Bipedal_Robot.getRestState() == true) Bipedal_Robot.setRestState(false);
	isServoResting = Bipedal_Robot.getRestState();

	char* arg;
	arg = BLECmd.next();		/*	Get next argument.	*/
	if (arg != NULL) {
		PresetTask = atoi(arg);
		Ultrasonic_Avoid_steps = 0;
		trackNumber = 0;
		Resting = 0;
		moveId = 0;
		dance_steps = 0;
		if (PresetTask == 1) {	/*	Ultrasonic Avoidance	*/
			moveId = 21;
		}
		if (PresetTask == 2) {	/*	Dance	*/
			moveId = 22;
			trackNumber = 4;
		}
		if (PresetTask == 0) {	/*	Stop	*/
			Resting = 1;
		}
	}

	sendFinalAck();
}
/*
	Listen to MP3 File

		Identifier = L.
		Perameters = 2. (Number of Times to Play, File Fumber)

	Example: L#1#1
	Files:	0	split.mp3
			1	Hello.mp3
			2	Nicetomeetyou.mp3
			3	goodbye.mp3
			4	1.mp3
			5	onfoot.mp3 (not in data folder)
			6	hello_cn.mp3
*/
void receiveMP3Play() {

	sendAck();

	if (!isPlayingMP3) {

		char* arg;
		arg = BLECmd.next();					/*  Get next argument.      */
		if (arg != NULL) {
			repeatNumber = String(arg).toInt();	/*	convert into Interger	*/
		}
		arg = BLECmd.next();					/*  Get next argument.      */
		if (arg != NULL) {
			trackNumber = String(arg).toInt();	/*	convert into Interger	*/
		}

		playmusic(repeatNumber, trackNumber);
	}

	sendFinalAck();

}
/*
	Interactive Music

		Identifier = M.
		Perameters = 1. (Action/Track)

		Example: M#1
		Action:	1	Hello
				2	Nice to meet you
				3	Goodbye

			Note!
				Got issues with playing MP3 file.
*/
void receiveAction() {

	int _action;
	sendAck();

	ws2812_task_mode = 0;
	emotion_task_mode = 0;
	if (Bipedal_Robot.getRestState() == true) Bipedal_Robot.setRestState(false);
	isServoResting = Bipedal_Robot.getRestState();
	char* arg;
	arg = BLECmd.next();                        /*  Get next argument.	*/
	if (arg != NULL) {
		_action = atoi(arg);
		trackNumber = _action;
		//playstartingmusic = _action;
		clearEmtions();
		if (trackNumber == 1) {
			moveId = 18;						/*	Hello.mp3			*/

#ifdef DEBUG_SOUND
			Serial.println("Hello");
#endif /*	DEBUG_SOUND	*/
		}
		else if (_action == 2) {	/*	Nice to meet you	*/
			moveId = 19;
#ifdef DEBUG_SOUND
			Serial.println("Nice to meet you");
#endif /*	DEBUG_SOUND	*/
		}
		else if (_action == 3) {	/*	Goodbye	*/
			moveId = 20;
#ifdef DEBUG_SOUND
			Serial.println("Goodbye");
#endif /*	DEBUG_SOUND	*/
		}
		else if (_action == 0) {	/*	Stop	*/
			moveId = 0;
		}
		playmusic(repeatNumber, trackNumber);

#ifdef DEBUG_SOUND
		Serial.print("moveId:");
		Serial.println(moveId);
#endif /*	DEBUG_SOUND	*/
	}
	else {
		moveId = 0;
		Resting = 1;
	}

	sendFinalAck();

}
void playmusic(int repeat, int song) {

	trackNumber = song;

	if (song == 0) {
		return;	/*	Exit the function	*/
	}
	else if (song == 1) {
		file = new AudioFileSourceLittleFS("/Hello.mp3");
	}
	else if (song == 2) {
		file = new AudioFileSourceLittleFS("/NiceToMeetYou.mp3");
	}
	else if (song == 3) {
		file = new AudioFileSourceLittleFS("/Goodbye.mp3");
	}
	else if (song == 4) {
		file = new AudioFileSourceLittleFS("/Dance.mp3");
	}
	else if (song == 5) {
		file = new AudioFileSourceLittleFS("/OK.mp3");
	}
	else if (song == 6) {
		file = new AudioFileSourceLittleFS("/Yes.mp3");
	}
	else if (song == 7) {
		file = new AudioFileSourceLittleFS("/No.mp3");
	}
	else if (song == 8) {
		file = new AudioFileSourceLittleFS("/Ready.mp3");
	}

	out = new AudioOutputI2SNoDAC(SPEAKER_PIN);
	out->SetGain(PlaybackVolume);
	mp3 = new AudioGeneratorMP3();
	mp3->begin(file, out);
	isPlayingMP3 = true;
	playCount = repeat;
	mp3Initialized = true;

#ifdef DEBUG_SOUND
	Serial.println("MP3 Set to play.");
#endif /*	DEBUG_SOUND	*/

}
void handleMP3Playback() {

	// Check if song 4 should be played 
	if (trackNumber == 4) {
#ifdef DEBUG_SOUND
		Serial.print("handleMP3Playback: trackNumber = ");
		Serial.println(trackNumber);
		Serial.print("handleMP3Playback: mp3Initialized = ");
		Serial.println(mp3Initialized);
		Serial.print("handleMP3Playback: mp3->isRunning() = ");
		Serial.println(mp3->isRunning());
#endif /*   DEBUG_SOUND */
		if (!mp3Initialized || !mp3->isRunning()) {
			//delay(2000);
			playmusic(1, trackNumber);
		}

	}

	// Handle non-blocking MP3 playback
	if (isPlayingMP3) {

		if (trackNumber == 0) {
			mp3->stop();
		}

		if (!mp3->isRunning() || !mp3->loop()) {

#ifdef DEBUG_SOUND
			Serial.println("if no loop");
#endif /*   DEBUG_SOUND */

			EndOfMP3Play();
			playCount--;
			if (playCount > 0) {
				playmusic(playCount, trackNumber);
			}
			else {
				isPlayingMP3 = false;
			}
		}
	}
}
void EndOfMP3Play() {

	delete file;
	delete mp3;
	out->flush();
	out->stop();
	mp3Initialized = false;
	pinMode(SPEAKER_PIN, OUTPUT);
	digitalWrite(SPEAKER_PIN, LOW);

#ifdef DEBUG_SOUND
	Serial.println("MP3 playback finished and resources cleaned up.");
#endif /*	DEBUG_SOUND	*/

}

/*
	Stop
		Identifier = S.
		Perameters = None

	Example: S

*/
void receiveStop() {

	moveId = 0;
	sendAck();

	sendFinalAck();
}
void sendAck() {
	Serial.flush();
	delay(10);
}
void sendFinalAck() {

	Serial.flush();
	delay(10);
}
/*	Sing	*/
void reciveSing() {

	int singNmber;
	sendAck();

	if (!isPlayingSong) {
		char* arg;
		arg = BLECmd.next();                        /*  Get next argument.	*/
		singNmber = atoi(arg);
		isPlayingSong = true;
		Bipedal_Robot.sing(singNmber);
		isPlayingSong = false;
	}

	sendFinalAck();

}


/*	===	Outgoing	===	*/

/*	Voltage	*/
void upLoadVoltageToApp() {
	float voltage = 0;
	voltage = Get_Battery_Voltage();
	char voltageData[10];
	snprintf(voltageData, sizeof(voltageData), "P#%d", int(voltage * 1000));									/*	Prepare voltage data in P# format	*/
	att_server_notify(connection_handle, characteristic_handle, (uint8_t*)voltageData, strlen(voltageData));	/*	Notify subscribers with voltage		*/
}
/* Distance */
void upLoadDistanceToApp() {
	Distance = Get_Sonar();

#ifdef DEBUG_ACTION
	Serial.println("Distance (raw): " + String(Distance) + "\n");													/*	Print ultrasonic distance	*/
#endif /*	DEBUG_ACTION	*/

	int distanceInt = static_cast<int>(Distance * 100);															/*	Convert to millimeters	*/
	char distanceData[10];
	snprintf(distanceData, sizeof(distanceData), "E#%d", distanceInt);											/*	Prepare distance data in E# format	*/

#ifdef DEBUG_ACTION
	Serial.println("Formatted Distance Data: " + String(distanceData) + "\n");									/*	Print formatted distance data	*/
#endif /*	DEBUG_ACTION	*/

	att_server_notify(connection_handle, characteristic_handle, (uint8_t*)distanceData, strlen(distanceData));	/*	Notify subscribers with Distance	*/
}


