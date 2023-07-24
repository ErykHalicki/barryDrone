const int MESSAGE_SIZE = 128;

void encodeControllerData(uint8_t* message, ControllerData d) {
  	memcpy(message, &d, sizeof(ControllerData));
}

void decodeControllerData(const uint8_t* message, ControllerData d) {
  	memcpy(&d, message, sizeof(ControllerData));
}

void encodeDiagnosticData(uint8_t* message, DiagnosticData d) {
  	memcpy(message, &d, sizeof(DiagnosticData));
}
void decodeDiagnosticData(const uint8_t* message, DiagnosticData d) {
  	memcpy(&d, message, sizeof(DiagnosticData));
}

void transmitDiagnosticInfo() {
	DiagnosticData dd;
	//write info to dd
  	uint8_t* message=(uint8_t*)malloc(MESSAGE_SIZE);
  	encodeDiagnosticData(message,dd);
  	esp_now_send(destAddress, message, MESSAGE_SIZE);
}
void receiveDiagnosticData(const uint8_t * mac, const uint8_t *message, int len) {
  	DiagnosticData dd;
  	decodeDiagnosticData(message, dd);
	//write dd to variables
}

void transmitControllerData() {
	ControllerData cd;
  	cd.calibrating = calibrating;
  	cd.cross = cross;
  	cd.tri = tri;
  	cd.sqr = sqr;
  	cd.circ = circ;
 	cd.leftTrigger = leftTrigger;
  	cd.rightTrigger = rightTrigger;
  	cd.leftShoulder = leftShoulder;
  	cd.rightShoulder = rightShoulder;
  	cd.leftStickX = leftStickX;
  	cd.leftStickY = leftStickY;
  	cd.rightStickX = rightStickX;
  	cd.rightStickY = rightStickY;
  	cd.gyroXOffset = gyroXOffset;
  	cd.gyroYOffset = gyroYOffset;
  	cd.gyroX = gyroX;
  	cd.gyroY = gyroY;
  	uint8_t* message=(uint8_t*)malloc(MESSAGE_SIZE);
  	encodeControllerData(message,cd);
  	esp_now_send(destAddress, message, MESSAGE_SIZE);
}

void receiveControllerData(const uint8_t * mac, const uint8_t *message, int len) {
  	ControllerData cd;
  	decodeControllerData(message, cd);
	circ = cd.circ;
	calibrating = cd.calibrating;
	cross = cd.cross;
	tri = cd.tri;
	sqr = cd.sqr;
	circ = cd.circ;
	leftTrigger = cd.leftTrigger;
	rightTrigger = cd.rightTrigger;
	leftShoulder = cd.leftShoulder;
	rightShoulder = cd.rightShoulder;
	leftStickX = cd.leftStickX;
	leftStickY = cd.leftStickY;
	rightStickX = cd.rightStickX;
	rightStickY = cd.rightStickY;
	gyroXOffset = cd.gyroXOffset;
	gyroYOffset = cd.gyroYOffset;
	gyroX = cd.gyroX;
	gyroY = cd.gyroY;
}
