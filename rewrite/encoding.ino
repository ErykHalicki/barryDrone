const int MESSAGE_SIZE = 128;


void encodeControllerData(const uint8_t* message,ControllerData d);
void decodeControllerData(const uint8_t* message, ControllerData d);

void transmitControllerData() {
  ControllerData controllerData;
  
  // Populate controllerData with the PS3 controller info
  
  // Create a buffer to hold the encoded message
  uint8_t* message=(uint8_t*)malloc(MESSAGE_SIZE);

  // Encode the controller data into the message
  encodeControllerData(message,controllerData);

  // Transmit the message using ESP-NOW
  esp_now_send(destAddress, message, MESSAGE_SIZE);
}

void encodeControllerData(uint8_t* message, ControllerData d) {
  memcpy(message, &d, sizeof(ControllerData));
}

void decodeControllerData(const uint8_t* message, ControllerData d) {
  memcpy(&d, message, sizeof(ControllerData));
}

void receiveControllerData(const uint8_t * mac, const uint8_t *message, int len) {
  ControllerData controllerData;
  
  // Decode the message into controllerData
  decodeControllerData(message, controllerData);

  // Process the received controller data
  // ...
}
