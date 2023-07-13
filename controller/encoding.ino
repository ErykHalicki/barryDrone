const int MESSAGE_SIZE = 128;

struct ControllerData {
  bool calibrating;
  bool cross;
  bool tri;
  bool sqr;
  bool circ;
  bool leftTrigger;
  bool rightTrigger;
  bool leftShoulder;
  bool rightShoulder;
  int leftStickX;
  int leftStickY;
  int rightStickX;
  int rightStickY;
  float gyroXOffset;
  float gyroYOffset;
  float gyroX;
  float gyroY;
};

void encodeControllerData(const ControllerData& data, uint8_t* message) {
  memcpy(message, &data, sizeof(ControllerData));
}

void decodeControllerData(const uint8_t* message, ControllerData& data) {
  memcpy(&data, message, sizeof(ControllerData));
}

void transmitControllerData() {
  ControllerData controllerData;
  
  // Populate controllerData with the PS3 controller info
  
  // Create a buffer to hold the encoded message
  uint8_t message[MESSAGE_SIZE];

  // Encode the controller data into the message
  encodeControllerData(controllerData, message);

  // Transmit the message using ESP-NOW
  esp_now_send(destAddress, message, MESSAGE_SIZE);
}

void receiveControllerData(const uint8_t* message) {
  ControllerData controllerData;
  
  // Decode the message into controllerData
  decodeControllerData(message, controllerData);

  // Process the received controller data
  // ...
}
