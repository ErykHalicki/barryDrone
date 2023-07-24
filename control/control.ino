#include <esp_now.h>
#include <Ps3Controller.h>
#include <WiFi.h>
#include "encoding.h"
esp_now_peer_info_t drone;

bool calibrating,cross,tri,sqr,circ,leftTrigger,rightTrigger, leftShoulder, rightShoulder;

bool outputVal=false;
int leftStickX,leftStickY,rightStickX,rightStickY, calibrationCountdown=5;
float gyroXOffset=0,gyroYOffset=0,gyroX,gyroY;

void setup(){
	Serial.begin(9600);

 	Serial.println(WiFi.macAddress());
	WiFi.mode(WIFI_STA);
	esp_now_init();

	memcpy(drone.peer_addr, destAddress, 6);
  	drone.channel = 0;  
  	drone.encrypt = false;

	esp_now_add_peer(&drone);

	esp_now_register_recv_cb(receiveDiagnosticData);

	Ps3.begin();
	Ps3.attach(update);
	Ps3.attachOnConnect(onConnect);
}
void loop(){
	transmitControllerData();	
}
