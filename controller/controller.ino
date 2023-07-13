#include<esp_now.h>
bool calibrating,cross,tri,sqr,circ,leftTrigger,rightTrigger, leftShoulder, rightShoulder;

int leftStickX,leftStickY,rightStickX,rightStickY, calibrationCountdown=5;
float gyroXOffset=0,gyroYOffset=0,gyroX,gyroY;

void calibrateGyro(){
	calibrating=true;
	Serial.println("~~~~~~PLACE CONTROLLER ON FLAT SURFACE~~~~~~");
	delay(1000);
	Serial.println("Calibrating in: ");
	for(int i=calibrationCountdown;i>0;i--){
		Serial.println(i);
		delay(1000);
	}
	for(int i=0;i<10;i++){
		gyroXOffset+=Ps3.data.sensor.accelerometer.x;
		gyroYOffset+=Ps3.data.sensor.accelerometer.y;
		delay(10);
	}
	gyroXOffset/=10;
	gyroYOffset/=10;
	calibrating=false;
}
void update(){
	recv=true;
	if(!calibrating){
		if( Ps3.event.button_down.cross )cross=true;
		if( Ps3.event.button_up.cross )cross=false;
	
	    if( Ps3.event.button_down.square )sqr=true;
	    if( Ps3.event.button_up.square )sqr=false;
	
	    if( Ps3.event.button_down.triangle )tri=true;
	    if( Ps3.event.button_up.triangle )tri=false;
	
	    if( Ps3.event.button_down.circle )circ=true;
	    if( Ps3.event.button_up.circle )circ=false;
		leftShoulder=false;
		if(Ps3.data.analog.button.l1>0)leftShoulder=true;
		rightShoulder=false;
		if(Ps3.data.analog.button.r1>0)rightShoulder=true;
		leftTrigger=false;
		if(Ps3.data.analog.button.l2>0)leftTrigger=true;
		rightTrigger=false;
		if(Ps3.data.analog.button.r2>0)rightTrigger=true;
	
	
	    leftStickX=Ps3.data.analog.stick.lx;
	   	leftStickY=-Ps3.data.analog.stick.ly;
	
	   	rightStickX=Ps3.data.analog.stick.rx;
		rightStickY=-Ps3.data.analog.stick.ry;
		gyroX+=Ps3.data.sensor.accelerometer.x-gyroXOffset;
		gyroY+=Ps3.data.sensor.accelerometer.y-gyroYOffset;
		gyroX/=2;
		gyroY/=2;
	}
}
void onConnect(){
	Serial.println("Connected");
	String address = Ps3.getAddress();
    Serial.println("Started receiving on: "+address);
}

void setup(){
	Serial.begin(9600);

	esp_now_init();
	esp_now_add_peer();
	esp_now_register_recv_cb(receiveControllerData);

	Ps3.begin();
	Ps3.attach(update);
	Ps3.attachOnConnect(onConnect);
}
void loop(){
	
}
//data format, {lx,ly,rx,ry,
