bool calibrating,cross,tri,sqr,circ,leftTrigger,rightTrigger, leftShoulder, rightShoulder;

bool outputVal=false;

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

		if(outputVal){	
			Serial.print("L1: ");
			Serial.print(leftShoulder);
			Serial.print("R1: ");
			Serial.print(rightShoulder);
			Serial.print("L2: ");
			Serial.print(leftTrigger);
			Serial.print("R2: ");
			Serial.print(rightTrigger);
			Serial.print("X: ");
			Serial.print(cross);
			Serial.print(" O: ");
			Serial.print(circ);
			Serial.print(" T: ");
			Serial.print(tri);
			Serial.print(" S: ");
			Serial.print(sqr);
			Serial.print(" LX: ");
			Serial.print(leftStickX);
			Serial.print(" LY: ");
			Serial.print(leftStickY);
			Serial.print(" RX: ");
			Serial.print(rightStickX);
			Serial.print(" RY: ");
			Serial.print(rightStickY);	
		    Serial.print(" X: ");
		    Serial.print(gyroX);
		    Serial.print(" Y: ");
		    Serial.println(gyroY);
		}
	}
}
void onConnect(){
	Serial.println("Connected");
	String address = Ps3.getAddress();
    Serial.println("Started receiving on: "+address);
}
/*
void setup(){
    Serial.begin(115200);
    Ps3.begin();
	Ps3.attach(update);
	Ps3.attachOnConnect(onConnect);
}

void loop(){
	if(Serial.available()>0){
		String temp=Serial.readString();
		temp.toUpperCase();
		if(temp.indexOf("CAL")!=-1)calibrateGyro();
	}
	delay(50);
}
*/
// add trigger and shoulder 
// add gyro smoothing
// start converting ps3 input to rc channel in actual code
