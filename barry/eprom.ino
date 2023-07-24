typedef union int16_ty{
  	int16_t d;
  	byte    b[2];
};
typedef union float_ty
{
  float d;
  byte  b[4];
};

void write_int16(int pos, int16_t d){
  	int16_ty loc;
  	loc.d = d;
  	EEPROM.write(pos++,loc.b[0]);
  	EEPROM.write(pos++,loc.b[1]);
}
int16_t read_int16(int pos){
  	int16_ty loc;
  	loc.b[0] = EEPROM.read(pos++);
  	loc.b[1] = EEPROM.read(pos++);
  	return loc.d;
}

void write_float(int pos, float d){
  	float_ty loc;
  	loc.d = d;
  	EEPROM.write(pos++,loc.b[0]);
  	EEPROM.write(pos++,loc.b[1]);
  	EEPROM.write(pos++,loc.b[2]);
  	EEPROM.write(pos++,loc.b[3]);
}
float read_float(int pos){
  	float_ty loc;
  	loc.b[0] = EEPROM.read(pos++);
  	loc.b[1] = EEPROM.read(pos++);
  	loc.b[2] = EEPROM.read(pos++);
  	loc.b[3] = EEPROM.read(pos++);
  	return loc.d;
}

void PID_Read(){
  	P_Gyro    = read_float(18);
  	I_Gyro    = read_float(22);
  	D_Gyro    = read_float(26);
  	P_Angle   = read_float(30);
  	I_Angle   = read_float(34);
  	D_Angle   = read_float(38);
}
void PID_Store(){
  	write_float(18,P_Gyro);
  	write_float(22,I_Gyro);
  	write_float(26,D_Gyro);
  	write_float(30,P_Angle);
  	write_float(34,I_Angle);
  	write_float(38,D_Angle);
  	EEPROM.write(62, 0xAA);
    EEPROM.commit();
}
