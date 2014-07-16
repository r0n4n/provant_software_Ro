#include "c_datapr_MultWii.h"

static const int _multwiisize=1024;;
uint16_t multwii_msgindex = 0;
uint8_t multwii_checksum = 0;

void serialize8(uint8_t a) {
  //Serial.write(a);
  multwii_msg[multwii_msgindex++] = a;
  multwii_msg[multwii_msgindex] = '\0';
  multwii_checksum ^= a &0x00FF;
  if (multwii_msgindex >= _multwiisize)
  {
    multwii_msgindex = 0;
  }
}


int get_raw_size()
{
  return multwii_msgindex;
}

char* get_raw_String()
{
  multwii_msgindex = 0;
  return multwii_msg;
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}


void serialize32_as16(int32_t a) {
  serialize8(((int16_t)a   ) & 0xFF);
  serialize8(((int16_t)a>>8) & 0xFF);
}

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void headSerialResponse(uint8_t size, uint8_t multwii_msg) {
  serialize8('$');
  serialize8('M');
  serialize8('>');
  multwii_checksum = 0; // start calculating a new multwii_checksum
  serialize8(size);
  serialize8(multwii_msg);
}

void tailSerialReply() {
  serialize8(multwii_checksum);	
}

/*********************End of serial comm************************************/
/*********************Begin of ... something************************************/

void send_attitude(float x,float y,float z)
{
     headSerialResponse(6,MSP_ATTITUDE);
     serialize32_as16(x);
     serialize32_as16(y);
     serialize32_as16(z);
     tailSerialReply();
}

void send_raw_imu(float* acc,float* gyr, float* mag)
{
	// not working?
     headSerialResponse(18,MSP_RAW_IMU);
     serialize32_as16(acc[0]);
     serialize32_as16(acc[1]);
     serialize32_as16(acc[2]);
     serialize32_as16(gyr[0]);
     serialize32_as16(gyr[1]);
     serialize32_as16(gyr[2]);
     serialize32_as16(mag[0]);
     serialize32_as16(mag[1]);
     serialize32_as16(mag[2]);
     tailSerialReply();
}


void send_altitude(float alt, float vario)
{
     headSerialResponse(6,MSP_ALTITUDE);
     serialize32(alt);
     serialize16(vario);
     tailSerialReply();
}

void send_bicopter_identifier()
{
     headSerialResponse(7,MSP_IDENT);
     serialize8(32);
     serialize8(4); //codigo do bicoptero
     serialize8(0); //not used
     serialize32(0);
     tailSerialReply();

}

void send_motor_pins()
{
  headSerialResponse(8, MSP_MOTOR_PINS);
  serialize8(1);
  serialize8(2);
  serialize8(0);
  serialize8(0);

  serialize8(0);
  serialize8(0);
  serialize8(0);
  serialize8(0);

  tailSerialReply();
}


void send_servos(float angle1,float angle2)
{
  headSerialResponse(16,MSP_SERVO);
  serialize16(0);
  serialize16(0);
  serialize16(0);
  serialize16(0);
  serialize16((int)(1500+8.333*angle1));
  serialize16((int)(1500+8.333*angle2));
  serialize16(0);
  serialize16(0);
  tailSerialReply();
}

void send_motor(float forca_esquerdo,float forca_direito)
{
    headSerialResponse(16, MSP_MOTOR);
    serialize32_as16((int)((forca_esquerdo*100)));
    serialize32_as16((int)((forca_direito*100)));
    serialize32_as16(0);
    serialize32_as16(0);

    serialize32_as16(0);
    serialize32_as16(0);
    serialize32_as16(0);
    serialize32_as16(0);
    tailSerialReply();
}

void send_debug(float debug1,float  debug2,float  debug3,float debug4)
{
    headSerialResponse(8, MSP_DEBUG);
    serialize32_as16((int)((debug1)));
    serialize32_as16((int)((debug2)));
    serialize32_as16((int)((debug3)));
    serialize32_as16((int)((debug4)));
    tailSerialReply();
}


/*
void setup()
{
	Serial.begin(115200);
}

int x=0;
float acc[] = {123,123,123};
void loop()
{
	x++;
	send_attitude(x/10,10,10);
	//send_raw_imu(acc,acc,acc);
	send_altitude(1234,0);
	delay(100);
}
*/