// library for I2C barometer
#include <Wire.h>
#include <Narcoleptic.h>

// libraries for radio transmitter
#include <RHDatagram.h>
#include <RH_ASK.h>
#include <SPI.h> // not used for needed for compile


// declare variables for the BMP180 barometer
short ac1;
short ac2;
short ac3;
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short b1;
short b2;
short mb;
short mc;
short md;
long PressureCompensate;
const unsigned char OSS = 3;
const int BMP180_ADDRESS = 0x77;
const int BMP180PowerPin = 2;
float temperature;
float pressure;

// define the height at the current location for sea level adjustment
short height = 101; // height at current location above sea level

// declare variables for radio transmitter
const int TxPowerPin = 4;
const int TRANSMITTER_ADDRESS = 0xDD;
const int RECEIVER_ADDRESS = 0xCD;
const int HEADER_TEMPERATURE = 0x25; // id for temperature variable
const int HEADER_PRESSURE = 0x28; // id for pressure variable
byte buf[RH_ASK_MAX_MESSAGE_LEN];

// setup variables for RH manager and driver
RH_ASK driver;  // use RH ASK driver
RHDatagram manager (driver, TRANSMITTER_ADDRESS);  // use the Datagram manager

// declare variables for ledPin
const int ledPowerPin = 14 ; //
const int ledEnabled = true; 

void setup()
{
  Serial.begin(9600);  // begin serial communications

  // setup the barometer
  pinMode(BMP180PowerPin, OUTPUT); // set the BMP180 Power pin to an output
  digitalWrite(BMP180PowerPin,HIGH);  // turn the power on to the BMP180
  delay(10); // small delay for BMP180 to start
  barometerInit();  // get the calibration variables for the BMP180
  pinMode(BMP180PowerPin, LOW);  // turn tho power off to the BMP180

  // setup the radio transmitter
  if (!driver.init())
    Serial.println("Radio setup failed");
  manager.setThisAddress(TRANSMITTER_ADDRESS);

  if (ledEnabled == true) {
    pinMode(ledPowerPin, OUTPUT);
  }

}

void loop()
{

  sleep(1); //sleep with narcoleptic for 1 minute

  pinMode(BMP180PowerPin, OUTPUT); // set the BMP180 Power pin to an output
  digitalWrite(BMP180PowerPin,HIGH);  // turn the power on to the BMP180
  delay(10);  // small delay for BMP180 to start
  temperature = barometerGetTemp(barometerReadUT());
  pressure = barometerGetPress(barometerReadUP()); 
  Serial.print(temperature/10, 1);
  Serial.println(" deg C");
  Serial.print(pressure/100, 1);
  Serial.println(" hPa");
  pinMode(BMP180PowerPin, LOW);  // turn the power off to the BMP180

  // call the temperature transmit routine
  Serial.print("Transmit temperature value: ");
  Serial.println(temperature);
  transmittemperature(temperature);


  delay (500);  // delay to give receiver time to decode previous message

  // call the pressure transmit routine
  Serial.print("Transmit pressure value: ");
  Serial.println(pressure, 0);
  transmitpressure(pressure);

}


void barometerInit(void)
{
  Wire.begin();
  ac1 = BMP180ReadInt(0xAA);
  ac2 = BMP180ReadInt(0xAC);
  ac3 = BMP180ReadInt(0xAE);
  ac4 = BMP180ReadInt(0xB0);
  ac5 = BMP180ReadInt(0xB2);
  ac6 = BMP180ReadInt(0xB4);
  b1 = BMP180ReadInt(0xB6);
  b2 = BMP180ReadInt(0xB8);
  mb = BMP180ReadInt(0xBA);
  mc = BMP180ReadInt(0xBC);
  md = BMP180ReadInt(0xBE);
}

// Read 1 byte from the BMP180 at 'address'
// Return: the read byte;
char BMP180Read(unsigned char address)
{
  //Wire.begin();
  unsigned char data;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP180_ADDRESS, 1);
  while(!Wire.available());
  return Wire.read();
}

// Read 2 bytes from the BMP180
// First byte will be from 'address'
// Second byte will be from 'address'+1
short BMP180ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP180_ADDRESS, 2);
  while(Wire.available()<2);
  msb = Wire.read();
  lsb = Wire.read();
  return (short) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned short barometerReadUT()
{
  unsigned short ut;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  delay(5);
  ut = BMP180ReadInt(0xF6);
  return ut;
}
// Read the uncompensated pressure value
unsigned long barometerReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  Wire.beginTransmission(BMP180_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = BMP180Read(0xF6);
  lsb = BMP180Read(0xF7);
  xlsb = BMP180Read(0xF8);
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  return up;
}

void writeRegister(short deviceAddress, byte address, byte val)
{
  Wire.beginTransmission(deviceAddress); // start transmission to device
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

short readRegister(short deviceAddress, byte address)
{
  short v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}


float barometerGetTemp(unsigned short ut)
{
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  PressureCompensate = x1 + x2;

  float temp = ((PressureCompensate + 8)>>4);
  temp = temp;
  return temp;
}

float barometerGetPress(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  b6 = PressureCompensate - 4000;
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  float pressure = (p/pow(1-(height/44330.0),5.255));
  pressure = pressure;  // convert to hPa from Pa
  return pressure;
}

void transmittemperature(int tempRF)
{
  pinMode(TxPowerPin, OUTPUT); // set the TX Power pin to an output
  digitalWrite(TxPowerPin,HIGH);  // turn the power on to the TX
  char tempdata [6];  // char size of temperature -xx.xC + NULL rounded to even
  itoa(tempRF, tempdata, 10);  // convert variable to array
  manager.setHeaderId(HEADER_TEMPERATURE);

  // Print the pressure data message
  Serial.print("This the is the Temperature data message: ");
  Serial.println(tempdata);

  Serial.println("Sending message");
  // turn the led on if enabled
  ledOn();
  // Send a message to the server
  manager.sendto((uint8_t *)tempdata, sizeof(tempdata), RECEIVER_ADDRESS);

  // Wait until the message has been sent before returning
  manager.waitPacketSent();
  Serial.println("Message Sent");
  // turn the led off if enabled
  ledOff();
  digitalWrite(TxPowerPin, LOW); // turn off the power to the TX

}

void transmitpressure(long pressureRF)
{
  pinMode(TxPowerPin, OUTPUT); // set the TX Power pin to an output
  digitalWrite(TxPowerPin,HIGH);  // turn the power on to the TX
  char pressuredata [8];  // size of pressure xxxxxx plus NULL rounded to even
  ltoa(pressureRF, pressuredata, 10);  // convert long variable to array
  manager.setHeaderId(HEADER_PRESSURE); // set the headerID to indicate pressure

  // Print the pressure data message
  Serial.print("This the is the Pressure data message: ");
  Serial.println(pressuredata);

  Serial.println("Sending message");
  // turn the led on if enabled
  ledOn();
  // Send a message to the server
  manager.sendto((uint8_t *)pressuredata, sizeof(pressuredata), RECEIVER_ADDRESS);

  // Wait until the message has been sent before returning
  manager.waitPacketSent();
  Serial.println("Message Sent");
  // turn the led off if enabled
  ledOff();
  digitalWrite(TxPowerPin, LOW); // turn off the power to the TX
}

// Allows us to specify a sleep time in minutes.
void sleep(int minutes){
  for (int count = 0; count < (minutes * 2); count++){
    Narcoleptic.delay(30000);
  }
}

void ledOn()
{
  if (ledEnabled == true) {
    digitalWrite(ledPowerPin, HIGH);
  }
}

void ledOff()
{
  if (ledEnabled == true) {
    digitalWrite(ledPowerPin, LOW);
  }
}







