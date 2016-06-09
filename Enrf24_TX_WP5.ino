#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>

int incomingByte = 0;   // for incoming serial data

const int rd =  10;      // the number of the LED pin
const int gn =  19;      // the number of the LED pin
const int bl =  12;      // the number of the LED pin

const int ir =  11;      // the number of the LED pin
const int ir2 =  5;      // the number of the LED pin

int analogPin = 2;
int val = 0;

Enrf24 radio(8, 9, 12);  // P2.0=CE, P2.1=CSN, P2.2=IRQ
const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };

const char *str_on = "ON";
const char *str_off = "OFF";

void dump_radio_status_to_serialport(uint8_t);

void setup() {
  
  // put your setup code here, to run once:
  pinMode(rd, OUTPUT); 
  pinMode(gn, OUTPUT); 
  //pinMode(bl, OUTPUT); 
  pinMode(ir, OUTPUT); 
  pinMode(ir2, OUTPUT); 
  
  digitalWrite(rd, LOW);
  digitalWrite(gn, LOW);
  //digitalWrite(bl, LOW);
  digitalWrite(ir, LOW);
  digitalWrite(ir2, LOW);
  
  //analogReference(INTERNAL1V5);
  ADC10CTL0= REFON + REFOUT + REF2_5V;
  
  Serial.begin(9600);

  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  radio.begin();  // Defaults 1Mbps, channel 0, max TX power
  dump_radio_status_to_serialport(radio.radioState());

  radio.setTXaddress((void*)txaddr);
}

void loop() {
  Serial.print("Sending packet: ");
  radio.print(getADC());
  ADC10CTL0 |= REFON + REFOUT + REF2_5V;
  //radio.print(";");
  //radio.print(getVCC());
  //radio.print(";");
  //radio.println(getTemp());
  radio.flush();  // Force transmit (don't wait for any more data)
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(1000);
  
  radio.print(getVCC());
  ADC10CTL0 |= REFON + REFOUT + REF2_5V;
  //radio.print(";");
  //radio.print(getVCC());
  //radio.print(";");
  //radio.println(getTemp());
  radio.flush();  // Force transmit (don't wait for any more data)
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(1000);
  
  radio.print(getTemp());
  ADC10CTL0 |= REFON + REFOUT + REF2_5V;
  //radio.print(";");
  //radio.print(getVCC());
  //radio.print(";");
  //radio.println(getTemp());
  radio.flush();  // Force transmit (don't wait for any more data)
  dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  delay(1000);
  
  delay(26900);
  
            // send data only when you receive data:
        if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
                switch(incomingByte){
                 case 'i':
                 // say what you got:
                Serial.print("I received: ");
                Serial.println(incomingByte, DEC);
                break;
                
                case 'r':
                digitalWrite(rd, HIGH);
                break;
                
                case 't':
                digitalWrite(rd, LOW);
                break;
                
                case 'g':
                digitalWrite(gn, HIGH);
                break;
                
                case 'h':
                digitalWrite(gn, LOW);
                break;
                
                case 'b':
                digitalWrite(bl, HIGH);
                break;
                
                case 'n':
                digitalWrite(bl, LOW);
                break;
                
                case 'q':
                digitalWrite(ir, HIGH);
                break;
                
                case 'w':
                digitalWrite(ir, LOW);
                break;
                
                case 'x':
                Serial.print(getADC());
                Serial.print(";");
                Serial.print(getVCC());
                Serial.print(";");
                Serial.println(getTemp());
                break;
                
                }
                
                //val = analogRead(analogPin);    // read the input pin
                //Serial.println(getADC());             // debug value
                
        }
}

int getADC(){
  int i=0;
  long val=0;
  
  while(i<256)
  {
    val+=analogRead(analogPin);
    i++;
  }
  val/=16;
  
  return val;
}

int getTemp(){
  int i=0;
  long val=0;
  
  while(i<256)
  {
    val+=analogRead(TEMPSENSOR);
    i++;
  }
  val/=16;
  
  return val;
}

int getVCC(){
  int i=0;
  long val=0;
  
  while(i<256)
  {
    val+=analogRead(INCH_11);
    i++;
  }
  val/=16;
  
  return val;
}

void dump_radio_status_to_serialport(uint8_t status)
{
  Serial.print("Enrf24 radio transceiver status: ");
  switch (status) {
    case ENRF24_STATE_NOTPRESENT:
      Serial.println("NO TRANSCEIVER PRESENT");
      break;

    case ENRF24_STATE_DEEPSLEEP:
      Serial.println("DEEP SLEEP <1uA power consumption");
      break;

    case ENRF24_STATE_IDLE:
      Serial.println("IDLE module powered up w/ oscillators running");
      break;

    case ENRF24_STATE_PTX:
      Serial.println("Actively Transmitting");
      break;

    case ENRF24_STATE_PRX:
      Serial.println("Receive Mode");
      break;

    default:
      Serial.println("UNKNOWN STATUS CODE");
  }
}
