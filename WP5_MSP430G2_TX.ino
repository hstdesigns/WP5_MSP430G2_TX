#include <Enrf24.h>
#include <nRF24L01.h>
#include <string.h>
#include <SPI.h>

int incomingByte = 0;   // for incoming serial data

const int rd =  10;      // the number of the LED pin
const int gn =  18;      // the number of the LED pin
const int bl =  12;      // the number of the LED pin

const int ir =  11;      // the number of the LED pin
const int ir2 =  5;      // the number of the LED pin

const int analogOPV = 2;
const int analogDAC = 5;
int val = 0;

Enrf24 radio(8, 9, 13);  // P2.0=CE, P2.1=CSN, P2.2=IRQ
const uint8_t txaddr[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x01 };

const char *str_on = "ON";
const char *str_off = "OFF";

void dump_radio_status_to_serialport(uint8_t);

#define TEMP 0
#define MVAL 1
#define NVAL 2

#define DEBUGLEVEL 1

struct masterBootRecord {
           /** Code Area for master boot program. */
  uint8_t  codeArea[440];
           /** Optional WindowsNT disk signature. May contain more boot code. */
  uint32_t diskSignature;
           /** Usually zero but may be more boot code. */
  uint16_t usuallyZero;
           /** First MBR signature byte. Must be 0X55 */
  uint8_t  mbrSig0;
           /** Second MBR signature byte. Must be 0XAA */
  uint8_t  mbrSig1;
};
/** Type name for masterBootRecord */
typedef struct masterBootRecord mbr_t;

// f(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2

const float p00 =      -21.87;  // (-22.2, -21.53)
const float p10 =    -0.04634;  // (-0.06036, -0.03231)
const float p01 =       4.326;  // (4.321, 4.331)
const float p20 =  -0.0002811;  // (-0.0004589, -0.0001033)
const float p11 =    0.009101;  // (0.009047, 0.009154)
const float p02 =   5.385e-05;  // (3.428e-05, 7.343e-05)

float calcPressureComp(float T, float mV){
  float mbar=p00 + p10*T + p01*mV + p20*T*T + p11*T*mV + p02*mV*mV;
  return mbar;
}

const float pcc24_base_val[][3]{
     -2.971025641,	4.308043897,	        -22.26698376
,6.19230769230769,	4.39836189253926,	-22.9277146126782
,11.1161538461538,	4.43915619451586,	-22.9686787325047
,16.1244871794872,	4.48561941795294,	-23.1641856752349
,21.1671794871795,	4.52935982677534,	-23.1033558267446
,25.8638461538462,	4.57257852802868,	-23.4376711220657
,30.7515384615385,	4.61588841504755,	-23.6868599349341
,35.4934615384615,	4.65824853625457,	-24.0810530809115
,40.3941025641026,	4.70201216940864,	-24.3932906941516
,45.2202564102564,	4.74650644794932,	-24.6804668350968
,49.934358974359,	4.79042324307334,	-25.192815111471
,54.9092307692308,	4.83565965771321,	-25.5807847291393
,59.5867948717949,	4.87840473691108,	-25.9959635677649
,64.1074358974359,	4.92125018380386,	-26.5745102400811
,73.03051282,           5.037447619,            -29.52103867
};

struct Tmn_2_t{
  float T1;
  float T2;
  float m1;
  float m2;
  float n1;
  float n2;
} Tmn_2;

struct Tmn_2_t *ptr = &Tmn_2;

void base_values(float T, struct Tmn_2_t *Tmn){
  uint8_t i = 0;			//weniger als 256 verschiedene T Werte
  while(T < pcc24_base_val[i][TEMP]){
    i++;
  }
  Tmn->T1 = pcc24_base_val[i][TEMP];
  Tmn->T2 = pcc24_base_val[i+1][TEMP];
  Tmn->m1 = pcc24_base_val[i][MVAL];
  Tmn->m2 = pcc24_base_val[i+1][MVAL];
  Tmn->n1 = pcc24_base_val[i][NVAL];
  Tmn->n2 = pcc24_base_val[i+1][NVAL];
}

void debugTable(){
  byte i = 0;			//weniger als 256 verschiedene T Werte
  while(i < 15){
    Serial.print(pcc24_base_val[i][TEMP]);
    Serial.print(";");
    Serial.print(pcc24_base_val[i][MVAL]);
    Serial.print(";");
    Serial.println(pcc24_base_val[i][NVAL]);
    i++;
  }
}

float y_lin_interpol(float x, float x_1, float x_2, float y_1, float y_2){
  return y_1 + (y_2 - y_1) / (x_2 - x_1) * (x - x_1);
}

float m,n;

float calc_pressure(float T, float u){
  base_values(T, &Tmn_2);
  m=y_lin_interpol(T, ptr->T1, ptr->T2, ptr->m1,ptr->m2);
  n=y_lin_interpol(T, ptr->T1, ptr->T2, ptr->n1,ptr->n2);
  
//  m=y_lin_interpol(T, Tmn_2.T1, Tmn_2.T2, Tmn_2.m1,Tmn_2.m2);
//  n=y_lin_interpol(T, Tmn_2.T1, Tmn_2.T2, Tmn_2.n1,Tmn_2.n2);
  
//  Serial.print(m);
//  Serial.print(";");
//  Serial.println(n);
  
  return m*u + n;
}

void debugprint(char *msg){
  if (DEBUGLEVEL > 0){
    Serial.println(msg);
  } 
}

// init procedure
//--------------------------------------
void setup() {  
  delay(500);  
  
  // GPIO Setup
  //--------------------------------------
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
  
  // init uart with 9600,8,1
  //--------------------------------------
  Serial.begin(9600);

  // init NRF24L01+ funk module
  //--------------------------------------
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  radio.begin();  // Defaults 1Mbps, channel 0, max TX power
  dump_radio_status_to_serialport(radio.radioState());

  radio.setTXaddress((void*)txaddr);
  
  // init PWM module
  //--------------------------------------
  P2DIR |= 0x40;  // output for PWM
  P2SEL |= 0x40;  // gpio config for PWM output

  TA0CCTL1 = CM_0 | CCIS_0 | OUTMOD_7;  // reset/set mode
  TA0CTL = TASSEL_2 | ID_0 | MC_1;  // SMCLK, upmode
  TA0CCR0 = 4096 - 1;  // 16MHz div 4096 = 3906Hz PWM Frequenz

  TA0CCR1 = 2048;  // init with 50% duty cycle
}

word adcV = 0;
word dacV = 0;
word adcTemp = 0;

float adcU = 0;
word pwmVal = 0;
float T = 0.0;
float mbar = 0.0;

const int adcOPVoffset = -1640;
const int adcOPVslew = 20.0/116.0;

#define RFON 1  // 1: enable, 0: disable NRF24L01 transmission

void loop() {
  
  digitalWrite(rd, HIGH);  // enable red led  

  ADC10CTL0 |= REFON + REFOUT + REF2_5V;
  delay(1);
  adcV = getADC();  
  
  ADC10CTL0 |= REFON + REFOUT + REF2_5V;
  delay(1);
  dacV = getVCC();
  
  ADC10CTL0 |= REFON + REFOUT + REF2_5V;
  delay(1);
  adcTemp = getTemp();
  
  ADC10CTL0 |= REFON + REFOUT + REF2_5V;
  delay(1);
  
  T=calc_T(adcTemp);  // count to temperature incl compensation
  adcU = calc_u(T, (adcV));  // convert adc counts to mV --> currently without compensation
  
  adcU -= 0;
  //mbar = calc_pressure(T, adcU);  // get mbar value 
  mbar = calcPressureComp(T, adcU);  // get mbar value 
  pwmVal = calc_pwm(mbar);  // convert analog to adc counts
  
  TA0CCR1 = pwmVal;  // set dac value 0...3600mV
  
  if (RFON>0){  
    //debugprint("Sending packet: ");
    //Serial.print("Sending packet: ");
  
    radio.print(adcV);
    radio.print(";");
    radio.print(adcU);
    radio.print(";");
    radio.print(adcTemp);
    radio.print(";");
    radio.print(mbar);
    radio.flush();
    
    delay(100);
  
    //dump_radio_status_to_serialport(radio.radioState());  // Should report IDLE
  }
  
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
                
                case '1':
                  mbar = calc_pressure(0.1,196.0);
                  Serial.println(mbar);
                  mbar = calc_pressure(6.2,107.0);
                  Serial.println(mbar);
                  mbar = calc_pressure(62.7,168.0);
                  Serial.println(mbar);
                  mbar = calc_pressure(70.0,15.4);
                  Serial.println(mbar);
                break;
                
                case '2':
                debugTable();
                break;
                
                case '3':
                  mbar = calc_pressure(4.0,40.0);
                  Serial.println(mbar);
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
                //val = analogRead(analogOPV);    // read the input pin
                //Serial.println(getADC());             // debug value                
        }
}

int getADC(){
  int i=0;
  long val=0;
  
  while(i<256)
  {
    val+=analogRead(analogOPV);
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
    val+=analogRead(analogDAC);
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

const float adCcoeffA = 2500.0/16384.0;
const float adCoeffB = 1.0/30.0/4.0;

float calc_u(float T, int adc_val){
  float temp = adc_val * adCcoeffA;
  
  //temp-=-0.2199*T  + 191.2;
  //temp -= 140.769;
  return temp/5.4-42.05;
  //return temp*adCoeffB;
}

const float tempCoeff = (2.5/16384.0)/0.00355*1.43;

float calc_T(word tempVal){
  return (tempVal*tempCoeff)-287.43; 
}

word calc_pwm(float mbar){
  word pwm_val = 228; //0 bar entspricht 200 mV
  while(mbar > 0)
  {
    mbar -= 1;
    pwm_val += 2;
  }
  return pwm_val;
}

void dump_radio_status_to_serialport(uint8_t status){
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

