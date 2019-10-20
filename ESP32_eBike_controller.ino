#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VescUart.h>
#include <TinyGPS++.h>
#include "AiEsp32RotaryEncoder.h"

//Inits
#define sw1 32
#define sw2 2
int sw1State, sw2State;
int potPin = 34;
int throttle = 0;
int maxRPM = 16000;
int rpm = 0;
int rpmInt = 250;
int rpmSens = 50;
int prevRPM = 0;
bool safetyInit = false;
bool safetyMax = false;
bool safetyMin = false;
int lcdCurPage = 1;
static const uint8_t lcdPages[] = {1, 2};

// Rotary Encoder
#define ROTARY_ENCODER_A_PIN 18
#define ROTARY_ENCODER_B_PIN 19
#define ROTARY_ENCODER_BUTTON_PIN 23
#define ROTARY_ENCODER_VCC_PIN -1
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, ROTARY_ENCODER_VCC_PIN);
int test_limits = 2;
int bPushed = 0;

// The TinyGPS++ object
TinyGPSPlus gps;
bool gpsOn = false;
float speed = 0.0;

// LCD SDA 21 SCL 22
LiquidCrystal_I2C lcd(0x27, 20, 4);
char line1[21];
char line2[21];
char line3[21];
char line4[21];

// Thermistor variables
int ThermistorPin = 14;
int Vo;
float R1 = 10000;
float logR2, R2, T, Tc, Tf;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
int therCount = 0;
int therAvgCount = 10;
float therAvg = 0.0;
bool noLcd = false;

// GPS UART
#define RXD1 5
#define TXD1 17

// VESC UART
VescUart UART;
#define RXD2 25
#define TXD2 27
int dRPM = 0;
float dV = 0.00;
float dC = 0.000;
float dP = 0.0;
float dTv = 0.0;
float dTm = 0.0;
float dAhC = 0.0;
float dAh = 0.0;
bool noBreak = false;

void rotary_loop() {
	// rotary encoder button click
	if (rotaryEncoder.currentButtonState() == BUT_RELEASED) {
		bPushed = 0;
	}
  if (rotaryEncoder.currentButtonState() == BUT_PUSHED) {
		bPushed = 1;
	}

	// rotary encoder change check
	int16_t encoderDelta = rotaryEncoder.encoderChanged();
	// if no change
	if (encoderDelta == 0) return;
	// increased or decreased
	// if (encoderDelta>0) Serial.print("+");
	// if (encoderDelta<0) Serial.print("-");
	if (encoderDelta!=0) {
		//now we need current value
		int16_t encoderValue = rotaryEncoder.readEncoder();
		lcdCurPage = encoderValue;
	} 
}


void setup() {
  // Set encoder pins as inputs  
  //we must initialize rorary encoder 
	rotaryEncoder.begin();
	rotaryEncoder.setup([]{rotaryEncoder.readEncoder_ISR();});
	//optionally we can set boundaries and if values should cycle or not
	rotaryEncoder.setBoundaries(1, 2, true); //minValue, maxValue, cycle values (when max go to min and vice versa)
  
  // LCD
  lcd.init();
  Serial.begin(115200); // Serial terminal
  Serial2.begin(115200);  // UART VESC
  Serial1.begin(9600, SERIAL_8N1, RXD1); // GPS
  UART.setSerialPort(&Serial2);

  // switches
  pinMode(sw1, INPUT_PULLDOWN);
  pinMode(sw2, INPUT_PULLDOWN);
}

void loop() {
  // Thermistor
  Vo = analogRead(ThermistorPin);
  lcd.setCursor(14, 3);
  R2 = R1 * (4095.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  therAvg += Tc;
  therCount += 1;
  if (therCount == therAvgCount){
    dTm = therAvg/therAvgCount;
    therAvg = 0.0;
    therCount = 0;
  }

  // switches
  sw1State = digitalRead(sw1);
  sw2State = digitalRead(sw2);

  if (sw2State == 1){
    noBreak = true;
  }else if (sw2State == 0){
    noBreak = false;
  }
  if (sw1State == 1){
    noLcd = true;
  }else if (sw1State == 0){
    noLcd = false;
  }
  if (noLcd == true) {
    lcd.noBacklight();
  } else {
    lcd.backlight();
  }
  
  
  unsigned long currentMillis = millis();
  throttle = analogRead(potPin);
  rpm = map(throttle, 0, 4095, 0, maxRPM);
  // lcd.backlight();
  //Serial.print(xPortGetCoreID());

  // Safety startup
  while (safetyInit == false) {
    throttle = analogRead(potPin);
    rpm = map(throttle, 0, 4095, 0, maxRPM);
    sprintf(line1, "%-12s", "SAFETY LOCK"); 
    sprintf(line2, "%-4s%-6s%-4s%-6s", "MAX", (String)safetyMax, "MIN", (String)safetyMin); 
    sprintf(line4, "Throttle: %-7s", String(rpm/(float)maxRPM*100, 0)); 
    printOnLcd();
    // wait for max min on throttle
    if (safetyMax == false) {
      if (rpm > maxRPM - rpmInt){safetyMax = true; Serial.println("SafetyMax released");}
    }
    else {
      if (rpm < rpmInt){safetyMin = true;}
    }
    if (safetyMax == true && safetyMin == true){safetyInit = true;}
    delay(10);
    }
  rotary_loop();
  rotaryEncoder.enable ();
 

  // Receive VESC values
  if ( UART.getVescValues()) {
    dRPM = UART.data.rpm;
    dV = UART.data.inputVoltage;
    dC = UART.data.avgMotorCurrent;
    dP = UART.data.inputVoltage*UART.data.avgInputCurrent;
    dTv = UART.data.fetTemp;
    dAh = UART.data.ampHours;
    dAhC = UART.data.ampHoursCharged;
  }

  
  if (noBreak == true){
    UART.setBrakeCurrent(0);
    prevRPM = 0;
  }
  else {
    //Set RPM based on pot value
    while (abs(prevRPM - rpm) > rpmInt){
      if (prevRPM - rpm > 0){
        prevRPM -= rpmSens;
      } else{
        prevRPM += rpmSens;
      }
      UART.setRPM(prevRPM);
      delay(4);
    }
    //prevRPM = rpm;
    UART.setRPM(prevRPM);
  }


  // Print things on LCD
  if (lcdCurPage == 1){
    processPage1();
  } else if (lcdCurPage == 2){
    processPage2();
  }
  printOnLcd();
}

void printOnLcd(){
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
  lcd.setCursor(0, 2);
  lcd.print(line3);
  lcd.setCursor(0, 3);
  lcd.print(line4);
}

void processPage1(){
  sprintf(line1, "%-5s%-5s%-4s%-3s%-3s", "Volt", "Curr",  "Pow", "Tv", "Tm"); 
  sprintf(line2, "%-5s%-5s%-4s%-3s%-3s", String(dV, 1), String(dC, 1), String(dP, 0), String(dTv, 0), String(dTm, 0)); 
  sprintf(line3, "%-4s%-4s%-5s%-4s%-3s", "RPM", "set", "Sp.", "Ah", "-Ah"); 
  sprintf(line4, "%-4s%-4s%-5s%-4s%-3s", String(dRPM/100., 0), String(prevRPM/100., 0), String(speed/100., 1), String(dAh/100., 0), String(dAhC/100., 0)); 
}

void processPage2(){
  sprintf(line1, "%04d.%02d.%02d  %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()); 
  sprintf(line2, "la %-6s lo %-6s", String(gps.location.lat(), 4), String(gps.location.lng(), 4)); 
  sprintf(line3, "%-3s%-3s%-3s%-4s  %-5s", "S1", "S2", "RS", "GPS", "Break"); 
  sprintf(line4, "%-3d%-3d%-3d%-4s  %-5s", sw1State, sw2State, bPushed, gps.location.isValid()? "ON":"OFF", noBreak? "OFF":"ON"); 
}
