#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <VescUart.h>
#include <PushButton.h>
#include <Smoothed.h>

//Inits

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI  23
#define OLED_CLK   18
#define OLED_DC    21
#define OLED_CS    5
#define OLED_RESET 4
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#define swBrake 27
#define swAllOff 14
#define swCruise 26
int sw1State, brakeState, allOffState, cruiseState;
int potPin = 34, maxRPM = 18000, rpm = 0;
float throttle = 0.0;
Smoothed <int> rpmRead;
// int rpmInt = 150, rpmSens = 50, prevRPM = 0;


// Thermistor variables (installed on motor)
int ThermistorPin = 4, Vo;
float R1 = 10000, logR2, R2, T, Tc, Tf, c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
Smoothed <int> motorTemp;

// VESC UART
VescUart UART;
#define RXD2 35
#define TXD2 19
Smoothed <int> dRPM;
Smoothed <float> dC;
Smoothed <float> dP;
Smoothed <float> dV;
float dTv = 0.0, dTm = 0.0, dAhC = 0.0, dAh = 0.0, rpmTokmph = 0.00133333333333;
bool noBrake = false;
bool allOff = false;
bool cruise = false;
bool lastCruiseState = false;


void setup() {
  Serial.begin(115200); // Serial terminal
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // UART VESC
  UART.setSerialPort(&Serial2);

  pinMode(ThermistorPin, INPUT);

  // switches
  pinMode(swBrake, INPUT_PULLDOWN);
  pinMode(swAllOff, INPUT_PULLDOWN);
  pinMode(swCruise, INPUT_PULLDOWN);
  // Smoothed values
  motorTemp.begin(SMOOTHED_AVERAGE, 20);
  rpmRead.begin(SMOOTHED_AVERAGE, 8);
  dRPM.begin(SMOOTHED_AVERAGE, 15);
  dC.begin(SMOOTHED_AVERAGE, 15);
  dP.begin(SMOOTHED_AVERAGE, 15);
  dV.begin(SMOOTHED_AVERAGE, 15);
  dRPM.add(0);  // avoid a division error
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
}

void loop() {
  // Thermistor
  /* tempCalculator(); */

  // throttle
  throttle = getThrottlePercent();

  // switches
  switchesFunc();

  // VESC read values
  vescRead();

  // Set RPM values
  setRpm();

  renderDisplay();

  delay(100);
}

void renderDisplay(){
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println((float)dRPM.get()*rpmTokmph, 1);
  display.setCursor(78, 0);
  display.setTextSize(1);
  display.println(F("set km/h"));
  display.setCursor(78, 8);
  display.setTextSize(2);
  display.println(F("22.6"));
  display.setTextSize(2);
  display.println(dV.get());
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.println(F("+2300 -100 mAh"));
  display.setTextSize(1);
  display.setCursor(0, 54);
  display.println(F("Brake 56.5C cruise"));
  display.display();      // Show initial text
}

void tempCalculator(){
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (4095.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  motorTemp.add(Tc);
}

void switchesFunc(){
  brakeState = digitalRead(swBrake);
  if (brakeState == 1){
    noBrake = true;
  }else if (brakeState == 0){
    noBrake = false;
  }

  allOffState = digitalRead(swAllOff);
  if (allOffState == 1){
    allOff = true;
  }else if (allOffState == 0){
    allOff = false;
  }

  cruiseState = digitalRead(swCruise);
  if (cruiseState == 1){
    cruise = true;
  }else if (cruiseState == 0){
    cruise = false;
  }
}

void vescRead(){
  // Receive VESC values
  if ( UART.getVescValues()) {
    Serial.println(UART.data.inputVoltage);
    dRPM.add(UART.data.rpm);
    dV.add(UART.data.inputVoltage);
    dC.add(UART.data.avgMotorCurrent);
    dP.add(UART.data.inputVoltage*UART.data.avgInputCurrent);
    dTv = UART.data.fetTemp;
    dAh = UART.data.ampHours;
    dAhC = UART.data.ampHoursCharged;
  }
}

float getThrottlePercent(){
  float percentage;
  percentage = map(analogRead(potPin), 780, 2880, 0, 10000)/100.0;
  if (percentage <= 1.0) {
    return 0.0;
  }
  else if (percentage >= 99.0){
    return 1.0;
  }
  else {
    return percentage/100.0;
  }
}


void setRpm(){
/*   if (allOff == true) { */
/*     /\* Serial.println("All off"); *\/ */
/*   } */
/*   else if (cruise == true) { */
/*     /\* Serial.println("Cruise On"); *\/ */
/*   } */
/*   else { */
/*     /\* Serial.println("Center"); *\/ */
/*   } */

/*   if (cruise != lastCruiseState) { */
/*     if (cruise == true){ */
/*       rpm = throttle * maxRPM; */
/*     } */
/*   } */

/*   if (allOff == true){ */
/*     UART.setBrakeCurrent(0); */
/*   } */
/*   else if (cruise == true){ */
/*     UART.setRPM(rpm); */
/*   } */
/*   else if (noBrake == true){ */
/*     if (UART.data.rpm - 0.1*maxRPM > throttle*maxRPM || throttle < 0.15) { */
/*       UART.setBrakeCurrent(0); */
/*     } */
/*     else { */
/*       UART.setRPM(throttle*maxRPM); */
/*     } */
/*   } */
/*   else { */
/*     UART.setRPM(throttle*maxRPM); */
/*   } */
/*   lastCruiseState = cruise; */
UART.setRPM(2000);
}
