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

#define swLowSpeed 14
#define swAllOff 33
#define swCruise 26
int sw1State, lowSpeedSwitch, allOffState, cruiseState;
int potPin = 34;
int rpm = 0;
int maxRPMLowSpeed = 7000;
int maxRPMHighSpeed = 18000;
int maxRPM;
float throttle = 0.0;

// VESC UART
VescUart UART;
#define RXD2 27
#define TXD2 25
Smoothed <int> dRPM;
Smoothed <float> dC;
Smoothed <float> dP;
Smoothed <float> dV;
float dTv = 0.0, dTm = 0.0, dAhC = 0.0, dAh = 0.0, rpmTokmph = 0.002285;
bool lowSpeed = true;
bool allOff = false;
bool cruise = false;
bool lastCruiseState = false;


void setup() {
  Serial.begin(115200); // Serial terminal
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // UART VESC
  UART.setSerialPort(&Serial2);

  // switches
  pinMode(swLowSpeed, INPUT_PULLDOWN);
  pinMode(swAllOff, INPUT_PULLDOWN);
  pinMode(swCruise, INPUT_PULLDOWN);
  // Smoothed values
  dRPM.begin(SMOOTHED_AVERAGE, 10);
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
  // set speed based on two switch states
  if (lowSpeed == true) {
    maxRPM = maxRPMLowSpeed;
  } else {
    maxRPM = maxRPMHighSpeed;
  }

  // switches
  switchesFunc();

  // throttle
  throttle = getThrottlePercent();

  // VESC read values
  vescRead();

  // Set RPM values
  setRpm();

  renderDisplay();

  delay(50);
}

void renderDisplay(){
  display.ssd1306_command(SSD1306_DISPLAYON);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(3);
  display.setCursor(0, 0);
  display.println(dRPM.get()*rpmTokmph, 0);
  display.setCursor(65, 0);
  display.setTextSize(1);
  display.print(F("set ("));
  display.print(throttle*100, 0);
  display.print(F("%)"));
  display.setCursor(60, 8);
  display.setTextSize(2);
  display.print(rpm*rpmTokmph, 0);
  display.setCursor(104, 11);
  display.setTextSize(1);
  display.println(F("km/h"));
  display.setCursor(0, 27);
  display.setTextSize(2);
  display.print(dV.get(), 1);
  display.print(F("V "));
  display.print(dC.get(), 0);
  display.print(F("A"));
  display.setTextSize(1);
  display.setCursor(0, 47);
  display.print(dAh, 0);
  display.print(F("mAh  "));
  display.print(dTv, 0);
  display.print(F("C "));
  display.print(dRPM.get());
  display.print(F(" "));
  display.setTextSize(1);
  display.setCursor(0, 57);
  if (allOff == true) {
    display.print(F("OFF "));
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
  else if (cruise == true) {
    display.print(F("CRUISE "));
  }
  else if (lowSpeed == true) {
    display.print(F("LOW "));
  }
  display.print(dP.get(), 0);
  display.print(F("W "));
  display.display();      // Show initial text
}

void switchesFunc(){
  lowSpeedSwitch = digitalRead(swLowSpeed);
  if (lowSpeedSwitch == 1){
    lowSpeed = true;
  }else if (lowSpeedSwitch == 0){
    lowSpeed = false;
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
  if (percentage <= 5.0) {
    return 0.0;
  }
  else if (percentage >= 98.0){
    return 1.0;
  }
  else {
    return percentage/100.0;
  }
}


void setRpm(){
  if (cruise != lastCruiseState) {
    if (cruise == true){
      rpm = throttle * maxRPM;
    }
  }

  if (allOff == true){
    rpm = 0;
  }
  else if (cruise == false) {
    rpm = throttle*maxRPM;
  }
  lastCruiseState = cruise;
  UART.setRPM(rpm);
}
