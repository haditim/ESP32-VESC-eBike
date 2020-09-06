#include <Wire.h>
#include <VescUart.h>
#include <PushButton.h>
#include <Smoothed.h>

//Inits
#define swBrake 27
#define swAllOff 14
#define swCruise 26
int sw1State, brakeState, allOffState, cruiseState;
int potPin = 34, maxRPM = 16000, rpm = 0;
float throttle = 0.0;
Smoothed <int> rpmRead;
// int rpmInt = 150, rpmSens = 50, prevRPM = 0;

// Safety init
bool safetyInit = false, safetyMax = false, safetyMin = false;

// Touchpad
#define TOUCH 33
Smoothed <int> touchValue;


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

  pinMode(TOUCH, INPUT);
  pinMode(ThermistorPin, INPUT);

  // switches
  pinMode(swBrake, INPUT_PULLDOWN);
  pinMode(swAllOff, INPUT_PULLDOWN);
  pinMode(swCruise, INPUT_PULLDOWN);

  // Smoothed values
  touchValue.begin(SMOOTHED_AVERAGE, 4);
  motorTemp.begin(SMOOTHED_AVERAGE, 20);
  rpmRead.begin(SMOOTHED_AVERAGE, 8);
  dRPM.begin(SMOOTHED_AVERAGE, 15);
  dC.begin(SMOOTHED_AVERAGE, 15);
  dP.begin(SMOOTHED_AVERAGE, 15);
  dV.begin(SMOOTHED_AVERAGE, 15);
  dRPM.add(0);  // avoid a division error
}

void loop() {
  // Thermistor
  tempCalculator();

  // throttle
  throttle = getThrottlePercent();

  // switches
  switchesFunc();

  // VESC read values
  vescRead();

  // Set RPM values
  setRpm();
  delay(100);
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
  if (allOff == true) {
    /* Serial.println("All off"); */
  }
  else if (cruise == true) {
    /* Serial.println("Cruise On"); */
  }
  else {
    /* Serial.println("Center"); */
  }

  if (cruise != lastCruiseState) {
    Serial.println("Cruise state changed");
    if (cruise == true){
      rpm = throttle * maxRPM;
    }
  }

  if (allOff == true){
    Serial.print("All off ");
    Serial.println();
    UART.setBrakeCurrent(0);
  }
  else if (cruise == true){
    Serial.print("Cruise ");
    Serial.print(rpm);
    Serial.println();
    UART.setRPM(rpm);
  }
  else if (noBrake == true){
    Serial.print("noBrake ");
    Serial.print(throttle);
    Serial.println();
    UART.setDuty(throttle);
  }
  else {
    Serial.print("Brake ");
    Serial.print(throttle*maxRPM);
    Serial.println();
    UART.setRPM(throttle*maxRPM);
  }
  lastCruiseState = cruise;
}
