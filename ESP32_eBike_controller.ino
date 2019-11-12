#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <VescUart.h>
#include <TinyGPS++.h>
#include <PushButton.h>
#include <Smoothed.h>

//Inits
#define sw1 27
int sw1State, potPin = 34, throttle = 0, maxRPM = 16000, rpm = 0;
Smoothed <int> rpmRead;
// int rpmInt = 150, rpmSens = 50, prevRPM = 0;

// Safety init
bool safetyInit = false, safetyMax = false, safetyMin = false;

// Buttons
#define BUTTON_UP 12
#define BUTTON_MID 13
#define BUTTON_DOWN 14
#define LED 2

PushButton buttonUp(BUTTON_UP);
PushButton buttonMid(BUTTON_MID);
PushButton buttonDown(BUTTON_DOWN);

// Touchpad
#define TOUCH 33
Smoothed <int> touchValue;


// LCD SDA 21 SCL 22
int lcdCurPage = 1;
int lcdPages = 2;
LiquidCrystal_I2C lcd(0x27, 20, 4);
char line1[21], line2[21], line3[21], line4[21];

// Thermistor variables (installed on motor)
int ThermistorPin = 4, Vo;
float R1 = 10000, logR2, R2, T, Tc, Tf, c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
Smoothed <int> motorTemp;

// The TinyGPS++ object
TinyGPSPlus gps;
// GPS UART
#define RXD1 5
#define TXD1 18
bool gpsOn = false;
char gpsDateTime[21];
float longitude = 0.0000, latitude = 0.0000;
int gpsSats = 0, gpsAvailable = 0;
Smoothed <float> gpsSpeed;

// VESC UART
VescUart UART;
#define RXD2 35
#define TXD2 19
Smoothed <int> dRPM;
Smoothed <float> dC;
Smoothed <float> dP;
Smoothed <float> dV;
float dTv = 0.0, dTm = 0.0, dAhC = 0.0, dAh = 0.0, rpmTokmph = 0.00133333333333;
bool noBreak = false;


void setup() {
  // LCD
  lcd.init();
  lcd.backlight();
  Serial.begin(115200); // Serial terminal
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // UART VESC
  Serial1.begin(9600, SERIAL_8N1, RXD1); // GPS
  UART.setSerialPort(&Serial2);

  pinMode(TOUCH, INPUT);
  pinMode(ThermistorPin, INPUT);

  // switches
  pinMode(sw1, INPUT_PULLDOWN);
  pinMode(BUTTON_UP, INPUT_PULLDOWN); // Set button as input
  pinMode(BUTTON_MID, INPUT_PULLDOWN); // Set button as input
  pinMode(BUTTON_DOWN, INPUT_PULLDOWN); // Set button as input
  pinMode(LED, OUTPUT); // Set onboard LED as output

  // Smoothed values
  touchValue.begin(SMOOTHED_AVERAGE, 4);
  motorTemp.begin(SMOOTHED_AVERAGE, 20);
  rpmRead.begin(SMOOTHED_AVERAGE, 8);
  gpsSpeed.begin(SMOOTHED_AVERAGE, 10);
  dRPM.begin(SMOOTHED_AVERAGE, 15);
  dC.begin(SMOOTHED_AVERAGE, 15);
  dP.begin(SMOOTHED_AVERAGE, 15);
  dV.begin(SMOOTHED_AVERAGE, 15);
  dRPM.add(0);  // avoid a division error
}

void loop() {
  // Buttons
  buttonUp.update();
  buttonMid.update();
  buttonDown.update();
  
  // Safety startup
  safetyStart();
  
  // Thermistor
  tempCalculator();
  
  // Buttons
  buttonsFunc();
  
  // switches
  switchesFunc();

  // VESC read values
  vescRead();

  // Receive GPS values
  gpsFunc();

  // Set RPM values
  setRpm();
  
  // Print things on LCD
  processPage(lcdCurPage);
}

void safetyStart(){
  while (safetyInit == false) {
    throttle = map(analogRead(potPin), 0, 4095, 0, maxRPM);
    processPage(10);  
    // wait for max min on throttle
    if (safetyMax == false) {
      if (throttle > maxRPM - 100){safetyMax = true;}
    }
    else {
      if (throttle < 100){safetyMin = true;}
    }
    if (safetyMax == true && safetyMin == true){safetyInit = true;}
    delay(10);
  }
}

void tempCalculator(){
  Vo = analogRead(ThermistorPin);
  lcd.setCursor(14, 3);
  R2 = R1 * (4095.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  motorTemp.add(Tc);
}

void buttonsFunc(){
  if (buttonUp.isClicked()) // Click event
  {
    if (lcdCurPage == lcdPages){
      lcdCurPage = 1;
    }else{
      lcdCurPage++;
    }
  }
  if (buttonMid.isClicked()) // Click event
  {
    Serial.println("Button MIDDLE clicked!");
  }
  if (buttonDown.isClicked()) // Click event
  {
    if (lcdCurPage == 1){
      lcdCurPage = lcdPages;
    }else{
      lcdCurPage--;
    }
  }
  if (buttonUp.isActive() || buttonDown.isActive() || buttonMid.isActive())
  {
    digitalWrite(LED, HIGH);
  }
  else
  {
    digitalWrite(LED, LOW);
  }
}

void switchesFunc(){
  sw1State = digitalRead(sw1);

  if (sw1State == 1){
    noBreak = true;
  }else if (sw1State == 0){
    noBreak = false;
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

void setRpm(){
  touchValue.add(touchRead(TOUCH));
  throttle = analogRead(potPin);

  if (noBreak == true){
    UART.setBrakeCurrent(0);
    rpmRead.add(0);
  }
  else if (touchValue.get() < 10){
    rpmRead.add(0);
    UART.setRPM(rpmRead.get());
  }
  else {
    rpmRead.add(map(throttle, 0, 4095, 0, maxRPM));
    UART.setRPM(rpmRead.get());
  }
}

void gpsFunc(){
  while (Serial1.available() > 0){
    if (gps.encode(Serial1.read())){
      sprintf(gpsDateTime, "%04d.%02d.%02d  %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
      longitude = gps.location.lng();
      latitude = gps.location.lat(); 
      gpsSats = gps.satellites.value();
      gpsAvailable = gps.location.isValid()?1:0;

      // Speed averaging
      gpsSpeed.add(gps.speed.kmph());
    }
  }
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

void processPage(int pageNum){
  switch (pageNum){
    case 1:
      sprintf(line1, "%-5s%-5s%-4s%-3s%-3s", "Volt", "Curr",  "Pow", "Tv", "Tm"); 
      sprintf(line2, "%-5s%-5s%-4s%-3s%3d", String(dV.get(), 1), String(dC.get(), 1), String(dP.get(), 0), String(dTv, 0), motorTemp.get()); 
      sprintf(line3, "%-4s%-4s%-4s%-4s%-4s", "RPM", "set", "vSp", "gSp", "Brea"); 
      sprintf(line4, "%-4s%-4s%-4s%-4s%-3s", String(dRPM.get()/100., 0), String(rpmRead.get()/100., 0), String((float)dRPM.get()*rpmTokmph, 0), String(gpsSpeed.get(), 0), noBreak? "OFF":"ON"); 
      break;
    case 2:
      sprintf(line1, "%-20s", gpsDateTime); 
      sprintf(line2, "%-3s %-7s %-7s", gpsAvailable? "GPS":"N/A", String(latitude, 4), String(longitude, 4)); 
      sprintf(line3, "%-5s%-5s%-5s%-3s", "mAh", "-mAh", "Sat", "Touch"); 
      sprintf(line4, "%-5s%-5s%-5d%3d", String(dAh*1000, 0), String(dAhC*1000, 0), gpsSats, touchValue.get()); 
      break;
    case 10:
      sprintf(line1, "%-12s", "SAFETY LOCK"); 
      sprintf(line2, "%-4s%-6s%-4s%-6s", "MAX", (String)safetyMax, "MIN", (String)safetyMin); 
      sprintf(line4, "Throttle: %-7s", String(rpm/(float)maxRPM*100, 0)); 
  }
  printOnLcd();
}