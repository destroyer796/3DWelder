#include <PID_v1.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Wire.h>

//Defines screen width/height
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

//Not using a reset pin
#define OLED_RESET -1

//Creates a display named display, gives it parameters
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// EC11 code
const int EC11PinA = 8;
const int EC11PinB = 7;
volatile in encoderPos = 0;
in lastEncoded = 0;

/* -----------  USER SETTINGS  ----------- */
const int   thermistorPin   = A0;
const int   heaterPin       = 6;     // PWM pin
const int   statusLedPin    = 13;    // Built‑in LED on most Arduinos
const float R_FIXED         = 100000.0;   // 100 kΩ divider resistor

// Steinhart–Hart coefficients for Semitec 104GT‑2 / 104NT
const float A = 1.009249522e-3;
const float B = 2.378405444e-4;
const float C = 2.019202697e-7;

// PID tuning
double Kp = 4.0, Ki = 0.3, Kd = 12.0;

/* Temperature targets */
double setpointC   = 60.0;   // Desired temperature
const double MAX_TEMP_C      = 300.0;   // Hard safety limit
const double MIN_VALID_VOLT  = 0.10;   // Thermistor‑open detection

/* -----------  GLOBALS  ----------- */
double inputC, outputPWM;          // For PID library
PID myPID(&inputC, &outputPWM, &setpointC, Kp, Ki, Kd, DIRECT);

enum FaultState { OK, OVER_TEMPERATURE, SENSOR_ERROR };
FaultState fault = OK;

/* Timing helpers */
unsigned long lastBlink   = 0;
bool          ledState    = false;

void setup() {
  //Start I2C
  Wire.begin();

  // Initializes display using I2C address 0x3C, if it fails it prints error message
  // Switchcapvcc thing tells it that display has internal voltage regulator
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.begin(9600);
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  //Display test commands
  display.clearDisplay(); // Clears display
  display.setTextSize(1); // Sets text size, 1 = 6/8 pixels, 5/7 without spacing, 2 = 2x, 3 = 3x
  display.setTextColor(SSD1306_WHITE); // Set white text
  display.setCursor(0,0); // Sets cursor to top-left
  display.println(F("Hello SH-S091!")); // Print message to display buffer
  display.display(); // Push everything in buffer to scren
  // display.setFont(), allows you to change fonts, need to install them
  // display.fillRect(x, y, width, height, SSD1306_BLACK) draws a black rect, way to clear part of screen

  //EC11 Code
  pinMode(EC11PinA, INPUT_PULLUP);
  pinMode(EC11PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EC11PinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EC11PinB), updateEncoder, CHANGE);


  pinMode(heaterPin,   OUTPUT);
  pinMode(statusLedPin, OUTPUT);
  Serial.begin(9600);

  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(250);          // 4 Hz PID updates
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //EC11 Code
  static int lastPos = 0;
  if (encoderPos != lastPos) {
    Serial.print("Value: ");
    Serial.println(encoderPos);

    display.fillRect(0, 0, 64, 32, SSD1306_BLACK);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F(encoderPos));
    display.display

    lastPos = encoderPos;
  }


  /* ---------- Read thermistor ---------- */
  int   adc = analogRead(thermistorPin);
  float v   = adc * 5.0 / 1023.0;

  /* Sensor fault check */
  if (v < MIN_VALID_VOLT)  fault = SENSOR_ERROR;

  /* Convert to °C if no sensor fault */
  if (fault == OK) {
    double R = R_FIXED * (5.0 / v - 1.0);
    double lnR = log(R);
    double tempK = 1.0 / (A + B*lnR + C*lnR*lnR*lnR);
    inputC = tempK - 273.15;

    /* Over‑temperature check */
    if (inputC >= MAX_TEMP_C) fault = OVER_TEMPERATURE;
  }

  /* ---------- Safety Handling ---------- */
  if (fault != OK) {
    myPID.SetMode(MANUAL);       // Freeze PID
    outputPWM = 0;               // Heater OFF
    analogWrite(heaterPin, 0);
  } else {
    myPID.Compute();             // Let PID calculate outputPWM
    analogWrite(heaterPin, (int)outputPWM);
  }

  /* ---------- Status LED ----------
     • ON  …… actively heating (PWM > 0)
     • Slow blink …… at set‑point (PID output ~0)
     • Fast blink …… fault
  ---------------------------------- */
  unsigned long now = millis();
  unsigned long blinkPeriod =
      (fault != OK)          ? 150 :        // fast blink on fault
      (outputPWM < 2)        ? 800 :        // slow blink at set‑point
                               0;           // solid ON while heating

  if (blinkPeriod == 0) {
    digitalWrite(statusLedPin, HIGH);
  } else if (now - lastBlink >= blinkPeriod) {
    ledState = !ledState;
    digitalWrite(statusLedPin, ledState);
    lastBlink = now;
  }

  /* ---------- Serial Monitor ---------- */
  Serial.print("Temp: ");
  Serial.print((fault == OK) ? inputC : NAN);
  Serial.print(" °C | PWM: ");
  Serial.print(outputPWM);
  Serial.print(" | Fault: ");
  Serial.println(
      (fault == OK) ? "None" :
      (fault == OVER_TEMPERATURE) ? "Over-Temp" : "Sensor Error");


  // Shows temp on display
  display.fillRect(64, 0, 64, 32, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(64,0);
  display.println(F(inputC));
  display.display
}

//EC11 code, runs everytime a change in position is detected
void updateEncoder() {
  int MSB = digitalREad(EC11PinA);
  int LSB = digitalRead(EC11PinB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncocded << 2) | encoded;

  //Clockwise
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos += 5;
  }
  //Counter-Clockwise
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos -= 5;
  }

  lastEncoded = encoded
}
