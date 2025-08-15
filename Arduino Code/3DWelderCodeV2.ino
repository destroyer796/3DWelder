#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>



#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C

#define THERMISTOR_PIN A0
#define RELAY_PIN 3

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);


//Define Variables we'll be connecting to
double Setpoint = 200; // initial target temperature
double Input, Output;

//Specify the links and initial tuning parameters
double Kp=3.0, Ki=0.05, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 300;

unsigned long windowStartTime;


const int EC11PinA = 5;
const int EC11PinB = 4;

volatile int encoderPos = 0;
int displayPos = 0;

// Track previous stable state
volatile uint8_t prevState = 0;

// Lookup table for quadrature encoder
const int8_t table[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};


const float SERIES_RESISTOR = 4700.0;    // bottom resistor = 470 Ω
const float R0 = 100000.0;              // thermistor nominal 100k @ 25°C
const float T0_K = 25.0 + 273.15;       // 25°C in Kelvin
const float B_COEFFICIENT = 4267.0;     // Beta value (good starting point for 104GT-2)

int readADCavg(int pin, int samples = 64) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return (int)(sum / samples);
}

  // Steinhart–Hart coefficients for Semitec 104GT‑2 / 104NT
const float A = 1.009249522e-3;
const float B = 2.378405444e-4;
const float C = 2.019202697e-7;


float getTemperatureC() {
  int adcValue = readADCavg(THERMISTOR_PIN, 128); // 128 samples gives good smoothing
  float adc = (float)adcValue;

  if (adc <= 2)  { Serial.println("ADC too low — check wiring!");  return NAN; }
  if (adc >= 1021){ Serial.println("ADC too high — check wiring!"); return NAN; }

  // For wiring: 5V -> thermistor -> A0 -> SERIES_RESISTOR -> GND
  float Rtherm = SERIES_RESISTOR * (1023.0 / adc - 1.0);

  // Beta formula: T = 1 / ( (1/B) * ln(R/R0) + 1/T0 )
  float lnR = log(Rtherm / R0);
  float invT = (1.0 / B_COEFFICIENT) * lnR + (1.0 / T0_K);
  float Tk = 1.0 / invT;
  float Tc = Tk - 273.15;

  // Debug output (compact)
  Serial.print("ADC=");
  Serial.print(adcValue);
  Serial.print(" R=");
  Serial.print(Rtherm, 1);
  Serial.print("Ω T=");
  Serial.print(Tc, 2);
  Serial.println("C");

  return Tc;
}


void setup() {
  Serial.begin(250000);
  pinMode(EC11PinA, INPUT_PULLUP);
  pinMode(EC11PinB, INPUT_PULLUP);
  // Initialize prevState
  prevState = (digitalRead(EC11PinA) << 1) | digitalRead(EC11PinB);

  // Enable pin change interrupts for pins 4 and 5
  PCICR |= (1 << PCIE2);        // Enable PCINT[23:16]
  PCMSK2 |= (1 << PCINT20);     // Pin 4
  PCMSK2 |= (1 << PCINT21);     // Pin 5
  pinMode(RELAY_PIN, OUTPUT);
  analogReference(DEFAULT); // Using 5V as ADC reference (adjust if using external)

  windowStartTime = millis();

  windowStartTime = millis();
  Setpoint = encoderPos;
  Input = 0; Output = 0;

  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetSampleTime(1000); // 1 s sample time
  myPID.SetMode(AUTOMATIC);

  Serial.println("Controller ready (fixed 5V top, 4.7kΩ bottom)");

  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Hello World!");
  display.display();
}

void loop() {
  noInterrupts();
  int pos = encoderPos;
  interrupts();

  if (pos != displayPos) {
    displayPos = pos;
    Serial.print("Position: ");
    Serial.println(displayPos);
  }
  Setpoint = displayPos;

  float temp = getTemperatureC();

  if (isnan(temp)) return; // skip PID if bad reading
  Input = temp;

  myPID.Compute();

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize) { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > millis() - windowStartTime) {
    digitalWrite(RELAY_PIN, HIGH);
    //Serial.println("Heater ON");
  } else {
    digitalWrite(RELAY_PIN, LOW);
    //Serial.println("Heater OFF");
  }

  bool heaterOn = (Output > (millis() - windowStartTime));

  // Debug: see what PID is doing
  double error = Setpoint - Input;
  //Serial.print("SP="); Serial.print(Setpoint,1);
  //Serial.print(" PV="); Serial.print(Input,1);
  //Serial.print(" ERR="); Serial.print(error,1);
  //Serial.print(" OUT(ms)="); Serial.print(Output,0);
  //Serial.print(" HEATER="); Serial.println(heaterOn ? "ON" : "OFF");

  updateDisplay();
}


ISR(PCINT2_vect) {
  uint8_t MSB = digitalRead(EC11PinA);
  uint8_t LSB = digitalRead(EC11PinB);
  uint8_t state = (MSB << 1) | LSB;

  // Combine previous and current state to get table index
  uint8_t index = (prevState << 2) | state;
  int8_t movement = table[index];

  // Only count full detents (+/- 4 quadrature steps)
  static int stepCount = 0;
  stepCount += movement;

  if (stepCount >= 4) {
    encoderPos += 5; // CW
    stepCount = 0;
  } else if (stepCount <= -4) {
    encoderPos -= 5; // CCW
    stepCount = 0;
  }

  prevState = state;
}


void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,8);
  display.println(int(Input));
  display.setCursor(64,8);
  display.println(int(Setpoint));
  display.display();
}






