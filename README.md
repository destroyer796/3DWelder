# 3DWelder
This is an overbuilt 3D pen designed to be used for "welding" 3D printed parts together using 1.75mm filament. This project is funded by [Hack Club](https://hackclub.com/) as a part of the [Highway](https://highway.hackclub.com/) program.

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/3DWelderRender.png">

This design uses a CHC Pro hotend and a custom Orbiter extruder adapted to use a DC motor. This DC motor's speed is controlled by a drill trigger adapted into the handle. It has a small screen and a rotary encoder for adjusting and showing the temperature target. The whole thing is powered by a drill battery to make it more portable and easier to use. This project is currently completely untested.

## Why?
So, why did I choose to make this ridiculous thing anyways? Well, the best way to join 3D printed parts together if they can't be screwed is by literally joining them together with the same plastic they're made of. This can be very difficult with a normal 3D pen though, as they're quite low powered and actually can't even melt many higher temperature plastics. I wanted a solution to this, so I decided to basically make a regular 3D printer extruding setup, but make it portable. It has some other uses aswell, if you use EVA filament, it could function essentially as a hot glue gun. It could also theoretically be used as a 3D pen, but not that well.

## PCB
<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/3DWelderSchematic.PNG">
<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/3DWelderPCB.PNG">

The pcb was designed using KiCad, and it is only 2 layers. The pcb controlls everything except for voltage control and the motor, which is controlled by the drill trigger. It uses screw terminals and jst-xh connections. The brains of the whole thing is an Arduino Nano.

## Wiring/Assembly
<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-183DGunDiagram.png">

All 3D printed parts screw together using M3 heatset inserts and various M3/M4 screws. For wiring, 2 spade connectors are used as terminals on the battery. From there, there's 2 ways of wiring everything else. You can either put the buck converter inside the space above the battery, then have wires coming off of that to the drill trigger and pcb, or have the buck converter up by the pcb, then have wires going back down to the drill trigger. Having the Buck converter in the handle is much cleaner, but may be very difficult to fit inside that small space.

## Code
<details>
  <summary>
    Arduino Code
  </summary>
  
  ~~~
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
  ~~~

</details>

This is the firmware to be uploaded to the Arduino Nano. It's purpose is for PID control of the heater via the thermistor readings and mosfet. It has temperature control via the EC11 rotary encoder, and the temperature target and current temperature are displayed on the 0.91 inch OLED screen.

## BOM

| Item | Quantity | Price | Source | Notes | Owned/Buying |
|--------|--------|--------|--------|--------|--------|
| Drill Trigger | 1 | $3.57 | [Aliexpress](https://www.aliexpress.com/item/3256806562336203.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%209.39%21USD%203.57%21%21USD%203.57%21%21%21%402103146c17528271550647744ef8ea%2112000038176249576%21ct%21US%212965353747%21%211%210&_gl=1*1qs30jx*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | For regulating motor | Buying |
| CHC Pro | 1 | $22.42 | [Aliexpress](https://www.aliexpress.com/item/3256806914810913.html?spm=a2g0o.cart.0.0.755738daw2eCeF&mp=1&pdp_npi=5%40dis%21USD%21USD%2049.83%21USD%2022.42%21%21USD%2022.42%21%21%21%402101e07217528229181472693e61f9%2112000045551090949%21ct%21US%212965353747%21%211%210&_gl=1*1s341fi*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjM1OTkkajIwJGwwJGgw) | Hotend | Buying |
| Screw Terminal 4P | 1 | $2.50 | [Aliexpress](https://www.aliexpress.com/item/3256807282935438.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%202.59%21USD%202.50%21%21USD%202.50%21%21%21%402103146c17528271550647744ef8ea%2112000040883353771%21ct%21US%212965353747%21%211%210&_gl=1*1qs30jx*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | 7.62mm pitch, Power inputs | Buying |
| Screw Terminal 2P | 1 | $1.87 | [Aliexpress](https://www.aliexpress.com/item/3256807282935438.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%201.94%21USD%201.87%21%21USD%201.87%21%21%21%402103146c17528271550647744ef8ea%2112000040883353769%21ct%21US%212965353747%21%211%210&_gl=1*1qs30jx*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | 7.62mm pitch, Heater outputs | Buying |
| 2510 24V Fan | 1 | $2.08 | [Aliexpress](https://www.aliexpress.com/item/3256807251337149.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%202.18%21USD%202.08%21%21USD%202.08%21%21%21%402103146c17528271550647744ef8ea%2112000040754390185%21ct%21US%212965353747%21%211%210&_gl=1*1qs30jx*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | Cools hotend | Owned |
| EC11 Rotary Encoder | 1 | $3.08 | [Aliexpress](https://www.aliexpress.com/item/3256805796819763.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%203.08%21USD%203.08%21%21USD%203.08%21%21%21%402103146c17528271550647744ef8ea%2112000035172713579%21ct%21US%212965353747%21%211%210&_gl=1*1gfgklc*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | For controlling temp | Buying |
| I2C OLED | 1 | $1.85 | [Aliexpress](https://www.aliexpress.com/item/3256806179530924.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%204.03%21USD%201.85%21%21USD%201.85%21%21%21%402103146c17528271550647744ef8ea%2112000036911966887%21ct%21US%212965353747%21%211%210&_gl=1*1gfgklc*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | 0.91in, Displays target/current temp | Buying |
| TO-220 Radiator | 1 | $1.94 | [Aliexpress](https://www.aliexpress.com/item/2255800003999447.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%202.01%21USD%201.94%21%21USD%201.94%21%21%21%402103146c17528271550647744ef8ea%2110000000710306628%21ct%21US%212965353747%21%211%210&_gl=1*1gfgklc*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | Cools mosfet | Buying |
| IRLZ34NPBF Mosfet | 1 | $3.20 | [Aliexpress](https://www.aliexpress.com/item/3256808552158121.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%206.97%21USD%203.20%21%21USD%203.20%21%21%21%402103146c17528271550647744ef8ea%2112000046468959878%21ct%21US%212965353747%21%211%210&_gl=1*1gfgklc*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | TO-220, Regulates hotend heater | Buying |
| 24V-12V Buck Converter | 1 | $4.12 | [Aliexpress](https://www.aliexpress.com/item/3256805319593185.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%204.12%21USD%204.12%21%21USD%204.12%21%21%21%402103146c17528271550647744ef8ea%2112000033343621926%21ct%21US%212965353747%21%211%210&_gl=1*1gfgklc*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | Converts battery 24V to 12V for Arduino | Buying |
| DC Motor | 1 | $13.35 | [Aliexpress](https://www.aliexpress.com/item/3256807142640791.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%2029.02%21USD%2013.35%21%21USD%2013.35%21%21%21%402103146c17528271595267798ef8ea%2112000040287640067%21ct%21US%212965353747%21%211%210&_gl=1*1djn14*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | 24V, Worm gear, 27rpm, drives extruder | Buying |
| Kobalt 24V Battery | 1 | $28.35 | [Aliexpress](https://www.aliexpress.com/item/3256808341351094.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%2098.74%21USD%2028.35%21%21USD%2028.35%21%21%21%402103146c17528271595267798ef8ea%2112000045570945299%21ct%21US%212965353747%21%211%210&_gl=1*1djn14*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | Any Ah, mines 6.0Ah, Powers whole thing | Buying |
| 100k Resistor | 1 | $1.27 | [Aliexpress](https://www.aliexpress.us/item/3256805491339263.html?spm=a2g0o.productlist.main.2.5acb113dWCWHEi&algo_pvid=d885d319-e63c-46cb-9890-1f3fe4b1ec6b&algo_exp_id=d885d319-e63c-46cb-9890-1f3fe4b1ec6b-1&pdp_ext_f=%7B%22order%22%3A%221339%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21USD%211.88%211.26%21%21%2113.46%219.02%21%402103247917528275691891949e5c61%2112000033982954181%21sea%21US%212965353747%21X&curPageLogUid=0ChrWbtCVhkG&utparam-url=scene%3Asearch%7Cquery_from%3A) | For thermistor logic | Owned |
| 10k Resistor | 1 | $1.34 | [Aliexpress](https://www.aliexpress.us/item/3256805491339263.html?spm=a2g0o.productlist.main.2.5acb113dWCWHEi&algo_pvid=d885d319-e63c-46cb-9890-1f3fe4b1ec6b&algo_exp_id=d885d319-e63c-46cb-9890-1f3fe4b1ec6b-1&pdp_ext_f=%7B%22order%22%3A%221339%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21USD%211.88%211.26%21%21%2113.46%219.02%21%402103247917528275691891949e5c61%2112000033982954181%21sea%21US%212965353747%21X&curPageLogUid=0ChrWbtCVhkG&utparam-url=scene%3Asearch%7Cquery_from%3A) | Heater gate pulldown | Owned |
| 220R Resistor | 1 | $1.35 | [Aliexpress](https://www.aliexpress.us/item/3256805491339263.html?spm=a2g0o.productlist.main.2.5acb113dWCWHEi&algo_pvid=d885d319-e63c-46cb-9890-1f3fe4b1ec6b&algo_exp_id=d885d319-e63c-46cb-9890-1f3fe4b1ec6b-1&pdp_ext_f=%7B%22order%22%3A%221339%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21USD%211.88%211.26%21%21%2113.46%219.02%21%402103247917528275691891949e5c61%2112000033982954181%21sea%21US%212965353747%21X&curPageLogUid=0ChrWbtCVhkG&utparam-url=scene%3Asearch%7Cquery_from%3A) | Heater gate drive | Owned |
| 2 Pin JST XH Female | 2 | $2.37 | [Aliexpress](https://www.aliexpress.com/item/3256803235887618.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%202.49%21USD%202.37%21%21USD%202.37%21%21%21%402103146c17528270600956401ef8ea%2112000025716082487%21ct%21US%212965353747%21%211%210&_gl=1*1djn14*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | For fan and thermistor | Owned |
| 18 AWG Wire | ~2 meters | $2.11 | [Aliexpress](https://www.aliexpress.com/item/3256806379805687.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%202.20%21USD%202.11%21%21USD%202.11%21%21%21%402103146c17528270600956401ef8ea%2112000037691464120%21ct%21US%212965353747%21%211%210&_gl=1*1djn14*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | Wiring everything | Owned |
| Arduino Nano | 1 | $2.94 | [Aliexpress](https://www.aliexpress.com/item/3256806587205161.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%202.94%21USD%202.94%21%21USD%202.94%21%21%21%402103146c17528270600956401ef8ea%2112000038256018925%21ct%21US%212965353747%21%211%210&_gl=1*6iw3e9*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | Type C, Microprocessor | Buying |
| M3 Heatset Inserts | ~18 | $2.05 | [Aliexpress](https://www.aliexpress.com/item/3256803396040989.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%202.17%21USD%202.05%21%21USD%202.05%21%21%21%402103146c17528270600956401ef8ea%2112000026370649758%21ct%21US%212965353747%21%211%210&_gl=1*6iw3e9*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | M3x5x4, Gives 3D printed parts threads | Owned |
| M3x6 Screws | ~11 | $2.04 | [Aliexpress](https://www.aliexpress.com/item/3256806983352954.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%204.09%21USD%202.04%21%21USD%202.04%21%21%21%402103146c17528270600956401ef8ea%2112000039685363236%21ct%21US%212965353747%21%211%210&_gl=1*6iw3e9*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | For mounting pcb, battery terminal, and handle | Owned |
| M3x8 Screws | ~2 | $2.14 | [Aliexpress](https://www.aliexpress.com/item/3256806983352954.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%204.24%21USD%202.12%21%21USD%202.12%21%21%21%402103146c17528270600956401ef8ea%2112000039685363239%21ct%21US%212965353747%21%211%210&_gl=1*6iw3e9*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | For Orbiter mount | Owned |
| M3x10 Screws | ~4 | $1.38 | [Aliexpress](https://www.aliexpress.com/item/3256805692722422.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%201.42%21USD%201.38%21%21USD%201.38%21%21%21%402103146c17528270600956401ef8ea%2112000034679037237%21ct%21US%212965353747%21%211%210&_gl=1*6iw3e9*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | For mounting Hotend mount and hotend clamp | Owned |
| M4x8 Screws | 8 | $2.03 | [Aliexpress](https://www.aliexpress.com/item/3256806983352954.html?spm=a2g0o.cart.0.0.755738danhf7AI&mp=1&pdp_npi=5%40dis%21USD%21USD%204.05%21USD%202.03%21%21USD%202.03%21%21%21%402103146c17528270600956401ef8ea%2112000049313623910%21ct%21US%212965353747%21%211%210&_gl=1*1cp6qeh*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjcwNjAkajU5JGwwJGgw) | For motor mounting | Owned |
| Orbiter Gears | 1 | $23.32 | [Aliexpress](https://www.aliexpress.com/item/3256802185372532.html?spm=a2g0o.cart.0.0.755738daqLWaOW&mp=1&pdp_npi=5%40dis%21USD%21USD%2023.32%21USD%2023.32%21%21USD%2023.32%21%21%21%402103146c17528269270114546ef8ea%2112000020383149424%21ct%21US%212965353747%21%211%210&_gl=1*zzjcrw*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjY4NTgkajU3JGwwJGgw) | Extruder drive gears | Buying |
| Orbiter Handle Shell | 1 | $3.17 | [Aliexpress](https://www.aliexpress.com/item/3256802185372532.html?spm=a2g0o.cart.0.0.755738daqLWaOW&mp=1&pdp_npi=5%40dis%21USD%21USD%203.17%21USD%203.17%21%21USD%203.17%21%21%21%402103146c17528269270114546ef8ea%2112000020383149423%21ct%21US%212965353747%21%211%210&_gl=1*zzjcrw*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjY4NTgkajU3JGwwJGgw) | Extruder lever + shaft | Buying |
| Orbiter Handle Screw | 1 | $2.67 | [Aliexpress](https://www.aliexpress.com/item/3256802185372532.html?spm=a2g0o.cart.0.0.755738daqLWaOW&mp=1&pdp_npi=5%40dis%21USD%21USD%202.67%21USD%202.67%21%21USD%202.67%21%21%21%402103146c17528269270114546ef8ea%2112000020383149428%21ct%21US%212965353747%21%211%210&_gl=1*zzjcrw*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjY4NTgkajU3JGwwJGgw) | Extruder lever screw/spring | Buying |
| Alternative Full Orbiter | 1 | $30.30 | [Aliexpress](https://www.aliexpress.us/item/3256803143364574.html?spm=a2g0o.productlist.main.3.32711918kejPC1&algo_pvid=031a119a-d9e4-4ec0-9dbd-65a221c951d0&algo_exp_id=031a119a-d9e4-4ec0-9dbd-65a221c951d0-2&pdp_ext_f=%7B%22order%22%3A%2233%22%2C%22eval%22%3A%221%22%7D&pdp_npi=4%40dis%21USD%2136.80%2130.30%21%21%2136.80%2130.30%21%40210337c117528255697113962ecb0f%2112000028794303599%21sea%21US%212965353747%21X&curPageLogUid=faCfA8m1JzG1&utparam-url=scene%3Asearch%7Cquery_from%3A) | Around same price as above but full kit, worse parts | NOT Buying |
| PTFE Tube | 1 | $1.83 | [Aliexpress](https://www.aliexpress.com/item/3256808595111724.html?spm=a2g0o.cart.0.0.755738daqLWaOW&mp=1&pdp_npi=5%40dis%21USD%21USD%201.90%21USD%201.83%21%21USD%201.83%21%21%21%402103146c17528268578343657ef8ea%2112000046644154648%21ct%21US%212965353747%21%211%210&_gl=1*14pyl8l*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjY4NTgkajU3JGwwJGgw) | Between extruder and hotend, need very little | Owned |
| Spade Terminals | 2 | $2.38 | [Aliexpress](https://www.aliexpress.com/item/3256802579044914.html?spm=a2g0o.cart.0.0.755738daqLWaOW&mp=1&pdp_npi=5%40dis%21USD%21USD%202.48%21USD%202.38%21%21USD%202.38%21%21%21%402103146c17528268578343657ef8ea%2112000022078614610%21ct%21US%212965353747%21%211%210&_gl=1*14pyl8l*_gcl_aw*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_dc*R0NMLjE3NTE4NDcxMzYuQ2owS0NRand2YWpEQmhDTkFSSXNBRUUyOVdvb1JYN0FnSzczYmNCVXN5VHRNRGlUUjV4QXJ0RzRDSVl6Tmx5ZEZ2alByYWdRQjczVklkSWFBc3FnRUFMd193Y0I.*_gcl_au*MTM0OTQyNjgzMy4xNzUwMTkyNzY4*_ga*MjcyMzA3OTUzLjE3NTI4MjM0NjE.*_ga_VED1YSGNC7*czE3NTI4MjM0NjEkbzEkZzEkdDE3NTI4MjY4NTgkajU3JGwwJGgw) | For battery connection | Owned |
| Custom PCB | 1 | $8.52 | [JLC PCB](https://jlcpcb.com/?from=VGSU&utm_source=google&utm_medium=cpc&utm_campaign=14179457750&gad_source=1&gad_campaignid=14179457750&gbraid=0AAAAABS1QqnKxzJxFraqirNeN0Jsk7SAi&gclid=CjwKCAjw4efDBhATEiwAaDBpbgJS_HEbmtEoijaZsA4da2X2VfovqTt3S1jdUBIx32Qqjq6h8wbXnBoC3nMQAvD_BwE) | For connecting all components |

Total(No shipping/taxes): $148.12   <br>
My Total: $157.54 (with shipping/taxes, excluding owned items)
