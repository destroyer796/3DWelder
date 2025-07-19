---
title: "3DWelder"
author: "Liam Newbill"
description: "An overbuilt 3D pen using a CHC Pro and custom dc motor extruder powered by a power tool battery."
created_at: "2025-7-3"
---

## July 3rd: Pretty much just planning and some research for parts I could use (2.5 hours)
I did a lot of research into using a stepper motor powered extruder like a 3d printer but I realized that it would probably be simpler and cheaper to use a dc motor instead for this use case, I'm planning on using a 12v dc motor now. 
I'm thinking I'll use a 12v Dewalt battery or something and power this motor and an arduino nano off of it, I might add a boost converter to get 24v for the hotend or just run with 12v if I can get enough power. I need to do more research on that.
I think I'll do like a hot glue gun design with a trigger to vary the speed of the motor, and add a screen and dial for setting the temp.

[Motor](https://www.amazon.com/Greartisan-Geared-Turbine-Reduction-JSX330-370/dp/B0721W14ZN/ref=sr_1_1_sspa?crid=BVQR541YGE08&dib=eyJ2IjoiMSJ9.oLpiQnpHtL-iijUgXmggbt_SvO7hYySToax_XGpDeLkfFn2bHxRKIfLfuPBssr_R_k7cdtel90G_L7utPMuTbUV14ISLVm8h2fp54XC--dKuJBjgMzX_s6-z8TtYZV-H3nkxiFmWx4oEOBn7_fDCv_bDNUpMzZRVYODyPTjY5bYlZtv_M10yP7OoldZfhLQk8tCjm8eXxAFSSlDhC6_9fKfoNHYQJ5ctz5mGXOHPFHmyxhDLtOpcIiH_vWJblV4JCW4QLWGQymfdHmJrWmVovV3qiar1aBt6uVYWmtaIpS0.AJz89m5LABnsaEO1EdVASL3rXrM84tjpxKPGArhlyLQ&dib_tag=se&keywords=12v%2Bgeared%2Bdc%2Bmotor&qid=1751592427&sprefix=12v%2Bgeared%2Bdc%2Bmotor%2Caps%2C108&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9hdGY&th=1)

## July 5th: Schematic Design and parts research (3 hours)
Today I fleshed out the whole electronics setup and made the schematic. I also researched thermal management solutions. My plan is to have a small heatsink on the mosfets and use a buck converter to get 12v from the 18v battery, then drive the hotend directly off the battery voltage.
I am now planning to use a 0.91 inch OLED display and EC11 rotary encoder for the temperature selection. I'll use the screen to display the temp, dial to adjust it and press the dial to put the gun to sleep, like keep the temp lower but not fully shut off.

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-6Schematic.PNG"/>

## July 6th/7th: Rethought schematic some, lots of CAD work, PCB design completed, got a semi-final BOM (12 hours)--Through midnight
This was a mega session, I have basically everything done except for the CAD, I still have to figure out how to do the battery though. This was some quite challenging CAD work, as it's a lot of weird parts and packaging is very challenging. This thing is going to be way bigger than I'd like, partially because the motor I chose is massive, but it's too late now to change it. I removed the mosfet control for the motor, that's now just done off of the drill trigger. It turns out, drill triggers are actually all in one speed controllers. For the extruder design, I've adapted the LDO Orbiter to work with the DC motor. I essentially cut out the gearing entirely so the motor just attaches straight to the drive gear. Unfortunately there wasen't really any good place to put the pcb, so there's just gonna be a huge box taking up most of the front, it's not elegant at all, but nothing about this thing is, so whatever. I'm really not looking forward to finishing the case and making the handle. Fitting that trigger in is going to be very difficult, as it's a weird geometry and I'm horrible at organic modelling. I can't decide if I want to just buy a battery adapter or make my own, the problem with buying one is I can't find any dimensions to mount it, so I think I'm just gonna have to figure out my own adapter. I'm regretting my choice of screw terminals for a lot of the connections, as they take up a ton of space. I'd like to keep them, but I might end up redesigning the pcb if I get tired of having that huge box.

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-7Schematic.PNG"/>

New schematic

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-7PCB.PNG"/>

PCB

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-7CAD.PNG"/>

CAD

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-7Cart.PNG"/>

Current BOM

## July 8/9th: Went through midnight again. Corrected schematic some, redid some of pcb routing, lots of CAD work (7 hours)
Adjusted the schematic because I realized I setup the diode wrong. I also redid some of the pcb design and added thicker traces for the power supply. Most of the time was spend on CAD, I created a battery mounting system, modelled the handle and mounting for the handle, added pcb mounting, and added a cooling fan mount for the extruder heatsink. The handle was quite a pain, but I think it turned out alright. I kept running into headaches because of my poor CAD habits, like not putting sketches under their proper component. This led to a lot of redoing stuff because I had to adjust things. I really need to work on keeping an organized timeline too, I kinda like to work without it but I should definately put more effort to learning Fusion stuff more. Anyways, I'm basically done with hardware now. The only thing left(hopefully) is adding a cosmetic shell, and coding.

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-9Schematic.PNG"/>

Schematic

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-9PCB.PNG"/>

PCB

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-9CAD.PNG"/>

CAD

## July 9th: Corrected one thing on schematic/pcb and wrote code (2.5 hours)
Today was mostly just coding, I wrote code for the pid control of the heater and got the rotary encoder stuff. I still need to do the screen and wrap it all together. I tried simulating some stuff in tinkercad to test the code but I couldn't figure out anything good for pid control testing so I gave up on that. I also just had to change the input pin for the thermistor and added some stuff to the schematic to represent the battery and buck converter. I'll add images of that in a future session when it's done.

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-9Code1.PNG"/>
<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-9Code2.PNG"/>
Code

## July 17/18th: Went through midnight. Corrected screen pins to Arduino, small CAD detail coded screen, and BOM (6 hours)
Lots of small changes today, I realized the Oled was connected to the wrong pins on the arduino so I had to correct that. I added a little slope to the handle to route the wires through, and tried to work towards an enclosure but kinda gave up because I can't think of a good way to leave the extruder exposed enough, while still looking ok, plus I think it's cool exposed. Most of the time was spend on coding, I added code for the screen and updating it and stuff. It's pretty rough rn since I don't have anything to test it on, but it should work in theory. I made the whole BOM, everything is sourced from Aliexpress. It's around $150 for the whole thing, which I think is quite good, considering that included a lot of hardware most people probably have.

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-18Schematic.PNG">

Schematic

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-18PCB.PNG">

PCB

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-18CAD.PNG">

CAD


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

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-18BOM.PNG">

BOM

## July 18th: Got all the documentation set up, added a knob, and made a small correction to the PCB. (3 hours)
Today was mostly spend getting all the documentation ready for submission. During this process I noticed a couple things to correct. I added a knob to the rotary encoder, and fixed an issue with the DRC on the PCB. For documentation, I wrote basically the whole README, got final images/renders uploaded, made a wiring diagram, uploaded CAD files and code, and licenced the Github with MIT license.

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/7-183DGunDiagram.png">

Wiring Diagram

<img src="https://github.com/destroyer796/3DWelder/blob/main/Images/3DWelderRender.png">

Render

