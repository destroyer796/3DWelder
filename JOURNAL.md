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
