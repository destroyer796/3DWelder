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
