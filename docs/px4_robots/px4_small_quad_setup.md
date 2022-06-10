---
layout: default
title:  "4. Small Quad Setup"
date:   2022-05-09
math: katex
parent: Px4 Robot
nav_order: 4
---

## Assembly
Forward of the drone is the direction of the white arms. To set up, we need a Pixhawk Cube Orange, Raspberry Pi, and Here 3, with two mounts, foam tape, zip ties, and double sided tape. First place the foam tape on the center of the upper black platoform on the msall quad. Then place the Pixhawk so that it is centered, and that the side with the telem ports is facing the front. (Include the picture here).

Take the lower platform (picture here) and remove the crossbeams on one side for microusb acess (picture here), and add double sided tape to the bottom of the legs. Place the platform on black platoform of the small quad, so that the microusb side of the pixhawk is unobstructed (picture here). Zip tie the lower white platform using the holes on thes side of the black platform. 

Make sure all wires and the case is removed from the Raspberry Pi. Grab the raspberry pi and mount it so that the usb side is facing forward using zip ties through its mounting holes and the holes in the lower platform (picture here). 

Place double sided tape on the bottom of the legs and the middle of the top of the upper white platform. mount the platform on the side ridges on the top of the lower white platform. take the Here 3, and mount it on the upper platform such that the arrow on the top of the here 3 logo is facing forward.

## Wiring

Two telemetry wires are needed for wiring the pi to the pixhawk. We will be using 2 Green (Tx), 2 Yellow (Rx), and 2 Black (Ground) wires, 2 six header pins, 2 single pin dupont, a 2 pin dupont, and a 3 pin dupont. Plug the ground into the 6th pin from the left, the green the third, and the yellow in the second (Picture here). They can just be pushed in from the __ end (pic here?). then put the yellow wire in the single pin dupont and the green and balck wires on the the ends of each 2 and 3 pin dupont.

## Pixhawk
1. Set up the pixhawk by connecting it to the ground station via microUSB
2. In the settings of QGC, flash the latest firmware (rn it's _____)
3.  Make sure to set the airframe as generic quadcopter
4. Make sure to run the calibration first, as recalibrating sensors resets the parameters
5. set the parameters properly
    - Set MAV_SYS_ID to be a different value from other quads
    - Check off Hold and Offboard for COM_RCL_EXCEPT 
    - Check off GPS, vision position fusion, and yaw position fusion for EKF2_AID_MASK if outdoor, and only vision position fusino and yaw position fusion if indoor
        - NOTE: Code for vision position fusion is different whether or note use GPS is checked
    - Set EKF2_GPS_P_NOISE to 0.2 m
    - Set EKF2_GPS_V_NOISE to 0.15 m/s
    - Set EKF2_HGT_MODE to GPS if doing outdoor testing and Vision if doing indoor testing
    - set MAV_0_RATE to 0 B/s
    - set COM_RC_IN_MODE set to joystick only
    - set MAV_0_RADIO_CTL to disable
    - set SER_TEL1_BAUD to 115200 8N1
    - set SER_TEL2_BAUD to 921600
    - set RTPS_CONFIG to TELEM 2

