---
layout: default
title:  "Vicon Calibration"
date:   2025-06-18
math: katex
parent: FlyLab
nav_order: 1
---

<style>
    img {
        max-height: 400px;
        width: auto;
    }
</style>


## Initial Steps

The system should always be on - never turn the Vicon off.

Enter the control room. Contact DASC lab for the password if the door is locked.

![](./images/control_room_door.png)

Straight ahead is the Vicon computer. If it is asleep, wake it up. Never turn this computer off. Open the Vicon software if it’s not already open.

![](./images/vicon_computer.png)

It is a good idea to recalibrate every so often since the cameras can get bumped. If a camera is bumped, it will show up with a red icon as shown in the picture in the System Settings tab on the left.

![alt text](./images/bumped.png)

## Masking

Ensure there are no markers visible to the system by looking at the 3D View.![alt text](./images/masking_no_markers.png)
The image below shows what it looks like when there are markers in view of the cameras.
![alt text](./images/masking_markers_visible.png)

Open the Camera Calibration tab on the left side of the screen and open the Cameras view.

![alt text](./images/masking_initial.png)

Click Mask All. You should see the dots change from light blue to dark blue. When the lights have settled like in the picture below, click Stop.

![alt text](./images/masking_mask_all.png)

Masking is now complete.

## Waving the Wand

Grab the wand on top of the computer

![](./images/wand_storage.png)

Turn it on with the switch. Ensure it is in Continuous mode.

![](./images/wand_switch.png)

If the battery is low, charge it with the cable attached to the computer before using it.

![](./images/charge_wand.png)

Now in the Camera Calibration tab, click the settings icon to check that Auto-Stop is enabled for the wand, and that the rest of the settings match the picture below. Then click Wave All.

![](./images/wand_settings.png)

The screen will now look like this:

![](./images/wand_initial.png)

Each camera needs to receive 3000 wand frames to have a robust calibration, and you will achieve this by waving the wand in the general direction of each camera. Once each camera has seen enough frames, the process will auto-stop.

Out in the FlyLab space, wave the wand around like you are washing a window. Make sure to wave it in the direction of each camera, until all of the lower cameras have filled their progress bars and shine a green light

![Lower wand](./images/lower_wand.gif)

Here is an example of the progress bar on this camera. The red light will turn green once the progress bar is full. By waving the wand in all 6 axes of motion (3 translational and 3 rotational), you will eventually complete the progress bar.

![](./images/lower_camera_progress_bar.png)

During this process, the computer screen will also show the status of each camera

![](./images/wand_in_progress.png)

To get the upper cameras, we attach the wand to a rope, located next to the control room door.

![](./images/rope.png)

Untie the rope from the anchor and lower the end of the rope to ground level.

![](./images/rope_lowered.png)

Then attach the wand through the 3D printed attachment and make sure it is secured.

![](./images/wand_attachment.png)

Raise the wand using the rope and shake the rope up and down to make the wand jump and spin around to get enough coverage on each upper camera.

![](./images/pull_rope.png)

<img src="./images/upper_wand.gif" alt="Upper wand" style="width:30%; max-height:none;">

Each upper camera will light up green once it has seen enough frames.

![](./images/upper_rope_green_camera.png)

Once all cameras have enough detections, the calibration should auto-stop and all camera lights will turn red as shown below, indicating normal operation.

![](./images/complete_camera.png)

The computer screen will look like this

![](./images/wand_finished.png)

## Setting the Origin

After the wand waving process, the origin of the Vicon coordinate frame needs to be set. Go back to the 3D View and you will see that the default origin has been placed at a camera.

![](./images/origin_initial.png)

Take the wand and place it in the yellow tape in the FlyLab.

![](./images/wand_origin.png)

Pan and zoom (left click and right click while moving the mouse) in the 3D View to ensure the wand markers are visible, then click Start Set Origin, then right after click Set Origin.

![](./images/origin_pan_zoom.png)

The origin has now been set at the wand's location, which you can verify in the 3D View.

![](./images/origin_final.png)
