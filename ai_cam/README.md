# AI Cam Sketch

This Arduino sketch for an ESP32-CAM performs object detection and transmits the object's location.

## Functionality

1. Camera Initialization: Sets up the ESP32-CAM's camera module.
2. Object Detection: Uses an Edge Impulse model (nwad123-hackusu-25_inferencing.h) to detect objects in camera frames.
3. Location Calculation: Determines the object's position relative to the camera's center.
4. ESP-NOW Transmission: Sends the object's (x, y) coordinates via ESP-NOW to a broadcast address. If no object is detected, it sends a special "not in frame" value.

## Dependencies

- `eloquent_esp32cam`
- `eloquent_esp32cam/edgeimpulse/fomo`
- `nwad123-hackusu-25_inferencing.h` (custom Edge Impulse model)

### To install `nwad123-hackusu-25_inferencing.h`

The edge impulse model is located in the ai_cam/ directory and named
`ei-nwad123-hackusu-25-arduino-1.0.2.zip`.
In the Arduino IDE, go to Sketch -> Include Library -> Add .ZIP Library...
and select the edge impulse model file.

## Hardware

    - ESP32-CAM (AI Thinker board)
