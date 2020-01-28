# OpenRow
 CircuitPython code for Stroke Coach.
 
 v2 - GPS and accelerometer for tracking distance and strokes in a [boat](https://www.youtube.com/watch?v=bkroMesEigI).

## Hardware
[Parts List](http://www.adafruit.com/wishlists/490932)

## Software
This is divided into two categories:
1. Track overall distance with the GPS.
2. Track strokes with the accelerometer.

Once complication is that we do not a-priori know which way is "down" so instead, average the last few samples and subtract off the average (`GravityRemover`).
This yields essentially a zero vector for constant velocity motion.
When the acceleration peaks (during a stroke), capture it via thresholding (`HistoryThresholder`).
