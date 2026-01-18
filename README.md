# QNX Camera example modified to track a puck

Requires OpenCV cross-compiling. Run as `./demo -u 1`. Build with `make`

Compile OpenCV with the QNX port patch, and disable MOI, VT, and anything else that fails to compile really.

This repository consists of two processes:

- pwm_controller receives a delta between the puck and paddle and moves paddle. Uses named channels and `MsgSend`/`MsgReceive`
- demo receives a camera video feed, runs OpenCV shape detection, and sends a delta to pwm_controller
