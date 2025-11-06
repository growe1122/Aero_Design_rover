# Rover Control Test Framework
# Mary V. Rowe 11-05-2025

This PlatformIO project supports both Arduino Uno (hardware) and native (desktop) testing.

## Usage
### Run unit tests (desktop)
    pio test -e native
### Upload to Arduino
    pio run -t upload -e uno
### Trigger Hardware Demo
Open Serial Monitor at 115200 baud and type \start\ to begin.

Servo smoothing can be toggled in \
over_main.ino\ via:
    #define USE_EASED_SERVO