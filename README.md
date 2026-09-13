An autonomous robot built on the STM32F103 microcontroller designed for searching and automatically extinguishing fire hazards. The system detects hotspots using a thermal imaging camera, verifies target distance with a laser ToF rangefinder, and aims a pan-tilt turret equipped with a water pump.

Here is a link to a video of it.
https://drive.google.com/drive/folders/1A6Z49xJr-joiZc6H49M-nv8G-IkJ48fF?usp=sharing

Components:

Microcontroller: STM32F103C8T6.
Thermal Camera: Melexis MLX90640.
Rangefinder: VL53L1X.
Pan-Tilt Turret (2-DOF): MG995 servos driven by hardware PWM.
Extinguishing System: 5V water pump controlled via an N-channel MOSFET with flyback diode protection.
Power Supply: 2S Li-Ion battery  with step-down DC-DC converters.

Hardware Architecture & Technical Details

1. PWM Configuration
An STM32 hardware timer is used to generate 50 Hz PWM signals for servo control:
Prescaler (PSC): 35 each tick equals 1ms.
Auto-Reload Register (ARR):19999 20000ms.

2. I2C Bus Interface
Both the MLX90640 thermal camera and VL53L1X  rangefinder share the same I2C bus.
External 4.7 kOm pull-up resistors to 3.3 V ensure clean signal rise times at 400 kHz.

System Logic Flow

1.Scanning:
The STM32 reads frame data from the MLX90640 over I2C and locates the peak pixel temperature.
2.Thresholding:
If T_max > 150°C, the system calculates the angular offset of the target relative to the frame center.
3.Targeting:
Offset values update the timer's CCR registers, rotating the servo turret toward the heat source.
4. Distance Measurement & Pitch Adjustment:
The VL53L1X sensor measures the distance to the target:
   If Distance > 20 cm: The top servo tilts UP by 6° to compensate for water trajectory arc.
   If Distance > 15 cm (15–20 cm): The top servo tilts UP by 4°.
   If Distance < 15 cm: No pitch angle offset applied.
5. Suppression:The pump operates for a 5s burst, followed by a rescan to verify fire suppression.