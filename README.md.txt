An autonomous embedded system built on the STM32F103 microcontroller designed for real-time fire detection, target tracking, distance verification, and automated fire suppression.

Components:

Real-time Thermal Imaging: Uses MLX90640 with dynamic thresholding to pinpoint fire heat sources.
Laser Distance Verification: Integrates VL53L1X ToF sensor for precise target distance measuring.
2-DOF Pan-Tilt Turret: Fast target acquisition using dual MG995 servos driven by high-precision hardware PWM.
Robust Power & Motor Isolation: Power management with 2S Li-Ion, flyback diode protection for the 5V submersible pump, and isolated logic rails.
Bus Diagnostics & Lockup Recovery: Hardware I2C bus scanner logic with auto-recovery for I2C bus lockup scenarios.
