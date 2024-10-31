VCC (The breakout board has a voltage regulator. Therefore, you can connect the board to 3.3V and 5V sources.)
GND
SCL (Serial Clock Line of the I2C protocol.)
SDA (Serial Data Line of the I2C protocol.)
XDA (Auxiliary data => I2C master serial data for connecting the module to external sensors.)
XCL (Auxiliary clock => I2C master serial clock for connecting the module to external sensors.)
AD0 (If this pin is LOW, the I2C address of the board will be 0x68. Otherwise, if the pin is HIGH, the address will be 0x69.)
INT (Interrupt digital output)

Arduino Uno, Arduino Ethernet, Arduino Nano: A4 or SDA, A5 or SCL
Arduino Mega2560: 20 (SDA), 21 (SCL)
Arduino Leonardo: 2 (SDA), 3 (SCL)
Arduino Due: 20 (SDA), 21 (SCL)
