/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).
Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/

#include <Wire.h>
#include <L3G.h>

L3G gyro;
String gyroString;

void GyroSetup() {
  Wire.begin();

  if (!gyro.init())
  {
    PublishDebugMessage("Failed to autodetect gyro type! Gyro timeout");
    nh.spinOnce();
    while (1);
  }

  gyro.enableDefault();
}

void readGyroValues() {
  gyro.read();

//TODO re-enable me when using gyro / on an ATMEGA2560 board w/ enough memory for float buffer
/*
  gyro_msg.x = gyro.g.x;
  gyro_msg.y = gyro.g.y;
  gyro_msg.z = gyro.g.z;
  */
}
