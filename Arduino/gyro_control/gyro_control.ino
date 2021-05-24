#include <Wire.h>
#include <Servo.h>


#define  CTRL_REG1  0x20
#define  CTRL_REG2  0x21
#define  CTRL_REG3  0x22
#define  CTRL_REG4  0x23
#define  CTRL_REG5  0x24
#define  CTRL_REG6  0x25

int gyroI2CAddr=105;

int gyroRaw[3];                         // raw sensor data, each axis, pretty useless really but here it is.
double gyroDPS[3];                      // gyro degrees per second, each axis

float heading[3]={0.0f};                // heading[x], heading[y], heading [z]

int gyroZeroRate[3];                    // Calibration data.  Needed because the sensor does center at zero, but rather always reports a small amount of rotation on each axis.
int gyroThreshold[3];                   // Raw rate change data less than the statistically derived threshold is discarded.

#define  NUM_GYRO_SAMPLES  50           // As recommended in STMicro doc
#define  GYRO_SIGMA_MULTIPLE  3         // As recommended

float dpsPerDigit=.00875f;              // for conversion to degrees per second
double xpos = 0;
double ypos = 0;
double zpos = 0;

Servo SR1;
Servo SR2;
Servo SL1;
Servo SL2;

int ZR1 = 180;
int ZR2 = 5;
int ZL1 = 0;
int ZL2 = 175;

int DR1 = -1;
int DR2 = 1;
int DL1 = 1;
int DL2 = -1;

int S1A = 0;
int S2A = 0;

bool deactivated = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupGyro();
  calibrateGyro();
  SR1.attach(5);
  SR1.write(ZR1);
  SR2.attach(6);
  SR2.write(ZR2);
  SL1.attach(9);
  SL1.write(ZL1);
  SL2.attach(10);
  SL2.write(ZL2);
}

void loop() {
  updateGyroValues();
  updateHeadings();

  xpos += float(gyroDPS[0]) / 50;
  ypos += float(gyroDPS[1]) / 70;
  zpos += float(gyroDPS[2]) / 50;

  
  S1A = -1 * zpos;
  S1A = (S1A > 0) ? S1A : 0;
  S1A = (S1A < 100) ? S1A : 100;

  S2A = -1 * xpos;
  S2A = (S2A > 0) ? S2A : 0;
  S2A = (S2A < 70) ? S2A : 70;

  if (!(ypos < -20 || ypos > 20)) {
    if (deactivated) {
      xpos = 0;
      zpos = 0;
    }
    deactivated = false;
  } else {
    deactivated = true;
  }

  if (!deactivated) {
    // move the servos
    SR1.write(ZR1 + S1A * DR1);
    SR2.write(ZR2 + S2A * DR2);
    SL1.write(ZL1 + S1A * DL1);
    SL2.write(ZL2 + S2A * DL2);
  }
  
  delay(20);
}

void printDPS()
{
  Serial.print("DPS X: ");
  Serial.print(gyroDPS[0]);
  Serial.print("  Y: ");
  Serial.print(gyroDPS[1]);
  Serial.print("  Z: ");
  Serial.print(gyroDPS[2]);
}

void printHeadings()
{
  Serial.print("Heading X: ");
  Serial.print(heading[0]);
  Serial.print("  Y: ");
  Serial.print(heading[1]);
  Serial.print("  Z: ");
  Serial.print(heading[2]);
}

void updateHeadings()
{
   
  float deltaT=getDeltaTMicros();

  for (int j=0;j<3;j++)
    heading[j] -= (gyroDPS[j]*deltaT)/1000000.0f;
}

// this simply returns the elapsed time since the last call.
unsigned long getDeltaTMicros()
{
  static unsigned long lastTime=0;
 
  unsigned long currentTime=micros();
 
  unsigned long deltaT=currentTime-lastTime;
  if (deltaT < 0.0)
     deltaT=currentTime+(4294967295-lastTime);
   
  lastTime=currentTime;
 
  return deltaT;
}

// I called this from the loop function to see what the right values were for the calibration constants.
// If you are trying to reduce the amount of time needed for calibration just try not to go so low that consecutive
// calibration calls give you completely unrelated data.  Some sensors are probably better than others.
void testCalibration()
{
  calibrateGyro();
  for (int j=0;j<3;j++)
  {
    Serial.print(gyroZeroRate[j]);
    Serial.print("  ");
    Serial.print(gyroThreshold[j]);
    Serial.print("  "); 
  }
  Serial.println();
  return;
}

// The settings here will suffice unless you want to use the interrupt feature.
void setupGyro()
{
  gyroWriteI2C(CTRL_REG1, 0x1F);        // Turn on all axes, disable power down
  gyroWriteI2C(CTRL_REG3, 0x08);        // Enable control ready signal
  setGyroSensitivity500();

  delay(100);
}

// Call this at start up.  It's critical that your device is completely stationary during calibration.
// The sensor needs recalibration after lots of movement, lots of idle time, temperature changes, etc, so try to work that in to your design.
void calibrateGyro()
{
  long int gyroSums[3]={0};
  long int gyroSigma[3]={0};

  for (int i=0;i<NUM_GYRO_SAMPLES;i++)
  {
    updateGyroValues();
    for (int j=0;j<3;j++)
    {
      gyroSums[j]+=gyroRaw[j];
      gyroSigma[j]+=gyroRaw[j]*gyroRaw[j];
    }
  }
  for (int j=0;j<3;j++)
  {
    int averageRate=gyroSums[j]/NUM_GYRO_SAMPLES;
   
    // Per STM docs, we store the average of the samples for each axis and subtract them when we use the data.
    gyroZeroRate[j]=averageRate;
   
    // Per STM docs, we create a threshold for each axis based on the standard deviation of the samples times 3.
    gyroThreshold[j]=sqrt((double(gyroSigma[j]) / NUM_GYRO_SAMPLES) - (averageRate * averageRate)) * GYRO_SIGMA_MULTIPLE;   
  }
}

void updateGyroValues() {

  while (!(gyroReadI2C(0x27) & B00001000)){}      // Without this line you will get bad data occasionally
 
  //if (gyroReadI2C(0x27) & B01000000)
  //  Serial.println("Data missed!  Consider using an interrupt");
   
  int reg=0x28;
  for (int j=0;j<3;j++)
  {
    gyroRaw[j]=(gyroReadI2C(reg) | (gyroReadI2C(reg+1)<<8));
    reg+=2;
  }

  int deltaGyro[3];
  for (int j=0;j<3;j++)
  {
    deltaGyro[j]=gyroRaw[j]-gyroZeroRate[j];      // Use the calibration data to modify the sensor value.
    if (abs(deltaGyro[j]) < gyroThreshold[j])
      deltaGyro[j]=0;
    gyroDPS[j]= dpsPerDigit * deltaGyro[j];      // Multiply the sensor value by the sensitivity factor to get degrees per second.
  }
}

void setGyroSensitivity250(void)
{
  dpsPerDigit=.00875f;
  gyroWriteI2C(CTRL_REG4, 0x80);        // Set scale (250 deg/sec)
}

void setGyroSensitivity500(void)
{
  dpsPerDigit=.0175f;
  gyroWriteI2C(CTRL_REG4, 0x90);        // Set scale (500 deg/sec)
}

void setGyroSensitivity2000(void)
{
  dpsPerDigit=.07f;
  gyroWriteI2C(CTRL_REG4,0xA0);
}

int gyroReadI2C (byte regAddr) {
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.endTransmission();
  Wire.requestFrom(gyroI2CAddr, 1);
  while(!Wire.available()) {};
  return (Wire.read());
}

int gyroWriteI2C( byte regAddr, byte val){
  Wire.beginTransmission(gyroI2CAddr);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}
