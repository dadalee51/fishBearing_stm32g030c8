#include <SimpleFOC.h>
// MA
TwoWire i2c1 = TwoWire(PB9, PB8); // sda, scl
MagneticSensorI2C sensorA = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motorA = BLDCMotor(7);
BLDCDriver3PWM driverA = BLDCDriver3PWM(PA8, PA9, PA10, PB15);


float target_angle = 0;
float target_velocity = 10;
HardwareSerial hs(PB7, PB6);
void setup()
{
  // MA
  sensorA.init(&i2c1);
  motorA.linkSensor(&sensorA);
  driverA.voltage_power_supply = 8;
  driverA.init();
  motorA.linkDriver(&driverA);
  motorA.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // motor.controller = MotionControlType::angle;
  motorA.controller = MotionControlType::velocity;
  motorA.PID_velocity.P = 0.2f;
  motorA.PID_velocity.I = 0.1f;
  motorA.PID_velocity.D = 0;
  motorA.voltage_limit = 8;
  motorA.LPF_velocity.Tf = 0.01f;
  motorA.P_angle.P = 20;
  motorA.velocity_limit = 100;
  motorA.PID_velocity.output_ramp = 1000;

  // init and FOC area
  motorA.init();
  
  motorA.initFOC();
  

  // initialize sensors at PB14, PB12, PB11,PB1,  ir LED @PA_12
  
  pinMode(PB0, OUTPUT_OPEN_DRAIN); //R
  pinMode(PC7, OUTPUT_OPEN_DRAIN);//G
  pinMode(PC6, OUTPUT_OPEN_DRAIN);//B
  
  digitalWrite(PB0, 0);
  digitalWrite(PC7, 1);
  digitalWrite(PC6, 1);
  
  hs.begin(115200);
  hs.println("Hello from Fish Bearing");
  delay(1000);
}

void loop()
{
 

  motorA.loopFOC();
  motorA.move(target_velocity);
  
}