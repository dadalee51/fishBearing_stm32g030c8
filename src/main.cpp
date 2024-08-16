#include <SimpleFOC.h>
// MA
TwoWire i2c1 = TwoWire(PB9, PB8); // sda, scl
MagneticSensorI2C sensorA = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motorA = BLDCMotor(7);
BLDCDriver3PWM driverA = BLDCDriver3PWM(PA8, PA9, PA10, PB15);
HardwareSerial hs(PB7, PB6);
void setup()
{
  // MA
  sensorA.init(&i2c1);
  motorA.linkSensor(&sensorA);
  driverA.voltage_power_supply = 12;
  driverA.init();
  motorA.linkDriver(&driverA);
  motorA.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motorA.controller = MotionControlType::angle;
  // motorA.controller = MotionControlType::velocity;
  motorA.PID_velocity.P = 0.2f;
  motorA.PID_velocity.I = 0.01f;
  motorA.PID_velocity.D = 0;
  motorA.voltage_limit = 12;
  motorA.LPF_velocity.Tf = 0.01f;
  motorA.P_angle.P = 20;
  motorA.velocity_limit = 1000;
  motorA.current_limit = 20.0;
  motorA.PID_velocity.output_ramp = 1000;

  // init and FOC area
  motorA.init();
  delay(500);
  motorA.initFOC();
  delay(500);

  // initialize sensors at PB14, PB12, PB11,PB1,  ir LED @PA_12
  
  pinMode(PB0, OUTPUT_OPEN_DRAIN); //R
  pinMode(PC7, OUTPUT_OPEN_DRAIN);//G
  pinMode(PC6, OUTPUT_OPEN_DRAIN);//B
  
  digitalWrite(PB15, 1);//en_ma

  digitalWrite(PB0, 1);
  analogWrite(PC7, 0xFE);
  digitalWrite(PC6, 1);
  
  hs.begin(115200);
  hs.println("Hello from Fish Bearing");
  delay(1000);
}
long last_cur = 5000;
long target = 10;
void loop()
{
  motorA.loopFOC();
  if(millis() - last_cur > 5000){
    target = -target;
    last_cur = millis();
    // motorA.current_limit +=0.5;
  }
  
  motorA.move(target);
}