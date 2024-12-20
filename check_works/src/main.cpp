#include <SimpleFOC.h>
#include <Arduino.h>
// Define the SPI pins for the encoder
#define ENCODER_CS 15   // Chip Select pin
#define ENCODER_CLK 18  // Clock pin
#define ENCODER_MISO 23 // MISO pin
#define ENCODER_MOSI 19 // MOSI pin

#define MOTOR_KV              14.0f
#define MOTOR_R               11.8f
#define MOTOR_POLE            11
#define MOTOR_TORQUE_CONSTANT 0.68f
#define DRIVER_RSENSE         0.01f
#define DRIVER_GAIN           50.0f

// MagneticSensorSPI object for the AS5147 encoder
MagneticSensorSPI sensor(AS5147_SPI, ENCODER_CS);

// Motor and driver instance
BLDCMotor motor0 = BLDCMotor(MOTOR_POLE, MOTOR_R, MOTOR_KV); // Set pole pair count according to your motor
BLDCDriver3PWM driver0 = BLDCDriver3PWM(32, 33, 25, 22);

// Teoretycznie można zrobić tak, żeby nie było potrzeby podawania pinów, ale z jakichś powodów nie działało to dobrze.



    //motor         = new BLDCMotor(MOTOR_POLE, MOTOR_R, MOTOR_KV);
    //driver        = new BLDCDriver3PWM(phA, phB, phC, en);

// Inline current sense
InlineCurrentSense current_sense = InlineCurrentSense(0.01f, 50.0f, (int)39, (int)36);

// Commander communication instance
Commander command = Commander(Serial);
void doMotor0(char *cmd) { command.motor(&motor0, cmd); }

void setup() {
  analogRead(39);
  analogRead(36);
  // SPI initialization for the encoder
  SPI.begin(ENCODER_CLK, ENCODER_MISO, ENCODER_MOSI);
  sensor.init();
  motor0.linkSensor(&sensor);

  // Driver setup
  driver0.voltage_power_supply = 24; // Adjust according to your power supply
  driver0.init();
  motor0.linkDriver(&driver0);

  // Current limit and voltage limit
  motor0.current_limit = 2;         // Adjust based on your motor's specifications
  motor0.voltage_limit = 24;        // Adjust according to your power supply


  // Current sense setup
  current_sense.init();
  current_sense.gain_b *= -1;
  current_sense.gain_a *= -1;
  current_sense.skip_align = true;
  //current_sense.calibrateOffsets();
  motor0.linkCurrentSense(&current_sense);


  motor0.controller = MotionControlType::angle;
motor0.motion_downsample = 0.0;

// Velocity loop PID parameters
motor0.PID_velocity.P = 0.15;
motor0.PID_velocity.I = 0.0;
motor0.PID_velocity.D = 0.0;
motor0.PID_velocity.output_ramp = 500.0;
motor0.PID_velocity.limit = 2.0;

// Low pass filtering for velocity
motor0.LPF_velocity.Tf = 0.02;

// Angle loop PID parameters
motor0.P_angle.P = 20.0;
motor0.P_angle.I = 0.0;
motor0.P_angle.D = 0.0;
motor0.P_angle.output_ramp = 0.0;
motor0.P_angle.limit = 3.0;

// Low pass filtering for angle
motor0.LPF_angle.Tf = 0.0;

// Current q loop PID parameters
motor0.PID_current_q.P = 7.0;
motor0.PID_current_q.I = 300.0;
motor0.PID_current_q.D = 0.0;
motor0.PID_current_q.output_ramp = 0.0;
motor0.PID_current_q.limit = 24.0;

// Low pass filtering for current q
motor0.LPF_current_q.Tf = 0.005;

// Current d loop PID parameters
motor0.PID_current_d.P = 7.0;
motor0.PID_current_d.I = 300.0;
motor0.PID_current_d.D = 0.0;
motor0.PID_current_d.output_ramp = 0.0;
motor0.PID_current_d.limit = 24.0;

// Low pass filtering for current d
motor0.LPF_current_d.Tf = 0.005;

// Limits
motor0.velocity_limit = 10.0;    // [rad/s]
motor0.voltage_limit = 24.0;     // [V]
motor0.current_limit = 2.0;      // [A]

  // Monitoring setup
  Serial.begin(115200);
  motor0.useMonitoring(Serial);
  motor0.monitor_downsample = 1000;
  motor0.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

  // Motor initialization
  motor0.init();
  motor0.initFOC();

  // Initial target value
  motor0.target = 0.05;

  // Map motor to commander
  command.add('A', doMotor0, "motor 0");

  Serial.println(F("Single motor sketch ready."));
}

int iter = 0;
void loop() {
  // FOC phase voltage setting
  motor0.loopFOC();

  // Outer loop target setting
  motor0.move();

  // User communication
  command.run();
  //motor0.monitor();
PhaseCurrent_s currents = current_sense.getPhaseCurrents();
float current_magnitude = current_sense.getDCCurrent();
  // Print motor currents 
// print every 100th iteration
if( iter++ > 1000 ){
  iter = 0;
  Serial.print("Ia: ");
  Serial.print(currents.a, 4);
  Serial.print("\t Ib: ");
  Serial.print(currents.b, 4);
  Serial.print("\t Ic: ");
  Serial.print(currents.c, 4);
  Serial.print("\t| I: ");
  Serial.print(current_magnitude, 4);
  Serial.println("A");
}
}
