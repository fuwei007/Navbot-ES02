  #include <Arduino.h>
  #include <SimpleFOC.h>
  #include <Preferences.h>// This library is used for key-value data storage and retrieval in ESP32, enabling data persistence
  #include "SlotCalibration.h"
  #include "FUTABA_SBUS.h"
  #include "ServoControl.h"
  #include "ICM42688.h"
  #include "MahonyFilter.h"
  #include "OllieFOCdrive.h"
  #include "filter.h"
  #include "touchscreen.h"

  // commander communication instance
  Commander command = Commander(Serial);

  // Set working mode
  #define SensorSwitch          2   // 1: SPI  2: IIC AS5600     
  #define Communication_object  0   // 0: 2-wheel balance || 4-wheel balance movement  1: simpleFOC Studio host computer  2: control dual motors  3: sample torque data
  #define TorqueCompensation    0   // 1: torque compensation  0: no torque compensation (cannot be modified)
  #define SwitchUser            4   // 0: view encoder position and direction  1: sample motor 1 torque compensation data  2: sample motor 2 torque compensation data  3: torque  4: speed  5: angle mode  
  #define  CurrentUser          0   // 1: enable current loop
  #define  M2CurrentUser        0   // 1: enable current loop for motor 2

  #define  AdjusParameter       0   // 0: balance, speed, yaw, roll parameter tuning   1: ball pushing
  #define  SwitchingPattern     0   // 0: two-wheel  1: four-wheel  switching mode 
  #define  MasterSlaveSelection 1   // 0: slave   1: master 

  // Body
  #define Thigh 0.035f      // Thigh length (m)
  #define Shank 0.072f      // Shank length (m)
  float TargetLegLength = 0; // Target leg length
  float LegLength = 0.06f;   // Leg length
  float BarycenterX = 0;     // Center of mass X
  float BodyPitching = 0;    // Pitch
  float BodyRoll = 0;        // Roll
  float MovementSpeed = 0;   // Movement speed
  float BodyTurn = 0;        // Turning
  float SlideStep = 0;       // Slide step
  float BodyX = 0;           // X position (controller output)
  int RobotTumble = 0;       // Robot tumble (fall detection)


  body_t body;


  //滤波
  float LegLength_f = 0.06f;  // Leg length
  float BarycenterX_f = 0;    // Center of mass X
  float BodyPitching_f = 0;   // Pitch
  float BodyRoll_f = 0;       // Roll
  float MovementSpeed_f = 0;  // Movement speed
  float BodyTurn_f = 0;       // Turning
  float SlideStep_f = 0;      // Slide step
  float BodyX_f = 0;          // X position
  biquadFilter_t FilterLPF[15]; // Second-order low-pass filter
  float TouchY_Pid_outputF = 0;
  float TouchX_Pid_outputF = 0;

  float cutoffFreq = 200;
  float enableDFilter = 1;
  float LpfOut[6];//

  void CutoffFreq(char* cmd) {
    command.scalar(&cutoffFreq, cmd);
  }

  void EnableDFilter(char* cmd) {
    command.scalar(&enableDFilter, cmd);
  }

  const int LED_Pin = 35; //LED IO
  int LED_HL = 1;
  int LED_count = 0;
  int LED_dt = 100;
  const int analogInPin = 17; // Battery voltage IO
  int sensorValue = 0;        // value read from the pot
  biquadFilter_t VoltageFilterLPF; // Second-order low-pass filter
  uint16_t VoltageADC = 0; // Battery voltage ADC data
  float VoltageADCf = 0; // Battery voltage ADC data
  float Voltage = 0; // Battery voltage

  //IMU
  // Create MahonyFilter object, set proportional gain and integral gain
  MahonyFilter mahonyFilter(0.4f, 0.001f);
  // Accelerometer range (here set to ±8g)
  const float accelRange = 8.0;  // Unit: g
  // Gyroscope range (here assumed to be ±2000°/s)
  const float gyroRange = 2000.0;       // Unit: °/s
  attitude_t attitude;
  float roll_ok;//
  float pitch_ok;//
    
  zeroBias_t zeroBias; // Zero offset
  unsigned long timestamp_prev = 0;
  float IMUtime_dt = 0;

  /* Low-pass filter parameters */
  float RATE_HZ_last = 1000.0f; // Sampling frequency
  float LPF_CUTOFF_FREQ_last = 50.0f; // Cutoff frequency

  float RATE_HZ = 1000.0f; // Sampling frequency
  float LPF_CUTOFF_FREQ = 50.0f; // Cutoff frequency
  biquadFilter_t ImuFilterLPF[6]; // Second-order low-pass filter


  void ImuRATE_HZ(char* cmd)      {
    command.scalar(&RATE_HZ, cmd);
  }
  void ImuLPF_CUTOFF_FREQ(char* cmd)      {
    command.scalar(&LPF_CUTOFF_FREQ, cmd);
  }

  void Target_Leg_Length(char* cmd)      {
    command.scalar(&TargetLegLength, cmd);
  }



  //Complementary filter  
  float angleGyroX, angleGyroY, angleGyroZ,
        angleAccX, angleAccY;
  float angleX, angleY, angleZ;
  float accCoef = 0.02f;
  float gyroCoef = 0.98f;



  //  Declare a Preferences object for subsequent read/write operations to flash memory
  Preferences preferences;
  //  Define a floating-point array to store Euler angle data
  float zeroBiasFlash[9];
  //  Define a character string pointer array to store the keys corresponding to the roll and pitch angles, which can be directly modified by the user
  //  The key name is used to uniquely identify data in flash memory
  const char* zeroBiasKeys[9] = {
    "roll",
    "pitch",
    "gyroX",
    "gyroY",
    "gyroZ",
    "servoAngle1",
    "servoAngle2",
    "servoAngle3",
    "servoAngle4"
  };


  //  Define the number of times the function is called
  const int CALL_COUNT = 100;//  Set the number of times the function is called
  int callCounter = 0;  //  Call counter



  //Servo
  const int CUSTOM_SERVO_1_PIN = 11;
  const int CUSTOM_SERVO_2_PIN = 12;
  const int CUSTOM_SERVO_3_PIN = 21;
  const int CUSTOM_SERVO_4_PIN = 14;

  void zeroBias_servo1(char* cmd)      {
    command.scalar(&zeroBias.servo1, cmd);
  }
  void zeroBias_servo2(char* cmd)      {
    command.scalar(&zeroBias.servo2, cmd);
  }
  void zeroBias_servo3(char* cmd)      {
    command.scalar(&zeroBias.servo3, cmd);
  }
  void zeroBias_servo4(char* cmd)      {
    command.scalar(&zeroBias.servo4, cmd);
  }



  //  Create ServoControl object, pass in the custom pin
  ServoControl servoControl(CUSTOM_SERVO_1_PIN, CUSTOM_SERVO_2_PIN, CUSTOM_SERVO_3_PIN, CUSTOM_SERVO_4_PIN);

  //Remote control
  FUTABA_SBUS sBus;
  float sbuschx[8] = {0};
  int sbus_dt_ms = 0;
  int sbus_swa = 0;
  int sbus_swb = 0;
  int sbus_swc = 0;
  int sbus_swd = 0;
  float sbus_vra = 0;
  float sbus_vrb = 0;
  float sbus_vraf = 0;
  float sbus_vrbf = 0;

  #define SBUS_chMax 1792
  #define SBUS_chMin 192


  #define Serial1_START1  12
  #define Serial1_START2  34
  #define Serial1_END1    0


  //  Create PID controller instance
  float Select = 0;//Select the data to print
  float CalibrationSelect = 0;//Save calibration data 0: Calibration end  1: Calibrate gyroscope  2: Calibrate Euler angle  3: Calibrate servo

  float PidParameterTuning = 0;//0: Disable parameter tuning  1: Enable parameter tuning

  PIDController AnglePid (0, 0, 0 , 0 , 0); //4 22 0.08   (Kp, Ki, Kd ,ramp ,limit) 
  PIDController SpeedPid (0.1, 0.1, 0 , 0 , 50); //
  PIDController YawPid   (11, 33, 0 , 0 , 0); //
  PIDController RollPid  (0.06, 1.5, 0.003 , 0 , 2); //
  PIDController TouchXPid(0.2, 0, 0.04 , 0 , 0); //
  PIDController TouchYPid(0.2, 0, 0.08 , 0 , 0); //

  float control_torque_compensation = 0;//Control torque compensation

  float PidDt = 0.01;

  //  Create MyPIDController instance, set initial parameters
  MyPIDController Angle_Pid(0, 0, 0, 0, 0,PidDt,0,0);//p i d iLimit outputLimit dt EnableDFilter cutoffFreq
  MyPIDController Speed_Pid(0, 0, 0, 0, 0,PidDt,0,0);
  MyPIDController Yaw_Pid  (0, 0, 0, 0, 0,PidDt,0,0);
  MyPIDController Roll_Pid  (0, 0, 0, 0, 0,PidDt,0,0);
  MyPIDController Pitching_Pid  (0, 0, 0, 0, 0,PidDt,0,0);

  MyPIDController TouchX_Pid  (0, 0, 0, 0, 10,PidDt,0,0);
  MyPIDController TouchY_Pid  (0, 0, 0, 0, 8,PidDt,0,0);

  void ControlTorqueCompensation(char* cmd) {
    command.scalar(&control_torque_compensation, cmd);
  }


  void Pid_Parameter_Tuning(char* cmd)      {
    command.scalar(&PidParameterTuning, cmd);
  }

  void TwoKp(char* cmd)      {
    command.scalar(&mahonyFilter.twoKp, cmd);
  }
  void TwoKi(char* cmd)      {
    command.scalar(&mahonyFilter.twoKi, cmd);
  }

  void KeyScalar(char* cmd)      {
    command.scalar(&Select, cmd);
  }
  void KeyCalibration(char* cmd) {
    command.scalar(&CalibrationSelect, cmd);
  }

  #if AdjusParameter == 0
  void CbAnglePid(char* cmd) {
    command.pid(&AnglePid, cmd);
  }
  void CbSpeedPid(char* cmd) {
    command.pid(&SpeedPid, cmd);
  }
  void CbYawPid(char* cmd)  {
    command.pid(&YawPid, cmd);
  }

  void CbRollPid(char* cmd)  {
    command.pid(&RollPid, cmd);
  }

  #elif AdjusParameter == 1

  void CbTouchXPid(char* cmd)  {
    command.pid(&TouchXPid, cmd);
  }

  void CbTouchYPid(char* cmd)  {
    command.pid(&TouchYPid, cmd);
  }
  #endif


  float  Motor1_voltage_compensation = 0;
  float  Motor2_voltage_compensation = 0;
  double Motor1_place_last = 0;
  float  Motor1_Velocity = 0;
  float  Motor1_Velocity_f = 0;
  LowPassFilter Motor1_Velocity_filter = LowPassFilter(0.01); // Tf = 10ms

  double Motor2_place_last = 0;
  float  Motor2_Velocity = 0;
  float  Motor2_Velocity_f = 0;
  LowPassFilter Motor2_Velocity_filter = LowPassFilter(0.01); // Tf = 10ms

  Serial_t serial1;
  Serial_t serial2;

  float Motor1_Target = 0;
  float Motor2_Target = 0;

  double RightMotorAngle = 0;
  double LeftMotorAngle = 0;

  float time_dt = 0;
  unsigned long now_us = 0;
  unsigned long now_us1 = 0;
  unsigned long now_us2 = 0;
  // BLDC motor & driver instance
  BLDCMotor motor1 = BLDCMotor(7);//Motor pole pairs
  BLDCDriver3PWM driver = BLDCDriver3PWM(15, 7, 6, 16);

  BLDCMotor motor2 = BLDCMotor(7);
  BLDCDriver3PWM driver2 = BLDCDriver3PWM(40, 39, 38, 37);

  #if SensorSwitch == 1
  // MagneticSensorSPI(int cs, float _cpr, int _angle_register)
  // config           - SPI config
  //  cs              - SPI chip select pin
  MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, 19);
  MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5147_SPI, 23);
  // these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
  SPIClass * hspi = NULL;

  #elif SensorSwitch == 2
  MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
  MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
  TwoWire I2Cone = TwoWire(0);
  TwoWire I2Ctwo = TwoWire(1);

  #endif

  // 1. Define the parameters of the current sensor
  #define SENSOR_MV_PER_AMP 90.0f   // ACS712-05B sensitivity is 185mV/A

  #if CurrentUser == 1
  // inline current sensor instance
  // ACS712-05B has the resolution of 0.185mV per Amp
  InlineCurrentSense current_sense1 = InlineCurrentSense(SENSOR_MV_PER_AMP, 18, 17);
  #endif

  #if M2CurrentUser == 1
  // inline current sensor instance
  // ACS712-05B has the resolution of 0.185mV per Amp
  InlineCurrentSense current_sense2 = InlineCurrentSense(SENSOR_MV_PER_AMP, 35, 36);
  #endif


  void doMotion1(char* cmd) {
    command.motion(&motor1, cmd);
  }
  void doMotor1(char* cmd) {
    command.motor(&motor1, cmd);
  }

  void doMotion2(char* cmd) {
    command.motion(&motor2, cmd);
  }
  void doMotor2(char* cmd) {
    command.motor(&motor2, cmd);
  }

  void Send_Serial1(void);
  void Read_Serial1(void); //Read serial port 1 data;
  void Send_Serial2(void);
  void Read_Serial2(void); //Read serial port 2 data;
  void RXsbus();
  int RightInverseKinematics(float x, float y, float p, float* ax);
  int LeftInverseKinematics(float x, float y, float p, float* ax);
  void print_data(void);
  void ImuUpdate(void);
  void FlashSave(int sw);
  void FlashInit(void);
  void PIDcontroller_angle(float dt);
  void PIDcontroller_posture(float dt);
  void PIDcontroller_posture_4wheel(float dt);
  void RemoteControlFiltering(void);
  void ReadVoltage(void);
  void PidParameter(void);
  void Robot_Tumble(void);
  void body_data_init(void);
  void TrotGaitAlgorithm(void);//Trot gait
  void MotorOperatingMode(void);



  void setup() {


    if((MasterSlaveSelection==0)&&(SwitchingPattern==1))//Slave && 4-wheel mode
      Serial2.begin(1000000, SERIAL_8N1, RXD2,TXD2 );
    else if(MasterSlaveSelection==1)//Master
      Serial1.begin(1000000, SERIAL_8N1, RXD1, TXD1);


    
    Serial.begin(250000);
    FlashInit();//Read flash data
    pinMode(LED_Pin, OUTPUT);
    digitalWrite(LED_Pin, LOW);   //亮

    body_data_init();
    //Initialize second-order low-pass filter
    for (int axis = 0; axis < 6; axis++)
    {
      biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
    }

    biquadFilterInitLPF(&VoltageFilterLPF,50.0f, 100);//Voltage filter function initialization

    //  Initialize servo
    servoControl.initialize();
   
    servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1);
    _delay(555);

    servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1); //Assembly position
    //IMU
    if(MasterSlaveSelection==1)//Master
    {
      if (!initICM42688()) {
        Serial.println("ICM42688 initialization failed!");
        while (1);
      }
      Serial.println("ICM42688 initialized successfully!");    
    }


    //calibrateGyro();
    
    //Remote control
    if(MasterSlaveSelection==1)//Master uses
      sBus.begin();
      
    for (int i = 0; i < 6; i++)
      biquadFilterInitLPF(&FilterLPF[i], 100, (unsigned int)cutoffFreq);//Remote control filter
    
    biquadFilterInitLPF(&FilterLPF[8], 50, (unsigned int)cutoffFreq);//Remote control filter
    biquadFilterInitLPF(&FilterLPF[9], 50, (unsigned int)cutoffFreq);//Remote control filter
    biquadFilterInitLPF(&FilterLPF[10], 200, (unsigned int)400);//
    biquadFilterInitLPF(&FilterLPF[11], 200, (unsigned int)400);//
    biquadFilterInitLPF(&FilterLPF[12], 50, (unsigned int)cutoffFreq);//Remote control filter

    
    // use monitoring with serial
    if((SwitchingPattern==0)||(MasterSlaveSelection==0))//Two-wheel or slave mode
      TouchscreenInit(500);
    // enable more verbose output for debugging
    // comment out if not needed
    SimpleFOCDebug::enable(&Serial);

  #if SensorSwitch == 1
    hspi = new SPIClass(HSPI);
    hspi->begin(18, 5, 17);//(sck, miso, mosi)
    //initialise magnetic sensor1 hardware
    sensor1.init(hspi);
    sensor2.init(hspi);
  #elif SensorSwitch == 2
    I2Cone.begin(4, 5, 400000);
    I2Ctwo.begin(41, 42, 400000);   //SDA1,SCL1
    sensor1.init(&I2Cone);
    sensor2.init(&I2Ctwo);
  #endif


    // sensor1.min_elapsed_time = 0.0001; // 100us by default
    // sensor2.min_elapsed_time = 0.0001; // 100us by default

    // link the motor to the sensor
    motor1.linkSensor(&sensor1);
    motor2.linkSensor(&sensor2);

    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 8.4;
    driver.init();

    driver2.voltage_power_supply = 8.4;
    driver2.init();
    // link driver
    motor1.linkDriver(&driver);
    motor2.linkDriver(&driver2);
    // link current sense and the driver
  #if CurrentUser == 1
    current_sense1.linkDriver(&driver);
  #endif

  #if M2CurrentUser == 1
    current_sense2.linkDriver(&driver2);
  #endif


    // control loop type and torque mode  velocity angle
    if (CurrentUser == 1)
      motor1.torque_controller = TorqueControlType::dc_current;   //foc_current   dc_current  voltage
    else
      motor1.torque_controller = TorqueControlType::voltage;

    if ((SwitchUser == 1)||(SwitchUser == 5))
      motor1.controller = MotionControlType::angle;
    else if (SwitchUser == 3)
      motor1.controller = MotionControlType::torque;
    else if (SwitchUser == 4)
      motor1.controller = MotionControlType::velocity;

    motor1.motion_downsample = 0.0;//

    // velocity loop PID
    motor1.PID_velocity.P = 0.006;//0.07;
    motor1.PID_velocity.I = 0;
    if(SwitchingPattern==1)
      motor1.PID_velocity.I = 0.8;
    motor1.PID_velocity.D = 0.0;
    motor1.PID_velocity.output_ramp = 10000;
    motor1.PID_velocity.limit = 8.4;
    // Low pass filtering time constant
    motor1.LPF_velocity.Tf = 0.001;
    // angle loop PID
    motor1.P_angle.P = 15.0;
    motor1.P_angle.I = 22.0;
    motor1.P_angle.D = 0.0;
    motor1.P_angle.output_ramp = 10000;
    motor1.P_angle.limit = 111.0;
    // Low pass filtering time constant
    motor1.LPF_angle.Tf = 0.001;
    // current q loop PID
    motor1.PID_current_q.P = 2;
    motor1.PID_current_q.I = 222;
    motor1.PID_current_q.D = 0.0;
    motor1.PID_current_q.output_ramp = 11111;
    motor1.PID_current_q.limit = 8.4;
    // Low pass filtering time constant
    motor1.LPF_current_q.Tf = 0.01;
    // current d loop PID
    motor1.PID_current_d.P = motor1.PID_current_q.P;
    motor1.PID_current_d.I = motor1.PID_current_q.I;
    motor1.PID_current_d.D = motor1.PID_current_q.D;
    motor1.PID_current_d.output_ramp = motor1.PID_current_q.output_ramp;
    motor1.PID_current_d.limit = motor1.PID_current_q.limit;
    // Low pass filtering time constant
    motor1.LPF_current_d.Tf = motor1.LPF_current_q.Tf;
    // Limits
    motor1.velocity_limit = 88.0;
    motor1.voltage_limit = 8.4;
    motor1.current_limit = 5.0;
    // sensor zero offset - home position
    //motor1.sensor_offset = -68924.77299999999;
    // general settings
    // motor phase resistance
    motor1.phase_resistance = 22;
    // pwm modulation settings
    motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // Set PWM modulation to center alignment mode
    motor1.modulation_centered = 1.0;

    if (M2CurrentUser == 1)
      // control loop type and torque mode velocity angle
      motor2.torque_controller = TorqueControlType::foc_current;//foc_current   dc_current  voltage
    else
      // control loop type and torque mode velocity angle
      motor2.torque_controller = TorqueControlType::voltage;//foc_current   dc_current  voltage


    if((SwitchUser == 2)||(SwitchUser == 5))
      motor2.controller = MotionControlType::angle;
    else if (SwitchUser == 3)
      motor2.controller = MotionControlType::torque;
    else if (SwitchUser == 4)
      motor2.controller = MotionControlType::velocity;


    motor2.motion_downsample = 0.0;

    // velocity loop PID
    motor2.PID_velocity.P = 0.006;
    motor2.PID_velocity.I = 0; 
    if(SwitchingPattern==1)
      motor2.PID_velocity.I = 0.8;
    motor2.PID_velocity.D = 0;
    motor2.PID_velocity.output_ramp = 10000;
    motor2.PID_velocity.limit = 8.4;
    // Low pass filtering time constant
    motor2.LPF_velocity.Tf = 0.001;
    // angle loop PID
    motor2.P_angle.P = 22;
    motor2.P_angle.I = 111;
    motor2.P_angle.D = 0;
    motor2.P_angle.output_ramp = 10000;
    motor2.P_angle.limit = 88;
    // Low pass filtering time constant
    motor2.LPF_angle.Tf = 0.001;

    // current q loop PID
    motor2.PID_current_q.P = 2;
    motor2.PID_current_q.I = 222;
    motor2.PID_current_q.D = 0.0;
    motor2.PID_current_q.output_ramp = 11111;
    motor2.PID_current_q.limit = 5.0;
    // Low pass filtering time constant
    motor2.LPF_current_q.Tf = 0.01;
    // current d loop PID
    motor2.PID_current_d.P = motor2.PID_current_q.P;
    motor2.PID_current_d.I = motor2.PID_current_q.I;
    motor2.PID_current_d.D = motor2.PID_current_q.D;
    motor2.PID_current_d.output_ramp = motor2.PID_current_q.output_ramp;
    motor2.PID_current_d.limit = motor2.PID_current_q.limit;
    // Low pass filtering time constant
    motor2.LPF_current_d.Tf = motor2.LPF_current_q.Tf;

    // Limits
    motor2.velocity_limit = motor1.velocity_limit;
    motor2.voltage_limit = motor1.voltage_limit;
    motor2.current_limit = motor1.current_limit;
    // sensor zero offset - home position
    //motor2.sensor_offset = -68924.77299999999;
    // general settings
    // motor phase resistance
    motor2.phase_resistance = motor1.phase_resistance;
    // pwm modulation settings
    motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor2.modulation_centered = motor1.modulation_centered;

  #if CurrentUser == 1
    // current sense init and linking
    current_sense1.init();
    motor1.linkCurrentSense(&current_sense1);
  #endif

  #if M2CurrentUser == 1
    // current sense init and linking
    current_sense2.init();
    motor2.linkCurrentSense(&current_sense2);
  #endif



    // initialise motor
    motor1.init();
    motor2.init();
    // align encoder and start FOC

    if (SwitchUser == 0)
    {
      motor1.initFOC();
      motor2.initFOC();
      Serial.print("Sensor1 zero offset is:");
      Serial.print(motor1.zero_electric_angle, 6);//Initial electrical angle
      Serial.print("  Sensor1 natural direction is: ");
      Serial.println(motor1.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");//Motor rotation direction (clockwise, counterclockwise)

      Serial.print("Sensor2 zero offset is:");
      Serial.print(motor2.zero_electric_angle, 6);//Initial electrical angle
      Serial.print("  Sensor2 natural direction is: ");
      Serial.println(motor2.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");//Motor rotation direction (clockwise, counterclockwise)

      while (1);
    }
    else
    {

      //motor1.sensor_direction=Direction::CCW; // or Direction::CCW
      //motor1.zero_electric_angle=2.586293;   // use the real value!
      motor1.initFOC();

      //motor2.sensor_direction=Direction::CW; // or Direction::CCW
      //motor2.zero_electric_angle=4.166292;   // use the real value!
      motor2.initFOC();
    }


    // set the inital target value
    motor1.target = 0;
    motor2.target = 0;


    // comment out if not needed

    motor1.useMonitoring(Serial);
    motor1.monitor_downsample = 10; // disable intially
    //motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle

    // subscribe motor to the commander
    command.add('T', doMotion1, "motion1 control");//Set motor target value
    command.add('M', doMotor1, "motor1");

    command.add('A', zeroBias_servo1, "my zeroBias_servo1"); //Set servo 1 bias
    command.add('B', zeroBias_servo2, "my zeroBias_servo2"); //
    command.add('C', zeroBias_servo3, "my zeroBias_servo3"); //
    command.add('D', zeroBias_servo4, "my zeroBias_servo4"); //

    command.add('H', ImuRATE_HZ, "my ImuRATE_HZ");
    command.add('Z', ImuLPF_CUTOFF_FREQ, "my ImuLPF_CUTOFF_FREQ");


    command.add('Q', TwoKp, "my TwoKp"); //MahonyFilter
    command.add('I', TwoKi, "my TwoKi"); //MahonyFilter

    command.add('K', KeyScalar, "my Select");
    command.add('E', KeyCalibration, "my CalibrationSelect");

    command.add('F', CutoffFreq, "my CutoffFreq");
    command.add('J', EnableDFilter, "my EnableDFilter");

  #if AdjusParameter == 0
    command.add('P', CbAnglePid, "my AnglePid");
    command.add('S', CbSpeedPid, "my SpeedPid");
    command.add('Y', CbYawPid, "my YawPid");
    command.add('R', CbRollPid, "my RollPid");
    command.add('O', Target_Leg_Length, "my Target_Leg_Length");
  #elif AdjusParameter == 1  
    command.add('L', CbTouchXPid, "my CbTouchXPid");
    command.add('N', CbTouchYPid, "my CbTouchYPid");
    command.add('G', ControlTorqueCompensation, "my ControlTorqueCompensation");
  #endif

    command.add('U', Pid_Parameter_Tuning, "my Pid_Parameter_Tuning");


    // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
    Serial.println("Motor ready.");

    _delay(1000);
    timestamp_prev = micros();
  }



  boolean crc1(unsigned char buffer[])
  {
    unsigned int crc_bit1 = 0;
    unsigned int sum1 = 0;

    for (int j = 2; j <= 27; j++)
    {
      sum1 += buffer[j];
    }
    crc_bit1 = sum1 & 0xff;
    if ((unsigned char)crc_bit1 == buffer[28])
      return true;
    else
      return false;
  }


  unsigned char crc2(unsigned char buffer[])
  {
    unsigned int crc_bit1 = 0;
    unsigned int sum1 = 0;

    for (int j = 2; j <= 27; j++)
    {
      sum1 += buffer[j];
    }
    crc_bit1 = sum1 & 0xff;

    return (unsigned char)crc_bit1;
  }



  void Send_Serial1(void)
  {
    //Start flag
    serial1.txbuf[0] = Serial1_START1;
    serial1.txbuf[1] = Serial1_START2;
    //Left leg coordinate x
    serial1.txbuf[2] = ((uint8_t *)&body.xo3)[0]; //
    serial1.txbuf[3] = ((uint8_t *)&body.xo3)[1];
    serial1.txbuf[4] = ((uint8_t *)&body.xo3)[2];
    serial1.txbuf[5] = ((uint8_t *)&body.xo3)[3];
    //Left leg coordinate z
    serial1.txbuf[6] = ((uint8_t *)&body.zo3)[0];
    serial1.txbuf[7] = ((uint8_t *)&body.zo3)[1];
    serial1.txbuf[8] = ((uint8_t *)&body.zo3)[2];
    serial1.txbuf[9] = ((uint8_t *)&body.zo3)[3];
    //Right leg coordinate x
    serial1.txbuf[10] = ((uint8_t *)&body.xo4)[0]; //
    serial1.txbuf[11] = ((uint8_t *)&body.xo4)[1];
    serial1.txbuf[12] = ((uint8_t *)&body.xo4)[2];
    serial1.txbuf[13] = ((uint8_t *)&body.xo4)[3];
    //Right leg coordinate z
    serial1.txbuf[14] = ((uint8_t *)&body.zo4)[0];
    serial1.txbuf[15] = ((uint8_t *)&body.zo4)[1];
    serial1.txbuf[16] = ((uint8_t *)&body.zo4)[2];
    serial1.txbuf[17] = ((uint8_t *)&body.zo4)[3];
    //Left leg motor target value
    serial1.txbuf[18] = ((uint8_t *)&body.MT[2])[0];
    serial1.txbuf[19] = ((uint8_t *)&body.MT[2])[1];
    serial1.txbuf[20] = ((uint8_t *)&body.MT[2])[2];
    serial1.txbuf[21] = ((uint8_t *)&body.MT[2])[3];
    //Right motor target value
    serial1.txbuf[22] = ((uint8_t *)&body.MT[3])[0];
    serial1.txbuf[23] = ((uint8_t *)&body.MT[3])[1];
    serial1.txbuf[24] = ((uint8_t *)&body.MT[3])[2];
    serial1.txbuf[25] = ((uint8_t *)&body.MT[3])[3];

    //Motor operating mode
    serial1.txbuf[26] = body.MotorMode;

    //
    serial1.txbuf[27] = body.Ts;  

    //Checksum
    serial1.txbuf[28] = crc2(serial1.txbuf);
    //End flag
    serial1.txbuf[29] = Serial1_END1;

    Serial1.write(serial1.txbuf, sizeof(serial1.txbuf));

  }




  void Read_Serial1(void) //Read serial port 1 data
  {

    while(Serial1.available())
    {
      serial1.dat = Serial1.read();
      //Serial.println(serial1.dat);
      if ((serial1.count == 0) && (serial1.dat == Serial1_START1))
      {
        serial1.rxbuf[serial1.count] = serial1.dat;
        serial1.count = 1;
      }
      else if ((serial1.count == 1) && (serial1.dat == Serial1_START2))
      {
        serial1.rxbuf[serial1.count] = serial1.dat;
        serial1.recstatu = 1;
        serial1.count = 2;
      }
      else if (serial1.recstatu == 1) //Header byte is correct
      {
        serial1.rxbuf[serial1.count] = serial1.dat;
        serial1.count++;
        if (serial1.count >= 29)
        {
          if (crc1(serial1.rxbuf))
          {
            body.Serial1count++;
            serial1.recstatu = 0;
            serial1.packerflag = 1;//For system notification of successful reception
            serial1.count = 0;
            //Touch screen x data
            ((uint8_t *)&Touch.XPdatF)[0] = serial1.rxbuf[2];
            ((uint8_t *)&Touch.XPdatF)[1] = serial1.rxbuf[3];
            ((uint8_t *)&Touch.XPdatF)[2] = serial1.rxbuf[4];
            ((uint8_t *)&Touch.XPdatF)[3] = serial1.rxbuf[5];
            //Touch screen y data
            ((uint8_t *)&Touch.YPdatF)[0] = serial1.rxbuf[6];
            ((uint8_t *)&Touch.YPdatF)[1] = serial1.rxbuf[7];
            ((uint8_t *)&Touch.YPdatF)[2] = serial1.rxbuf[8];
            ((uint8_t *)&Touch.YPdatF)[3] = serial1.rxbuf[9];
            //Left motor speed
            ((uint8_t *)&body.MotorVelocityF[2])[0] = serial1.rxbuf[10];
            ((uint8_t *)&body.MotorVelocityF[2])[1] = serial1.rxbuf[11];
            ((uint8_t *)&body.MotorVelocityF[2])[2] = serial1.rxbuf[12];
            ((uint8_t *)&body.MotorVelocityF[2])[3] = serial1.rxbuf[13];
            //Right motor speed
            ((uint8_t *)&body.MotorVelocityF[3])[0] = serial1.rxbuf[14];
            ((uint8_t *)&body.MotorVelocityF[3])[1] = serial1.rxbuf[15];
            ((uint8_t *)&body.MotorVelocityF[3])[2] = serial1.rxbuf[16];
            ((uint8_t *)&body.MotorVelocityF[3])[3] = serial1.rxbuf[17];  

            Touch.state = serial1.rxbuf[27];  
            /*        
            //Left leg motor target value
            ((uint8_t *)&body.MT[2])[0] = serial1.rxbuf[18];
            ((uint8_t *)&body.MT[2])[1] = serial1.rxbuf[19];
            ((uint8_t *)&body.MT[2])[2] = serial1.rxbuf[20];
            ((uint8_t *)&body.MT[2])[3] = serial1.rxbuf[21];  
            //Right motor target value
            ((uint8_t *)&body.MT[3])[0] = serial1.rxbuf[22];
            ((uint8_t *)&body.MT[3])[1] = serial1.rxbuf[23];
            ((uint8_t *)&body.MT[3])[2] = serial1.rxbuf[24];
            ((uint8_t *)&body.MT[3])[3] = serial1.rxbuf[25];  
            //Motor operating mode
            body.MotorMode = serial1.rxbuf[26];
          
            //Gait time
            body.Ts = serial1.rxbuf[27];

            */
          }
          else
          {
            serial1.rxbuf[0] = 0;
            serial1.rxbuf[1] = 0;
            serial1.recstatu = 0;
            serial1.packerflag = 0;//Receive failed
            serial1.count = 0;
            //Serial.println("on2..............................");
          }
        }
      }
      else
      {
        serial1.rxbuf[0] = 0;
        serial1.rxbuf[1] = 0;
        serial1.recstatu = 0;
        serial1.packerflag = 0;//For system notification of failed reception
        serial1.count = 0;
        serial1.dat = 0;
        //Serial.println("on1..............................");
      }
    }
  }



  void Send_Serial2(void)
  {
    //Start flag
    serial2.txbuf[0] = Serial1_START1;
    serial2.txbuf[1] = Serial1_START2;

    //Touch screen x position
    serial2.txbuf[2] = ((uint8_t *)&Touch.XPdatF)[0]; //
    serial2.txbuf[3] = ((uint8_t *)&Touch.XPdatF)[1];
    serial2.txbuf[4] = ((uint8_t *)&Touch.XPdatF)[2];
    serial2.txbuf[5] = ((uint8_t *)&Touch.XPdatF)[3];
    //Touch screen y position
    serial2.txbuf[6] = ((uint8_t *)&Touch.YPdatF)[0];
    serial2.txbuf[7] = ((uint8_t *)&Touch.YPdatF)[1];
    serial2.txbuf[8] = ((uint8_t *)&Touch.YPdatF)[2];
    serial2.txbuf[9] = ((uint8_t *)&Touch.YPdatF)[3];
    //Left motor speed
    serial2.txbuf[10] = ((uint8_t *)&Motor1_Velocity_f)[0]; //
    serial2.txbuf[11] = ((uint8_t *)&Motor1_Velocity_f)[1];
    serial2.txbuf[12] = ((uint8_t *)&Motor1_Velocity_f)[2];
    serial2.txbuf[13] = ((uint8_t *)&Motor1_Velocity_f)[3];
    //Right motor speed
    serial2.txbuf[14] = ((uint8_t *)&Motor2_Velocity_f)[0];
    serial2.txbuf[15] = ((uint8_t *)&Motor2_Velocity_f)[1];
    serial2.txbuf[16] = ((uint8_t *)&Motor2_Velocity_f)[2];
    serial2.txbuf[17] = ((uint8_t *)&Motor2_Velocity_f)[3];
    //Left leg motor target value
    serial2.txbuf[18] = 0;//((uint8_t *)&body.MT[2])[0];
    serial2.txbuf[19] = 0;//((uint8_t *)&body.MT[2])[1];
    serial2.txbuf[20] = 0;//((uint8_t *)&body.MT[2])[2];
    serial2.txbuf[21] = 0;//((uint8_t *)&body.MT[2])[3];
    //Right motor target value
    serial2.txbuf[22] = 0;//((uint8_t *)&body.MT[3])[0];
    serial2.txbuf[23] = 0;//((uint8_t *)&body.MT[3])[1];
    serial2.txbuf[24] = 0;//((uint8_t *)&body.MT[3])[2];
    serial2.txbuf[25] = 0;//((uint8_t *)&body.MT[3])[3];

    //Motor operating mode
    serial2.txbuf[26] = 0;//body.MotorMode;

    //
    serial2.txbuf[27] = Touch.state;//body.Ts;  

    //Checksum
    serial2.txbuf[28] = crc2(serial2.txbuf);
    //End flag
    serial2.txbuf[29] = Serial1_END1;

    Serial2.write(serial2.txbuf, sizeof(serial2.txbuf));

  }


  void Read_Serial2(void) //Read serial port 2 data
  {
    
    while(Serial2.available())
    {
      serial2.dat = Serial2.read();
      //Serial.println(serial2.dat);
      if ((serial2.count == 0) && (serial2.dat == Serial1_START1))
      {
        serial2.rxbuf[serial2.count] = serial2.dat;
        serial2.count = 1;
      }
      else if ((serial2.count == 1) && (serial2.dat == Serial1_START2))
      {
        serial2.rxbuf[serial2.count] = serial2.dat;
        serial2.recstatu = 1;
        serial2.count = 2;
      }
      else if (serial2.recstatu == 1) //Header byte is correct
      {
        serial2.rxbuf[serial2.count] = serial2.dat;
        serial2.count++;
        if (serial2.count >= 29)
        {
          if (crc1(serial2.rxbuf))
          {
            body.Serial1count++;
            serial2.recstatu = 0;
            serial2.packerflag = 1;//For system notification of successful reception
            serial2.count = 0;
            //Left leg coordinate x
            ((uint8_t *)&body.xo3)[0] = serial2.rxbuf[2];
            ((uint8_t *)&body.xo3)[1] = serial2.rxbuf[3];
            ((uint8_t *)&body.xo3)[2] = serial2.rxbuf[4];
            ((uint8_t *)&body.xo3)[3] = serial2.rxbuf[5];
            //Left leg coordinate z
            ((uint8_t *)&body.zo3)[0] = serial2.rxbuf[6];
            ((uint8_t *)&body.zo3)[1] = serial2.rxbuf[7];
            ((uint8_t *)&body.zo3)[2] = serial2.rxbuf[8];
            ((uint8_t *)&body.zo3)[3] = serial2.rxbuf[9];
            //Right leg coordinate x
            ((uint8_t *)&body.xo4)[0] = serial2.rxbuf[10];
            ((uint8_t *)&body.xo4)[1] = serial2.rxbuf[11];
            ((uint8_t *)&body.xo4)[2] = serial2.rxbuf[12];
            ((uint8_t *)&body.xo4)[3] = serial2.rxbuf[13];
            //Right leg coordinate z
            ((uint8_t *)&body.zo4)[0] = serial2.rxbuf[14];
            ((uint8_t *)&body.zo4)[1] = serial2.rxbuf[15];
            ((uint8_t *)&body.zo4)[2] = serial2.rxbuf[16];
            ((uint8_t *)&body.zo4)[3] = serial2.rxbuf[17];          
            //Left leg motor target value
            ((uint8_t *)&body.MT[2])[0] = serial2.rxbuf[18];
            ((uint8_t *)&body.MT[2])[1] = serial2.rxbuf[19];
            ((uint8_t *)&body.MT[2])[2] = serial2.rxbuf[20];
            ((uint8_t *)&body.MT[2])[3] = serial2.rxbuf[21];  
            //Right motor target value
            ((uint8_t *)&body.MT[3])[0] = serial2.rxbuf[22];
            ((uint8_t *)&body.MT[3])[1] = serial2.rxbuf[23];
            ((uint8_t *)&body.MT[3])[2] = serial2.rxbuf[24];
            ((uint8_t *)&body.MT[3])[3] = serial2.rxbuf[25];  
            //Motor operating mode
            body.MotorMode = serial2.rxbuf[26];
          
      
            //body.Ts = serial2.rxbuf[27];


            /*
                            Serial.println("\t");
                            Serial.print("Motor1_Target:");
                            Serial.print(Motor1_Target);
                            Serial.print("  Motor2_Target:");
                            Serial.println(Motor2_Target);
            */
            //disconnection = 0;
          }
          else
          {
            serial2.rxbuf[0] = 0;
            serial2.rxbuf[1] = 0;
            serial2.recstatu = 0;
            serial2.packerflag = 0;//Receive failed
            serial2.count = 0;
            //Serial.println("on2..............................");
          }
        }
      }
      else
      {
        serial2.rxbuf[0] = 0;
        serial2.rxbuf[1] = 0;
        serial2.recstatu = 0;
        serial2.packerflag = 0;//For system notification of failed reception
        serial2.count = 0;
        serial2.dat = 0;
        //Serial.println("on1..............................");
      }
    }
  }


  float mapf(long x, long in_min, long in_max, float out_min, float out_max)
  {
    long divisor = (in_max - in_min);
    if (divisor == 0) {
      return -1; //AVR returns -1, SAM returns 0
    }
    return (x - in_min) * (out_max - out_min) / divisor + out_min;
  }

  void RXsbus()
  {
    static unsigned long now_ms = millis();
    
    sBus.FeedLine();
    if (sBus.toChannels == 1)
    {
      sbus_dt_ms = millis()-now_ms;
      now_ms = millis();
      sBus.toChannels = 0;
      sBus.UpdateChannels();
      sBus.toChannels = 0;

      MovementSpeed =  mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -15, 15);
      BodyTurn      =  -mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -11, 11);
      sbus_swa      =  map(sBus.channels[4], SBUS_chMin, SBUS_chMax, 0, 2);
      sbus_swb      =  map(sBus.channels[5], SBUS_chMin, SBUS_chMax, 0, 1);
      sbus_swc      =  map(sBus.channels[6], SBUS_chMin, SBUS_chMax, 0, 1);
      sbus_swd      =  map(sBus.channels[7], SBUS_chMin, SBUS_chMax, 0, 2);
      sbuschx[8]    =  map(sBus.channels[8], SBUS_chMin, SBUS_chMax, 0, 100);
      sbuschx[9]    =  map(sBus.channels[9], SBUS_chMin, SBUS_chMax, 0, 100);

      //Top ball
      sbus_vra    =  mapf(sBus.channels[8], SBUS_chMin, SBUS_chMax, -5, 5);//Modify top ball target position
      sbus_vrb    =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, -5, 5);

      if (sbus_swd == 0)//Attitude control 1
      {
        //SlideStep = mapf(sBus.channels[0],SBUS_chMin, SBUS_chMax, -0.05,0.05);//Slide step
        if(sBus.channels[1]<=992)
          LegLength = mapf(sBus.channels[1], SBUS_chMin, 992, 0.05, 0.06); //Leg height
        else
          LegLength = mapf(sBus.channels[1], 993, SBUS_chMax, 0.06, 0.1); //Leg height
      }
      else if (sbus_swd == 1)//Attitude control 2
      {
        //BodyRoll =  mapf(sBus.channels[0], SBUS_chMin, SBUS_chMax, -0.011, 0.011); //Roll
        //sbus_vrb    =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, -25, 25);
        BodyPitching = mapf(sBus.channels[1], SBUS_chMin, SBUS_chMax, -12, 12); //Pitching       + sbus_vrb
      }
      else if (sbus_swd == 2)//Attitude control 3
      {
        //LegLength = 0.06;
        if(sBus.channels[1]<=992)
          LegLength = mapf(sBus.channels[1], SBUS_chMin, 992, 0.05, 0.06); 
        else
          LegLength = mapf(sBus.channels[1], 993, SBUS_chMax, 0.06, 0.07); 

      
        //BodyRoll =  mapf(sBus.channels[0], SBUS_chMin, SBUS_chMax, -0.011, 0.011); 
        //BodyPitching = mapf(sBus.channels[1], SBUS_chMin, SBUS_chMax, -22, 22);  
        //BodyPitching = mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, -45, 45); 
      }


      BodyRoll =  mapf(sBus.channels[0], SBUS_chMin, SBUS_chMax, -0.011, 0.011); 

      if(Voltage<=7.4)
      {
        //sbus_swa = 0;
        Serial.print(" Voltage:");
        Serial.println(Voltage, 5);      
      }

  /*
      static int js = 0;
      if((sBus.channels[0]>1700)&&(sBus.channels[1]<200)&&(sBus.channels[2]<200)&&(sBus.channels[3]<200))//外八
      {
        js++;
        if(js>=200)//Last for 1 second
        {
          js=0;
          //SwitchingPattern = 0;
          for (int i = 0; i < 10; i++)
          {
            digitalWrite(LED_Pin, LOW);   
            delay(100);
            digitalWrite(LED_Pin, HIGH);  
            delay(100);       
          }

        }
      }
      else if((sBus.channels[0]<200)&&(sBus.channels[1]<200)&&(sBus.channels[2]<200)&&(sBus.channels[3]>1700))//内八
      {
        js++;
        if(js>=200)
        {
          js=0;
          //SwitchingPattern = 1;
          for (int i = 0; i < 10; i++)
          {
            digitalWrite(LED_Pin, LOW);   
            delay(100);
            digitalWrite(LED_Pin, HIGH);   
            delay(100);       
          }        
        }
      }
      else
      {
        js = 0;
      }
  */      

    }
  }



  float ArcToAngle(float arc)//Convert radians to degrees
  {
    float angle = arc * (180 / PI);
    return angle;
  }

  float AngleToArc(float angle)//Convert degrees to radians
  {
    float art = angle * (PI / 180);
    return art;
  }


  //Five-link dynamic inverse kinematics
  int RightInverseKinematics(float x, float y, float p, float* ax)
  {
    x = constrain(x, -0.05, 0.05);
    y = constrain(y, 0.05, 0.1);
    
    int error = 0;//Coordinate setting exception
    float AB = Thigh;// Thigh length (m) AB=ED
    float BC = Shank;// Shank length (m) BC=DC

    float OA = 0.017f; //
    float aOCF = 0;
    float aOCF2 = 0;
    float aAOC = 0;
    float OC = 0;
    float OF = 0;
    float FC = 0;
    float AC = 0;
    float aOAC = 0;
    float aOCA = 0;
    float aBAC = 0;
    float aBAG = 0;
    float OE = OA; //
    float aEOC = 0;
    float EC = 0;
    float aOCE = 0;
    float aOEC = 0;
    float aDEC = 0;
    float aDEH = 0;

    float pitch, x1, y1;

    pitch = AngleToArc(p);  //Pitch angle

    x1 = x * cosf(pitch) - y * sinf(pitch);
    y1 = x * sinf(pitch) + y * cosf(pitch);


    OF = x1;
    FC = y1;

    //Joint 1
    OC = sqrtf(pow(OF, 2) + pow(FC, 2)); 
    aOCF = asinf(OF / OC);
    aAOC = AngleToArc(90) + aOCF;

    AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC)); 
    aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
    aOAC = PI - aOCA - aAOC;
    aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
    aBAG = PI - aBAC - aOAC;
    ax[0] = ArcToAngle(aBAG);//Joint 1 angle

    //Joint 2
    aOCF2 = -aOCF;
    aEOC = AngleToArc(90) + aOCF2;

    EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
    aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
    aOEC = PI - aOCE - aEOC;
    aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
    aDEH = PI - aDEC - aOEC;
    ax[1] = ArcToAngle(aDEH);//Joint 1 angle


    if (AC >= (AB + BC))  //Exceeds the maximum range of the structure
      return error = 1;
    else if (EC >= (AB + BC)) //Exceeds the maximum range of the structure
      return error = 2;


    return error;

  }



  int LeftInverseKinematics(float x, float y, float p, float* ax)
  {
    x = constrain(x, -0.05, 0.05);
    y = constrain(y, 0.05, 0.1);
    
    x = -x;
    p = -p;
    int error = 0;//Coordinate setting exception
    float AB = Thigh;// Thigh length (m) AB=ED
    float BC = Shank;// Shank length (m) BC=DC

    float OA = 0.017f; //
    float aOCF = 0;
    float aOCF2 = 0;
    float aAOC = 0;
    float OC = 0;
    float OF = 0;
    float FC = 0;
    float AC = 0;
    float aOAC = 0;
    float aOCA = 0;
    float aBAC = 0;
    float aBAG = 0;
    float OE = OA; //
    float aEOC = 0;
    float EC = 0;
    float aOCE = 0;
    float aOEC = 0;
    float aDEC = 0;
    float aDEH = 0;

    float pitch, x1, y1;

    pitch = AngleToArc(p);  //Pitch angle

    x1 = x * cosf(pitch) - y * sinf(pitch);
    y1 = x * sinf(pitch) + y * cosf(pitch);


    OF = -x1;
    FC = y1;

    //Joint 1
    OC = sqrtf(pow(OF, 2) + pow(FC, 2)); 
    aOCF = asinf(OF / OC);
    aAOC = AngleToArc(90) + aOCF;

    AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC)); 
    aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
    aOAC = PI - aOCA - aAOC;
    aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
    aBAG = PI - aBAC - aOAC;
    ax[0] = ArcToAngle(aBAG);//Joint 1 angle

    //Joint 2
    aOCF2 = -aOCF;
    aEOC = AngleToArc(90) + aOCF2;

    EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
    aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
    aOEC = PI - aOCE - aEOC;
    aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
    aDEH = PI - aDEC - aOEC;
    ax[1] = ArcToAngle(aDEH);//Joint 1 angle


    if (AC >= (AB + BC))  //Exceeds the maximum range of the structure
      return error = 1;
    else if (EC >= (AB + BC)) //Exceeds the maximum range of the structure
      return error = 2;


    return error;

  }


  void ImuUpdate(void)
  {
    unsigned long timestamp_now = micros();
    IMUtime_dt = (timestamp_now - timestamp_prev) * 1e-6f;

    int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp;
    readIMUData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp);

    // Subtract the gyroscope zero bias value
    gyroX -= (int16_t)gyroBiasX;
    gyroY -= (int16_t)gyroBiasY;
    gyroZ -= (int16_t)gyroBiasZ;

    // Convert the accelerometer data to g units
    attitude.acc.x = (float)accelX * accelRange / 32768.0;
    attitude.acc.y = (float)accelY * accelRange / 32768.0;
    attitude.acc.z = (float)accelZ * accelRange / 32768.0;


    // Convert the gyroscope data to rad/s
    attitude.gyro.x = (float)gyroX * gyroRange / 32768.0 * (3.1415926f / 180.0f);
    attitude.gyro.y = (float)gyroY * gyroRange / 32768.0 * (3.1415926f / 180.0f);
    attitude.gyro.z = (float)gyroZ * gyroRange / 32768.0 * (3.1415926f / 180.0f);


    //Software second-order low-pass filter
    attitude.gyrof.x = biquadFilterApply(&ImuFilterLPF[0], attitude.gyro.x);
    attitude.gyrof.y = biquadFilterApply(&ImuFilterLPF[1], attitude.gyro.y);
    attitude.gyrof.z = biquadFilterApply(&ImuFilterLPF[2], attitude.gyro.z);

    attitude.accf.x = biquadFilterApply(&ImuFilterLPF[3], attitude.acc.x);
    attitude.accf.y = biquadFilterApply(&ImuFilterLPF[4], attitude.acc.y);
    attitude.accf.z = biquadFilterApply(&ImuFilterLPF[5], attitude.acc.z);


    // Convert temperature data
    attitude.temp = (float)temp / 132.48 + 25;
    // Run the Mahony filter algorithm, pass dt
    //mahonyFilter.update(attitude.gyro.x, attitude.gyro.y, attitude.gyro.z, attitude.acc.x, attitude.acc.y, attitude.acc.z, IMUtime_dt);
    mahonyFilter.update(attitude.gyrof.x, attitude.gyrof.y, attitude.gyrof.z, attitude.accf.x, attitude.accf.y, attitude.accf.z, IMUtime_dt);

    // Get the filtered quaternion
    float q0_out, q1_out, q2_out, q3_out;
    mahonyFilter.getQuaternion(q0_out, q1_out, q2_out, q3_out);
    // Convert quaternion to Euler angles
    quaternionToEuler(q0_out, q1_out, q2_out, q3_out, &attitude.roll, &attitude.pitch, &attitude.yaw);

    roll_ok = attitude.roll - zeroBias.roll;
    pitch_ok = attitude.pitch - zeroBias.pitch;


    //////Complementary filter//////
    angleAccX = atan2(attitude.acc.y, attitude.acc.z + abs(attitude.acc.x)) * 360 / 2.0 / PI;
    angleAccY = atan2(attitude.acc.x, attitude.acc.z + abs(attitude.acc.y)) * 360 / -2.0 / PI;

    gyroX = (float)gyroX * gyroRange / 32768.0;
    gyroY = (float)gyroY * gyroRange / 32768.0;
    gyroZ = (float)gyroZ * gyroRange / 32768.0;

    angleGyroX += gyroX * IMUtime_dt;
    angleGyroY += gyroY * IMUtime_dt;
    angleGyroZ += gyroZ * IMUtime_dt;

    angleX = (gyroCoef * (angleX + gyroX * IMUtime_dt)) + (accCoef * angleAccX);
    angleY = (gyroCoef * (angleY + gyroY * IMUtime_dt)) + (accCoef * angleAccY);
    angleZ = angleGyroZ;


    timestamp_prev = timestamp_now;

  }


  void FlashInit(void)
  {
    preferences.begin("preferences", false);

    //Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
    zeroBias.roll = preferences.getFloat(zeroBiasKeys[0], 0.0);
    zeroBias.pitch = preferences.getFloat(zeroBiasKeys[1], 0.0);

    //Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
    gyroBiasX = preferences.getFloat(zeroBiasKeys[2], 0.0);
    gyroBiasY = preferences.getFloat(zeroBiasKeys[3], 0.0);
    gyroBiasZ = preferences.getFloat(zeroBiasKeys[4], 0.0);

    //Servo angle
    zeroBias.servo1 = preferences.getFloat(zeroBiasKeys[5], 0.0);
    zeroBias.servo2 = preferences.getFloat(zeroBiasKeys[6], 0.0);
    zeroBias.servo3 = preferences.getFloat(zeroBiasKeys[7], 0.0);
    zeroBias.servo4 = preferences.getFloat(zeroBiasKeys[8], 0.0);


    // Close flash access, release related resources
    preferences.end();

    Serial.println(" ");
    // Output zero bias
    Serial.print("  Roll Zero Bias: ");
    Serial.print(zeroBias.roll);
    Serial.print("  Pitch Zero Bias: ");
    Serial.println(zeroBias.pitch);

    // Output zero bias
    Serial.print("  gyroBiasX:");
    Serial.print(gyroBiasX);
    Serial.print("  gyroBiasY:");
    Serial.print(gyroBiasY);
    Serial.print("  gyroBiasZ:");
    Serial.println(gyroBiasZ);


    // Output zero bias
    Serial.print("  servo1:");
    Serial.print(zeroBias.servo1);
    Serial.print("  servo2:");
    Serial.print(zeroBias.servo2);
    Serial.print("  servo3:");
    Serial.print(zeroBias.servo3);
    Serial.print("  servo4:");
    Serial.println(zeroBias.servo4);



  }


  // Function to calculate the Euler angle zero bias
  void calculateZeroBias() {
    // Initialize the accumulator
    static float rollSum = 0;
    static float pitchSum = 0;

    // Accumulate the Euler angle
    rollSum += attitude.roll;
    pitchSum += attitude.pitch;

    // Increase the call counter
    callCounter++;

    if (callCounter >= CALL_COUNT)
    {
      // Calculate the average value to get the zero bias
      zeroBias.roll = rollSum / CALL_COUNT;
      zeroBias.pitch = pitchSum / CALL_COUNT;

      // Initialize flash access, open the "preferences" namespace
      // The second parameter is false, indicating that the namespace is opened in write mode
      preferences.begin("preferences", false);

      //Write data
      zeroBiasFlash[0] = zeroBias.roll;
      preferences.putFloat(zeroBiasKeys[0], zeroBiasFlash[0]);
      zeroBiasFlash[1] = zeroBias.pitch;
      preferences.putFloat(zeroBiasKeys[1], zeroBiasFlash[1]);
      //Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
      zeroBias.roll = preferences.getFloat(zeroBiasKeys[0], 0.0);
      zeroBias.pitch = preferences.getFloat(zeroBiasKeys[1], 0.0);

      // Close flash access, release related resources
      preferences.end();

      // Output zero bias
      Serial.print("  Roll Zero Bias: ");
      Serial.print(zeroBias.roll);
      Serial.print("  Pitch Zero Bias: ");
      Serial.println(zeroBias.pitch);
      rollSum = 0;
      pitchSum = 0;
      callCounter = 0;//Clear the next time
      CalibrationSelect = 0;//Calibration complete exit calibration

    }

  }


  void FlashSave(int sw)
  {

    static float servo1_last = zeroBias.servo1;//Last deviation
    static float servo2_last = zeroBias.servo2;
    static float servo3_last = zeroBias.servo3;
    static float servo4_last = zeroBias.servo4;

    switch (sw)
    {
      case 1:
        // Gyroscope calibration
        calibrateGyro();
        preferences.begin("preferences", false);

        //Write data
        preferences.putFloat(zeroBiasKeys[2], gyroBiasX);
        preferences.putFloat(zeroBiasKeys[3], gyroBiasY);
        preferences.putFloat(zeroBiasKeys[4], gyroBiasZ);

        //Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
        gyroBiasX = preferences.getFloat(zeroBiasKeys[2], 0.0);
        gyroBiasY = preferences.getFloat(zeroBiasKeys[3], 0.0);
        gyroBiasZ = preferences.getFloat(zeroBiasKeys[4], 0.0);

        // Close flash access, release related resources
        preferences.end();

        // Output zero bias
        Serial.print("  gyroBiasX:");
        Serial.print(gyroBiasX);
        Serial.print("  gyroBiasY:");
        Serial.print(gyroBiasY);
        Serial.print("  gyroBiasZ:");
        Serial.println(gyroBiasZ);


        CalibrationSelect = 0;//Calibration complete
        break;

      case 2:
        // Function to calculate the Euler angle zero bias
        calculateZeroBias();

        break;

      case 3:

        preferences.begin("preferences", false);

        if (zeroBias.servo1 != servo1_last) //Parameter adjusted, save
        {
          servo1_last = zeroBias.servo1;//
          //Write data
          preferences.putFloat(zeroBiasKeys[5], zeroBias.servo1);
          //Read servo angle
          zeroBias.servo1 = preferences.getFloat(zeroBiasKeys[5], 0.0);
          //Print data
          Serial.print("  zeroBias.servo1:");
          Serial.println(zeroBias.servo1);

        }

        if (zeroBias.servo2 != servo2_last) //Parameter adjusted, save
        {
          servo2_last = zeroBias.servo2;//
          //Write data
          preferences.putFloat(zeroBiasKeys[6], zeroBias.servo2);
          //Read servo angle
          zeroBias.servo2 = preferences.getFloat(zeroBiasKeys[6], 0.0);
          //Print data
          Serial.print("  zeroBias.servo2:");
          Serial.println(zeroBias.servo2);

        }

        if (zeroBias.servo3 != servo3_last) //Parameter adjusted, save
        {
          servo3_last = zeroBias.servo3;//
          //Write data
          preferences.putFloat(zeroBiasKeys[7], zeroBias.servo3);
          //Read servo angle
          zeroBias.servo3 = preferences.getFloat(zeroBiasKeys[7], 0.0);
          //Print data
          Serial.print("  zeroBias.servo3:");
          Serial.println(zeroBias.servo3);

        }

        if (zeroBias.servo4 != servo4_last) //Parameter adjusted, save
        {
          servo4_last = zeroBias.servo4;//
          //Write data
          preferences.putFloat(zeroBiasKeys[8], zeroBias.servo4);
          //Read servo angle
          zeroBias.servo4 = preferences.getFloat(zeroBiasKeys[8], 0.0);
          //Print data
          Serial.print("  zeroBias.servo4:");
          Serial.println(zeroBias.servo4);
        }


        // Close flash access, release related resources
        preferences.end();
        //CalibrationSelect = 0;//Manual setting calibration end

        break;



      default:

        break;
    }
  }


  void print_data(void)
  {
    switch ((int)Select)
    {
      case 1:
            // Output Euler angle
            Serial.print("dt:");
            Serial.print(time_dt,6);
            Serial.print(" Roll:");
            Serial.print(attitude.roll);
            Serial.print(" Pitch:");
            Serial.print(attitude.pitch);
            Serial.print(" Yaw:");
            Serial.println(attitude.yaw);
      
            break;

      case 2:
            // Output acc
            Serial.print("dt:");
            Serial.print(time_dt,6);
            Serial.print(" accx:");
            Serial.print(attitude.acc.x);
            Serial.print(" accy:");
            Serial.print(attitude.acc.y);
            Serial.print(" accz:");
            Serial.println(attitude.acc.z);
      
            break;

      case 3:
            // Output
            Serial.print("dt:");
            Serial.print(time_dt,6);
            Serial.print(" gyrox:");
            Serial.print(attitude.gyro.x,4);
            Serial.print(" gyroy:");
            Serial.print(attitude.gyro.y,4);
            Serial.print(" gyroz:");
            Serial.println(attitude.gyro.z,4);
      
            break;

      case 4:
            // Output Euler angle
            Serial.print("dt:");
            Serial.print(time_dt,6);
            Serial.print(" Roll:");
            Serial.print(attitude.roll - zeroBias.roll);
            Serial.print(" Pitch:");
            Serial.print(attitude.pitch - zeroBias.pitch);
            Serial.print(" Yaw:");
            Serial.println(attitude.yaw - zeroBias.yaw);
      
            break;

      case 5:
            //
            Serial.print("dt:");
            Serial.print(time_dt,6);
            Serial.print(" eRoll:");
            Serial.print(zeroBias.roll);
            Serial.print(" ePitch:");
            Serial.print(zeroBias.pitch);
            Serial.print(" eYaw:");
            Serial.println(zeroBias.yaw);
      
            break;

      case 6:
            //
            Serial.print(" v1:");
            Serial.print(Motor1_Velocity);
            Serial.print(" v2:");
            Serial.println(Motor2_Velocity);
      
      
            break;

      case 7:
            //
            Serial.print(" v1:");
            Serial.print(Motor1_Velocity);
            Serial.print(" v1f:");
            Serial.println(Motor1_Velocity_f);
            break;

      case 8:
            //
            for (int i = 0; i < 10; i++)
            {
              Serial.print(" ch:"); Serial.print(sBus.channels[i]);
            }
      
            Serial.print(" sbus_dt_ms:");
            Serial.print(sbus_dt_ms);
            Serial.println(" ");
            break;

      case 9:
            //
            Serial.print(" PP:");
            Serial.print(Angle_Pid.Kp);
            Serial.print(" PI:");
            Serial.print(Angle_Pid.Ki);
            Serial.print(" PD:");
            Serial.print(Angle_Pid.Kd);
      
            Serial.print(" SP:");
            Serial.print(Speed_Pid.Kp);
            Serial.print(" SI:");
            Serial.print(Speed_Pid.Ki);
            Serial.print(" SD:");
            Serial.print(Speed_Pid.Kd);
      
            Serial.print(" YP:");
            Serial.print(Yaw_Pid.Kp);
            Serial.print(" YI:");
            Serial.print(Yaw_Pid.Ki);
            Serial.print(" YD:");
            Serial.print(Yaw_Pid.Kd);
      
            Serial.print("dt:");
            Serial.println(time_dt,6);
      
            break;


      case 10:
              //
              Serial.print(" twoKp:");
              Serial.print(mahonyFilter.twoKp);
              Serial.print(" twoKi:");
              Serial.print(mahonyFilter.twoKi);
        
              Serial.print(" Roll:");
              Serial.print(attitude.roll);
              Serial.print(" Pitch:");
              Serial.print(attitude.pitch);
        
              Serial.print("IMUdt:");
              Serial.println(IMUtime_dt, 6);
              break;

      case 11:
              //
        
              Serial.print(" x:");
              Serial.print( angleX);
              Serial.print(" y:");
              Serial.print( angleY);
              Serial.print(" z:");
              Serial.print( angleZ);
        
        
              Serial.print(" gx:");
              Serial.print( angleGyroX);
              Serial.print(" gy:");
              Serial.print( angleGyroY);
              Serial.print(" gz:");
              Serial.print( angleGyroZ);
        
        
              Serial.print(" IMUdt:");
              Serial.println( IMUtime_dt, 6);
        
              break;

      case 12:
              //
        
              Serial.print(" x:");
              Serial.print( angleX);
              //Serial.print(" y:");
              //Serial.print( angleY);
        
              Serial.print(" Roll:");
              Serial.println( attitude.roll);
              //Serial.print(" Pitch:");
              //Serial.print( attitude.pitch);
        
        
              //Serial.print(" IMUdt:");
              //Serial.println(IMUtime_dt,6);
        
              break;


      case 13:
              // Output
              Serial.print("dt:");
              Serial.print(time_dt,6);
              Serial.print(" gyroxf:");
              Serial.print(attitude.gyrof.x);
              Serial.print(" gyroyf:");
              Serial.print(attitude.gyrof.y);
              Serial.print(" gyrozf:");
              Serial.println(attitude.gyrof.z);
              break;


      case 14:
              // Output acc
              Serial.print("dt:");
              Serial.print(time_dt,6);
              Serial.print(" accx:");
              Serial.print(attitude.accf.x);
              Serial.print(" accy:");
              Serial.print(attitude.accf.y);
              Serial.print(" accz:");
              Serial.println(attitude.accf.z);
              break;


      case 15:
              // Output acc
              //Serial.print("dt:");
              //Serial.print(time_dt,6);
              Serial.print(" accy:");
              Serial.print(attitude.acc.y);
              Serial.print(" accyf:");
              Serial.println(attitude.accf.y);
              break;

      case 16:
              // Output acc
              //Serial.print("dt:");
              //Serial.print(time_dt,6);
              Serial.print(" gyro:");
              Serial.print(attitude.gyro.y);
              Serial.print(" gyrof:");
              Serial.println(attitude.gyrof.y);
              break;


      case 17:
              // Output acc
              //Serial.print("dt:");
              //Serial.print(time_dt,6);
              Serial.print(" current_sp:");
              Serial.println(motor2.current_sp, 6);
              break;

      case 18:
              // Output acc
              Serial.print(" t:");
              Serial.print(motor2.target, 6);
              Serial.print(" a1:");
              Serial.print(sensor1.getAngle(), 6);
              Serial.print(" a11:");
              Serial.print(sensor1.getMechanicalAngle(), 6);
        
              Serial.print(" a2:");
              Serial.print(sensor2.getAngle(), 6);
              Serial.print(" a22:");
              Serial.println(sensor2.getMechanicalAngle(), 6);
              break;

      case 19:
              //
              Serial.print(" servo1:");
              Serial.print(zeroBias.servo1);
              Serial.print(" servo2:");
              Serial.print(zeroBias.servo2);
              Serial.print(" servo3:");
              Serial.print(zeroBias.servo3);
              Serial.print(" servo4:");
              Serial.println(zeroBias.servo4);
              break;

      case 20:
              Serial.print(" vra:");
              Serial.print(sbus_vra, 6);
              Serial.print(" BodyRoll:");
              Serial.print(BodyRoll, 6);
              Serial.print(" LegLength:");
              Serial.println(LegLength, 6);
              break;

      case 21:
              Serial.print(" roll_ok:");
              Serial.print(roll_ok, 6);
              Serial.print(" BodyPitching:");
              Serial.println(BodyPitching, 6);
              break;


      case 22:
              /*
              Serial.print(" PP:");
              Serial.print(Angle_Pid.Kp, 5);
              Serial.print(" PI:");
              Serial.print(Angle_Pid.Ki, 5);
              Serial.print(" PD:");
              Serial.print(Angle_Pid.Kd, 5);   
              */  
              Serial.print(" it:");
              Serial.print(Angle_Pid.iLimit, 5);
              Serial.print(" il:");
              Serial.print(Angle_Pid.integral, 5);      
              Serial.print(" oI:");
              Serial.print(Angle_Pid.outI, 5);
              Serial.print(" out:");
              Serial.println(Angle_Pid.output, 5);
              break;


      case 23:
            /*
              Serial.print(" SP:");
              Serial.print(Speed_Pid.Kp, 5);
              Serial.print(" SI:");
              Serial.print(Speed_Pid.Ki, 5);
              */
              Serial.print(" it:");
              Serial.print(Speed_Pid.iLimit, 5);
              Serial.print(" il:");
              Serial.print(Speed_Pid.integral, 5);         
              Serial.print(" oI:");
              Serial.print(Speed_Pid.outI, 5);
              Serial.print(" A:");
              Serial.print(BodyPitching_f, 5);
              Serial.print(" out:");
              Serial.println(Speed_Pid.output, 5);
              break;

      case 24:
              Serial.print(" EN:");
              Serial.print(enableDFilter);
              Serial.print(" HZ:");
              Serial.println(cutoffFreq, 5);
              break;

      case 25:
              Serial.print(" LpfOut:");
              Serial.print(BodyPitching_f, 6);
              Serial.print(" BodyPitching:");
              Serial.println(BodyPitching, 6);
              break;

      case 26:

              if(Touch.state == 1)
              {
                Serial.print("  aX:");
                Serial.print(Touch.XPdat); 
                Serial.print("  aY:");
                Serial.println(Touch.YPdat);          
              }
              else if(Touch.state == 0)
              {
                Serial.print("  tX:");
                Serial.print(Touch.XLdat); 
                Serial.print("  tY:");
                Serial.println(Touch.YLdat);           
              }
              break;


      case 27:

              Serial.print("  aX:");
              Serial.print(Touch.XPdat); 
              Serial.print("  aY:");
              Serial.print(Touch.YPdat);          
              Serial.print("  aXF:");
              Serial.print(Touch.XPdatF); 
              Serial.print("  aYF:");
              Serial.println(Touch.YPdatF);  
              break;

      case 28:

              Serial.print("  P:");
              Serial.print(BodyPitching_f); 
              Serial.print("  R:");
              Serial.print(BodyRoll_f,5);          
              Serial.print("  H:");
              Serial.print(LegLength_f,5); 
              Serial.print("  S:");
              Serial.print(SlideStep_f);  
              Serial.print("  vra:");
              Serial.print(sbus_vra);
              Serial.print("  vra:");
              Serial.println(sbus_vrb);
              break;

      case 29:
              Serial.print(" Kp:");
              Serial.print(TouchY_Pid.Kp,6);
              Serial.print(" Ki:");
              Serial.print(TouchY_Pid.Ki,6);      
              Serial.print(" Kd:");
              Serial.print(TouchY_Pid.Kd,6);

              Serial.print(" deriv:");
              Serial.print(TouchY_Pid.deriv);
              Serial.print(" out:");
              Serial.println(TouchY_Pid.output);
              break;

      case 30:
              Serial.print(" deriv:");
              Serial.println(TouchY_Pid.deriv);
              break;

      case 31:
              Serial.print(" E:");
              Serial.print(Roll_Pid.error, 6);
              Serial.print(" it:");
              Serial.print(Roll_Pid.iLimit, 5);
              Serial.print(" il:");
              Serial.print(Roll_Pid.integral, 5);         
              Serial.print(" oI:");
              Serial.print(Roll_Pid.outI, 5);
              Serial.print(" out:");
              Serial.println(Roll_Pid.output, 5);
              break;

      case 32:

              Serial.print(" RP:");
              Serial.print(Roll_Pid.Kp, 6);
              Serial.print(" RI:");
              Serial.print(Roll_Pid.Ki, 6);
              Serial.print(" RD:");
              Serial.println(Roll_Pid.Kd, 6);   
              break;
              
      case 33:
              Serial.print(" it:");
              Serial.print(Yaw_Pid.iLimit, 5);
              Serial.print(" il:");
              Serial.print(Yaw_Pid.integral, 5);         
              Serial.print(" oI:");
              Serial.print(Yaw_Pid.outI, 5);
              Serial.print(" A:");
              Serial.print(BodyPitching_f, 5);
              Serial.print(" out:");
              Serial.println(Yaw_Pid.output, 5);
              break;

      case 34:
              Serial.print(" Kp:");
              Serial.print(TouchX_Pid.Kp,6);
              Serial.print(" Ki:");
              Serial.print(TouchX_Pid.Ki,6);      
              Serial.print(" Kd:");
              Serial.print(TouchX_Pid.Kd,6);

              Serial.print(" deriv:");
              Serial.print(TouchX_Pid.deriv);
              Serial.print(" out:");
              Serial.println(TouchX_Pid.output);
              break;

      case 35:
              Serial.print(" deriv:");
              Serial.println(TouchX_Pid.deriv);
              break;

      case 36:
              Serial.print(" state:");
              Serial.print(Touch.state);
              Serial.print(" start:");
              Serial.println(Touch.start);            
              break;

      case 37:
              Serial.print(" sbus_vra:");
              Serial.print(sbus_vra);
              Serial.print(" sbus_vraf:");
              Serial.print(sbus_vraf);      
              Serial.print(" sbus_vrb:");
              Serial.print(sbus_vrb);
              Serial.print(" sbus_vrbf:");
              Serial.println(sbus_vrbf);            
              break;

      case 38:
              Serial.print(" X OUT:");
              Serial.print(BodyPitching);
              Serial.print(" Y OUT:");
              Serial.println(TouchY_Pid.output);            
              break;            


      case 39:
              Serial.print(" it:");
              Serial.print(TouchY_Pid.iLimit, 5);
              Serial.print(" il:");
              Serial.print(TouchY_Pid.integral, 5);         
              Serial.print(" oI:");
              Serial.print(TouchY_Pid.outI, 5);
              Serial.print(" out:");
              Serial.println(TouchY_Pid.output, 5);
              break;
              

      case 40:

              if(Touch.state == 1)
              {
                Serial.print("  aX:");
                Serial.print(Touch.XPressDat); 
                Serial.print("  aY:");
                Serial.println(Touch.YPressDat);          
              }
              else if(Touch.state == 0)
              {
                Serial.print("  tX:");
                Serial.print(Touch.XPressDat); 
                Serial.print("  tX:");
                Serial.println(Touch.YPressDat);           
              }
              break;

      case 41:

              Serial.print(" roll_ok:");
              Serial.print(roll_ok, 5);            
              Serial.print(" pa:");
              Serial.print(BodyPitching, 5);
              Serial.print(" out:");
              Serial.println(Speed_Pid.output, 5);
              break;

      case 42:

              Serial.print(" P:");
              Serial.print(roll_ok, 5);            
              Serial.print(" P1:");
              Serial.print(BodyPitching, 5);
              Serial.print(" P3:");
              Serial.println(BodyPitchingCorrect(BodyPitching_f), 5);
              break;

      case 43:

              Serial.print(" Vdat:");
              Serial.print(VoltageADC);            
              Serial.print(" Vdatf:");
              Serial.print(VoltageADCf);
              Serial.print(" V:");
              Serial.println(Voltage, 5);
              break;

      case 44:
  
              Serial.print(" PidParameterTuning:");
              Serial.print(PidParameterTuning);
              Serial.print(" TargetLegLength:");
              Serial.println(TargetLegLength, 6);
              
              break;

      case 45:

              Serial.print(" RobotTumble:");
              Serial.print(RobotTumble);   
              Serial.print(" roll_ok:");
              Serial.print(roll_ok, 6);    
              Serial.print(" Angle_Pid.error:");
              Serial.println(Angle_Pid.error, 6);
    
              break;

      case 46:


    
              break;

      case 47: 
              Serial.print(" Serial1HZ:");
              Serial.print(body.Serial1HZ); 
              
              Serial.print(" sbus_swb:");
              Serial.println(sbus_swb); 
    
              break;    

      case 48: 
              Serial.print(" xo3:");
              Serial.print(body.xo3*100,4); 
          
              Serial.print(" zo3:");
              Serial.print(body.zo3*100,4); 
              Serial.print(" Ts:");
              Serial.println(body.Ts,4);  
    
              break;  

      case 49: 
        
              Serial.print(" xo4:");
              Serial.print(body.xo4*100,4); 
          
              Serial.print(" zo4:");
              Serial.print(body.zo4*100,4); 
          
              Serial.print(" Ts:");
              Serial.println(body.Ts,4);  
    
              break;   

      case 50: 
        
              Serial.print(" body.Ts:");
              Serial.print(body.Ts,4); 
          
              Serial.print(" bodyH:");
              Serial.println(sbus_vrb,4);  
    
              break;   

      case 51: 
              Serial.print(" mv1:");
              Serial.print(body.MotorVelocityF[0],3); 
              Serial.print(" mv2:");
              Serial.print(body.MotorVelocityF[1],3); 
              Serial.print(" mv3:");
              Serial.print(body.MotorVelocityF[2],3); 
              Serial.print(" mv4:");
              Serial.println(body.MotorVelocityF[3],3); 
              break;                                    
        
      case 52: 
              Serial.print(" xt:");
              Serial.print(body.xt,4); 
              Serial.print(" h:");
              Serial.print(body.h,4); 
              Serial.print(" Ts:");
              Serial.println(body.Ts,4); 
              break;   

      case 53: 
      
              Serial.print(" BodyPitching4WheelTF:");
              Serial.print(body.BodyPitching4WheelTF,4);     
              Serial.print(" BodyPitching4Wheel:");
              Serial.println(body.BodyPitching4Wheel,4); 
              break;  

      case 54:
              Serial.print(" E:");
              Serial.print(Pitching_Pid.error, 6);
              Serial.print(" it:");
              Serial.print(Pitching_Pid.iLimit, 5);
              Serial.print(" il:");
              Serial.print(Pitching_Pid.integral, 5);         
              Serial.print(" oI:");
              Serial.print(Pitching_Pid.outI, 5);
              Serial.print(" out:");
              Serial.println(Pitching_Pid.output, 5);
              break;
              
            
      default:

        break;
    }
  }





  float BodyPitchingCorrect(float x)//Pitch angle correction
  {
    float y = 0.000004*x*x + 0.0004*x - 0.0008;//y = 4E-06x2 + 0.0004x - 0.0008    y = -2E-07x2 + 0.0002x - 0.0029
    return y;
  }




  void PIDcontroller_angle(float dt)
  {
    //Speed loop 
    Speed_Pid.Kp = SpeedPid.P;
    Speed_Pid.Ki = SpeedPid.I;
    Speed_Pid.Kd = SpeedPid.D;

    float speedError = (Motor1_Velocity_f + Motor2_Velocity_f) * 0.5 - MovementSpeed; //Measured value minus target value
    float speedOutput = Speed_Pid.compute(speedError, dt);

    //Balance loop
    Angle_Pid.Kp = AnglePid.P;
    Angle_Pid.Ki = AnglePid.I;
    Angle_Pid.Kd = AnglePid.D;

    //LpfOut BodyPitching
    float angleError = roll_ok - speedOutput - (-BodyPitching); //Measured value minus target value
    float angleOutput = Angle_Pid.compute(angleError, dt);

    //Turn loop
    Yaw_Pid.Kp = YawPid.P;
    Yaw_Pid.Ki = YawPid.I;
    Yaw_Pid.Kd = YawPid.D;

    float yawError = attitude.gyro.z - BodyTurn;//Measured value minus target value
    float yawOutput = Yaw_Pid.compute(yawError, dt);

    float target1 = angleOutput - yawOutput;
    float target2 = angleOutput + yawOutput;

    if (control_torque_compensation != 0)
    {
      if (target1 > 0)
        target1 = target1 + control_torque_compensation;
      else if (target1 < 0)
        target1 = target1 + (-control_torque_compensation);

      if (target2 > 0)
        target2 = target2 + control_torque_compensation;
      else if (target2 < 0)
        target2 = target2 + (-control_torque_compensation);
    }

    motor1.target = target1;
    motor2.target = target2;
  }


  void PidParameter(void)
  {
    if(sbus_swa == 1)//No touch screen
    {
        //Roll
        RollPid.P = 0.06;
        RollPid.I = 1.5;
        RollPid.D = 0.0028;
        RollPid.limit = 2;//Integral limit

        //Speed loop
        SpeedPid.P = 0.1;
        SpeedPid.I = 0.1;
        SpeedPid.D = 0;
        SpeedPid.limit = 50;//Integral limit    

        //Balance loop
        AnglePid.P = 7;
        AnglePid.I = 222;
        AnglePid.D = 0.08;
        AnglePid.limit = 0.1;//Integral limit
        
    } 
    else if(sbus_swa == 2)//With touch screen
    {
        //Roll
        RollPid.P = 0.08;
        RollPid.I = 1.5;
        RollPid.D = 0.005;
        RollPid.limit = 2;//Integral limit

        //Speed loop
        SpeedPid.P = 0.12;
        SpeedPid.I = 0.12;
        SpeedPid.D = 0;
        SpeedPid.limit = 50;//Integral limit    

        //Balance loop
        AnglePid.P = 9;
        AnglePid.I = 222;
        AnglePid.D = 0.11;
        AnglePid.limit = 0.1;//Integral limit
            
    } 

      YawPid.P = 11;
      YawPid.I = 33; 
      YawPid.D = 0;
      YawPid.limit = 0;  

      //Touch screen
      TouchXPid.P = 0.2;
      TouchXPid.I = 0;
      TouchXPid.D = 0.04;
      TouchXPid.limit = 0;//Integral limit
      
      TouchYPid.P = 0.2;
      TouchYPid.I = 0;
      TouchYPid.D = 0.08;
      TouchYPid.limit = 0;//Integral limit
  }


  void PIDcontroller_posture(float dt)
  {
    if((int)PidParameterTuning==0)
      PidParameter();
    
    //Touch screen
    TouchX_Pid.Kp = TouchXPid.P/100;
    TouchX_Pid.Ki = TouchXPid.I/100;
    TouchX_Pid.Kd = TouchXPid.D/100;
    TouchX_Pid.iLimit = TouchXPid.limit;//Integral limit
    
    TouchY_Pid.Kp = TouchYPid.P/100;
    TouchY_Pid.Ki = TouchYPid.I/100;
    TouchY_Pid.Kd = TouchYPid.D/100;
    TouchY_Pid.iLimit = TouchYPid.limit;//Integral limit
    
    float touchXError = Touch.XPdatF;  
    float touchYError = Touch.YPdatF;  
    static float TouchX_kd = 0;
    static float TouchY_kd = 0;

    TouchX_Pid.compute(touchXError, dt); 
    TouchY_Pid.compute(touchYError, dt);   

    TouchX_Pid.deriv = constrain(TouchX_Pid.deriv, -11000, 11000);
    TouchX_Pid.outD = TouchX_kd * TouchX_Pid.deriv;  
    
    if((sbus_swd == 2)||(sbus_swc == 1))//Top ball mode 
    {
      BodyPitching = TouchX_Pid.outP + TouchX_Pid.outI + TouchX_Pid.outD + sbus_vraf;
      BodyPitching = -BodyPitching;    
    }  

    TouchY_Pid.deriv = constrain(TouchY_Pid.deriv, -11000, 11000);
    TouchY_Pid.outD = TouchY_kd * TouchY_Pid.deriv;  
    TouchY_Pid.output = TouchY_Pid.outP + TouchY_Pid.outI + TouchY_Pid.outD + sbus_vrbf;
    if((int)enableDFilter==1)
    {
      TouchY_Pid_outputF = biquadFilterApply(&FilterLPF[10], TouchY_Pid.output);    
      TouchX_Pid_outputF = biquadFilterApply(&FilterLPF[11], TouchX_Pid.output);  
    }
    else
    {
      TouchY_Pid_outputF = TouchY_Pid.output;    
      TouchX_Pid_outputF = TouchX_Pid.output;   
    }

    if((Touch.state==1)&&(Touch.P_count<4))
    {
      Touch.P_count++;
      TouchX_Pid.integral = 0;
      //TouchX_Pid.output = 0;
      //BodyPitching = 0;
      TouchY_Pid.integral = 0;
      //TouchY_Pid_outputF = 0;  
    }
      
    if(Touch.P_count>=4)
    {
      if(TouchX_kd<TouchX_Pid.Kd)
      {
        TouchX_kd = TouchX_kd + (TouchX_Pid.Kd/22);
      }
      if(TouchY_kd<TouchY_Pid.Kd)
      {
        TouchY_kd = TouchY_kd + (TouchY_Pid.Kd/22);
      }  
      Touch.start = 2;
    } 

    if((Touch.state==0)&&(Touch.L_count<44))
      Touch.L_count++;
    else
      Touch.L_count = 0;
        
    if(Touch.L_count>=44)
    {
      TouchX_Pid.integral = 0;
      //TouchX_Pid.output = 0;
      //BodyPitching = 0;
      TouchY_Pid.integral = 0;
      //TouchY_Pid_outputF = 0;  
      Touch.P_count = 0;
      TouchX_kd = 0;   
      TouchY_kd = 0; 
      Touch.start = -2;    
    }  
    
    if((sbus_swd != 2)||(sbus_swc != 1))//Non-top ball mode 
    {
      TouchX_Pid.integral = 0;
      TouchX_Pid.output = 0;
      //BodyPitching = 0;
      TouchY_Pid.integral = 0;
      TouchY_Pid.output = 0;
      TouchY_Pid_outputF = 0; 
      TouchX_kd = 0;   
      TouchY_kd = 0; 
    }



    //Roll
    Roll_Pid.Kp = RollPid.P/100;
    Roll_Pid.Ki = RollPid.I/100;
    Roll_Pid.Kd = RollPid.D/100;
    Roll_Pid.iLimit = RollPid.limit;//Integral limit

    float TargetBodyRoll = BodyRoll_f*777; //Roll
    if(sbus_swd==2)//Top ball禁止手动横滚
      TargetBodyRoll = 0;
    float RollError = (-pitch_ok) - (-TargetBodyRoll) - (-TouchY_Pid_outputF);  
    if(sbus_swc == 1)//Roll leveling
    {
        Roll_Pid.compute(RollError, dt);    
    }
    else
    {
        Roll_Pid.output = 0;
        Roll_Pid.integral = 0;
    }


    //Speed loop
    Speed_Pid.Kp = SpeedPid.P/100;
    Speed_Pid.Ki = SpeedPid.I/100;
    Speed_Pid.Kd = SpeedPid.D/100;
    Speed_Pid.iLimit = SpeedPid.limit;//Integral limit
    
    float speedError = (Motor1_Velocity_f + Motor2_Velocity_f) * 0.5 - MovementSpeed; //Measured value minus target value
    BodyX = Speed_Pid.compute(speedError, dt) + BodyPitchingCorrect(BodyPitching_f);// 
    
    //Balance loop
    Angle_Pid.Kp = AnglePid.P;
    Angle_Pid.Ki = AnglePid.I;
    Angle_Pid.Kd = AnglePid.D;
    Angle_Pid.iLimit = AnglePid.limit;//Integral limit


    float angleError = roll_ok - (-BodyPitching_f); //Measured value minus target value
    float angleOutput = Angle_Pid.compute(angleError, dt);

    //Yaw loop
    Yaw_Pid.Kp = YawPid.P;
    Yaw_Pid.Ki = YawPid.I; 
    Yaw_Pid.Kd = YawPid.D;
    Yaw_Pid.iLimit = YawPid.limit;
    if(sbus_swc != 1)//Lock heading angle
    {
      Yaw_Pid.Ki = 0;
      Yaw_Pid.integral = 0;
      
    }
    
    float yawError = attitude.gyro.z - BodyTurn;//Measured value minus target value
    float yawOutput = Yaw_Pid.compute(yawError, dt);

    float target1 = angleOutput - yawOutput;
    float target2 = angleOutput + yawOutput;

    if (control_torque_compensation != 0)
    {
      if (target1 > 0)
        target1 = target1 + control_torque_compensation;
      else if (target1 < 0)
        target1 = target1 + (-control_torque_compensation);

      if (target2 > 0)
        target2 = target2 + control_torque_compensation;
      else if (target2 < 0)
        target2 = target2 + (-control_torque_compensation);
    }

    motor1.target = target1;
    motor2.target = target2;
  
  }




  void RemoteControlFiltering(void)//Remote control filter
  {
    static int enableDFilter_last = (int)enableDFilter;
    static int cutoffFreq_last = (int)cutoffFreq;

    sbus_vraf = biquadFilterApply(&FilterLPF[8], sbus_vra);
    sbus_vrbf = biquadFilterApply(&FilterLPF[9], sbus_vrb); 

    if((int)enableDFilter==1)
    {
      if(body.MotorMode>=3)
        body.BodyPitching4WheelT = mapf(sBus.channels[1], SBUS_chMin, SBUS_chMax, -0.011, 0.011); //Pitch      
      else
        BodyPitching_f = biquadFilterApply(&FilterLPF[0], BodyPitching);// 

      body.BodyPitching4WheelTF = biquadFilterApply(&FilterLPF[12], body.BodyPitching4WheelT);         
      BodyRoll_f = biquadFilterApply(&FilterLPF[1], BodyRoll);
      LegLength_f = biquadFilterApply(&FilterLPF[2], LegLength);
      SlideStep_f = biquadFilterApply(&FilterLPF[3], SlideStep);
      
    }
    else
    {
      BodyPitching_f = BodyPitching;
      BodyRoll_f = BodyRoll;
      LegLength_f = LegLength;
      SlideStep_f = SlideStep;    
    }
    

    if(((int)enableDFilter!=enableDFilter_last)||((int)cutoffFreq!=cutoffFreq_last))
    {
      for (int i = 0; i < 6; i++)
      {
        biquadFilterInitLPF(&FilterLPF[i], 100, (unsigned int)cutoffFreq);//Remote control filter
        //TouchscreenInit((unsigned int)cutoffFreq);  
      }

      enableDFilter_last = (int)enableDFilter;
      cutoffFreq_last = (int)cutoffFreq;   
      Serial.println(" "); 
      Serial.println(" ok "); 
    } 
      
  }

  void ReadVoltage(void)
  {
    VoltageADC = analogRead(analogInPin);
    VoltageADCf = biquadFilterApply(&VoltageFilterLPF, VoltageADC);
    Voltage = (float)7.77/813.43*VoltageADCf;
  }


  void Robot_Tumble(void)
  {
      static int x = 0;
      if(abs(roll_ok)>=35)
      {
        x++;
        if(x>=20)
        {
          x=20;
          RobotTumble=1;//Machine fall
        }      
      }  
      else
      {
        if((RobotTumble==1)&&(abs(roll_ok)<=5))//Machine fall after fall
        {
          x--;
          if(x<=0)
          {
            x = 0;
            RobotTumble = 0;  
          }
        }
      }
  }





  void body_data_init(void)//
  {
    //  Gait parameters
    body.delayTime = 0.005;   // Delay time per step
    body.CurrentSteps = 0;//Current step  
    body.xt = 0.015;     // Starting position
    body.xs1 = 0;     // Starting position
    body.xf1 = body.xt;  // End position
    body.xs2 = 0;     // Starting position
    body.xf2 = body.xt;  // End position  
    body.xs3 = 0;     // Starting position
    body.xf3 = body.xt;  // End position  
    body.xs4 = 0;     // Starting position
    body.xf4 = body.xt;  // End position      
    body.h  =  0.02;   // Highest position
    body.zs = 0;     // Starting height
    body.Ts = 0.5;     // Period

    body.lambda[0] = 0.5;// λ parameter 
    body.lambda[1] = 1.0f;

    body.H_fron = 0.075;  //     
    body.H_back = 0.075;;  //   
    
    body.MotorMode = 0;  //
    
  }


  void TrotGaitAlgorithm(void)//Trot gait
  {
      body.CurrentSteps = body.CurrentSteps + body.delayTime;//Gait time
      if(body.CurrentSteps > body.Ts)
      {
        body.CurrentSteps = 0;   
      }

      if((body.CurrentSteps >= 0)&&(body.CurrentSteps < (body.lambda[0]*body.Ts)))//First stage
      {
        
        body.sigma = 2*PI*body.CurrentSteps/(body.lambda[0]*body.Ts);//First stage time converted to 360 degrees
      
        body.xo1 = (body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (-body.xt/2); // Cycloidal trajectory calculation
        body.zo1 = body.h * (1 - cos(body.sigma)) + body.zs;

        body.xo2 = (body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (-body.xt/2); // Cycloidal trajectory calculation
        body.zo2 = 0;  

        body.xo3 = (body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (-body.xt/2); // Cycloidal trajectory calculation
        body.zo3 = 0;  

        body.xo4 = (body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (-body.xt/2); // Cycloidal trajectory calculation
        body.zo4 = body.h * (1 - cos(body.sigma)) + body.zs; 
    
      }
      else if((body.CurrentSteps >= (body.lambda[0]*body.Ts))&&(body.CurrentSteps < (body.lambda[1]*body.Ts)))//Second stage 
      {
        body.sigma = 2*PI*(body.CurrentSteps - (body.lambda[0]*body.Ts) ) / ((body.lambda[1]-body.lambda[0])*body.Ts);//Second stage time converted to 360 degrees

        body.xo1 = (-body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (body.xt/2); // Cycloidal trajectory calculation
        body.zo1 = 0;  

        body.xo2 = (-body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (body.xt/2); // Cycloidal trajectory calculation
        body.zo2 = body.h * (1 - cos(body.sigma)) + body.zs;    

        body.xo3 = (-body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (body.xt/2); // Cycloidal trajectory calculation
        body.zo3 = body.h * (1 - cos(body.sigma)) + body.zs;

        body.xo4 = (-body.xt) * (body.sigma - sin(body.sigma)) / (2 * PI) + (body.xt/2); // Cycloidal trajectory calculation
        body.zo4 = 0;         

      } 

      if(body.MotorMode<2)
      {
        if(body.xt<0)
        {
          body.zo1 = body.zo1 - 0.01;
          body.zo2 = body.zo2 - 0.01;      
        }
        if(body.xt>0)
        {
          body.zo3 = body.zo3 - 0.01;
          body.zo4 = body.zo4 - 0.01;
        }      
      }
      else if(body.MotorMode >= 2)
      {
        body.zo1 = body.zo1 + LegLength - body.BodyRoll4Wheel + body.BodyPitching4Wheel - Roll_Pid.output - TouchX_Pid_outputF + TouchY_Pid_outputF;
        body.zo2 = body.zo2 + LegLength + body.BodyRoll4Wheel + body.BodyPitching4Wheel + Roll_Pid.output + TouchX_Pid_outputF + TouchY_Pid_outputF;
        body.zo3 = body.zo3 + LegLength - body.BodyRoll4Wheel - body.BodyPitching4Wheel - Roll_Pid.output - TouchX_Pid_outputF - TouchY_Pid_outputF;
        body.zo4 = body.zo4 + LegLength + body.BodyRoll4Wheel - body.BodyPitching4Wheel + Roll_Pid.output + TouchX_Pid_outputF - TouchY_Pid_outputF;
      }
      
      if((int)Select==46)
      {
        Serial.print(" XT:");
        Serial.print(body.xt*100,4); 
        
        Serial.print(" xo1:");
        Serial.print(body.xo1*100,4); 
    
        Serial.print(" zo1:");
        Serial.print(body.zo1*100,4); 
    
        Serial.print(" xo2:");
        Serial.print(body.xo2*100,4); 
    
        Serial.print(" zo2:");
        Serial.print(body.zo2*100,4); 
    
        Serial.print(" sigma:");
        Serial.println(body.sigma,4);       
      }
  }


  void loop() {
    now_us = micros();
    //now_us2 = micros();

    // iterative function setting the outter loop target

    if (Communication_object == 1)
    {
      motor1.monitor();//When using the simpleFOC Studio upper computer, this sentence must be opened. But it will affect the program execution speed
    }
    else if (Communication_object == 2)
    {
      motor2.target = motor1.target;
    }

    motor1.move();
    motor2.move();

    //Torque compensation
    float indexF = sensor1.getMechanicalAngle() / AngleResolutionRatio;
    int index = round(indexF);
    if ((TorqueCompensation == 1) && (SwitchUser != 1) && (SwitchUser != 2))
    {
      motor1.current_sp = motor1.current_sp + Motor1_Current_sp_data[index];
    }

    indexF = sensor2.getMechanicalAngle() / AngleResolutionRatio;
    index = round(indexF);
    if ((TorqueCompensation == 1) && (SwitchUser != 1) && (SwitchUser != 2))
    {
      motor2.current_sp = motor2.current_sp + Motor2_Current_sp_data[index];
    }


    // iterative setting FOC phase voltage
    motor1.loopFOC();
    motor2.loopFOC();


    //Serial.print(Motor2_voltage_compensation,6);
    //Serial.print("\t");
    //Serial.println(motor2.voltage.q+Motor2_voltage_compensation,6);

    RightMotorAngle = -sensor1.getPreciseAngle();
    LeftMotorAngle  = sensor2.getPreciseAngle();

    //user communication
    command.run();
    


    if(MasterSlaveSelection==1)
    {
      ImuUpdate();//Update IMU data   
      RXsbus(); 
    }

    

    if((SwitchingPattern==0)||(MasterSlaveSelection==0))//Two-wheel or slave mode
      ReadTouchDat();
      
    time_dt = (now_us - now_us1) / 1000000.0f;
    if (time_dt >= 0.005f)
    {
      static int Serial1_count = 0;
      Serial1_count++;
      if(Serial1_count>=5)
      {
        Serial1_count = 0;
        body.Serial1HZ =  body.Serial1count*40;
        body.Serial1count = 0;
      }
4轮模式
      if(SwitchingPattern==1)//4-wheel mode
        MotorOperatingMode();
      
      if((MasterSlaveSelection==0)&&(SwitchingPattern==1))//Slave && 4-wheel mode
        Read_Serial2();
      else if((MasterSlaveSelection==1)&&(SwitchingPattern==1))//Host && 4-wheel mode
        Read_Serial1();

      if((MasterSlaveSelection==0)||(SwitchingPattern==0))//Slave || 2-wheel mode
        TouchBiquadFilter();//Touch screen filter
        
      RemoteControlFiltering();//Remote control signal filtering
      ReadVoltage();//Battery
      print_data();//Serial port data printing

      if(SwitchingPattern==0)//2-wheel mode
        Robot_Tumble();//Machine fall detection
      LED_count++;
      if(LED_count>=LED_dt)
      {
        //Serial.print(LED_HL);
        //Serial.println(" LED:");
        LED_count = 0 ;
        if(LED_HL==1)
        {
          digitalWrite(LED_Pin, LOW);   //On 
          LED_HL = 0;
        }
        else
        {
          digitalWrite(LED_Pin, HIGH);  //Off 
          LED_HL = 1;
        }
        
      }
      
      if(Voltage<=7.4)
        LED_dt = 20;
      else
        LED_dt = 100;
        
      
      
      if (RATE_HZ !=  RATE_HZ_last)
      {
        //Initialize second-order low-pass filter
        for (int axis = 0; axis < 6; axis++)
        {
          biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
        }
        Serial.print(" RATE_HZ:");
        Serial.print(RATE_HZ);
        RATE_HZ_last = RATE_HZ;
      }
      if (LPF_CUTOFF_FREQ !=  LPF_CUTOFF_FREQ_last)
      {
        //Initialize second-order low-pass filter
        for (int axis = 0; axis < 6; axis++)
        {
          biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
        }
        Serial.print(" LPF_CUTOFF_FREQ:");
        Serial.print(LPF_CUTOFF_FREQ);
        LPF_CUTOFF_FREQ_last = LPF_CUTOFF_FREQ;
      }

      FlashSave((int)CalibrationSelect);//Save calibration data

      Motor1_Velocity = (sensor1.getAngle() - Motor1_place_last) / 0.01f;
      Motor1_Velocity_f = Motor1_Velocity_filter(Motor1_Velocity);
      Motor1_place_last = sensor1.getAngle();

      Motor2_Velocity = -(sensor2.getAngle() - Motor2_place_last) / 0.01f;
      Motor2_Velocity_f = Motor2_Velocity_filter(Motor2_Velocity);
      Motor2_place_last = sensor2.getAngle();

      body.MotorVelocityF[0]= Motor1_Velocity_f;
      body.MotorVelocityF[1]= Motor2_Velocity_f;


      if ((SwitchUser == 1) && (Slot_calibration_mark == 0))
      {
        Serial.print(" motor1 ");
        CalibrationCurrentSp(-sensor1.getAngle(), Motor1_Velocity_f, &motor1);
      }
      if ((SwitchUser == 2) && (Slot_calibration_mark == 0))
      {
        Serial.print(" motor2 ");
        CalibrationCurrentSp(sensor2.getAngle(), Motor2_Velocity_f, &motor2);
      }

      float Rax[2];
      float Lax[2];
      float bodyH = 0.06f;
      float bodyRoll = BodyRoll_f;

      if(MasterSlaveSelection==1)//Host mode
      {
        if((sbus_swa == 0)||(RobotTumble == 1))
        {
          if (Communication_object == 0 && SwitchUser != 1 && SwitchUser != 2) //
          {
            motor1.target = 0;
            motor2.target = 0;
    
          }
    
          bodyH = 0.06;
          bodyRoll = 0;
          BodyX = 0;
          bodyRoll = 0;
          BodyPitching_f = 0;
    
          Angle_Pid.integral = 0;
          Speed_Pid.integral = 0;
          Yaw_Pid.integral = 0;
          
          body.zo1 = 0;
          body.zo2 = 0;
          body.zo3 = 0;
          body.zo4 = 0;      
      
          body.xo1 = 0;
          body.xo2 = 0;      
          body.xo3 = 0;
          body.xo4 = 0;      

          body_data_init();
        }
        else if((sbus_swa >= 1)&&(RobotTumble == 0))//
        {

          if(SwitchingPattern==0)//2-wheel mode
          {
            PIDcontroller_posture(time_dt);//PID controller

            body.zo1 = 0;
            body.zo2 = 0;
            body.zo3 = 0;
            body.zo4 = 0;      
        
            body.xo1 = 0;
            body.xo2 = 0;      
            body.xo3 = 0;
            body.xo4 = 0;
    
            if(sbus_swc == 1)
              bodyRoll = Roll_Pid.output;
        
            if(TargetLegLength==0)
              bodyH = LegLength_f;
            else
              bodyH = TargetLegLength;
    
          }
          else if((SwitchingPattern==1)&&(MasterSlaveSelection==1))//4-wheel mode && Host mode
          {
            TrotGaitAlgorithm();//Gait
            PIDcontroller_posture_4wheel(time_dt);
            
            Send_Serial1();//Send data to slave
            motor1.target = body.MT[0];
            motor2.target = body.MT[1];          
            

            bodyH = body.H_fron;
              
            bodyRoll = 0;
            BodyX = 0;
            bodyRoll = 0;
            BodyPitching_f = 0;      
          }

    
        }      
      }
      else //Slave mode
      {

        bodyH = body.H_back;
        
        bodyRoll = 0;
        BodyX = 0;
        bodyRoll = 0;
        BodyPitching_f = 0;  

        Send_Serial2();
        if(body.Serial1HZ>=50)
        {
          
          body.xo2 = -body.xo4;
          body.xo1 = -body.xo3;    
          body.zo2 = body.zo4;
          body.zo1 = body.zo3;  

          motor1.target = body.MT[2];
          motor2.target = body.MT[3];
                  
        } 
        else
        {
          bodyH = 0.06;
          bodyRoll = 0;
          BodyX = 0;
          bodyRoll = 0;
          BodyPitching_f = 0;
                  
          body.xo2 = 0;
          body.xo1 = 0;    
          body.zo2 = 0;
          body.zo1 = 0;   

          motor1.target = 0;
          motor2.target = 0;                
        } 
      }
    

      if(RobotTumble == 1)//Machine fall
      {
        bodyH = 0.06;
        bodyRoll = 0;
        BodyX = 0;
        bodyRoll = 0;
        BodyPitching_f = 0;
      }

      
      if (RightInverseKinematics(BarycenterX - BodyX + body.xo2, bodyH - bodyRoll - body.zo2, BodyPitching_f, Rax))
        Serial.println("RightInverseKinematics no");

      if (LeftInverseKinematics(BarycenterX - BodyX - body.xo1, bodyH + bodyRoll - body.zo1 , BodyPitching_f, Lax))
        Serial.println("LeftInverseKinematics no");

        

      if(sbus_swb == 0)//Posture
      {

        // Set the angle of the four servos
        servoControl.setServosAngle(1, Lax[0] - zeroBias.servo1, -1, Lax[1] - zeroBias.servo2, -1, Rax[0] - zeroBias.servo3, 1, Rax[1] - zeroBias.servo4,1);
      }
      else//Assembly position and calibration
      {
          if ((int)CalibrationSelect == 3)//Servo calibration
          {
            servoControl.setServosAngle(1, 0 - zeroBias.servo1, -1, 0 - zeroBias.servo2, -1, 0 - zeroBias.servo3, 1, 0 - zeroBias.servo4,1); //标定偏差
          }
          else
          {
            servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0,1); //Assembly position
          }      
      }


      now_us1 = now_us;
    }
    //  Serial.print("  dt:");
    //  Serial.println(micros()-now_us2);

  }



  void MotorOperatingMode(void)
  {
    if(motor1.controller!=MotionControlType::velocity)
    {
      motor1.controller = MotionControlType::velocity;
      motor2.controller = MotionControlType::velocity;            
    }    
    if(MasterSlaveSelection == 1)//Host
    {
      if(sbus_swc==0)
      {  
          if((sbus_swd==0)&&(SwitchingPattern==1))
          {
            body.MotorMode = 0; 
            body.BodyPitching4Wheel = 0;
            body.xt =  mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -0.04, 0.04);//Step length 
            body.h  =  0.025;//mapf(sBus.channels[8], SBUS_chMin, SBUS_chMax, 0.005, 0.02);//Step height VRA
            //body.Ts =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, 0.5, 1);//Stride period S VRB
      
            BodyTurn = mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -55, 55);
            
            body.MT[0] = -BodyTurn;
            body.MT[1] = BodyTurn;
            body.MT[2] = -BodyTurn;
            body.MT[3] = BodyTurn;
            
          }
          else if((sbus_swd==1)&&(SwitchingPattern==1))
          {
            body.MotorMode = 1;  

            body.BodyPitching4Wheel = 0;
            body.xt =  mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -0.04, 0.04);//Step length 
            MovementSpeed =  -mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -33, 33);
            
            body.h  =  0.025;//mapf(sBus.channels[8], SBUS_chMin, SBUS_chMax, 0.005, 0.025);//Step height VRA
            //body.Ts =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, 0.5, 1);//Stride period S VRB 
      
            BodyTurn = mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -55, 55);
            body.MT[0] = -BodyTurn + MovementSpeed;
            body.MT[1] = BodyTurn + MovementSpeed;
            body.MT[2] = -BodyTurn + MovementSpeed;
            body.MT[3] = BodyTurn + MovementSpeed;
                              
          }  
          else if((sbus_swd==2)&&(SwitchingPattern==1))
          {
            body.MotorMode = 2;   
      
              ///body.H_R
              body.xt =  0 ;//Step length
            MovementSpeed =  -mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -111, 111);
            body.h  =  0;//Step height
            //body.Ts =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, 0.5, 1);//Stride period S VRB 
            BodyTurn = mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -111, 111);

            LegLength = mapf(sBus.channels[1], SBUS_chMin, SBUS_chMax, 0.03, -0.03); // leg height
            body.BodyRoll4Wheel =  mapf(sBus.channels[0], SBUS_chMin, SBUS_chMax, -0.011, 0.011); //Roll
      
                    
            body.MT[0] = -BodyTurn + MovementSpeed;
            body.MT[1] = BodyTurn + MovementSpeed;
            body.MT[2] = -BodyTurn + MovementSpeed;
            body.MT[3] = BodyTurn + MovementSpeed;
            
            
          }       
      }
      else if(sbus_swc==1)
      {
          if((sbus_swd==0)&&(SwitchingPattern==1))
          {
            body.MotorMode = 3;  

            ///body.H_R
            body.xt =  0 ;//Step length
            MovementSpeed =  -mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -111, 111);
            body.h  =  0;//Step height
            BodyTurn = mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -111, 111);

            LegLength = 0; // leg height

            body.MT[0] = -BodyTurn + MovementSpeed;
            body.MT[1] = BodyTurn + MovementSpeed;
            body.MT[2] = -BodyTurn + MovementSpeed;
            body.MT[3] = BodyTurn + MovementSpeed;

            body.BodyPitching4Wheel = body.BodyPitching4WheelT;
            body.BodyRoll4Wheel =  mapf(sBus.channels[0], SBUS_chMin, SBUS_chMax, -0.011, 0.011); //Roll
            

            
          }
          else if((sbus_swd==1)&&(SwitchingPattern==1))
          {
            body.MotorMode = 4; 
            ///body.H_R
            body.xt =  0 ;//Step length
            body.h  =  0;//Step height
            MovementSpeed =  -mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -111, 111);
            
            //body.Ts =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, 0.5, 1);//Stride period S VRB 
            BodyTurn = mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -111, 111);
            //
            LegLength = 0; // leg height
            body.BodyRoll4Wheel =  0; //Roll
                    
            body.MT[0] = -BodyTurn + MovementSpeed;
            body.MT[1] = BodyTurn + MovementSpeed;
            body.MT[2] = -BodyTurn + MovementSpeed;
            body.MT[3] = BodyTurn + MovementSpeed;          

                              
          }  
          else if((sbus_swd==2)&&(SwitchingPattern==1))
          {
            body.MotorMode = 5; 
            ///body.H_R
            body.xt =  0 ;//Step length
            body.h  =  0;//Step height
            MovementSpeed =  -mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -33, 33);
            
            //body.Ts =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, 0.5, 1);//Stride period S VRB 
            BodyTurn = mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -55, 55);
            //body.BodyPitching4Wheel = body.BodyPitching4WheelT;
            LegLength = 0; // leg height
            body.BodyRoll4Wheel =  0; //Roll
                    
            body.MT[0] = -BodyTurn + MovementSpeed;
            body.MT[1] = BodyTurn + MovementSpeed;
            body.MT[2] = -BodyTurn + MovementSpeed;
            body.MT[3] = BodyTurn + MovementSpeed;   

                
          }          
      }      
    }
    else if(MasterSlaveSelection == 0)//Slave
    {
      if(body.MotorMode == 0)
      {

      }
      else if(body.MotorMode == 1)
      {
      
      }
      else if(body.MotorMode == 2)
      {
      
      }   
    }
  }



  void PidParameter4wheel(void)
  {

      //Roll
      RollPid.P = 0.04;
      RollPid.I = 0.5;
      RollPid.D = 0.003;
      RollPid.limit = 4.4;//Integral limit·

      //Pitching
      AnglePid.P = 0.08;
      AnglePid.I = 1;
      AnglePid.D = 0.005;
      AnglePid.limit = 2.2;//Integral limit


      //Touch screen
      TouchXPid.P = 0.1;
      TouchXPid.I = 0;
      TouchXPid.D = 0.1;
      TouchXPid.limit = 0;//Integral limit
      
      TouchYPid.P = 0.15;
      TouchYPid.I = 0;
      TouchYPid.D = 0.11;
      TouchYPid.limit = 0;//Integral limit
  }



  void PIDcontroller_posture_4wheel(float dt)
  {
    if((int)PidParameterTuning==0)
      PidParameter4wheel();
    
    //Touch screen
    TouchX_Pid.Kp = TouchXPid.P/1000;
    TouchX_Pid.Ki = TouchXPid.I/1000;
    TouchX_Pid.Kd = TouchXPid.D/1000;
    TouchX_Pid.iLimit = TouchXPid.limit;//Integral limit
    
    TouchY_Pid.Kp = TouchYPid.P/1000;
    TouchY_Pid.Ki = TouchYPid.I/1000;
    TouchY_Pid.Kd = TouchYPid.D/1000;
    TouchY_Pid.iLimit = TouchYPid.limit;//Integral limit
    
    float touchXError = Touch.XPdatF/100;  
    float touchYError = Touch.YPdatF/100;  
    static float TouchX_kd = 0;
    static float TouchY_kd = 0;

    TouchX_Pid.compute(touchXError, dt); 
    TouchY_Pid.compute(touchYError, dt);   

    TouchX_Pid.deriv = constrain(TouchX_Pid.deriv, -77, 77);
    TouchX_Pid.outD = TouchX_kd * TouchX_Pid.deriv;  

    TouchY_Pid.deriv = constrain(TouchY_Pid.deriv, -77, 77);
    TouchY_Pid.outD = TouchY_kd * TouchY_Pid.deriv;  
    
    if((sbus_swd == 2)&&(sbus_swc == 1)&&(sbus_swa >= 1))//Top ball mode 
    {
      TouchX_Pid.output = TouchX_Pid.outP + TouchX_Pid.outI + TouchX_Pid.outD + sbus_vraf*0.002;

      TouchY_Pid.output = TouchY_Pid.outP + TouchY_Pid.outI + TouchY_Pid.outD + sbus_vrbf*0.002;

      TouchX_Pid.output = -TouchX_Pid.output;
      TouchY_Pid.output = -TouchY_Pid.output;
    } 
    else
    {
      TouchY_Pid_outputF = 0;    
      TouchX_Pid_outputF = 0;   
      TouchY_Pid.output  = 0;    
      TouchX_Pid.output  = 0;          
    } 

    if((int)enableDFilter==1)
    {
      //TouchY_Pid_outputF = biquadFilterApply(&FilterLPF[10], TouchY_Pid.output);    
      //TouchX_Pid_outputF = biquadFilterApply(&FilterLPF[11], TouchX_Pid.output);  
      TouchY_Pid_outputF = TouchY_Pid.output;    
      TouchX_Pid_outputF = TouchX_Pid.output; 
      
    }
    else
    {
      TouchY_Pid_outputF = TouchY_Pid.output;    
      TouchX_Pid_outputF = TouchX_Pid.output;   
    }

    if((Touch.state==1)&&(Touch.P_count<4))
    {
      Touch.P_count++;
      TouchX_Pid.integral = 0;
      //TouchX_Pid.output = 0;
      //TouchX_Pid.output = 0;
      TouchY_Pid.integral = 0;
      //TouchY_Pid_outputF = 0;  
    }
      
    if(Touch.P_count>=4)
    {
      if(TouchX_kd<TouchX_Pid.Kd)
      {
        TouchX_kd = TouchX_kd + (TouchX_Pid.Kd/22);
      }
      if(TouchY_kd<TouchY_Pid.Kd)
      {
        TouchY_kd = TouchY_kd + (TouchY_Pid.Kd/22);
      }  
      Touch.start = 2;
    } 

    if((Touch.state==0)&&(Touch.L_count<44))
      Touch.L_count++;
    else
      Touch.L_count = 0;
        
    if(Touch.L_count>=44)
    {
      TouchX_Pid.integral = 0;
      //TouchX_Pid.output = 0;
      //TouchX_Pid.output = 0;
      TouchY_Pid.integral = 0;
      //TouchY_Pid_outputF = 0;  
      Touch.P_count = 0;
      TouchX_kd = 0;   
      TouchY_kd = 0; 
      Touch.start = -2;    
    }  
    
    if((sbus_swd != 2)||(sbus_swc != 1)||(body.MotorMode != 5))//Non-top ball mode 
    {
      TouchX_Pid.integral = 0;
      TouchX_Pid.output = 0;
      //TouchX_Pid.output = 0;
      TouchY_Pid.integral = 0;
      TouchY_Pid.output = 0;
      TouchY_Pid_outputF = 0; 
      TouchX_kd = 0;   
      TouchY_kd = 0; 
    }

    //Roll Pitching
    Roll_Pid.Kp = RollPid.P/100;
    Roll_Pid.Ki = RollPid.I/100;
    Roll_Pid.Kd = RollPid.D/100;
    Roll_Pid.iLimit = RollPid.limit;//Integral limit

    Pitching_Pid.Kp = AnglePid.P/100;
    Pitching_Pid.Ki = AnglePid.I/100;
    Pitching_Pid.Kd = AnglePid.D/100;
    Pitching_Pid.iLimit = AnglePid.limit;//Integral limit

    float TargetBodyRoll = BodyRoll_f*1222; //Roll
    float TargetBodyPitching = body.BodyPitching4WheelTF*666; //Pitching
    if(body.MotorMode==5)
    {
      TargetBodyRoll = 0;
      TargetBodyPitching = 0;   
    }

      
    float RollError = (-pitch_ok) - (-TargetBodyRoll);// - (-TouchX_Pid_outputF); 
    float PitchingError = (-roll_ok) - (TargetBodyPitching);// - (TouchY_Pid_outputF);  
    
    if((sbus_swc == 1)&&(sbus_swa >= 1)&&(body.MotorMode==4))//Roll Pitching
    {
        Roll_Pid.compute(RollError, dt); 
        body.BodyPitching4Wheel = -Pitching_Pid.compute(PitchingError, dt); 
    }
    else
    {
        Roll_Pid.output = 0;
        Roll_Pid.integral = 0;

        Pitching_Pid.output = 0;
        Pitching_Pid.integral = 0;      
    }


  
  }
