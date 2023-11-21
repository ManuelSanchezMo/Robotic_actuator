

#include <SPI.h>
#include <mcp2515.h>      //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)

#include <FiniteStateMachine.h>
#include <SimpleFOC.h>
#include <can_parser.h>

#define   INH_A PB_3
#define   INH_B PB_4
#define   INH_C PB_5
#define   EN_GATE PA_2
#define   M_PWM PB_7 
#define   M_OC PC_15
#define   OC_ADJ PC_14

struct can_frame canMsg;
//Can and sensor Cs SPI pin config
int Cs_pin_can= PA_7;
 MCP2515 mcp2515(Cs_pin_can);
int Cs_pin_encoder = PB_0;
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, Cs_pin_encoder);
//motor instance
int nPolePairs=21;
BLDCMotor motor = BLDCMotor(nPolePairs);
int8_t mec_out_dcl= 6, elec_out_dcl = 8; 
// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);
//Default motorparam
float P_vel = 10.0, I_vel = 0.02, D_vel = 0.0, V_aling = 2.0, Vel_lim = 20.0, Volt_lim = 20.0;
float P_controller = 10.0, I_controller = 0.1, D_controller = 0.0, calibrate = 0.0, zero_angle_elec = 0.0;
float cmd_angle = 0;
//Finite state machine states
State INITIALIZATION = State(initMotorFSM); 
State PREOPERATIONAL = State(preopMotorFSM);
State OPERATIONAL = State(runMotorFSM);
State STOPPED = State(stopMotorFSM);
FSM MotorFSM = FSM(INITIALIZATION);
//can frames
int motor_cobid = 10;
int motor_config_1_frame = 1;
int motor_config_2_frame = 2;
int motor_config_3_frame = 3;
int motor_out_elec_frame = 4;
int motor_out_mec_frame = 5;
int transition_frame = 6;
int motor_sp = 7;

void setup() {
//start serial, CAN and FSM
  Serial.begin(9600);
  Serial.println("setup!");
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();
  MotorFSM.update();

}
void loop() {
 
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){ 

    int motor_frame = int(canMsg.can_id) - motor_cobid;
    switch (motor_frame) //switch to run callback for the target function
    {
      case 1:
        config1_fnc(canMsg);
        break;
      case 2:
        config2_fnc(canMsg);
        break;
      case 3:
        config3_fnc(canMsg);
        break;
      case 6:
        Transition_FSM(int(canMsg.data[0]));
        break;
      case 7:
        set_sp_fnc(canMsg);
        break;
      default:
        break;
      }
  }
  if(MotorFSM.isInState(OPERATIONAL))  MotorFSM.update();  //run the motor callback (update foc) when the motor is in running mode
}

void config1_fnc(can_frame canConfigMsg){
  //Callback for config_1 
  Serial.println("Config 1 ");
  CONFIG_1_t* config_1;
  Unpack_CONFIG_1_can_parser(config_1, canMsg.data, canMsg.can_dlc );
  P_controller = config_1->P_control_phys;
  I_controller = config_1->I_control_phys;
  D_controller = config_1->D_control_phys;
  return;
}
void config2_fnc(can_frame canConfigMsg){
  //Callback for config_2
  Serial.println("Config 2 ");
  CONFIG_2_t* config_2;
  Unpack_CONFIG_2_can_parser(config_2, canMsg.data, canMsg.can_dlc );
  P_vel = config_2->P_vel_phys;
  I_vel = config_2->I_vel_phys;
  D_vel = config_2->D_vel_phys;
  Vel_lim = config_2->Vel_lim_phys;
  return;
}

void config3_fnc(can_frame canConfigMsg){
    //Callback for config_2
      Serial.println("Config 3 ");
      CONFIG_3_t* config_3;
      Unpack_CONFIG_3_can_parser(config_3, canMsg.data, canMsg.can_dlc );
      Volt_lim = config_3->Volt_lim_phys;
      V_aling = config_3->V_aling_phys;
      calibrate = config_3->Calibrate;
      zero_angle_elec = config_3->Zero_angle_elec_phys;
  return;
}
void set_sp_fnc(can_frame canConfigMsg){
  //Callback to set the set point 
  MOTOR_ANGLE_SP_t* angel_sp;
  Unpack_MOTOR_ANGLE_SP_can_parser(angel_sp, canMsg.data, canMsg.can_dlc );
  cmd_angle = angel_sp->angle_sp_phys;
  return;
}

void initMotorFSM(){
  //init motor. Always gets called on motor boot-up
  pinMode(PA_3, OUTPUT); //Pin CS - CAN
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  sensor.init();
  // initialise magnetic sensor hardware
  // driver config
  motor.linkSensor(&sensor);
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);
  // power supply voltage [V]
  driver.voltage_power_supply = 30;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;
  Serial.println("in init!");
}

void preopMotorFSM(){
  //In preop motor, motor params are refreshed if where changed through config# message
  Serial.println("in preopMotorFSM!");
  // angle P controller
  motor.PID_velocity.P = P_vel;
  motor.PID_velocity.I = I_vel;
  motor.PID_velocity.D = D_vel;
  //maximal voltage to be set to the motor
  motor.voltage_limit = Volt_lim;
  motor.P_angle.P = P_controller; 
  motor.P_angle.I = I_controller;  // usually only P controller is enough 
  motor.P_angle.D = D_controller;  // usually only P controller is enough   
  motor.velocity_limit = Vel_lim; // maximal velocity of the position control
  motor.voltage_sensor_align = V_aling; // maximal Volts during alingment
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE |_MON_VOLT_Q|_MON_VOLT_D|_MON_CURR_Q|_MON_CURR_D; //vars to monitor
  motor.useMonitoring(Serial);
  motor.init();
  
  if(calibrate == 1.0){ //if not calibrate, use params sent
    motor.electrical_angle =  zero_angle_elec;
    motor.sensor_direction = Direction::CCW;
    motor.initFOC();
    }
  else motor.initFOC();
  cmd_angle = motor.shaft_angle; //Current angle as command so the motor doesnt move on start
  motor.enable();
  Serial.println("Motor ready.");
}

void runMotorFSM(){
  //On Run loop  FOC and send out electric and mechanic params 
  motor.move(cmd_angle);
  motor.loopFOC();
  MOTOR_OUT_MEC_t* mec_out;
  can_frame canMsg_mec_out;
  mec_out->shaft_angle_phys = motor.shaft_angle; 
  mec_out->shaft_angle_sp_phys = motor.shaft_angle_sp; 
  mec_out->shaft_velocity_phys = motor.shaft_velocity; 
  __CoderDbcCanFrame_t__* cframe;
  Pack_MOTOR_OUT_MEC_can_parser(mec_out, cframe);
  canMsg_mec_out.can_id = motor_out_mec_frame + motor_cobid;
  canMsg_mec_out.can_dlc = mec_out_dcl;
  uint8_t i; for (i = 0; (i < mec_out_dcl) ;canMsg_mec_out.data[i] = cframe->Data[i]);
  mcp2515.sendMessage(MCP2515::TXB1, &canMsg_mec_out);

  MOTOR_OUT_ELEC_t* elec_out;
  can_frame canMsg_elec_out;
  elec_out->Ua_phys = motor.Ua;
  elec_out->Ub_phys = motor.Ub; 
  elec_out->current_phys = motor.current.q; 
  elec_out->electrical_angle_phys = motor.target; 
  __CoderDbcCanFrame_t__* cframe_mec;
  Pack_MOTOR_OUT_ELEC_can_parser(elec_out, cframe);
  canMsg_elec_out.can_id = motor_out_elec_frame + motor_cobid;
  canMsg_elec_out.can_dlc = elec_out_dcl;
  for (i = 0; (i < elec_out_dcl) ;canMsg_elec_out.data[i] = cframe_mec->Data[i]);
  mcp2515.sendMessage(MCP2515::TXB1, &canMsg_elec_out);
  Serial.println("in runMotorFSM!");
}

void stopMotorFSM(){
  Serial.println("in stopMotorFSM!");
  motor.disable();
}

void Transition_FSM(int transition)
{
    //Transition table for the FSM
    switch (transition)
    {
      case 1:
          if ( MotorFSM.isInState(INITIALIZATION)){
              MotorFSM.transitionTo(PREOPERATIONAL);
          }
          break;
      case 2:
          if ( MotorFSM.isInState(PREOPERATIONAL)){
              MotorFSM.transitionTo(INITIALIZATION);
          }
          break;
      case 3:
          if ( MotorFSM.isInState(PREOPERATIONAL) ){
            MotorFSM.transitionTo(OPERATIONAL);
          }
          break;
        case 4:
          if (  MotorFSM.isInState(OPERATIONAL) ){
            MotorFSM.transitionTo(PREOPERATIONAL);
          }
          break;
      case 5:
          if ( MotorFSM.isInState(OPERATIONAL)){
            MotorFSM.transitionTo(STOPPED);
          }  
          break;
      case 6:
          if ( MotorFSM.isInState(STOPPED)){
              MotorFSM.transitionTo(PREOPERATIONAL);
          } 
          break;
      default:
          break;
    }
    MotorFSM.update();
}