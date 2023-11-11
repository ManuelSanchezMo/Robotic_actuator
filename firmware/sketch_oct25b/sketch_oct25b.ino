/*
  Blink onboard LED at 0.1 second interval
*/
#include <SPI.h>
#include <mcp2515.h>      //Library for using CAN Communication (https://github.com/autowp/arduino-mcp2515/)
#include <FiniteStateMachine.h>
#include <SimpleFOC.h>
//#include <can_decode.h>
#define   INH_A PB_3
#define   INH_B PB_4
#define   INH_C PB_5
#define   EN_GATE PA_2
#define   M_PWM PB_7 
#define   M_OC PC_15
#define   OC_ADJ PC_14

struct can_frame canMsg;

int Cs_pin_can= PA_3;
 MCP2515 mcp2515(PA_3);

int Cs_pin_encoder = PA_5;
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, Cs_pin_encoder);

int nPolePairs=21;
BLDCMotor motor = BLDCMotor(nPolePairs);

// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);
float v_P = 10.0, v_I = 0.02, v_D = 0.0, volt_aling = 2.0, v_limit = 20.0, volt_supply=10.0;
float P_controller = 10.0, I_controller = 0.1, D_controller = 0.0;
float cmd_angle = 0;
bool calibration = true;

State INITIALIZATION = State(initMotorFSM); 
State PREOPERATIONAL = State(preopMotorFSM);
State OPERATIONAL = State(runMotorFSM);
State STOPPED = State(stopMotorFSM);
FSM MotorFSM = FSM(INITIALIZATION);
enum FSMTransitions {InitToPreop, PreopToOp, PreopToStop, PreopToinit, 
                     OptoPreop, OptoStop, OptoInit, StoptopOp, StoptoPreop, StoptoInit };
                     
FSMTransitions asked_transition;
int Cobid = 0x008;
int FSN_transition_frame = 0x007;
int OpToPreop = 0x003;
int CobidTostop = 0x004;
int Cobidanglesp = 005;
int Cobidmotordata = 0x006;
int Cobidcommangle = 0x008;
int Cobidconfig1 = 0x008;
int Cobidconfig2 = 0x010;

int max_val_int = 65535;
int data_read;
void Transition_FSM(int transition);
void setup() {
  // initialize digital pin PB2 as an output.

  Serial.begin(9600);
  Serial.println("setup!");

}
void loop() {

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){ 
            Serial.println(canMsg.can_id);
             int x = (uint16_t)((canMsg.data[0] << 8) + canMsg.data[1]);
            float sol = int_to_float(x, 32.5);
            Serial.println(canMsg.data[0]);
            Serial.println(canMsg.data[1]);
            Serial.println(x);
            Serial.print("frame ");
            Serial.println(canMsg.can_id);

            Serial.print("dol ");
        

            Serial.println(sol);
            asked_transition = FSMTransitions(int(canMsg.can_id));
            Serial.print("Ask trans ");
            Serial.println(int(canMsg.can_id)-Cobid);
            Serial.print("here ");
            if (((int(canMsg.can_id)-Cobid) > 0) && (int(canMsg.can_id)-Cobid) <= 10) Transition_FSM(int(canMsg.can_id)-Cobid);
            
            if (canMsg.can_id == 0x008) cmd_angle = canMsg.data[0];
            
            if (canMsg.can_id == FSN_transition_frame){

              asked_transition = FSMTransitions(int(canMsg.can_id));
              Serial.print("Ask trans ");
              Serial.println(int(canMsg.data[0]));

   
                    Serial.print("here ");
                   Transition_FSM(asked_transition);
            }
            if (canMsg.can_id == Cobidconfig1){
                motorConfig1(canMsg);
              }
            if (canMsg.can_id == Cobidconfig2){
                motorConfig2(canMsg);
              }
    }
     if(MotorFSM.isInState(OPERATIONAL))  MotorFSM.update();
    
}

void motorConfig1(can_frame frame){
  /*
   v_P = 10.0; 
   v_I = 0.02;
   v_D = 0.0;
   volt_aling = 2.0;
  */
  }

void motorConfig2(can_frame frame){
  /*
      v_limit = 20.0;
      P_controller = 10.0;
      I_controller = 0.1;
      D_controller = 0.0;*/
  }
void motorConfig3(can_frame frame){
  /*
     calibration = true;
      electrical_angle = 10.0;
      direction = 1;
      volt_supply=10;*/
  }  
void initMotorFSM(){
  pinMode(LED_BUILTIN, OUTPUT); // LED connect to pin PC13
  pinMode(PA_3, OUTPUT); //Pin CS - CAN

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();

  //Serial.println("in init!");
  data_read = canMsg.data[0];
  byte gear1 = canMsg.data[0];
  byte gear2 = canMsg.data[1];
  int gearCombo = (gear1<<8) | gear2;
  //Serial.print("gear ");
  //Serial.println(gearCombo);
  // M_OC  - enable over-current protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);

  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);
  sensor.init();

  motor.linkSensor(&sensor);
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);
  // initialise magnetic sensor hardware
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = volt_supply;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value

  int res = float_to_int(37.54123,80);
  Serial.print("ires ");
  Serial.println(res);

  byte low =  res & 0xFF;  // Take just the lowest 8 bits.
  byte high = res >> 8;  // Shift the integer right 8 bits.
  //Serial.println(low);
  //Serial.println(high);
  canMsg.data[0] = low;
  canMsg.data[0] = high;
  struct can_frame canMsg_out;
  canMsg_out.can_id = 0x15;
  canMsg_out.can_dlc = 2;
  canMsg_out.data[1] = low;
  canMsg_out.data[0] = high;
  mcp2515.sendMessage(MCP2515::TXB1, &canMsg_out);
  }

void preopMotorFSM(){
  Serial.println("in preopMotorFSM!");
  motor.PID_velocity.P = v_P;
  motor.PID_velocity.I = v_I;
  motor.PID_velocity.D = v_D;
  // maximal voltage to be set to the motor
  motor.voltage_limit = volt_supply;


  // angle P controller
  motor.P_angle.P = P_controller; 
  motor.P_angle.I = I_controller;  // usually only P controller is enough 
  motor.P_angle.D = D_controller;  // usually only P controller is enough   // maximal velocity of the position control
  motor.velocity_limit = v_limit;
  motor.voltage_sensor_align = volt_aling;
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE |_MON_VOLT_Q|_MON_VOLT_D|_MON_CURR_Q|_MON_CURR_D;
  motor.useMonitoring(Serial);
  motor.init();

  if(calibration){
    motor.electrical_angle =  2.1;
    motor.sensor_direction = Direction::CCW;
    motor.initFOC();
  }
  else motor.initFOC();
  cmd_angle = motor.shaft_angle; //Current angle as command so the motor doesnt move on start
  motor.enable();
  Serial.println("Motor ready."); 
 
 
}

void runMotorFSM(){
  Serial.println("in runMotorFSM!");
  
  motor.move(cmd_angle);
  motor.loopFOC();
  motor.Ua;
  motor.Ub;
  motor.current;
  motor.electrical_angle;
  motor.shaft_angle;
  motor.shaft_angle_sp;
  motor.shaft_velocity;
}

void stopMotorFSM(){
  Serial.println("in stopMotorFSM!");
  motor.disable();
}

float  int_to_float(int data, float range_float){
  return float(range_float/max_val_int*data - range_float/2);

}

int  float_to_int(float data, float range_float){
  return int(max_val_int/range_float*data + max_val_int/2);

}



void Transition_FSM(int transition)
{
    Serial.println('asked trans');
    Serial.println(transition);
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
          if (  MotorFSM.isInState(PREOPERATIONAL) ){
            MotorFSM.transitionTo(INITIALIZATION);
          }
          break;

      case 5:
          if ( MotorFSM.isInState(OPERATIONAL)){
            MotorFSM.transitionTo(PREOPERATIONAL);
          }  
          break;
      case 6:
          if ( MotorFSM.isInState(OPERATIONAL)){
              MotorFSM.transitionTo(STOPPED);
          } 
          break;

      default:
          break;
    }
    MotorFSM.update();
}
