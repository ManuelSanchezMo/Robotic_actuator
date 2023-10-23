#include <FiniteStateMachine.h>
#include <mcp2515.h> 
#include <SimpleFOC.h>
#define BIT(n,i) (n>>i&1)

//define states and working functions
//motor states
State INITIALIZATION = State(initMotor); 
State PREOPERATIONAL = State(preopMotor);
State OPERATIONAL = State(runMotor);
State STOPPED = State(stopMotor);

#define   INH_A PB_3
#define   INH_B PB_4
#define   INH_C PB_5
#define   EN_GATE PA_2
#define   M_PWM PB_7 
#define   M_OC PC_15
#define   OC_ADJ PC_14

int anglesp = 4;
int CobidToinit= 0x001;
int CobidTopreop = 0x002;
int CobidTooper = 0x003;
int CobidTostop = 0x004;
int Cobidanglesp = 005;
int Cobidmotordata = 0x006;

int angleSp = 127;
FSM Motor = FSM(INITIALIZATION);
bool initDone = false;
bool calibrationDone = false;


int Cs_pin_can=PA_3;
MCP2515 mcp2515(Cs_pin_can);
struct can_frame canMsg;
const byte  Cs_pin_sensor=PB_8;

uint8_t varA = 0x01; //1 Dec
uint8_t varB = 0x71; //113 Dec
uint16_t myVar = 0;
float zero_angle = 0;
float zero_elec_angle = 0;

// motor instance
int nPolePairs = 21;
BLDCMotor motor = BLDCMotor(nPolePairs);

// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);



void setup() {
 
  Serial.begin(115200);
  Serial.println("AS5048A - Codificador de posición magnetico");
  SPI.begin();

  pinMode(PA_3, OUTPUT); //Pin CS - salida
  pinMode(PB_8, OUTPUT); //Pin CS - salida

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); //Sets CAN at speed 500KBPS and Clock 8MHz
  mcp2515.setNormalMode();
MagneticSensorSPIConfig_s AS5048_SPI_custom = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 100000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15
};
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI_custom, Cs_pin_sensor);
  // M_OC  - enable over-current protection
  pinMode(M_OC,OUTPUT);
  digitalWrite(M_OC,LOW);

  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM,OUTPUT);
  digitalWrite(M_PWM,HIGH);

  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ,OUTPUT);
  digitalWrite(OC_ADJ,HIGH);
  // initialise magnetic sensor hardware
  sensor.init();

  motor.linkSensor(&sensor);
  delay(10000); //espera 2 seg

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 25;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 10.0f;
  motor.PID_velocity.I = 0.02f;
  motor.PID_velocity.D = 0.0f;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 15;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.1f;
  motor.LPF_angle.Tf = 0.1f;
  // angle P controller
  motor.P_angle.P = 10.0; 
  motor.P_angle.I = 0.1;  // usually only P controller is enough 
  motor.P_angle.D = 0.0;  // usually only P controller is enough   // maximal velocity of the position control
  motor.velocity_limit = 100;
  motor.voltage_sensor_align = 7;
  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE |_MON_VOLT_Q|_MON_VOLT_D|_MON_CURR_Q|_MON_CURR_D;
  //motor.useMonitoring(Serial);
  motor.init();

  motor.initFOC();
  //initiate state machines
  //Serial.println("Motor ready."); 

}

void loop() {
   //motor.loopFOC();
  //motor.move(1);
 // print the string when a newline arrives:
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){ 
     if(canMsg.can_id==Cobidanglesp){
       varA = canMsg.data[0]; //1 Dec
       varB = canMsg.data[1]; //113 Dec

      myVar = canMsg.data[0]; //1 Dec
      myVar <<= 8; //Left shit 8 bits, so from 1 Dec it's not 256 Dec. From 0x01 to 0x100;
      myVar = myVar | varB; //OR operation, basically 0x100 merged with 0x71, which will result in 0x171

      /*Serial.println(myVar,DEC);
      Serial.println(myVar,HEX);
      Serial.println(myVar,BIN);
      
      Serial.println(canMsg.data[1],DEC);
      Serial.println(canMsg.data[1],HEX);
      Serial.println(canMsg.data[1],BIN);

      Serial.println(("estat teur."));
      float angle_sp = 20.0/32768.0*(float(myVar)-32768.0);
                 
      Serial.println(("angle."));
      Serial.print(angle_sp);*/


     }
    }
     //Serial.println(canMsg.can_id);
     //Serial.println(AS5048_SPI.clock_speed);


    //set motor state depending on serial string
    if(canMsg.can_id==CobidTopreop){
      if(Motor.isInState(INITIALIZATION)|| Motor.isInState(OPERATIONAL)||Motor.isInState(STOPPED)){
        //Serial.println("to preop!");
        Motor.transitionTo(PREOPERATIONAL);
      }
    }else if(canMsg.can_id==CobidTooper){
      if(Motor.isInState(STOPPED) || Motor.isInState(PREOPERATIONAL)) {
        Serial.println("tooperational!");
        if (canMsg.data[0] == 1){
            float zero_angle = 20.0/32768.0*(float(myVar)-32768.0);         
            //Serial.println(("zero angle."));
            //Serial.print(zero_angle);
        }
        //Serial.println(canMsg.data[0]);
        //Serial.println(canMsg.data[1]);
        Motor.transitionTo(OPERATIONAL);
      }
    }else if(canMsg.can_id==CobidToinit){
      if(Motor.isInState(OPERATIONAL) || Motor.isInState(PREOPERATIONAL) ||Motor.isInState(STOPPED)){
        //Serial.println("toninit!");
        Motor.transitionTo(INITIALIZATION);
      }
    }else if(canMsg.can_id==CobidTostop){
      if( Motor.isInState(OPERATIONAL) || Motor.isInState(PREOPERATIONAL)){
        //Serial.println("tostop!");
        Motor.transitionTo(STOPPED);
      }
    }else{;}
    // clear the string:
   

    // Run state machine
    Motor.update();
  
 
}


void initMotor(){
  //Serial.println("in init!");

}

void preopMotor(){
  // link the motor to the sensor
  if (!calibrationDone){
     if (canMsg.data[0] == 1){
        //Serial.println("calibrating");
        //motor.initFOC();
        }
      else {
        //Serial.println("skiip");
        //motor.initFOC(2.1, Direction::CCW);
       }
       calibrationDone = true;
    }
 }



void runMotor(){
  Serial.println("runn foc!");
  //Serial.println((angleSp-127)*6/127);

  motor.loopFOC();
  motor.move(0);
    //Serial.println("foc!");

  canMsg.can_id  = Cobidmotordata;           //CAN id as 0x036
  canMsg.can_dlc = 8;               //CAN data length as 8
  canMsg.data[0] = -20;               //Update humidity value in [0]
  canMsg.data[1] = 1;               //Update temperature value in [1]
  canMsg.data[2] = 2;            //Rest all with 0
  canMsg.data[3] = 3;
  canMsg.data[4] = 4;
  canMsg.data[5] = 5;
  canMsg.data[6] = 6;
  canMsg.data[7] = 7;
 
  mcp2515.sendMessage(&canMsg);     //Sends the CAN message
 motor.monitor();
 // Serial.println(int(motor.voltage.q*1000));
 //Serial.println((motor.voltage.q));
 //Serial.println((angleSp-125)*0.048);


}



void stopMotor(){
        //Serial.println("stopped!");
}
