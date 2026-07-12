//#include "pinout.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"
//#include <SPI.h>         // needed for Arduino versions later than 0018
//#include <Ethernet2.h>
//#include <EthernetUdp2.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
//byte mac[] = {
//  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
//};
//IPAddress ip(192, 168, 0, 100);

//unsigned int localPort = 60000; 
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE];
//EthernetUDP Udp;
//char  ReplyBuffer[] = "acknowledged"; 
//#define LED_PIN 38
//내가 만든 보드에서는 X가 Z에 해당하고 Z가 X에 해당한다
#define Z_STEP_PIN 4
#define Y_STEP_PIN 3
#define X_STEP_PIN 2
#define Z_DIR_PIN 7
#define Y_DIR_PIN 6
#define X_DIR_PIN 5
#define Z_ENABLE_PIN 8
#define Y_ENABLE_PIN 8
#define X_ENABLE_PIN 8
#define PUMP_PIN A0 //밸브랑 펌프랑 같은신호
//#define VALVE_PIN A1
#define LAMP_G A3
#define LAMP_Y A4
#define LAMP_R A5
//#define LAMP_B A7 //사용불가
#define EMG_STOP 9
//추가부분
#define joy1_sw 13
#define joy1_x A0
#define joy1_y A3
#define joy2_sw 12
#define joy2_x A2
#define joy2_y A1

#define SWA A5 //기본자세
#define SWB A4 //호밍

//#include <Stepper.h>
#include <Servo.h>

Servo pump; 
Servo valve;
bool gripper_state = false;
bool stepper_enabled = false;
unsigned long main_t = 0;
unsigned long sub_t = 0;

//Stepper stepper(2400, STEPPER_GRIPPER_PIN_0, STEPPER_GRIPPER_PIN_1, STEPPER_GRIPPER_PIN_2, STEPPER_GRIPPER_PIN_3);
RampsStepper stepperRotate(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
//RampsStepper stepperExtruder(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN);
//FanControl fan(FAN_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(20);
Command command;
bool old_queue_state = true;
float h_offset = 0;
float l_offset = 0;

bool old_emg_stop = HIGH;

float target_x = 0;
float target_y = 19.5;
float target_z = 134;

void setup() {
  Serial.begin(9600);
  pump.attach(10);
  valve.attach(11);
  pinMode(joy1_sw,INPUT_PULLUP); //로봇 활성할거냐 말거냐?
  pinMode(joy2_sw,INPUT_PULLUP); //그리퍼 토글제어

  pinMode(SWA,INPUT_PULLUP);
  pinMode(SWB,INPUT_PULLUP);
    
  //pinMode(A5,OUTPUT);
  //pinMode(LAMP_G,OUTPUT);
  //pinMode(LAMP_Y,OUTPUT);
  //pinMode(LAMP_R,OUTPUT);
  //pinMode(LAMP_B,OUTPUT);
  //pinMode(EMG_STOP,INPUT_PULLUP);
  
  //reduction of steppers..
  stepperHigher.setReductionRatio(32.0 / 9.0, 200 * 16);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
  stepperLower.setReductionRatio( 32.0 / 9.0, 200 * 16);
  stepperRotate.setReductionRatio(32.0 / 9.0, 200 * 16);
  //stepperExtruder.setReductionRatio(32.0 / 9.0, 200 * 16);
  
  //start positions..
  stepperHigher.setPositionRad(0);  //90°
  stepperLower.setPositionRad(0);          // 0°
  stepperRotate.setPositionRad(0);         // 0°
  //stepperExtruder.setPositionRad(0);
  
  //enable and init..
  setStepperEnable(false);
  //interpolator.setC  yurrentPos(0,19.5,134,0);
  interpolator.setInterpolation(0,19.5,134,0, 0,19.5,134,0);
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  //geometry.set(0,19.5,134);
  //Serial.println("start");

  h_offset = geometry.getHighRad();
  l_offset = geometry.getLowRad();
  //Serial.print(h_offset);
  //Serial.print(", ");
  //Serial.print(l_offset);
  //Serial.println();
}

void setStepperEnable(bool enable) {
  stepperRotate.enable(enable);
  stepperLower.enable(enable);
  stepperHigher.enable(enable); 
  //stepperExtruder.enable(enable); 
  //fan.enable(enable);
  stepper_enabled = enable;
}

void loop () {
  if(digitalRead(joy1_sw) == LOW){
    if(stepper_enabled){
      setStepperEnable(false);
    }else{
      setStepperEnable(true);
    }
    delay(300);
  }
  if(digitalRead(joy2_sw) == LOW){
    if(gripper_state){
      pump.write(0);
      valve.write(0);
      gripper_state = false;
    }else{
      pump.write(180);
      valve.write(180);
      gripper_state = true;
    }
    delay(300);
  }
  
  if(digitalRead(SWA) == LOW){
    //초기자세
    target_x = 0;
    target_y = 89.5;
    target_z = 134;
    String data = "G1 X0 Y89.5 Z134 F30";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    delay(300);
  }
  if(digitalRead(SWB) == LOW){
    //호밍
    target_x = 0;
    target_y = 19.5;
    target_z = 134;
    String data = "G1 X0 Y19.5 Z134 F30";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    delay(300);
  }

  
  /*
  if(Serial.available()){
    char c = Serial.read();
    if(c == '0'){
      pump.write(0);
      pump.detach();
      valve.write(0);
      valve.detach();
      gripper_state = false;
    }else if(c == '1'){
      pump.attach(PUMP_PIN);
      pump.write(180);
      valve.attach(VALVE_PIN);
      valve.write(180);
      gripper_state = true;
    }
  }
  */
  /*
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    String gcode = packetBuffer;
    command.processMessage(gcode);
    queue.push(command.getCmd());
  }
  if(digitalRead(btn1)==LOW){
    pump.attach(PUMP_PIN);
    pump.write(180);
    valve.attach(VALVE_PIN);
    valve.write(180);
    gripper_state = true;
  }
  if(digitalRead(btn2)==LOW){
    pump.write(0);
    pump.detach();
    valve.write(0);
    valve.detach();
    gripper_state = false;
  }
  */
  if(old_queue_state == false && queue.isEmpty() == true){
    //Serial.println("큐 비워짐");
    Serial.println("OK");
  }

  if(millis() - sub_t > 10){
    sub_t = millis();
    
  }
  
  old_queue_state = queue.isEmpty();
  if(millis() - main_t > 80){
    main_t = millis();
    /*
    Serial.print(stepperRotate.getPosition()); //기어비 반영
    Serial.print(", ");
    Serial.print(stepperLower.getPosition()); //기어비 반영
    Serial.print(", ");
    Serial.print(stepperHigher.getPosition()); //기어비 반영
    */
    
    Serial.print(geometry.getXmm());
    Serial.print(", ");
    Serial.print(geometry.getYmm());
    Serial.print(", ");
    Serial.print(geometry.getZmm());
    Serial.print(", ");
    Serial.print(geometry.getRotRad()*(180/PI)); //기어비 반영
    Serial.print(", ");
    Serial.print((geometry.getLowRad()-l_offset)*(180/PI)); //기어비 반영
    Serial.print(", ");
    Serial.print((geometry.getHighRad()-h_offset)*(180/PI)); //기어비 반영
    Serial.print(", ");
    Serial.print(interpolator.isFinished());
    Serial.print(", ");
    Serial.print(gripper_state);
    Serial.print(", ");
    Serial.print(digitalRead(EMG_STOP));
    Serial.print(", ");
    Serial.print(stepper_enabled);
    Serial.println();

    //여기서 작성하기~
    int joy1_x_value = analogRead(joy1_x);
    int joy1_y_value = analogRead(joy1_y);

    int joy2_x_value = analogRead(joy2_x);
    int joy2_y_value = analogRead(joy2_y);
    
    if(joy1_x_value > 800){
      target_x += 2;
      String data = "G1 X"+String(target_x)
      +" Y"+String(target_y)
      +" Z"+String(target_z)
      + " F20";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    }else if(joy1_x_value < 200){
      target_x -= 2;
      String data = "G1 X"+String(target_x)
      +" Y"+String(target_y)
      +" Z"+String(target_z)
      + " F20";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    }
    
    if(joy1_y_value > 800){
      target_y += 2;
      String data = "G1 X"+String(target_x)
      +" Y"+String(target_y)
      +" Z"+String(target_z)
      + " F20";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    }else if(joy1_y_value < 200){
      target_y -= 2;
      String data = "G1 X"+String(target_x)
      +" Y"+String(target_y)
      +" Z"+String(target_z)
      + " F20";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    }

    if(joy2_x_value > 800){
      target_z += 2;
      String data = "G1 X"+String(target_x)
      +" Y"+String(target_y)
      +" Z"+String(target_z)
      + " F20";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    }else if(joy2_x_value < 200){
      target_z -= 2;
      String data = "G1 X"+String(target_x)
      +" Y"+String(target_y)
      +" Z"+String(target_z)
      + " F20";
      if (!queue.isFull() && command.processMessage(data)) {       
        queue.push(command.getCmd());
      }
    }
    














    
  }

  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad (geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  //stepperExtruder.stepToPositionRad(interpolator.getEPosmm());
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update(); 
  //fan.update();
  
  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
      //printOk();
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
  }
}

void cmdMove(Cmd (&cmd)) {
  target_x = cmd.valueX;
  target_y = cmd.valueY;
  target_z = cmd.valueZ;
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
}
void cmdDwell(Cmd (&cmd)) {
  delay(int(cmd.valueT * 1000));
}
void cmdGripperOn(Cmd (&cmd)) {
  //digitalWrite(A5,HIGH);
  //pump.attach(PUMP_PIN);
  pump.write(180);
  //valve.attach(VALVE_PIN);
  valve.write(180);
  gripper_state = true;
  /*
  stepper.setSpeed(5);
  stepper.step(int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
  */
}
void cmdGripperOff(Cmd (&cmd)) {
  //digitalWrite(A5,LOW);
  pump.write(0);
  //pump.detach();
  valve.write(0);
  //valve.detach();
  gripper_state = false;
  /*
  stepper.setSpeed(5);
  stepper.step(-int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
  */
}
void cmdStepperOn() {
  setStepperEnable(true);
}
void cmdStepperOff() {
  setStepperEnable(false);
} 
void cmdFanOn() {
  //fan.enable(true);
}
void cmdFanOff() {
  //fan.enable(false);
}

void handleAsErr(Cmd (&cmd)) {
  printComment("Unknown Cmd " + String(cmd.id) + String(cmd.num) + " (queued)"); 
  printFault();
}

void executeCommand(Cmd cmd) {
  if (cmd.id == -1) {
    String msg = "parsing Error";
    printComment(msg);
    handleAsErr(cmd);
    return;
  }
  
  if (cmd.valueX == NAN) {
    cmd.valueX = interpolator.getXPosmm();
  }
  if (cmd.valueY == NAN) {
    cmd.valueY = interpolator.getYPosmm();
  }
  if (cmd.valueZ == NAN) {
    cmd.valueZ = interpolator.getZPosmm();
  }
  if (cmd.valueE == NAN) {
    cmd.valueE = interpolator.getEPosmm();
  }
  
   //decide what to do
  if (cmd.id == 'G') {
    switch (cmd.num) {
      case 0: cmdMove(cmd); break;
      case 1: cmdMove(cmd); break;
      case 4: cmdDwell(cmd); break;
      //case 21: break; //set to mm
      //case 90: cmdToAbsolute(); break;
      //case 91: cmdToRelative(); break;
      //case 92: cmdSetPosition(cmd); break;
      default: handleAsErr(cmd); 
    }
  } else if (cmd.id == 'M') {
    switch (cmd.num) {
      //case 0: cmdEmergencyStop(); break;
      case 3: cmdGripperOn(cmd); break;
      case 5: cmdGripperOff(cmd); break;
      case 17: cmdStepperOn(); break;
      case 18: cmdStepperOff(); break;
      case 106: cmdFanOn(); break;
      case 107: cmdFanOff(); break;
      default: handleAsErr(cmd); 
    }
  } else {
    handleAsErr(cmd); 
  }
}
