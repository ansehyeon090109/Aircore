#include "pinout.h"
#include "robotGeometry.h"
#include "interpolation.h"
#include "fanControl.h"
#include "RampsStepper.h"
#include "queue.h"
#include "command.h"

#include <Stepper.h>
bool gripper_state = false;
bool stepper_enabled = false;
unsigned long main_t = 0;
unsigned long sub_t = 0;
bool old_queue_state = true;
float h_offset = 0;
float l_offset = 0;
float target_x = 0;
float target_y = 19.5;
float target_z = 134;
Stepper stepper(2400, STEPPER_GRIPPER_PIN_0, STEPPER_GRIPPER_PIN_1, STEPPER_GRIPPER_PIN_2, STEPPER_GRIPPER_PIN_3);
RampsStepper stepperRotate(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
RampsStepper stepperLower(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
RampsStepper stepperHigher(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
RampsStepper stepperExtruder(E_STEP_PIN, E_DIR_PIN, E_ENABLE_PIN);
FanControl fan(FAN_PIN);
RobotGeometry geometry;
Interpolation interpolator;
Queue<Cmd> queue(15);
Command command;


void setup() {
  Serial.begin(9600);
  
  //various pins..
  pinMode(HEATER_0_PIN  , OUTPUT);
  pinMode(HEATER_1_PIN  , OUTPUT);
  pinMode(LED_PIN       , OUTPUT);
  
  //unused Stepper..
  pinMode(E_STEP_PIN   , OUTPUT);
  pinMode(E_DIR_PIN    , OUTPUT);
  pinMode(E_ENABLE_PIN , OUTPUT);
  
  //unused Stepper..
  pinMode(Q_STEP_PIN   , OUTPUT);
  pinMode(Q_DIR_PIN    , OUTPUT);
  pinMode(Q_ENABLE_PIN , OUTPUT);
  
  //GripperPins
  pinMode(STEPPER_GRIPPER_PIN_0, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_GRIPPER_PIN_3, OUTPUT);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);

  
  //reduction of steppers..
  stepperHigher.setReductionRatio(32.0 / 9.0, 200 * 16);  //big gear: 32, small gear: 9, steps per rev: 200, microsteps: 16
  stepperLower.setReductionRatio( 32.0 / 9.0, 200 * 16);
  stepperRotate.setReductionRatio(32.0 / 9.0, 200 * 16);
  stepperExtruder.setReductionRatio(32.0 / 9.0, 200 * 16);
  
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
    Serial.print(stepper_enabled);
    Serial.println();
  }
  //update and Calculate all Positions, Geometry and Drive all Motors...
  interpolator.updateActualPosition();
  geometry.set(interpolator.getXPosmm(), interpolator.getYPosmm(), interpolator.getZPosmm());
  stepperRotate.stepToPositionRad(geometry.getRotRad());
  stepperLower.stepToPositionRad (geometry.getLowRad());
  stepperHigher.stepToPositionRad(geometry.getHighRad());
  stepperExtruder.stepToPositionRad(interpolator.getEPosmm());
  stepperRotate.update();
  stepperLower.update();
  stepperHigher.update(); 
  fan.update();
  
  if (!queue.isFull()) {
    if (command.handleGcode()) {
      queue.push(command.getCmd());
      printOk();
    }
  }
  if ((!queue.isEmpty()) && interpolator.isFinished()) {
    executeCommand(queue.pop());
  }
    
  if (millis() %500 <250) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}




void cmdMove(Cmd (&cmd)) {
  interpolator.setInterpolation(cmd.valueX, cmd.valueY, cmd.valueZ, cmd.valueE, cmd.valueF);
}
void cmdDwell(Cmd (&cmd)) {
  delay(int(cmd.valueT * 1000));
}
void cmdGripperOn(Cmd (&cmd)) {
  stepper.setSpeed(5);
  stepper.step(int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
}
void cmdGripperOff(Cmd (&cmd)) {
  stepper.setSpeed(5);
  stepper.step(-int(cmd.valueT));
  delay(50);
  digitalWrite(STEPPER_GRIPPER_PIN_0, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_1, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_2, LOW);
  digitalWrite(STEPPER_GRIPPER_PIN_3, LOW);
  //printComment("// NOT IMPLEMENTED");
  //printFault();
}
void cmdStepperOn() {
  setStepperEnable(true);
}
void cmdStepperOff() {
  setStepperEnable(false);
}
void cmdFanOn() {
  fan.enable(true);
}
void cmdFanOff() {
  fan.enable(false);
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

//G1 X0 Y100 Z134  -> 로봇이 앞으로 움직인다
//G1 X0 Y100 Z0   -> 로봇이 아래로 하강한다
//G1 X0 Y19.5 Z134 -> 로봇이 원점으로 이동한다

