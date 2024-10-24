#include <Joystick.h>

#define B_INTAKEIN_CUBEMID 3
#define B_INTAKEOUT_CONEMID 4
#define CLAWGRIP_UNIVERSAL 5
#define HOME 6
#define L_INTAKEIN_CUBEHI 7
#define R_INTAKEIN_CONEHI 8
#define CLAWTILT_FEEDER 9
#define TOGGLE 10
#define ELEV_HEIGHT A4
#define ELEV_ANGLE A5 

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  14, 0,
  true, true, false,
  false, false, false,
  false, false,
  false, false, false);


void setup()  {

 pinMode(B_INTAKEIN_CUBEMID, INPUT_PULLUP); // INTAKE BOTH IN / CUBE MID
 pinMode(B_INTAKEOUT_CONEMID, INPUT_PULLUP); // INTAKE BOTH OUT / CONE MID
 pinMode(CLAWGRIP_UNIVERSAL, INPUT_PULLUP); // CLAW GRIP / UNIVERSAL SCORE
 pinMode(HOME, INPUT_PULLUP); //           / HOME
 pinMode(L_INTAKEIN_CUBEHI, INPUT_PULLUP); // INTAKE LEFT IN / CUBE HIGH
 pinMode(R_INTAKEIN_CONEHI, INPUT_PULLUP); // INTAKE RIGHT IN / CONE HIGH
 pinMode(CLAWTILT_FEEDER, INPUT_PULLUP); // CLAW TILT / FEEDER
 pinMode(TOGGLE, INPUT_PULLUP); // BUTTON TOGGLE
 pinMode(ELEV_HEIGHT, INPUT); // ELEVATOR UP/DOWN
 pinMode(ELEV_ANGLE, INPUT); // ELEVATOR ANGLE
  

 Joystick.begin();
 Joystick.setXAxisRange(-100, 100);
 Joystick.setYAxisRange(-100, 100);
}

void loop() {
  {
   int HI_input = analogRead(ELEV_HEIGHT);
  //  double HI_VAL = ((HI_input-512.0)/512.0);
  //  Joystick.setYAxis(HI_VAL);
   Joystick.setYAxis(map(HI_input, 0, 1023, -100, 100));
   

   int AN_input = analogRead(ELEV_ANGLE);
  //  double AN_VAL = ((AN_input-512.0)/512.0);
  //  Joystick.setXAxis(AN_VAL);
   Joystick.setXAxis(map(AN_input, 0, 1023, -100, 100));

   Joystick.setButton(0, !digitalRead(B_INTAKEIN_CUBEMID)&&digitalRead(TOGGLE));    // 1
   Joystick.setButton(1, !digitalRead(B_INTAKEOUT_CONEMID)&&digitalRead(TOGGLE));   // 2
   Joystick.setButton(2, !digitalRead(CLAWGRIP_UNIVERSAL)&&digitalRead(TOGGLE));    // 3
   Joystick.setButton(3, !digitalRead(HOME) && digitalRead(TOGGLE));                // 4

   Joystick.setButton(4, !digitalRead(L_INTAKEIN_CUBEHI)&&digitalRead(TOGGLE));     // 5
   Joystick.setButton(5, !digitalRead(R_INTAKEIN_CONEHI)&&digitalRead(TOGGLE));     // 6
   Joystick.setButton(6, !digitalRead(CLAWTILT_FEEDER)&&digitalRead(TOGGLE));       // 7


   Joystick.setButton(7, !digitalRead(B_INTAKEIN_CUBEMID) && !digitalRead(TOGGLE));  // 8
   Joystick.setButton(8, !digitalRead(B_INTAKEOUT_CONEMID) && !digitalRead(TOGGLE)); // 9
   Joystick.setButton(9, !digitalRead(CLAWGRIP_UNIVERSAL) && !digitalRead(TOGGLE));  // 10
   Joystick.setButton(10, !digitalRead(HOME) && !digitalRead(TOGGLE));               // 11

   Joystick.setButton(11, !digitalRead(L_INTAKEIN_CUBEHI) && !digitalRead(TOGGLE));   // 12
   Joystick.setButton(12, !digitalRead(R_INTAKEIN_CONEHI) && !digitalRead(TOGGLE));   // 13
   Joystick.setButton(13, !digitalRead(CLAWTILT_FEEDER) && !digitalRead(TOGGLE));     // 14

   //Joystick.setButton(8, !digitalRead(B_INTAKEIN_CUBEMID)&&!digitalRead(TOGGLE)); //8

   //Joystick.setButton(7, !digitalRead(B_INTAKEOUT_CONEMID)&&!digitalRead(TOGGLE));  //8
   //Joystick.setButton(8, !digitalRead(CLAWGRIP_UNIVERSAL)&&!digitalRead(TOGGLE)); //9
   //Joystick.setButton(9, !digitalRead(HOME)&&!digitalRead(TOGGLE));               //10
   //Joystick.setButton(10, !digitalRead(L_INTAKEIN_CUBEHI)&&!digitalRead(TOGGLE)); //11
   //Joystick.setButton(11, !digitalRead(R_INTAKEIN_CONEHI)&&!digitalRead(TOGGLE)); //12
   //Joystick.setButton(12, !digitalRead(CLAWTILT_FEEDER)&&!digitalRead(TOGGLE));   //13
   

  }
 delay(10);
}
