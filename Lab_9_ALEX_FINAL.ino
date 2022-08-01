// This code is designed to find the center of a room, follow a line out of the room
// and complete a subsequent maze to exit the area. 
// Lab 9 by Alex South and Sam Butts 05/02/2022

#include <Servo.h> //for whatever reason this is VITAL to have first
//#include <SimpleRSLK.h>
#include "SimpleRSLK.h"

#define WHEELSPEED_L 15 //Set nominal wheel speeds
#define WHEELSPEED_R 15
#define SpeedScale 2 //Adjustment amount for changing motor powers
#define AdjustScale 1 //Adjustment amount for activating the change in motor powers

#define TWHEELSPEED_L 15 //Set a slightly lower turning speed
#define TWHEELSPEED_R 15

#define WheelDia 7.0 //Set wheel diameter to 7 cm
#define WheelBase 13.80 //Set Wheel base 10.235
#define STOP 0 //Useful to use words instead of the number 0

uint8_t trig = 0; //This is a variable used to operate with the StartB function to wait for the Bump before starting
uint8_t trig2 = 0;

Servo myservo;  // create servo object to control a servo
                // a maximum of eight servo objects can be created
                
int arrsize[11]; //This is our array to store the numbers
int rdg ; // This is the readings from the ultra sonic sensor
long pulseLength;
long centimeters;
const int trigPin = 32; // Port Pin 3.5
const int echoPin = 33; // Port Pin 5.1
#define BLUE 77 // Define blue pin

// Variable for the bottem sensors
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

// This code sets the servo to zero
void StartM(void){
 myservo.attach(17); // attaches the servo on Port 2.4 to the servo object (this was not used for the final code)
 myservo.write(0); // Send it to the default position
}

//This code allows us to hit the bump in between tests and is only reset when trig is set back to zero
void StartB(void) {
 while( trig == 0) {
 if(isBumpSwitchPressed(0) == true){
 trig = trig + 1;
 }
 else{}
 }
}

//This part of the code allows us to average 10 ultrasonic measurements
void sensorRDG(){
 for (rdg=0; rdg<11; rdg++) {

    digitalWrite(trigPin, LOW); // low for clean pulse
    delayMicroseconds(10); // rest
    digitalWrite(trigPin, HIGH); // higher trigger device
    delayMicroseconds(10); // rest
    digitalWrite(trigPin, LOW); // low for clean pulse
    delayMicroseconds(10); // settle
    pulseLength = pulseIn (echoPin, HIGH); // measure pulse back
    centimeters = pulseLength / 58;
    arrsize[rdg] = centimeters;

    delay(20);
 }
   int i,j,tmp;
   for (i = 0; i < 11; i++) { //Loop for ascending ordering
    for (int j = 0; j < 11; j++) { //Loop for comparing other values
      if (arrsize[j] > arrsize[i]) { //Comparing other array elements
        tmp = arrsize[i]; //Using temp var for storing last value
        arrsize[i] = arrsize[j]; //replacing value
          arrsize[j] = tmp; //storing last value
         }
       }
    }
 }
 
//This code turns left exactly to the degrees we adjust to while keeping the center of the bot on a pin
void TurnL (int deg) {
     // Define speed and encoder count variables
     uint16_t l_motor_speed_h = TWHEELSPEED_L + SpeedScale;
     uint16_t l_motor_speed_l = TWHEELSPEED_L - SpeedScale;
     uint16_t r_motor_speed_h = TWHEELSPEED_R + SpeedScale;
     uint16_t r_motor_speed_l = TWHEELSPEED_R - SpeedScale;
      
     uint16_t l_motor_speed = TWHEELSPEED_L;
     uint16_t r_motor_speed = TWHEELSPEED_R;
     uint16_t l_totalCount = 0;
     uint16_t r_totalCount = 0;
     // Amount of encoder pulses needed to achieve distance
     uint16_t lstraight = WheelBase / WheelDia * deg;//mathamatically should be 2 not 1.4
     uint16_t rstraight = WheelBase / WheelDia * deg;
     // Set up the motors and encoders
     resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse count back to 0
     setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD); // Cause the robot to drive forward
     setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);
     enableMotor(BOTH_MOTORS); // "Turn on" the motor
     setMotorSpeed(LEFT_MOTOR, l_motor_speed); // Set motor speeds - variable,
     setMotorSpeed(RIGHT_MOTOR, r_motor_speed); // may change (adjust) later
    
     // Drive both motors until both have received the correct number of pulses to travel
     while( (l_totalCount<lstraight) || (r_totalCount<rstraight) ) {
    
     l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
     if((l_totalCount) == r_totalCount) {


     setMotorSpeed(LEFT_MOTOR, l_motor_speed);
     setMotorSpeed(RIGHT_MOTOR, r_motor_speed);
     }
    
     if((l_totalCount + AdjustScale) < r_totalCount) {
     setMotorSpeed(LEFT_MOTOR, l_motor_speed_h);
     setMotorSpeed(RIGHT_MOTOR, r_motor_speed_l);
     }
    
     if(l_totalCount > (r_totalCount + AdjustScale)) {
     setMotorSpeed(LEFT_MOTOR, l_motor_speed_l);
     setMotorSpeed(RIGHT_MOTOR, r_motor_speed_h);
     }
    
     if (l_totalCount >= lstraight) disableMotor(LEFT_MOTOR);
     if (r_totalCount >= rstraight) disableMotor(RIGHT_MOTOR);

     }
     delay(100);
// trig = 0; //for whatever reason this caused our code to break
}

//This code turns right a specified amount of degrees
void TurnR (int deg) {
       // Define speed and encoder count variables
       uint16_t l_motor_speed_h = TWHEELSPEED_L + SpeedScale;
       uint16_t l_motor_speed_l = TWHEELSPEED_L - SpeedScale;
       uint16_t r_motor_speed_h = TWHEELSPEED_R + SpeedScale;
       uint16_t r_motor_speed_l = TWHEELSPEED_R - SpeedScale;
      
       uint16_t l_motor_speed = TWHEELSPEED_L;
       uint16_t r_motor_speed = TWHEELSPEED_R;
       uint16_t l_totalCount = 0;
       uint16_t r_totalCount = 0;
       // Amount of encoder pulses needed to achieve distance
       uint16_t lstraight = WheelBase / WheelDia * deg;//mathamatically should be 2 not 1.4
       uint16_t rstraight = WheelBase / WheelDia * deg;
       // Set up the motors and encoders
       resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse count back to 0
       setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD); // Cause the robot to drive forward
       setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);
       enableMotor(BOTH_MOTORS); // "Turn on" the motor
       setMotorSpeed(LEFT_MOTOR, l_motor_speed); // Set motor speeds - variable,
       setMotorSpeed(RIGHT_MOTOR, r_motor_speed); // may change (adjust) later
      
      // waitBtnPressed (A6);
       // Drive both motors until both have received the correct number of pulses to travel
       while( (l_totalCount<lstraight) || (r_totalCount<rstraight) ) {
      
       l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
       if((l_totalCount) == r_totalCount) {
       setMotorSpeed(LEFT_MOTOR, l_motor_speed);
       setMotorSpeed(RIGHT_MOTOR, r_motor_speed);
       }
      
       if((l_totalCount + AdjustScale) < r_totalCount) {
       setMotorSpeed(LEFT_MOTOR, l_motor_speed_h);
       setMotorSpeed(RIGHT_MOTOR, r_motor_speed_l);
       }
      
       if(l_totalCount > (r_totalCount + AdjustScale)) {
       setMotorSpeed(LEFT_MOTOR, l_motor_speed_l);
       setMotorSpeed(RIGHT_MOTOR, r_motor_speed_h);
       }
      
       // Stop motors if they reach 1 meter
       if (l_totalCount >= lstraight) disableMotor(LEFT_MOTOR);
       if (r_totalCount >= rstraight) disableMotor(RIGHT_MOTOR);
      
       }
 delay(100);
// trig = 0;
}

//This part of the code goes straight a specified distance
void Straight (int Distance) {
     // Define speed and encoder count variables
     uint16_t l_motor_speed_h = WHEELSPEED_L + SpeedScale;
     uint16_t l_motor_speed_l = WHEELSPEED_L - SpeedScale;
     uint16_t r_motor_speed_h = WHEELSPEED_R + SpeedScale;
     uint16_t r_motor_speed_l = WHEELSPEED_R - SpeedScale;
    
     uint16_t l_motor_speed = WHEELSPEED_L;
     uint16_t r_motor_speed = WHEELSPEED_R;
    
     uint16_t l_totalCount = 0;
     uint16_t r_totalCount = 0;
     // Amount of encoder pulses needed to achieve distance
     uint16_t lstraight = ((Distance/21.99)*360);
     uint16_t rstraight = ((Distance/21.99)*360);
     // Set up the motors and encoders
     resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse count back to 0
     setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward
     enableMotor(BOTH_MOTORS); // "Turn on" the motor
    
     // Drive both motors until both have received the correct number of pulses to travel
     while( (l_totalCount<lstraight) || (r_totalCount<rstraight) ) {
    
     l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    
         if((l_totalCount) == r_totalCount) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed);
         }
        
         if((l_totalCount + AdjustScale) < r_totalCount) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed_h);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed_l);
         }
        
         if(l_totalCount > (r_totalCount + AdjustScale)) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed_l);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed_h);
         }
        
         // Stop motors if they reach 1 meter
         if (l_totalCount >= lstraight) disableMotor(LEFT_MOTOR);
         if (r_totalCount >= rstraight) disableMotor(RIGHT_MOTOR);
     }
 delay(100);
// trig = 0;
}

//this is an attempt to move backward 
void Backward (int Distance) {
     // Define speed and encoder count variables
     uint16_t l_motor_speed_h = WHEELSPEED_L + SpeedScale;
     uint16_t l_motor_speed_l = WHEELSPEED_L - SpeedScale;
     uint16_t r_motor_speed_h = WHEELSPEED_R + SpeedScale;
     uint16_t r_motor_speed_l = WHEELSPEED_R - SpeedScale;
    
     uint16_t l_motor_speed = WHEELSPEED_L;
     uint16_t r_motor_speed = WHEELSPEED_R;
    
     uint16_t l_totalCount = 0;
     uint16_t r_totalCount = 0;
     // Amount of encoder pulses needed to achieve distance
     uint16_t lstraight = ((Distance/21.99)*360);
     uint16_t rstraight = ((Distance/21.99)*360);
     // Set up the motors and encoders
     resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse count back to 0
     setMotorDirection(BOTH_MOTORS,MOTOR_DIR_BACKWARD); // Cause the robot to drive forward
     enableMotor(BOTH_MOTORS); // "Turn on" the motor
    
     // Drive both motors until both have received the correct number of pulses to travel
     while( (l_totalCount<lstraight) || (r_totalCount<rstraight) ) {
    
     l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    
         if((l_totalCount) == r_totalCount) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed);
         }
        
         if((l_totalCount + AdjustScale) < r_totalCount) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed_h);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed_l);
         }
        
         if(l_totalCount > (r_totalCount + AdjustScale)) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed_l);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed_h);
         }
        
         // Stop motors if they reach 1 meter
         if (l_totalCount >= lstraight) disableMotor(LEFT_MOTOR);
         if (r_totalCount >= rstraight) disableMotor(RIGHT_MOTOR);
     }
 delay(100);
// trig = 0;
}

//drift left
void DriftL (int Distance) {
     // Define speed and encoder count variables
     double drift = 0.8;
     
     double l_motor_speed_h = (WHEELSPEED_L + SpeedScale)*drift;
     double l_motor_speed_l = (WHEELSPEED_L - SpeedScale)*drift;
     double r_motor_speed_h = WHEELSPEED_R + SpeedScale;
     double r_motor_speed_l = WHEELSPEED_R - SpeedScale;
    
     double l_motor_speed = WHEELSPEED_L*drift;
     double r_motor_speed = WHEELSPEED_R;
    
     double l_totalCount = 0;
     double r_totalCount = 0;
     // Amount of encoder pulses needed to achieve distance
     double lstraight = ((Distance/21.99)*360)*drift;
     double rstraight = ((Distance/21.99)*360);
     // Set up the motors and encoders
     resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse count back to 0
     setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward
     enableMotor(BOTH_MOTORS); // "Turn on" the motor
    
     // Drive both motors until both have received the correct number of pulses to travel
     while( (l_totalCount<lstraight) || (r_totalCount<rstraight) ) {
    
     l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    
         if((l_totalCount) == r_totalCount*drift) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed);
         }
        
         if((l_totalCount + AdjustScale) < r_totalCount*drift) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed_h);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed_l);
         }
        
         if(l_totalCount > (r_totalCount + AdjustScale)*drift) {
         setMotorSpeed(LEFT_MOTOR, l_motor_speed_l);
         setMotorSpeed(RIGHT_MOTOR, r_motor_speed_h);
         }
        
         // Stop motors if they reach 1 meter
         if (l_totalCount >= lstraight) disableMotor(LEFT_MOTOR);
         if (r_totalCount >= rstraight) disableMotor(RIGHT_MOTOR);
     }
 delay(100);
// trig = 0;
}

// Line following function
void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

  delay(1000);

  Serial.println("Running calibration on floor");
  simpleCalibrate();
  Serial.println("Reading floor values complete");

//  btnMsg = "Push left button on Launchpad to begin line following.\n";
//  btnMsg += "Make sure the robot is on the line.\n";
//  /* Wait until button is pressed to start robot */
//  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
//  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS,20);

  for(int x = 0;x<100;x++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}

//variables are declared here 
//these variables are for finding the closest wall
bool isCalibrationComplete = false;
int testing = 1; //not sure why we have this in here it was useful at some point
double mems = 1; //this is a variable that we probably don't need anymore
int i; //variable to iterate through sensing loop
int a[31][2]; // this is the array where we store the values from the circle
int s; //variable to find minimum array value
int f; //variable to help find minimum array value
float tmp; //temp variable that helps with sorting
float tmp2;//temp variable that helps with sorting distances
int pos = 0; // variable to store the servo position

//center room variables
int Dup;
int Dleft;
int Ddown;
int Dright; 
int DmidSide;
int DmidTop;

//these variabes help us get EXACTLY square with the wall  
int Dist1;//first measured distance
int Dist2;
int RIGHTFIX = 88;

//These variables help us get through the maze 
int Dwall = 5; // Orginally was 15
int DBox = 30;// orginally 25
int Dfar = 40;// orginally 25

int Ddrift2;//this ended up not getting used but helps us stay close with the wall

int Cleft; //both of these did not get used 
int Cright;

//setup function has a calibration portion and initialization 
void setup()
{
   Serial.begin(115200);
   setupRSLK();
   setupWaitBtn(LP_LEFT_BTN);
   /* Red led in rgb led */
   setupLed(RED_LED);
   clearMinMax(sensorMinVal,sensorMaxVal);
   myservo.attach(17); // attaches the servo to pin 17
   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
   pinMode(BLUE, OUTPUT); // Blue LED
   delay(500);
}

//The bump initiates the code after calibration 
void loop()
{
  //these are line following variables 
 uint16_t normalSpeed = 0;
 uint16_t fastSpeed = 15; 
 uint8_t lineColor = DARK_LINE;
 
  /* Run this setup only once */
 if(isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }

 StartB(); //wait for the far right bumper to be hit to start the code
 delay(1000);

 for (i = 0; i < 16; i++) { //this part of the code is the 180° sweep part
     sensorRDG(); //take an accurate sensor reading
     a[i][0] = i; //store which iteration it is on
     a[i][1] = arrsize[6]; //store the distance reading as well
     int Servoposition = 0; //initialize variable to keep track of which degree we are on
     Servoposition = i*12; //This makes the servo turn after each reading 10 times to get to 180°
     myservo.write(Servoposition); // tell servo to go to position in variable 'pos'
     delay(1000); // waits 15ms for the servo to reach the position
     Serial.print("Sorted Distance in centimeters");//all testing checks here
     Serial.println(" ");
     Serial.print(a[i][0]);Serial.print("     ");
     Serial.print(a[i][1]);Serial.print("   =  ");
     Serial.println(arrsize[6]);
     delay(100);

 }
   TurnL(180);//turn 180°
   myservo.write(0); //reset the servo position to 0
   delay(1000);

 for (i = 16; i < 31; i++) {//do the second 180° sweep with the servo same comments as above
     sensorRDG();
     a[i][0] = i;//this code uses the same array but takes array positions 11-20 instead of 1-10
     a[i][1] = arrsize[6];
     int Servoposition = 0;
     Servoposition = (i*12)-180;
     myservo.write(Servoposition); // tell servo to go to position in variable 'pos'
     delay(1000); // waits 15ms for the servo to reach the position
     Serial.print("Sorted Distance in centimeters");
     Serial.println(" ");
     Serial.print(a[i][0]);Serial.print("     ");
     Serial.print(a[i][1]);Serial.print("   =  ");
     Serial.println(arrsize[6]);
     delay(100);

 }
 for (s = 0; s < 31; s++) { //This is a niffty loop that sorts a two dimensional array based upon the second column

    for (f = 0; f < 31; f++) { //but keeps the first column values corrolated with them
        if (a[f][1] > a[s][1]) { //Comparing other array elements

           tmp = a[s][1]; //Using temp var for storing last distance array value
           tmp2 = a[s][0]; //using temp2 to store number of times we have turned thus far
          
           a[s][1] = a[f][1]; //replacing value
           a[s][0] = a[f][0];
          
           a[f][1] = tmp; //storing last value
           a[f][0] = tmp2; //storing last value
        }
      }
   }

  
   myservo.write(90); //Resets the servo to "looking forward" position
   TurnL(90); //Hit a sick 90 to put us at 0° relative to all of the servo positions
   TurnL(a[0][0]*12); //Turn to the left to the recorded lowest servo iteration position

   TurnR(50);//this allows us to start a more accurate sweep that places us at exactly 90 with the wall. 

  //this loop is supposed to throw us at exactly 90° against the wall
  while(testing == 1){
    sensorRDG();  
    Serial.print(arrsize[6]);
    Dist1=arrsize[6];
    delay(500);    
    TurnL(5);              //Turn 10° to the left
    delay(500);
    sensorRDG();
    Dist2=arrsize[6];

    if (Dist1 > Dist2){}
    else if(Dist1 == Dist2){
      mems = mems + 1;
    }
    else{
      TurnR(5 * mems/2 + 1);
      testing = 0;
    }
  }

//this little section finds the size of the room
   sensorRDG();
   Dup = arrsize[6];
   delay (500);
   TurnL(RIGHTFIX); //Hit a sick 90 to put us at 0° relative to all of the servo positions
   sensorRDG();
   Dleft = arrsize[6];
   delay (500);
   TurnL(RIGHTFIX); //Hit a sick 90 to put us at 0° relative to all of the servo positions
   sensorRDG();
   Ddown = arrsize[6];
   TurnL(RIGHTFIX); //Hit a sick 90 to put us at 0° relative to all of the servo positions
   delay(500);
   sensorRDG();
   Dright = arrsize[6];
   TurnL(RIGHTFIX);

////  THIS PART GOES TO THE MIDDLE OF TE SQUARE
   if(Dleft > Dright){
    DmidSide = Dright;
    DmidTop = Dup;
    TurnL(RIGHTFIX);
    Straight((Dright+Dleft)/2-(DmidSide));
    TurnL(RIGHTFIX);
    Straight((Dup+Ddown)/2-(DmidTop));
   }
   else{
    DmidSide = Dleft;
    DmidTop = Dup;
    TurnR(RIGHTFIX);
    Straight((Dright+Dleft)/2-(DmidSide));
    TurnR(RIGHTFIX);
    Straight((Dup+Ddown)/2-(DmidTop));
   }
   
   delay(300);
   Serial.println("made it to loop");

// Part 2 finding the line by hitting a sick 360 kickflip 
   while(sensorVal[7] < 2300)
    {
      /* Set both motors direction forward */
      setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
     /* Enable both motors */
      enableMotor(BOTH_MOTORS);
      
      readLineSensor(sensorVal);
      setMotorSpeed(RIGHT_MOTOR, 20);
      setMotorSpeed(LEFT_MOTOR, 0);
      
      //small troubleshooting prints here 
      Serial.println(" ");
        Serial.print(sensorVal[0]);Serial.print("   ");
        Serial.print(sensorVal[1]);Serial.print("   ");
        Serial.print(sensorVal[2]);Serial.print("   ");
        Serial.print(sensorVal[3]);Serial.print("   ");
        Serial.print(sensorVal[4]);Serial.print("   ");
        Serial.print(sensorVal[5]);Serial.print("   ");
        Serial.print(sensorVal[6]);Serial.print("   ");
        Serial.print(sensorVal[7]);Serial.print("   ");
      delay(100);
    }
    
    TurnR(90);//this aligns us with the line 
    /* Set both motors direction forward */
    setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
    /* Enable both motors */
    enableMotor(BOTH_MOTORS);
        Serial.println("made it to loop2");

//This loop follows the line until the end sensors hit the T line 
while(sensorVal[0] < 2400 || sensorVal[7] < 2400)
{ readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);

  uint32_t linePos = getLinePosition(sensorCalVal,lineColor);
        delay(10);  Serial.println(linePos);

        //this section allows us to follow the line very quickly since the motor 
        //is stopped when it is not suppose to be moving 
        
  if(linePos > 0 && linePos < 3000) {
    setMotorSpeed(LEFT_MOTOR,normalSpeed);
    setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  } else if(linePos > 3000 ) {
    setMotorSpeed(LEFT_MOTOR,fastSpeed);
    setMotorSpeed(RIGHT_MOTOR,normalSpeed);
  }   else {
    setMotorSpeed(LEFT_MOTOR,fastSpeed);
    setMotorSpeed(RIGHT_MOTOR,fastSpeed);
  }
  }

  //after we hit the T line we want to stop and mentally prepare for the next phase in our journey 
  setMotorSpeed(BOTH_MOTORS,0);
  resetLeftEncoderCnt(); resetRightEncoderCnt(); // Set encoder pulse count back to 0

  //This puts us next to the wall so we can begin following
  TurnL(RIGHTFIX);
  Straight(14);
  
// The Great maze!!!
  while(trig2 == 0){

    myservo.write(90);//turn servo forward 
    delay(1000);
    sensorRDG();//Scan forward 
    Dup = arrsize[6]; //forward measurement 
    Serial.print(arrsize[6]);Serial.println("     ");
    myservo.write(180);//turn servo left 
    delay(1000);
    sensorRDG();//Scan Left
    Dleft = arrsize[6]; //left measurement 

     if(Dleft-7 > Dfar){//wall far left
        //CONDITIONAL CHOICE VOID
        //This part of the if else statement hits a mean U turn around the edge of the wall
        Straight(25);
        TurnL(87);//turn left
        Straight(30);
        TurnL(87);//turn left
        Straight(34);
      }
      else if(Dup <= Dwall+10){//something forward turn right
          myservo.write(0);//turn servo right 
          delay(1000);
          sensorRDG();//Scan Right
          Dright = arrsize[6]; //servo scan to the right 
      
          if(Dright-7 <= Dwall){//something is right
            TurnR(180);//do a 180
          }
          else{//nothing is right? go right
          TurnR(85);
          Straight(Dwall);//go forward distance-robot width 
          }
      }

    else if(Dleft-7 < Dwall && Dup > Dwall){//Veered too close to the wall
      if(Dup > Dwall){//distance is greater than robot width
        TurnR(2);
        Straight(12);//go forward 1 robot width 
      }
      else{
        TurnR(2);
        Straight(Dup - Dwall);//go forward distance-robot width
      }
    }
    else if(Dleft-7 == Dwall && Dup > Dwall){//perfect distance from the wall
      if(Dup > Dwall){//distance is greater than robot width
        Straight(12);//go forward 1 robot width 
      }
      else{
        Straight(Dup - Dwall);//go forward distance-robot width
      }
    }
    else if(Dleft-7 > Dwall && Dup > Dwall && Dleft-7 < 15){Veered from the wall but not a gap yet 
      if(Dup > Dwall){//distance is greater than robot width
        TurnL(2);
        Straight(12);//go forward 1 robot width 
      }
      else{
        TurnL(2);
        Straight(Dup - Dwall);//go forward distance-robot width
      }
    }
    else if(Dleft-7 >= 15 && Dup > Dwall && Dleft-7 <= Dfar){//THE BOX situation 
    Straight(14);
    TurnL(87);
    Straight(14);
    }
    
    //this was supposed to be a backup in case we hit the wall but it never worked out 
    else if(isBumpSwitchPressed(0) == true || isBumpSwitchPressed(1) == true || isBumpSwitchPressed(2) == true || isBumpSwitchPressed(3) == true || isBumpSwitchPressed(4) == true || isBumpSwitchPressed(5) == true){
      Backward(14);
      TurnR(87);
    }

    else{//infinite nothing left infinite nothing straight
      //trig2 = 1;
      Serial.println("stuck");Serial.println("stuck");Serial.println("stuck");Serial.println("stuck");Serial.println("stuck");
    }
  }
  
   trig = 0; //reset the testing loop (this took 5 hours to fix)
}
