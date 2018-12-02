// add motor library
#include <AFMotor.h>

//global variables
AF_DCMotor rightMotor(1,MOTOR12_64KHZ);
AF_DCMotor leftMotor(3,MOTOR34_64KHZ);
#define EncoderPinRM 3
#define EncoderPinLM 2

bool LeftEncoderHigh = 0; // bool for signal of the left encoder
bool RightEncoderHigh = 0; // bool for signal of the right encoder
bool StateChangedRightEncoder = 0; // checks if there was a change in the right encoder signal
bool StateChangedLeftEncoder = 0; // checks if there was a change in the left encoder signal
unsigned long time;
long dt; // time taken for sampling encoder signal
double encoderM = 0.00375; // encoder specifics
//int right_encoder_counter; // counter of ticks read right encoder
//int left_encoder_counter; // counter of ticks read left encoder
float v_left = 0.0;
float v_right = 0.0;


// parameters to set
//float input_right_motor = 107; // pre-calculated intial input right motor
//float input_left_motor = 128; // pre-calculated intial input left motor
float input_right_motor = 255; // pre-calculated intial input right motor
float input_left_motor = 255; // pre-calculated intial input left motor

float ref_vel = 0.09; //reference velocity (m/s) for both motors
float p_reg = 100.0; // P controller gain

// simple P controller for maintaining both motors at a specific velocity
void encoderController(float &input_right_motor, float &input_left_motor, float dt, int right_encoder_counter, int left_encoder_counter);
// check changes in the encoders state
int EncoderEventRight();
int EncoderEventLeft();

void runForwardLM(int input);
void runForwardRM(int input);

void setup()
{
  // set pins to encoder and set interrupt
  pinMode(EncoderPinRM, INPUT);
  pinMode(EncoderPinLM, INPUT);
  attachInterrupt(0, EncoderEventRight, CHANGE);
  attachInterrupt(1, EncoderEventLeft, CHANGE);
  Serial.begin(9600);
}

// reads the right encoder constantly
// returns the state of the encoder
int EncoderEventRight()
{
  int EncoderPosRight = 0;
  if(digitalRead(EncoderPinRM) == HIGH)
    {
      RightEncoderHigh = 1;
    }
  // if's used for easier debbuging, change later
  if ((RightEncoderHigh == 1))
    {
      if(digitalRead(EncoderPinRM) == LOW)
        {
          StateChangedRightEncoder = 1;
          RightEncoderHigh = 0;
          EncoderPosRight++;
        }
      //Serial.print("Encoder Right: ");
      //Serial.println(EncoderPosRight);
    }
  StateChangedRightEncoder = 0;
  return EncoderPosRight;
}

// reads the left encoder constantly
// returns the state of the encoder
int EncoderEventLeft()
{
  int EncoderPosLeft = 0;
  if(digitalRead(EncoderPinLM) == HIGH)
    {
      LeftEncoderHigh = 1;
    }
  // if's used for easier debbuging, change later
  if ((LeftEncoderHigh == 1))
    {
    if(digitalRead(EncoderPinLM) == LOW)
      {
        StateChangedLeftEncoder = 1;
        LeftEncoderHigh = 0;
        EncoderPosLeft++;
       }
     // Serial.print("Encoder Left: ");
      //Serial.println(EncoderPosLeft);
    }
  StateChangedLeftEncoder = 0;
  return EncoderPosLeft;
}

// send input to right motor
// int input: input value
void runForwardRM(int input)
  {
    rightMotor.setSpeed(input);
    rightMotor.run(FORWARD);
  };
  
// send input to left motor
// int input: input value
void runForwardLM(int input)
  {
    leftMotor.setSpeed(input);
    leftMotor.run(FORWARD);
  };

// main loop
void loop(){
  
  int right_encoder_counter = 0;
  int left_encoder_counter = 0; 
  int time_interval = 2000000;
  float convert_to_sec = 1000000.0;
  
  //gets time in microseconds
  time = micros();
  // count encoder ticks in a given time interval
  while (micros() - time < 4000000) 
    {
      right_encoder_counter = right_encoder_counter + EncoderEventRight();
      left_encoder_counter = left_encoder_counter + EncoderEventLeft();
      dt = (micros() - time)/1000000; // checks the time (better accuracy??)
    }
  // call controller
  encoderController(input_right_motor, input_left_motor, dt ,right_encoder_counter, left_encoder_counter);
  // resers
  //right_encoder_counter = 0;
  //left_encoder_counter = 0; 
  
  Serial.print("Motor Velocity right: ");
  Serial.println(input_right_motor);
   
  Serial.print("Motor Velocity left: ");
  Serial.println(input_left_motor);
  // send input to motors
  runForwardRM(input_right_motor);
  runForwardLM(input_left_motor);

}

// P-Controller for the velocity of the motors, updates the input given to the motors
// float input_right_motor and input_left_motor: current input value
// float dt: time for sampling encoder ticks
// int right_encoder_counter and left_encoder_counter: ticks counted from the encoders
void encoderController(float &input_right_motor, float &input_left_motor, float dt, int right_encoder_counter, int left_encoder_counter)
  {
    v_left = left_encoder_counter*encoderM/dt; // calculates current velocity of the left motor
    v_right = right_encoder_counter*encoderM/dt; // calculates current velocity of the right motor
    Serial.print("Velocity right: ");
    Serial.println(v_right);
    Serial.print("Velocity left: ");
    Serial.println(v_left);
    //Serial.println(right_encoder_counter);
    //Serial.println(left_encoder_counter);
    
    float diff_left = ref_vel - v_left; // gets left motor velocity error
    float diff_right = ref_vel - v_right; // gets right motor velocity error

    // P-Controller
    input_right_motor = input_right_motor + p_reg*diff_right;
    input_left_motor = input_left_motor + p_reg*diff_left;
    
    // checks if the input values are out of limits
    if (input_left_motor < 0){
      input_left_motor =0;
    }
    if (input_right_motor < 0){
      input_right_motor = 0;
    }
    if (input_left_motor > 255){
      input_left_motor =255;
    }
    if (input_right_motor > 255){
      input_right_motor = 255;
    }

};
