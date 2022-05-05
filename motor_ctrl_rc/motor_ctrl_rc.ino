#include <ServoInput.h> //Read RC Receiver
#include <Servo.h>  //Control ESCs



// RC Receiver defines and objects
#define PIN_CH1 2 //Receiver channel 1 (mode switch)
#define PIN_CH2 3 //Receiver channel 2 (left thruster)
#define PIN_CH3 7 //Receiver channel 2 (right thruster)

#define PIN_ESC_L 5
#define PIN_ESC_R 6


ServoInputPin<PIN_CH1> mode_switch;
ServoInputPin<PIN_CH2> thruster_l;
ServoInputPin<PIN_CH3> thruster_r;

Servo esc_l;
Servo esc_r;



void setThrust(int left, int right){
  esc_l.writeMicroseconds(left);
  esc_r.writeMicroseconds(right);
}

void processSerial(const char* data){
  int left_speed = data[0]*4;
  int left_dir = (data[1] == 0) ? 1: -1;
  int right_speed = data[2]*4;
  int right_dir = (data[3] == 0) ? 1: -1;

  int left = (left_speed * left_dir) + 1500;
  int right = (right_speed * right_dir) + 1500;

  setThrust(left, right);
}

// Serial protocol
// byte 1: left thruster speed 0-100
// byte 2: left thruster direction (0 forwards, 1 reverse)
// byte 3: right thruster speed 0-100
// byte 4: right thruster direction (0 forwards, 1 reverse)
// byte 5: terminating byte 0xFF
void readSerial(const byte in){
  static char input[10];
  static unsigned int pos = 0;

  if(in == 0xFF){
    input[pos] = 0;
    processSerial(input);

    pos = 0;
  }
  else{
    if(pos < 9){
      input[pos++] = in;
    }
  } 
}



void setup() {  
  Serial.begin(9600);

  esc_l.attach(PIN_ESC_L);
  esc_r.attach(PIN_ESC_R);
  
  setThrust(1500, 1500);  //make sure thrusters are stopped
}

void loop() {    
  if(mode_switch.getBoolean()){ //switch activated - manual control
  
    //Write PWM values direct to ESCs
    setThrust(thruster_l.getPulseRaw(), thruster_r.getPulseRaw()); 
  }
  else{ //switch not activated - auto control via serial

    while(Serial.available() > 0){
      readSerial(Serial.read());
    }
  }

}
