#include <PPMReader.h>
#include <Servo.h>


// Initialize a PPMReader on digital pin 3 with 6 expected channels.
byte interruptPin = 3;
byte channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);
int channel_value_list[6];

byte LEngine_PINS = 10;
byte REngine_PINS = 9;

int LServo_PINS = 6;
int RServo_PINS = 5;
Servo my_l_servo;
Servo my_r_servo;

Servo l_engine;
Servo r_engine;

int LEngine_pwm_value = 127;
int REngine_pwm_value  = 128;
int LServo_pwm_value = 128;
int RServo_pwm_value = 128;

int throttle, pitch, roll, yaw = 128;
int arm,flightmode=1000;
void setup() {
    Serial.begin(115200);
    delay(500);
    //TCCR0B = TCCR0B & B11111000 | B00000101;
    //TCCR2B = TCCR2B & B11111000 | B00000100; 
    my_l_servo.attach(LServo_PINS);
    my_r_servo.attach(RServo_PINS); 
    
    my_l_servo.write(90);
    my_r_servo.write(90);

    l_engine.attach(LEngine_PINS);
    r_engine.attach(REngine_PINS);
   
    //analogWrite(LEngine_PINS, 127); 
    //analogWrite(REngine_PINS, 128); 

    //analogWrite(LServo_PINS, 128); 
    //analogWrite(RServo_PINS, 128); 



}

void rc_ppm_read_values(){
  for (byte channel = 1; channel <= channelAmount; ++channel) {
        channel_value_list[channel] = ppm.latestValidChannelValue(channel, 0);
        //Serial.print(String(channel_value_list[channel]) + "\t");
    }
  throttle = channel_value_list[3];
  pitch = channel_value_list[2];
  roll = channel_value_list[1];
  yaw = channel_value_list[4];
  arm = channel_value_list[5];
  flightmode = channel_value_list[6];
  }

void compute_control(){
  compute_servo_control();
  compute_engine_control();
  }

void compute_engine_control(){
  LEngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) + normalizeThrottle(throttle)*1000*normalizePRY(roll) +normalizeThrottle(throttle)*1000*normalizePRY(yaw);
  REngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) - normalizeThrottle(throttle)*1000*normalizePRY(roll) -normalizeThrottle(throttle)*1000*normalizePRY(yaw);
  if(arm<1500){
      LEngine_pwm_value=1000;
      REngine_pwm_value=1000;
    }
  else{
      if(LEngine_pwm_value<1050){
          LEngine_pwm_value=1050;
        }
      if(REngine_pwm_value<1050){
          REngine_pwm_value=1050;
        }
    }
  }


void compute_servo_control(){
  LServo_pwm_value =( 90 + 1*(80*normalizePRY(pitch)+80*normalizePRY(yaw)-80*normalizePRY(roll))); 
  RServo_pwm_value =( 90 + -1*(80*normalizePRY(pitch)-80*normalizePRY(yaw)+80*normalizePRY(roll)));
  }

void apply_control(){
  apply_servo_control();
  apply_eingine_control();
  }

void apply_servo_control(){
  my_l_servo.write(int(LServo_pwm_value));
  my_r_servo.write(int(RServo_pwm_value));
  //analogWrite(LServo_PINS, int(LServo_pwm_value)); 
  //analogWrite(RServo_PINS, int(RServo_pwm_value)); 
  }
  
void apply_eingine_control(){
  l_engine.writeMicroseconds(int(LEngine_pwm_value));
  r_engine.writeMicroseconds(int(REngine_pwm_value));
  //analogWrite(LEngine_PINS, int(LEngine_pwm_value)); 
  //analogWrite(REngine_PINS, int(REngine_pwm_value)); 
  }

void loop() {
    // Print latest valid values from all channels
    float pre_time = micros();
    //handleSerial();
    rc_ppm_read_values();
    compute_control();
    apply_control();
    
    Serial.print(pitch);
    Serial.print(" ");
    
    
    Serial.print(REngine_pwm_value);
    Serial.print(" ");
    
    Serial.print(LServo_pwm_value);
    Serial.print(" ");
    
    
    Serial.print(RServo_pwm_value);
    Serial.print(" ");
    
    Serial.print(micros()-pre_time);
    Serial.println();
    //delay(5);
}
