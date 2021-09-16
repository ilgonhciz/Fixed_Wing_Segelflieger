 #include <PPMReader.h>
#include <Servo.h>


// Initialize a PPMReader on digital pin 3 with 6 expected channels.
byte interruptPin = 3;
byte channelAmount = 6;
PPMReader ppm(interruptPin, channelAmount);
int channel_value_list[7];

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
int arm = 1000;
int flightmode=1000;
boolean do_calibration = false;
boolean armed = false;

int armed_pwm_value = 1050;
int current_pwm_value = 1000;

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

    if(do_calibration){
        calibrate_esc();
      }
    
    l_engine.writeMicroseconds(int(1000));
    r_engine.writeMicroseconds(int(1000));
    //analogWrite(LEngine_PINS, 127); 
    //analogWrite(REngine_PINS, 128); 

    //analogWrite(LServo_PINS, 128); 
    //analogWrite(RServo_PINS, 128); 
}


void calibrate_esc(){
    l_engine.writeMicroseconds(int(2000));
    r_engine.writeMicroseconds(int(2000));
    delay(6000);
    l_engine.writeMicroseconds(int(1000));
    r_engine.writeMicroseconds(int(1000));
    delay(4000);
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
  if (flightmode<1333){
      LEngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) + (normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(roll)*abs(normalizePRY(roll));// +(normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(yaw)*abs(normalizePRY(yaw));
      REngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) - (normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(roll)*abs(normalizePRY(roll));// -(normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(yaw)*abs(normalizePRY(yaw));
    }
  else{
    if (flightmode<1666){
        LEngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) + (0.2)*1000*normalizePRY(roll)*abs(normalizePRY(roll));// +(normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(yaw)*abs(normalizePRY(yaw));
        REngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) - (0.2)*1000*normalizePRY(roll)*abs(normalizePRY(roll));// -(normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(yaw)*abs(normalizePRY(yaw));

      }
    else{
        LEngine_pwm_value=LEngine_pwm_value;
        REngine_pwm_value=REngine_pwm_value;
      }
    }
  //LEngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) + (normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(roll)*abs(normalizePRY(roll));// +(normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(yaw)*abs(normalizePRY(yaw));
  //REngine_pwm_value = 1000 + 1000 * normalizeThrottle(throttle) - (normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(roll)*abs(normalizePRY(roll));// -(normalizeThrottle(throttle)/5+0.2)*1000*normalizePRY(yaw)*abs(normalizePRY(yaw));
  if(arm<1500){
      LEngine_pwm_value=1000;
      REngine_pwm_value=1000;
      armed = false;
    }
  else{
      start_engine_or_hold_speed();
      armed = true;
    }
  }

void start_engine_or_hold_speed(){
    if(armed){
      if(LEngine_pwm_value<armed_pwm_value){
          LEngine_pwm_value=armed_pwm_value;
        }
      if(REngine_pwm_value<armed_pwm_value){
          REngine_pwm_value=armed_pwm_value;
        }
      }
    else{
          while(current_pwm_value <= armed_pwm_value){
              current_pwm_value += 10;
              REngine_pwm_value=current_pwm_value;
              LEngine_pwm_value=current_pwm_value;
              delay(250);
            } 
        } 
  }


void compute_servo_control(){
  int effective_servo_range = 60;
  if(abs(pitch-1500)<25){
      pitch=1500;
    }
  if(abs(roll-1500)<25){
      roll=1500;
    }
  if(abs(yaw-1500)<25){
      yaw=1500;
    }
  if (flightmode<1333){
     LServo_pwm_value =( 90 + 1*(effective_servo_range*normalizePRY(pitch)+effective_servo_range*normalizePRY(yaw)-effective_servo_range*normalizePRY(roll))); 
     RServo_pwm_value =( 90 + -1*(effective_servo_range*normalizePRY(pitch)-effective_servo_range*normalizePRY(yaw)+effective_servo_range*normalizePRY(roll)));
     }
  else{
    if (flightmode<1666){
        LServo_pwm_value =( 90 + 1*(effective_servo_range*normalizePRY(pitch)*abs(normalizePRY(pitch))+effective_servo_range*normalizePRY(yaw)*abs(normalizePRY(yaw))-effective_servo_range*normalizePRY(roll)*abs(normalizePRY(roll)))); 
        RServo_pwm_value =( 90 + -1*(effective_servo_range*normalizePRY(pitch)*abs(normalizePRY(pitch))-effective_servo_range*normalizePRY(yaw)*abs(normalizePRY(yaw))+effective_servo_range*normalizePRY(roll)*abs(normalizePRY(roll))));
      }
    else{
        LServo_pwm_value =( 90 + normalizeThrottle(throttle)*(effective_servo_range*normalizePRY(pitch)*abs(normalizePRY(pitch))+effective_servo_range*normalizePRY(yaw)*abs(normalizePRY(yaw))-effective_servo_range*normalizePRY(roll)*abs(normalizePRY(roll)))); 
        RServo_pwm_value =( 90 + -normalizeThrottle(throttle)*(effective_servo_range*normalizePRY(pitch)*abs(normalizePRY(pitch))-effective_servo_range*normalizePRY(yaw)*abs(normalizePRY(yaw))+effective_servo_range*normalizePRY(roll)*abs(normalizePRY(roll))));
      }
    }
  
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
    //handleSerial();
    rc_ppm_read_values();
    compute_control();
    apply_control();
    print_all();
    
    
    //delay(5);
}
