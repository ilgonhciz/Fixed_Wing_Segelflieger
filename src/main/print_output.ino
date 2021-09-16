float pre_time = micros();
void print_all(){
    if (micros()-pre_time > 25000){
         print_rc_input();
         print_rc_output();
         Serial.println();
         pre_time = micros();
      }
  }


void print_rc_input(){
    Serial.print("T: ");
    Serial.print(throttle);
    Serial.print(" ");
    
    Serial.print("P: ");
    Serial.print(pitch);
    Serial.print(" ");

    Serial.print("R: ");
    Serial.print(roll);
    Serial.print(" ");
    
    Serial.print("Y: ");
    Serial.print(yaw);
    Serial.print(" ");

    Serial.print("A: ");
    Serial.print(arm);
    Serial.print(" ");

    Serial.print("M: ");
    Serial.print(flightmode);
    Serial.print(" ");
  }

void print_rc_output(){
    Serial.print("LEn: ");
    Serial.print(LEngine_pwm_value);
    Serial.print(" ");
    
    Serial.print("REn: ");
    Serial.print(REngine_pwm_value);
    Serial.print(" ");

    Serial.print("LSe: ");
    Serial.print(LServo_pwm_value);
    Serial.print(" ");
    
    Serial.print("RSe: ");
    Serial.print(RServo_pwm_value);
    Serial.print(" ");
  }
