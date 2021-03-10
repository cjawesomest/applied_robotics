#include <Servo.h>

#define encoder_press_pin 20
#define encoder_read_pin 2
#define encoder_clock_pin 3
#define pwm_pin_1 6
#define pwm_pin_2 7
#define volt_pin A0

//For Interrupts of encoder
volatile boolean TurnDetected;  // need volatile for Interrupts
volatile boolean rotationdirection;  // CW or CCW rotation
volatile boolean joint_select = true;

int encoder_position=0;    // To store Stepper Motor Position
int prev_pos=0;
int joint_encode=0;
int start_position_enc=0;

int last_position;     // Previous Rotary position Value to check accuracy
//int StepsToTake;      // How much to move Stepper

Servo servo_test_1;
Servo servo_test_2;

//int stop_pos = 170;
unsigned int prevUpdateMillis = 0; // initialize sample counter
unsigned int prevRotateMillis = 0; // initialize rotation counter
unsigned int update_time = 200;
unsigned int wait_time = 2000;
const int updates = 180;
int step_num = 0;
const int taught_updates = 10;
int voltages[updates];
int positions_taught_1[taught_updates] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const int positions_1[updates] =
{67, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 69, 69, 69, 69, 70, 70, 70, 71, 71, 71, 72, 72, 72, 73, 73, 74, 74, 75, 75, 76, 76, 77, 77, 78, 78, 79, 80, 80, 81, 82, 82, 83, 83, 84, 85, 
86, 86, 87, 88, 89, 89, 90, 90, 91, 91, 92, 93, 94, 95, 96, 96, 97, 98, 99, 100, 100, 101, 102, 103, 103, 104, 105, 106, 106, 107, 108, 109, 109, 110, 111, 111, 112, 113, 113, 114, 115, 115, 116, 117, 117, 118, 118, 119, 120, 120, 121, 121, 122, 122, 123, 123, 124, 124, 125, 125, 126, 126, 126, 127, 127, 128, 128, 128, 129, 129, 129, 130, 130, 130, 130, 131, 131, 131, 131, 131, 131, 
132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 132, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131, 131};
int start_pos_1 = 0; //positions_1[0];
int positions_taught_2[taught_updates] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const int positions_2[updates] =
{31, 31, 31, 31, 31, 31, 31, 31, 31, 30, 30, 30, 30, 30, 29, 29, 29, 29, 28, 28, 28, 27, 27, 26, 26, 26, 25, 25, 24, 24, 24, 23, 23, 22, 22, 21, 21, 20, 20, 19, 19, 18, 18, 17, 17, 16, 16, 15, 
15, 14, 14, 13, 13, 12, 12, 11, 11, 10, 10, 10, 9, 9, 8, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 14, 14, 15, 15, 16, 17, 17, 18, 18, 19, 20, 20, 21, 22, 22, 23, 23, 24, 25, 25, 26, 26, 27, 28, 28, 29, 29, 30, 31, 31, 32, 32, 33, 33, 34, 34, 35, 35, 36, 36, 37, 37, 37, 38, 38, 38, 39, 39, 39, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 41};
int start_pos_2 = 0;//positions_2[0];

//unsigned int times[updates];
//int update_nums[updates];
int update_counter = 0;
bool read_enable = true;
bool rotate_init = true;
bool rotate_wait = false;
bool print_enable = true;
bool profile_mode = false;
bool done_teaching = false;


void setup() {
  pinMode(encoder_clock_pin,INPUT);
  pinMode(encoder_read_pin,INPUT);  
  pinMode(encoder_press_pin,INPUT);
  digitalWrite(encoder_press_pin, HIGH); // Pull-Up for switch
  attachInterrupt (digitalPinToInterrupt(encoder_read_pin), encoder_isr, FALLING);
  attachInterrupt (digitalPinToInterrupt(encoder_press_pin), teach_isr, FALLING);  
  
  servo_test_1.attach(pwm_pin_1, 500, 2500);
  servo_test_2.attach(pwm_pin_2, 500, 2500);
  Serial.begin(9600);
}


void loop() {

  unsigned int currUpdateMillis = millis();
  unsigned int currRotateMillis = millis();

  //Joint code
   if (rotate_init)
   {
     servo_test_1.write(start_pos_1);
     servo_test_2.write(start_pos_2);
     rotate_init = false;
     rotate_wait = true;
   }
   if ((rotate_wait) && (currRotateMillis - prevRotateMillis >= wait_time))
   {
     read_enable = true;
     rotate_wait = false;
   }
   if ((read_enable) && (done_teaching) && (currUpdateMillis - prevUpdateMillis >= update_time))
   {
     //times[update_counter] = millis();
     //update_nums[update_counter] = update_counter;
     voltages[update_counter] = analogRead(volt_pin);
     servo_test_1.write(positions_taught_1[update_counter]);
     servo_test_2.write(positions_taught_2[update_counter]);
     update_counter++;
     prevUpdateMillis = millis();
     if (update_counter >= taught_updates)
     {
       read_enable = false;
       print_enable = true;
     }
   }

  //Encoder code
  if (!(digitalRead(encoder_press_pin))) {   // check if button is pressed
    if(joint_select && (encoder_position == positions_taught_1[min(0, step_num-1)])){
    }
    else if((!joint_select ) && (encoder_position == positions_taught_1[min(0, step_num-1)])){
    }
    else {
//        Serial.print("Encoder Pulses: ");
//        Serial.println(encoder_position-start_position_enc);
//        
//        if(joint_select){
//          joint_select = false;
//          if(!(done_teaching)){
//            Serial.print("Wrote Joint 1 [");
//            Serial.print(step_num);
//            Serial.print("]:");
//            Serial.println(encoder_position);
//            positions_taught_1[step_num]=encoder_position;
//          }
//        }
//        else{
//          joint_select = true;
//          step_num = step_num+1;
//          if(!(done_teaching)){
//            Serial.print("Wrote Joint 2 [");
//            Serial.print(step_num);
//            Serial.print("]:");
//            Serial.println(encoder_position);
//            positions_taught_2[step_num]=encoder_position;
//          }
//        }
//        if (step_num <= 0){
//          prev_pos = encoder_position;
//          encoder_position=0;
//        }
//        else{
//          if(joint_select){
//            encoder_position = positions_taught_1[step_num-1];
//          }
//          else{
//            encoder_position = positions_taught_2[step_num-1];
//          }
//        }
//         // Reset position to previous step
//        if (step_num >= taught_updates){
//          Serial.println("Teaching Complete! Time for playback!");
//          done_teaching=true;
//        }
      }
  }

  // Runs if rotation was detected
  if (TurnDetected)  {
    last_position = encoder_position; // Save previous position in variable
    if (rotationdirection) {
      encoder_position=encoder_position-1;
      if (joint_encode > 0){
        joint_encode=joint_encode-1;
      }
    }
    else {
      encoder_position=encoder_position+1;
      if (joint_encode < 180){
        joint_encode=joint_encode+1;
      }
    }
//    if (print_enable) {
//      Serial.print("Encoder Position: ");
//      Serial.println(encoder_position);
//    }
    if(joint_select){
      servo_test_1.write(encoder_position);
    }
    else{
      servo_test_2.write(encoder_position);
    }

    TurnDetected = false;  // do NOT repeat IF loop until new rotation detected
  }

   //Print Code
   if (print_enable && profile_mode)
   {
       Serial.println("[");
       for (int i = 0; i < updates; i++)
       {
           //Serial.print(update_nums[i]);
           //Serial.print(",");
           //Serial.print(times[i]);
           //Serial.print(",");
           //Serial.print(positions[i]);
           //Serial.print(",");
           Serial.print(voltages[i]);
           Serial.print(", ");
       }
       Serial.println("]");
       print_enable = false;
   }
}

//Interrupts
void encoder_isr ()  {
  delay(4);  // delay for Debouncing
  if (digitalRead(encoder_clock_pin))
    rotationdirection= digitalRead(encoder_read_pin);
  else
    rotationdirection= !digitalRead(encoder_read_pin);
  TurnDetected = true;

}

void teach_isr () {
  delay(4);  // delay for Debouncing
  if(joint_select){
    joint_select = false;
    if(!(done_teaching)){
      Serial.print("Wrote Joint 1 [");
      Serial.print(step_num);
      Serial.print("]:");
      Serial.println(encoder_position);
      positions_taught_1[step_num]=encoder_position;
    }
  }
  else{
    joint_select = true;
    if(!(done_teaching)){
      Serial.print("Wrote Joint 2 [");
      Serial.print(step_num);
      Serial.print("]:");
      Serial.println(encoder_position);
      positions_taught_2[step_num]=encoder_position;
    }
    step_num = step_num+1;
  }
  if (step_num <= 0){
    prev_pos = encoder_position;
    encoder_position=0;
  }
  else{
    if(joint_select){
      encoder_position = positions_taught_1[step_num-1];
    }
    else{
      encoder_position = positions_taught_2[step_num-1];
    }
  }
   // Reset position to previous step
  if (step_num >= taught_updates){
    Serial.println("Teaching Complete! Time for playback!");
    done_teaching=true;
  }
}
