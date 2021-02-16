/*
* RC Servo Motor - ECE 403 Week 5 Lab
*
* Team 1 - Richard Ligmanowski & Kristiana Gerxhi
*/
#include <Servo.h>
#define pwm_pin 2
#define volt_pin A0

Servo servo_test;
int start_pos = 0;
//int stop_pos = 170;
unsigned int prevUpdateMillis = 0; // initialize sample counter
unsigned int prevRotateMillis = 0; // initialize rotation counter
unsigned int update_time = 10;
unsigned int wait_time = 2000;
const int updates = 180;
int voltages[updates];
const int positions[updates] =
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
1,2,3,4,5,7,9,11,13,15,18,21,24,27,31,34,38,
43,47,51,55,60,64,68,72,77,81,85,89,94,98,102,
106,111,115,119,123,128,132,136,139,143,146,149,
152,155,157,159,161,163,165,166,167,168,169,170,
170,170,170,170,170,170,170,170,170,170,170,170,
170,170,170,170,170,170,170,170,170,170,170,170,
169,168,167,166,165,163,161,159,157,155,152,149,
146,143,139,136,132,128,123,119,115,111,106,102,
98,94,89,85,81,77,72,68,64,60,55,51,47,43,38,34,
31,27,24,21,18,15,13,11,9,7,5,4,3,2,1,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

//unsigned int times[updates];
//int update_nums[updates];
int update_counter = 0;
bool read_enable = true;
bool rotate_init = true;
bool rotate_wait = false;
bool print_enable = false;


void setup() {
   servo_test.attach(pwm_pin, 500, 2500);
   Serial.begin(9600);
}


void loop() {

    unsigned int currUpdateMillis = millis();
    unsigned int currRotateMillis = millis();
  
   if (rotate_init)
   {
     servo_test.write(start_pos);
     rotate_init = false;
     rotate_wait = true;
   }
   if ((rotate_wait) && (currRotateMillis - prevRotateMillis >= wait_time))
   {
     read_enable = true;
     rotate_wait = false;
   }
   if ((read_enable) && (currUpdateMillis - prevUpdateMillis >= update_time))
   {
     //times[update_counter] = millis();
     //update_nums[update_counter] = update_counter;
     voltages[update_counter] = analogRead(volt_pin);
     servo_test.write(positions[update_counter]);
     update_counter++;
     prevUpdateMillis = millis();
     if (update_counter >= updates)
     {
       read_enable = false;
       print_enable = true;
     }
   }
   if (print_enable)
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
