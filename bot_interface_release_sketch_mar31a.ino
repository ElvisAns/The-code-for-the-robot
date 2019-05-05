#include <Timer.h> //our time base
#include <Arduino.h> //to manupulate pins
#include <TM1637Display.h> //the display library
#include <LM35.h> //the allione lib for get temperature value
#include <EEPROM.h> //to save tempory data
#include <Keypad.h> //well used to manage the matrix keypad
#include <Servo.h> //Used to command servos


#define CLK1 60 //Display time left side
#define DIO1 61
#define CLK2 4 //Display time right side
#define DIO2 5
#define CLK3 6 //Display time in front
#define DIO3 7
#define CLK4 8 //Display time  behind
#define DIO4 9
#define CLK5 10 //Display time clock
#define DIO5 11
#define CLK6 12 //Display temperature
#define DIO6 13

//Pin for changing Delay time, coming from traffic detectors like raspberry pie and tensorflow

#define Road_state_normal 14  //delay time is 1min
#define Road_state_traffic 15 //delay time is 3min
#define Road_state_free 16 //all cars are allowed

//Servo motors for robot motorization

#define servo_hand_left 17
#define servo_hand_right 18
#define servo_head 19
#define servo_contour 20
#define servo_body 21


#define light_switch 22 //This pin will use a relay to switch on lamps when the time is between 6pm and 6am
#define temperature_sensor 0  //A0 pin ,This provide temperature value in analog voltage
#define alarmm_out 24 //This pin will be high once it is 12:00
#define sound_detective 3 //Interruption on 1 when This pin is high the system pause and stop all ways it means there is a urgence like ambulance

//TRaffic light in Green-Yellow-Red format Per Road (4road A,B,C,D)

#define TR_A_G 26
#define TR_A_Y 27
#define TR_A_R 28
#define TR_B_G 29
#define TR_B_Y 30
#define TR_B_R 31
#define TR_C_G 32
#define TR_C_Y 33
#define TR_C_R 34
#define TR_D_G 35
#define TR_D_Y 36
#define TR_D_R 37


//led output or any thing else that can light up and placed in a box either show a red blocking light or a green one showing a pass sign
#define PASS_BOX_LEFT 38
#define STOP_BOX_LEFT 39
#define PASS_BOX_RIGHT 40
#define STOP_BOX_RIGHT 41
#define PASS_BOX_inFRONT 42
#define STOP_BOX_inFRONT 43
#define PASS_BOX_BEHIND 44
#define STOP_BOX_BEHIND 45

#define button_setup 2 //which is 0eme interrupt source button for starting time setup


const uint8_t STOP[] = { //Dispaly STOP on LED Display
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,  // S
  SEG_D | SEG_E | SEG_F | SEG_G, //T
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,   // P
};

const uint8_t PASS[] = { //Display Pass on LED Display
  SEG_A | SEG_B | SEG_E | SEG_F | SEG_G,   // P
  SEG_A | SEG_B | SEG_C | SEG_E | SEG_F | SEG_G,   // A
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,  // S
  SEG_A | SEG_C | SEG_D | SEG_F | SEG_G,  // S
};

const uint8_t deg[] = { //Display Degree  Celicius symbol on one digit-led
  SEG_A | SEG_B | SEG_F | SEG_G ,//deg

  SEG_A | SEG_D | SEG_E | SEG_F //c

};



uint8_t data[] = { 0xff, 0xff, 0xff, 0xff }; //variable to store data for timer display
uint8_t data_time[] = { 0xff, 0xff, 0xff, 0xff }; //variable to store data for time display

unsigned int temperature; //store the converted temperature

int toggle = LOW; //this variable will change when a 500ms elapse


//create instance that handle displayers
TM1637Display display_left(CLK1, DIO1);
TM1637Display display_right(CLK2, DIO2);
TM1637Display display_behind(CLK3, DIO3);
TM1637Display display_inFront(CLK4, DIO4);
TM1637Display display_clock(CLK5, DIO5);
TM1637Display display_temperature(CLK6, DIO6);

const byte _ROWS = 4; //four rows
const byte _COLS = 3; //three columns
char keys[_ROWS][_COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'S', '0', 'B'} //S for save and B for delete previous character
};
byte rowPins[_ROWS] = {46, 47, 48, 49}; //connect to the row pinouts of the keypad
byte colPins[_COLS] = {50, 51, 52}; //connect to the column pinouts of the keypad

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, _ROWS, _COLS );

int timeConv = 0; //This variable the time in 1245 format for 12:45
int timerConv = 0; //This variable store the the countdown value in 1356 format 55:54 minutes & seconds

bool inv_display = false;

String raw_time = "";
volatile int dot_timers = 0b00000000, dot_clock = 0b00000000;
volatile int sec_clk, min_clk = 0, hr_clk = 0; //values displayed on the clock
volatile int sec_t = 0, min_t = 0, hr_t = 0; //the value displayed on the timers
volatile bool setting = false, timer_stop = false;
volatile int clock_event, timers_event;

volatile int x = 0, y = 0; //for help to count half second

Timer t;//the instance for the timer

LM35 sensor(temperature_sensor);


void setup() {

  for (int i = 0; i < 62; i++)
    pinMode(i, OUTPUT); //Declare 62pins as output
  pinMode(2, INPUT_PULLUP);
  clock_event = t.every(500, increment_second_for_clock) ;
  timers_event = t.every(500, increment_second_for_timers) ;
//*****************************
//You must attach servo code here myServo.attach(pin);
//  *****************************
  attachInterrupt(digitalPinToInterrupt(button_setup), time_setup, FALLING); //Pressing a button make the pin goes low
  attachInterrupt(digitalPinToInterrupt(sound_detective), emergency, CHANGE); //once the sound is still hearible we have a high value

  display_left.setBrightness(0x0f);

  display_right.setBrightness(0x0f);

  display_behind.setBrightness(0x0f);

  display_inFront.setBrightness(0x0f);

  display_clock.setBrightness(0x0f);

  display_temperature.setBrightness(0x0f);

}
void loop() {
  temperature = sensor.getTemp();
  t.update();
  if (setting) { //setting mode by the keypad
    char key = keypad.getKey();
    if (key) {
      int x;
      x++; //count how many digit are now writen
      if (x < 5) { //prevent the user to only enter 4digit if more do nothing
        switch (key) {
          case 'S':;;
            break;
          case 'B':
            raw_time = String(timeConv);
            raw_time.remove(raw_time.length()-1, 1);
            timeConv = raw_time.toInt();
            x--;
            break;
          default:
            raw_time = String(timeConv);
            raw_time += key;
            timeConv = raw_time.toInt();
            break;
        }

       
      }
    }

  }
  else {
    timeConv = hr_clk * 100 + min_clk; //store the time in 1256 format, the raw format of 12:56

  }



  display_clock.showNumberDecEx(timeConv,0b01000000, true, 4, 0);

  display_temperature.showNumberDec(temperature, true , 2, 0);

  display_temperature.setSegments(deg, 2, 2);

  timerConv = min_t * 100 + sec_t;
  display_behind.showNumberDecEx(timerConv, dot_timers, true, 4, 0);
  display_left.showNumberDecEx(timerConv, dot_timers, true, 4, 0);
  display_inFront.showNumberDecEx(timerConv, dot_timers, true, 4, 0);
  display_right.showNumberDecEx(timerConv, dot_timers, true, 4, 0);

/*
 * This timeConv<10 mean we delay only for 10sec and change state...Remember this value will be changed according to state of the road
 * and this checking is perfomed once one step in finish in the motion block flow
* The next task is to study how to implement the traffic reading and write values to the traffic light according to the value of timeConv
* also make change to decremment in timer displayer not increment
* ####################################################################By ELVIS 31/03/2019 02:41
 */

 
  if (timerConv < 100 & !timer_stop) {

    if (inv_display) { //right and left can pass


      digitalWrite(PASS_BOX_inFRONT, LOW);
      digitalWrite(PASS_BOX_BEHIND, LOW);
      digitalWrite(PASS_BOX_RIGHT, HIGH);
      digitalWrite(PASS_BOX_LEFT, HIGH);

      digitalWrite(STOP_BOX_BEHIND, !digitalRead(PASS_BOX_BEHIND)); //MAke sure pass and stop never occur twice
      digitalWrite(STOP_BOX_inFRONT, !digitalRead(PASS_BOX_inFRONT));
      digitalWrite(STOP_BOX_RIGHT, !digitalRead(PASS_BOX_RIGHT));
      digitalWrite(STOP_BOX_LEFT, !digitalRead(PASS_BOX_LEFT));
    }
    else { //right and left become blocked

      digitalWrite(PASS_BOX_inFRONT, HIGH);
      digitalWrite(PASS_BOX_BEHIND, HIGH);
      digitalWrite(PASS_BOX_RIGHT, LOW);
      digitalWrite(PASS_BOX_LEFT, LOW);

      digitalWrite(STOP_BOX_BEHIND, !digitalRead(PASS_BOX_BEHIND)); //MAke sure pass and stop never occur twice
      digitalWrite(STOP_BOX_inFRONT, !digitalRead(PASS_BOX_inFRONT));
      digitalWrite(STOP_BOX_RIGHT, !digitalRead(PASS_BOX_RIGHT));
      digitalWrite(STOP_BOX_LEFT, !digitalRead(PASS_BOX_LEFT));
    }
  }

  else { //transitional function for motor to turn the robot, this occur when 3min are elapsed
    
/*Put here all motion steps
 * **************************
 * ************************
 * By ELvis
 */


    inv_display = !inv_display; //
    hr_t = 0;
    min_t = 0;
    sec_t = 0;
  }

}


void increment_second_for_clock() {

  x++;
if(x==1)
  dot_clock = 0b00000000;

 else {

    sec_clk++;
    dot_clock = 0b01000000;
    x = 1;
  }

  if (sec_clk > 59)
  {
    sec_clk = 0;
    min_clk++;
    if (min_clk > 59)
    {
      min_clk = 0;
      hr_clk++;
      if (hr_clk > 23)
        hr_clk = 0;
    }
  }
}

void increment_second_for_timers() {

  y++;
  if(y==1)
  dot_timers = 0b00000000;
  else {
    dot_timers = 0b01000000;
    y = 1;
    sec_t++;
  }
  
  if (sec_t > 59)
  {
    sec_t = 0;
    min_t++;
    if (min_t > 59)
    {
      min_t = 0;
      hr_t++;
      if (hr_t > 23)
        hr_t = 0;
    }
  }
}

void time_setup() {
  setting = !setting;
  if (setting)
    t.stop(clock_event);
  else{
    if(raw_time.length()<=2){
       min_clk=raw_time.toInt()%60;
       hr_clk=0;
       }

       else if(raw_time.length()==3){
        String tmp1,tmp2;
        tmp1=raw_time.substring(1,3);
        min_clk=(tmp1).toInt()%60;
        tmp2=raw_time.charAt(0);
        hr_clk=(tmp2).toInt()%24;
       }
       

       else {
        String tmp3,tmp4;
        tmp3=raw_time.substring(2,4);
        min_clk=(tmp3).toInt()%60;
        tmp4=raw_time.substring(0,2);
        hr_clk=(tmp4).toInt()%24;
       }
    clock_event = t.every(500, increment_second_for_clock) ;
  }
}

void emergency() {
  timer_stop = !timer_stop;
  if (timer_stop)
    t.stop(timers_event);
  else
    timers_event = t.every(500, increment_second_for_timers);

  digitalWrite(STOP_BOX_BEHIND, HIGH); //make sure all ways are stopped
  digitalWrite(STOP_BOX_inFRONT, HIGH);
  digitalWrite(STOP_BOX_RIGHT, HIGH);
  digitalWrite(STOP_BOX_LEFT, HIGH);

  digitalWrite(PASS_BOX_BEHIND, !digitalRead(STOP_BOX_BEHIND)); //MAke sure pass and stop never occur twice
  digitalWrite(PASS_BOX_inFRONT, !digitalRead(STOP_BOX_inFRONT));
  digitalWrite(PASS_BOX_RIGHT, !digitalRead(STOP_BOX_RIGHT));
  digitalWrite(PASS_BOX_LEFT, !digitalRead(STOP_BOX_LEFT));
}
