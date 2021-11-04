/* 
 * Copyright (c) 2018 Ali Saghiran.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Canbus.h"   // don't forget to include these
#include "defaults.h" // Sparkfun Can shield libraries
#include "global.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"

#include "TimerOne.h" // Timer library (easier to use)

#include <inttypes.h> // byte types , clean 
#include <avr/io.h>   
#include <avr/sleep.h>// sleep mode library
#include <SoftwareSerial.h>


//Major Macros
#define BAT   0x10
#define MTR   0x20
#define CSL   0x08    //40?

#define SIZE   30
#define REVERSEMODE 0     // To make wheels turn in reverse (maybe useful for something other than a bike)

/* Define Joystick connection pins */
#define UP     A1
#define DOWN   A3
#define LEFT   A2
#define RIGHT  A5
#define CLICK  A4

#define K_ADAPT   4
#define POS_CORR  25
#define NEG_CORR  15
#define UPD_PRD 5

//Volatile global variables
volatile uint8_t Flag_period = 0;
volatile uint8_t Flag_update = UPD_PRD;
volatile uint8_t Flag_startstate = 0;

volatile uint32_t table[SIZE] = {0};
volatile uint32_t* ptr = table;
volatile uint32_t mean_speed = 0;
volatile uint32_t mean_troque = 0 ;
volatile uint8_t  assis_level = 0;
volatile double old_th = 0;
volatile double new_th = 0;

volatile tCAN toto;
volatile tCAN* received_msg = &toto;


//Global Variables
int j = 0;
char state = 0;
char sleep = 0;
uint32_t spdalg = 0;                      // "Speed" reading from POT
uint32_t spdref = 0;                      // Speed Reference from 0 to 55 km/h
uint32_t spdreq = 0;                      // Speed Required to motor
uint32_t spdcrt = 0;                      // Speed Correction Factor

uint32_t spdtst = 0;                      // Speed math test 
uint32_t spdout = 0;                      // Speed output to motor
uint32_t mtrspd = 0;                      // Speed from motor
uint32_t spddis = 0;                      // Speed Display in Human Readable Format
uint32_t mtrtrq = 0;                      // Torque from motor
int batvlt = 0;                           // General value for a fully charged battery
boolean strt = false;                     // Check for irregular cycle if its in the start up or not
int irrlp = 0;                            // Irregular CAN Loop every 3 cycles

int edge = 7;
int much_r = 0;
int much_l = 0;
int x = 0;
int t = 0;

SoftwareSerial LCD(3,6); // pin 6 = TX, pin 3 = RX (unused)

void add_to_table(uint32_t val){
  *ptr = val;
  ptr = ptr + 1;
  if (ptr > &table[SIZE-1]) ptr = table;
}

uint16_t mean_spd_simple(void){
  long int sum = 0;
  float m = 0;
  int i;
  for (i=0; i<SIZE; i++){
    sum += (table[i]&0x0000FFFF);
  }
  m = sum/30;
  //printf(" s_mean %X \t", (int)m);
  return m;
}

uint16_t mean_trq_simple(void){
  long int sum = 0;
  float m = 0;
  int i;
  for (i=0; i<SIZE; i++){
    sum += ((table[i]&0xFFFF0000)>>16);
  }
  m = sum/30;
  //printf(" t_mean %X \n", (int)m);
  return m;
}

// LCD Configuration functions =================================================
void clearScreen(){
  // Clear LCD screen
  LCD.write(0xFE); // move cursor to beginning of first line
  LCD.write(0x01);
}

void selectLineOne(){ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(128);  //position
}
  
void selectLineTwo(){ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(192); //position
}

void lcdSpeedUpd(int spd){
  clearScreen();
  if (spd>99)  LCD.print("Errspd");
  else{
    if (spd<10) LCD.print("0");
    LCD.print(spd);
    LCD.print("km/h");
    LCD.print("#");
    LCD.print(mean_troque);
    LCD.print(" ");
    LCD.print(spdout);
  }
}

void lcdBatUpd(int level){
  double step = 3.5;                //  Max0xDC - Min0xB5 divided by number of levels
  int i=0;
  selectLineTwo();
  int n = ((level)/step);
  LCD.print(level);
  LCD.print(">");
    //}
}

void lcdAssLevel(int ass){
  LCD.write(254);LCD.write(142);

  if(ass>9 || ass<0) LCD.print("ER");
  else{
    if (ass<5) {
      LCD.print("G");
      LCD.print(5-ass);
    }
    else if (ass>5){
      LCD.print("A");
      LCD.print(ass-5);
    }
    else LCD.print("00");
  }
}

void lcdAssStatus(int spdout, int ass){
  int step = 10;
  int i = 0;
  LCD.write(254);LCD.write(200);
  if (spdout>0x45){           // 0x45 is the highest value given by the battery
    LCD.print("<ErrAss>");
  }
  else if (ass<5){
    LCD.print("<rechrg>");
  }
  else if (ass>=5){
    LCD.print("<");
    int n = (spdout/step);
    for(i=0;i<n;i++){
      LCD.print("#");
    }
    while(i<6){
      LCD.print(" ");
      i++;
    }
    LCD.print(">");
  }
}

void lcdUpdateData(){
  lcdSpeedUpd(spddis);
  lcdBatUpd(batvlt);
  lcdAssLevel(assis_level);
  lcdAssStatus(spdout,assis_level);
}
//==============================================================================

  
void printMessage(tCAN message){
  if (&message == NULL) return;
  Serial.print("ID: ");
  Serial.print(message.id,HEX);
  Serial.print(", ");
  Serial.print("Data: ");
  for(int i=0;i<message.header.length;i++){
    Serial.print(message.data[i],HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

uint8_t receiveFrame(tCAN* message_ptr){
  // interrupt indicates when message is available (inside check message)
  while (mcp2515_check_message()) { 
    if (mcp2515_get_message(message_ptr)){
      //Serial.print("   >Got : ");
      if (Flag_startstate == 0) Flag_startstate = 1;
      //printMessage(*message_ptr);
      updateDataVariables(*message_ptr);

      //Serial.print(mtrspd,HEX); 
      //Serial.print(" : ");                         
      //Serial.println(mtrtrq ,HEX);  
    }
    delay(2);
  }
}

uint8_t flushBuffer(tCAN* message_ptr){
  message_ptr->id = 0;
  message_ptr->header.rtr = 1;
  message_ptr->header.length = 4;
  for(int i=0;i<8;i++){
    message_ptr->data[i]= 0;
  }
}

void updateDataVariables(tCAN msg){
  /*if (msg.id == 0x8 && msg.data[1] == 0x11){    // Motor Speed
    mtrspd = msg.data[3];
    pedinput ();     
    //if (mtrspd > spdref) {                            // Speed compensation
    // spdcrt --;
    //  }
    //if (mtrspd < spdref) {
    //spdcrt ++;
    //}
  spddis = map(mtrspd, 0, 47, 0, 55);               // motor speed to KM/h equvilents
  }
  else if (msg.id == 0x8 && msg.data[1] == 0x14){    // Torque/Force Applied to wheel
    mtrtrq = msg.data[3];
    uint32_t val = (mtrtrq<<16)+mtrspd;
    
    add_to_table((((uint32_t)mtrtrq)<<16)+mtrspd);
    mean_speed = mean_spd_simple();
    mean_troque = mean_trq_simple();
  }*/
  if (msg.id == 0x8 && msg.data[1] == 0x11){
      mtrspd = msg.data[3];
      spddis = map(mtrspd, 0, 47, 0, 55);               // motor speed to KM/h equvilents
  }
  else if (msg.id == 0x8 && msg.data[1] == 0x21){
      mtrtrq = msg.data[3];
      uint32_t val = (mtrtrq<<16)+spddis;
      add_to_table(val);
      mean_speed = mean_spd_simple();
      mean_troque = mean_trq_simple();
      pedinput();
  }
  else if (msg.id == 0x8 && msg.data[1] == 0x3B){
    batvlt = msg.data[3];
  }
}

void pedinput () {

  int cmd_step = 0x10;
  //Serial.println(spdalg);                                 // Diagnostic Serial Connection

  if (assis_level < 5){
    if (assis_level == 4) spdout = 0;
    else if (assis_level == 3)  spdout = 0xFFFB;
    else if (assis_level == 2)  spdout = 0xFFF6;
    else if (assis_level == 1)  spdout = 0xFFEC;
    else if (assis_level == 0)  spdout = 0xFFD8;
  }
  
  else {
    spdreq = 0x5;
    spdref = 0x1;

    // Assistance Algorithm
    //    if (assis_level > 8) assis_level = 9;
    new_th = ((assis_level-5)*mean_troque)/K_ADAPT;
    if (new_th>old_th) old_th = old_th + (new_th - old_th)*10/POS_CORR;
    else  old_th = (old_th - new_th)*10/NEG_CORR;
    //Serial.println("Values:");
    //Serial.println(old_th);
    //Serial.println(new_th);
    spdout = static_cast<unsigned int>(old_th);
    
    //spdout = ((assis_level-5) * cmd_step);
    //spdout = 0x00;
  }
}
 
void sendFrame(uint16_t identifier, uint8_t len, uint32_t command){
  tCAN msg ;
  uint32_t tmp = 0;
    
  receiveFrame(received_msg);
  
  if(len != 0x4 && len != 0x2){
    // or size message is different than length
    Serial.print("ERROR SENDFRAME! -");
    Serial.println(len);
    //exit(0);
  }

  msg.header.length = len;
  msg.id = identifier;
  for(int i = 0; i<len; i++){
    tmp = (uint32_t(command & (uint32_t(0xFF) << (8*i))) >> (8*i));
    msg.data[len-i-1] = uint8_t(tmp);
  }
  for(int i = len; i<8; i++){
    msg.data[i] = 0x0;
  }
  mcp2515_send_message(&msg);
  Serial.print(">>SENT  :");
  printMessage(msg);
  delay(2);

}



void initialCommands(){

  
  sendFrame(BAT, 4, 0x200000);        // Start code of the system's set of CAN Bus packages.

  sendFrame(BAT, 4, 0x210001);        // Start code of the system's set of CAN Bus packages.

  sendFrame(BAT, 2, 0x003B);
  
  sendFrame(BAT, 2, 0x003C);
  
  sendFrame(BAT, 4, 0x00220000);        // Unknown Function  
  
  sendFrame(BAT, 2, 0x003D);
  
  sendFrame(MTR, 4, 0x00020000);        // Unknown Function

  sendFrame(MTR, 4, 0x00410000);        // Unknown Function

  sendFrame(MTR, 4, 0x00420001);         // Reverse or Forward 0 = Reverse and 1 = Forward
  
  sendFrame(MTR, 2, 0x0020);             // Unknown Function 

  sendFrame(MTR, 2, 0x0011);             // Check Speed to see if already moving
}

void startCANloop (){                  // Start sequence for a series of CAN requests. Must follow the Battery voltage check.
  // Start code of the system's set of CAN Bus packages.
  sendFrame(0x58, 2, 0x009C);

  sendFrame(BAT, 2, 0x003B);

}


void regularCANmtr (){      // Regular CAN loop requests to the Motor
      
  // Currently Unknown Code, however due to the nature of the tests,  
  // assumed to be related to foot pedal power or regenerative breaking.
  sendFrame(MTR, 2, 0x0021);

  // Send the Speed desired to the wheel
  sendFrame(MTR, 4, uint32_t(0x090000+spdout)); 

  // Send the Speed desired to the wheel, repeated as BionX does
  sendFrame(MTR, 4, uint32_t(0x090000+spdout));

  // Check the Speed of the wheel
  sendFrame(MTR, 2, 0x0011);
      
  // Check the Torque Sensor of the Wheel, 14=Torque Sensor Value
  sendFrame(MTR, 2, 0x0014);

  if(REVERSEMODE) sendFrame(MTR, 4, 0x00420000);         // Reverse or Forward 0 = Reverse and 1 = Forward

}
  
void regularCANbat () {
  // Send Text to appear to request the status of the battery.
  sendFrame(BAT, 2, 0x0032);
}

void regular_cycle(){  
  startCANloop ();                       // Start of the CAN Loop
  regularCANmtr ();                      // One Standard Motor CAN Cycle  
  regularCANmtr ();                      // One Standard Motor CAN Cycle 
  regularCANmtr ();                      // One Standard Motor CAN Cycle
  regularCANmtr ();                      // One Standard Motor CAN Cycle
  regularCANbat ();                      // Ending Battery Cycle

}

void shutdown_click(){
  spdout = 0x00;
  sendFrame(MTR, 4, 0x420000);
  delay(5);
  for (int i =0;i<1;i++){
    initialCommands ();
    //Serial.println("  >... Ended sending initial commands.");
    //Serial.println("Starting regular cycle...");
    regular_cycle();
    regularCANbat();
  }
  Serial.println("... Battery shutdown");
  delay(1000);
  //sendFrame(MTR, 4, 0x410000);
  //delay(5);
  sendFrame(MTR, 4, 0x420001);
  delay(5);
  sendFrame(BAT, 4, 0x250001);
  Serial.println("Commands are sent!");
  delay(100);
  sendFrame(MTR, 4, 0x420001);
  delay(5);
  sendFrame(BAT, 4, 0x250001);
  delay(1000);
}

 
void setup(){
  Serial.begin(115200);
  LCD.begin(9600);
  
  //Initialise MCP2515 CAN controller at the specified speed
  if(Canbus.init(CANSPEED_125)) Serial.print("CAN Init ok");
  else Serial.print("Can't Init CAN");
  delay(1000);
  //Serial.println("=> end init.");

  Serial.println(">>>Starting up CAN cycle ...");
  clearScreen();
  LCD.write(">Wake up battery");
  selectLineTwo();
  LCD.write("Waitin 4 answer");
  delay(1000);

  
  pinMode(CLICK,INPUT);    //click to shutdown 
  pinMode(RIGHT,INPUT);    //move right to increase speedout value
  pinMode(LEFT,INPUT);     //move lest to decrease speedout value
  //pinMode(,INPUT);
  digitalWrite(CLICK, HIGH);
  digitalWrite(RIGHT, HIGH); 
  digitalWrite(LEFT, HIGH);

  //attachInterrupt(digitalPinToInterrupt(CLICK), shutdown_click, LOW);

  //interrupt Setup
  Timer1.initialize(50000);                  // 20Hz sampling
  Timer1.attachInterrupt(CYCLE_PERIOD_ISR);
  //Suficient value to change assistance level
  
  noInterrupts();                //Stop Interrupts

  spdalg = 0;                    // "Speed" reading from POT
  spdref = 0;                    // Speed Reference from 0 to 55 km/h
  spdreq = 0;                    // Speed Required to motor
  spdcrt = 0;                    // Speed Correction Factor
  spdout = 0;                    // Speed output to motor
  state = 0;
  assis_level = 5;               //5 means no assistance      
  delay(1000);

  while (Flag_startstate == 0){
    initialCommands ();
    //Serial.println("  >... Ended sending initial commands.");
    regularCANmtr ();              // Appears to give one motor cycle afterwards.
    regularCANbat ();              // Ending Battery Cycle
  }
  interrupts();
  Serial.println("Free Isr!! Keeping interrupts doing job ;)");
}


 
void loop() { 
  //Serial.print(assis_level);
  //Serial.print("  -  ");
  //Serial.println(spdout,HEX);
  if (digitalRead(CLICK) == 0 && sleep ==0){
    Timer1.detachInterrupt();
    delay(1000);
    shutdown_click();
    sleepNow();
  }

  if (Flag_period){
    Flag_period = 0;
    Flag_update --;
    if (Flag_update == 0){
      lcdUpdateData();
      Flag_update=UPD_PRD;
    }
    //x = millis();Serial.println(x-t);t=x;
    
    // Analogic joystik, cannot configure an interrupt on this input
    // after long push, value is higher than edge
    // -> then increase assistance level.
    if (digitalRead(RIGHT) == 0 ){
      much_r++;
      if (much_r > edge){
        if (assis_level > 8) {
          assis_level = 9;
          much_r = 0;
        }
        else {
          assis_level ++;
          much_r = 0;
        }
      }
    }
    if (digitalRead(LEFT) == 0){
      // Same with joystic left push, decrease assistance level
      much_l++;
      if (much_l > edge){
        if (assis_level < 1) {
          assis_level = 0;
          much_l=0;
        }
        else {
          assis_level --;
          much_l = 0;
        }
      }
    }

    // Regular Cycle (exceptional irregular cycle isnot used in this code)
    switch (state){
    case 0:
      startCANloop();
      regularCANmtr();
      break;
    case 1:
    case 2:
    case 3:
      regularCANmtr();
      break;
    case 4:
      regularCANmtr();
      regularCANbat();
      break;
    }
    state = ((state+1)%5);
  }
}


//Interrupt routines

void CYCLE_PERIOD_ISR(){
  //if (Flag_period == 1) Serial.println("WTF!! too slow");
  Flag_period = 1;
}

void sleepNow()         // here we put the arduino to sleep
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin

  clearScreen();
  LCD.print("Battery is off.");
  selectLineTwo();
  LCD.print(">Going to sleep");
  //  attachInterrupt(digitalPinToInterrupt (2),wake, LOW);
  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  // wakeUpNow code will not be executed
  // during normal running time. 
}

void wake(void){
    sleep_disable();         // first thing after waking from sleep:
    // detachInterrupt(digitalPinToInterrupt (2));      // disables interrupt 0 on pin 2 so the
    // disable sleep...
}

//Optional stuff
/*
  void irgCANcyc () {                     // Irregular Cycle, appears to happen every few times of a normal cycle
  msg.id = 16;                  // Apparent Sending of the battery voltage for reasons unknown with sperate code, however it matches the bat volt
  msg.header.length = 2;                  
  msg.data[1] = 0x33;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                  // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0x60;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0x61;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0x8;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 32;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0x12;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  regCANmtr ();                      // Appears to give one motor cycle afterwards.

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0x80;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0x31;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0xA4;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0xA3;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0xA2;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  msg.id = 16;                 // Unknown Function
  msg.header.length = 2;
  msg.data[1] = 0xA1;
  txmsg ();
  delay(1);
  receiveFrame(received_msg);

  }
*/
