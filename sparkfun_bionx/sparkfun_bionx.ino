#include "Canbus.h"  // don't forget to include these
#include "defaults.h"
#include "global.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"
#include <inttypes.h>
#include <avr/io.h>


#include <TimerOne.h>
//Major Macros
#define BAT   0x10
#define MTR   0x20
#define CSL   0x08    //40?

#define SIZE   30
#define REVERSEMODE 0

/* Define Joystick connection pins */
#define UP     A1
#define DOWN   A3
#define LEFT   A2
#define RIGHT  A5
#define CLICK  A4


volatile uint8_t Flag_Recv = 0;

volatile uint32_t table[SIZE] = {0};
volatile uint32_t* ptr = table;
volatile uint32_t mean_speed = 0;
volatile uint32_t mean_troque = 0 ;
volatile uint8_t  retro_level = 0;
//float mean_s = 0;
//float mean_t = 0;

//Global Variables
int j = 0;
uint32_t spdalg = 0;                         // "Speed" reading from POT
uint32_t spdref = 0;                         // Speed Reference from 0 to 55 km/h
uint32_t spdreq = 0;                         // Speed Required to motor
uint32_t spdcrt = 0;                         // Speed Correction Factor
uint32_t spdtst = 0;                         // Speed math test 
uint32_t spdout = 0x10;                      // Speed output to motor
uint32_t mtrspd = 0;                         // Speed from motor
uint32_t spddis = 0;                         // Speed Display in Human Readable Format
uint32_t mtrtrq = 0;                          // Torque from motor
int batvlt = 0;                       // General value for a fully charged battery
boolean strt = false;                    // Check for irregular cycle if its in the start up or not
int irrlp = 0;                           // Irregular CAN Loop every 3 cycles



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
  uint8_t i = 0;
  
  while (mcp2515_check_message()) {
      if (mcp2515_get_message(message_ptr)){
        Serial.print("   >Got : ");
        printMessage(*message_ptr);
        updateDataVariables(*message_ptr);
        //Serial.print(mtrspd,HEX); 
        //Serial.print(" : ");                         
        //Serial.println(mtrtrq ,HEX);  
      }
      flushBuffer(message_ptr);
      i++;
  }
  
  if (i>0)  return true;
  //Serial.println(">Got no response");
  return false;
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
  if (msg.id == 0x8 && msg.data[1] == 0x11){    // Motor Speed
    mtrspd = msg.data[3];
    pedinput ();      
    /*if (mtrspd > spdref) {                            // Speed compensation
      spdcrt --;
    }
    if (mtrspd < spdref) {
      spdcrt ++;
      }*/
    spddis = map(mtrspd, 0, 47, 0, 55);               // motor speed to KM/h equvilents
  }
  else if (msg.id == 0x8 && msg.data[1] == 0x14){    // Torque/Force Applied to wheel
    mtrtrq = msg.data[3];
    uint32_t val = (mtrtrq<<16)+mtrspd;
    
    add_to_table((((uint32_t)mtrtrq)<<16)+mtrspd);
    mean_speed = mean_spd_simple();
    mean_troque = mean_trq_simple();
  }
  else if (msg.id == 0x8 && msg.data[1] == 0x3B){
    batvlt = msg.data[3];
  }
}

void pedinput () {
/*
  spdalg = analogRead(0);                               
  // Analog Read due to limit switch, throwing
  // too much voltage in the open state due to
  // Stray capacitance issues, limited the effect below*/
  
  delay(5);                                             
  spdalg = 600;
//Serial.println(spdalg);                                 // Diagnostic Serial Connection

/*
    To compensate the issue with the limit switch treated it as a analog read
    with a cut off above the threshold (0-1023 are the Arduino limits) to cause
    the Arduino to turn on only when it has reached a high enough value.
    Code is a sign change from ">" to "<" etc for the normally closed posistion.
    Appears to solve the self starting issues.
    
    Due to the varying torque, and us requiring the max amount, code tells the motor to drive
    as quickly as possible.
*/
  if (retro_level){
    if (retro_level == 1)  spdout=0xFFFB;
    else if (retro_level == 2)  spdout=0xFFF6;
    else if (retro_level == 3)  spdout=0xFFEC;
    else if (retro_level == 4)  spdout=0xFFD8;
    }
  
  else if (spdalg < 850) {
    spdreq = 0x5;
    spdref = 0x1;
    spdout = 0xFF;
  }
  else if (spdalg > 850) {
    spdreq = 0x0;
    spdref = 0x0;
    spdout = 0x0;
    spdcrt = 0x0;
  }
}
 
void sendFrame(uint16_t identifier, uint8_t len, uint32_t command){
  tCAN msg ;
  uint32_t tmp = 0;
  tCAN toto;
  tCAN* received_msg = &toto;

  while (Flag_Recv == 1){
    Flag_Recv = 0;
    receiveFrame(received_msg);
  }
  
  if(len != 0x4 && len != 0x2){
    // or size message is different than length
    Serial.print("ERROR SENDFRAME! -");
    Serial.println(len);
    //exit(0);
    }

  msg.header.length = len;
  msg.id = identifier;
  //Serial.println(uint32_t(command),HEX);
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

}



void initialCommands(){

  
  sendFrame(BAT, 4, 0x200000);        // Start code of the system's set of CAN Bus packages.
  delay(1);
  receiveFrame(received_msg);

  sendFrame(BAT, 4, 0x210001);        // Start code of the system's set of CAN Bus packages.
  delay(1);
  receiveFrame(received_msg);

  sendFrame(BAT, 2, 0x003B);
  delay(1);
  receiveFrame(received_msg);
  

  sendFrame(BAT, 2, 0x003C);
  delay(1);
  receiveFrame(received_msg);
  
  
  sendFrame(BAT, 4, 0x00220000);        // Unknown Function  
  delay(1);
  receiveFrame(received_msg);

  sendFrame(BAT, 2, 0x003D);
  delay(1);
  receiveFrame(received_msg);
  
  sendFrame(MTR, 4, 0x00020000);        // Unknown Function
  delay(1);
  receiveFrame(received_msg);


  sendFrame(MTR, 4, 0x00410000);        // Unknown Function
  delay(1);
  receiveFrame(received_msg);  

  sendFrame(MTR, 4, 0x00420001);         // Reverse or Forward 0 = Reverse and 1 = Forward 
  delay(1);
  receiveFrame(received_msg); 

  sendFrame(MTR, 2, 0x0020);             // Unknown Function 
  delay(1);
  receiveFrame(received_msg); 

  sendFrame(MTR, 2, 0x0011);             // Check Speed to see if already moving
  delay(1);
  receiveFrame(received_msg);
}

void startCANloop (){                  // Start sequence for a series of CAN requests. Must follow the Battery voltage check.
  // Start code of the system's set of CAN Bus packages.
  tCAN toto;
  tCAN* received_msg = &toto;
  flushBuffer(received_msg);
  sendFrame(0x58, 2, 0x009C);
  delay(1);
  receiveFrame(received_msg);
  sendFrame(BAT, 2, 0x003B);
  delay(1);
  receiveFrame(received_msg);
  

}


void regularCANmtr (){      // Regular CAN loop requests to the Motor
  tCAN toto;
  tCAN* received_msg = &toto;
  flushBuffer(received_msg);
      
      // Currently Unknown Code, however due to the nature of the tests,  
    // assumed to be related to foot pedal power or regenerative breaking.
  sendFrame(MTR, 2, 0x0021);
  receiveFrame(received_msg) ;

      // Send the Speed desired to the wheel
  sendFrame(MTR, 4, uint32_t(0x090000+spdout)); 

      // Send the Speed desired to the wheel, repeated as BionX does
  sendFrame(MTR, 4, uint32_t(0x090000+spdout));

      // Check the Speed of the wheel
  sendFrame(MTR, 2, 0x0011);
  receiveFrame(received_msg);
      
      // Check the Torque Sensor of the Wheel, 14=Torque Sensor Value
  sendFrame(MTR, 2, 0x0014);
  receiveFrame(received_msg);

  if(REVERSEMODE) sendFrame(MTR, 4, 0x00420000);         // Reverse or Forward 0 = Reverse and 1 = Forward

}
  
void regularCANbat () {
  tCAN toto;
  tCAN* received_msg = &toto;
  flushBuffer(received_msg);

      // Send Text to appear to request the status of the battery.
  sendFrame(BAT, 2, 0x0032);
  receiveFrame(received_msg);
}

void regular_cycle(){
  tCAN toto;
  tCAN* received_msg = &toto;
  flushBuffer(received_msg);
  /*Serial.println(millis());
  Serial.print("s ");
  Serial.print(mean_speed);
  Serial.print(" - t");
  Serial.println(mean_troque);*/

    startCANloop ();                       // Start of the CAN Loop
    regularCANmtr ();                      // One Standard Motor CAN Cycle  
    //regularCANmtr ();                      // One Standard Motor CAN Cycle 
    //regularCANmtr ();                      // One Standard Motor CAN Cycle
    //regularCANmtr ();                      // One Standard Motor CAN Cycle
    regularCANbat ();                      // Ending Battery Cycle

}

void shutdown_click(){
  tCAN toto;
  tCAN* received_msg = &toto;
  flushBuffer(received_msg);
  regularCANbat ();                      // Ending Battery Cycle
  Serial.println("... Battery shutdown");
  delay(1000);
  sendFrame(MTR, 4, 0x420001);
  delay(10);
  receiveFrame(received_msg);
  
  sendFrame(BAT, 4, 0x250001);
  delay(10);
  receiveFrame(received_msg);

  exit(0);
 }

void MCP2515_ISR()
{
     Flag_Recv = 1;
}

void setup() {
  Serial.begin(115200);
  //Initialise MCP2515 CAN controller at the specified speed
  if(Canbus.init(CANSPEED_125)) Serial.println("CAN Init ok");
  else Serial.println("Can't Init CAN");
  delay(1000);
  Serial.println("end setup");

  Serial.println("  >Start up CAN cycle ...");
  Serial.println("HelloWorld!");
  delay(1000);
  //Serial.println("speed \t\t troque");

  //pinMode(CLICK,INPUT);
  //digitalWrite(CLICK, HIGH);


  //attachInterrupt(digitalPinToInterrupt(CLICK), shutdown_click, LOW);
   attachInterrupt(0, MCP2515_ISR, FALLING);  // interrupt indicates when message is available
  //interrupt Setup
  //Timer1.initialize(75000);
  //Timer1.attachInterrupt(regular_cycle); 
}



void loop() {
 noInterrupts();
 tCAN toto;
 tCAN* received_msg = &toto;
 flushBuffer(received_msg);

 spdalg = 0;                          // "Speed" reading from POT
 spdref = 0;                          // Speed Reference from 0 to 55 km/h
 spdreq = 0;                          // Speed Required to motor
 spdcrt = 0;                          // Speed Correction Factor
 spdout = 0;                         // Speed output to motor
 delay(1000);

 
 initialCommands ();
 //Serial.println("  >... Ended sending initial commands.");
 regularCANmtr ();                     // Appears to give one motor cycle afterwards.
  //Serial.println("  >... Ended regular Motor procedure.");  
  //Serial.println("Starting regular Battery procedure...");
 regularCANbat ();                      // Ending Battery Cycle
  //Serial.println("  >... Ended regular Battery procedure.");

 interrupts();
 Serial.println("Free Isr!!");
 Serial.println("Keeping interrupts doing job ;)");
 
 while(1){
   regular_cycle();
   delay(50);
  }

  /* 
  Serial.println("... Battery shutdown");
  sendFrame(MTR, 4, 0x420001);
  delay(10);
  receiveFrame(received_msg);
  
  sendFrame(BAT, 4, 0x250001);
  delay(10);
  receiveFrame(received_msg);
  Serial.println("... Battery shutdown");
  while(1);
  */
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
