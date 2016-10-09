#include "Canbus.h"  // don't forget to include these
#include "defaults.h"
#include "global.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"
#include <inttypes.h>
#include <avr/io.h>
#define BAT   0x10
#define MTR   0x20
#define CSL   0x08    //40?

int j = 0;
int spdalg = 0;                          // "Speed" reading from POT
int spdref = 0;                          // Speed Reference from 0 to 55 km/h
int spdreq = 0;                          // Speed Required to motor
int spdcrt = 0;                          // Speed Correction Factor
int spdtst = 0;                          // Speed math test 
int spdout = 0;                          // Speed output to motor
int mtrspd = 0;                          // Speed from motor
int spddis = 0;                          // Speed Display in Human Readable Format
int mtrtrq = 0;                          // Torque from motor
int batvlt = 0xD3;                       // General value for a fully charged battery
boolean strt = false;                    // Check for irregular cycle if its in the start up or not
int irrlp = 0;                           // Irregular CAN Loop every 3 cycles


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
      //Serial.println("Got smtg!!");
      delay(1);
      if (mcp2515_get_message(message_ptr)){
        //Serial.print("   >Got : ");
        //printMessage(*message_ptr);
        //updateDataVariables(*message_ptr);
        //Serial.print(mtrspd,HEX); 
        //Serial.print(" \t\t ");                         
        //Serial.println(mtrtrq ,HEX);  
        flushBuffer(message_ptr);
      }
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
        if (mtrspd > spdref) {                            // Speed compensation
          spdcrt --;
        }
        if (mtrspd < spdref) {
          spdcrt ++;
        }
        spddis = map(mtrspd, 0, 47, 0, 55);               // motor speed to KM/h equvilents
      } 
      if (msg.id == 0x8 && msg.data[1] == 0x14){    // Torque/Force Applied to wheel
        mtrtrq = msg.data[3];
      }
  }

void pedinput () {
/*
  spdalg = analogRead(0);                               
  // Analog Read due to limit switch, throwing
  // too much voltage in the open state due to
  // Stray capacitance issues, limited the effect below*/
  delay(5);                                             
  spdreq = 0x0;
  spdref = 0x0;
  spdout = 0x0;
  spdcrt = 0x0;
  }
 
char sendFrame(uint16_t identifier, uint8_t len, uint32_t command){
  tCAN msg ;
  uint32_t tmp = 0;
  if(len != 0x4 && len != 0x2){
    // or size message is different than length
    Serial.println(len);
    exit(0);
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
  //Serial.print(">>SENT  :");
  //printMessage(msg);
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
  Serial.println("speed \t\t troque");

}

void startCANloop (){                  // Start sequence for a series of CAN requests. Must follow the Battery voltage check.
  // Start code of the system's set of CAN Bus packages.
  tCAN toto;
  tCAN* received_msg = &toto;
  flushBuffer(received_msg);
  sendFrame(0x58, 2, 0x009C);
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
  delay(1);
  receiveFrame(received_msg) ;

      // Send the Speed desired to the wheel
  sendFrame(MTR, 4, uint32_t(0x090000+spdout)); 
  delay(1);
  receiveFrame(received_msg);

      // Send the Speed desired to the wheel, repeated as BionX does
  sendFrame(MTR, 4, uint32_t(0x090000+spdout));
  delay(1);

      // Check the Speed of the wheel
  sendFrame(MTR, 2, 0x0011);
  delay(1);
  receiveFrame(received_msg);
      
      // Check the Torque Sensor of the Wheel, 14=Torque Sensor Value
  sendFrame(MTR, 2, 0x0014);
  delay(1);
  receiveFrame(received_msg);
  delay(1);
  receiveFrame(received_msg);
}

void regularCANbat () {
  tCAN toto;
  tCAN* received_msg = &toto;
  flushBuffer(received_msg);

      // Send Text to appear to request the status of the battery.
  sendFrame(BAT, 2, 0x0032);
  delay(10);
  receiveFrame(received_msg);
}

void initialCommands(){
  tCAN toto;
  tCAN* received_msg = &toto;
  
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


void loop() {
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

  //Serial.println("Starting regular Motor procedure...");
  regularCANmtr ();                     // Appears to give one motor cycle afterwards.
  //Serial.println("  >... Ended regular Motor procedure.");

  
  //Serial.println("Starting regular Battery procedure...");
  regularCANbat ();                      // Ending Battery Cycle
  //Serial.println("  >... Ended regular Battery procedure.");

  Serial.println(millis());
  startCANloop ();                       // Start of the CAN Loop
  regularCANmtr ();                      // One Standard Motor CAN Cycle  
  regularCANmtr ();                      // One Standard Motor CAN Cycle 
  regularCANmtr ();                      // One Standard Motor CAN Cycle
  regularCANmtr ();                      // One Standard Motor CAN Cycle

  //Serial.print("time :");
  Serial.println(millis());
  
  regularCANbat ();                      // Ending Battery Cycle
  Serial.println("... Battery shutdown");
  delay(1000);
  sendFrame(MTR, 4, 0x420001);
  delay(10);
  receiveFrame(received_msg);
  
  sendFrame(BAT, 4, 0x250001);
  delay(10);
  receiveFrame(received_msg);
  Serial.println("... Battery shutdown");
  while(1);
  
}
