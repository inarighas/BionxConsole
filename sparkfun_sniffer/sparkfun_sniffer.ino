#include "Canbus.h"  // don't forget to include these
#include "defaults.h"
#include "global.h"
#include "mcp2515.h"
#include "mcp2515_defs.h"

unsigned long time = 0;
int j = 0;
void setup()
{
Serial.begin(115200);

//Initialise MCP2515 CAN controller at the specified speed
if(Canbus.init(CANSPEED_125))
  Serial.println("CAN Init ok");
else
  Serial.println("Can't Init CAN");
delay(1000);
}
/*
Shield initialization will be required for all tasks. Here, we define our CAN bitrate and import our library. Every vehicle might use different bitrate speeds. For our example, we use 500 kbps.

Available options are:

CANSPEED_125 //CAN speed at 125 kbps
CANSPEED_250 //CAN speed at 250 kbps
CANSPEED_500 //CAN speed at 500 kbps
*/
void loop()
{
tCAN message;


if (mcp2515_check_message()) {
    if (mcp2515_get_message(&message)){
               //Serial.print(" >>Start loop...");
               Serial.println("");
               Serial.print("ID: ");
               Serial.print(message.id,HEX);
               Serial.print(", ");
               Serial.print("Data: ");
               for(int i=0;i<message.header.length;i++){
                  Serial.print(message.data[i],HEX);
                  Serial.print(" ");
                  }
               //j++;
               //if(j == 20)  return;
             }
    }
}
