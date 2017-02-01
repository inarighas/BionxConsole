// SparkFun Serial LCD example 1
// Clear the display and say "Hello World!"

// This sketch is for Arduino versions 1.0 and later
// If you're using an Arduino version older than 1.0, use
// the other example code available on the tutorial page.

// Use the softwareserial library to create a new "soft" serial port
// for the display. This prevents display corruption when uploading code.
#include <SoftwareSerial.h>

// Attach the serial display's RX line to digital pin 2
SoftwareSerial LCD(3,6); // pin 6 = TX, pin 3 = RX (unused)


/*
2400 baud, "<control>k" => 0x0B
4800 baud, "<control>l" => 0x0C
9600 baud, "<control>m" => 0x0D
14400 baud, "<control>n" => 0x0E
19200 baud, "<control>o" => 0x0F
38400 baud, "<control>p" => 0x10
reset to default baud while LCD is the splash screen is still active, "<control>r" => 0x12
*/
void changeBaud(){
  LCD.write(0x7C);// special command byte => 0d124 or 0x7C
  LCD.write(0x10); //change current baud to 19200 baud
  delay(100);
}

void setup()
{
  //LCD.begin(19200);// all SerLCDs come at 9600 Baud by default
  //changeBaud();
  LCD.begin(9600);
}
//-------------------------------------------------------------------------------------------
void loop()
{ int lvl = 0;
  int ass = -5;
  int spdout = -2;
  clearScreen();
  //scrollingMarquee();
  //tempAndHumidity();
  for(int i=0;i<25;i++){
    lcdSpeedUpd(i);
    lcdBatUpd(lvl);
    lcdAssLevel(ass);
    lcdAssStatus(spdout,ass);
    ass ++;
    lvl += 10;
    spdout += 7;
    delay(200);
  }
  delay(300);
  //lcdSpeedUpd();
  //backlight();
  //cursors();
}
//-------------------------------------------------------------------------------------------
void clearScreen()
{
  //clears the screen, you will use this a lot!
  LCD.write(0xFE);
  LCD.write(0x01); 
}
//-------------------------------------------------------------------------------------------
void selectLineOne()
{ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(128);  //position
}
//-------------------------------------------------------------------------------------------
void selectLineTwo()
{ 
  //puts the cursor at line 0 char 0.
  LCD.write(0xFE); //command flag
  LCD.write(192); //position
}
//-------------------------------------------------------------------------------------------
void moveCursorRightOne()
{
  //moves the cursor right one space
  LCD.write(0xFE); //command flag
  LCD.write(20); // 0x14
}
//-------------------------------------------------------------------------------------------
void moveCursorLeftOne()
{
  //moves the cursor left one space
  LCD.write(0xFE); //command flag
  LCD.write(16); // 0x10
}
//-------------------------------------------------------------------------------------------
void scrollRight()
{
  //same as moveCursorRightOne
  LCD.write(0xFE); //command flag
  LCD.write(20); // 0x14
}
//-------------------------------------------------------------------------------------------
void scrollLeft()
{
  //same as moveCursorLeftOne
  LCD.write(0xFE); //command flag
  LCD.write(24); // 0x18
}
//-------------------------------------------------------------------------------------------
void turnDisplayOff()
{
  //this tunrs the display off, but leaves the backlight on. 
  LCD.write(0xFE); //command flag
  LCD.write(8); // 0x08
}
//-------------------------------------------------------------------------------------------
void turnDisplayOn()
{
  //this turns the dispaly back ON
  LCD.write(0xFE); //command flag
  LCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void underlineCursorOn()
{
  //turns the underline cursor on
  LCD.write(0xFE); //command flag
  LCD.write(14); // 0x0E
}
//-------------------------------------------------------------------------------------------
void underlineCursorOff()
{
  //turns the underline cursor off
  LCD.write(0xFE); //command flag
  LCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void boxCursorOn()
{
  //this turns the box cursor on
  LCD.write(0xFE); //command flag
  LCD.write(13); // 0x0D
}
//-------------------------------------------------------------------------------------------
void boxCursorOff()
{
  //this turns the box cursor off
  LCD.write(0xFE); //command flag
  LCD.write(12); // 0x0C
}
//-------------------------------------------------------------------------------------------
void toggleSplash()
{
  //this toggles the spalsh screenif off send this to turn onif on send this to turn off
  LCD.write(0x7C); //command flag = 124 dec
  LCD.write(9); // 0x09
}
//-------------------------------------------------------------------------------------------
int backlight(int brightness)// 128 = OFF, 157 = Fully ON, everything inbetween = varied brightnbess 
{
  //this function takes an int between 128-157 and turns the backlight on accordingly
  LCD.write(0x7C); //NOTE THE DIFFERENT COMMAND FLAG = 124 dec
  LCD.write(brightness); // any value between 128 and 157 or 0x80 and 0x9D
}
//-------------------------------------------------------------------------------------------
void scrollingMarquee()
{
//This function scroll text across the screen on both lines
  clearScreen(); // it's always good to clear the screen before movonh onto a new print
  for(int j = 0; j < 17; j++)
  {
    selectLineOne();
    for(int i = 0; i < j;i++)
      moveCursorRightOne();
    LCD.print("SPARK");
    selectLineTwo();
    for(int i = 0; i < j;i++)
      moveCursorRightOne();
    LCD.print(" FUN");
    delay(500); // you must have a delay, otherwise the screen will print and clear before you can see the text
    clearScreen();
  }
}
//-------------------------------------------------------------------------------------------
void counter()
{
  //this function prints a simple counter that counts to 10
  clearScreen();
  for(int i = 0; i <= 10; i++)
  {
    LCD.print("Counter = ");
    LCD.print(i, DEC);
    delay(500);
    clearScreen();
  }
}

void lcdSpeedUpd(int spd){
  clearScreen();
  if (spd>99)  LCD.print("Errspd");
  else{
    if (spd<10) LCD.print("0");
    LCD.print(spd);
    LCD.print("km/h");
  }
}

void lcdBatUpd(int level){
  int edge = 10;
  int i=0;
  selectLineTwo();
  int n = (level/edge)-1;
  if (n>5) LCD.print("ErrBat");
  else{
    for(i=0;i<n;i++){
      LCD.print("#");
    }
    while(i<5){
      LCD.print(" ");
      i++;
    }
    LCD.print(">");
  }
}

void lcdAssLevel(int ass){
  LCD.write(254);LCD.write(142);

  if(ass>4 || ass<-4) LCD.print("ER");
  else{
    if (ass<0) {
      LCD.print("G");
      LCD.print(ass*-1);
    }
    else if (ass>0){
      LCD.print("A");
      LCD.print(ass);
    }
    else LCD.print("00");
  }
}

void lcdAssStatus(int spdout, int ass){
  int edge = 9;
  int i =0;
  LCD.write(254);LCD.write(200);
  if (spdout>50||spdout<0){
    LCD.print("<ErrAss>");
  }
  else if (ass<0){
    LCD.print("<");
    LCD.print("rechrg>");
  }
  else if (ass>=0){
    LCD.print("<");
    int n = (spdout/edge)-1;
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
  
//-------------------------------------------------------------------------------------------
void tempAndHumidity()
{
  //this function shows how you could read the data from a temerature and humidity 
  //sensro and then print that data to the SerLCD.
  
  //these could be varaibles instead of static numbers 
  float tempF = 77.0; 
  float tempC = 25.0;
  float humidity = 67.0;
  
  clearScreen();
  selectLineOne();
  LCD.print(" Temp = ");
  LCD.print((long)tempF, DEC);
  LCD.print("F ");
  LCD.print((long)tempC, DEC);
  LCD.print("C");
  selectLineTwo();
  LCD.print(" Humidity = ");
  LCD.print((long)humidity, DEC); 
  LCD.print("%");
  delay(2500);
}
//-------------------------------------------------------------------------------------------
void backlight()
{
  //this function shows the different brightnesses to which the backlight can be set 
  clearScreen();
  for(int i = 128; i < 158; i+=2)// 128-157 are the levels from off to full brightness
  {
    backlight(i);
    delay(100);
    LCD.print("Backlight = ");
    LCD.print(i, DEC);
    delay(500);
    clearScreen();
  }
}
//-------------------------------------------------------------------------------------------
void cursors()
{
  //this function shows the different cursors avaiable on the SerLCD
  clearScreen();
  
  boxCursorOn();
  LCD.print("Box On");
  delay(1500);
  clearScreen();
  
  boxCursorOff();
  LCD.print("Box Off");
  delay(1000);
  clearScreen();
  
  underlineCursorOn();
  LCD.print("Underline On");
  delay(1500);
  clearScreen();
  
  underlineCursorOff();
  LCD.print("Underline Off");
  delay(1000);
  clearScreen();
}