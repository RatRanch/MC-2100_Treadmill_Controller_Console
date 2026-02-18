/*
  MC-2100 Treadmill Motor Controller Interface

  Builds on original work by Joe Schoolcraft and Brian Schoolcraft - May 2013
  https://sonsofinvention.wordpress.com/2013/05/22/arduino-compatible-mc-2100-controller-and-lathe-tachometer

  Additional work on speed control PWM by FanMan 170122

  Added incline control and notes - JL 2026 https://jimandnoreen.com 

  Pinout of connector HD2 on the MC2100:
  
    1 black       ground
    2 red         v+ 12V
    3 green       5V PWM belt speed control
    4 blue        belt tach to console (signaling protocol TBD)
    5 orange      incline up (1.5 - 5V relative to pin 6, not ground!)
    6 yellow      incline down (1.5 - 5V relative to pin 5)
    7 violet      incline pulse (when moving, 3 pulses per degree of incline)
    8 black/wh    ground (unconfirmed reports that this also carries signal info on some models)

  Note incline direction: up is pin 5+/6-, down 5-/6+
  Incline motor can be tested by attaching a 1.5V battery between pins 5 and 6
  Belt motor can be tested using a generic PWM controller running at 20 Hz 
*/

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <TimerOne.h> //Used for PWM


/* Specifics for treadmill model:
 *  NordicTrack NTL22011
 *  Elite 9700 Pro
 */

#define MAX_SPEED 12 //Max speed in MPH
#define MIN_INCLINE -3 //Can be less than zero degrees 
#define MAX_INCLINE 15 //Max degrees incline

/* 
 * Pin Assignments 
 */
#define INCLINE_READ 2 //Incline pulse from violet wire of MC2100
#define ON_OFF 12 //Treadmill belt PWM SPST on/off switch
#define PWM_OUT 9 //Connected to blue wire of MC2100 (50ms period PWM out)
#define POT_READ A0 //Wiper of pot connected as voltage divider (PWM speed command)
#define INCLINE_UP 8 //Incline up switch
#define INCLINE_DOWN 7 //Incline down switch
#define INCLINE_UP_OUT 11 //Incline up output (5V relative to pin 10)
#define INCLINE_DOWN_OUT 10 //Incline down output (5V relative to pin 11)

/*
 * Other settings that shouldn't need change
 */
#define PWM_CYCLE 50.0 //Output Signal PWM Period (50ms)
#define POT_DIF 4 //Change detection threshold on pot
#define MAX_DUTY 869 //Max Duty Cycle expected by MC-2100 (85% of 1023)
#define MIN_DUTY 0 //Min Duty Cycle expected by MC-2100 (0% of 1023)

LiquidCrystal_I2C lcd(0x3F, 16, 2); //Initialize 2 line LCD display

const int timeoutValue = 5;

volatile unsigned long lastPulseTime;
volatile unsigned long interval = 0;
volatile int timeoutCounter;
volatile int pulseCount;
int lastPulseCount = 0;
float currentSpeed = 0;
float maxDuty = MAX_DUTY;
float maxSpeed = MAX_SPEED;

volatile int currentPosition = 9; //FOR TESTING, ZERO DEGREES INCLINE
const int maxPosition = 54;
int currentInclineDegrees = 0;

int inclineDirection = 0; // 1 when calling for increase, -1 for decrease
byte inclineUpState = 1;
byte inclineDownState = 1;

int potTemp;
int potValue;
int lastPotValue;
int potCheck;
int speedLevel;

byte onOffState = 0;
byte lastonOffState = 0;
unsigned long lastOnOffTime = 0;

void setup(){
  lcd.init();
  //lcd.write(0xFE);lcd.write(0x1E); //disable splash screen on some LCDs.  Need to run just once.
  lcd.backlight();
  lcd.clear();  
  pinMode(INCLINE_READ, INPUT);
  pinMode(ON_OFF, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  digitalWrite(INCLINE_READ, HIGH); 
  pinMode(INCLINE_UP, INPUT_PULLUP);
  pinMode(INCLINE_DOWN, INPUT_PULLUP);
  pinMode(INCLINE_UP_OUT, OUTPUT); //Pin for 5V output to incline
  pinMode(INCLINE_DOWN_OUT, OUTPUT);
  pinMode(PWM_OUT, OUTPUT);

  //set up PWM speed pulse output
  Timer1.initialize(PWM_CYCLE * 1000); //Set pin 9 and 10 period to 50 ms
  Timer1.pwm(PWM_OUT, 0); //Start PWM at 0% duty cycle 

  //attaches an interrupt handler to the incline pulse pin
  attachInterrupt(digitalPinToInterrupt(INCLINE_READ), inclinePulse, RISING);
  
  Serial.begin(9600);
  delay(200);
  
  lastPulseTime = micros();
  timeoutCounter = 0;
  pulseCount = 0; // 54 pulses for entire bed travel, 3 pulses per degree of incline
}

void inclinePulse(){
  pulseCount++;
  //Toggle built-in LED on pin 13 as pulses are received
  if ( (pulseCount % 2) == 0) { digitalWrite(13, HIGH); } else { digitalWrite(13, LOW); }
}

boolean reachedLimit(){
  if(currentPosition == 0 || currentPosition == maxPosition){
    inclineStop();
    if(currentPosition == maxPosition){      
      Serial.println(" -- Now in Max position"); 
    } else { 
      Serial.println(" -- Now in Min position"); 
    }      
    return true;
  } else {
    return false; 
  }
}

void inclineStop(){
  digitalWrite(INCLINE_UP_OUT, LOW);
  digitalWrite(INCLINE_DOWN_OUT, LOW);
}

void inclineDecrease(){
  if(currentPosition > 0){
    inclineDirection = -1;
    digitalWrite(INCLINE_UP_OUT, LOW);
    digitalWrite(INCLINE_DOWN_OUT, HIGH);
  } else {
    reachedLimit();
  }
}

void inclineIncrease(){
  if(currentPosition < maxPosition){
    inclineDirection = 1;
    digitalWrite(INCLINE_DOWN_OUT, LOW);
    digitalWrite(INCLINE_UP_OUT, HIGH);
  } else {
    reachedLimit();
  }
}

void updateLCD(){
   currentInclineDegrees = currentPosition / 3 - 3;
   currentSpeed = (speedLevel / maxDuty) * maxSpeed; //KLUDGE Need to derive from tach instead.

   lcd.setCursor(0,0);
   lcd.print("  Speed: ");
   if (onOffState == LOW){
      lcd.print(currentSpeed, 1);
   }
   else
   {
       lcd.print("---"); 
   }
   //lcd.print(" MPH ");  
   
   lcd.setCursor(0,1);
   lcd.print("Incline: ");
   lcd.print(currentInclineDegrees);
   lcd.print("  ");
   //lcd.setCursor(14,1);
   //lcd.print(pulseCount);  
}

void loop(){ 

  //Read and condition pot value
  potTemp = analogRead(POT_READ);
  potCheck = abs(potTemp - potValue);
  if (potCheck >= POT_DIF) { //Only accept new value if it’s far enough from the current accepted value
    potValue = potTemp;
  }
  onOffState = digitalRead(ON_OFF); //PWM output state
  speedLevel = map(potValue, 0, 1023, 0, MAX_DUTY); //Convert Pot input to pwm level to send to MC-2100

  if (onOffState == HIGH) { //ON  switch to ground is open
    Timer1.setPwmDuty(PWM_OUT, 0); //Shut down MC-2100
  }

  if (onOffState == LOW) { //OFF switch to ground is closed
    Timer1.setPwmDuty(PWM_OUT, speedLevel); //Send speed command to MC-2100
  }

   // Increment or decrement pulse count, depending on last incline switch pressed
   if(lastPulseCount < pulseCount){
    currentPosition = currentPosition + ((pulseCount - lastPulseCount) * inclineDirection);
   }
   
   lastPulseCount = pulseCount;
   inclineUpState = digitalRead(INCLINE_UP);
   inclineDownState = digitalRead(INCLINE_DOWN);

  if (inclineUpState == LOW) { //Switch to ground is closed
    inclineIncrease();
  }
  
  if (inclineDownState == LOW) { //Switch to ground is closed
    inclineDecrease();
  }

  if (inclineDownState == HIGH && inclineUpState == HIGH) { // if BOTH the switches are open
      inclineStop();
  }

  updateLCD();
}
