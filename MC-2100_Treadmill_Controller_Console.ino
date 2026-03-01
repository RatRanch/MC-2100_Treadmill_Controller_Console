/*
  MC-2100 Treadmill Motor Controller Interface
  Ver 0.9

  Builds on original work by Joe Schoolcraft and Brian Schoolcraft - May 2013
  https://sonsofinvention.wordpress.com/2013/05/22/arduino-compatible-mc-2100-controller-and-lathe-tachometer

  Additional work on speed control PWM by FanMan 170122

  Added incline control and speedometer - JL 2026 https://jimandnoreen.com 

  Pinout of connector HD2 on the MC2100:
  
    1 black       ground
    2 red         v+ 12V
    3 green       5V PWM belt speed control (1K resistor recommended)
    4 blue        belt tach to console (2.4K pullup resistor required; see wiring diagram)
    5 orange      incline up (1.5 - 5V relative to pin 6, not ground!) 220 ohm resistor recommended
    6 yellow      incline down (1.5 - 5V relative to pin 5) 220 resistor recommended
    7 violet      incline pulse (when moving, 3 pulses per degree of incline. Use a 2.4K pullup resistor.)
    8 black/wh    ground (unconfirmed reports say this also carries signal info on some models)

  Note incline direction: up is pin 5+ / pin 6-, down is 5- / 6+
  Incline motor can be tested by attaching a 1.5V battery between pins 5 and 6
  Belt motor can be tested using a generic PWM controller running at 20 Hz
  Incline and belt pulses require 5V from Arduino to drive the PC817 optocoupler on MC-2100 board 
*/

#include <Wire.h> //I2C support for LCD
#include <LiquidCrystal_I2C.h> //For LCD display
#include <TimerOne.h> //Used for PWM. This library uses hardware timer 1


/* Specifics for treadmill model:
 *  NordicTrack NTL22011
 *  Elite 9700 Pro
 */

#define MAX_SPEED 12 //Max speed in MPH
#define MIN_INCLINE -3 //Can be less than zero degrees on some models
#define MAX_INCLINE 15 //Max degrees incline
#define FEET_PER_PULSE 1.0 //Observed 70ft/min @ 25% duty (PWM 217)
#define PULSES_PER_DEGREE_INCLINE 3 //Number of pulses for 1 degree of movement

/* 
 * Pin Assignments 
 */
#define INCLINE_READ 2 //Incline pulse from violet wire of MC2100
#define INCLINE_LED 4 //LED to indicate incline pulse received
#define BELT_READ 3 //Belt pulse from green wire of MC2100
#define BELT_LED 13 //LED to indicate belt pulse received
#define ON_OFF 12 //Treadmill belt PWM SPST on/off switch
#define PWM_OUT 9 //Connected to blue wire of MC2100 (50ms period PWM out)
#define POT_READ A0 //Wiper of pot connected as voltage divider (PWM speed command)
#define INCLINE_UP 8 //Incline up switch
#define INCLINE_DOWN 7 //Incline down switch
#define INCLINE_UP_OUT 10 //Incline up output (5V relative to pin 10)
#define INCLINE_DOWN_OUT 11 //Incline down output (5V relative to pin 11)

/*
 * Other settings that shouldn't need change
 */
#define POT_DIF 2 //Change detection threshold on pot.  Try 4 if pot is noisy.
#define MAX_DUTY 869 //Max Duty Cycle expected by MC-2100 (85% of 1023)
#define MIN_DUTY 64 //Min Duty Cycle expected by MC-2100 (6.25% of 1023)

LiquidCrystal_I2C lcd(0x3F, 16, 2); //Initialize 16 char X 2 line LCD display

volatile unsigned long beltPulseCount = 0;
unsigned long lastBeltPulseTime = 0;
unsigned long currentBeltPulseRate = 0;
unsigned long tempBeltPulseRate = 0;
unsigned long lastBeltPulseRate = 0;
unsigned long beltPulseCountAtLastMeasurement = 0;
unsigned long currentTime = 0;
unsigned long lastTime = 0;
unsigned long lastInclinePulseTime;
volatile unsigned long inclinePulseCount;
int pulsesPerDegreeIncline = PULSES_PER_DEGREE_INCLINE;
float feetPerPulse = FEET_PER_PULSE;
int lastInclinePulseCount = 0;
int minIncline = MIN_INCLINE;
float currentSpeed = 0;
float maxDuty = MAX_DUTY;
float minDuty = MIN_DUTY;
float maxSpeed = MAX_SPEED;
float distanceTraveled = 0; //in feet

/* 
 * Temporary handling of incline.  Needs to be updated to:
 * - save position in EEPROM (using EEPROM.put(), EEPROM.get() )
 * - implement calibrate function to set hi/low limits
 * - change behavior of up/down switch to seek to next incline position based on pulse received
 *   so that we are always storing an accurate position in EEPROM when the treadmill is switched off
*/
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
  pinMode(BELT_READ, INPUT);
  pinMode(ON_OFF, INPUT_PULLUP);
  pinMode(BELT_LED, OUTPUT);
  pinMode(INCLINE_LED, OUTPUT);
  pinMode(INCLINE_UP, INPUT_PULLUP);
  pinMode(INCLINE_DOWN, INPUT_PULLUP);
  pinMode(INCLINE_UP_OUT, OUTPUT); //Pin for 5V output to incline
  pinMode(INCLINE_DOWN_OUT, OUTPUT);
  pinMode(PWM_OUT, OUTPUT);

  //set up PWM speed pulse output
  Timer1.initialize(50000); //Set pin 9 and 10 period to 50 ms (20 Hz)
  Timer1.pwm(PWM_OUT, 0); //Start PWM at 0% duty cycle 

  //attaches an interrupt handler to the incline pulse pin
  attachInterrupt(digitalPinToInterrupt(INCLINE_READ), inclinePulse, RISING);

  //attaches an interrupt handler to the belt pulse pin
  attachInterrupt(digitalPinToInterrupt(BELT_READ), beltPulse, RISING); 
  
  Serial.begin(9600);
  delay(200);
  
  lastInclinePulseTime = micros();
  inclinePulseCount = 0; 
}

void inclinePulse(){
  inclinePulseCount++;
  //Toggle LED as pulses are received
  if ( (inclinePulseCount % 2) == 0) { digitalWrite(INCLINE_LED, HIGH); } else { digitalWrite(INCLINE_LED, LOW); }
}

void beltPulse(){
  beltPulseCount++;
  //Toggle LED as pulses are received
  if ( (beltPulseCount % 2) == 0) { digitalWrite(BELT_LED, HIGH); } else { digitalWrite(BELT_LED, LOW); }
}

boolean reachedLimit(){
  if(currentPosition == 0 || currentPosition == maxPosition){
    inclineStop();
    if(currentPosition == maxPosition){      
      //Serial.println(" -- Now in Max position"); 
    } else { 
      //Serial.println(" -- Now in Min position"); 
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
   currentInclineDegrees = currentPosition / pulsesPerDegreeIncline + minIncline;
   //currentSpeed = (speedLevel / maxDuty) * maxSpeed; //Expected speed based on PWM
   
   currentSpeed = currentBeltPulseRate * feetPerPulse * 0.681818; //Convert feet/second to MPH

   distanceTraveled = (beltPulseCount * feetPerPulse) / 5280; //5280 feet per mile

   lcd.setCursor(0,0); //LCD line 1
   if (onOffState == LOW){
    lcd.print(currentSpeed, 1);
    lcd.print(" MPH");
    lcd.setCursor(10,0);
    lcd.print(distanceTraveled);
    lcd.print("mi");
   }
   else {
    lcd.print("--- MPH"); 
   }
   
   lcd.setCursor(6,1); //LCD line 2
   lcd.print(currentInclineDegrees);
   lcd.print(char(223)); //degree symbol
   lcd.print(" Incline");

}

void loop(){ 
  //Read and condition pot value
  potTemp = analogRead(POT_READ);
  potCheck = abs(potTemp - potValue);
  if (potCheck >= POT_DIF) { //Only accept new value if it’s far enough from the current accepted value
    potValue = potTemp;
  }
  onOffState = digitalRead(ON_OFF); //PWM output state
  speedLevel = map(potValue, 0, 1023, MIN_DUTY, MAX_DUTY); //Convert Pot input to PWM level to send to MC-2100

  if (onOffState == HIGH) { //ON  switch to ground is open
    Timer1.setPwmDuty(PWM_OUT, 0); //Shut down MC-2100
  }

  if (onOffState == LOW) { //OFF switch to ground is closed
    Timer1.setPwmDuty(PWM_OUT, speedLevel); //Send speed command to MC-2100
  }

   // Increment or decrement pulse count, depending on last incline switch pressed
   if(lastInclinePulseCount < inclinePulseCount){
    currentPosition = currentPosition + ((inclinePulseCount - lastInclinePulseCount) * inclineDirection);
   }
   
   lastInclinePulseCount = inclinePulseCount;
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
  currentTime = millis();
  
    // Calculate PPS every 1000ms
  if (currentTime - lastTime >= 1000) {
    // Calculate pulses per second
    tempBeltPulseRate = (beltPulseCount - beltPulseCountAtLastMeasurement) / ((currentTime - lastTime)/1000);
    /*
     * Accept changed pulse rate if we get it twice in a row
     * This is a kludge for now.  Need to re-examine best way to smooth
     * readings.  For example, at around 5 MPH, we might capture 7 or 8 pulses per second
     */

    if (tempBeltPulseRate == lastBeltPulseRate) {
      currentBeltPulseRate = tempBeltPulseRate;
    }

    // Reset pulse counter for next measurement
    beltPulseCountAtLastMeasurement = beltPulseCount;
    
    // Update timestamp
    lastTime = currentTime;
        
    // Store last pulse rate for comparison
    lastBeltPulseRate = tempBeltPulseRate;
    Serial.println(currentBeltPulseRate);
  }
  
  updateLCD();
}
