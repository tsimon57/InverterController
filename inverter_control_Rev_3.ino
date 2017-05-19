


//**************************************************
int is_fault();
int is_run();
float battery_voltage();
void button_press();
void show_status();
void set_status( int stat_in);
void delay_with_updates(long delay_value);
//**************************************************

/*
   pins:  rev 3 board
   D14 - A0  - FAULT
   D15 - A1 - GND
   D16 - A2 - GND
   D17 - A3 - FAULT
   D18 - A4 - SDA
   D19 - A5 - SCL
   --  - A6 -
   --  - A7 - VIN

  Rev 3 is adding support for LCD
  Need A4 and A5 for I2C

  ------------------
  /*
   pins:  rev 2 board
   D14 - A0 FAULT
   D15 - A1 - GND b
   D16 - A2 - GND
   D17 - A3 - RUN
   D18 - GND - LOW
   D19 - A5 - V12
*/

// Pick just one Board
//#define BRD_REV1 TRUE
//#define BRD_REV2 TRUE
#define BRD_REV3 TRUE
//
#define DEBUG 1

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#ifdef BRD_REV4   // same as rev 3 for now
#define USING_LCD 1
#define RUN 3       // analog pin
#define FAULT 0     // analog pin

#define VDD 5.0    // Sshould be exact because of buck converter
#define VPLUS 7     // analog pin
#define CTLPIN 2    // digital pin
#define LEDPIN 13    // digital
#endif BRD_REV4

#ifdef BRD_REV3
#define USING_LCD 1
#define RUN 3       // analog pin
#define FAULT 0     // analog pin
//#define VDD 4.69  // when using USB - this needs to be measured
#define VDD 4.91    // using LDO witih heat sink
#define VPLUS 7     // analog pin
#define CTLPIN 2    // digital pin
#define LEDPIN 13    // digital
#endif BRD_REV3

#ifdef BRD_REV2
#define RUN 3       // analog pin
#define FAULT 0     // analog pin
#define VDD 4.69  // when using USB - this needs to be measured
#define VPLUS 5     // analog pin
#define CTLPIN 2    // digital pin
#define LEDPIN 13    // digital
#endif BRD_REV2

#ifdef BRD_REV1
#define RUN 1
#define FAULT 2
#define VDD 5.0
#define VDD 4.69  // when using USB - this needs to be measured
#define CTLPIN 2
#define LEDPIN 13
#endif BRD_REV1

#define ARRAY_SIZE 100    // for averaging analog reads on battery
#define RESTART_V 13.4
#define SHUTDOWN_V 12.5     // was 12.0

#define RUN_VOLTAGE 8
#define FAULT_VOLTAGE 2

#define STAT_UV  2  // under voltage
#define STAT_FL  3  // fault
#define STAT_RN  1   // running
#define STAT_PS  4    // paused witing for load to reset


#ifdef USING_LCD
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display


#endif USING_LCD


int stat = 5;             // holds status from last action
#define BLINK_TIME  100

;
float R1 = 1000000.0; // resistance of R1 (100K) -see text!
float R2 = 100000.0; // resistance of R2 (10K) - see text!
int value = 0;
int restart_retries = 0;    // to check for motor 'lock' - need several minutes to recover

void setup() {

#ifdef DEBUG
  Serial.begin(9600);
#endif DEBUG
#ifdef USING_LCD
  lcd.init();                      // initialize the lcd
  lcd.setCursor(8, 0);
  lcd.backlight();
  lcd.print("Battery");
  lcd.setCursor(8, 1);
  lcd.print("Voltage");
#endif USING_LCD

#ifdef BRD_REV3
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);

  pinMode(CTLPIN, OUTPUT); // pin goes low at reset, drive output of fet high - swtch closed
  pinMode(LEDPIN, OUTPUT);
#endif BRD_REV3

#ifdef BRD_REV2
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);

  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);

  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);

  pinMode(CTLPIN, OUTPUT); // pin goes low at reset, drive output of fet high - sw closed
  pinMode(LEDPIN, OUTPUT);
#endif BRD_REV2

#ifdef BRD_REV1
// all gnds connect to gnd line, not DPINS
  pinMode(CTLPIN, OUTPUT); // pin goes low at reset, drive output of fet high - sw closed
  pinMode(LEDPIN, OUTPUT);
#endif BRD_REV1


  if (battery_voltage() >= RESTART_V) {
    // start inverter if we have voltage to run
    set_status(STAT_RN);
    delay_with_updates(1000);    // wait so our button press is detected

  } else {
    set_status(STAT_UV);
  }
  digitalWrite(CTLPIN, HIGH);  // set pin high to 1, sw open
  show_status();
  delay_with_updates(15000);    // let startup current spike settle down


} // end of setup
///////////////////////////////////////////////////////

void loop() {

  //    check battery voltage
  show_status();

  //      restart if high enough
  if (is_run() == 0) {
    if (battery_voltage() >= RESTART_V) {
      set_status(STAT_RN);
      button_press();
      if ((battery_voltage() <= SHUTDOWN_V) ) {    // overloaded then....
        set_status(STAT_PS);    // overload so we pause
        show_status();
        button_press();  // will act like button press and shut down
        delay_with_updates((long) 120000);    // wait
      }
    }
    else {
      set_status(STAT_UV);
    }
  }


  //      shut down if too low
  if (is_run() == 1) {
    set_status(STAT_RN);    // we are running
    if (battery_voltage() <= SHUTDOWN_V) {
      set_status(STAT_UV);
      button_press();  // will act like button press and shut down
    }
  }
  //    check for fault and then shut down
  // should never really happen with the other checks in place
  if (is_run() == 1) {
    if (is_fault() == 1) {
      set_status(STAT_FL);
      button_press();
    }
  }


} // end of loop
/////////////////////////////////////////////////////////////////


int is_fault() {

  // read the value at analog input

  float vout = 0.0;
  float vin = 0.0;
  value = analogRead(FAULT);
  vout = (value * VDD) / 1024.0; // see text
  vin = vout / (R2 / (R1 + R2));
  if (vin < 0.09) {
    vin = 0.0; //statement to quash undesired reading !
  }
#ifdef DEBUG
  Serial.print("                            Fault line is:");
  Serial.println(vin);
#endif DEBUG

  if (vin < FAULT_VOLTAGE)  {
#ifdef DEBUG
    Serial.println("Fault detetected");
#ifdef USING_LCD
    lcd.setCursor(0, 3);
    lcd.print ("FLT");
#endif USING_LCD
#endif DEBUG

    return (1);
  }
  else {
    return (0);
  }

}
////////////////////////////////////////////////////////////////////

int is_run() {
  // read the value at analog input

  float vout = 0.0;
  float vin = 0.0;
  value = analogRead(RUN);
  vout = (value * VDD) / 1024.0; // see text
  vin = vout / (R2 / (R1 + R2));
  if (vin < 0.09) {
    vin = 0.0; //statement to quash undesired reading !
  }
#ifdef DEBUG
  Serial.print("         Run line is:");
  Serial.println(vin);
#endif DEBUG
  if (vin < RUN_VOLTAGE) {
    return (1);
  }
  else {
    return (0);
  }
}
///////////////////////////////////////////////////////////////////

float battery_voltage() {
  // read the value at analog input
  int x;
  long sum_of_reads;
  float vout = 0.0;
  float vin = 0.0;

  sum_of_reads = 0;
  for (x=0; x< ARRAY_SIZE; x++) {
    sum_of_reads = sum_of_reads + analogRead(VPLUS);
  }
  value = sum_of_reads/ARRAY_SIZE;
  
  vout = (value * VDD) / 1024.0; // see text
  vin = vout / (R2 / (R1 + R2));
  if (vin < 0.09) {
    vin = 0.0; //statement to quash undesired reading !
  }
#ifdef DEBUG
  Serial.print("Battery voltage is:");
  Serial.println(vin);
#endif DEBUG
#ifdef USING_LCD

  lcd.setCursor(7, 3);
  delay(10);
  lcd.print(vin);
  delay(10);
#endif USING_LCD
  return (vin);
}
////////////////////////////////////////////////////////////////////

void button_press() {
  digitalWrite(CTLPIN, LOW);  // set pin low to pull line high - button depressed
#ifdef USING_LCD
  lcd.setCursor(0, 2);
  lcd.print ("Pressing Button");
#endif USING_LCD
#ifdef DEBUG
  Serial.println("effecting button press");
#endif DEBUG
  delay(1000);
  digitalWrite(CTLPIN, HIGH);  // set pin high to pull line to GND - button released, triggers mode change
  delay_with_updates(15000);
#ifdef USING_LCD
  lcd.setCursor(0, 2);
  lcd.print ("               ");
#endif USING_LCD

}
///////////////////////////////////////////////////////////////////

void set_status(int stat_in) {

  stat = stat_in;   //load up global var
}

void show_status() {
  int x;
#ifndef USING_LCD
  for (x = 0; x < stat; x++) {
    digitalWrite(LEDPIN, HIGH);
    delay(BLINK_TIME);
    digitalWrite(LEDPIN, LOW);
    delay(BLINK_TIME);
  }
#endif USING_LCD

#ifdef USING_LCD

  switch (stat) {
    case STAT_UV:
      lcd.setCursor(0, 0);
      lcd.print ("UNV ");
      break;

    case STAT_RN:
      lcd.setCursor(0, 0);
      lcd.print ("RUN ");
      break;

    case STAT_FL:
      lcd.setCursor(0, 3);
      lcd.print ("FLT ");
      break;

    case STAT_PS:
      lcd.setCursor(0, 0);
      lcd.print ("OVL ");
      break;
      
    default:
      break;
  }
#endif USING_LCD
  delay(100);   // so we can see them separately

}
/////////////////////////////////////////////////////////////////

void delay_with_updates(long delay_value) {
  unsigned long start_t;
  start_t = millis();

  while ((millis() - start_t) < delay_value) {
    battery_voltage();     // keep display updated.
    show_status();
    delay(500);
  }

}

