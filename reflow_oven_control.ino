//Adafruit Feather code for Rocksat 2019 Soldering Project

//Library Includes
#include <SPI.h>
#include <SD.h>
#include "Adafruit_MAX31855.h"

//Hardware definitions
#define fan_pwm 12
#define fan_tach 11
#define therm_0_cs 0
#define therm_1_cs 1
#define therm_2_cs 10
#define fan_enable 9
#define heat_enable 8
#define g_sw 5
#define temp_amb 0
#define led_g 8
#define led_r 13
#define sd_cs 4

//Calibration Values
#define tmp_36_voltage_offset  -0.05;

//Globals
File logfile;
unsigned long previousMillis = 0;
int fanDuty = 0;
boolean gswitchActive = false;
unsigned long gswitchtime = 0;
Adafruit_MAX31855 thermocouple0(therm_0_cs);
Adafruit_MAX31855 thermocouple1(therm_1_cs);
Adafruit_MAX31855 thermocouple2(therm_2_cs);

//Function Declarations
//Interrupt Service Routines
void gswitchISR()
{
  gswitchtime = millis();
  gswitchActive = true;
  detachInterrupt(digitalPinToInterrupt(g_sw));
}

//Fan Functions
void setup_pwm_fan()
{
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(48) | // Divide the 48 MHz clock source by 48 to get 1 MHz Generic Clock
                    GCLK_GENDIV_ID(4);    // Select Generic Clock (GCLK) 4
  while(GCLK->STATUS.bit.SYNCBUSY);       // Wait for synchronisation

  //Set the source for GCLK 4
  REG_GCLK_GENCTRL = GCLK_GENCTRL_GENEN |        // Enable GCLK 4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48 MHz clock source
                     GCLK_GENCTRL_ID(4);         // Select GCLK 4
  while(GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronisation

  //We want fan PWM on feather pin 12 aka PA19 at 25 kHz (TC3 pg 29 datasheet)
  //Feed GCLK4 to TC3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable CLKCTRL
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK3 to TCC2 and TC3
  while(GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronisation

  //Enable the port multiplexer for pin 12
  PORT->Group[g_APinDescription[12].ulPort].PINCFG[g_APinDescription[12].ulPin].bit.PMUXEN = 1;

  //Connect the TCC timers to the port outputs
  PORT->Group[g_APinDescription[12].ulPort].PMUX[g_APinDescription[12].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;

  //Set waveform to single slope PWM operation
  REG_TC3_CTRLA |= TC_CTRLA_WAVEGEN_NPWM | // Enable normal slope PWM on TC3
                   TC_CTRLA_MODE_COUNT8;   // Enable 8 bit count mode
  while(TC3->COUNT8.STATUS.bit.SYNCBUSY);  // Wait for synchronisation

  REG_TC3_COUNT8_PER = 40; // 1 MHz / 40 = 25 kHz
  while(TC3->COUNT8.STATUS.bit.SYNCBUSY); //Wait for synchronisation

  //Set the duty cycle (capture/compare register 30.8.14.1)
  REG_TC3_COUNT8_CC1 = 0;
  while(TC3->COUNT8.STATUS.bit.SYNCBUSY);

  REG_TC3_CTRLA |= TC_CTRLA_ENABLE; //Enable the TC3 output
  Serial.println("Fan PWM setup complete");
}

//Set the fan PWM duty cycle - values are valid from 20% to 100%, above and below will be clipped
void set_pwm_fan(int dutyCycle)
{
  int setds;
  if(dutyCycle < 20)
  {
    REG_TC3_CTRLA &= !TC_CTRLA_ENABLE; //Disable the PWM below 20 % duty
    return;
  }
  else if(dutyCycle > 100)
  {
    setds = 40;
  }
  else
  {
    setds = map(dutyCycle, 0, 100, 0, 40);
  }
  REG_TC3_COUNT8_CC1 = setds;
  while(TC3->COUNT8.STATUS.bit.SYNCBUSY);
  REG_TC3_CTRLA |= TC_CTRLA_ENABLE; //Enable the PWM output
}

//TMP36 Functions
//A function to get the temperature and voltage reading from the TMP36 sensor
//Arguments: a 2 element float array, element 0 is temperature and element 1 is voltage
void getTMP36(float *data)
{
  data[0] = (analogRead(temp_amb)*0.0032258) + tmp_36_voltage_offset; //Scaling factor: (3.3/1023) = 0.0032258
  data[1] = 100.0*data[0] - 50.0;
}

void setup()
{
  //Set up hardware pins
  pinMode(fan_pwm, OUTPUT);
  pinMode(fan_tach, INPUT);
  pinMode(therm_0_cs, OUTPUT);
  pinMode(therm_1_cs, OUTPUT);
  pinMode(therm_2_cs, OUTPUT);
  pinMode(fan_enable, OUTPUT); //ACTIVE HIGH
  pinMode(heat_enable, OUTPUT); //ACTIVE HIGH
  pinMode(g_sw, INPUT_PULLUP); //ACTIVE LOW - check input pullup before flight
  pinMode(led_g, OUTPUT);
  pinMode(led_r, OUTPUT);
  //Initialise hardware pin states
  digitalWrite(fan_pwm, LOW);
  digitalWrite(therm_0_cs, HIGH); //thermocouple CS is active LOW
  digitalWrite(therm_1_cs, HIGH);
  digitalWrite(therm_2_cs, HIGH);
  digitalWrite(fan_enable, LOW);
  digitalWrite(heat_enable, LOW);
  digitalWrite(led_g, LOW);
  digitalWrite(led_r, LOW);
  //Enable hardware interrupts
  attachInterrupt(digitalPinToInterrupt(g_sw), gswitchISR, FALLING);
  //Set up serial communication
  Serial.begin(115200);
  while(!Serial) //REMOVE BEFORE FLIGHT
  {
    digitalWrite(led_g, HIGH);
    digitalWrite(led_r, HIGH);
  }
  digitalWrite(led_g, LOW);
  digitalWrite(led_r, LOW);
  Serial.println("Setup started");
  //Set up SD card stuff
  if(!SD.begin(sd_cs))
  {
    Serial.println("Card initialisation failed");
    digitalWrite(led_r, HIGH);
    for(;;);
  }
  Serial.println("SD Card Initialized");
  char fname[15];
  strcpy(fname, "rslog000.csv"); //must use 8.3 filenames
  for(uint8_t i = 0; i < 1000; i++)
  {
    fname[5] = '0' + i/100;
    fname[6] = '0' + (i%100)/10;
    fname[7] = '0' + i%10;
    if(!SD.exists(fname))
    {
      break;
    }
    else if (i >= 999)
    {
      Serial.println("Somehow there are already 1000 log files. File creation failed.");
      digitalWrite(led_r, HIGH);
      for(;;);
    }
  }
  logfile = SD.open(fname, FILE_WRITE);
  if(!logfile)
  {
    Serial.print("Failed to create ");
    Serial.println(fname);
    digitalWrite(led_r, HIGH);
    for(;;);
  }
  Serial.print("Created logfile ");
  Serial.println(fname);
  logfile.println("MILLIS,GSWITCH,TEMP1,TEMP2,TEMP3,TEMP1I,TEMP2I,TEMP3I,TEMPA,FAN,HEAT"); //CSV header
  //logfile.close();
  logfile.flush();
  digitalWrite(led_g, HIGH);
  //Initialise the fan
  digitalWrite(fan_enable, HIGH);
  setup_pwm_fan();
  set_pwm_fan(0);
  //Initialise timing
  previousMillis = millis();
  //Finish setup
  Serial.println("Setup complete");
}

void loop()
{
  if(millis() - previousMillis >= 800)
  {
    previousMillis = millis();
    digitalWrite(led_r, !digitalRead(led_r));
    Serial.print("Amplifier Internal Temp = ");
    Serial.println(thermocouple0.readInternal());
    double temp1 = thermocouple0.readCelsius();
    double temp2 = 0.0;
    double temp3 = 0.0;
    double tempI1 = 0.0;
    double tempI2 = 0.0;
    double tempI3 = 0.0;
    if (isnan(temp1))
    {
     Serial.println("Something wrong with thermocouple!");
    }
    else
    {
     Serial.print("C = "); 
     Serial.println(temp1);
   }
   float tmp36data[2];
   getTMP36(tmp36data);
   //float vtmp36 = (analogRead(temp_amb)*0.0032258) + tmp_36_voltage_offset; //Scaling factor: (3.3/1023) = 0.0032258
   Serial.print("TMP 36 Voltage: ");
   Serial.print(tmp36data[1]);
   Serial.println(" V");
   //float ttmp36 = 100.0*vtmp36 - 50.0;
   Serial.print("TMP 36 Temperature: ");
   Serial.print(tmp36data[0]);
   Serial.println(" C");
   //String line = "" + millis() +',' + (gswitchActive ? '1' : '0') + ',' + temp1 + ',' + temp2 + ',' + temp3 + ',' + tempI1 + ',' + tempI2 + ',' + tempI3 + ',' + tmp36data[0] + ',' + '0' + ',' + '0';
   logfile.print(millis());
   logfile.print(',');
   logfile.print(gswitchActive ? '1' : '0');
   logfile.print(',');
   logfile.print(temp1);
   logfile.print(',');
   logfile.print(temp2);
   logfile.print(',');
   logfile.print(temp3);
   logfile.print(',');
   logfile.print(tempI1);
   logfile.print(',');
   logfile.print(tempI2);
   logfile.print(',');
   logfile.print(tempI3);
   logfile.print(',');
   logfile.print(tmp36data[0]);
   logfile.print(',');
   logfile.print('0');
   logfile.print(',');
   logfile.println('0');
   logfile.flush();
   Serial.println();
  }
}
