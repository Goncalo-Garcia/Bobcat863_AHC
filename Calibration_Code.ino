#include <EEPROM.h>

// LINK: https://forum.arduino.cc/index.php?topic=532861.0
// ARDUINO NANO : https://components101.com/microcontrollers/arduino-nano
// ATMEGA328P DS: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf

#define ON    HIGH
#define OFF   LOW

#define DEBUG ON

#define TOTAL_SAMPLES_BIN 2  // 64 in rotation bits 2^n

#define FORWARD  HIGH
#define BACKWARD LOW

#define CALIBRATION_TILT           0  // EEPROM Register index for handle tilt calibration values
#define CALIBRATION_LIFT           5  // EEPROM Register index for handle lift calibration values
#define CALIBRATION_T_FDBACK       10 // EEPROM Register index for feedback calibration values
#define CALIBRATION_L_FDBACK       15 // EEPROM Register index for feedback calibration values

#define CALIBRATION_MIN            0  // Minimum offset

#define CALIBRATION_MID_L          1  // Medium  offset
#define CALIBRATION_MID_H          2  // Medium  offset

#define CALIBRATION_MAX_L          3  // Maximum offset
#define CALIBRATION_MAX_H          4  // Maximum offset

enum
{
  LIFT = 0,
  TILT,
  CONTRL_TOTAL
};

// LIFT ACTUATOR DEFINES
//-------------------------------------------------------------
#define HANDLE_L                 A0    // Pin of Right Handle potentiometer

#define ACTUATOR_L_FB            A1    // Pin of potentiometer Feedback of Right Actuator
#define ACTUATOR_L_SPEED         10    // 14 PB2 10 PWM signal to EN of motor driver L298N
#define ACTUATOR_L_PORT          PORTB // Port of right actuator pins
#define ACTUATOR_L_DIR           DDRB  // Dir  of right actuator pins
#define ACTUATOR_L_FORWARD_PIN   PINB4 // 16 PB4 12 Backwards enable pin to motor driver
#define ACTUATOR_L_BACKWARD_PIN  PINB3 // 15 PB3 11 Forwards  enable pin to motor driver

// TILT ACTUATOR DEFINES
//-------------------------------------------------------------
#define HANDLE_T                 A2    // Pin of Left Handle potentiometer

#define ACTUATOR_T_FB            A3    // Pin of potentiometer Feedback of Left Actuator
#define ACTUATOR_T_SPEED         5     // 9  PD5 5 PWM signal to EN of motor driver L298N
#define ACTUATOR_T_PORT          PORTD // Port of right actuator pins
#define ACTUATOR_T_DIR           DDRD  // Dir  of right actuator pins
#define ACTUATOR_T_FORWARD_PIN   PIND6 // 10 PD6 6 Backwards enable pin to motor driver
#define ACTUATOR_T_BACKWARD_PIN  PIND7 // 11 PD7 7 Forwards  enable pin to motor driver

//-------------------------------------------------------------

void     setDirection (uint8_t direction, uint8_t actuator);
uint16_t samples_read (uint8_t channel);

void setup() 
{
  Serial.begin(115200);

  analogReference(EXTERNAL); // 4V3 BobCat voltage reference
  
  // Set as Output
  ACTUATOR_L_DIR |= ((1 << ACTUATOR_L_FORWARD_PIN) | (1 << ACTUATOR_L_BACKWARD_PIN));
  ACTUATOR_T_DIR |= ((1 << ACTUATOR_T_FORWARD_PIN) | (1 << ACTUATOR_T_BACKWARD_PIN));

  Serial.println("########## CALIBRATION PROCEDURE ##########");
  Serial.println("-------------- Release  V1.4 --------------");
}

void loop() 
{
  uint16_t min, mid, max; 

  // LIFT FEEDBACK
  // Low-Neutral-High -> 0.4-1.72-3.0  85 -377-656
  Serial.println("FEEDBACK LIFT ACTUATOR CALIBRATION:");
  Serial.println("USE MULTIMETER TO READ FEEDBACK VOLTAGE:");
  Serial.println("SET MINIMUM ( 0.40V )");
  min = serialFeedBackProcess(LIFT);
  EEPROM.write(CALIBRATION_L_FDBACK + CALIBRATION_MIN,    (uint8_t)min);

  Serial.println("SET MEDIUM  ( 1.72V )");
  mid = serialFeedBackProcess(LIFT);
  EEPROM.write(CALIBRATION_L_FDBACK + CALIBRATION_MID_L,  (uint8_t)((mid >> 0) & 0x00FF));
  EEPROM.write(CALIBRATION_L_FDBACK + CALIBRATION_MID_H,  (uint8_t)((mid >> 8) & 0x00FF));

  Serial.println("SET MAXIMUM ( 3.00V )");
  max = serialFeedBackProcess(LIFT);
  EEPROM.write(CALIBRATION_L_FDBACK + CALIBRATION_MAX_L,  (uint8_t)((max >> 0) & 0x00FF));
  EEPROM.write(CALIBRATION_L_FDBACK + CALIBRATION_MAX_H,  (uint8_t)((max >> 8) & 0x00FF));

  
  Serial.println("ACTUATOR FEEDBACK LIFT CALIBRATION DONE!");
  Serial.println(" ");
  /*

  // TILT FEEDBACK
  
  // Low-Neutral-High -> 0.4-1.72-3.0  85 -377-656
  Serial.println("FEEDBACK TILT ACTUATOR CALIBRATION:");
  Serial.println("USE MULTIMETER TO READ FEEDBACK VOLTAGE:");
  Serial.println("SET MINIMUM ( 0.40V )");
  min = serialFeedBackProcess(TILT);
  EEPROM.write(CALIBRATION_T_FDBACK + CALIBRATION_MIN,    (uint8_t)min);

  Serial.println("SET MEDIUM  ( 1.72V )");
  mid = serialFeedBackProcess(TILT);
  EEPROM.write(CALIBRATION_T_FDBACK + CALIBRATION_MID_L,  (uint8_t)((mid >> 0) & 0x00FF));
  EEPROM.write(CALIBRATION_T_FDBACK + CALIBRATION_MID_H,  (uint8_t)((mid >> 8) & 0x00FF));  

  Serial.println("SET MAXIMUM ( 3.00V )");
  max = serialFeedBackProcess(TILT);
  EEPROM.write(CALIBRATION_T_FDBACK + CALIBRATION_MAX_L,  (uint8_t)((max >> 0) & 0x00FF));
  EEPROM.write(CALIBRATION_T_FDBACK + CALIBRATION_MAX_H,  (uint8_t)((max >> 8) & 0x00FF));
  
  Serial.println("ACTUATOR FEEDBACK TILT CALIBRATION DONE!");
  Serial.println(" ");  
*/
 
  // HANDLE
  // Low-Neutral-High -> 0.8-1.89-2.9  174-414-635
  Serial.println("LIFT HANDLE :");

  serialHandleProcess(LIFT);  

  Serial.println("LIFT HANDLE CALIBRATION DONE!");
  Serial.println(" ");
/* 
  Serial.println("TILT HANDLE :");

  serialHandleProcess(TILT);

  Serial.println("TILT HANDLE CALIBRATION DONE!");
  Serial.println(" ");
  */
  Serial.println("########## CALIBRATION PROCEDURE FINISH ##########");
  Serial.println("############  FLASH BOBCAT CONTROLLER  ###########");

  while(true); // FORCE CODE TO STOP
}

uint16_t serialHandleProcess(uint8_t handle)
{
  String   command = "";

  uint16_t min = 1024;
  uint16_t mid = 0, max = 0;

  uint16_t aux = 0;

  uint8_t adcChannel     = HANDLE_L;
  uint8_t eepromRegister = CALIBRATION_LIFT;

  if(handle == TILT)
  {
    adcChannel     = HANDLE_T;
    eepromRegister = CALIBRATION_TILT;
  }

  Serial.println("SET HANDLE TO LOW POSITION ( 0.80V )");
  while(true)
  {
    aux = samples_read(adcChannel);

    // If new min value found tell the user
    if (aux < min)
    {
      min = aux;
      Serial.print("MIN = "); Serial.println(min);
    }

    // Read Serial port for command
    if((Serial.available() > 0))
    {      
      command = Serial.readString();

      if(command.indexOf("OK") > -1)
      {
        break;
      }
    }
    // Only get out of the while loop when OK is recieved
    // Read Serial port for command
    if((Serial.available() > 0))
    {      
      command = Serial.readString();

      if(command.indexOf("OK") > -1)
      {
        break;
      }
    }
  }

  Serial.println("SET HANDLE TO HIGH POSITION ( 2.90V )");
  while(true)
  {
    aux = samples_read(adcChannel);

    // If new min value found tell the user
    if (aux > max)
    {
      max = aux;
      Serial.print("MAX = "); Serial.println(max);
    }

    // Only get out of the while loop when OK is recieved
    // Read Serial port for command
    if((Serial.available() > 0))
    {      
      command = Serial.readString();

      if(command.indexOf("OK") > -1)
      {
        break;
      }
    }
  }

  Serial.println("LEAVE HANDLE IN MIDDLE POSITION ( 1.89V )");
  while(true)
  {
    aux = samples_read(adcChannel);

    // If new min value found tell the user
    if (aux != mid)
    {
      mid = aux;
      Serial.print("MID = "); Serial.println(mid);
    }

    // Only get out of the while loop when OK is recieved
    // Read Serial port for command
    if((Serial.available() > 0))
    {      
      command = Serial.readString();

      if(command.indexOf("OK") > -1)
      {
        break;
      }
    }
  }

  EEPROM.write(eepromRegister + CALIBRATION_MIN,    (uint8_t)((min >> 0) & 0x00FF));
  
  EEPROM.write(eepromRegister + CALIBRATION_MID_L,  (uint8_t)((mid >> 0) & 0x00FF));
  EEPROM.write(eepromRegister + CALIBRATION_MID_H,  (uint8_t)((mid >> 8) & 0x00FF));

  EEPROM.write(eepromRegister + CALIBRATION_MAX_L,  (uint8_t)((max >> 0) & 0x00FF));
  EEPROM.write(eepromRegister + CALIBRATION_MAX_H,  (uint8_t)((max >> 8) & 0x00FF));
  
  Serial.println("VALUES SAVED");
}

uint16_t serialFeedBackProcess(uint8_t actuator)
{
  String   command = "";

  uint8_t adcChannel = ACTUATOR_L_FB;
  uint8_t pwmChannel = ACTUATOR_L_SPEED;

  if(actuator == TILT)
  {
    adcChannel = ACTUATOR_T_FB;
    pwmChannel = ACTUATOR_T_SPEED;
  }

  while(true)
  {
    // Read Serial port for command
    if((Serial.available() > 0))
    {      
      command = Serial.readString();

      if(command.indexOf("OK") > -1)
      {
        break;
      }
      
      //Move the actuator forward a small distance
      if(command.substring(0) == "+")
      {
        Serial.println("ACTUATOR MOVE FORWARD");

        setDirection (FORWARD, actuator);
        //analogWrite  (pwmChannel, 255);
        //delay(25); // wait 50ms
        analogWrite  (pwmChannel, 0 );  // Stop the actuator

        Serial.print("ADC = "); Serial.println(samples_read(adcChannel));
      }

      //Move the actuator forward a big distance
      if(command.substring(0) == "++")
      {
        Serial.println("ACTUATOR MOVE FORWARD");

        setDirection (FORWARD, actuator);
        analogWrite  (pwmChannel, 255);
        //delay(50); // wait 50ms
        //analogWrite  (pwmChannel, 0 );  // Stop the actuator

        Serial.print("ADC = "); Serial.println(samples_read(adcChannel));
      }

      //Move the actuator backwards a small distance
      if(command.substring(0) == "-")
      {
        Serial.println("ACTUATOR MOVE BACKWARDS");

        setDirection (BACKWARD, actuator);
        //analogWrite  (pwmChannel, 255);
        //delay(25); // wait 50ms
        analogWrite  (pwmChannel, 0 );  // Stop the actuator

        Serial.print("ADC = "); Serial.println(samples_read(adcChannel));
      }

      //Move the actuator backwards a big distance
      if(command.substring(0) == "--")
      {
        Serial.println("ACTUATOR MOVE BACKWARDS");

        setDirection (BACKWARD, actuator);
        analogWrite  (pwmChannel, 255);
        //delay(50); // wait 50ms
        //analogWrite  (pwmChannel, 0 );  // Stop the actuator
        
        Serial.print("ADC = "); Serial.println(samples_read(adcChannel));
      }
    }    
  }

  analogWrite  (pwmChannel, 0 );  // Stop the actuator
  Serial.println("SAVE!");

  return samples_read(adcChannel);
}

uint16_t samples_read(uint8_t channel)
{
  uint32_t average = 0;
  uint8_t  samples = 0;
  
  do
  {
    samples += 1;
    average += analogRead(channel);
  }
  while (samples < (1 << TOTAL_SAMPLES_BIN));
  
  average = (average >> TOTAL_SAMPLES_BIN);
  
  return (uint16_t)average;  
}

void setDirection(uint8_t direction, uint8_t actuator)
{
  switch (actuator)
  {
    case LIFT:
    {
      if (direction == FORWARD)
      { 
        ACTUATOR_L_PORT |=  (1 << ACTUATOR_L_FORWARD_PIN );
        ACTUATOR_L_PORT &= ~(1 << ACTUATOR_L_BACKWARD_PIN);
      }
      else
      {
        ACTUATOR_L_PORT &= ~(1 << ACTUATOR_L_FORWARD_PIN );
        ACTUATOR_L_PORT |=  (1 << ACTUATOR_L_BACKWARD_PIN); 
      }      
    }
    break;

    case TILT:
    {
      if (direction == FORWARD)
      { 
        ACTUATOR_T_PORT |=  (1 << ACTUATOR_T_FORWARD_PIN );
        ACTUATOR_T_PORT &= ~(1 << ACTUATOR_T_BACKWARD_PIN);
      }
      else
      {
        ACTUATOR_T_PORT &= ~(1 << ACTUATOR_T_FORWARD_PIN );
        ACTUATOR_T_PORT |=  (1 << ACTUATOR_T_BACKWARD_PIN); 
      }
    }
    break;
  }
}
