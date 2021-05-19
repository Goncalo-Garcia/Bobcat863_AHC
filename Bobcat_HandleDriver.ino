#include <EEPROM.h>
#include <PID_v1.h>

// LINK          : https://forum.arduino.cc/index.php?topic=532861.0
// ARDUINO NANO  : https://components101.com/microcontrollers/arduino-nano
// ATMEGA328P DS : https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
// PID LIB       : https://github.com/br3ttb/Arduino-PID-Library
// PID LIB       : https://github.com/br3ttb/Arduino-PID-AutoTune-Library

#define ON  HIGH
#define OFF LOW

#define SAFETY_BAR         ON    // Enables the code that checks if the safety bar is ON or OFF
#define SAFETY_DEBOUNCE    100    // (100ms) Debounce time
#define SAFETY_TIMEOUT     5000   // (5s) 5 seconds window that disables safety feature

#define EEPROM_DUMP OFF    // This outputs the values stored on the eeprom memory

#define PLOTTE_TILT OFF    // This enables the serial code to plot the graph of the feedback signal
#define PLOTTE_LIFT OFF    // This enables the serial code to plot the graph of the feedback signal

#define SERIAL_TILT OFF    // This enables the serial code to set the PID values kp, ki, kd
#define SERIAL_LIFT OFF    // This enables the serial code to set the PID values kp, ki, kd

#define FORWARD  HIGH
#define BACKWARD LOW

#define SAMPLE_TIME       30 // Sample time between pid computations

#define HANDLE_MID_TH     0    // Middle point threshold
#define FDBACK_MID_TH     0   // Middle point threshold

// EEPROM REGISTERS ADDRESS
//--------------------------------------------------------------------------------------------------------
#define CALIBRATION_TILT           0  // EEPROM Register index for handle tilt calibration values
#define CALIBRATION_LIFT           5  // EEPROM Register index for handle lift calibration values
#define CALIBRATION_T_FDBACK       10 // EEPROM Register index for feedback calibration values
#define CALIBRATION_L_FDBACK       15 // EEPROM Register index for feedback calibration values

#define CALIBRATION_MIN            0  // Minimum offset

#define CALIBRATION_MID_L          1  // Medium  offset
#define CALIBRATION_MID_H          2  // Medium  offset

#define CALIBRATION_MAX_L          3  // Maximum offset
#define CALIBRATION_MAX_H          4  // Maximum offset

//--------------------------------------------------------------------------------------------------------

enum
{
  // HANDLE
  // Low-Neutral-High -> 0.8-1.89-2.9  174-414-635
  HANDLE_L_MIN     = 0, // ~174
  HANDLE_L_MID        , // ~414
  HANDLE_L_MAX        , // ~635
  HANDLE_L_MID_TH_HI  , // (HANDLE_MID + HANDLE_MID_TH)
  HANDLE_L_MID_TH_LW  , // (HANDLE_MID - HANDLE_MID_TH
  HANDLE_L_DELTA_LW   , // (HANDLE_MID_TH_HI  - HANDLE_MIN)
  HANDLE_L_DELTA_HI   , // (HANDLE_MAX - HANDLE_MID_TH_HI)

  // HANDLE
  // Low-Neutral-High -> 0.8-1.89-2.9  174-414-635
  HANDLE_T_MIN        , // ~174
  HANDLE_T_MID        , // ~414
  HANDLE_T_MAX        , // ~635
  HANDLE_T_MID_TH_HI  , // (HANDLE_MID + HANDLE_MID_TH)
  HANDLE_T_MID_TH_LW  , // (HANDLE_MID - HANDLE_MID_TH
  HANDLE_T_DELTA_LW   , // (HANDLE_MID_TH_HI  - HANDLE_MIN)
  HANDLE_T_DELTA_HI   , // (HANDLE_MAX - HANDLE_MID_TH_HI)

  // FEEDBACK
  // Low-Neutral-High -> 0.4-1.72-3.0  85-377-656
  FDBACK_L_MIN          , // ~85
  FDBACK_L_MID          , // ~377
  FDBACK_L_MAX          , // ~656
  FDBACK_L_MID_TH_HI    , // (FDBACK_MID + FDBACK_MID_TH)
  FDBACK_L_MID_TH_LW    , // (FDBACK_MID - FDBACK_MID_TH)
  FDBACK_L_DELTA_LW     , // (FDBACK_MID_TH_HI  - FDBACK_MIN)
  FDBACK_L_DELTA_HI     , // (FDBACK_MAX - FDBACK_MID_TH_HI)

  // FEEDBACK
  // Low-Neutral-High -> 0.4-1.72-3.0  85-377-656
  FDBACK_T_MIN          , // ~85
  FDBACK_T_MID          , // ~377
  FDBACK_T_MAX          , // ~656
  FDBACK_T_MID_TH_HI    , // (FDBACK_MID + FDBACK_MID_TH)
  FDBACK_T_MID_TH_LW    , // (FDBACK_MID - FDBACK_MID_TH)
  FDBACK_T_DELTA_LW     , // (FDBACK_MID_TH_HI  - FDBACK_MIN)
  FDBACK_T_DELTA_HI     , // (FDBACK_MAX - FDBACK_MID_TH_HI)

  TOTAL_REGISTERS
};

// Registers index offsets to use in converterAdc function
enum
{
  MIN     = 0,
  MID        ,
  MAX        ,
  MID_TH_HI  ,
  MID_TH_LW  ,
  DELTA_LW   ,
  DELTA_HI   ,
};

uint16_t REGISTERS [TOTAL_REGISTERS] = { 0 };

enum
{
  LIFT = 0,
  TILT,
  CONTRL_TOTAL
};

// SAFETY BAR DEFINES
//-------------------------------------------------------------
#define AUX_INPIN               3       // Pin of aux switch
#define AUX_OUTPORT             PORTB   // Port of aux valve
#define AUX_OUTDIR              DDRB    // Dir of aux valve
#define AUX_OUTPIN              PINB0   // Pin of aux valve

// SAFETY BAR DEFINES
//-------------------------------------------------------------
#define SAFETY_PIN               2     // Pin of Right Handle potentiometer 
#define SAFETY_INPUT             PIND // Port of right actuator pins 

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
#define ACTUATOR_T_FORWARD_PIN   PIND7 // 10 PD6 6 Backwards enable pin to motor driver
#define ACTUATOR_T_BACKWARD_PIN  PIND6 // 11 PD7 7 Forwards  enable pin to motor driver

//-------------------------------------------------------------

// Variaveis para o safety bar pin
volatile uint8_t  Safety_State     = OFF;
volatile uint32_t Safety_Reference = 0; 

// Variaveis para o auxiliar
volatile uint8_t  Aux_State     = OFF;
volatile uint32_t Aux_Reference = 0; 

// Variaveis pa7ra os controladores PID
double Tilt_SetPoint, Tilt_FeedBack, Tilt_OutPut;
double Lift_SetPoint, Lift_FeedBack, Lift_OutPut;

// PID tunings for TILT
double TILT_KP = 0.4;
double TILT_KI = 0.05;
double TILT_KD = 0.01;

// PID tunings for LIFT
double LIFT_KP = 0.65;
double LIFT_KI = 0.1;
double LIFT_KD = 0.02;

// Controladores PID
PID tilt_PID(&Tilt_FeedBack, &Tilt_OutPut, &Tilt_SetPoint, TILT_KP, TILT_KI, TILT_KD, DIRECT);
PID lift_PID(&Lift_FeedBack, &Lift_OutPut, &Lift_SetPoint, LIFT_KP, LIFT_KI, LIFT_KD, DIRECT);

///////////////////////////////////////////////////////
void     readADCs (void);

uint16_t converterAdc(uint16_t adcRead, uint8_t index);

void     readCalibrationValues(void);

void     readSerialPort(uint8_t actuator);

void     readSafetyPin();

void     readAuxPin();

void setup()
{
  Serial.begin(115200);

  analogReference(EXTERNAL); // 4V3 BobCat voltage reference

  readCalibrationValues();

  // Set as OutPut the direction ins of actuators
  ACTUATOR_L_DIR |= ((1 << ACTUATOR_L_FORWARD_PIN) | (1 << ACTUATOR_L_BACKWARD_PIN));
  ACTUATOR_T_DIR |= ((1 << ACTUATOR_T_FORWARD_PIN) | (1 << ACTUATOR_T_BACKWARD_PIN));
  
  lift_PID.SetSampleTime  (SAMPLE_TIME);
  tilt_PID.SetSampleTime  (SAMPLE_TIME);

  lift_PID.SetOutputLimits(-255 , 255);
  tilt_PID.SetOutputLimits(-255 , 255);

  //turn the PID on
  lift_PID.SetMode(AUTOMATIC);
  tilt_PID.SetMode(AUTOMATIC);
  
#if (SAFETY_BAR == ON)

  pinMode(SAFETY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SAFETY_PIN), readSafetyPin, CHANGE);
  Safety_Reference = millis();
  
  Safety_State   = (SAFETY_INPUT & (1 << SAFETY_PIN)) ? (ON) : (OFF);

  pinMode(AUX_INPIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(AUX_INPIN), readAuxPin, RISING);
  Aux_Reference = millis();

  // Set as OutPut the direction ins of actuators
  AUX_OUTDIR |= (1 << AUX_OUTPIN);
  
  // Set pin foward to high
  AUX_OUTPORT |=  (1 << AUX_OUTPIN); 
  
#endif

#if ((PLOTTE_LIFT == OFF) && (PLOTTE_TILT == OFF))

  Serial.println("########## BOBCAT ARDUINO A.H.C/PWM CONTROLLER ##########");
  Serial.println("---------------------      V2.75      ---------------------");

#endif
}

void loop()
{
  #if (SAFETY_BAR == ON)

    if (Safety_State == ON)
    {
      lift_Loop();
      tilt_Loop();
    }
    else
    {
      // Set the setpoints in the middle
      Tilt_SetPoint = 2000;
      Lift_SetPoint = 2000;
      
      // Disables the actuatores drivers 
      analogWrite(ACTUATOR_L_SPEED, 0);
      analogWrite(ACTUATOR_T_SPEED, 0);
    }
  
  #else

    lift_Loop();
    tilt_Loop();
  
  #endif


  #if (SERIAL_LIFT == ON)
    readSerialPort(LIFT);
  #endif

  #if (SERIAL_TILT == ON)
    readSerialPort(TILT);
  #endif
}

void lift_Loop()
{
  read_LIFT_ADCs();

  if (lift_PID.Compute())
  {
    setDirection(Lift_OutPut, LIFT);

    #if (PLOTTE_TILT == ON)
      double test = Lift_OutPut;
    #endif

    Lift_OutPut = abs(Lift_OutPut);

    // Abaixo de 65 o motor não anda por causa das molas
    analogWrite(ACTUATOR_L_SPEED, Lift_OutPut);  // Set PWM speed value to adjust position
    
    #if (PLOTTE_LIFT == ON)
          // This is for plotter analysis
          //Serial.print("L_S : "); 
          Serial.print(Lift_SetPoint);
          Serial.print(" ");
          //Serial.print("L_F : "); 
          Serial.print(Lift_FeedBack);
          Serial.print(" "); 
          //Serial.print("L_O : "); 
          Serial.println(test); 
    #endif
  }
}

void tilt_Loop()
{
  read_TILT_ADCs();

  if (tilt_PID.Compute())
  {
    setDirection(Tilt_OutPut, TILT);

    #if (PLOTTE_TILT == ON)
      double test = Tilt_OutPut;
    #endif

    Tilt_OutPut = abs(Tilt_OutPut);
    
    // Abaixo de 65 o motor não anda por causa das molas
    analogWrite(ACTUATOR_T_SPEED, Tilt_OutPut);  // Set PWM speed value to adjust position

    #if (PLOTTE_TILT == ON)
          // This is for plotter analysis
          //Serial.print("T_S : "); 
          Serial.print(Tilt_SetPoint);
          Serial.print(" ");
          //Serial.print("T_F : "); 
          Serial.print(Tilt_FeedBack);
          Serial.print(" ");
          //Serial.print("T_O : "); 
          Serial.println(test); 
    #endif
  }
}

void read_LIFT_ADCs(void)
{
  Lift_SetPoint = ReadADC(HANDLE_L);
  Lift_FeedBack = ReadADC(ACTUATOR_L_FB);

  Lift_SetPoint = converterAdc(Lift_SetPoint, HANDLE_L_MIN);
  Lift_FeedBack = converterAdc(Lift_FeedBack, FDBACK_L_MIN);
}

void read_TILT_ADCs(void)
{
  Tilt_SetPoint = ReadADC(HANDLE_T);
  Tilt_FeedBack = ReadADC(ACTUATOR_T_FB);

  Tilt_SetPoint = converterAdc(Tilt_SetPoint, HANDLE_T_MIN);
  Tilt_FeedBack = converterAdc(Tilt_FeedBack, FDBACK_T_MIN);
}

uint16_t ReadADC(uint8_t ADCchannel)
{
  ADCchannel -= 14;     // Por causa das declarações do arduino
  
  //select ADC channel with safety mask
  ADMUX  = (ADMUX & 0xF0) | (ADCchannel & 0x0F);  // Set the new channel bit
  
  //single conversion mode
  ADCSRA |= (1 << ADSC);
  
  // wait until ADC conversion is complete
  while ( ADCSRA & (1 << ADSC) ); 

  // Divide pelo total de amostras e retorna a média
  return ADC;
}

inline void setDirection(double error, uint8_t actuator)
{
  uint8_t cont = 0;
  
  switch (actuator)
  {
    case LIFT:
      {
        // Clear previus pin state to low
        ACTUATOR_L_PORT &= ~((1 << ACTUATOR_L_BACKWARD_PIN) | (1 << ACTUATOR_L_FORWARD_PIN ));

        if (error >= 0)
        {
          // Set pin foward to high
          ACTUATOR_L_PORT |=  (1 << ACTUATOR_L_FORWARD_PIN );          
        }
        else
        {
          // Set pin backward to high
          ACTUATOR_L_PORT |=  (1 << ACTUATOR_L_BACKWARD_PIN);
        }
      }
      break;

    case TILT:
      {
        // Clear previus pin state to low
        ACTUATOR_T_PORT &= ~((1 << ACTUATOR_T_BACKWARD_PIN) | (1 << ACTUATOR_T_FORWARD_PIN ));
        
        if (error >= 0)
        {  
          // Set pin foward to high        
          ACTUATOR_T_PORT |=  (1 << ACTUATOR_T_FORWARD_PIN );          
        }
        else
        {
          // Set pin backward to high
          ACTUATOR_T_PORT |=  (1 << ACTUATOR_T_BACKWARD_PIN);
        }
      }
      break;
  }
}

uint16_t converterAdc(uint16_t adcRead, uint8_t index)
{  
  if (adcRead >= REGISTERS[index + MAX])
  {
  return 3000;  // Higher point value
  }
  
  if (adcRead <= REGISTERS[index + MIN])
  {
    return 1000; // Lower point value
  }

  if ((adcRead >= REGISTERS[index + MID_TH_LW]) && (adcRead <= REGISTERS[index + MID_TH_HI]))
  {
    return 2000; // Middle point value
  }
  else
  {
    if (adcRead < REGISTERS[index + MID_TH_LW] && adcRead >= REGISTERS[index + MIN])
    {
      return ((((double)adcRead - REGISTERS[index + MIN])       / REGISTERS[index + DELTA_LW]) + 1) * 1000;
    }
    else
    {
      return ((((double)adcRead - REGISTERS[index + MID_TH_HI]) / REGISTERS[index + DELTA_HI]) + 2) * 1000;
    }
  }
}

void readCalibrationValues(void)
{
  // Read LIFT Handle calibrations values
  REGISTERS[HANDLE_L_MIN] = (uint16_t)   EEPROM.read(CALIBRATION_LIFT + CALIBRATION_MIN);

  REGISTERS[HANDLE_L_MID] = (uint16_t)(((EEPROM.read(CALIBRATION_LIFT + CALIBRATION_MID_H)   << 8) & 0xFF00) |
                                       ((EEPROM.read(CALIBRATION_LIFT + CALIBRATION_MID_L)   << 0) & 0x00FF));

  REGISTERS[HANDLE_L_MAX] = (uint16_t)(((EEPROM.read(CALIBRATION_LIFT + CALIBRATION_MAX_H)   << 8) & 0xFF00) |
                                       ((EEPROM.read(CALIBRATION_LIFT + CALIBRATION_MAX_L)   << 0) & 0x00FF));

  // Read TILT Handle calibrations values
  REGISTERS[HANDLE_T_MIN] = (uint16_t)   EEPROM.read(CALIBRATION_TILT + CALIBRATION_MIN);

  REGISTERS[HANDLE_T_MID] = (uint16_t)(((EEPROM.read(CALIBRATION_TILT + CALIBRATION_MID_H)   << 8) & 0xFF00) |
                                       ((EEPROM.read(CALIBRATION_TILT + CALIBRATION_MID_L)   << 0) & 0x00FF));

  REGISTERS[HANDLE_T_MAX] = (uint16_t)(((EEPROM.read(CALIBRATION_TILT + CALIBRATION_MAX_H)   << 8) & 0xFF00) |
                                       ((EEPROM.read(CALIBRATION_TILT + CALIBRATION_MAX_L)   << 0) & 0x00FF));

  // Read LIFT Actuator calibrations values
  REGISTERS[FDBACK_L_MIN]   = (uint16_t)   EEPROM.read(CALIBRATION_L_FDBACK + CALIBRATION_MIN);
  
  REGISTERS[FDBACK_L_MID]   = (uint16_t)(((EEPROM.read(CALIBRATION_L_FDBACK + CALIBRATION_MID_H)   << 8) & 0xFF00) |
                                         ((EEPROM.read(CALIBRATION_L_FDBACK + CALIBRATION_MID_L)   << 0) & 0x00FF));

  REGISTERS[FDBACK_L_MAX]   = (uint16_t)(((EEPROM.read(CALIBRATION_L_FDBACK + CALIBRATION_MAX_H)   << 8) & 0xFF00) |
                                         ((EEPROM.read(CALIBRATION_L_FDBACK + CALIBRATION_MAX_L)   << 0) & 0x00FF));

  // Read TILT Actuator calibrations values
  REGISTERS[FDBACK_T_MIN]   = (uint16_t)   EEPROM.read(CALIBRATION_T_FDBACK + CALIBRATION_MIN);

  REGISTERS[FDBACK_T_MID]   = (uint16_t)(((EEPROM.read(CALIBRATION_T_FDBACK + CALIBRATION_MID_H)   << 8) & 0xFF00) |
                                         ((EEPROM.read(CALIBRATION_T_FDBACK + CALIBRATION_MID_L)   << 0) & 0x00FF));

  REGISTERS[FDBACK_T_MAX]   = (uint16_t)(((EEPROM.read(CALIBRATION_T_FDBACK + CALIBRATION_MAX_H)   << 8) & 0xFF00) |
                                         ((EEPROM.read(CALIBRATION_T_FDBACK + CALIBRATION_MAX_L)   << 0) & 0x00FF));                                     

  REGISTERS[HANDLE_L_MID_TH_HI] = REGISTERS[HANDLE_L_MID] + HANDLE_MID_TH;
  REGISTERS[HANDLE_L_MID_TH_LW] = REGISTERS[HANDLE_L_MID] - HANDLE_MID_TH ;
  REGISTERS[HANDLE_L_DELTA_LW]  = REGISTERS[HANDLE_L_MID_TH_LW] - REGISTERS[HANDLE_L_MIN];
  REGISTERS[HANDLE_L_DELTA_HI]  = REGISTERS[HANDLE_L_MAX] - REGISTERS[HANDLE_L_MID_TH_HI];

  REGISTERS[HANDLE_T_MID_TH_HI] = REGISTERS[HANDLE_T_MID] + HANDLE_MID_TH;
  REGISTERS[HANDLE_T_MID_TH_LW] = REGISTERS[HANDLE_T_MID] - HANDLE_MID_TH;
  REGISTERS[HANDLE_T_DELTA_LW]  = REGISTERS[HANDLE_T_MID_TH_LW] - REGISTERS[HANDLE_T_MIN];
  REGISTERS[HANDLE_T_DELTA_HI]  = REGISTERS[HANDLE_T_MAX] - REGISTERS[HANDLE_T_MID_TH_HI];

  REGISTERS[FDBACK_L_MID_TH_HI] = REGISTERS[FDBACK_L_MID] + FDBACK_MID_TH;
  REGISTERS[FDBACK_L_MID_TH_LW] = REGISTERS[FDBACK_L_MID] - FDBACK_MID_TH;
  REGISTERS[FDBACK_L_DELTA_LW]  = REGISTERS[FDBACK_L_MID_TH_LW] - REGISTERS[FDBACK_L_MIN];
  REGISTERS[FDBACK_L_DELTA_HI]  = REGISTERS[FDBACK_L_MAX] - REGISTERS[FDBACK_L_MID_TH_HI];

  REGISTERS[FDBACK_T_MID_TH_HI] = REGISTERS[FDBACK_T_MID] + FDBACK_MID_TH;
  REGISTERS[FDBACK_T_MID_TH_LW] = REGISTERS[FDBACK_T_MID] - FDBACK_MID_TH;
  REGISTERS[FDBACK_T_DELTA_LW]  = REGISTERS[FDBACK_T_MID_TH_LW] - REGISTERS[FDBACK_T_MIN];
  REGISTERS[FDBACK_T_DELTA_HI]  = REGISTERS[FDBACK_T_MAX] - REGISTERS[FDBACK_T_MID_TH_HI];

#if (EEPROM_DUMP == ON)
  Serial.println("########## EEPROM READ ##########");
  for (uint8_t index = 0 ; index < TOTAL_REGISTERS ; index += 1)
  {
    Serial.print("EEPROM [ "); Serial.print(index); Serial.print(" ] = "); Serial.println(REGISTERS[index]);
  }
#endif
}

void readSafetyPin()
{
  // Debounce Code
  if ((millis() - Safety_Reference) >= SAFETY_DEBOUNCE)
  {
    // Read safety pin
    Safety_State   = (SAFETY_INPUT & (1 << SAFETY_PIN)) ? (ON) : (OFF);

    if (((millis() - Safety_Reference) <= SAFETY_TIMEOUT) && (Safety_State == OFF))
    {
      Safety_State = ON;
    }
  }

  Safety_Reference = millis();
}

void readAuxPin()
{
  // Debounce Code
  if ((millis() - Aux_Reference) >= SAFETY_DEBOUNCE)
  {
    Aux_State = !Aux_State;

    if (Aux_State)
    {
        // Clear previus pin state to low
        AUX_OUTPORT &= ~(1 << AUX_OUTPIN);
    }
    else
    {
        // Set pin foward to high
        AUX_OUTPORT |=  (1 << AUX_OUTPIN); 
    } 
  }

  Aux_Reference = millis();
}

void readSerialPort(uint8_t actuator)
{
  double kp = 0;  
  double ki = 0; 
  double kd = 0;

  if (Serial.available() > 0) 
  {
    // Read the recieved serial string, ex. 10.52kd OR 0.1ki
    String command = Serial.readString();
    
    double myFloat = command.toFloat();

    // Check if the value is the Kp
    if(command.indexOf("kp") > -1)
    {
      kp = myFloat;
      
      Serial.print("KP = ");
    }    

    // Check if the value is the Ki
    if(command.indexOf("ki") > -1)
    { 
      ki = myFloat;

      Serial.print("KI = ");
    }    

    // Check if the value is the Kd
    if(command.indexOf("kd") > -1)
    {
      kd = myFloat;

      Serial.print("KD = ");
    }

    // Prints the received number
    Serial.println(myFloat);
     
    switch (actuator)
    {
      case LIFT:
        {
          // Check wich of the values as been changed
          LIFT_KP = (kp == 0) ? (LIFT_KP) : (kp);
          LIFT_KI = (ki == 0) ? (LIFT_KI) : (ki);
          LIFT_KD = (kd == 0) ? (LIFT_KD) : (kd);

          // Set the new values to the PID
          lift_PID.SetTunings(LIFT_KP, LIFT_KI, LIFT_KD);
        }
        break;
      case TILT:
        {
          // Check wich of the values as been changed
          TILT_KP = (kp == 0) ? (TILT_KP) : (kp);
          TILT_KI = (ki == 0) ? (TILT_KI) : (ki);
          TILT_KD = (kd == 0) ? (TILT_KD) : (kd);

          // Set the new values to the PID
          tilt_PID.SetTunings(TILT_KP, TILT_KI, TILT_KD);
        }
        break;
    }
  }
}