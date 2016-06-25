#ifndef _FUNCTIONS_H_
#define _FUNCTIONS_H_

#include "Average.h"
#include <EEPROM.h>

//##############################################################################
// PI CLASS
//##############################################################################
class PID {
  public:
    PID();
    float PI_Iterate_Voltage(float Setpoint, float Measured_Value); 
    float PI_Iterate_Current(float Setpoint, float Measured_Value);
    void PI_Clear();
    
  private:
    float ErrorSum_Voltage = 0;
    float ErrorSum_Current = 0;

    unsigned long delta_t_Voltage; 
    unsigned long Last_Time_Voltage; 

    unsigned long delta_t_Current; 
    unsigned long Last_Time_Current;    
};

PID::PID()
{
}

// -------------------------------------------------------------------------------
float PID::PI_Iterate_Voltage(float Setpoint, float Measured_Value)
{    
  float Error = Setpoint - Measured_Value;
  
  delta_t_Voltage = millis() - Last_Time_Voltage;
  Last_Time_Voltage = millis(); 

  ErrorSum_Voltage += (Error * KI_Voltage) * delta_t_Voltage;
  ErrorSum_Voltage = constrain(ErrorSum_Voltage, -1.0, 1.0);

  float Output = (Error * KP_Voltage) + ErrorSum_Voltage;
  
  return(Output);  
}

// -------------------------------------------------------------------------------
float PID::PI_Iterate_Current(float Setpoint, float Measured_Value)
{  
  float Error = Setpoint - Measured_Value;
        
  delta_t_Current = millis() - Last_Time_Current;
  Last_Time_Current = millis(); 

  ErrorSum_Current += (Error * KI_Current) * delta_t_Current;
  ErrorSum_Current = constrain(ErrorSum_Current, -75.0, 75.0);

  float Output = (Error * KP_Current) + ErrorSum_Current;
  
  return(Output);  
}

// -------------------------------------------------------------------------------
void PID::PI_Clear()
{
  ErrorSum_Voltage = 0;
  ErrorSum_Current = 0;  
}








































//##############################################################################
// PWM CONTROL CLASS
//##############################################################################
class Control {
  public:
    Control();      
    char * float2s(float f, unsigned int digits);  
    float Calculate_Average_Cell_Voltage(); 
    void Set_Duty(byte Channel, byte Duty);
    byte Get_Neutral_Duty(byte Channel);
    float Find_Range_of_Cell_Voltages(); 
    void Disable_Channel(byte Channel); 
    void Enable_Channel(byte Channel);           
    float Read_Voltage(byte Channel);    
    void Get_All_Channel_Voltages();
    void Get_All_Channel_Currents();
    void Calculate_Cell_Voltages();
    void Detect_Number_of_Cells();
    void Disable_PWM_Sleep();             
    void Enable_PWM_Sleep();
    void Order_Channels();
    void Init();
    
    float Channel_Current[8];
    float Channel_Voltage[8]; 
    float Ordered_Cell_Voltages[7];
    byte  Ordered_Channels[8];
    byte  Number_of_Cells;

    // Store for Voltage Sense Constants
    struct {      
      float Channel_Voltage_Sense_Constants[7];
      byte Exists_In_EEPROM;
    } CVSC;         

  private:
    char  Mode;
    Average Channel_Voltage_Avg[8];
    Average Channel_Current_Avg[8]; 
};

//--------------------------------------------------------------------------------
Control::Control()
{ 
}

//--------------------------------------------------------------------------------
void Control::Init()
{
  // Read Voltage_Sense_Constants from EEPROM. If no calibration data exists already, perform the calibration routine. For this, All channels must be supplied with 12.00volts
  EEPROM.get(0, CVSC);
  if(!CVSC.Exists_In_EEPROM) {
    Serial.println("Performing Calibration Routine...");
    for(byte i = 0; i <= 6; i++)  {
      CVSC.Channel_Voltage_Sense_Constants[i] = 1;                                // Set the CVSC to 1, a known value (and not zero!)
      CVSC.Channel_Voltage_Sense_Constants[i] = 11.33 / Read_Voltage(i+1);         // CVSC = 12.0 / ADC_Value(0-1023)
      Serial.print("Voltage Sense Constant for Channel ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(float2s(CVSC.Channel_Voltage_Sense_Constants[i], 5));
    }
    
    CVSC.Exists_In_EEPROM = 1;                                                    // Indicate that the constants have been saved for future restarts
    EEPROM.put(0, CVSC);    
  }
  
  // This ensures the rolling average filters are primed 
  for(byte i = 1; i <= 10; i++)  {                                                          
    Get_All_Channel_Voltages();
    Get_All_Channel_Currents();
  }  

  Detect_Number_of_Cells();
}

//--------------------------------------------------------------------------------
char * Control::float2s(float f, unsigned int digits)
{
  int index = 0;
  static char s[16];                                                              // buffer to build string representation
 
  if (f < 0.0) {                                                                  // handle sign
   s[index++] = '-';
   f = -f;
  } 
  
  if (isinf(f))  {                                                                // handle infinite values
   strcpy(&s[index], "INF");
   return s;
  }
  
  if (isnan(f))  {                                                                // handle Not a Number
   strcpy(&s[index], "NaN");
   return s;
  }
  
  if (digits > 6)                                                                 // max digits
    digits = 6;
  long multiplier = pow(10, digits);                                              // fix int => long
  
  int exponent = int(log10(f));
  float g = f / pow(10, exponent);
  if ((g < 1.0) && (g != 0.0))  {
   g *= 10;
   exponent--;
  }
  
  long whole = long(g);                                                           // single digit
  long part = long((g-whole)*multiplier);                                         // # digits
  char format[16];
  
  sprintf(format, "%%ld.%%0%dld E%%+d", digits);
  sprintf(&s[index], format, whole, part, exponent);
  
  return s;
}

// -------------------------------------------------------------------------------
float Control::Calculate_Average_Cell_Voltage()
{
// Can do this by just using VCC/Number_of_Cells or by adding up each cell voltage in turn. Should give the same result...
  
  return(Channel_Voltage[7] / Number_of_Cells);
}

// -------------------------------------------------------------------------------
float Control::Find_Range_of_Cell_Voltages()
{
  float Max_Cell_Voltage = Ordered_Cell_Voltages[0];
  float Min_Cell_Voltage = Ordered_Cell_Voltages[0];
  
  for (byte i = 1; i <= 6; i++)  
  {     
    if(Ordered_Cell_Voltages[i] > LOW_VALUE_THRESHOLD)
    {                                          
      Max_Cell_Voltage = max(Max_Cell_Voltage, Ordered_Cell_Voltages[i]);
      Min_Cell_Voltage = min(Min_Cell_Voltage, Ordered_Cell_Voltages[i]);     
    }                             
  }    

  return(Max_Cell_Voltage - Min_Cell_Voltage);  
}

// -------------------------------------------------------------------------------
void Control::Calculate_Cell_Voltages()
{  
  for (byte i = 0; i <= 5; i++)  {                                                 
    Ordered_Cell_Voltages[i] = max(0, Channel_Voltage[Ordered_Channels[i+1]] - Channel_Voltage[Ordered_Channels[i]]);  

    if(Ordered_Cell_Voltages[i] > 5.0)
    {
      Enable_PWM_Sleep();
      Serial.println("ERROR - NOT ALL BALANCE LEADS CONNECTED! SHUTTING DOWN...");
      //while(1)  {}
    }
  } 
}

// -------------------------------------------------------------------------------
void Control::Order_Channels()
// This routine writes the channel numbers of increasing voltage to the array. eg [0, 1, 2, 7, 0, 0, 0, 0] for channel voltages [0.00, 3.88, 7.40, 0.00, 0.00, 0.00, 0.00, 11.33]
{  
  // First copy the buffer into a tempory array
  float Temp_Channel_Voltage[8];
                                                                                                                
  for (byte i = 0; i <= 7; i++)  {                                                 
    Temp_Channel_Voltage[i] = Channel_Voltage[i];                                     
  }
  
  Ordered_Channels[0] = 0;                                                        // The circuit requires ground is connected to the -ve on the battery pack

  // Find index (ie. channel number) of next smallest non-zero value in the array Channel_Voltage[]:
  for(byte i = 1; i <= 6; i++)  
  {
    byte Index_of_lowest_voltage_channel;                                         // This is subsequently updated if it's not.
    float Lowest_Value_Found = NO_LOW_VALUE_FOUND;
    
    for(byte j = 1; j <= 7; j++)  
    {  
      if(Temp_Channel_Voltage[j] < Lowest_Value_Found && Temp_Channel_Voltage[j] > LOW_VALUE_THRESHOLD)
      {
        Index_of_lowest_voltage_channel = j;
        Lowest_Value_Found = Temp_Channel_Voltage[j];  
      }          
    }
    
    if(Lowest_Value_Found > LOW_VALUE_THRESHOLD && Lowest_Value_Found != NO_LOW_VALUE_FOUND)  
    {      
      Temp_Channel_Voltage[Index_of_lowest_voltage_channel] = 0;                  // Remove the lowest value.
      Ordered_Channels[i] = Index_of_lowest_voltage_channel;                      // Insert the index of the next lowest channel into the array.
    }
    else  {
      Ordered_Channels[i] = 0;
    }
  }
}

// -------------------------------------------------------------------------------
void Control::Get_All_Channel_Voltages()
{ 
  byte Mode_Store = Mode;
    
  Enable_PWM_Sleep();                                                             // Puts drivers to sleep. This stops all currents flowing  
  delay(2500);                                                                    // Allow time for voltages to equalise etc

  for (byte i = 0; i <= 7; i++)  
  {                                                                                         
    Channel_Voltage[i] = Channel_Voltage_Avg[i].Rolling_Average(Read_Voltage(i)); // Read the channel voltage
  }

  if(Mode_Store == ACTIVE_MODE) {
    Disable_PWM_Sleep();                                                          // Enable all outputs again only if we were in ACTIVE_MODE
  }
    
  delay(2500);
}

// -------------------------------------------------------------------------------
void Control::Get_All_Channel_Currents()
// Must have recently read all cell voltages first since the current is calculated from the change in voltage.
// It's very rough but works since the impedance of a LiPo cell is very low.
{   
  for(byte i = 1; i <= 6; i++)  
  {                                                     
    Channel_Current[i] = Channel_Current_Avg[i].Rolling_Average((Read_Voltage(i) - Channel_Voltage[i]) * CONDUCTANCE_OF_SENSE_RES);
  } 
}

// -------------------------------------------------------------------------------
void Control::Detect_Number_of_Cells()
{  
  Number_of_Cells = 0;
    
  for (byte i = 1; i <= 7; i++)  
  {                                                       
    if(Channel_Voltage[i] > 1.0)
      Number_of_Cells++;
  }    
}

// -------------------------------------------------------------------------------
byte Control::Get_Neutral_Duty(byte Channel)
{
  float Neutral_Duty = (Channel_Voltage[Channel] / Channel_Voltage[7]) * 255.0;
  Neutral_Duty = constrain(Neutral_Duty, 0.0, 255.0);
  return(round(Neutral_Duty));  
}

// -------------------------------------------------------------------------------
void Control::Enable_Channel(byte Channel)
{
  switch (Channel)  {
    case 1:
      digitalWrite(ENABLE_CH1, HIGH);
      return;
    case 2:
      digitalWrite(ENABLE_CH2, HIGH);
      return;
    case 3:
      digitalWrite(ENABLE_CH3, HIGH);
      return;
    case 4:
      digitalWrite(ENABLE_CH4, HIGH);
      return;
    case 5:
      digitalWrite(ENABLE_CH5, HIGH);
      return;
    case 6:
      digitalWrite(ENABLE_CH6, HIGH);
      return; 
  }  
}

// -------------------------------------------------------------------------------
void Control::Disable_Channel(byte Channel)
{
  switch (Channel)  {
    case 1:
      digitalWrite(ENABLE_CH1, LOW);
      return;
    case 2:
      digitalWrite(ENABLE_CH2, LOW);
      return;
    case 3:
      digitalWrite(ENABLE_CH3, LOW);
      return;
    case 4:
      digitalWrite(ENABLE_CH4, LOW);
      return;
    case 5:
      digitalWrite(ENABLE_CH5, LOW);
      return;
    case 6:
      digitalWrite(ENABLE_CH6, LOW);
      return; 
  }  
}

// -------------------------------------------------------------------------------
void Control::Set_Duty(byte Channel, byte Duty)
{
  switch (Channel)  {
    case 1:
      OCR0A = Duty;
      return;
    case 2:
      OCR0B = Duty;
      return;
    case 3:
      OCR1A = Duty;
      return;
    case 4:
      OCR1B = Duty;
      return;
    case 5:
      OCR2A = Duty;
      return;
    case 6:
      OCR2B = Duty;
      return; 
  }  
}

// -------------------------------------------------------------------------------
float Control::Read_Voltage(byte Channel)
{
  switch (Channel)  {
    case 0:
      return(0);
    case 1:
      return(analogRead(VOLTAGE_SENSE_CH1) * CVSC.Channel_Voltage_Sense_Constants[0]);
    case 2:
      return(analogRead(VOLTAGE_SENSE_CH2) * CVSC.Channel_Voltage_Sense_Constants[1]);
    case 3:
      return(analogRead(VOLTAGE_SENSE_CH3) * CVSC.Channel_Voltage_Sense_Constants[2]);
    case 4:
      return(analogRead(VOLTAGE_SENSE_CH4) * CVSC.Channel_Voltage_Sense_Constants[3]);
    case 5:
      return(analogRead(VOLTAGE_SENSE_CH5) * CVSC.Channel_Voltage_Sense_Constants[4]);
    case 6:
      return(analogRead(VOLTAGE_SENSE_CH6) * CVSC.Channel_Voltage_Sense_Constants[5]);
    default:
      return(analogRead(VOLTAGE_SENSE_VCC) * CVSC.Channel_Voltage_Sense_Constants[6]);  
  }  
}

// -------------------------------------------------------------------------------
void Control::Enable_PWM_Sleep()
{
  digitalWrite(SLEEP, LOW);
  Mode = SLEEP_MODE;
}

// -------------------------------------------------------------------------------
void Control::Disable_PWM_Sleep()
{
  digitalWrite(SLEEP, HIGH);
  Mode = ACTIVE_MODE;
}


#endif
