/*
 * This is the code for the active balancer circuit. There is still some work left to do but it's nearly done
 * 
 * 
 * We require to have the battery full voltage on the VCC/Cell7 connection.
 * We define current flowing into a channel as +ve. 
 * 
 *
 * Currents can be measured by finding the voltage drops across the 0.5ohm shunt resistors. Voltages of the cells must be measured
 * with and without current flowing into/out of them and the difference taken. Hopefully 0.5ohms is enough to dominate all other
 * resistances but I'm worried the connectors will have a significant resistance... 
 * 
 * To Do:
 *  Better Error detection and Catching (for example: if VCC is approximately equal to the highest cell voltage (within a diode drop or so) don't enable H-Bridges)
 *  Improve shutdown procedure and better channel idling -> Voltage difference small and channel correction current small.
 *  Push for a better balance. The 10bit resolution is limiting things but we could still go a bit tighter
 * 
 */
#include "Support.h"
#include "Functions.h"

// defines:
#define _DEBUG_MODE_                                                                        // Commenting out this will prevent all Serial port communication

// Function defines:
void Debug_Display();
unsigned long SysTick_Millis();
void SysTick_Delay(unsigned long Millis_to_Delay);

// Glabal Variables
unsigned long SysTick = 0;
unsigned long Time_Last_Outside_Deadband[6];
unsigned long Time_Last_Inside_Deadband[6];
unsigned long Time_When_Cells_Last_Unacceptably_Balanced;
byte Channel_Active[6];
Control PWM_Control;
PID PID_Controller[7];

//##############################################################################
// SETUP
//##############################################################################
void setup()
{
  // The library clock ticks 62.5 times faster than in the ordinary delay(), millis(), micros() functions  
  Serial.begin(115200);  
  Init_Timers_Pins();                                                                       // Initialises timers and configures pins as inputs/outputs etc.

  PWM_Control.Init();
  if(PWM_Control.CVSC.Exists_In_EEPROM) {
    Serial.println("EEPROM Contains Voltage Sense constants.");
  }
}

//##############################################################################
// LOOP  
//##############################################################################
void loop() 
{  
  SysTick_Delay(500);
  
  // A) Get all voltages and currents:  
  PWM_Control.Get_All_Channel_Voltages();                                                   // Reads all the channel voltages and stores them in array Channel_Voltage[8]
  PWM_Control.Get_All_Channel_Currents();                                                   // Reads all the channel currents and stores them in array Channel_Current[8]
  PWM_Control.Order_Channels();                                                             // This works out the ascending order of the channel voltages
  PWM_Control.Calculate_Cell_Voltages();                                                    // This works out the voltages of each cell - and error is detected if not all balance leads are connected


  // B) If the cell voltage range is >= 0.03 and has been for >30s then balance the cells. Otherwise enter a low power sleep mode.
  if(PWM_Control.Find_Range_of_Cell_Voltages() >= 0.035)  
  {
    PWM_Control.Disable_PWM_Sleep();
    Time_When_Cells_Last_Unacceptably_Balanced = SysTick_Millis();
  }
  else if(PWM_Control.Find_Range_of_Cell_Voltages() < 0.025)  
  {
    if(SysTick_Millis() - Time_When_Cells_Last_Unacceptably_Balanced >= 30000)  {           // If the range <= 0.025 and has been for >30 seconds
      PWM_Control.Enable_PWM_Sleep();
      Serial.println("The cells are balanced. Sleeping for 10 Seconds...");
      SysTick_Delay(10000);
    }
  }  


  // C) Loop through each connected channel and control the Duty cycle to it to balance the cells:
  for(byte i = 1; i < PWM_Control.Number_of_Cells; i++)
  {    
    // -------------------- For each channel in turn we cascade 2 PI control loops: --------------------
    //  - The 1st PI controller attempts to minimise the Voltage_Error by controlling the current into the corresponding Channel
    //    -> Voltage_Error = (VCC/Number_of_Cells * Channel) - Measured_Voltage
    //  - The 2nd PI controller attempts to minimise the Current_Error by controlling the PWM duty cycle offset from the neutral value.
    //  
    //  ----------- Essentially, we're trying to bring all cells to the average cell voltage -----------    

    // 1st: For ease of reading the code, copy class variables to better named local versions:
    byte Channel = PWM_Control.Ordered_Channels[i];
    float Channel_Voltage_Setpoint = PWM_Control.Calculate_Average_Cell_Voltage() * i;
    float Channel_Voltage_Measured = PWM_Control.Channel_Voltage[Channel];
    float Channel_Current_Measured = PWM_Control.Channel_Current[Channel];
    #ifdef _DEBUG_MODE_
      Serial.print("i: ");
      Serial.println(i);
      Serial.print("Channel: ");
      Serial.println(Channel);
      Serial.print("Channel_Voltage_Setpoint: ");
      Serial.println(Channel_Voltage_Setpoint);
      Serial.print("Channel_Voltage_Measured: ");
      Serial.println(Channel_Voltage_Measured);    
    #endif


    // 2nd: Perform the Voltage_Error -> Current_Demand PI control: 
    float Channel_Current_Setpoint = PID_Controller[i].PI_Iterate_Voltage(Channel_Voltage_Setpoint, Channel_Voltage_Measured);    
    Channel_Current_Setpoint = constrain(Channel_Current_Setpoint, -1.0, 1.0);
    #ifdef _DEBUG_MODE_
      Serial.print("Channel_Current_Setpoint: ");
      Serial.println(Channel_Current_Setpoint);
      Serial.print("Channel_Current_Measured: ");
      Serial.println(Channel_Current_Measured);
    #endif


    // 3rd: Perform the Current_Demand -> Duty_Cycle_Offset PI control: 
    float PWM_Duty_Offset = PID_Controller[i].PI_Iterate_Current(Channel_Current_Setpoint, Channel_Current_Measured);
    PWM_Duty_Offset = constrain(PWM_Duty_Offset, -75.0, 75.0);                              // This constraint prevents dangerous duty cycles from occuring
        
    int Duty = round(PWM_Duty_Offset) + PWM_Control.Get_Neutral_Duty(Channel);
    Duty = constrain(Duty, 0, 255);

    
    // 4th Coordinate the enabling and disabling of PWM on the channels depending on the voltage error, time the errors been occuring and the correction current being applied: 
    float Channel_Voltage_Error = abs(Channel_Voltage_Setpoint - Channel_Voltage_Measured);
    
    if(Channel_Voltage_Error > 0.020)                           
    {   
      Time_Last_Outside_Deadband[i] = SysTick_Millis();
      
      if(SysTick_Millis() - Time_Last_Inside_Deadband[i] >= 15000)                          // 1st Scenario: If voltage error > 0.020 for >15s -> Enable Channel 
      {   
        Channel_Active[i] = ACTIVE_MODE;       
      }
    }
    else if(Channel_Voltage_Error <= 0.020 && Channel_Voltage_Error > 0.010)                // 2nd Scenario: Voltage error in hysterisis band -> Keep doing whatever we were doing
    {
      Time_Last_Inside_Deadband[i] = SysTick_Millis();
      Time_Last_Outside_Deadband[i] = SysTick_Millis();
    }
    else if(Channel_Voltage_Error <= 0.010)                                                 // 3rd Scenario: If voltage error < 0.015 and |current| < 0.1 for >15seconds -> Disable channel
    {
      Time_Last_Inside_Deadband[i] = SysTick_Millis();  

      if(abs(Channel_Current_Setpoint) < 0.1)
      {
        if(SysTick_Millis() - Time_Last_Outside_Deadband[i] >= 15000)                          
        {   
          Channel_Active[i] = SLEEP_MODE;       
        }  
      }    
    }

    // 5th Enable and set the duty cycles of channels that are requested to be active 
    if(Channel_Active[i]) 
    {
      #ifdef _DEBUG_MODE_
        Serial.println("CHANNEL ACTIVE!");
        Serial.print("PWM_Duty_Offset: ");
        Serial.println(PWM_Duty_Offset);
        Serial.print("Duty: ");
        Serial.println(Duty);
      #endif 

      PWM_Control.Set_Duty(Channel, Duty);
      PWM_Control.Enable_Channel(Channel);
    }
    else  
    {
      #ifdef _DEBUG_MODE_
        Serial.println("CHANNEL INACTIVE: "); 
      #endif 
      
      PID_Controller[i].PI_Clear();
      PWM_Control.Disable_Channel(Channel);        
    }             
  }
  
  #ifdef _DEBUG_MODE_
    Debug_Display(); 
  #endif 
}

// -------------------------------------------------------------------------------
void Debug_Display()
{
  Serial.println("------------");
  Serial.print("Ch Voltages:  ");  
  for(int i = 0; i <= 7; i++) {    
    Serial.print(PWM_Control.Channel_Voltage[i]);
    Serial.print(", ");    
  }  
  Serial.println();
  
  Serial.print("Ch Currents:  ");  
  for(int i = 0; i <= 7; i++) {    
    Serial.print(PWM_Control.Channel_Current[i]);
    Serial.print(", ");    
  } 
  Serial.println();

  Serial.print("Ordered Ch:  ");  
  for(int i = 0; i <= 7; i++) {    
    Serial.print(PWM_Control.Ordered_Channels[i]);
    Serial.print(", ");    
  } 
  Serial.println();

  Serial.print("Ordered Cell Voltages:  ");  
  for(int i = 0; i <= 6; i++) {    
    Serial.print(PWM_Control.Ordered_Cell_Voltages[i]);
    Serial.print(", ");    
  } 
  Serial.println();

  Serial.print("Run time:  ");
  Serial.println(SysTick_Millis()/1000); 

  Serial.print("# Cells detected:  ");
  Serial.println(PWM_Control.Number_of_Cells);
  
  Serial.print("Average Cell Voltage:  ");
  Serial.println(PWM_Control.Calculate_Average_Cell_Voltage());

  Serial.print("Cell Voltage Range:  ");
  Serial.println(PWM_Control.Find_Range_of_Cell_Voltages());     
}

// -------------------------------------------------------------------------------
unsigned long SysTick_Millis()
{
  float MilliSeconds_Since_Start = (float)SysTick / 62.5;
  
  return(round(MilliSeconds_Since_Start));
}

// -------------------------------------------------------------------------------
void SysTick_Delay(unsigned long Millis_to_Delay)
{
  unsigned long Time = SysTick_Millis() + Millis_to_Delay;

  while(SysTick_Millis() < Time)
  {
    delay(1);  
  }  
}

//##############################################################################
// SETUP
//##############################################################################
ISR (TIMER2_OVF_vect)                                                         
{  
  SysTick++;   
} 








