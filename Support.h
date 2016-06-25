#ifndef _SUPPORT_H_
#define _SUPPORT_H_

//--- Defines: -------------------------------------------------------------------
#define         VOLTAGE_SENSE_CONST         2.710388E-2
#define         CONDUCTANCE_OF_SENSE_RES    1.2 
#define         FILTER_SIZE                 5

#define         KP_Voltage                  1.0
#define         KI_Voltage                  6.4e-6
#define         KP_Current                  5.0 
#define         KI_Current                  2.0e-4

#define         SLEEP_MODE                  0
#define         ACTIVE_MODE                 1
#define         NO_LOW_VALUE_FOUND          99.9
#define         LOW_VALUE_THRESHOLD         2.0

//--- Pin definitions: -----------------------------------------------------------
#define         SLEEP                       8

#define         ENABLE_CH1                  13
#define         ENABLE_CH2                  4
#define         ENABLE_CH3                  7
#define         ENABLE_CH4                  12 
#define         ENABLE_CH5                  A5
#define         ENABLE_CH6                  2

#define         OUT_CH1                     6
#define         OUT_CH2                     5
#define         OUT_CH3                     9
#define         OUT_CH4                     10
#define         OUT_CH5                     11
#define         OUT_CH6                     3

#define         VOLTAGE_SENSE_CH1           A1
#define         VOLTAGE_SENSE_CH2           A7
#define         VOLTAGE_SENSE_CH3           A2
#define         VOLTAGE_SENSE_CH4           A6        
#define         VOLTAGE_SENSE_CH5           A3
#define         VOLTAGE_SENSE_CH6           A4
#define         VOLTAGE_SENSE_VCC           A0



//--- Routines -------------------------------------------------------------------
void Init_Timers_Pins (void);




#endif
