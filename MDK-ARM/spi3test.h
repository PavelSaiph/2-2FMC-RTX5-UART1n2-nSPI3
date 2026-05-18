#ifndef	__SPI3TEST_H__
#define __SPI3TEST_H__

void SPI3_Receive(void);
void Parse_Modbus_Frame(uint8_t rx_len);

#define Unit_Type 0x0000
#define Unit_Number 0x000A
#define FPGA1_Version 0x000F
#define FPGA2_Version 0x0014
#define Arm1_Version 0x0019
#define Arm2_Version 0x001E
#define Unit_Power_Limit 0x0023
#define Unit_Voltage_Limit 0x0025
#define Unit_Current_Limit 0x0026

#define Output_Switch 0x0100
#define Regulation_Mode 0x0101
#define Setpoint 0x0102
#define Control_Mode 0x0104

#define Set_Comm_Watchdog_Timer 0x0200

#define User_Power_Limit 0x0300
#define User_Voltage_Limit 0x0302
#define User_Current_Limit 0x0303
#define Ignition_Switch 0x0304
#define Ignition_SetPoint 0x0305
#define Ripple_Level 0x0306

#define Enable_Pulsing 0x0400
#define Set_DC_Polarity 0x0401
#define Set_Boost_Voltage_Setpoint 0x0402
#define Set_Output_Deadtime1 0x0404
#define Set_Output_Deadtime2 0x0406
#define Set_Output_Frequency 0x0408
#define Set_Pulse_Duty_Cycle 0x040A

#define Enable_Power_Pulsing 0x0500
#define Set_Power_Pulsing_On_Time 0x0501
#define Set_Power_Pulsing_Off_Time 0x0502

#define Enable_Ramp 0x0600
#define Set_Step 0x0601
#define Ramping_Time 0x0602

#define Unit_Type_L 10
#define Unit_Number_L 5
#define FPGA1_Version_L 5
#define FPGA2_Version_L 5
#define Arm1_Version_L 5
#define Arm2_Version_L 5
#define Unit_Power_Limit_L 2
#define Unit_Voltage_Limit_L 1
#define Unit_Current_Limit_L 1

#define Output_Switch_L 1
#define Regulation_Mode_L 1
#define Setpoint_L 2
#define Control_Mode_L 1

#define Set_Comm_Watchdog_Timer_L 1

#define User_Power_Limit_L 2
#define User_Voltage_Limit_L 1
#define User_Current_Limit_L 1
#define Ignition_Switch_L 1
#define Ignition_SetPoint_L 1
#define Ripple_Level_L 1

#define Enable_Pulsing_L 1
#define Set_DC_Polarity_L 1
#define Set_Boost_Voltage_Setpoint_L 2
#define Set_Output_Deadtime1_L 2
#define Set_Output_Deadtime2_L 2
#define Set_Output_Frequency_L 2
#define Set_Pulse_Duty_Cycle_L 2

#define Enable_Power_Pulsing_L 1
#define Set_Power_Pulsing_On_Time_L 1
#define Set_Power_Pulsing_Off_Time_L 1

#define Enable_Ramp_L 1
#define Set_Step_L 1
#define Ramping_Time_L 1
#endif
