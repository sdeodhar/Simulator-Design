
# Simulator-Design

Design of C - DOS BOX based simulator for testing BIOS for a product

Virtual Prototyping (Behaviour and skin simulation) Using C
_______________________________________________________________________________________________

This is a simulation of a Peak Temperature Latching Indicator (PTL) 

This is an instrument that measures the peak temperature of molten metal in a ferrous foundry.
Displays the latched temperature on a four digit LCD display. 
Logs the readings with time Stamps in EEPROM , communicates over a wireless interface. 

Different components that have been simulated here are as follows:

I2C BUS
ADC
Thermocouple Sensor states
RTC
Input mechanism
Temperature latching
Seven Segment Display
EEPROM


I am still working on adding detailed functionalities like caliberation.
The Executible provided can be run in DOS-BOX or any IDE supporting BORLAND C

The machine works with following states:

Temperature sensing states for Thermocouple Sensor:

TCBREAK		Thermocouple Break
TCSHORT		Thermocouple Short
TCDIPPED	Thermocouple Dipped in molten metal
CYCLECOMPLETE	Measurement Cycle Complete
LATCH		Peak Temperature Latched
NOLATCH		Desired Temperatuer Peak not detected yet for sufficient amount of time
TCNOTDETECTED	Thermocouple Not Detected
TCDETECTED	Thermocouple Detected

States of machine Operation:

OPN 	Open
SHORT 	Short
MSR 	Measure
SPN     Span

