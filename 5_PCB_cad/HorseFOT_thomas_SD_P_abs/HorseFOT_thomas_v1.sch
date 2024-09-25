EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Regulator_Linear:LM7805_TO220 U2
U 1 1 619E8C6C
P 9650 1350
F 0 "U2" H 9650 1592 50  0000 C CNN
F 1 "LM7805_TO220" H 9650 1501 50  0000 C CNN
F 2 "SamacSys_Parts:SOT230P700X180-4N" H 9650 1575 50  0001 C CIN
F 3 "https://www.onsemi.cn/PowerSolutions/document/MC7800-D.PDF" H 9650 1300 50  0001 C CNN
	1    9650 1350
	1    0    0    -1  
$EndComp
$Comp
L Analog_ADC:MCP3201 U3
U 1 1 619EB980
P 4850 4350
F 0 "U3" H 4850 3961 50  0000 C CNN
F 1 "MCP3201" H 4850 3870 50  0000 C CNN
F 2 "SamacSys_Parts:SOIC127P600X175-8N" H 5600 4000 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21290D.pdf" H 5650 4250 50  0001 C CNN
	1    4850 4350
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MCP602 U5
U 1 1 619EBFEB
P 3300 4250
F 0 "U5" H 3300 4617 50  0000 C CNN
F 1 "MCP602" H 3300 4526 50  0000 C CNN
F 2 "SamacSys_Parts:SOP65P490X110-8N" H 3300 4250 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 3300 4250 50  0001 C CNN
	1    3300 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 619F4264
P 3750 3700
F 0 "C14" H 3865 3746 50  0000 L CNN
F 1 "100nF" H 3865 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 3788 3550 50  0001 C CNN
F 3 "~" H 3750 3700 50  0001 C CNN
	1    3750 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 619F4EDA
P 4200 3700
F 0 "C15" H 4315 3746 50  0000 L CNN
F 1 "1nF" H 4315 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 4238 3550 50  0001 C CNN
F 3 "~" H 4200 3700 50  0001 C CNN
	1    4200 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 619F5768
P 3950 5250
F 0 "C19" H 4065 5296 50  0000 L CNN
F 1 "100nF" H 4065 5205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 3988 5100 50  0001 C CNN
F 3 "~" H 3950 5250 50  0001 C CNN
	1    3950 5250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 619F7298
P 2500 5650
F 0 "C13" H 2615 5696 50  0000 L CNN
F 1 "220nF" H 2615 5605 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 2538 5500 50  0001 C CNN
F 3 "~" H 2500 5650 50  0001 C CNN
	1    2500 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 619F7EAE
P 2250 4150
F 0 "R6" V 2043 4150 50  0000 C CNN
F 1 "7.68K" V 2134 4150 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2180 4150 50  0001 C CNN
F 3 "~" H 2250 4150 50  0001 C CNN
	1    2250 4150
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 619F8D99
P 2350 5500
F 0 "R7" V 2143 5500 50  0000 C CNN
F 1 "7.68k" V 2234 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 2280 5500 50  0001 C CNN
F 3 "~" H 2350 5500 50  0001 C CNN
	1    2350 5500
	0    1    1    0   
$EndComp
Wire Wire Line
	1450 4200 2100 4200
Wire Wire Line
	2400 4150 3000 4150
Wire Wire Line
	3000 4350 3000 4650
Wire Wire Line
	3000 4650 3700 4650
Wire Wire Line
	4200 3550 4200 3350
Wire Wire Line
	4200 3550 4750 3550
Wire Wire Line
	4750 3550 4750 4050
Connection ~ 4200 3550
Wire Wire Line
	4350 4450 4050 4450
Wire Wire Line
	4050 4450 4050 3850
Connection ~ 4050 3850
Wire Wire Line
	4050 3850 4200 3850
Wire Wire Line
	4050 4450 4050 4650
Wire Wire Line
	4050 4650 4750 4650
Connection ~ 4050 4450
Wire Wire Line
	2500 5500 3200 5500
Connection ~ 2500 5500
Wire Wire Line
	3950 5100 4100 5100
Wire Wire Line
	2200 5500 1500 5500
Wire Wire Line
	3200 5700 3200 5950
Wire Wire Line
	3200 5950 3800 5950
Wire Wire Line
	3800 5950 3800 5600
$Comp
L Analog_ADC:MCP3201 U4
U 1 1 619EC55F
P 4900 5700
F 0 "U4" H 4900 5311 50  0000 C CNN
F 1 "MCP3201" H 4900 5220 50  0000 C CNN
F 2 "SamacSys_Parts:SOIC127P600X175-8N" H 5650 5350 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21290D.pdf" H 5700 5600 50  0001 C CNN
	1    4900 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 5600 4400 5600
Wire Wire Line
	4400 5800 4150 5800
Wire Wire Line
	4100 5100 4100 4900
Wire Wire Line
	4800 5100 4800 5400
Wire Wire Line
	4800 6000 4150 6000
Wire Wire Line
	4150 6000 4150 5800
Connection ~ 4100 5100
$Comp
L Device:C C20
U 1 1 619F616E
P 4400 5250
F 0 "C20" H 4515 5296 50  0000 L CNN
F 1 "1nF" H 4515 5205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 4438 5100 50  0001 C CNN
F 3 "~" H 4400 5250 50  0001 C CNN
	1    4400 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 5400 4150 5400
Wire Wire Line
	4150 5400 4150 5800
Connection ~ 4150 5400
Connection ~ 4150 5800
Wire Wire Line
	3600 4250 3700 4250
Wire Wire Line
	3700 4650 3700 4250
Connection ~ 3700 4250
Wire Wire Line
	3700 4250 4350 4250
Text GLabel 5550 4250 2    50   Input ~ 0
SPI_SENS_CLK
Text GLabel 5600 4450 2    50   Input ~ 0
SPI_SENS_CS
Text GLabel 5600 4350 2    50   Output ~ 0
SPI_SENS_D2
Wire Wire Line
	5550 4250 5350 4250
Wire Wire Line
	5600 4350 5350 4350
Wire Wire Line
	5600 4450 5350 4450
Text GLabel 5600 5600 2    50   Input ~ 0
SPI_SENS_CLK
Text GLabel 5650 5800 2    50   Input ~ 0
SPI_SENS_CS
Text GLabel 5650 5700 2    50   Output ~ 0
SPI_SENS_D1
Wire Wire Line
	5600 5600 5400 5600
Wire Wire Line
	5650 5700 5400 5700
Wire Wire Line
	5650 5800 5400 5800
Text GLabel 1450 3900 2    50   Input ~ 0
5V
Text GLabel 1450 4050 2    50   Input ~ 0
GND
Connection ~ 2400 4150
Text GLabel 2400 4450 3    50   Input ~ 0
GND
$Comp
L Device:C C12
U 1 1 619F69A0
P 2400 4300
F 0 "C12" H 2515 4346 50  0000 L CNN
F 1 "220nF" H 2515 4255 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 2438 4150 50  0001 C CNN
F 3 "~" H 2400 4300 50  0001 C CNN
	1    2400 4300
	1    0    0    -1  
$EndComp
Text GLabel 2500 5800 3    50   Input ~ 0
GND
Text GLabel 4100 6000 0    50   Input ~ 0
GND
Text GLabel 4000 4650 0    50   Input ~ 0
GND
Text GLabel 5800 3900 3    50   Input ~ 0
GND
Wire Wire Line
	4050 4650 4000 4650
Connection ~ 4050 4650
Text GLabel 4200 3350 0    50   Input ~ 0
5V
Text GLabel 4100 4900 0    50   Input ~ 0
5V
Wire Wire Line
	4100 6000 4150 6000
Connection ~ 4150 6000
$Comp
L Connector:Screw_Terminal_01x02 J5
U 1 1 61A4BEDE
P 7050 1450
F 0 "J5" H 7130 1442 50  0000 L TNN
F 1 "Screw_Terminal_01x02" V 7130 1351 50  0000 C TNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 7050 1450 50  0001 C CNN
F 3 "~" H 7050 1450 50  0001 C CNN
	1    7050 1450
	-1   0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 61A4CD90
P 8100 2450
F 0 "R3" H 8170 2496 50  0000 L CNN
F 1 "470K" H 8170 2405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8030 2450 50  0001 C CNN
F 3 "~" H 8100 2450 50  0001 C CNN
	1    8100 2450
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 61A4D7D1
P 8100 2050
F 0 "R2" V 7893 2050 50  0000 C CNN
F 1 "470K" V 7984 2050 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8030 2050 50  0001 C CNN
F 3 "~" H 8100 2050 50  0001 C CNN
	1    8100 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 61A4DD02
P 8100 1650
F 0 "R1" V 7893 1650 50  0000 C CNN
F 1 "470K" V 7984 1650 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 8030 1650 50  0001 C CNN
F 3 "~" H 8100 1650 50  0001 C CNN
	1    8100 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 61A53479
P 9050 1500
F 0 "C6" H 9165 1546 50  0000 L CNN
F 1 "220nF" H 9165 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 9088 1350 50  0001 C CNN
F 3 "~" H 9050 1500 50  0001 C CNN
	1    9050 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 61A54C9A
P 10050 1500
F 0 "C7" H 10165 1546 50  0000 L CNN
F 1 "100nF" H 10165 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 10088 1350 50  0001 C CNN
F 3 "~" H 10050 1500 50  0001 C CNN
	1    10050 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C8
U 1 1 61A57278
P 10500 1500
F 0 "C8" H 10618 1546 50  0000 L CNN
F 1 "100uF" H 10618 1455 50  0000 L CNN
F 2 "SamacSys_Parts:EEE1AA221P" H 10538 1350 50  0001 C CNN
F 3 "~" H 10500 1500 50  0001 C CNN
	1    10500 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C5
U 1 1 61A57D59
P 8600 1500
F 0 "C5" H 8718 1546 50  0000 L CNN
F 1 "220uF" H 8718 1455 50  0000 L CNN
F 2 "SamacSys_Parts:EEE1AA221P" H 8638 1350 50  0001 C CNN
F 3 "~" H 8600 1500 50  0001 C CNN
	1    8600 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 1350 10050 1350
Wire Wire Line
	9650 1650 10050 1650
Connection ~ 9650 1650
Wire Wire Line
	8100 1350 8100 1500
Wire Wire Line
	8100 1350 8600 1350
Wire Wire Line
	8100 1800 8100 1900
Wire Wire Line
	8100 2300 8100 2250
Wire Wire Line
	8100 2600 7650 2600
Wire Wire Line
	7650 2600 7650 1450
Wire Wire Line
	8500 1650 8600 1650
Wire Wire Line
	8500 2600 8100 2600
Connection ~ 8100 2600
Wire Wire Line
	10500 1350 10800 1350
Connection ~ 10500 1350
Wire Wire Line
	10500 1650 10800 1650
Connection ~ 10500 1650
Text GLabel 10800 1350 2    50   Output ~ 0
5V
Text GLabel 10800 1650 2    50   Output ~ 0
GND
Text GLabel 7550 2250 0    50   Output ~ 0
BATTERY_LVL
Wire Wire Line
	8100 2250 7550 2250
Connection ~ 8100 2250
Wire Wire Line
	8100 2250 8100 2200
Text GLabel 3400 1150 2    50   Input ~ 0
3.3V
Text GLabel 1250 1800 0    50   BiDi ~ 0
I2C_SENS_D
Text GLabel 1250 1700 0    50   BiDi ~ 0
I2C_SENS_CLK
Text GLabel 2100 2350 3    50   Input ~ 0
GND
Wire Wire Line
	3400 1500 3300 1500
Wire Wire Line
	3400 1150 3400 1500
$Comp
L Device:R R5
U 1 1 61A22836
P 1750 1450
F 0 "R5" H 1820 1496 50  0000 L CNN
F 1 "22K" H 1820 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1680 1450 50  0001 C CNN
F 3 "~" H 1750 1450 50  0001 C CNN
	1    1750 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1600 1500 1700
Wire Wire Line
	1750 1600 1750 1800
Wire Wire Line
	1750 1300 1750 1150
Connection ~ 1750 1150
Wire Wire Line
	1750 1150 1500 1150
Wire Wire Line
	1500 1300 1500 1150
Connection ~ 1500 1150
Wire Wire Line
	1500 1150 1400 1150
$Comp
L Device:R R4
U 1 1 61A21B89
P 1500 1450
F 0 "R4" H 1570 1496 50  0000 L CNN
F 1 "22K" H 1570 1405 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 1430 1450 50  0001 C CNN
F 3 "~" H 1500 1450 50  0001 C CNN
	1    1500 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 61A2C29D
P 1750 2350
F 0 "C10" V 1498 2350 50  0000 C CNN
F 1 "0.22uF" V 1589 2350 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 1788 2200 50  0001 C CNN
F 3 "~" H 1750 2350 50  0001 C CNN
	1    1750 2350
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1250 1700 1500 1700
Wire Wire Line
	1250 1800 1750 1800
Wire Wire Line
	1400 1150 1400 2350
$Comp
L Device:C C11
U 1 1 61A98176
P 1950 2200
F 0 "C11" V 1698 2200 50  0000 C CNN
F 1 "0.22uF" V 1789 2200 50  0000 C CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 1988 2050 50  0001 C CNN
F 3 "~" H 1950 2200 50  0001 C CNN
	1    1950 2200
	-1   0    0    1   
$EndComp
Connection ~ 1750 1800
Wire Wire Line
	1750 1800 2100 1800
Connection ~ 1500 1700
Wire Wire Line
	1500 1700 2100 1700
Wire Wire Line
	1950 1500 1950 2050
Wire Wire Line
	1950 1500 2100 1500
Wire Wire Line
	1900 2350 1950 2350
Connection ~ 1950 2350
Wire Wire Line
	1400 2350 1600 2350
Wire Wire Line
	1750 1150 3400 1150
$Comp
L Device:C C18
U 1 1 61B2C293
P 6000 3700
F 0 "C18" H 6115 3746 50  0000 L CNN
F 1 "100nF" H 6115 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 6038 3550 50  0001 C CNN
F 3 "~" H 6000 3700 50  0001 C CNN
	1    6000 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3550 6000 3350
Text GLabel 6000 3350 2    50   Input ~ 0
5V
$Comp
L Device:C C17
U 1 1 61B2C28D
P 5650 3700
F 0 "C17" H 5765 3746 50  0000 L CNN
F 1 "1nF" H 5765 3655 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 5688 3550 50  0001 C CNN
F 3 "~" H 5650 3700 50  0001 C CNN
	1    5650 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3850 5800 3850
Wire Wire Line
	4950 3550 4950 4050
Wire Wire Line
	4950 3550 5650 3550
Connection ~ 5650 3550
Wire Wire Line
	5650 3550 6000 3550
Connection ~ 6000 3550
Wire Wire Line
	5800 3900 5800 3850
Connection ~ 5800 3850
Wire Wire Line
	5800 3850 6000 3850
Text GLabel 6100 5400 2    50   Input ~ 0
GND
$Comp
L Device:C C22
U 1 1 61B6CE05
P 6050 5250
F 0 "C22" H 6165 5296 50  0000 L CNN
F 1 "1nF" H 6165 5205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 6088 5100 50  0001 C CNN
F 3 "~" H 6050 5250 50  0001 C CNN
	1    6050 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 5100 6050 4900
Text GLabel 6050 4900 2    50   Input ~ 0
5V
$Comp
L Device:C C21
U 1 1 61B6CE0D
P 5600 5250
F 0 "C21" H 5715 5296 50  0000 L CNN
F 1 "100nF" H 5715 5205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 5638 5100 50  0001 C CNN
F 3 "~" H 5600 5250 50  0001 C CNN
	1    5600 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 5100 5600 5100
Connection ~ 6050 5100
Wire Wire Line
	6100 5400 6050 5400
Connection ~ 6050 5400
Wire Wire Line
	5000 5400 5000 5100
Text GLabel 5050 850  0    50   Input ~ 0
PWM_MOTOR_OUT
Text GLabel 5050 950  0    50   Input ~ 0
GND
Connection ~ 8600 1350
Connection ~ 8600 1650
Wire Wire Line
	8600 1650 9050 1650
Wire Wire Line
	8600 1350 9050 1350
Connection ~ 9050 1350
Connection ~ 9050 1650
Wire Wire Line
	9050 1650 9650 1650
Wire Wire Line
	9050 1350 9350 1350
Connection ~ 10050 1350
Connection ~ 10050 1650
Wire Wire Line
	10050 1350 10500 1350
Wire Wire Line
	10050 1650 10500 1650
Wire Wire Line
	3750 3550 4200 3550
Wire Wire Line
	3750 3850 4050 3850
Connection ~ 4400 5100
Wire Wire Line
	4400 5100 4800 5100
Wire Wire Line
	4100 5100 4400 5100
Connection ~ 5600 5100
Wire Wire Line
	5600 5100 6050 5100
Wire Wire Line
	5600 5400 6050 5400
Wire Wire Line
	4150 5400 4400 5400
Wire Notes Line
	3850 2700 650  2700
Wire Notes Line
	650  600  3850 600 
Wire Notes Line
	650  3000 6550 3000
Wire Notes Line
	6550 6300 650  6300
Wire Notes Line
	6700 600  6700 2700
NoConn ~ 3300 1700
NoConn ~ 3300 1800
$Comp
L HorseFOT_thomas_v1-rescue:HIH6131-021-001-SamacSys_Parts IC1
U 1 1 619E7545
P 2100 1500
F 0 "IC1" H 2700 1765 50  0000 C CNN
F 1 "HIH6131-021-001" H 2700 1674 50  0000 C CNN
F 2 "SamacSys_Parts:HIH6131021001" H 3150 1600 50  0001 L CNN
F 3 "https://sensing.honeywell.com/honeywell-sensing-humidicon-hih6100-series-product-sheet-009059-6-en.pdf" H 3150 1500 50  0001 L CNN
F 4 "Board Mount Humidity Sensors I2C,5 %RH,SOIC-8 SMD HYDROPHOBIC FILTER" H 3150 1400 50  0001 L CNN "Description"
F 5 "2.055" H 3150 1300 50  0001 L CNN "Height"
F 6 "Honeywell Sensing and Productivity Solutions" H 3150 1200 50  0001 L CNN "Manufacturer_Name"
F 7 "HIH6131-021-001" H 3150 1100 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "" H 3150 1000 50  0001 L CNN "Mouser Part Number"
F 9 "" H 3150 900 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 3150 800 50  0001 L CNN "Arrow Part Number"
F 11 "" H 3150 700 50  0001 L CNN "Arrow Price/Stock"
	1    2100 1500
	1    0    0    -1  
$EndComp
Text Label 2100 1600 0    50   ~ 0
gnd
Text Label 2400 2350 0    50   ~ 0
gnd
Wire Wire Line
	1950 2350 2400 2350
Text GLabel 8350 4200 0    50   BiDi ~ 0
GND
Text GLabel 9550 4200 2    50   Output ~ 0
3.3V
Text GLabel 8350 4500 0    50   BiDi ~ 0
I2C_SENS_CLK
Text GLabel 8350 4400 0    50   BiDi ~ 0
I2C_SENS_D
Text GLabel 9550 4700 2    50   Input ~ 0
BATTERY_LVL
Text GLabel 9550 5100 2    50   Output ~ 0
SPI_ESP32_CLK
Text GLabel 9550 5000 2    50   Output ~ 0
SPI_ESP32_CS
Text GLabel 9550 4900 2    50   Input ~ 0
SPI_SENS_D1
Text GLabel 9550 4800 2    50   Input ~ 0
SPI_SENS_D2
Wire Wire Line
	9400 4200 9550 4200
Wire Wire Line
	9550 4700 9400 4700
Wire Wire Line
	9400 4800 9550 4800
Wire Wire Line
	9400 4900 9550 4900
Wire Wire Line
	9400 5000 9550 5000
Wire Wire Line
	9400 5100 9550 5100
Wire Wire Line
	8350 4500 8500 4500
Wire Wire Line
	8350 4400 8500 4400
Wire Wire Line
	8350 4200 8500 4200
Text GLabel 8350 4700 0    50   Output ~ 0
PWM_MOTOR_OUT
Wire Wire Line
	8500 4700 8350 4700
Text GLabel 9550 5300 2    50   Input ~ 0
5V
Wire Wire Line
	9400 5300 9550 5300
Wire Notes Line
	6700 3000 11100 3000
Wire Notes Line
	11100 3000 11100 6300
Wire Notes Line
	11100 6300 6700 6300
Wire Notes Line
	6700 6300 6700 3000
NoConn ~ 8500 4600
NoConn ~ 9400 4300
NoConn ~ 9400 4500
NoConn ~ 9400 4600
Text GLabel 1500 5200 2    50   Input ~ 0
5V
Text GLabel 1500 5350 2    50   Input ~ 0
GND
Wire Notes Line
	11100 600  11100 2700
Wire Notes Line
	6700 2700 11100 2700
Wire Notes Line
	11100 600  6700 600 
Wire Notes Line
	3850 600  3850 2700
Wire Notes Line
	650  600  650  2700
Wire Notes Line
	6550 3000 6550 6300
Wire Notes Line
	650  3000 650  6300
Wire Wire Line
	8500 1650 8500 2600
Wire Notes Line
	6550 600  6550 1300
Wire Notes Line
	4000 600  4000 1300
Wire Notes Line
	4000 1400 6550 1400
Wire Notes Line
	6550 1400 6550 2700
Wire Notes Line
	6550 2700 4000 2700
Wire Notes Line
	4000 2700 4000 1400
Text GLabel 5800 1550 2    50   Input ~ 0
5V
Text GLabel 4800 1550 0    50   Input ~ 0
3.3V
Text GLabel 4750 2600 0    50   Input ~ 0
SPI_ESP32_CS
Text GLabel 4800 1950 0    50   Input ~ 0
SPI_ESP32_CLK
Text GLabel 5800 2600 2    50   Output ~ 0
SPI_SENS_CS
Text GLabel 5800 1950 2    50   Output ~ 0
SPI_SENS_CLK
Wire Notes Line
	6550 600  4000 600 
Wire Notes Line
	4000 1300 6550 1300
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 61AB83B9
P 5250 850
F 0 "J1" H 5330 842 50  0000 L CNN
F 1 "Screw_Terminal_01x02" H 5330 751 50  0000 L CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 5250 850 50  0001 C CNN
F 3 "~" H 5250 850 50  0001 C CNN
	1    5250 850 
	1    0    0    -1  
$EndComp
$Comp
L HorseFOT_thomas_v1-rescue:press_sensor-ttgo-esp32 P1
U 1 1 61AE05D7
P 1200 3550
F 0 "P1" H 1142 3685 50  0000 C CNN
F 1 "press_sensor" H 1142 3776 50  0000 C CNN
F 2 "TTGO-ESP32-KICAD:ELVR-L01D-pressure_sensor" H 1200 3700 50  0001 C CNN
F 3 "" H 1200 3700 50  0001 C CNN
	1    1200 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	2100 4150 2100 4200
NoConn ~ 1450 4350
$Comp
L HorseFOT_thomas_v1-rescue:press_sensor-ttgo-esp32 P2
U 1 1 61AF7BD6
P 1250 4850
F 0 "P2" H 1192 4985 50  0000 C CNN
F 1 "flow_sensor" H 1192 5076 50  0000 C CNN
F 2 "TTGO-ESP32-KICAD:ELVR-L01D-pressure_sensor" H 1250 5000 50  0001 C CNN
F 3 "" H 1250 5000 50  0001 C CNN
	1    1250 4850
	-1   0    0    1   
$EndComp
NoConn ~ 1500 5650
Wire Wire Line
	7250 1450 7650 1450
Text GLabel 8350 4300 0    50   BiDi ~ 0
GND
Text GLabel 8350 5100 0    50   BiDi ~ 0
GND
Text GLabel 8350 5200 0    50   BiDi ~ 0
GND
Text GLabel 9750 5200 2    50   BiDi ~ 0
GND
Wire Wire Line
	9750 5200 9400 5200
Wire Wire Line
	8500 5200 8350 5200
Wire Wire Line
	8500 5100 8350 5100
Wire Wire Line
	8500 4300 8350 4300
Text GLabel 9000 2000 0    50   Input ~ 0
5V
Text GLabel 9050 2300 0    50   Input ~ 0
GND
$Comp
L Device:C CopAMP1
U 1 1 61AF42E7
P 9250 2150
F 0 "CopAMP1" H 9365 2196 50  0000 L CNN
F 1 "100nF" H 9365 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 9288 2000 50  0001 C CNN
F 3 "~" H 9250 2150 50  0001 C CNN
	1    9250 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2000 9250 2000
Connection ~ 9250 2000
Wire Wire Line
	9050 2300 9250 2300
Connection ~ 9250 2300
Wire Wire Line
	9250 2300 9750 2300
Wire Wire Line
	9250 2000 9750 2000
$Comp
L Device:C CopAMP2
U 1 1 61B22ADD
P 9750 2150
F 0 "CopAMP2" H 9865 2196 50  0000 L CNN
F 1 "1nF" H 9865 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.33x1.80mm_HandSolder" H 9788 2000 50  0001 C CNN
F 3 "~" H 9750 2150 50  0001 C CNN
	1    9750 2150
	1    0    0    -1  
$EndComp
$Comp
L HorseFOT_thomas_v1-rescue:BSS138PW-SamacSys_Parts Q1
U 1 1 61B0343C
P 5200 1650
F 0 "Q1" V 5675 1750 50  0000 C CNN
F 1 "BSS138PW" V 5766 1750 50  0000 C CNN
F 2 "SamacSys_Parts:SOT65P210X110-3N" H 5650 1600 50  0001 L CNN
F 3 "https://datasheet.datasheetarchive.com/originals/distributors/Datasheets-NXP/DSANXP010002286.pdf" H 5650 1500 50  0001 L CNN
F 4 "MOSFET,n-channel,60V,320mA,0.9ohm,SOT323 NXP BSS138PW N-channel MOSFET Transistor, 320 mA, 60 V, 3-Pin SOT-323" H 5650 1400 50  0001 L CNN "Description"
F 5 "1.1" H 5650 1300 50  0001 L CNN "Height"
F 6 "Nexperia" H 5650 1200 50  0001 L CNN "Manufacturer_Name"
F 7 "BSS138PW" H 5650 1100 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "N/A" H 5650 1000 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.com/Search/Refine.aspx?Keyword=N%2FA" H 5650 900 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 5650 800 50  0001 L CNN "Arrow Part Number"
F 11 "" H 5650 700 50  0001 L CNN "Arrow Price/Stock"
	1    5200 1650
	0    1    1    0   
$EndComp
$Comp
L HorseFOT_thomas_v1-rescue:BSS138PW-SamacSys_Parts Q2
U 1 1 61B05766
P 5200 2300
F 0 "Q2" V 5675 2400 50  0000 C CNN
F 1 "BSS138PW" V 5766 2400 50  0000 C CNN
F 2 "SamacSys_Parts:SOT65P210X110-3N" H 5650 2250 50  0001 L CNN
F 3 "https://datasheet.datasheetarchive.com/originals/distributors/Datasheets-NXP/DSANXP010002286.pdf" H 5650 2150 50  0001 L CNN
F 4 "MOSFET,n-channel,60V,320mA,0.9ohm,SOT323 NXP BSS138PW N-channel MOSFET Transistor, 320 mA, 60 V, 3-Pin SOT-323" H 5650 2050 50  0001 L CNN "Description"
F 5 "1.1" H 5650 1950 50  0001 L CNN "Height"
F 6 "Nexperia" H 5650 1850 50  0001 L CNN "Manufacturer_Name"
F 7 "BSS138PW" H 5650 1750 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "N/A" H 5650 1650 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.com/Search/Refine.aspx?Keyword=N%2FA" H 5650 1550 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 5650 1450 50  0001 L CNN "Arrow Part Number"
F 11 "" H 5650 1350 50  0001 L CNN "Arrow Price/Stock"
	1    5200 2300
	0    1    1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 61B0768F
P 4900 2400
F 0 "R9" H 4970 2446 50  0000 L CNN
F 1 "R" H 4970 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder" V 4830 2400 50  0001 C CNN
F 3 "~" H 4900 2400 50  0001 C CNN
	1    4900 2400
	1    0    0    -1  
$EndComp
Text GLabel 4800 2200 0    50   Input ~ 0
3.3V
$Comp
L Device:R R11
U 1 1 61B73424
P 5700 2400
F 0 "R11" H 5770 2446 50  0000 L CNN
F 1 "R" H 5770 2355 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder" V 5630 2400 50  0001 C CNN
F 3 "~" H 5700 2400 50  0001 C CNN
	1    5700 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 61B77D62
P 5700 1750
F 0 "R10" H 5770 1796 50  0000 L CNN
F 1 "R" H 5770 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder" V 5630 1750 50  0001 C CNN
F 3 "~" H 5700 1750 50  0001 C CNN
	1    5700 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 61B7C7BE
P 4900 1750
F 0 "R8" H 4970 1796 50  0000 L CNN
F 1 "R" H 4970 1705 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric_Pad1.30x1.75mm_HandSolder" V 4830 1750 50  0001 C CNN
F 3 "~" H 4900 1750 50  0001 C CNN
	1    4900 1750
	1    0    0    -1  
$EndComp
Text GLabel 5850 2150 2    50   Input ~ 0
5V
Wire Wire Line
	4800 1550 4900 1550
Wire Wire Line
	4900 1550 4900 1600
Wire Wire Line
	4800 1950 4900 1950
Wire Wire Line
	4900 1900 4900 1950
Connection ~ 4900 1950
Wire Wire Line
	4900 1950 5000 1950
Wire Wire Line
	5600 1950 5700 1950
Wire Wire Line
	5700 1900 5700 1950
Connection ~ 5700 1950
Wire Wire Line
	5700 1950 5800 1950
Wire Wire Line
	5700 1600 5700 1550
Wire Wire Line
	5700 1550 5800 1550
Wire Wire Line
	4900 1550 5200 1550
Wire Wire Line
	5200 1550 5200 1650
Connection ~ 4900 1550
Wire Wire Line
	4800 2200 4900 2200
Wire Wire Line
	5200 2200 5200 2300
Wire Wire Line
	4900 2200 4900 2250
Connection ~ 4900 2200
Wire Wire Line
	4900 2200 5200 2200
Wire Wire Line
	4750 2600 4900 2600
Wire Wire Line
	4900 2600 4900 2550
Wire Wire Line
	4900 2600 5000 2600
Connection ~ 4900 2600
Wire Wire Line
	5600 2600 5700 2600
Wire Wire Line
	5700 2550 5700 2600
Connection ~ 5700 2600
Wire Wire Line
	5700 2600 5800 2600
Wire Wire Line
	5700 2150 5700 2250
Wire Wire Line
	5700 2150 5850 2150
$Comp
L Device:D D1
U 1 1 61C2096B
P 7750 1350
F 0 "D1" H 7750 1133 50  0000 C CNN
F 1 "D" H 7750 1224 50  0000 C CNN
F 2 "Diode_SMD:D_1206_3216Metric" H 7750 1350 50  0001 C CNN
F 3 "~" H 7750 1350 50  0001 C CNN
	1    7750 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	7250 1350 7600 1350
Wire Wire Line
	7900 1350 8100 1350
Connection ~ 8100 1350
$Comp
L Amplifier_Operational:MCP602 U5
U 3 1 61B2467A
P 10500 2150
F 0 "U5" H 10458 2196 50  0000 L CNN
F 1 "MCP602" H 10458 2105 50  0000 L CNN
F 2 "SamacSys_Parts:SOP65P490X110-8N" H 10500 2150 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 10500 2150 50  0001 C CNN
	3    10500 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 2000 9750 1850
Wire Wire Line
	9750 1850 10400 1850
Connection ~ 9750 2000
Wire Wire Line
	9750 2300 9750 2450
Wire Wire Line
	9750 2450 10400 2450
Connection ~ 9750 2300
$Comp
L Amplifier_Operational:MCP602 U5
U 2 1 61BD5D5D
P 3500 5600
F 0 "U5" H 3500 5967 50  0000 C CNN
F 1 "MCP602" H 3500 5876 50  0000 C CNN
F 2 "SamacSys_Parts:SOP65P490X110-8N" H 3500 5600 50  0001 C CNN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/21314g.pdf" H 3500 5600 50  0001 C CNN
	2    3500 5600
	1    0    0    -1  
$EndComp
Connection ~ 3800 5600
$Comp
L HorseFOT_thomas_v1-rescue:MS563702BA03-50-SamacSys_Parts IC2
U 1 1 61B92103
P 1800 6900
F 0 "IC2" H 2300 7165 50  0000 C CNN
F 1 "MS563702BA03-50" H 2300 7074 50  0000 C CNN
F 2 "SamacSys_Parts:MS5637_1" H 2650 7000 50  0001 L CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=MS5637-02BA03&DocType=Data+Sheet&DocLang=English&PartCntxt=MS563702BA03-50&DocFormat=pdf" H 2650 6900 50  0001 L CNN
F 4 "Sensor Pressure 2bar Barometric SMD MS563702BA03-50, Barometric Pressure Sensor, 1200mbar 0 ??? 3.6 V Output, 4-Pin QFN" H 2650 6800 50  0001 L CNN "Description"
F 5 "" H 2650 6700 50  0001 L CNN "Height"
F 6 "TE Connectivity" H 2650 6600 50  0001 L CNN "Manufacturer_Name"
F 7 "MS563702BA03-50" H 2650 6500 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "824-MS563702BA03-50" H 2650 6400 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/Measurement-Specialties/MS563702BA03-50?qs=ZSTLft15nNCFYZwlCm%2FnTA%3D%3D" H 2650 6300 50  0001 L CNN "Mouser Price/Stock"
F 10 "MS563702BA03-50" H 2650 6200 50  0001 L CNN "Arrow Part Number"
F 11 "https://www.arrow.com/en/products/ms563702ba03-50/te-connectivity?region=nac" H 2650 6100 50  0001 L CNN "Arrow Price/Stock"
	1    1800 6900
	1    0    0    -1  
$EndComp
Text GLabel 1700 6900 0    50   Input ~ 0
3.3V
Text GLabel 2950 6900 2    50   Input ~ 0
GND
Text GLabel 2950 7000 2    50   Input ~ 0
I2C_SENS_CLK
Text GLabel 1700 7000 0    50   Input ~ 0
I2C_SENS_D
Wire Wire Line
	1700 6900 1800 6900
Wire Wire Line
	1700 7000 1800 7000
Wire Wire Line
	2800 6900 2950 6900
Wire Wire Line
	2800 7000 2950 7000
Wire Notes Line
	650  6400 3650 6400
Wire Notes Line
	3650 6400 3650 7600
Wire Notes Line
	650  7600 650  6400
Wire Notes Line
	650  7600 3650 7600
$Comp
L HorseFOT_thomas_v1-rescue:DM3CS-SF-SamacSys_Parts J2
U 1 1 61BD9C45
P 4750 6750
F 0 "J2" H 5400 7015 50  0000 C CNN
F 1 "DM3CS-SF" H 5400 6924 50  0000 C CNN
F 2 "" H 5900 6850 50  0001 L CNN
F 3 "https://datasheet.lcsc.com/szlcsc/Hirose-HRS-DM3CS-SF_C202111.pdf" H 5900 6750 50  0001 L CNN
F 4 "Memory Card Connectors 8P R/A SMT MICRO SD HINGE PUSH-PULL" H 5900 6650 50  0001 L CNN "Description"
F 5 "1.83" H 5900 6550 50  0001 L CNN "Height"
F 6 "Hirose" H 5900 6450 50  0001 L CNN "Manufacturer_Name"
F 7 "DM3CS-SF" H 5900 6350 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "798-DM3CSSF" H 5900 6250 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/Hirose-Connector/DM3CS-SF?qs=tTj%252BmQ3KZwbB9K5zh89Rcw%3D%3D" H 5900 6150 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 5900 6050 50  0001 L CNN "Arrow Part Number"
F 11 "" H 5900 5950 50  0001 L CNN "Arrow Price/Stock"
	1    4750 6750
	1    0    0    -1  
$EndComp
NoConn ~ 6050 6850
NoConn ~ 6050 6950
NoConn ~ 6050 7050
NoConn ~ 6050 7150
NoConn ~ 6050 7250
Text GLabel 4650 7250 0    50   Input ~ 0
GND
Text GLabel 4650 7050 0    50   Input ~ 0
3.3V
NoConn ~ 6050 6750
NoConn ~ 4750 6750
$Comp
L HorseFOT_thomas_v1-rescue:TTGO-ESP32-thomas-ttgo-esp32 ESP32-TTGO_vT1
U 1 1 61A5870E
P 8500 5450
F 0 "ESP32-TTGO_vT1" H 8950 6987 60  0000 C CNN
F 1 "TTGO-ESP32" H 8950 6881 60  0000 C CNN
F 2 "TTGO-ESP32-KICAD:ESP32-TTGO_vT" H 8500 5450 60  0001 C CNN
F 3 "" H 8500 5450 60  0001 C CNN
	1    8500 5450
	1    0    0    -1  
$EndComp
Text GLabel 8350 4800 0    50   Output ~ 0
SD_CS
Text GLabel 8350 4900 0    50   Output ~ 0
SD_MOSI
Text GLabel 8350 5000 0    50   Input ~ 0
SD_MISO
Wire Wire Line
	8500 4800 8350 4800
Wire Wire Line
	8500 4900 8350 4900
Wire Wire Line
	8500 5000 8350 5000
Text GLabel 9550 4400 2    50   Output ~ 0
SD_CLK
Wire Wire Line
	9550 4400 9400 4400
Text GLabel 4650 6850 0    50   Input ~ 0
SD_CS
Text GLabel 4650 7150 0    50   Input ~ 0
SD_CLK
Text GLabel 4650 6950 0    50   Input ~ 0
SD_MOSI
Text GLabel 4650 7350 0    50   Output ~ 0
SD_MISO
Wire Wire Line
	4650 6850 4750 6850
Wire Wire Line
	4750 6950 4650 6950
Wire Wire Line
	4750 7350 4650 7350
Wire Wire Line
	4750 7250 4650 7250
Wire Wire Line
	4750 7150 4650 7150
Wire Wire Line
	4750 7050 4650 7050
Wire Notes Line
	3700 7600 6550 7600
Wire Notes Line
	6550 7600 6550 6400
Wire Notes Line
	6550 6400 3700 6400
Wire Notes Line
	3700 6400 3700 7600
Text GLabel 8350 5300 0    50   Output ~ 0
3.3V
Wire Wire Line
	8500 5300 8350 5300
$EndSCHEMATC
