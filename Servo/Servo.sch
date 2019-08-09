EESchema Schematic File Version 4
LIBS:Servo-cache
EELAYER 29 0
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
L Driver_Motor:L293D U2
U 1 1 5D4D9D9D
P 7000 2600
F 0 "U2" H 7000 3781 50  0000 C CNN
F 1 "L293D" H 7000 3690 50  0000 C CNN
F 2 "Package_DIP:DIP-16_W7.62mm" H 7250 1850 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/l293.pdf" H 6700 3300 50  0001 C CNN
	1    7000 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5D4D4B5C
P 3750 4550
F 0 "#PWR0101" H 3750 4300 50  0001 C CNN
F 1 "GND" H 3755 4377 50  0000 C CNN
F 2 "" H 3750 4550 50  0001 C CNN
F 3 "" H 3750 4550 50  0001 C CNN
	1    3750 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5D4D5BAF
P 6800 3800
F 0 "#PWR0102" H 6800 3550 50  0001 C CNN
F 1 "GND" H 6805 3627 50  0000 C CNN
F 2 "" H 6800 3800 50  0001 C CNN
F 3 "" H 6800 3800 50  0001 C CNN
	1    6800 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 3400 6800 3800
Wire Wire Line
	6900 3400 6800 3400
Connection ~ 6800 3400
Wire Wire Line
	7100 3400 6900 3400
Connection ~ 6900 3400
Wire Wire Line
	7200 3400 7100 3400
Connection ~ 7100 3400
Wire Wire Line
	3750 4350 3750 4550
$Comp
L power:+3.3VA #PWR0106
U 1 1 5D4E119C
P 4800 800
F 0 "#PWR0106" H 4800 650 50  0001 C CNN
F 1 "+3.3VA" H 4815 973 50  0000 C CNN
F 2 "" H 4800 800 50  0001 C CNN
F 3 "" H 4800 800 50  0001 C CNN
	1    4800 800 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0107
U 1 1 5D4E16CC
P 3750 1750
F 0 "#PWR0107" H 3750 1600 50  0001 C CNN
F 1 "+3.3VA" H 3765 1923 50  0000 C CNN
F 2 "" H 3750 1750 50  0001 C CNN
F 3 "" H 3750 1750 50  0001 C CNN
	1    3750 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0108
U 1 1 5D4E1C39
P 1200 2300
F 0 "#PWR0108" H 1200 2150 50  0001 C CNN
F 1 "+3.3VA" H 1215 2473 50  0000 C CNN
F 2 "" H 1200 2300 50  0001 C CNN
F 3 "" H 1200 2300 50  0001 C CNN
	1    1200 2300
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR0109
U 1 1 5D4E5BD6
P 2100 1750
F 0 "#PWR0109" H 1900 1600 50  0001 C CNN
F 1 "Vdrive" H 2117 1923 50  0000 C CNN
F 2 "" H 2100 1750 50  0001 C CNN
F 3 "" H 2100 1750 50  0001 C CNN
	1    2100 1750
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR0110
U 1 1 5D4E6305
P 7200 1100
F 0 "#PWR0110" H 7000 950 50  0001 C CNN
F 1 "Vdrive" H 7217 1273 50  0000 C CNN
F 2 "" H 7200 1100 50  0001 C CNN
F 3 "" H 7200 1100 50  0001 C CNN
	1    7200 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0111
U 1 1 5D4E7251
P 6850 1050
F 0 "#PWR0111" H 6850 900 50  0001 C CNN
F 1 "+3.3VA" H 6865 1223 50  0000 C CNN
F 2 "" H 6850 1050 50  0001 C CNN
F 3 "" H 6850 1050 50  0001 C CNN
	1    6850 1050
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR0112
U 1 1 5D4E8347
P 5950 5800
F 0 "#PWR0112" H 5750 5650 50  0001 C CNN
F 1 "Vdrive" H 5950 5950 50  0000 C CNN
F 2 "" H 5950 5800 50  0001 C CNN
F 3 "" H 5950 5800 50  0001 C CNN
	1    5950 5800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1600 7100 1100
Wire Wire Line
	7100 1100 7200 1100
Wire Wire Line
	6850 1050 6850 1300
Wire Wire Line
	6850 1300 6900 1300
Wire Wire Line
	6900 1300 6900 1600
Text GLabel 2800 2650 0    50   Input ~ 0
SingleShot-2
Wire Wire Line
	3750 1750 3750 2150
$Comp
L Connector_Generic:Conn_01x02 J8
U 1 1 5D54EC66
P 8800 2650
F 0 "J8" H 8880 2642 50  0000 L CNN
F 1 "Motor1" H 8880 2551 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8800 2650 50  0001 C CNN
F 3 "~" H 8800 2650 50  0001 C CNN
	1    8800 2650
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J10
U 1 1 5D550BA0
P 8850 2050
F 0 "J10" H 8930 2042 50  0000 L CNN
F 1 "Motor2" H 8930 1951 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8850 2050 50  0001 C CNN
F 3 "~" H 8850 2050 50  0001 C CNN
	1    8850 2050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J9
U 1 1 5D551C55
P 8850 1300
F 0 "J9" H 8930 1292 50  0000 L CNN
F 1 "Laser" H 8930 1201 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 8850 1300 50  0001 C CNN
F 3 "~" H 8850 1300 50  0001 C CNN
	1    8850 1300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 5D555582
P 6750 5800
F 0 "J6" H 6830 5842 50  0000 L CNN
F 1 "Servo 1" H 6830 5751 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6750 5800 50  0001 C CNN
F 3 "~" H 6750 5800 50  0001 C CNN
	1    6750 5800
	1    0    0    -1  
$EndComp
$Comp
L MCU_Microchip_ATtiny:ATtiny2313A-PU U1
U 1 1 5D4D8AE9
P 3750 3250
F 0 "U1" H 3750 4531 50  0000 C CNN
F 1 "ATtiny2313A-PU" H 3750 4440 50  0000 C CNN
F 2 "Package_DIP:DIP-20_W7.62mm" H 3750 3250 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf" H 3750 3250 50  0001 C CNN
	1    3750 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 5D5685A2
P 2500 1750
F 0 "J3" H 2580 1742 50  0000 L CNN
F 1 "Power" H 2580 1651 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 2500 1750 50  0001 C CNN
F 3 "~" H 2500 1750 50  0001 C CNN
	1    2500 1750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5D569393
P 1900 2450
F 0 "J1" H 1980 2442 50  0000 L CNN
F 1 "Pi" H 1980 2351 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1900 2450 50  0001 C CNN
F 3 "~" H 1900 2450 50  0001 C CNN
	1    1900 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5D571947
P 6550 6200
F 0 "#PWR0103" H 6550 5950 50  0001 C CNN
F 1 "GND" H 6555 6027 50  0000 C CNN
F 2 "" H 6550 6200 50  0001 C CNN
F 3 "" H 6550 6200 50  0001 C CNN
	1    6550 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 6200 6550 5900
Wire Wire Line
	7500 2000 8650 2000
Wire Wire Line
	8650 2000 8650 2050
Wire Wire Line
	7500 2200 8650 2200
Wire Wire Line
	8650 2200 8650 2150
Wire Wire Line
	7500 2600 8600 2600
Wire Wire Line
	8600 2600 8600 2650
Wire Wire Line
	7500 2800 8600 2800
Wire Wire Line
	8600 2800 8600 2750
$Comp
L power:+3.3VA #PWR0104
U 1 1 5D58FF85
P 8550 1100
F 0 "#PWR0104" H 8550 950 50  0001 C CNN
F 1 "+3.3VA" H 8565 1273 50  0000 C CNN
F 2 "" H 8550 1100 50  0001 C CNN
F 3 "" H 8550 1100 50  0001 C CNN
	1    8550 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5D59082D
P 8550 1600
F 0 "#PWR0105" H 8550 1350 50  0001 C CNN
F 1 "GND" H 8555 1427 50  0000 C CNN
F 2 "" H 8550 1600 50  0001 C CNN
F 3 "" H 8550 1600 50  0001 C CNN
	1    8550 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 1100 8550 1300
Wire Wire Line
	8550 1300 8650 1300
Wire Wire Line
	8550 1600 8550 1400
Wire Wire Line
	8550 1400 8650 1400
Wire Wire Line
	2100 1750 2300 1750
$Comp
L power:GND #PWR0113
U 1 1 5D595C44
P 2300 2050
F 0 "#PWR0113" H 2300 1800 50  0001 C CNN
F 1 "GND" H 2305 1877 50  0000 C CNN
F 2 "" H 2300 2050 50  0001 C CNN
F 3 "" H 2300 2050 50  0001 C CNN
	1    2300 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 1850 2300 2050
$Comp
L power:GND #PWR0114
U 1 1 5D5979BC
P 1100 2800
F 0 "#PWR0114" H 1100 2550 50  0001 C CNN
F 1 "GND" H 1105 2627 50  0000 C CNN
F 2 "" H 1100 2800 50  0001 C CNN
F 3 "" H 1100 2800 50  0001 C CNN
	1    1100 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 2350 1200 2350
Wire Wire Line
	1200 2350 1200 2300
Wire Wire Line
	1700 2650 1100 2650
Wire Wire Line
	1100 2650 1100 2800
Text GLabel 1700 5850 0    50   Input ~ 0
SingleShot-2
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J2
U 1 1 5D4D895A
P 4400 1050
F 0 "J2" H 4450 1367 50  0000 C CNN
F 1 "ISP" H 4450 1276 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" H 4400 1050 50  0001 C CNN
F 3 "~" H 4400 1050 50  0001 C CNN
	1    4400 1050
	1    0    0    -1  
$EndComp
Text GLabel 3900 950  0    50   Input ~ 0
MISO
Text GLabel 3900 1050 0    50   Input ~ 0
SCK
Text GLabel 3900 1150 0    50   Input ~ 0
RESET
Text GLabel 4950 1050 2    50   Input ~ 0
MOSI-SDA
$Comp
L power:GND #PWR0115
U 1 1 5D5B8997
P 4800 1350
F 0 "#PWR0115" H 4800 1100 50  0001 C CNN
F 1 "GND" H 4805 1177 50  0000 C CNN
F 2 "" H 4800 1350 50  0001 C CNN
F 3 "" H 4800 1350 50  0001 C CNN
	1    4800 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 950  4200 950 
Wire Wire Line
	3900 1050 4200 1050
Wire Wire Line
	3900 1150 4200 1150
Wire Wire Line
	4700 1150 4800 1150
Wire Wire Line
	4800 1150 4800 1350
Wire Wire Line
	4700 950  4800 950 
Wire Wire Line
	4800 950  4800 800 
Wire Wire Line
	4700 1050 4950 1050
Text GLabel 2800 2450 0    50   Input ~ 0
RESET
Wire Wire Line
	2800 2450 3150 2450
Text GLabel 1550 2450 0    50   Input ~ 0
SCK
Wire Wire Line
	1550 2550 1700 2550
Text GLabel 4600 3150 2    50   Input ~ 0
SCK
Text GLabel 4600 3050 2    50   Input ~ 0
MISO
Text GLabel 4600 2950 2    50   Input ~ 0
MOSI-SDA
Wire Wire Line
	4350 3150 4600 3150
Wire Wire Line
	4350 3050 4600 3050
Wire Wire Line
	4350 2950 4600 2950
Text GLabel 1550 2550 0    50   Input ~ 0
MOSI-SDA
Wire Wire Line
	1550 2450 1700 2450
Text GLabel 4600 2750 2    50   Input ~ 0
Servo-1
Wire Wire Line
	4350 2750 4600 2750
Text GLabel 6450 5700 0    50   Input ~ 0
Servo-1
Text GLabel 4600 2850 2    50   Input ~ 0
Servo-2
Wire Wire Line
	4600 2650 4350 2650
Wire Wire Line
	6450 5700 6550 5700
Wire Wire Line
	5950 5800 6550 5800
$Comp
L Connector_Generic:Conn_01x03 J7
U 1 1 5D62539A
P 6850 4650
F 0 "J7" H 6930 4692 50  0000 L CNN
F 1 "Servo 2" H 6930 4601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6850 4650 50  0001 C CNN
F 3 "~" H 6850 4650 50  0001 C CNN
	1    6850 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5D625E90
P 6650 5000
F 0 "#PWR0116" H 6650 4750 50  0001 C CNN
F 1 "GND" H 6655 4827 50  0000 C CNN
F 2 "" H 6650 5000 50  0001 C CNN
F 3 "" H 6650 5000 50  0001 C CNN
	1    6650 5000
	1    0    0    -1  
$EndComp
Text GLabel 6550 4550 0    50   Input ~ 0
Servo-2
Wire Wire Line
	6550 4550 6650 4550
Wire Wire Line
	6650 4750 6650 5000
$Comp
L power:Vdrive #PWR0117
U 1 1 5D62A032
P 6050 4550
F 0 "#PWR0117" H 5850 4400 50  0001 C CNN
F 1 "Vdrive" H 6050 4700 50  0000 C CNN
F 2 "" H 6050 4550 50  0001 C CNN
F 3 "" H 6050 4550 50  0001 C CNN
	1    6050 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 4550 6050 4650
Wire Wire Line
	6050 4650 6650 4650
Wire Wire Line
	2800 2650 3150 2650
Text GLabel 2800 2850 0    50   Input ~ 0
SingleShot-1
Wire Wire Line
	2800 2850 3150 2850
Text GLabel 1650 5050 0    50   Input ~ 0
SingleShot-1
$Comp
L power:GND #PWR0118
U 1 1 5D64E6FE
P 1950 5250
F 0 "#PWR0118" H 1950 5000 50  0001 C CNN
F 1 "GND" H 1955 5077 50  0000 C CNN
F 2 "" H 1950 5250 50  0001 C CNN
F 3 "" H 1950 5250 50  0001 C CNN
	1    1950 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 5D64F1B0
P 2000 6050
F 0 "#PWR0119" H 2000 5800 50  0001 C CNN
F 1 "GND" H 2005 5877 50  0000 C CNN
F 2 "" H 2000 6050 50  0001 C CNN
F 3 "" H 2000 6050 50  0001 C CNN
	1    2000 6050
	1    0    0    -1  
$EndComp
Text GLabel 4750 3750 2    50   Input ~ 0
M2A
Text GLabel 6250 2000 0    50   Input ~ 0
M2A
Wire Wire Line
	6250 2000 6500 2000
Wire Wire Line
	4750 3750 4350 3750
Text GLabel 4750 3650 2    50   Input ~ 0
M2B
Text GLabel 6250 2200 0    50   Input ~ 0
M2B
Wire Wire Line
	6500 2200 6250 2200
Wire Wire Line
	4350 3650 4750 3650
Text GLabel 6250 2400 0    50   Input ~ 0
M2E
Wire Wire Line
	6250 2400 6500 2400
Text GLabel 4750 3850 2    50   Input ~ 0
M2E
Wire Wire Line
	4750 3850 4350 3850
Text GLabel 4600 2650 2    50   Input ~ 0
M1E
Text GLabel 6300 3000 0    50   Input ~ 0
M1E
Wire Wire Line
	6300 3000 6500 3000
Text GLabel 6250 2600 0    50   Input ~ 0
M1A
Text GLabel 6250 2800 0    50   Input ~ 0
M1B
Text GLabel 4600 2550 2    50   Input ~ 0
M1A
Text GLabel 4600 2450 2    50   Input ~ 0
M1B
Wire Wire Line
	4350 2450 4600 2450
Wire Wire Line
	4600 2550 4350 2550
Wire Wire Line
	6250 2600 6500 2600
Wire Wire Line
	6500 2800 6250 2800
$Comp
L Connector_Generic:Conn_01x03 J4
U 1 1 5D6E42E3
P 2150 5150
F 0 "J4" H 2230 5192 50  0000 L CNN
F 1 "Conn_01x03" H 2230 5101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2150 5150 50  0001 C CNN
F 3 "~" H 2150 5150 50  0001 C CNN
	1    2150 5150
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J5
U 1 1 5D6E5875
P 2200 5950
F 0 "J5" H 2280 5992 50  0000 L CNN
F 1 "Conn_01x03" H 2280 5901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2200 5950 50  0001 C CNN
F 3 "~" H 2200 5950 50  0001 C CNN
	1    2200 5950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0120
U 1 1 5D7117BA
P 1750 4750
F 0 "#PWR0120" H 1750 4600 50  0001 C CNN
F 1 "+3.3VA" H 1765 4923 50  0000 C CNN
F 2 "" H 1750 4750 50  0001 C CNN
F 3 "" H 1750 4750 50  0001 C CNN
	1    1750 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0121
U 1 1 5D7138FA
P 1800 5650
F 0 "#PWR0121" H 1800 5500 50  0001 C CNN
F 1 "+3.3VA" H 1815 5823 50  0000 C CNN
F 2 "" H 1800 5650 50  0001 C CNN
F 3 "" H 1800 5650 50  0001 C CNN
	1    1800 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 4750 1750 5150
Wire Wire Line
	1750 5150 1950 5150
Wire Wire Line
	1650 5050 1950 5050
Wire Wire Line
	1700 5850 2000 5850
Wire Wire Line
	1800 5650 1800 5950
Wire Wire Line
	1800 5950 2000 5950
Wire Wire Line
	4350 2850 4600 2850
Text GLabel 4600 3350 2    50   Input ~ 0
D0
Text GLabel 4600 3450 2    50   Input ~ 0
D1
Text GLabel 4600 3550 2    50   Input ~ 0
D2
Text GLabel 4600 3950 2    50   Input ~ 0
D6
Wire Wire Line
	4350 3350 4600 3350
Wire Wire Line
	4600 3450 4350 3450
Wire Wire Line
	4600 3550 4350 3550
Text GLabel 2950 5050 0    50   Input ~ 0
D0
$Comp
L power:GND #PWR0122
U 1 1 5D76619D
P 3250 5250
F 0 "#PWR0122" H 3250 5000 50  0001 C CNN
F 1 "GND" H 3255 5077 50  0000 C CNN
F 2 "" H 3250 5250 50  0001 C CNN
F 3 "" H 3250 5250 50  0001 C CNN
	1    3250 5250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J12
U 1 1 5D7661A3
P 3450 5150
F 0 "J12" H 3530 5192 50  0000 L CNN
F 1 "Conn_01x03" H 3530 5101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3450 5150 50  0001 C CNN
F 3 "~" H 3450 5150 50  0001 C CNN
	1    3450 5150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0123
U 1 1 5D7661A9
P 3050 4750
F 0 "#PWR0123" H 3050 4600 50  0001 C CNN
F 1 "+3.3VA" H 3065 4923 50  0000 C CNN
F 2 "" H 3050 4750 50  0001 C CNN
F 3 "" H 3050 4750 50  0001 C CNN
	1    3050 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 4750 3050 5150
Wire Wire Line
	3050 5150 3250 5150
Wire Wire Line
	2950 5050 3250 5050
Text GLabel 2900 5850 0    50   Input ~ 0
D2
$Comp
L power:GND #PWR0124
U 1 1 5D768D53
P 3200 6050
F 0 "#PWR0124" H 3200 5800 50  0001 C CNN
F 1 "GND" H 3205 5877 50  0000 C CNN
F 2 "" H 3200 6050 50  0001 C CNN
F 3 "" H 3200 6050 50  0001 C CNN
	1    3200 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J11
U 1 1 5D768D59
P 3400 5950
F 0 "J11" H 3480 5992 50  0000 L CNN
F 1 "Conn_01x03" H 3480 5901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3400 5950 50  0001 C CNN
F 3 "~" H 3400 5950 50  0001 C CNN
	1    3400 5950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0125
U 1 1 5D768D5F
P 3000 5550
F 0 "#PWR0125" H 3000 5400 50  0001 C CNN
F 1 "+3.3VA" H 3015 5723 50  0000 C CNN
F 2 "" H 3000 5550 50  0001 C CNN
F 3 "" H 3000 5550 50  0001 C CNN
	1    3000 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 5550 3000 5950
Wire Wire Line
	3000 5950 3200 5950
Wire Wire Line
	2900 5850 3200 5850
Text GLabel 4350 5050 0    50   Input ~ 0
D1
$Comp
L power:GND #PWR0126
U 1 1 5D76C71E
P 4650 5250
F 0 "#PWR0126" H 4650 5000 50  0001 C CNN
F 1 "GND" H 4655 5077 50  0000 C CNN
F 2 "" H 4650 5250 50  0001 C CNN
F 3 "" H 4650 5250 50  0001 C CNN
	1    4650 5250
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J13
U 1 1 5D76C724
P 4850 5150
F 0 "J13" H 4930 5192 50  0000 L CNN
F 1 "Conn_01x03" H 4930 5101 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4850 5150 50  0001 C CNN
F 3 "~" H 4850 5150 50  0001 C CNN
	1    4850 5150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0127
U 1 1 5D76C72A
P 4450 4750
F 0 "#PWR0127" H 4450 4600 50  0001 C CNN
F 1 "+3.3VA" H 4465 4923 50  0000 C CNN
F 2 "" H 4450 4750 50  0001 C CNN
F 3 "" H 4450 4750 50  0001 C CNN
	1    4450 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 4750 4450 5150
Wire Wire Line
	4450 5150 4650 5150
Wire Wire Line
	4350 5050 4650 5050
Text GLabel 4350 5850 0    50   Input ~ 0
D6
$Comp
L power:GND #PWR0128
U 1 1 5D76F68F
P 4650 6050
F 0 "#PWR0128" H 4650 5800 50  0001 C CNN
F 1 "GND" H 4655 5877 50  0000 C CNN
F 2 "" H 4650 6050 50  0001 C CNN
F 3 "" H 4650 6050 50  0001 C CNN
	1    4650 6050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J14
U 1 1 5D76F695
P 4850 5950
F 0 "J14" H 4930 5992 50  0000 L CNN
F 1 "Conn_01x03" H 4930 5901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4850 5950 50  0001 C CNN
F 3 "~" H 4850 5950 50  0001 C CNN
	1    4850 5950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0129
U 1 1 5D76F69B
P 4450 5550
F 0 "#PWR0129" H 4450 5400 50  0001 C CNN
F 1 "+3.3VA" H 4465 5723 50  0000 C CNN
F 2 "" H 4450 5550 50  0001 C CNN
F 3 "" H 4450 5550 50  0001 C CNN
	1    4450 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 5550 4450 5950
Wire Wire Line
	4450 5950 4650 5950
Wire Wire Line
	4350 5850 4650 5850
Wire Wire Line
	4350 3950 4600 3950
$Comp
L Device:C C1
U 1 1 5D4DE9E6
P 9000 3700
F 0 "C1" H 9115 3746 50  0000 L CNN
F 1 "C" H 9115 3655 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 9038 3550 50  0001 C CNN
F 3 "~" H 9000 3700 50  0001 C CNN
	1    9000 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5D4DF87C
P 9550 3700
F 0 "C2" H 9668 3746 50  0000 L CNN
F 1 "CP" H 9668 3655 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 9588 3550 50  0001 C CNN
F 3 "~" H 9550 3700 50  0001 C CNN
	1    9550 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C3
U 1 1 5D4DFBAB
P 10100 3650
F 0 "C3" H 10218 3696 50  0000 L CNN
F 1 "CP" H 10218 3605 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D8.0mm_P5.00mm" H 10138 3500 50  0001 C CNN
F 3 "~" H 10100 3650 50  0001 C CNN
	1    10100 3650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0130
U 1 1 5D4E213A
P 9000 3550
F 0 "#PWR0130" H 9000 3400 50  0001 C CNN
F 1 "+3.3VA" H 9015 3723 50  0000 C CNN
F 2 "" H 9000 3550 50  0001 C CNN
F 3 "" H 9000 3550 50  0001 C CNN
	1    9000 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR0131
U 1 1 5D4E350D
P 9550 3550
F 0 "#PWR0131" H 9550 3400 50  0001 C CNN
F 1 "+3.3VA" H 9565 3723 50  0000 C CNN
F 2 "" H 9550 3550 50  0001 C CNN
F 3 "" H 9550 3550 50  0001 C CNN
	1    9550 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 5D4E3946
P 9000 3850
F 0 "#PWR0132" H 9000 3600 50  0001 C CNN
F 1 "GND" H 9005 3677 50  0000 C CNN
F 2 "" H 9000 3850 50  0001 C CNN
F 3 "" H 9000 3850 50  0001 C CNN
	1    9000 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 5D4E3E1C
P 9550 3850
F 0 "#PWR0133" H 9550 3600 50  0001 C CNN
F 1 "GND" H 9555 3677 50  0000 C CNN
F 2 "" H 9550 3850 50  0001 C CNN
F 3 "" H 9550 3850 50  0001 C CNN
	1    9550 3850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0134
U 1 1 5D4E426E
P 10100 3800
F 0 "#PWR0134" H 10100 3550 50  0001 C CNN
F 1 "GND" H 10105 3627 50  0000 C CNN
F 2 "" H 10100 3800 50  0001 C CNN
F 3 "" H 10100 3800 50  0001 C CNN
	1    10100 3800
	1    0    0    -1  
$EndComp
$Comp
L power:Vdrive #PWR0135
U 1 1 5D4E5D6E
P 10100 3500
F 0 "#PWR0135" H 9900 3350 50  0001 C CNN
F 1 "Vdrive" H 10117 3673 50  0000 C CNN
F 2 "" H 10100 3500 50  0001 C CNN
F 3 "" H 10100 3500 50  0001 C CNN
	1    10100 3500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
