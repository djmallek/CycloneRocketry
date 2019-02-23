EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:FoxWestern
EELAYER 25 0
EELAYER END
$Descr A3 16535 11693
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
L ATSAMD21G18 U3
U 1 1 5C4D5029
P 4350 5500
F 0 "U3" H 5500 4150 60  0000 C CNN
F 1 "ATSAMD21G18" H 4650 5050 60  0000 C CNN
F 2 "Housings_QFP:TQFP-48_7x7mm_Pitch0.5mm" H 4350 5500 60  0001 C CNN
F 3 "" H 4350 5500 60  0001 C CNN
	1    4350 5500
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5C4D506A
P 4600 4650
F 0 "C7" H 4625 4750 50  0000 L CNN
F 1 "1u" H 4625 4550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4638 4500 50  0001 C CNN
F 3 "" H 4600 4650 50  0001 C CNN
	1    4600 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 4800 4600 4950
$Comp
L +3.3V #PWR01
U 1 1 5C4D50F9
P 4500 4950
F 0 "#PWR01" H 4500 4800 50  0001 C CNN
F 1 "+3.3V" H 4500 5090 50  0000 C CNN
F 2 "" H 4500 4950 50  0001 C CNN
F 3 "" H 4500 4950 50  0001 C CNN
	1    4500 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 4950 4700 4450
Wire Wire Line
	4700 4450 4600 4450
Wire Wire Line
	4600 4400 4600 4500
$Comp
L GND #PWR02
U 1 1 5C4D516D
P 4600 4400
F 0 "#PWR02" H 4600 4150 50  0001 C CNN
F 1 "GND" H 4600 4250 50  0000 C CNN
F 2 "" H 4600 4400 50  0001 C CNN
F 3 "" H 4600 4400 50  0001 C CNN
	1    4600 4400
	-1   0    0    1   
$EndComp
Connection ~ 4600 4450
Text Label 3600 5450 2    60   ~ 0
OSC1
Text Label 3600 5550 2    60   ~ 0
OSC2
$Comp
L C C4
U 1 1 5C4D51F4
P 3200 5750
F 0 "C4" H 3225 5850 50  0000 L CNN
F 1 "1u" H 3225 5650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3238 5600 50  0001 C CNN
F 3 "" H 3200 5750 50  0001 C CNN
	1    3200 5750
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 5750 3600 5750
$Comp
L GND #PWR03
U 1 1 5C4D5259
P 3000 5750
F 0 "#PWR03" H 3000 5500 50  0001 C CNN
F 1 "GND" H 3000 5600 50  0000 C CNN
F 2 "" H 3000 5750 50  0001 C CNN
F 3 "" H 3000 5750 50  0001 C CNN
	1    3000 5750
	0    1    1    0   
$EndComp
Wire Wire Line
	3000 5750 3050 5750
$Comp
L +3.3V #PWR04
U 1 1 5C4D5291
P 3550 5950
F 0 "#PWR04" H 3550 5800 50  0001 C CNN
F 1 "+3.3V" H 3550 6090 50  0000 C CNN
F 2 "" H 3550 5950 50  0001 C CNN
F 3 "" H 3550 5950 50  0001 C CNN
	1    3550 5950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3550 5950 3600 5950
Wire Wire Line
	3600 5850 3050 5850
Wire Wire Line
	3050 5850 3050 5750
$Comp
L +3.3V #PWR05
U 1 1 5C4D5400
P 4500 7150
F 0 "#PWR05" H 4500 7000 50  0001 C CNN
F 1 "+3.3V" H 4500 7290 50  0000 C CNN
F 2 "" H 4500 7150 50  0001 C CNN
F 3 "" H 4500 7150 50  0001 C CNN
	1    4500 7150
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 7150 4500 7000
$Comp
L GND #PWR06
U 1 1 5C4D5422
P 4600 7300
F 0 "#PWR06" H 4600 7050 50  0001 C CNN
F 1 "GND" H 4600 7150 50  0000 C CNN
F 2 "" H 4600 7300 50  0001 C CNN
F 3 "" H 4600 7300 50  0001 C CNN
	1    4600 7300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 7300 4600 7000
$Comp
L GND #PWR07
U 1 1 5C4D548E
P 6000 5550
F 0 "#PWR07" H 6000 5300 50  0001 C CNN
F 1 "GND" H 6000 5400 50  0000 C CNN
F 2 "" H 6000 5550 50  0001 C CNN
F 3 "" H 6000 5550 50  0001 C CNN
	1    6000 5550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5700 5550 6000 5550
$Comp
L +3.3V #PWR08
U 1 1 5C4D5508
P 5900 5400
F 0 "#PWR08" H 5900 5250 50  0001 C CNN
F 1 "+3.3V" H 5900 5540 50  0000 C CNN
F 2 "" H 5900 5400 50  0001 C CNN
F 3 "" H 5900 5400 50  0001 C CNN
	1    5900 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 5400 5900 5450
Wire Wire Line
	5900 5450 5700 5450
$Comp
L MCP73831 U4
U 1 1 5C4D5646
P 6800 1400
F 0 "U4" H 7000 1200 60  0000 C CNN
F 1 "MCP73831" H 6800 1600 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:TSOT-23-5" H 6450 1200 60  0001 C CNN
F 3 "" H 6450 1200 60  0001 C CNN
	1    6800 1400
	1    0    0    -1  
$EndComp
$Comp
L USB_A J1
U 1 1 5C4D56CF
P 950 1250
F 0 "J1" H 750 1700 50  0000 L CNN
F 1 "USB_A" H 750 1600 50  0000 L CNN
F 2 "Connectors:USB_Mini-B" H 1100 1200 50  0001 C CNN
F 3 "" H 1100 1200 50  0001 C CNN
	1    950  1250
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D1
U 1 1 5C4D57D2
P 2700 1150
F 0 "D1" H 2700 1250 50  0000 C CNN
F 1 "MBRA140" H 2700 1050 50  0000 C CNN
F 2 "DannysLib:DO-214AC" H 2700 1150 50  0001 C CNN
F 3 "" H 2700 1150 50  0001 C CNN
	1    2700 1150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 1300 6350 1300
$Comp
L LED D2
U 1 1 5C4D58CF
P 5700 1550
F 0 "D2" H 5700 1650 50  0000 C CNN
F 1 "LED" H 5700 1450 50  0000 C CNN
F 2 "Diodes_SMD:D_0603" H 5700 1550 50  0001 C CNN
F 3 "" H 5700 1550 50  0001 C CNN
	1    5700 1550
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 5C4D5902
P 5900 1800
F 0 "R3" V 5980 1800 50  0000 C CNN
F 1 "330" V 5900 1800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5830 1800 50  0001 C CNN
F 3 "" H 5900 1800 50  0001 C CNN
	1    5900 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	5700 1800 5700 1700
$Comp
L GND #PWR09
U 1 1 5C4D5982
P 7300 1550
F 0 "#PWR09" H 7300 1300 50  0001 C CNN
F 1 "GND" H 7300 1400 50  0000 C CNN
F 2 "" H 7300 1550 50  0001 C CNN
F 3 "" H 7300 1550 50  0001 C CNN
	1    7300 1550
	1    0    0    -1  
$EndComp
Text Label 1250 1250 0    60   ~ 0
USB_D+
Text Label 1250 1350 0    60   ~ 0
USB_D-
Text Label 5700 5650 0    60   ~ 0
USB_D+
Text Label 5700 5750 0    60   ~ 0
USB_D-
$Comp
L C C9
U 1 1 5C4D615D
P 5350 1550
F 0 "C9" H 5375 1650 50  0000 L CNN
F 1 "4.7u" H 5375 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 5388 1400 50  0001 C CNN
F 3 "" H 5350 1550 50  0001 C CNN
	1    5350 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1400 5700 1300
Connection ~ 5700 1300
Wire Wire Line
	5750 1800 5700 1800
Wire Wire Line
	6050 1800 6350 1800
Wire Wire Line
	6350 1800 6350 1500
Wire Wire Line
	7300 1550 7300 1500
Wire Wire Line
	7300 1500 7250 1500
$Comp
L R R9
U 1 1 5C4D648C
P 7450 1600
F 0 "R9" V 7530 1600 50  0000 C CNN
F 1 "2k" V 7450 1600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7380 1600 50  0001 C CNN
F 3 "" H 7450 1600 50  0001 C CNN
	1    7450 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7450 1450 7450 1400
Wire Wire Line
	7450 1400 7250 1400
$Comp
L GND #PWR010
U 1 1 5C4D65E3
P 7450 1750
F 0 "#PWR010" H 7450 1500 50  0001 C CNN
F 1 "GND" H 7450 1600 50  0000 C CNN
F 2 "" H 7450 1750 50  0001 C CNN
F 3 "" H 7450 1750 50  0001 C CNN
	1    7450 1750
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 5C4D6674
P 7750 1500
F 0 "C13" H 7775 1600 50  0000 L CNN
F 1 "10u" H 7775 1400 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 7788 1350 50  0001 C CNN
F 3 "" H 7750 1500 50  0001 C CNN
	1    7750 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1250 7750 1350
Wire Wire Line
	7750 1300 7250 1300
Connection ~ 7750 1300
$Comp
L GND #PWR011
U 1 1 5C4D6788
P 7750 1750
F 0 "#PWR011" H 7750 1500 50  0001 C CNN
F 1 "GND" H 7750 1600 50  0000 C CNN
F 2 "" H 7750 1750 50  0001 C CNN
F 3 "" H 7750 1750 50  0001 C CNN
	1    7750 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 1750 7750 1650
$Comp
L GND #PWR012
U 1 1 5C4D6A49
P 950 1750
F 0 "#PWR012" H 950 1500 50  0001 C CNN
F 1 "GND" H 950 1600 50  0000 C CNN
F 2 "" H 950 1750 50  0001 C CNN
F 3 "" H 950 1750 50  0001 C CNN
	1    950  1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  1750 950  1650
Wire Wire Line
	950  1650 850  1650
$Comp
L GND #PWR013
U 1 1 5C4D6C75
P 5350 1700
F 0 "#PWR013" H 5350 1450 50  0001 C CNN
F 1 "GND" H 5350 1550 50  0000 C CNN
F 2 "" H 5350 1700 50  0001 C CNN
F 3 "" H 5350 1700 50  0001 C CNN
	1    5350 1700
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR014
U 1 1 5C4D7394
P 7750 1250
F 0 "#PWR014" H 7750 1100 50  0001 C CNN
F 1 "+BATT" H 7750 1390 50  0000 C CNN
F 2 "" H 7750 1250 50  0001 C CNN
F 3 "" H 7750 1250 50  0001 C CNN
	1    7750 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 1200 5350 1400
$Comp
L VBUS #PWR015
U 1 1 5C4CBE65
P 5350 1200
F 0 "#PWR015" H 5350 1050 50  0001 C CNN
F 1 "VBUS" H 5350 1350 50  0000 C CNN
F 2 "" H 5350 1200 50  0001 C CNN
F 3 "" H 5350 1200 50  0001 C CNN
	1    5350 1200
	1    0    0    -1  
$EndComp
Connection ~ 5350 1300
$Comp
L VBUS #PWR016
U 1 1 5C4CBF8D
P 1300 950
F 0 "#PWR016" H 1300 800 50  0001 C CNN
F 1 "VBUS" H 1300 1100 50  0000 C CNN
F 2 "" H 1300 950 50  0001 C CNN
F 3 "" H 1300 950 50  0001 C CNN
	1    1300 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 950  1300 1050
Wire Wire Line
	1300 1050 1250 1050
$Comp
L +BATT #PWR017
U 1 1 5C4CC045
P 2700 950
F 0 "#PWR017" H 2700 800 50  0001 C CNN
F 1 "+BATT" H 2700 1090 50  0000 C CNN
F 2 "" H 2700 950 50  0001 C CNN
F 3 "" H 2700 950 50  0001 C CNN
	1    2700 950 
	1    0    0    -1  
$EndComp
$Comp
L VBUS #PWR018
U 1 1 5C4CC0F0
P 2400 950
F 0 "#PWR018" H 2400 800 50  0001 C CNN
F 1 "VBUS" H 2400 1100 50  0000 C CNN
F 2 "" H 2400 950 50  0001 C CNN
F 3 "" H 2400 950 50  0001 C CNN
	1    2400 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 950  2700 1000
Wire Wire Line
	2400 950  2400 1350
Wire Wire Line
	2400 1350 3500 1350
Wire Wire Line
	2700 1300 2700 1400
Connection ~ 2700 1350
$Comp
L C C3
U 1 1 5C4CC1B6
P 2700 1550
F 0 "C3" H 2725 1650 50  0000 L CNN
F 1 "10u" H 2725 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 2738 1400 50  0001 C CNN
F 3 "" H 2700 1550 50  0001 C CNN
	1    2700 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 5C4CC281
P 2700 1750
F 0 "#PWR019" H 2700 1500 50  0001 C CNN
F 1 "GND" H 2700 1600 50  0000 C CNN
F 2 "" H 2700 1750 50  0001 C CNN
F 3 "" H 2700 1750 50  0001 C CNN
	1    2700 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1750 2700 1700
$Comp
L SPX3819M5-L-3-3 U2
U 1 1 5C4CC2F0
P 3800 1450
F 0 "U2" H 3650 1675 50  0000 C CNN
F 1 "SPX3819M5-L-3-3" H 3800 1675 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 3800 1775 50  0001 C CNN
F 3 "" H 3800 1450 50  0001 C CNN
	1    3800 1450
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5C4CC3F3
P 3300 1450
F 0 "R1" V 3380 1450 50  0000 C CNN
F 1 "10k" V 3300 1450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 3230 1450 50  0001 C CNN
F 3 "" H 3300 1450 50  0001 C CNN
	1    3300 1450
	0    1    1    0   
$EndComp
Wire Wire Line
	3450 1450 3500 1450
Wire Wire Line
	3150 1450 3150 1350
Connection ~ 3150 1350
$Comp
L GND #PWR020
U 1 1 5C4CC50D
P 3800 1800
F 0 "#PWR020" H 3800 1550 50  0001 C CNN
F 1 "GND" H 3800 1650 50  0000 C CNN
F 2 "" H 3800 1800 50  0001 C CNN
F 3 "" H 3800 1800 50  0001 C CNN
	1    3800 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1800 3800 1750
NoConn ~ 4100 1450
$Comp
L C C5
U 1 1 5C4CC58C
P 4300 1550
F 0 "C5" H 4325 1650 50  0000 L CNN
F 1 "10u" H 4325 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 4338 1400 50  0001 C CNN
F 3 "" H 4300 1550 50  0001 C CNN
	1    4300 1550
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 5C4CC629
P 4650 1550
F 0 "C8" H 4675 1650 50  0000 L CNN
F 1 "1u" H 4675 1450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4688 1400 50  0001 C CNN
F 3 "" H 4650 1550 50  0001 C CNN
	1    4650 1550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 5C4CC661
P 4300 1800
F 0 "#PWR021" H 4300 1550 50  0001 C CNN
F 1 "GND" H 4300 1650 50  0000 C CNN
F 2 "" H 4300 1800 50  0001 C CNN
F 3 "" H 4300 1800 50  0001 C CNN
	1    4300 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR022
U 1 1 5C4CC6A3
P 4650 1800
F 0 "#PWR022" H 4650 1550 50  0001 C CNN
F 1 "GND" H 4650 1650 50  0000 C CNN
F 2 "" H 4650 1800 50  0001 C CNN
F 3 "" H 4650 1800 50  0001 C CNN
	1    4650 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4650 1800 4650 1700
Wire Wire Line
	4300 1700 4300 1800
Wire Wire Line
	4100 1350 4650 1350
Wire Wire Line
	4300 1350 4300 1400
Wire Wire Line
	4650 1300 4650 1400
Connection ~ 4300 1350
$Comp
L +3.3V #PWR023
U 1 1 5C4CC85E
P 4650 1300
F 0 "#PWR023" H 4650 1150 50  0001 C CNN
F 1 "+3.3V" H 4650 1440 50  0000 C CNN
F 2 "" H 4650 1300 50  0001 C CNN
F 3 "" H 4650 1300 50  0001 C CNN
	1    4650 1300
	1    0    0    -1  
$EndComp
Connection ~ 4650 1350
$Comp
L +BATT #PWR024
U 1 1 5C4CD437
P 1900 950
F 0 "#PWR024" H 1900 800 50  0001 C CNN
F 1 "+BATT" H 1900 1090 50  0000 C CNN
F 2 "" H 1900 950 50  0001 C CNN
F 3 "" H 1900 950 50  0001 C CNN
	1    1900 950 
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 5C4CD46B
P 1900 1450
F 0 "#PWR025" H 1900 1200 50  0001 C CNN
F 1 "GND" H 1900 1300 50  0000 C CNN
F 2 "" H 1900 1450 50  0001 C CNN
F 3 "" H 1900 1450 50  0001 C CNN
	1    1900 1450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J2
U 1 1 5C4CD4BF
P 2100 1150
F 0 "J2" H 2100 1250 50  0000 C CNN
F 1 "1S LiPo" H 2100 950 50  0000 C CNN
F 2 "DannysLib:JST-PH2" H 2100 1150 50  0001 C CNN
F 3 "" H 2100 1150 50  0001 C CNN
	1    2100 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 1450 1900 1250
Wire Wire Line
	1900 1150 1900 950 
$Comp
L R R12
U 1 1 5C4CF3BE
P 8900 1250
F 0 "R12" V 8980 1250 50  0000 C CNN
F 1 "10k" V 8900 1250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 8830 1250 50  0001 C CNN
F 3 "" H 8900 1250 50  0001 C CNN
	1    8900 1250
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 5C4CF443
P 8900 1650
F 0 "R13" V 8980 1650 50  0000 C CNN
F 1 "10k" V 8900 1650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 8830 1650 50  0001 C CNN
F 3 "" H 8900 1650 50  0001 C CNN
	1    8900 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 1400 8900 1500
$Comp
L GND #PWR026
U 1 1 5C4CF530
P 8900 1850
F 0 "#PWR026" H 8900 1600 50  0001 C CNN
F 1 "GND" H 8900 1700 50  0000 C CNN
F 2 "" H 8900 1850 50  0001 C CNN
F 3 "" H 8900 1850 50  0001 C CNN
	1    8900 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 1850 8900 1800
Text Label 3600 5650 2    60   ~ 0
A0
Text Label 3600 6050 2    60   ~ 0
A1
Text Label 3600 6150 2    60   ~ 0
A2
Text Label 3600 6250 2    60   ~ 0
A3
Text Label 3600 6350 2    60   ~ 0
A4
Text Label 3600 6450 2    60   ~ 0
D8
Text Label 3600 6550 2    60   ~ 0
D9
Text Label 3350 5650 2    60   ~ 0
BATTERY_VOLTAGE
Wire Wire Line
	3350 5650 3600 5650
$Comp
L +BATT #PWR027
U 1 1 5C4CFE62
P 8900 1050
F 0 "#PWR027" H 8900 900 50  0001 C CNN
F 1 "+BATT" H 8900 1190 50  0000 C CNN
F 2 "" H 8900 1050 50  0001 C CNN
F 3 "" H 8900 1050 50  0001 C CNN
	1    8900 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 1050 8900 1100
Text Label 8800 1450 2    60   ~ 0
BATTERY_VOLTAGE
Wire Wire Line
	8800 1450 8900 1450
Connection ~ 8900 1450
$Comp
L ECS-.327-12.5-17X-TR U1
U 1 1 5C4D0C1A
P 1400 3750
F 0 "U1" V 1050 3500 60  0000 C CNN
F 1 "ECS-.327-12.5-17X-TR" V 1450 3700 60  0000 C CNN
F 2 "DannysLib:XC1195CT-ND" H 1400 3750 60  0001 C CNN
F 3 "" H 1400 3750 60  0001 C CNN
	1    1400 3750
	0    -1   -1   0   
$EndComp
$Comp
L C C1
U 1 1 5C4D0E8A
P 1200 4600
F 0 "C1" H 1225 4700 50  0000 L CNN
F 1 "22p" H 1225 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1238 4450 50  0001 C CNN
F 3 "" H 1200 4600 50  0001 C CNN
	1    1200 4600
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5C4D0EEC
P 1650 4600
F 0 "C2" H 1675 4700 50  0000 L CNN
F 1 "22p" H 1675 4500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1688 4450 50  0001 C CNN
F 3 "" H 1650 4600 50  0001 C CNN
	1    1650 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 4450 1200 4300
Wire Wire Line
	1200 4300 1350 4300
Wire Wire Line
	1500 4300 1650 4300
Wire Wire Line
	1650 4300 1650 4450
Wire Wire Line
	1200 4750 1200 4850
Wire Wire Line
	1200 4850 1650 4850
Wire Wire Line
	1650 4850 1650 4750
$Comp
L GND #PWR028
U 1 1 5C4D1096
P 1400 4950
F 0 "#PWR028" H 1400 4700 50  0001 C CNN
F 1 "GND" H 1400 4800 50  0000 C CNN
F 2 "" H 1400 4950 50  0001 C CNN
F 3 "" H 1400 4950 50  0001 C CNN
	1    1400 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 4950 1400 4850
Connection ~ 1400 4850
Text Label 1200 4300 2    60   ~ 0
OSC1
Text Label 1650 4300 0    60   ~ 0
OSC2
Text Label 4100 7000 3    60   ~ 0
D4
Text Label 4200 7000 3    60   ~ 0
D3
Text Label 4300 7000 3    60   ~ 0
D1
Text Label 4400 7000 3    60   ~ 0
D0
Text Label 4700 7000 3    60   ~ 0
MOSI
Text Label 4800 7000 3    60   ~ 0
SCK
Text Label 4900 7000 3    60   ~ 0
MISO
Text Label 5100 7000 3    60   ~ 0
D2
Text Label 5200 7000 3    60   ~ 0
D5
NoConn ~ 5000 7000
Text Label 5700 6450 0    60   ~ 0
D13
Text Label 5700 6550 0    60   ~ 0
D11
Text Label 5700 6350 0    60   ~ 0
D10
Text Label 5700 6250 0    60   ~ 0
D12
Text Label 5700 6150 0    60   ~ 0
D6
Text Label 5700 6050 0    60   ~ 0
D7
Text Label 5700 5950 0    60   ~ 0
SDA
Text Label 5700 5850 0    60   ~ 0
SCL
Text Label 5200 4950 1    60   ~ 0
TXD
Text Label 5100 4950 1    60   ~ 0
RXD
Text Label 5000 4950 1    60   ~ 0
TXLED
Text Label 4900 4950 1    60   ~ 0
~RESET
Text Label 4800 4950 1    60   ~ 0
USBHOSTEN
Text Label 4400 4950 1    60   ~ 0
SWCLK
Text Label 4300 4950 1    60   ~ 0
SWDIO
Text Label 4200 4950 1    60   ~ 0
A5
Text Label 4100 4950 1    60   ~ 0
RXLED
Wire Wire Line
	5700 6550 6200 6550
Wire Wire Line
	5700 6450 6200 6450
Text Label 6200 6450 0    60   ~ 0
MAX_M8_RX
Text Label 6200 6550 0    60   ~ 0
MAX_M8_TX
Wire Wire Line
	4100 7000 4100 7250
Text Label 4100 7250 3    60   ~ 0
SD_CS
Wire Wire Line
	3600 6350 3200 6350
Wire Wire Line
	3650 6450 3200 6450
Wire Wire Line
	3650 6550 3200 6550
Text Label 3200 6350 2    60   ~ 0
LORA_RESET
Text Label 3200 6450 2    60   ~ 0
LORA_INTERRUPT
Text Label 3200 6550 2    60   ~ 0
RADIO_CS
$Comp
L BNO055 U7
U 1 1 5C4D6FC8
P 10100 5750
F 0 "U7" H 10800 5150 60  0000 C CNN
F 1 "BNO055" H 9350 6300 60  0000 C CNN
F 2 "DannysLib:BNO055-LGA" H 10100 5750 60  0001 C CNN
F 3 "" H 10100 5750 60  0001 C CNN
	1    10100 5750
	-1   0    0    1   
$EndComp
Text Label 9250 5900 2    60   ~ 0
SCL
Text Label 9600 6550 3    60   ~ 0
SDA
$Comp
L GND #PWR029
U 1 1 5C4D72DF
P 9600 4900
F 0 "#PWR029" H 9600 4650 50  0001 C CNN
F 1 "GND" H 9600 4750 50  0000 C CNN
F 2 "" H 9600 4900 50  0001 C CNN
F 3 "" H 9600 4900 50  0001 C CNN
	1    9600 4900
	-1   0    0    1   
$EndComp
Wire Wire Line
	9600 4900 9600 5000
NoConn ~ 9700 6550
NoConn ~ 9800 6550
NoConn ~ 9900 6550
NoConn ~ 10000 6550
Text Label 10300 6550 3    60   ~ 0
BNO_OSC_IN
Text Label 10200 6550 3    60   ~ 0
BNO_OSC_OUT
$Comp
L C C16
U 1 1 5C4D7641
P 10250 7450
F 0 "C16" H 10275 7550 50  0000 L CNN
F 1 "6.8n" H 10275 7350 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 10288 7300 50  0001 C CNN
F 3 "" H 10250 7450 50  0001 C CNN
	1    10250 7450
	0    1    1    0   
$EndComp
$Comp
L C C17
U 1 1 5C4D76D9
P 10250 7700
F 0 "C17" H 10275 7800 50  0000 L CNN
F 1 "120n" H 10275 7600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 10288 7550 50  0001 C CNN
F 3 "" H 10250 7700 50  0001 C CNN
	1    10250 7700
	0    1    1    0   
$EndComp
Wire Wire Line
	10100 6550 10100 7800
Connection ~ 10100 7450
Wire Wire Line
	10400 6550 10400 7700
Connection ~ 10400 7450
$Comp
L +3.3V #PWR030
U 1 1 5C4D8599
P 10400 7700
F 0 "#PWR030" H 10400 7550 50  0001 C CNN
F 1 "+3.3V" H 10400 7840 50  0000 C CNN
F 2 "" H 10400 7700 50  0001 C CNN
F 3 "" H 10400 7700 50  0001 C CNN
	1    10400 7700
	0    1    1    0   
$EndComp
$Comp
L GND #PWR031
U 1 1 5C4D85E7
P 10100 7800
F 0 "#PWR031" H 10100 7550 50  0001 C CNN
F 1 "GND" H 10100 7650 50  0000 C CNN
F 2 "" H 10100 7800 50  0001 C CNN
F 3 "" H 10100 7800 50  0001 C CNN
	1    10100 7800
	1    0    0    -1  
$EndComp
Connection ~ 10100 7700
NoConn ~ 10500 6550
$Comp
L +3.3V #PWR032
U 1 1 5C4D9F16
P 11350 5700
F 0 "#PWR032" H 11350 5550 50  0001 C CNN
F 1 "+3.3V" H 11350 5840 50  0000 C CNN
F 2 "" H 11350 5700 50  0001 C CNN
F 3 "" H 11350 5700 50  0001 C CNN
	1    11350 5700
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 5C4D9F62
P 11050 5700
F 0 "R16" V 11130 5700 50  0000 C CNN
F 1 "10k" V 11050 5700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 10980 5700 50  0001 C CNN
F 3 "" H 11050 5700 50  0001 C CNN
	1    11050 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	10850 5700 10900 5700
Wire Wire Line
	10850 5800 11350 5800
Wire Wire Line
	11350 5800 11350 5700
Wire Wire Line
	11200 5700 11200 5800
Connection ~ 11200 5800
$Comp
L GND #PWR033
U 1 1 5C4DA377
P 10900 5950
F 0 "#PWR033" H 10900 5700 50  0001 C CNN
F 1 "GND" H 10900 5800 50  0000 C CNN
F 2 "" H 10900 5950 50  0001 C CNN
F 3 "" H 10900 5950 50  0001 C CNN
	1    10900 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10900 5950 10900 5900
Wire Wire Line
	10900 5900 10850 5900
$Comp
L GND #PWR034
U 1 1 5C4DA6A5
P 10900 5550
F 0 "#PWR034" H 10900 5300 50  0001 C CNN
F 1 "GND" H 10900 5400 50  0000 C CNN
F 2 "" H 10900 5550 50  0001 C CNN
F 3 "" H 10900 5550 50  0001 C CNN
	1    10900 5550
	-1   0    0    1   
$EndComp
Wire Wire Line
	10900 5550 10900 5600
Wire Wire Line
	10900 5600 10850 5600
$Comp
L C C15
U 1 1 5C4DB080
P 10200 4700
F 0 "C15" H 10225 4800 50  0000 L CNN
F 1 "100n" H 10225 4600 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 10238 4550 50  0001 C CNN
F 3 "" H 10200 4700 50  0001 C CNN
	1    10200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 4850 10200 5000
Wire Wire Line
	10500 4400 10500 5000
Wire Wire Line
	10500 4450 10200 4450
Wire Wire Line
	10200 4450 10200 4550
$Comp
L GND #PWR035
U 1 1 5C4DB4B6
P 10500 4400
F 0 "#PWR035" H 10500 4150 50  0001 C CNN
F 1 "GND" H 10500 4250 50  0000 C CNN
F 2 "" H 10500 4400 50  0001 C CNN
F 3 "" H 10500 4400 50  0001 C CNN
	1    10500 4400
	-1   0    0    1   
$EndComp
Connection ~ 10500 4450
NoConn ~ 10400 5000
NoConn ~ 10300 5000
NoConn ~ 10100 5000
NoConn ~ 9700 5000
NoConn ~ 9800 5000
NoConn ~ 9900 5000
$Comp
L R R15
U 1 1 5C4DB9C1
P 10000 4700
F 0 "R15" V 10080 4700 50  0000 C CNN
F 1 "10k" V 10000 4700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 9930 4700 50  0001 C CNN
F 3 "" H 10000 4700 50  0001 C CNN
	1    10000 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 4850 10000 5000
$Comp
L GND #PWR036
U 1 1 5C4DBAF1
P 10000 4450
F 0 "#PWR036" H 10000 4200 50  0001 C CNN
F 1 "GND" H 10000 4300 50  0000 C CNN
F 2 "" H 10000 4450 50  0001 C CNN
F 3 "" H 10000 4450 50  0001 C CNN
	1    10000 4450
	-1   0    0    1   
$EndComp
Wire Wire Line
	10000 4450 10000 4550
$Comp
L GND #PWR037
U 1 1 5C4DC103
P 8850 5900
F 0 "#PWR037" H 8850 5650 50  0001 C CNN
F 1 "GND" H 8850 5750 50  0000 C CNN
F 2 "" H 8850 5900 50  0001 C CNN
F 3 "" H 8850 5900 50  0001 C CNN
	1    8850 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 5600 8850 5900
Wire Wire Line
	8850 5800 9250 5800
Wire Wire Line
	9250 5600 8850 5600
Connection ~ 8850 5800
$Comp
L R R14
U 1 1 5C4DC270
P 9050 5700
F 0 "R14" V 9130 5700 50  0000 C CNN
F 1 "10k" V 9050 5700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 8980 5700 50  0001 C CNN
F 3 "" H 9050 5700 50  0001 C CNN
	1    9050 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	9200 5700 9250 5700
Wire Wire Line
	8900 5700 8850 5700
Connection ~ 8850 5700
$Comp
L ECS-.327-12.5-17X-TR U8
U 1 1 5C4DD7A7
P 12000 6400
F 0 "U8" V 11650 6150 60  0000 C CNN
F 1 "ECS-.327-12.5-17X-TR" V 12050 6350 60  0000 C CNN
F 2 "DannysLib:XC1195CT-ND" H 12000 6400 60  0001 C CNN
F 3 "" H 12000 6400 60  0001 C CNN
	1    12000 6400
	0    -1   -1   0   
$EndComp
$Comp
L C C18
U 1 1 5C4DD7AD
P 11800 7250
F 0 "C18" H 11825 7350 50  0000 L CNN
F 1 "22p" H 11825 7150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 11838 7100 50  0001 C CNN
F 3 "" H 11800 7250 50  0001 C CNN
	1    11800 7250
	1    0    0    -1  
$EndComp
$Comp
L C C19
U 1 1 5C4DD7B3
P 12250 7250
F 0 "C19" H 12275 7350 50  0000 L CNN
F 1 "22p" H 12275 7150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 12288 7100 50  0001 C CNN
F 3 "" H 12250 7250 50  0001 C CNN
	1    12250 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	11800 7100 11800 6950
Wire Wire Line
	11800 6950 11950 6950
Wire Wire Line
	12100 6950 12250 6950
Wire Wire Line
	12250 6950 12250 7100
Wire Wire Line
	11800 7400 11800 7500
Wire Wire Line
	11800 7500 12250 7500
Wire Wire Line
	12250 7500 12250 7400
$Comp
L GND #PWR038
U 1 1 5C4DD7C0
P 12000 7600
F 0 "#PWR038" H 12000 7350 50  0001 C CNN
F 1 "GND" H 12000 7450 50  0000 C CNN
F 2 "" H 12000 7600 50  0001 C CNN
F 3 "" H 12000 7600 50  0001 C CNN
	1    12000 7600
	1    0    0    -1  
$EndComp
Wire Wire Line
	12000 7600 12000 7500
Connection ~ 12000 7500
Text Label 11800 6950 2    60   ~ 0
BNO_OSC_OUT
Text Label 12250 6950 0    60   ~ 0
BNO_OSC_IN
$Comp
L LED D3
U 1 1 5C4E1AA1
P 6150 9200
F 0 "D3" H 6150 9300 50  0000 C CNN
F 1 "LED" H 6150 9100 50  0000 C CNN
F 2 "Diodes_SMD:D_0603" H 6150 9200 50  0001 C CNN
F 3 "" H 6150 9200 50  0001 C CNN
	1    6150 9200
	0    -1   -1   0   
$EndComp
$Comp
L LED D4
U 1 1 5C4E1B4A
P 6400 9200
F 0 "D4" H 6400 9300 50  0000 C CNN
F 1 "LED" H 6400 9100 50  0000 C CNN
F 2 "Diodes_SMD:D_0603" H 6400 9200 50  0001 C CNN
F 3 "" H 6400 9200 50  0001 C CNN
	1    6400 9200
	0    -1   -1   0   
$EndComp
$Comp
L LED D5
U 1 1 5C4E1BDB
P 6650 9200
F 0 "D5" H 6650 9300 50  0000 C CNN
F 1 "LED" H 6650 9100 50  0000 C CNN
F 2 "Diodes_SMD:D_0603" H 6650 9200 50  0001 C CNN
F 3 "" H 6650 9200 50  0001 C CNN
	1    6650 9200
	0    -1   -1   0   
$EndComp
$Comp
L R R4
U 1 1 5C4E1CA9
P 6150 9600
F 0 "R4" V 6230 9600 50  0000 C CNN
F 1 "470" V 6150 9600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6080 9600 50  0001 C CNN
F 3 "" H 6150 9600 50  0001 C CNN
	1    6150 9600
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5C4E1D63
P 6400 9600
F 0 "R5" V 6480 9600 50  0000 C CNN
F 1 "470" V 6400 9600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6330 9600 50  0001 C CNN
F 3 "" H 6400 9600 50  0001 C CNN
	1    6400 9600
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5C4E2022
P 6650 9600
F 0 "R6" V 6730 9600 50  0000 C CNN
F 1 "470" V 6650 9600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6580 9600 50  0001 C CNN
F 3 "" H 6650 9600 50  0001 C CNN
	1    6650 9600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 9350 6150 9450
Wire Wire Line
	6400 9350 6400 9450
Wire Wire Line
	6650 9350 6650 9450
$Comp
L GND #PWR039
U 1 1 5C4E22B9
P 6400 9850
F 0 "#PWR039" H 6400 9600 50  0001 C CNN
F 1 "GND" H 6400 9700 50  0000 C CNN
F 2 "" H 6400 9850 50  0001 C CNN
F 3 "" H 6400 9850 50  0001 C CNN
	1    6400 9850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 9750 6150 9800
Wire Wire Line
	6150 9800 6650 9800
Wire Wire Line
	6400 9750 6400 9850
Connection ~ 6400 9800
Wire Wire Line
	6650 9800 6650 9750
Text Label 6150 9050 1    60   ~ 0
D12
Text Label 6400 9050 1    60   ~ 0
D6
Text Label 6650 9050 1    60   ~ 0
D7
$Comp
L Conn_01x01 J10
U 1 1 5C4E4C2D
P 10900 8500
F 0 "J10" H 10900 8600 50  0000 C CNN
F 1 "CHASSIS_GND" H 10900 8400 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 10900 8500 50  0001 C CNN
F 3 "" H 10900 8500 50  0001 C CNN
	1    10900 8500
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J11
U 1 1 5C4E53E1
P 10900 8800
F 0 "J11" H 10900 8900 50  0000 C CNN
F 1 "CHASSIS_GND" H 10900 8700 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 10900 8800 50  0001 C CNN
F 3 "" H 10900 8800 50  0001 C CNN
	1    10900 8800
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J12
U 1 1 5C4E548D
P 10900 9100
F 0 "J12" H 10900 9200 50  0000 C CNN
F 1 "CHASSIS_GND" H 10900 9000 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 10900 9100 50  0001 C CNN
F 3 "" H 10900 9100 50  0001 C CNN
	1    10900 9100
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x01 J13
U 1 1 5C4E553E
P 10900 9400
F 0 "J13" H 10900 9500 50  0000 C CNN
F 1 "CHASSIS_GND" H 10900 9300 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 10900 9400 50  0001 C CNN
F 3 "" H 10900 9400 50  0001 C CNN
	1    10900 9400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR040
U 1 1 5C4E55C0
P 10550 9550
F 0 "#PWR040" H 10550 9300 50  0001 C CNN
F 1 "GND" H 10550 9400 50  0000 C CNN
F 2 "" H 10550 9550 50  0001 C CNN
F 3 "" H 10550 9550 50  0001 C CNN
	1    10550 9550
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 8500 10550 9550
Wire Wire Line
	10550 9400 10700 9400
Wire Wire Line
	10550 9100 10700 9100
Connection ~ 10550 9400
Wire Wire Line
	10550 8800 10700 8800
Connection ~ 10550 9100
Wire Wire Line
	10550 8500 10700 8500
Connection ~ 10550 8800
Wire Wire Line
	4200 4950 4200 4450
$Comp
L Buzzer BZ1
U 1 1 5C4E6BB4
P 4300 2850
F 0 "BZ1" H 4450 2900 50  0000 L CNN
F 1 "Buzzer" H 4450 2800 50  0000 L CNN
F 2 "DannysLib:CUI_CMT-5023S-SMT" V 4275 2950 50  0001 C CNN
F 3 "" V 4275 2950 50  0001 C CNN
	1    4300 2850
	0    -1   -1   0   
$EndComp
Text Label 4200 4450 1    60   ~ 0
BUZZER
Text Label 4200 3600 3    60   ~ 0
BUZZER
$Comp
L R R2
U 1 1 5C4E74E9
P 4200 3400
F 0 "R2" V 4280 3400 50  0000 C CNN
F 1 "330" V 4200 3400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 4130 3400 50  0001 C CNN
F 3 "" H 4200 3400 50  0001 C CNN
	1    4200 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 2950 4200 3250
Wire Wire Line
	4200 3600 4200 3550
Wire Wire Line
	4400 2950 4400 3050
$Comp
L GND #PWR041
U 1 1 5C4E8153
P 4400 3050
F 0 "#PWR041" H 4400 2800 50  0001 C CNN
F 1 "GND" H 4400 2900 50  0000 C CNN
F 2 "" H 4400 3050 50  0001 C CNN
F 3 "" H 4400 3050 50  0001 C CNN
	1    4400 3050
	1    0    0    -1  
$EndComp
$Comp
L MPL3115A2 U5
U 1 1 5C4EC2C6
P 8250 4150
F 0 "U5" H 8500 3900 60  0000 C CNN
F 1 "MPL3115A2" H 8250 4400 60  0000 C CNN
F 2 "DannysLib:MPL3115A2" H 8250 3750 60  0001 C CNN
F 3 "" H 8250 3750 60  0001 C CNN
	1    8250 4150
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 5C4F0AF7
P 6500 4250
F 0 "C10" H 6525 4350 50  0000 L CNN
F 1 "100n" H 6525 4150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6538 4100 50  0001 C CNN
F 3 "" H 6500 4250 50  0001 C CNN
	1    6500 4250
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 5C4F0BD7
P 6750 4250
F 0 "C11" H 6775 4350 50  0000 L CNN
F 1 "10u" H 6775 4150 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 6788 4100 50  0001 C CNN
F 3 "" H 6750 4250 50  0001 C CNN
	1    6750 4250
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 5C4F0C9B
P 7000 4250
F 0 "C12" H 7025 4350 50  0000 L CNN
F 1 "100n" H 7025 4150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 7038 4100 50  0001 C CNN
F 3 "" H 7000 4250 50  0001 C CNN
	1    7000 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 4000 7750 4000
Wire Wire Line
	6750 4000 6750 4100
Wire Wire Line
	7000 4100 7750 4100
Wire Wire Line
	6500 3900 6500 4100
Connection ~ 6750 4000
Wire Wire Line
	7750 4300 7550 4300
Wire Wire Line
	7550 4300 7550 4000
Connection ~ 7550 4000
Wire Wire Line
	7750 4200 7300 4200
Wire Wire Line
	7300 4200 7300 4450
Wire Wire Line
	7300 4450 6500 4450
Wire Wire Line
	7000 4450 7000 4400
Wire Wire Line
	6750 4450 6750 4400
Connection ~ 7000 4450
Wire Wire Line
	6500 4400 6500 4550
Connection ~ 6750 4450
$Comp
L GND #PWR042
U 1 1 5C4F1901
P 6500 4550
F 0 "#PWR042" H 6500 4300 50  0001 C CNN
F 1 "GND" H 6500 4400 50  0000 C CNN
F 2 "" H 6500 4550 50  0001 C CNN
F 3 "" H 6500 4550 50  0001 C CNN
	1    6500 4550
	1    0    0    -1  
$EndComp
Connection ~ 6500 4450
$Comp
L +3.3V #PWR043
U 1 1 5C4F1A78
P 6500 3900
F 0 "#PWR043" H 6500 3750 50  0001 C CNN
F 1 "+3.3V" H 6500 4040 50  0000 C CNN
F 2 "" H 6500 3900 50  0001 C CNN
F 3 "" H 6500 3900 50  0001 C CNN
	1    6500 3900
	1    0    0    -1  
$EndComp
Connection ~ 6500 4000
Text Label 8750 4000 0    60   ~ 0
SCL
Text Label 8750 4100 0    60   ~ 0
SDA
NoConn ~ 8750 4200
NoConn ~ 8750 4300
$Comp
L R R7
U 1 1 5C4F4F31
P 7000 5350
F 0 "R7" V 7080 5350 50  0000 C CNN
F 1 "2k" V 7000 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 6930 5350 50  0001 C CNN
F 3 "" H 7000 5350 50  0001 C CNN
	1    7000 5350
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5C4F503A
P 7200 5350
F 0 "R8" V 7280 5350 50  0000 C CNN
F 1 "2k" V 7200 5350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7130 5350 50  0001 C CNN
F 3 "" H 7200 5350 50  0001 C CNN
	1    7200 5350
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR044
U 1 1 5C4F52CD
P 7100 5100
F 0 "#PWR044" H 7100 4950 50  0001 C CNN
F 1 "+3.3V" H 7100 5240 50  0000 C CNN
F 2 "" H 7100 5100 50  0001 C CNN
F 3 "" H 7100 5100 50  0001 C CNN
	1    7100 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 5100 7100 5150
Wire Wire Line
	7000 5150 7200 5150
Wire Wire Line
	7000 5150 7000 5200
Wire Wire Line
	7200 5150 7200 5200
Connection ~ 7100 5150
Text Label 7000 5500 3    60   ~ 0
SCL
Text Label 7200 5500 3    60   ~ 0
SDA
Text Label 1900 9450 1    60   ~ 0
BUZZER
Text Label 1800 9450 1    60   ~ 0
SWDIO
Text Label 1700 9450 1    60   ~ 0
SWCLK
$Comp
L +3.3V #PWR045
U 1 1 5C4FAC8D
P 2700 10150
F 0 "#PWR045" H 2700 10000 50  0001 C CNN
F 1 "+3.3V" H 2700 10290 50  0000 C CNN
F 2 "" H 2700 10150 50  0001 C CNN
F 3 "" H 2700 10150 50  0001 C CNN
	1    2700 10150
	1    0    0    1   
$EndComp
Text Label 1800 9950 3    60   ~ 0
MOSI
Text Label 1900 9950 3    60   ~ 0
SCK
Text Label 1700 9950 3    60   ~ 0
MISO
Text Label 2200 9950 3    60   ~ 0
MAX_M8_TX
Text Label 2100 9950 3    60   ~ 0
MAX_M8_RX
$Comp
L GND #PWR046
U 1 1 5C4FE25A
P 2300 10100
F 0 "#PWR046" H 2300 9850 50  0001 C CNN
F 1 "GND" H 2300 9950 50  0000 C CNN
F 2 "" H 2300 10100 50  0001 C CNN
F 3 "" H 2300 10100 50  0001 C CNN
	1    2300 10100
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J14
U 1 1 5C5013D4
P 2200 2050
F 0 "J14" H 2200 2150 50  0000 C CNN
F 1 "1S LiPo" H 2200 1850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch2.54mm" H 2200 2050 50  0001 C CNN
F 3 "" H 2200 2050 50  0001 C CNN
	1    2200 2050
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR047
U 1 1 5C501502
P 1950 2000
F 0 "#PWR047" H 1950 1850 50  0001 C CNN
F 1 "+BATT" H 1950 2140 50  0000 C CNN
F 2 "" H 1950 2000 50  0001 C CNN
F 3 "" H 1950 2000 50  0001 C CNN
	1    1950 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2000 1950 2050
Wire Wire Line
	1950 2050 2000 2050
$Comp
L GND #PWR048
U 1 1 5C5016D6
P 1950 2200
F 0 "#PWR048" H 1950 1950 50  0001 C CNN
F 1 "GND" H 1950 2050 50  0000 C CNN
F 2 "" H 1950 2200 50  0001 C CNN
F 3 "" H 1950 2200 50  0001 C CNN
	1    1950 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2200 1950 2150
Wire Wire Line
	1950 2150 2000 2150
$Comp
L Conn_02x11_Odd_Even J3
U 1 1 5C7119CA
P 2200 9750
F 0 "J3" H 2250 10350 50  0000 C CNN
F 1 "EXPANSION_HEADER" H 2250 9150 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x10_Pitch1.00mm" H 2200 9750 50  0001 C CNN
F 3 "" H 2200 9750 50  0001 C CNN
	1    2200 9750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR049
U 1 1 5C7130A4
P 2000 9950
F 0 "#PWR049" H 2000 9700 50  0001 C CNN
F 1 "GND" H 2000 9800 50  0000 C CNN
F 2 "" H 2000 9950 50  0001 C CNN
F 3 "" H 2000 9950 50  0001 C CNN
	1    2000 9950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR050
U 1 1 5C713157
P 2000 9450
F 0 "#PWR050" H 2000 9200 50  0001 C CNN
F 1 "GND" H 2000 9300 50  0000 C CNN
F 2 "" H 2000 9450 50  0001 C CNN
F 3 "" H 2000 9450 50  0001 C CNN
	1    2000 9450
	-1   0    0    1   
$EndComp
Text Label 2100 9450 1    60   ~ 0
A1
Text Label 2200 9450 1    60   ~ 0
A2
Text Label 2300 9450 1    60   ~ 0
A3
Text Label 2400 9450 1    60   ~ 0
LORA_RESET
Text Label 2500 9450 1    60   ~ 0
LORA_INTERRUPT
Text Label 2600 9450 1    60   ~ 0
RADIO_CS
Text Label 2700 9450 1    60   ~ 0
SD_CS
Wire Wire Line
	2700 10150 2700 9950
Wire Wire Line
	2600 9950 2600 10050
Wire Wire Line
	2600 10050 2700 10050
Connection ~ 2700 10050
Wire Wire Line
	2300 10100 2300 9950
Wire Wire Line
	2400 9950 2400 10000
Wire Wire Line
	2300 10000 2500 10000
Connection ~ 2300 10000
Wire Wire Line
	2500 10000 2500 9950
Connection ~ 2400 10000
$EndSCHEMATC