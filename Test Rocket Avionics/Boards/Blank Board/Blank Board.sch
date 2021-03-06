EESchema Schematic File Version 4
LIBS:RF Board-cache
EELAYER 29 0
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
L RF-Board-rescue:Conn_01x01-Avionics-Board-1-rescue J13
U 1 1 5C4E553E
P 4750 10100
F 0 "J13" H 4750 10200 50  0000 C CNN
F 1 "CHASSIS_GND" H 4750 10000 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 4750 10100 50  0001 C CNN
F 3 "" H 4750 10100 50  0001 C CNN
	1    4750 10100
	1    0    0    -1  
$EndComp
$Comp
L RF-Board-rescue:GND-Avionics-Board-1-rescue #PWR040
U 1 1 5C4E55C0
P 4400 10250
F 0 "#PWR040" H 4400 10000 50  0001 C CNN
F 1 "GND" H 4400 10100 50  0000 C CNN
F 2 "" H 4400 10250 50  0001 C CNN
F 3 "" H 4400 10250 50  0001 C CNN
	1    4400 10250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 10100 4550 10100
Text Label 1900 9450 1    60   ~ 0
BUZZER
Text Label 1800 9450 1    60   ~ 0
SWDIO
Text Label 1700 9450 1    60   ~ 0
SWCLK
$Comp
L RF-Board-rescue:+3.3V-Avionics-Board-1-rescue #PWR045
U 1 1 5C4FAC8D
P 2100 10150
F 0 "#PWR045" H 2100 10000 50  0001 C CNN
F 1 "+3.3V" H 2100 10290 50  0000 C CNN
F 2 "" H 2100 10150 50  0001 C CNN
F 3 "" H 2100 10150 50  0001 C CNN
	1    2100 10150
	1    0    0    1   
$EndComp
Text Label 1900 9950 3    60   ~ 0
MOSI
Text Label 1800 9950 3    60   ~ 0
SCK
Text Label 1700 9950 3    60   ~ 0
MISO
Text Label 2700 9950 3    60   ~ 0
MAX_M8_TX
Text Label 2600 9950 3    60   ~ 0
MAX_M8_RX
$Comp
L RF-Board-rescue:GND-Avionics-Board-1-rescue #PWR046
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
L RF-Board-rescue:GND-Avionics-Board-1-rescue #PWR049
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
L RF-Board-rescue:GND-Avionics-Board-1-rescue #PWR050
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
	2300 9950 2300 10000
Wire Wire Line
	2400 9950 2400 10000
Wire Wire Line
	2300 10000 2400 10000
Connection ~ 2300 10000
Wire Wire Line
	2500 10000 2500 9950
Connection ~ 2400 10000
Text Label 2900 9950 3    60   ~ 0
SCL
Text Label 2800 9950 3    60   ~ 0
SDA
Wire Wire Line
	4400 10100 4400 10250
Wire Wire Line
	2300 10000 2300 10100
Wire Wire Line
	2400 10000 2500 10000
Wire Wire Line
	2100 10150 2100 10050
Wire Wire Line
	2200 9950 2200 10050
Wire Wire Line
	2200 10050 2100 10050
Connection ~ 2100 10050
Wire Wire Line
	2100 10050 2100 9950
$Comp
L Connector_Generic:Conn_02x13_Odd_Even J1
U 1 1 5CB1EFA8
P 2300 9750
F 0 "J1" V 2396 9062 50  0000 R CNN
F 1 "EXPANSION_HEADER" V 2305 9062 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x13_Pitch1.00mm" H 2300 9750 50  0001 C CNN
F 3 "~" H 2300 9750 50  0001 C CNN
	1    2300 9750
	0    -1   -1   0   
$EndComp
Text Label 2800 9450 1    60   ~ 0
USB_D+
Text Label 2900 9450 1    60   ~ 0
USB_D-
Connection ~ 4400 10100
Wire Wire Line
	4400 9500 4400 9800
Wire Wire Line
	4400 9800 4400 10100
Connection ~ 4400 9500
Wire Wire Line
	4400 9200 4550 9200
Connection ~ 4400 9800
Wire Wire Line
	4400 9500 4550 9500
Wire Wire Line
	4400 9800 4550 9800
Wire Wire Line
	4400 9200 4400 9500
$Comp
L RF-Board-rescue:Conn_01x01-Avionics-Board-1-rescue J12
U 1 1 5C4E548D
P 4750 9800
F 0 "J12" H 4750 9900 50  0000 C CNN
F 1 "CHASSIS_GND" H 4750 9700 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 4750 9800 50  0001 C CNN
F 3 "" H 4750 9800 50  0001 C CNN
	1    4750 9800
	1    0    0    -1  
$EndComp
$Comp
L RF-Board-rescue:Conn_01x01-Avionics-Board-1-rescue J11
U 1 1 5C4E53E1
P 4750 9500
F 0 "J11" H 4750 9600 50  0000 C CNN
F 1 "CHASSIS_GND" H 4750 9400 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 4750 9500 50  0001 C CNN
F 3 "" H 4750 9500 50  0001 C CNN
	1    4750 9500
	1    0    0    -1  
$EndComp
$Comp
L RF-Board-rescue:Conn_01x01-Avionics-Board-1-rescue J10
U 1 1 5C4E4C2D
P 4750 9200
F 0 "J10" H 4750 9300 50  0000 C CNN
F 1 "CHASSIS_GND" H 4750 9100 50  0000 C CNN
F 2 "Mounting_Holes:MountingHole_2.7mm_M2.5_Pad" H 4750 9200 50  0001 C CNN
F 3 "" H 4750 9200 50  0001 C CNN
	1    4750 9200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
