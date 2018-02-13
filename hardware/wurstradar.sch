EESchema Schematic File Version 2
LIBS:power
LIBS:device
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
LIBS:wurstradar-cache
EELAYER 25 0
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
L Antenna AE?
U 1 1 5A82E702
P 4100 3850
F 0 "AE?" V 4317 3795 50  0000 C CNN
F 1 "Antenna" V 4226 3795 50  0000 C CNN
F 2 "" H 4100 3850 50  0001 C CNN
F 3 "" H 4100 3850 50  0001 C CNN
	1    4100 3850
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5A82E80C
P 5100 3550
F 0 "R?" H 5170 3596 50  0000 L CNN
F 1 "8k8" H 5170 3505 50  0000 L CNN
F 2 "" V 5030 3550 50  0001 C CNN
F 3 "" H 5100 3550 50  0001 C CNN
	1    5100 3550
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A82E877
P 5100 4150
F 0 "R?" H 5170 4196 50  0000 L CNN
F 1 "1k2" H 5170 4105 50  0000 L CNN
F 2 "" V 5030 4150 50  0001 C CNN
F 3 "" H 5100 4150 50  0001 C CNN
	1    5100 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A82E8F0
P 5100 4300
F 0 "#PWR?" H 5100 4050 50  0001 C CNN
F 1 "GND" H 5105 4127 50  0000 C CNN
F 2 "" H 5100 4300 50  0001 C CNN
F 3 "" H 5100 4300 50  0001 C CNN
	1    5100 4300
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A82E995
P 5100 3400
F 0 "#PWR?" H 5100 3250 50  0001 C CNN
F 1 "+3.3V" H 5115 3573 50  0000 C CNN
F 2 "" H 5100 3400 50  0001 C CNN
F 3 "" H 5100 3400 50  0001 C CNN
	1    5100 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 3700 5100 3850
Wire Wire Line
	5100 3850 5100 4000
Wire Wire Line
	4300 3850 4700 3850
Wire Wire Line
	5000 3850 5100 3850
Wire Wire Line
	5100 3850 5850 3850
Connection ~ 5100 3850
$Comp
L LM321 U?
U 1 1 5A82ECB8
P 6150 3950
F 0 "U?" H 6491 3996 50  0000 L CNN
F 1 "LM321" H 6491 3905 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 6150 3950 50  0001 C CNN
F 3 "" H 6150 3950 50  0001 C CNN
	1    6150 3950
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A82EDA6
P 6050 3650
F 0 "#PWR?" H 6050 3500 50  0001 C CNN
F 1 "+3.3V" H 6065 3823 50  0000 C CNN
F 2 "" H 6050 3650 50  0001 C CNN
F 3 "" H 6050 3650 50  0001 C CNN
	1    6050 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A82EE1E
P 6050 4250
F 0 "#PWR?" H 6050 4000 50  0001 C CNN
F 1 "GND" H 6055 4077 50  0000 C CNN
F 2 "" H 6050 4250 50  0001 C CNN
F 3 "" H 6050 4250 50  0001 C CNN
	1    6050 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 4050 5700 4050
Wire Wire Line
	5700 4050 5700 4650
Wire Wire Line
	5700 4650 5700 4700
$Comp
L R Rg
U 1 1 5A82EE6F
P 5700 4850
F 0 "Rg" H 5770 4896 50  0000 L CNN
F 1 "1k" H 5770 4805 50  0000 L CNN
F 2 "" V 5630 4850 50  0001 C CNN
F 3 "" H 5700 4850 50  0001 C CNN
	1    5700 4850
	1    0    0    -1  
$EndComp
$Comp
L R Rf
U 1 1 5A82EF73
P 6250 4650
F 0 "Rf" V 6043 4650 50  0000 C CNN
F 1 "3k5" V 6134 4650 50  0000 C CNN
F 2 "" V 6180 4650 50  0001 C CNN
F 3 "" H 6250 4650 50  0001 C CNN
	1    6250 4650
	0    1    1    0   
$EndComp
Wire Wire Line
	6100 4650 5700 4650
Connection ~ 5700 4650
Wire Wire Line
	6400 4650 6800 4650
Wire Wire Line
	6800 4650 6800 3950
Wire Wire Line
	7250 3950 7550 3950
Wire Wire Line
	7550 3950 7950 3950
$Comp
L R R?
U 1 1 5A82F133
P 7550 3650
F 0 "R?" H 7620 3696 50  0000 L CNN
F 1 "4k7" H 7620 3605 50  0000 L CNN
F 2 "" V 7480 3650 50  0001 C CNN
F 3 "" H 7550 3650 50  0001 C CNN
	1    7550 3650
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5A82F139
P 7550 4250
F 0 "R?" H 7620 4296 50  0000 L CNN
F 1 "4k7" H 7620 4205 50  0000 L CNN
F 2 "" V 7480 4250 50  0001 C CNN
F 3 "" H 7550 4250 50  0001 C CNN
	1    7550 4250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A82F13F
P 7550 4400
F 0 "#PWR?" H 7550 4150 50  0001 C CNN
F 1 "GND" H 7555 4227 50  0000 C CNN
F 2 "" H 7550 4400 50  0001 C CNN
F 3 "" H 7550 4400 50  0001 C CNN
	1    7550 4400
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR?
U 1 1 5A82F145
P 7550 3500
F 0 "#PWR?" H 7550 3350 50  0001 C CNN
F 1 "+3.3V" H 7565 3673 50  0000 C CNN
F 2 "" H 7550 3500 50  0001 C CNN
F 3 "" H 7550 3500 50  0001 C CNN
	1    7550 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 3800 7550 3950
Wire Wire Line
	7550 3950 7550 4100
Connection ~ 7550 3950
Connection ~ 6800 3950
Text Notes 4500 3700 1    60   ~ 0
-300 — +300mV
$Comp
L C C?
U 1 1 5A82F99F
P 4850 3850
F 0 "C?" V 4598 3850 50  0000 C CNN
F 1 "1u" V 4689 3850 50  0000 C CNN
F 2 "" H 4888 3700 50  0001 C CNN
F 3 "" H 4850 3850 50  0001 C CNN
	1    4850 3850
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5A82FAEE
P 7100 3950
F 0 "C?" V 6848 3950 50  0000 C CNN
F 1 "1u" V 6939 3950 50  0000 C CNN
F 2 "" H 7138 3800 50  0001 C CNN
F 3 "" H 7100 3950 50  0001 C CNN
	1    7100 3950
	0    1    1    0   
$EndComp
Wire Wire Line
	6450 3950 6800 3950
Wire Wire Line
	6800 3950 6950 3950
Text Notes 5550 3700 1    60   ~ 0
+100 — +700mV
$Comp
L GND #PWR?
U 1 1 5A8304C8
P 5700 5000
F 0 "#PWR?" H 5700 4750 50  0001 C CNN
F 1 "GND" H 5705 4827 50  0000 C CNN
F 2 "" H 5700 5000 50  0001 C CNN
F 3 "" H 5700 5000 50  0001 C CNN
	1    5700 5000
	1    0    0    -1  
$EndComp
Text Notes 5950 3250 0    60   ~ 0
Amplification:\n1+Rf/Rg\n= 4.5
Text Notes 6800 3700 1    60   ~ 0
+450 — +3150mV
Text Notes 7950 3700 1    60   ~ 0
+300 — +3000mV
Text Notes 6150 2800 1    60   ~ 0
MCP6273T-E/CH
$EndSCHEMATC
