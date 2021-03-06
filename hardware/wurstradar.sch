EESchema Schematic File Version 4
LIBS:wurstradar-cache
EELAYER 30 0
EELAYER END
$Descr A4 8268 11693 portrait
encoding utf-8
Sheet 1 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 2550 6100 650  400 
U 5A8CB6E2
F0 "RX Path" 60
F1 "rxpath.sch" 60
F2 "Iin" I L 2550 6200 60 
F3 "Qin" I L 2550 6400 60 
F4 "Iout" I R 3200 6200 60 
F5 "Qout" I R 3200 6400 60 
$EndSheet
NoConn ~ 1950 6600
$Comp
L power:+5V #PWR?
U 1 1 5A8CE197
P 2050 6500
F 0 "#PWR?" H 2050 6350 50  0001 C CNN
F 1 "+5V" H 2050 6640 50  0000 C CNN
F 2 "" H 2050 6500 50  0001 C CNN
F 3 "" H 2050 6500 50  0001 C CNN
	1    2050 6500
	0    1    1    0   
$EndComp
Wire Wire Line
	2050 6500 1950 6500
$Comp
L power:GND #PWR?
U 1 1 5A8CE1EE
P 2100 6300
F 0 "#PWR?" H 2100 6050 50  0001 C CNN
F 1 "GND" H 2100 6150 50  0000 C CNN
F 2 "" H 2100 6300 50  0001 C CNN
F 3 "" H 2100 6300 50  0001 C CNN
	1    2100 6300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2100 6300 1950 6300
Wire Wire Line
	1950 6400 2550 6400
Wire Wire Line
	1950 6200 2550 6200
$Comp
L power:GND #PWR?
U 1 1 5A8CF3F9
P 2000 4600
F 0 "#PWR?" H 2000 4350 50  0001 C CNN
F 1 "GND" H 2000 4450 50  0000 C CNN
F 2 "" H 2000 4600 50  0001 C CNN
F 3 "" H 2000 4600 50  0001 C CNN
	1    2000 4600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1600 4600 2000 4600
$Comp
L power:+5V #PWR?
U 1 1 5A8CF475
P 1800 4700
F 0 "#PWR?" H 1800 4550 50  0001 C CNN
F 1 "+5V" H 1800 4840 50  0000 C CNN
F 2 "" H 1800 4700 50  0001 C CNN
F 3 "" H 1800 4700 50  0001 C CNN
	1    1800 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 4700 1600 4700
$Comp
L power:GND #PWR?
U 1 1 5A8CF818
P 2000 3900
F 0 "#PWR?" H 2000 3650 50  0001 C CNN
F 1 "GND" H 2000 3750 50  0000 C CNN
F 2 "" H 2000 3900 50  0001 C CNN
F 3 "" H 2000 3900 50  0001 C CNN
	1    2000 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5A8CF859
P 2200 3800
F 0 "#PWR?" H 2200 3650 50  0001 C CNN
F 1 "VCC" H 2200 3950 50  0000 C CNN
F 2 "" H 2200 3800 50  0001 C CNN
F 3 "" H 2200 3800 50  0001 C CNN
	1    2200 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	2200 3800 2100 3800
Wire Wire Line
	1600 3900 2000 3900
$Comp
L power:VCC #PWR?
U 1 1 5A8CF8C9
P 1800 4500
F 0 "#PWR?" H 1800 4350 50  0001 C CNN
F 1 "VCC" H 1800 4650 50  0000 C CNN
F 2 "" H 1800 4500 50  0001 C CNN
F 3 "" H 1800 4500 50  0001 C CNN
	1    1800 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	1800 4500 1600 4500
Wire Wire Line
	3850 3950 3850 4050
Wire Wire Line
	3250 3950 3550 3950
Wire Wire Line
	3250 4450 3550 4450
Wire Wire Line
	3850 4450 3850 4350
Wire Wire Line
	2650 4050 2650 3850
Wire Wire Line
	2650 4350 2650 4550
$Comp
L power:GND #PWR?
U 1 1 5A8D12BD
P 2650 4550
F 0 "#PWR?" H 2650 4300 50  0001 C CNN
F 1 "GND" H 2650 4400 50  0000 C CNN
F 2 "" H 2650 4550 50  0001 C CNN
F 3 "" H 2650 4550 50  0001 C CNN
	1    2650 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5A8D1307
P 3550 4550
F 0 "#PWR?" H 3550 4300 50  0001 C CNN
F 1 "GND" H 3550 4400 50  0000 C CNN
F 2 "" H 3550 4550 50  0001 C CNN
F 3 "" H 3550 4550 50  0001 C CNN
	1    3550 4550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5A8D13B4
P 2650 3850
F 0 "#PWR?" H 2650 3700 50  0001 C CNN
F 1 "VCC" H 2650 4000 50  0000 C CNN
F 2 "" H 2650 3850 50  0001 C CNN
F 3 "" H 2650 3850 50  0001 C CNN
	1    2650 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5A8D13FE
P 3550 3850
F 0 "#PWR?" H 3550 3700 50  0001 C CNN
F 1 "+5V" H 3550 3990 50  0000 C CNN
F 2 "" H 3550 3850 50  0001 C CNN
F 3 "" H 3550 3850 50  0001 C CNN
	1    3550 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 6200 3600 6200
Wire Wire Line
	3200 6400 3600 6400
Wire Wire Line
	3550 4550 3550 4450
Connection ~ 3550 4450
Wire Wire Line
	3550 3950 3550 3850
Connection ~ 3550 3950
Wire Wire Line
	3250 3950 3250 4050
Wire Wire Line
	3250 4450 3250 4350
Wire Wire Line
	3150 7800 3150 8000
Wire Wire Line
	2750 7800 2950 7800
Wire Wire Line
	2750 7800 2750 7900
Wire Wire Line
	3150 8300 3150 8100
Wire Wire Line
	2750 8300 2950 8300
Wire Wire Line
	2750 8300 2750 8200
$Comp
L power:+5V #PWR?
U 1 1 5A959E88
P 2950 7700
F 0 "#PWR?" H 2950 7550 50  0001 C CNN
F 1 "+5V" H 2965 7873 50  0000 C CNN
F 2 "" H 2950 7700 50  0001 C CNN
F 3 "" H 2950 7700 50  0001 C CNN
	1    2950 7700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 7700 2950 7800
Connection ~ 2950 7800
Wire Wire Line
	2950 8300 2950 8550
Connection ~ 2950 8300
$Comp
L power:GND #PWR?
U 1 1 5A959FE0
P 2950 9150
F 0 "#PWR?" H 2950 8900 50  0001 C CNN
F 1 "GND" H 2955 8977 50  0000 C CNN
F 2 "" H 2950 9150 50  0001 C CNN
F 3 "" H 2950 9150 50  0001 C CNN
	1    2950 9150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 8950 2950 9050
Wire Wire Line
	2950 9050 2750 9050
Connection ~ 2950 9050
Wire Wire Line
	1800 9050 2350 9050
Connection ~ 2350 9050
Wire Wire Line
	1800 3800 1600 3800
Wire Wire Line
	3150 8100 3450 8100
Wire Wire Line
	3150 8000 3450 8000
$Comp
L power:GND #PWR?
U 1 1 5A958F28
P 6000 8300
F 0 "#PWR?" H 6000 8050 50  0001 C CNN
F 1 "GND" V 6005 8172 50  0000 R CNN
F 2 "" H 6000 8300 50  0001 C CNN
F 3 "" H 6000 8300 50  0001 C CNN
	1    6000 8300
	0    1    1    0   
$EndComp
Wire Wire Line
	6000 8300 6100 8300
Wire Wire Line
	6100 8400 5400 8400
NoConn ~ 6100 8500
Wire Wire Line
	3550 4450 3850 4450
Wire Wire Line
	3550 3950 3850 3950
Wire Wire Line
	2950 7800 3150 7800
Wire Wire Line
	2950 8300 3150 8300
Wire Wire Line
	2950 9050 2950 9150
Wire Wire Line
	2350 9050 2450 9050
Text Label 5400 8400 0    60   ~ 0
PA2
Text Label 3600 6200 0    60   ~ 0
PA0
Text Label 3600 6400 0    60   ~ 0
PB0
Text Label 1800 9050 0    60   ~ 0
PB6
$Comp
L Connector:Conn_01x06_Male J?
U 1 1 5DEDC44D
P 6250 6200
F 0 "J?" H 6358 6581 50  0000 C CNN
F 1 "Conn_01x06_Male" H 6358 6490 50  0000 C CNN
F 2 "" H 6250 6200 50  0001 C CNN
F 3 "~" H 6250 6200 50  0001 C CNN
	1    6250 6200
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5DEDD3FB
P 5750 5800
F 0 "#PWR?" H 5750 5650 50  0001 C CNN
F 1 "+5V" H 5765 5973 50  0000 C CNN
F 2 "" H 5750 5800 50  0001 C CNN
F 3 "" H 5750 5800 50  0001 C CNN
	1    5750 5800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6050 5900 5750 5900
Wire Wire Line
	5750 5900 5750 5800
Wire Wire Line
	6050 6400 5750 6400
Wire Wire Line
	5750 6400 5750 6500
$Comp
L power:GND #PWR?
U 1 1 5DEE8317
P 5750 6500
F 0 "#PWR?" H 5750 6250 50  0001 C CNN
F 1 "GND" H 5755 6327 50  0000 C CNN
F 2 "" H 5750 6500 50  0001 C CNN
F 3 "" H 5750 6500 50  0001 C CNN
	1    5750 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 6300 5750 6300
Wire Wire Line
	6050 6200 5750 6200
Wire Wire Line
	6050 6100 5750 6100
Wire Wire Line
	6050 6000 5750 6000
Text Label 5750 6300 2    60   ~ 0
PA0
Text Label 5750 6200 2    60   ~ 0
PB0
Text Label 5750 6000 2    60   ~ 0
PA2
Text Label 5750 6100 2    60   ~ 0
PB6
Text Notes 2100 5300 0    60   ~ 0
2 channel analog frontend
Text Notes 1800 3350 0    60   ~ 0
Power supply 7-28V input 5V output
Text Notes 5700 7450 0    60   ~ 0
Debug header
Text Notes 2400 7350 0    60   ~ 0
PWM output
Text Notes 5300 5350 0    60   ~ 0
Connections to 1bitsy module
Wire Notes Line
	4500 3000 1000 3000
Wire Notes Line
	1000 3000 1000 9500
Wire Notes Line
	4500 3000 4500 9500
Wire Notes Line
	1000 5000 7500 5000
Wire Notes Line
	1000 7000 7500 7000
Wire Notes Line
	1000 9500 7500 9500
Wire Notes Line
	7500 5000 7500 9500
Text Notes 2200 1950 0    79   ~ 0
NOTE\nthis schematic only shows what approximately was done on the\nhand-soldered prototype board.\n
$Comp
L Device:C C?
U 1 1 5DED57E8
P 2650 4200
F 0 "C?" H 2765 4246 50  0000 L CNN
F 1 "10u 50V" H 2765 4155 50  0000 L CNN
F 2 "" H 2688 4050 50  0001 C CNN
F 3 "~" H 2650 4200 50  0001 C CNN
	1    2650 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C?
U 1 1 5DED9043
P 3250 4200
F 0 "C?" H 3368 4246 50  0000 L CNN
F 1 "47u 16V" H 3368 4155 50  0000 L CNN
F 2 "" H 3288 4050 50  0001 C CNN
F 3 "~" H 3250 4200 50  0001 C CNN
	1    3250 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C?
U 1 1 5DEDB68D
P 3850 4200
F 0 "C?" H 3968 4246 50  0000 L CNN
F 1 "47u 16V" H 3968 4155 50  0000 L CNN
F 2 "" H 3888 4050 50  0001 C CNN
F 3 "~" H 3850 4200 50  0001 C CNN
	1    3850 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5DEE2A14
P 2600 9050
F 0 "R?" V 2400 9050 50  0000 C CNN
F 1 "470" V 2500 9050 50  0000 C CNN
F 2 "" V 2530 9050 50  0001 C CNN
F 3 "~" H 2600 9050 50  0001 C CNN
	1    2600 9050
	0    -1   -1   0   
$EndComp
$Comp
L Device:D D?
U 1 1 5DEE53A4
P 2750 8050
F 0 "D?" V 2704 8129 50  0000 L CNN
F 1 "1N4148" V 2795 8129 50  0000 L CNN
F 2 "" H 2750 8050 50  0001 C CNN
F 3 "~" H 2750 8050 50  0001 C CNN
	1    2750 8050
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 5DED5801
P 1400 3900
F 0 "J?" H 1508 4081 50  0000 C CNN
F 1 "7-28V power input" H 1508 3990 50  0000 C CNN
F 2 "" H 1400 3900 50  0001 C CNN
F 3 "~" H 1400 3900 50  0001 C CNN
	1    1400 3900
	1    0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Female J?
U 1 1 5DED96F8
P 1400 4600
F 0 "J?" H 1292 4275 50  0000 C CNN
F 1 "TPSM84205" H 1292 4366 50  0000 C CNN
F 2 "" H 1400 4600 50  0001 C CNN
F 3 "~" H 1400 4600 50  0001 C CNN
	1    1400 4600
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x05_Male J?
U 1 1 5DEDEF22
P 1750 6400
F 0 "J?" H 1723 6332 50  0000 R CNN
F 1 "RSM2650" H 1723 6423 50  0000 R CNN
F 2 "" H 1750 6400 50  0001 C CNN
F 3 "~" H 1750 6400 50  0001 C CNN
	1    1750 6400
	1    0    0    1   
$EndComp
$Comp
L Connector:Conn_01x02_Male J?
U 1 1 5DEE5960
P 3650 8000
F 0 "J?" H 3622 7974 50  0000 R CNN
F 1 "Analog display" H 3622 7883 50  0000 R CNN
F 2 "" H 3650 8000 50  0001 C CNN
F 3 "~" H 3650 8000 50  0001 C CNN
	1    3650 8000
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Male J?
U 1 1 5DEEB3D4
P 6300 8400
F 0 "J?" H 6272 8424 50  0000 R CNN
F 1 "USART" H 6272 8333 50  0000 R CNN
F 2 "" H 6300 8400 50  0001 C CNN
F 3 "~" H 6300 8400 50  0001 C CNN
	1    6300 8400
	-1   0    0    -1  
$EndComp
$Comp
L Device:D D?
U 1 1 5DF0132B
P 1950 3800
F 0 "D?" H 1950 4016 50  0000 C CNN
F 1 "1N4001" H 1950 3925 50  0000 C CNN
F 2 "" H 1950 3800 50  0001 C CNN
F 3 "~" H 1950 3800 50  0001 C CNN
	1    1950 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 9050 2350 8750
Wire Wire Line
	2350 8750 2650 8750
$Comp
L pspice:MNMOS M?
U 1 1 5DED53BF
P 2850 8750
F 0 "M?" H 3138 8796 50  0000 L CNN
F 1 "N-Channel MOSFET LR3636" H 3138 8705 50  0000 L CNN
F 2 "" H 2825 8750 50  0001 C CNN
F 3 "~" H 2825 8750 50  0001 C CNN
	1    2850 8750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 8950 3050 9050
Wire Wire Line
	3050 9050 2950 9050
$EndSCHEMATC
