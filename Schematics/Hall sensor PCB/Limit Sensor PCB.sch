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
L CONN_01X03 P?
U 1 1 56FBB216
P 3775 2050
F 0 "P?" H 3775 2250 50  0000 C CNN
F 1 "CONN_01X03" V 3875 2050 50  0001 C CNN
F 2 "" H 3775 2050 50  0000 C CNN
F 3 "" H 3775 2050 50  0000 C CNN
	1    3775 2050
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 56FBB246
P 3775 2675
F 0 "P?" H 3775 2875 50  0000 C CNN
F 1 "CONN_01X03" V 3875 2675 50  0001 C CNN
F 2 "" H 3775 2675 50  0000 C CNN
F 3 "" H 3775 2675 50  0000 C CNN
	1    3775 2675
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 56FBB28C
P 3775 3350
F 0 "P?" H 3775 3550 50  0000 C CNN
F 1 "CONN_01X03" V 3875 3350 50  0001 C CNN
F 2 "" H 3775 3350 50  0000 C CNN
F 3 "" H 3775 3350 50  0000 C CNN
	1    3775 3350
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 56FBB2D5
P 3775 4025
F 0 "P?" H 3775 4225 50  0000 C CNN
F 1 "CONN_01X03" V 3875 4025 50  0001 C CNN
F 2 "" H 3775 4025 50  0000 C CNN
F 3 "" H 3775 4025 50  0000 C CNN
	1    3775 4025
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X03 P?
U 1 1 56FBB333
P 3775 4725
F 0 "P?" H 3775 4925 50  0000 C CNN
F 1 "CONN_01X03" V 3875 4725 50  0001 C CNN
F 2 "" H 3775 4725 50  0000 C CNN
F 3 "" H 3775 4725 50  0000 C CNN
	1    3775 4725
	-1   0    0    -1  
$EndComp
$Comp
L CONN_01X08 P?
U 1 1 56FBB3ED
P 7825 2300
F 0 "P?" H 7825 2750 50  0000 C CNN
F 1 "CONN_01X08" V 7925 2300 50  0000 C CNN
F 2 "" H 7825 2300 50  0000 C CNN
F 3 "" H 7825 2300 50  0000 C CNN
	1    7825 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3975 1950 7625 1950
$Comp
L +5V #PWR?
U 1 1 56FBB5A7
P 7125 1600
F 0 "#PWR?" H 7125 1450 50  0001 C CNN
F 1 "+5V" H 7125 1740 50  0000 C CNN
F 2 "" H 7125 1600 50  0000 C CNN
F 3 "" H 7125 1600 50  0000 C CNN
	1    7125 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7125 1600 7125 1950
Connection ~ 7125 1950
$Comp
L Earth #PWR?
U 1 1 56FBB5D1
P 7125 2825
F 0 "#PWR?" H 7125 2575 50  0001 C CNN
F 1 "Earth" H 7125 2675 50  0001 C CNN
F 2 "" H 7125 2825 50  0000 C CNN
F 3 "" H 7125 2825 50  0000 C CNN
	1    7125 2825
	1    0    0    -1  
$EndComp
Wire Wire Line
	6625 2650 7625 2650
Wire Wire Line
	7125 2550 7125 2825
Wire Wire Line
	3975 3925 4225 3925
Wire Wire Line
	4225 1950 4225 4625
Connection ~ 4225 1950
Wire Wire Line
	3975 2575 4225 2575
Connection ~ 4225 2575
Wire Wire Line
	3975 3250 4225 3250
Connection ~ 4225 3250
Wire Wire Line
	6625 2650 6625 5425
Connection ~ 7125 2650
Wire Wire Line
	3975 3450 4400 3450
Wire Wire Line
	4400 2150 4400 5425
Connection ~ 4400 4125
Wire Wire Line
	3975 2775 4400 2775
Connection ~ 4400 3450
Wire Wire Line
	3975 2150 4400 2150
Connection ~ 4400 2775
Connection ~ 4400 4825
Wire Wire Line
	3975 4125 4400 4125
Wire Wire Line
	4225 4625 3975 4625
Connection ~ 4225 3925
$Comp
L R 1K
U 1 1 56FBB7F3
P 4825 2050
F 0 "1K" V 4905 2050 50  0000 C CNN
F 1 "R" V 4825 2050 50  0000 C CNN
F 2 "" V 4755 2050 50  0000 C CNN
F 3 "" H 4825 2050 50  0000 C CNN
	1    4825 2050
	0    1    1    0   
$EndComp
$Comp
L R 2K2
U 1 1 56FBB86D
P 5125 2275
F 0 "2K2" V 5205 2275 50  0000 C CNN
F 1 "R" V 5125 2275 50  0000 C CNN
F 2 "" V 5055 2275 50  0000 C CNN
F 3 "" H 5125 2275 50  0000 C CNN
	1    5125 2275
	1    0    0    -1  
$EndComp
$Comp
L R 1K
U 1 1 56FBB985
P 4825 2675
F 0 "1K" V 4905 2675 50  0000 C CNN
F 1 "R" V 4825 2675 50  0000 C CNN
F 2 "" V 4755 2675 50  0000 C CNN
F 3 "" H 4825 2675 50  0000 C CNN
	1    4825 2675
	0    1    1    0   
$EndComp
$Comp
L R 1K
U 1 1 56FBB9F6
P 4825 3350
F 0 "1K" V 4905 3350 50  0000 C CNN
F 1 "R" V 4825 3350 50  0000 C CNN
F 2 "" V 4755 3350 50  0000 C CNN
F 3 "" H 4825 3350 50  0000 C CNN
	1    4825 3350
	0    1    1    0   
$EndComp
$Comp
L R 1K
U 1 1 56FBBA6A
P 4825 4025
F 0 "1K" V 4905 4025 50  0000 C CNN
F 1 "R" V 4825 4025 50  0000 C CNN
F 2 "" V 4755 4025 50  0000 C CNN
F 3 "" H 4825 4025 50  0000 C CNN
	1    4825 4025
	0    1    1    0   
$EndComp
Wire Wire Line
	3975 2050 4675 2050
Wire Wire Line
	4675 2675 3975 2675
Wire Wire Line
	3975 3350 4675 3350
Wire Wire Line
	4675 4025 3975 4025
Wire Wire Line
	4675 4725 3975 4725
$Comp
L Earth #PWR?
U 1 1 56FBBDF0
P 5125 2500
F 0 "#PWR?" H 5125 2250 50  0001 C CNN
F 1 "Earth" H 5125 2350 50  0001 C CNN
F 2 "" H 5125 2500 50  0000 C CNN
F 3 "" H 5125 2500 50  0000 C CNN
	1    5125 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 2500 5125 2425
Wire Wire Line
	5125 2125 5125 2050
Wire Wire Line
	4975 2050 7625 2050
$Comp
L R 2K?
U 1 1 56FBBEBA
P 5125 2900
F 0 "2K?" V 5205 2900 50  0000 C CNN
F 1 "R" V 5125 2900 50  0000 C CNN
F 2 "" V 5055 2900 50  0000 C CNN
F 3 "" H 5125 2900 50  0000 C CNN
	1    5125 2900
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 56FBBEC0
P 5125 3125
F 0 "#PWR?" H 5125 2875 50  0001 C CNN
F 1 "Earth" H 5125 2975 50  0001 C CNN
F 2 "" H 5125 3125 50  0000 C CNN
F 3 "" H 5125 3125 50  0000 C CNN
	1    5125 3125
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 3125 5125 3050
Wire Wire Line
	5125 2750 5125 2675
Wire Wire Line
	4975 2675 5475 2675
$Comp
L R 2K?
U 1 1 56FBC031
P 5125 3575
F 0 "2K?" V 5205 3575 50  0000 C CNN
F 1 "R" V 5125 3575 50  0000 C CNN
F 2 "" V 5055 3575 50  0000 C CNN
F 3 "" H 5125 3575 50  0000 C CNN
	1    5125 3575
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 56FBC037
P 5125 3850
F 0 "#PWR?" H 5125 3600 50  0001 C CNN
F 1 "Earth" H 5125 3700 50  0001 C CNN
F 2 "" H 5125 3850 50  0000 C CNN
F 3 "" H 5125 3850 50  0000 C CNN
	1    5125 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 3800 5125 3725
Wire Wire Line
	5125 3425 5125 3350
Wire Wire Line
	4975 3350 5600 3350
$Comp
L R 2K?
U 1 1 56FBC040
P 5125 4250
F 0 "2K?" V 5205 4250 50  0000 C CNN
F 1 "R" V 5125 4250 50  0000 C CNN
F 2 "" V 5055 4250 50  0000 C CNN
F 3 "" H 5125 4250 50  0000 C CNN
	1    5125 4250
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 56FBC046
P 5125 4475
F 0 "#PWR?" H 5125 4225 50  0001 C CNN
F 1 "Earth" H 5125 4325 50  0001 C CNN
F 2 "" H 5125 4475 50  0000 C CNN
F 3 "" H 5125 4475 50  0000 C CNN
	1    5125 4475
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 4475 5125 4400
Wire Wire Line
	5125 4100 5125 4025
Wire Wire Line
	4975 4025 5725 4025
$Comp
L R 2K?
U 1 1 56FBC115
P 5125 4950
F 0 "2K?" V 5205 4950 50  0000 C CNN
F 1 "R" V 5125 4950 50  0000 C CNN
F 2 "" V 5055 4950 50  0000 C CNN
F 3 "" H 5125 4950 50  0000 C CNN
	1    5125 4950
	1    0    0    -1  
$EndComp
$Comp
L Earth #PWR?
U 1 1 56FBC11B
P 5125 5175
F 0 "#PWR?" H 5125 4925 50  0001 C CNN
F 1 "Earth" H 5125 5025 50  0001 C CNN
F 2 "" H 5125 5175 50  0000 C CNN
F 3 "" H 5125 5175 50  0000 C CNN
	1    5125 5175
	1    0    0    -1  
$EndComp
Wire Wire Line
	5125 5175 5125 5100
Wire Wire Line
	5125 4800 5125 4725
Wire Wire Line
	4975 4725 5875 4725
$Comp
L R 1K?
U 1 1 56FBC164
P 4825 4725
F 0 "1K?" V 4905 4725 50  0000 C CNN
F 1 "R" V 4825 4725 50  0000 C CNN
F 2 "" V 4755 4725 50  0000 C CNN
F 3 "" H 4825 4725 50  0000 C CNN
	1    4825 4725
	0    1    1    0   
$EndComp
Wire Wire Line
	4400 5425 6625 5425
Wire Wire Line
	3975 4825 4400 4825
Connection ~ 5125 2050
Wire Wire Line
	7625 2150 5475 2150
Wire Wire Line
	5475 2150 5475 2675
Connection ~ 5125 2675
Wire Wire Line
	7625 2250 5600 2250
Wire Wire Line
	5600 2250 5600 3350
Connection ~ 5125 3350
Wire Wire Line
	7625 2350 5725 2350
Wire Wire Line
	5725 2350 5725 4025
Connection ~ 5125 4025
Wire Wire Line
	7625 2450 5875 2450
Wire Wire Line
	5875 2450 5875 4725
Connection ~ 5125 4725
Wire Wire Line
	7625 2550 7125 2550
$EndSCHEMATC