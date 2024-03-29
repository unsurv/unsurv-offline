EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "unsurv offline"
Date "2021-05-14"
Rev "0.2"
Comp "unsurv"
Comment1 "on 4.5 cm x 3 cm PCB"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Battery_Management:MCP73831-2-OT U2
U 1 1 5E2A0C54
P 1800 1850
F 0 "U2" H 1800 2331 50  0000 C CNN
F 1 "MCP73831-2-OT" H 1800 2240 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 1850 1600 50  0001 L CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001984g.pdf" H 1650 1800 50  0001 C CNN
	1    1800 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5E30F837
P 850 4200
F 0 "#PWR025" H 850 3950 50  0001 C CNN
F 1 "GND" H 855 4027 50  0000 C CNN
F 2 "" H 850 4200 50  0001 C CNN
F 3 "" H 850 4200 50  0001 C CNN
	1    850  4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5E320DA1
P 2350 1150
F 0 "C1" V 2098 1150 50  0000 C CNN
F 1 "4.7yF" V 2189 1150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2388 1000 50  0001 C CNN
F 3 "~" H 2350 1150 50  0001 C CNN
	1    2350 1150
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5E32223A
P 700 1650
F 0 "C2" H 585 1604 50  0000 R CNN
F 1 "4.7yF" H 585 1695 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 738 1500 50  0001 C CNN
F 3 "~" H 700 1650 50  0001 C CNN
	1    700  1650
	-1   0    0    1   
$EndComp
$Comp
L Device:R R2
U 1 1 5E328050
P 1100 2100
F 0 "R2" H 1170 2146 50  0000 L CNN
F 1 "10k" H 1170 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1030 2100 50  0001 C CNN
F 3 "~" H 1100 2100 50  0001 C CNN
	1    1100 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5E32973F
P 2400 2200
F 0 "R3" H 2470 2246 50  0000 L CNN
F 1 "1k" H 2470 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2330 2200 50  0001 C CNN
F 3 "~" H 2400 2200 50  0001 C CNN
	1    2400 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5E32B05A
P 2400 2650
F 0 "D3" V 2347 2728 50  0000 L CNN
F 1 "LED" V 2438 2728 50  0000 L CNN
F 2 "digikey-footprints:LED_0603" H 2400 2650 50  0001 C CNN
F 3 "~" H 2400 2650 50  0001 C CNN
	1    2400 2650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5E32E8C7
P 1800 2350
F 0 "#PWR015" H 1800 2100 50  0001 C CNN
F 1 "GND" H 1805 2177 50  0000 C CNN
F 2 "" H 1800 2350 50  0001 C CNN
F 3 "" H 1800 2350 50  0001 C CNN
	1    1800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2150 1800 2350
Wire Wire Line
	2400 2500 2400 2350
Wire Wire Line
	2400 1950 2200 1950
Wire Wire Line
	2400 2050 2400 1950
Wire Wire Line
	1100 1950 1400 1950
$Comp
L power:GND #PWR014
U 1 1 5E331B06
P 1100 2350
F 0 "#PWR014" H 1100 2100 50  0001 C CNN
F 1 "GND" H 1105 2177 50  0000 C CNN
F 2 "" H 1100 2350 50  0001 C CNN
F 3 "" H 1100 2350 50  0001 C CNN
	1    1100 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 2350 1100 2250
Wire Wire Line
	600  1500 700  1500
Wire Wire Line
	1800 1550 1800 1500
$Comp
L power:GND #PWR010
U 1 1 5E3363DB
P 700 1950
F 0 "#PWR010" H 700 1700 50  0001 C CNN
F 1 "GND" H 705 1777 50  0000 C CNN
F 2 "" H 700 1950 50  0001 C CNN
F 3 "" H 700 1950 50  0001 C CNN
	1    700  1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	700  1950 700  1800
Wire Wire Line
	2200 1750 2200 1600
$Comp
L power:GND #PWR05
U 1 1 5E338243
P 2650 1150
F 0 "#PWR05" H 2650 900 50  0001 C CNN
F 1 "GND" H 2655 977 50  0000 C CNN
F 2 "" H 2650 1150 50  0001 C CNN
F 3 "" H 2650 1150 50  0001 C CNN
	1    2650 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1150 2500 1150
$Comp
L power:GND #PWR01
U 1 1 5E33995B
P 1250 950
F 0 "#PWR01" H 1250 700 50  0001 C CNN
F 1 "GND" H 1255 777 50  0000 C CNN
F 2 "" H 1250 950 50  0001 C CNN
F 3 "" H 1250 950 50  0001 C CNN
	1    1250 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR08
U 1 1 5E33AE8F
P 2650 1600
F 0 "#PWR08" H 2650 1450 50  0001 C CNN
F 1 "+BATT" H 2665 1773 50  0000 C CNN
F 2 "" H 2650 1600 50  0001 C CNN
F 3 "" H 2650 1600 50  0001 C CNN
	1    2650 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1600 2200 1600
Connection ~ 2200 1600
Wire Wire Line
	2200 1600 2200 1150
$Comp
L power:+5V #PWR018
U 1 1 5E34F77A
P 1350 3500
F 0 "#PWR018" H 1350 3350 50  0001 C CNN
F 1 "+5V" H 1365 3673 50  0000 C CNN
F 2 "" H 1350 3500 50  0001 C CNN
F 3 "" H 1350 3500 50  0001 C CNN
	1    1350 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 3500 1150 3500
$Comp
L power:+5V #PWR07
U 1 1 5E350CEC
P 600 1500
F 0 "#PWR07" H 600 1350 50  0001 C CNN
F 1 "+5V" H 615 1673 50  0000 C CNN
F 2 "" H 600 1500 50  0001 C CNN
F 3 "" H 600 1500 50  0001 C CNN
	1    600  1500
	1    0    0    -1  
$EndComp
Text GLabel 3550 7250 2    50   Input ~ 0
4
Text GLabel 3550 7150 2    50   Input ~ 0
23
Text GLabel 3550 6950 2    50   Input ~ 0
18
Text GLabel 3550 6750 2    50   Input ~ 0
19
$Comp
L power:+3.3V #PWR046
U 1 1 5E3B9642
P 3850 7050
F 0 "#PWR046" H 3850 6900 50  0001 C CNN
F 1 "+3.3V" H 3865 7223 50  0000 C CNN
F 2 "" H 3850 7050 50  0001 C CNN
F 3 "" H 3850 7050 50  0001 C CNN
	1    3850 7050
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 7050 2350 7050
$Comp
L power:GND #PWR044
U 1 1 5E3D06FF
P 3850 6850
F 0 "#PWR044" H 3850 6600 50  0001 C CNN
F 1 "GND" H 3855 6677 50  0000 C CNN
F 2 "" H 3850 6850 50  0001 C CNN
F 3 "" H 3850 6850 50  0001 C CNN
	1    3850 6850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 6850 2350 6850
$Comp
L power:GND #PWR036
U 1 1 5E3D4C21
P 2450 5850
F 0 "#PWR036" H 2450 5600 50  0001 C CNN
F 1 "GND" H 2455 5677 50  0000 C CNN
F 2 "" H 2450 5850 50  0001 C CNN
F 3 "" H 2450 5850 50  0001 C CNN
	1    2450 5850
	-1   0    0    1   
$EndComp
$Comp
L Device:R R4
U 1 1 5E41108F
P 4100 2200
F 0 "R4" H 4170 2246 50  0000 L CNN
F 1 "4.7k" H 4170 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4030 2200 50  0001 C CNN
F 3 "~" H 4100 2200 50  0001 C CNN
	1    4100 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E412365
P 4400 2200
F 0 "R5" H 4470 2246 50  0000 L CNN
F 1 "4.7k" H 4470 2155 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4330 2200 50  0001 C CNN
F 3 "~" H 4400 2200 50  0001 C CNN
	1    4400 2200
	1    0    0    -1  
$EndComp
Text GLabel 4100 2350 3    50   Input ~ 0
SDA
Text GLabel 4400 2350 3    50   Input ~ 0
SCL
Wire Wire Line
	2350 6550 2450 6550
Wire Wire Line
	2450 5850 2450 6250
Wire Wire Line
	2350 6450 2450 6450
Connection ~ 2450 6450
Wire Wire Line
	2450 6450 2450 6550
Wire Wire Line
	2350 6350 2450 6350
Connection ~ 2450 6350
Wire Wire Line
	2450 6350 2450 6450
Wire Wire Line
	2350 6250 2450 6250
Connection ~ 2450 6250
Wire Wire Line
	2450 6250 2450 6350
Text GLabel 1250 3800 2    50   Input ~ 0
D-
Text GLabel 1250 3700 2    50   Input ~ 0
D+
Wire Wire Line
	1250 3800 1150 3800
Wire Wire Line
	1250 3700 1150 3700
$Comp
L unsurv_offline_pcb_symbols:UMH3N Q1
U 1 1 5E8E5B40
P 9250 4400
F 0 "Q1" H 9275 4725 50  0000 C CNN
F 1 "UMH3N" H 9275 4634 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:UMH3N" H 9300 4150 50  0001 C CNN
F 3 "" H 9300 4150 50  0001 C CNN
	1    9250 4400
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR024
U 1 1 5E93703A
P 10450 4050
F 0 "#PWR024" H 10450 3900 50  0001 C CNN
F 1 "+3.3V" H 10465 4223 50  0000 C CNN
F 2 "" H 10450 4050 50  0001 C CNN
F 3 "" H 10450 4050 50  0001 C CNN
	1    10450 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 5000 10350 4900
Wire Wire Line
	10350 4900 10450 4900
Wire Wire Line
	10450 4900 10450 5000
$Comp
L power:GND #PWR040
U 1 1 5E9530ED
P 10700 6200
F 0 "#PWR040" H 10700 5950 50  0001 C CNN
F 1 "GND" H 10705 6027 50  0000 C CNN
F 2 "" H 10700 6200 50  0001 C CNN
F 3 "" H 10700 6200 50  0001 C CNN
	1    10700 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	10700 6200 10450 6200
Text GLabel 11000 6000 2    50   Input ~ 0
RTS
Text GLabel 11000 5900 2    50   Input ~ 0
DTR
Text GLabel 11000 5200 2    50   Input ~ 0
RX0
Text GLabel 11000 5300 2    50   Input ~ 0
TX0
Text GLabel 9900 5500 0    50   Input ~ 0
D+
Text GLabel 9900 5600 0    50   Input ~ 0
D-
Wire Wire Line
	11000 5200 10850 5200
Wire Wire Line
	11000 5300 10850 5300
Wire Wire Line
	11000 5900 10850 5900
Wire Wire Line
	11000 6000 10850 6000
Wire Wire Line
	10050 5500 9900 5500
Wire Wire Line
	10050 5600 9900 5600
Text GLabel 8900 4300 0    50   Input ~ 0
RTS
Text GLabel 8900 4400 0    50   Input ~ 0
DTR
Text GLabel 8900 4500 0    50   Input ~ 0
0
Text GLabel 9650 4500 2    50   Input ~ 0
DTR
Text GLabel 9650 4400 2    50   Input ~ 0
RTS
Text GLabel 9650 4300 2    50   Input ~ 0
RESET
Wire Wire Line
	9650 4300 9500 4300
Wire Wire Line
	9650 4400 9500 4400
Wire Wire Line
	9500 4500 9650 4500
Wire Wire Line
	9050 4300 8900 4300
Wire Wire Line
	9050 4400 8900 4400
Wire Wire Line
	9050 4500 8900 4500
Connection ~ 2200 1150
Wire Wire Line
	4100 2050 4250 2050
Wire Wire Line
	4250 1950 4250 2050
Connection ~ 4250 2050
Wire Wire Line
	4250 2050 4400 2050
$Comp
L power:+BATT #PWR026
U 1 1 5EE84493
P 1450 4500
F 0 "#PWR026" H 1450 4350 50  0001 C CNN
F 1 "+BATT" H 1465 4673 50  0000 C CNN
F 2 "" H 1450 4500 50  0001 C CNN
F 3 "" H 1450 4500 50  0001 C CNN
	1    1450 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR028
U 1 1 5E35363C
P 600 4800
F 0 "#PWR028" H 600 4650 50  0001 C CNN
F 1 "+5V" H 615 4973 50  0000 C CNN
F 2 "" H 600 4800 50  0001 C CNN
F 3 "" H 600 4800 50  0001 C CNN
	1    600  4800
	1    0    0    -1  
$EndComp
$Comp
L Interface_USB:CH340G U5
U 1 1 5E8D4BE0
P 10450 5600
F 0 "U5" H 10450 4911 50  0000 C CNN
F 1 "CH340G" H 10450 4820 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 10500 5050 50  0001 L CNN
F 3 "http://www.datasheet5.com/pdf-local-2195953" H 10100 6400 50  0001 C CNN
	1    10450 5600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR012
U 1 1 5F0E4E41
P 10300 1950
F 0 "#PWR012" H 10300 1800 50  0001 C CNN
F 1 "+3.3V" H 10315 2123 50  0000 C CNN
F 2 "" H 10300 1950 50  0001 C CNN
F 3 "" H 10300 1950 50  0001 C CNN
	1    10300 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 2100 9450 2000
Wire Wire Line
	9450 2000 9550 2000
Wire Wire Line
	9650 2000 9650 2100
Text GLabel 8650 2400 0    50   Input ~ 0
SDA
Text GLabel 8650 2500 0    50   Input ~ 0
SCL
Text GLabel 8650 2600 0    50   Input ~ 0
5
Wire Wire Line
	9550 2100 9550 2000
Connection ~ 9550 2000
Wire Wire Line
	9550 2000 9650 2000
$Comp
L Device:Q_PMOS_GSD Q2
U 1 1 5EFD8503
P 1350 4850
F 0 "Q2" H 1555 4896 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 1555 4805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1550 4950 50  0001 C CNN
F 3 "~" H 1350 4850 50  0001 C CNN
	1    1350 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5F0139A0
P 8800 1000
F 0 "D1" H 8793 1216 50  0000 C CNN
F 1 "LED" H 8793 1125 50  0000 C CNN
F 2 "digikey-footprints:LED_0603" H 8800 1000 50  0001 C CNN
F 3 "~" H 8800 1000 50  0001 C CNN
	1    8800 1000
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 5F025A1B
P 8400 1000
F 0 "R1" H 8470 1046 50  0000 L CNN
F 1 "1k" H 8470 955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8330 1000 50  0001 C CNN
F 3 "~" H 8400 1000 50  0001 C CNN
	1    8400 1000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9050 1000 8950 1000
Wire Wire Line
	8650 1000 8550 1000
Text GLabel 8100 1000 0    50   Input ~ 0
27
Wire Wire Line
	8250 1000 8100 1000
$Comp
L Device:R R9
U 1 1 5F0C74DA
P 2400 3600
F 0 "R9" H 2470 3646 50  0000 L CNN
F 1 "10k" H 2470 3555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2330 3600 50  0001 C CNN
F 3 "~" H 2400 3600 50  0001 C CNN
	1    2400 3600
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR019
U 1 1 5F13AEBA
P 2050 3600
F 0 "#PWR019" H 2050 3450 50  0001 C CNN
F 1 "+3.3V" H 2065 3773 50  0000 C CNN
F 2 "" H 2050 3600 50  0001 C CNN
F 3 "" H 2050 3600 50  0001 C CNN
	1    2050 3600
	1    0    0    -1  
$EndComp
Text GLabel 3250 3450 1    50   Input ~ 0
RESET
$Comp
L power:GND #PWR020
U 1 1 5F1AED51
P 4300 3700
F 0 "#PWR020" H 4300 3450 50  0001 C CNN
F 1 "GND" H 4305 3527 50  0000 C CNN
F 2 "" H 4300 3700 50  0001 C CNN
F 3 "" H 4300 3700 50  0001 C CNN
	1    4300 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 3700 4200 3700
$Comp
L Device:C C4
U 1 1 5F1D1B83
P 2750 3750
F 0 "C4" H 2865 3796 50  0000 L CNN
F 1 "100nF" H 2865 3705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2788 3600 50  0001 C CNN
F 3 "~" H 2750 3750 50  0001 C CNN
	1    2750 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5F1D1B89
P 2750 3950
F 0 "#PWR022" H 2750 3700 50  0001 C CNN
F 1 "GND" H 2755 3777 50  0000 C CNN
F 2 "" H 2750 3950 50  0001 C CNN
F 3 "" H 2750 3950 50  0001 C CNN
	1    2750 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 3900 2750 3950
Wire Wire Line
	2050 3600 2250 3600
Wire Wire Line
	2550 3600 2750 3600
Connection ~ 2750 3600
Wire Wire Line
	2750 3600 3250 3600
Wire Wire Line
	3250 3450 3250 3600
Connection ~ 3250 3600
Connection ~ 2650 1600
Wire Wire Line
	3250 3600 3600 3600
$Comp
L Device:R R7
U 1 1 5F0CC6B1
P 2900 2450
F 0 "R7" H 2970 2496 50  0000 L CNN
F 1 "100k" H 2970 2405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2830 2450 50  0001 C CNN
F 3 "~" H 2900 2450 50  0001 C CNN
	1    2900 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R8
U 1 1 5F0D6240
P 2900 2950
F 0 "R8" H 2970 2996 50  0000 L CNN
F 1 "100k" H 2970 2905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2830 2950 50  0001 C CNN
F 3 "~" H 2900 2950 50  0001 C CNN
	1    2900 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 5F0DFB7C
P 2900 3150
F 0 "#PWR017" H 2900 2900 50  0001 C CNN
F 1 "GND" H 2905 2977 50  0000 C CNN
F 2 "" H 2900 3150 50  0001 C CNN
F 3 "" H 2900 3150 50  0001 C CNN
	1    2900 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 3100 2900 3150
Text GLabel 2900 2700 2    50   Input ~ 0
35
$Comp
L Device:LED D2
U 1 1 5F139FB7
P 10900 1950
F 0 "D2" V 10847 2028 50  0000 L CNN
F 1 "LED" V 10938 2028 50  0000 L CNN
F 2 "digikey-footprints:LED_0603" H 10900 1950 50  0001 C CNN
F 3 "~" H 10900 1950 50  0001 C CNN
	1    10900 1950
	0    1    1    0   
$EndComp
Wire Wire Line
	10900 1800 10900 1650
$Comp
L Device:R R6
U 1 1 5F1450A8
P 10900 2300
F 0 "R6" H 10970 2346 50  0000 L CNN
F 1 "1k" H 10970 2255 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 10830 2300 50  0001 C CNN
F 3 "~" H 10900 2300 50  0001 C CNN
	1    10900 2300
	-1   0    0    1   
$EndComp
Wire Wire Line
	10900 2150 10900 2100
$Comp
L power:GND #PWR09
U 1 1 5F1640D6
P 10900 1650
F 0 "#PWR09" H 10900 1400 50  0001 C CNN
F 1 "GND" H 10905 1477 50  0000 C CNN
F 2 "" H 10900 1650 50  0001 C CNN
F 3 "" H 10900 1650 50  0001 C CNN
	1    10900 1650
	-1   0    0    1   
$EndComp
Wire Wire Line
	10450 4050 10450 4300
Connection ~ 700  1500
Wire Wire Line
	700  1500 1100 1500
Wire Wire Line
	900  1800 1100 1800
Wire Wire Line
	1100 1800 1100 1500
Connection ~ 1100 1500
Wire Wire Line
	1100 1500 1800 1500
Wire Wire Line
	2400 2800 900  2800
Wire Wire Line
	900  2800 900  1800
$Comp
L Device:D_Schottky D4
U 1 1 5EF00E40
P 1100 5250
F 0 "D4" H 1100 5034 50  0000 C CNN
F 1 "D_Schottky" H 1100 5125 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323" H 1100 5250 50  0001 C CNN
F 3 "~" H 1100 5250 50  0001 C CNN
	1    1100 5250
	-1   0    0    1   
$EndComp
Wire Wire Line
	750  5150 750  5300
$Comp
L power:GND #PWR032
U 1 1 5F03AD41
P 750 5300
F 0 "#PWR032" H 750 5050 50  0001 C CNN
F 1 "GND" H 755 5127 50  0000 C CNN
F 2 "" H 750 5300 50  0001 C CNN
F 3 "" H 750 5300 50  0001 C CNN
	1    750  5300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 5F02E84F
P 750 5000
F 0 "R11" H 820 5046 50  0000 L CNN
F 1 "100k" H 820 4955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 680 5000 50  0001 C CNN
F 3 "~" H 750 5000 50  0001 C CNN
	1    750  5000
	-1   0    0    1   
$EndComp
Wire Wire Line
	850  4850 850  5250
Wire Wire Line
	850  5250 950  5250
Wire Wire Line
	1250 5250 1450 5250
Wire Wire Line
	1450 5250 1450 5100
$Comp
L Switch:SW_DIP_x01 S2
U 1 1 5F67CDC1
P 10450 4600
F 0 "S2" H 10450 4867 50  0000 C CNN
F 1 "SW_DIP_x01" H 10450 4776 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:CHS-01TA" H 10450 4600 50  0001 C CNN
F 3 "~" H 10450 4600 50  0001 C CNN
	1    10450 4600
	0    -1   -1   0   
$EndComp
$Comp
L Connector:USB_B_Micro USB1
U 1 1 5F6E1661
P 850 3700
F 0 "USB1" H 907 4167 50  0000 C CNN
F 1 "USB_B_Micro" H 907 4076 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Amphenol_10103594-0001LF_Horizontal" H 1000 3650 50  0001 C CNN
F 3 "~" H 1000 3650 50  0001 C CNN
	1    850  3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  4200 850  4100
Wire Wire Line
	10350 2600 10600 2600
Wire Wire Line
	10600 2600 10600 1700
Wire Wire Line
	10600 1700 8250 1700
Wire Wire Line
	8250 1700 8250 2700
Wire Wire Line
	8250 2700 8650 2700
$Comp
L Device:C C3
U 1 1 5F9619A4
P 10000 2000
F 0 "C3" H 10115 2046 50  0000 L CNN
F 1 "4.7yF" H 10115 1955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10038 1850 50  0001 C CNN
F 3 "~" H 10000 2000 50  0001 C CNN
	1    10000 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	10300 1950 10200 1950
Wire Wire Line
	10200 1950 10200 1850
Wire Wire Line
	10200 1850 10000 1850
Wire Wire Line
	10000 1850 9650 1850
Wire Wire Line
	9650 1850 9650 2000
Connection ~ 10000 1850
Connection ~ 9650 2000
Wire Wire Line
	10900 2700 10900 2450
Wire Wire Line
	10350 2700 10900 2700
$Comp
L Regulator_Linear:AP2112K-3.3 U4
U 1 1 5F4F8345
P 3550 4600
F 0 "U4" H 3550 4942 50  0000 C CNN
F 1 "AP2112K-3.3" H 3550 4851 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 3550 4925 50  0001 C CNN
F 3 "https://www.diodes.com/assets/Datasheets/AP2112.pdf" H 3550 4700 50  0001 C CNN
	1    3550 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	600  4800 600  4850
Connection ~ 850  4850
$Comp
L Device:R R10
U 1 1 5F57130E
P 2800 4600
F 0 "R10" H 2870 4646 50  0000 L CNN
F 1 "100k" H 2870 4555 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2730 4600 50  0001 C CNN
F 3 "~" H 2800 4600 50  0001 C CNN
	1    2800 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	3250 4600 2950 4600
Wire Wire Line
	2650 4600 2350 4600
Wire Wire Line
	2350 4600 2350 4500
Wire Wire Line
	1450 5100 1900 5100
Wire Wire Line
	2200 5100 2200 4500
Wire Wire Line
	2200 4500 2350 4500
Connection ~ 1450 5100
Wire Wire Line
	1450 5100 1450 5050
Connection ~ 2350 4500
$Comp
L power:GND #PWR029
U 1 1 5F59E1F9
P 3550 5000
F 0 "#PWR029" H 3550 4750 50  0001 C CNN
F 1 "GND" H 3555 4827 50  0000 C CNN
F 2 "" H 3550 5000 50  0001 C CNN
F 3 "" H 3550 5000 50  0001 C CNN
	1    3550 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4900 3550 5000
$Comp
L Device:C C8
U 1 1 5F5D6662
P 1900 5300
F 0 "C8" H 1785 5254 50  0000 R CNN
F 1 "1yF" H 1785 5345 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1938 5150 50  0001 C CNN
F 3 "~" H 1900 5300 50  0001 C CNN
	1    1900 5300
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR035
U 1 1 5F5D6668
P 1900 5600
F 0 "#PWR035" H 1900 5350 50  0001 C CNN
F 1 "GND" H 1905 5427 50  0000 C CNN
F 2 "" H 1900 5600 50  0001 C CNN
F 3 "" H 1900 5600 50  0001 C CNN
	1    1900 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1900 5600 1900 5450
Wire Wire Line
	1900 5150 1900 5100
Connection ~ 1900 5100
Wire Wire Line
	1900 5100 2200 5100
$Comp
L Device:C C6
U 1 1 5F5F467E
P 4000 4700
F 0 "C6" H 3885 4654 50  0000 R CNN
F 1 "1yF" H 3885 4745 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4038 4550 50  0001 C CNN
F 3 "~" H 4000 4700 50  0001 C CNN
	1    4000 4700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR030
U 1 1 5F5F4684
P 4000 5000
F 0 "#PWR030" H 4000 4750 50  0001 C CNN
F 1 "GND" H 4005 4827 50  0000 C CNN
F 2 "" H 4000 5000 50  0001 C CNN
F 3 "" H 4000 5000 50  0001 C CNN
	1    4000 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 5000 4000 4850
$Comp
L power:+3.3V #PWR027
U 1 1 5F609BC9
P 4650 4500
F 0 "#PWR027" H 4650 4350 50  0001 C CNN
F 1 "+3.3V" H 4665 4673 50  0000 C CNN
F 2 "" H 4650 4500 50  0001 C CNN
F 3 "" H 4650 4500 50  0001 C CNN
	1    4650 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5F61EE73
P 9050 1000
F 0 "#PWR04" H 9050 750 50  0001 C CNN
F 1 "GND" H 9055 827 50  0000 C CNN
F 2 "" H 9050 1000 50  0001 C CNN
F 3 "" H 9050 1000 50  0001 C CNN
	1    9050 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 4500 4000 4500
Wire Wire Line
	4000 4550 4000 4500
Connection ~ 4000 4500
$Comp
L power:GND #PWR013
U 1 1 5F988C7A
P 10400 2150
F 0 "#PWR013" H 10400 1900 50  0001 C CNN
F 1 "GND" H 10405 1977 50  0000 C CNN
F 2 "" H 10400 2150 50  0001 C CNN
F 3 "" H 10400 2150 50  0001 C CNN
	1    10400 2150
	1    0    0    -1  
$EndComp
$Comp
L dk_RF-Receivers:CAM-M8C-0-10 U3
U 1 1 5F75E10D
P 9450 2700
F 0 "U3" H 10394 2553 60  0000 L CNN
F 1 "CAM-M8C-0-10" H 10394 2447 60  0000 L CNN
F 2 "unsurv_offline_pcb_footprints:GPS_Module_CAM-M8C-0" H 9650 2900 60  0001 L CNN
F 3 "https://www.u-blox.com/sites/default/files/CAM-M8-FW3_DataSheet_%28UBX-15031574%29.pdf" H 9650 3000 60  0001 L CNN
F 4 "672-CAM-M8C-0-10CT-ND" H 9650 3100 60  0001 L CNN "Digi-Key_PN"
F 5 "CAM-M8C-0-10" H 9650 3200 60  0001 L CNN "MPN"
F 6 "RF/IF and RFID" H 9650 3300 60  0001 L CNN "Category"
F 7 "RF Receivers" H 9650 3400 60  0001 L CNN "Family"
F 8 "https://www.u-blox.com/sites/default/files/CAM-M8-FW3_DataSheet_%28UBX-15031574%29.pdf" H 9650 3500 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/u-blox-america-inc/CAM-M8C-0-10/672-CAM-M8C-0-10CT-ND/6150677" H 9650 3600 60  0001 L CNN "DK_Detail_Page"
F 10 "RF RCVR GNSS/GPS 1.575GHZ MODULE" H 9650 3700 60  0001 L CNN "Description"
F 11 "U-Blox America Inc." H 9650 3800 60  0001 L CNN "Manufacturer"
F 12 "Active" H 9650 3900 60  0001 L CNN "Status"
	1    9450 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 2150 10400 2150
$Comp
L Device:C C7
U 1 1 5F70EA42
P 4400 4700
F 0 "C7" H 4285 4654 50  0000 R CNN
F 1 "10yF" H 4285 4745 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4438 4550 50  0001 C CNN
F 3 "~" H 4400 4700 50  0001 C CNN
	1    4400 4700
	-1   0    0    1   
$EndComp
Wire Wire Line
	4400 4550 4400 4500
Wire Wire Line
	4400 4500 4650 4500
Wire Wire Line
	4000 4500 4400 4500
Connection ~ 4400 4500
Wire Wire Line
	4400 4850 4400 5000
Wire Wire Line
	4400 5000 4000 5000
Connection ~ 4000 5000
Wire Wire Line
	850  4850 1150 4850
Wire Wire Line
	2350 4500 3250 4500
Wire Wire Line
	1450 4500 1450 4650
Connection ~ 10450 4900
Text GLabel 6950 4200 0    50   Input ~ 0
ANT1
Text GLabel 6950 4500 0    50   Input ~ 0
ANT2
$Comp
L Device:C C5
U 1 1 5F860850
P 7350 4350
F 0 "C5" H 7465 4396 50  0000 L CNN
F 1 "10pF" H 7465 4305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7388 4200 50  0001 C CNN
F 3 "~" H 7350 4350 50  0001 C CNN
	1    7350 4350
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint ANT1
U 1 1 5F879317
P 7900 4150
F 0 "ANT1" H 7958 4268 50  0000 L CNN
F 1 "ANT1" H 7958 4177 50  0000 L CNN
F 2 "unsurv_offline_pcb_footprints:PogoPin_2_2mm_circular" H 8100 4150 50  0001 C CNN
F 3 "~" H 8100 4150 50  0001 C CNN
	1    7900 4150
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint ANT2
U 1 1 5F884C06
P 7900 4600
F 0 "ANT2" H 7958 4718 50  0000 L CNN
F 1 "ANT2" H 7958 4627 50  0000 L CNN
F 2 "unsurv_offline_pcb_footprints:PogoPin_2_2mm_circular" H 8100 4600 50  0001 C CNN
F 3 "~" H 8100 4600 50  0001 C CNN
	1    7900 4600
	-1   0    0    1   
$EndComp
Wire Wire Line
	7900 4150 7350 4150
Wire Wire Line
	7350 4150 7350 4200
Wire Wire Line
	7350 4500 7350 4550
Wire Wire Line
	7350 4550 7900 4550
Wire Wire Line
	6950 4500 7350 4500
Connection ~ 7350 4500
Wire Wire Line
	6950 4200 7350 4200
Connection ~ 7350 4200
Wire Wire Line
	7900 4600 7900 4550
Wire Wire Line
	2200 800  2200 1150
Wire Wire Line
	1600 800  1250 800 
Wire Wire Line
	1250 800  1250 950 
$Comp
L Connector:TestPoint BATT-1
U 1 1 5F807976
P 1600 800
F 0 "BATT-1" H 1658 918 50  0000 L CNN
F 1 "-" H 1658 827 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 1800 800 50  0001 C CNN
F 3 "~" H 1800 800 50  0001 C CNN
	1    1600 800 
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint BATT+1
U 1 1 5F808F0F
P 2200 800
F 0 "BATT+1" H 2258 918 50  0000 L CNN
F 1 "+" H 2258 827 50  0000 L CNN
F 2 "TestPoint:TestPoint_Pad_D1.0mm" H 2400 800 50  0001 C CNN
F 3 "~" H 2400 800 50  0001 C CNN
	1    2200 800 
	1    0    0    -1  
$EndComp
Connection ~ 750  4850
Wire Wire Line
	750  4850 850  4850
Wire Wire Line
	600  4850 750  4850
$Comp
L unsurv_offline_pcb_symbols:TF-101B CARD1
U 1 1 5E8A0B52
P 2250 7350
F 0 "CARD1" H 1633 8675 50  0000 C CNN
F 1 "TF-101B" H 1633 8584 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:TF-SMD_TF-101B" H 2250 6950 50  0000 C CNN
F 3 "" H 2250 6950 50  0001 C CNN
	1    2250 7350
	1    0    0    -1  
$EndComp
$Comp
L unsurv_offline_pcb_symbols:TTGO-Micro32-ESP32 U1
U 1 1 60D03794
P 6100 1750
F 0 "U1" H 6125 2715 50  0000 C CNN
F 1 "TTGO-Micro32-ESP32" H 6125 2624 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:TTGO-Micro32-ESP32" H 5550 2550 50  0001 C CNN
F 3 "" H 5550 2550 50  0001 C CNN
	1    6100 1750
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR06
U 1 1 60D05778
P 5200 1250
F 0 "#PWR06" H 5200 1100 50  0001 C CNN
F 1 "+3V3" H 5215 1423 50  0000 C CNN
F 2 "" H 5200 1250 50  0001 C CNN
F 3 "" H 5200 1250 50  0001 C CNN
	1    5200 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 1250 5200 1250
$Comp
L power:GND #PWR02
U 1 1 60D1B6D8
P 5400 950
F 0 "#PWR02" H 5400 700 50  0001 C CNN
F 1 "GND" H 5405 777 50  0000 C CNN
F 2 "" H 5400 950 50  0001 C CNN
F 3 "" H 5400 950 50  0001 C CNN
	1    5400 950 
	-1   0    0    1   
$EndComp
Wire Wire Line
	5400 1150 5400 950 
$Comp
L power:GND #PWR03
U 1 1 60D26BB7
P 6850 950
F 0 "#PWR03" H 6850 700 50  0001 C CNN
F 1 "GND" H 6855 777 50  0000 C CNN
F 2 "" H 6850 950 50  0001 C CNN
F 3 "" H 6850 950 50  0001 C CNN
	1    6850 950 
	-1   0    0    1   
$EndComp
Wire Wire Line
	6850 1150 6850 950 
$Comp
L power:GND #PWR016
U 1 1 60D33837
P 5400 2750
F 0 "#PWR016" H 5400 2500 50  0001 C CNN
F 1 "GND" H 5405 2577 50  0000 C CNN
F 2 "" H 5400 2750 50  0001 C CNN
F 3 "" H 5400 2750 50  0001 C CNN
	1    5400 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2550 5400 2750
Text GLabel 5400 1350 0    50   Input ~ 0
RESET
Text GLabel 5400 1750 0    50   Input ~ 0
35
Text GLabel 5400 2250 0    50   Input ~ 0
27
Text GLabel 5400 2050 0    50   Input ~ 0
ACC_INT1
Text GLabel 6850 1250 2    50   Input ~ 0
23
Text GLabel 6850 1350 2    50   Input ~ 0
SCL
Text GLabel 6850 1650 2    50   Input ~ 0
SDA
Text GLabel 6850 1850 2    50   Input ~ 0
19
Text GLabel 6850 1950 2    50   Input ~ 0
18
Text GLabel 6500 2750 3    50   Input ~ 0
15
Text GLabel 5800 2750 3    50   Input ~ 0
13
Text GLabel 6850 2050 2    50   Input ~ 0
5
Text GLabel 6850 2350 2    50   Input ~ 0
4
Text GLabel 6850 1550 2    50   Input ~ 0
RX0
Text GLabel 6850 1450 2    50   Input ~ 0
TX0
Text GLabel 6850 2450 2    50   Input ~ 0
0
Text GLabel 5400 2150 0    50   Input ~ 0
ACC_INT2
Text GLabel 5050 6650 0    50   Input ~ 0
ACC_INT1
Text GLabel 5050 6550 0    50   Input ~ 0
ACC_INT2
$Comp
L unsurv_offline_pcb_symbols:BMA400-I2C U7
U 1 1 60E450DC
P 5450 7150
F 0 "U7" H 5425 7975 50  0000 C CNN
F 1 "BMA400-I2C" H 5425 7884 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:BMA400" H 5200 7900 50  0001 C CNN
F 3 "" H 5200 7900 50  0001 C CNN
	1    5450 7150
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR041
U 1 1 60E5F5C8
P 6650 6550
F 0 "#PWR041" H 6650 6400 50  0001 C CNN
F 1 "+3V3" H 6665 6723 50  0000 C CNN
F 2 "" H 6650 6550 50  0001 C CNN
F 3 "" H 6650 6550 50  0001 C CNN
	1    6650 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR042
U 1 1 60E7299B
P 5950 6700
F 0 "#PWR042" H 5950 6450 50  0001 C CNN
F 1 "GND" H 5955 6527 50  0000 C CNN
F 2 "" H 5950 6700 50  0001 C CNN
F 3 "" H 5950 6700 50  0001 C CNN
	1    5950 6700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5850 6700 5950 6700
Wire Wire Line
	5800 6750 5850 6750
Wire Wire Line
	5850 6750 5850 6700
Text GLabel 5050 6950 0    50   Input ~ 0
SDA
Text GLabel 5800 7050 2    50   Input ~ 0
SCL
$Comp
L Device:C_Small C12
U 1 1 60EB3AB1
P 6400 6650
F 0 "C12" H 6492 6696 50  0000 L CNN
F 1 "100nF" H 6492 6605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6400 6650 50  0001 C CNN
F 3 "~" H 6400 6650 50  0001 C CNN
	1    6400 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 6550 6400 6550
Wire Wire Line
	5800 6650 5850 6650
Wire Wire Line
	5850 6650 5850 6700
Connection ~ 5850 6700
$Comp
L power:GND #PWR043
U 1 1 60EE5393
P 6400 6800
F 0 "#PWR043" H 6400 6550 50  0001 C CNN
F 1 "GND" H 6405 6627 50  0000 C CNN
F 2 "" H 6400 6800 50  0001 C CNN
F 3 "" H 6400 6800 50  0001 C CNN
	1    6400 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 6750 6400 6800
$Comp
L power:+3V3 #PWR045
U 1 1 60F02DD5
P 4100 6850
F 0 "#PWR045" H 4100 6700 50  0001 C CNN
F 1 "+3V3" H 4115 7023 50  0000 C CNN
F 2 "" H 4100 6850 50  0001 C CNN
F 3 "" H 4100 6850 50  0001 C CNN
	1    4100 6850
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C13
U 1 1 60F02DDB
P 4400 6950
F 0 "C13" H 4492 6996 50  0000 L CNN
F 1 "100nF" H 4492 6905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4400 6950 50  0001 C CNN
F 3 "~" H 4400 6950 50  0001 C CNN
	1    4400 6950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR047
U 1 1 60F02DE4
P 4400 7100
F 0 "#PWR047" H 4400 6850 50  0001 C CNN
F 1 "GND" H 4405 6927 50  0000 C CNN
F 2 "" H 4400 7100 50  0001 C CNN
F 3 "" H 4400 7100 50  0001 C CNN
	1    4400 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 7050 4400 7100
Connection ~ 6400 6550
Wire Wire Line
	5800 6550 6400 6550
Text GLabel 6950 5500 0    50   Input ~ 0
ANT1
Text GLabel 6950 5600 0    50   Input ~ 0
ANT2
Text GLabel 7850 5600 2    50   Input ~ 0
SDA
Text GLabel 7850 5700 2    50   Input ~ 0
SCL
Text GLabel 7850 6000 2    50   Input ~ 0
13
$Comp
L Device:C C11
U 1 1 60D0818A
P 8500 5650
F 0 "C11" H 8615 5696 50  0000 L CNN
F 1 "470nF" H 8615 5605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8538 5500 50  0001 C CNN
F 3 "~" H 8500 5650 50  0001 C CNN
	1    8500 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5500 8500 5500
$Comp
L power:GND #PWR038
U 1 1 60D08191
P 8500 5850
F 0 "#PWR038" H 8500 5600 50  0001 C CNN
F 1 "GND" H 8505 5677 50  0000 C CNN
F 2 "" H 8500 5850 50  0001 C CNN
F 3 "" H 8500 5850 50  0001 C CNN
	1    8500 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 5800 8500 5850
$Comp
L Device:C C10
U 1 1 60D3AC38
P 6400 5350
F 0 "C10" H 6515 5396 50  0000 L CNN
F 1 "1yF" H 6515 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6438 5200 50  0001 C CNN
F 3 "~" H 6400 5350 50  0001 C CNN
	1    6400 5350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 60D3AC3E
P 5900 5350
F 0 "C9" H 6015 5396 50  0000 L CNN
F 1 "0.1yF" H 6015 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5938 5200 50  0001 C CNN
F 3 "~" H 5900 5350 50  0001 C CNN
	1    5900 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 5200 5900 5200
Connection ~ 6400 5200
$Comp
L power:GND #PWR034
U 1 1 60D3AC47
P 6150 5550
F 0 "#PWR034" H 6150 5300 50  0001 C CNN
F 1 "GND" H 6155 5377 50  0000 C CNN
F 2 "" H 6150 5550 50  0001 C CNN
F 3 "" H 6150 5550 50  0001 C CNN
	1    6150 5550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR031
U 1 1 60D3AC4D
P 5700 5200
F 0 "#PWR031" H 5700 5050 50  0001 C CNN
F 1 "+3.3V" H 5715 5373 50  0000 C CNN
F 2 "" H 5700 5200 50  0001 C CNN
F 3 "" H 5700 5200 50  0001 C CNN
	1    5700 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 5200 5900 5200
Connection ~ 5900 5200
Wire Wire Line
	5900 5500 6150 5500
Wire Wire Line
	6150 5500 6150 5550
Wire Wire Line
	6400 5500 6150 5500
Connection ~ 6150 5500
Wire Wire Line
	6950 5400 6950 5200
Wire Wire Line
	6400 5200 6950 5200
Text GLabel 6950 5700 0    50   Input ~ 0
15
$Comp
L power:GND #PWR023
U 1 1 60D0D21B
P 4000 4050
F 0 "#PWR023" H 4000 3800 50  0001 C CNN
F 1 "GND" H 4005 3877 50  0000 C CNN
F 2 "" H 4000 4050 50  0001 C CNN
F 3 "" H 4000 4050 50  0001 C CNN
	1    4000 4050
	1    0    0    -1  
$EndComp
$Comp
L unsurv_offline_pcb_symbols:switch_würth_SMT_3.0x2.6mm S1
U 1 1 60D42342
P 3700 3850
F 0 "S1" H 3900 4365 50  0000 C CNN
F 1 "switch_würth_SMT_3.0x2.6mm" H 3900 4274 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:würth_SMT_3.0x2.6mm" H 3900 4250 50  0001 C CNN
F 3 "" H 3900 4250 50  0001 C CNN
	1    3700 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4050 4000 4050
Wire Wire Line
	3850 4050 3950 4050
Connection ~ 3950 4050
Wire Wire Line
	4200 3600 4200 3700
Connection ~ 4200 3700
$Comp
L Jumper:SolderJumper_2_Open JP1
U 1 1 60D44F59
P 2900 1900
F 0 "JP1" H 2900 2105 50  0000 C CNN
F 1 "SolderJumper_2_Open" H 2900 2014 50  0000 C CNN
F 2 "Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm" H 2900 1900 50  0001 C CNN
F 3 "~" H 2900 1900 50  0001 C CNN
	1    2900 1900
	0    1    1    0   
$EndComp
Wire Wire Line
	2900 2600 2900 2800
Wire Wire Line
	2900 1600 2900 1750
Wire Wire Line
	2650 1600 2900 1600
Wire Wire Line
	2900 2050 2900 2300
Wire Wire Line
	5800 6850 6150 6850
Wire Wire Line
	4100 6850 4400 6850
$Comp
L power:GND #PWR048
U 1 1 60F0B07C
P 5050 7200
F 0 "#PWR048" H 5050 6950 50  0001 C CNN
F 1 "GND" H 5055 7027 50  0000 C CNN
F 2 "" H 5050 7200 50  0001 C CNN
F 3 "" H 5050 7200 50  0001 C CNN
	1    5050 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 7050 5050 7200
Connection ~ 4400 6850
Wire Wire Line
	4400 6850 4800 6850
$Comp
L unsurv_offline_pcb_symbols:RF430CL330H U6
U 1 1 60D6C0D5
P 7400 5750
F 0 "U6" H 7400 6515 50  0000 C CNN
F 1 "RF430CL330H" H 7400 6424 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:RF430CL330H" H 7600 5450 50  0001 C CNN
F 3 "" H 7600 5450 50  0001 C CNN
	1    7400 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR033
U 1 1 60DC43B2
P 8000 5300
F 0 "#PWR033" H 8000 5050 50  0001 C CNN
F 1 "GND" H 8005 5127 50  0000 C CNN
F 2 "" H 8000 5300 50  0001 C CNN
F 3 "" H 8000 5300 50  0001 C CNN
	1    8000 5300
	-1   0    0    1   
$EndComp
Wire Wire Line
	8000 5300 8000 5400
Wire Wire Line
	8000 5400 7850 5400
$Comp
L power:GND #PWR039
U 1 1 60DD8818
P 8200 5900
F 0 "#PWR039" H 8200 5650 50  0001 C CNN
F 1 "GND" H 8205 5727 50  0000 C CNN
F 2 "" H 8200 5900 50  0001 C CNN
F 3 "" H 8200 5900 50  0001 C CNN
	1    8200 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 5800 8200 5800
Wire Wire Line
	8200 5800 8200 5900
Wire Wire Line
	7850 5900 8200 5900
Connection ~ 8200 5900
Wire Wire Line
	6150 6850 6150 7500
Wire Wire Line
	6150 7500 4800 7500
Wire Wire Line
	4800 7500 4800 6850
Connection ~ 4800 6850
Wire Wire Line
	4800 6850 5050 6850
Wire Wire Line
	3400 6750 2350 6750
Connection ~ 3400 6750
Wire Wire Line
	3150 6950 2350 6950
Connection ~ 3150 6950
Connection ~ 2900 7150
Wire Wire Line
	2650 7250 2350 7250
Connection ~ 2650 7250
Wire Wire Line
	2350 7150 2900 7150
Wire Wire Line
	3550 6750 3400 6750
Wire Wire Line
	3550 6950 3150 6950
Wire Wire Line
	2900 7150 3550 7150
Wire Wire Line
	3550 7250 2650 7250
Wire Wire Line
	3400 6250 3400 6750
Wire Wire Line
	3150 6250 3150 6950
Wire Wire Line
	2900 6250 2900 7150
Wire Wire Line
	2650 6250 2650 7250
Wire Wire Line
	2650 5950 2900 5950
Wire Wire Line
	2900 5950 3050 5950
Connection ~ 2900 5950
Connection ~ 3050 5950
Wire Wire Line
	3150 5950 3050 5950
Connection ~ 3150 5950
Wire Wire Line
	3050 5950 3050 5850
Wire Wire Line
	3400 5950 3150 5950
$Comp
L Device:R R15
U 1 1 5E3BE686
P 3400 6100
F 0 "R15" H 3470 6146 50  0000 L CNN
F 1 "10k" H 3470 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3330 6100 50  0001 C CNN
F 3 "~" H 3400 6100 50  0001 C CNN
	1    3400 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5E3BDCF5
P 3150 6100
F 0 "R14" H 3220 6146 50  0000 L CNN
F 1 "10k" H 3220 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3080 6100 50  0001 C CNN
F 3 "~" H 3150 6100 50  0001 C CNN
	1    3150 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5E3BD7DA
P 2900 6100
F 0 "R13" H 2970 6146 50  0000 L CNN
F 1 "10k" H 2970 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2830 6100 50  0001 C CNN
F 3 "~" H 2900 6100 50  0001 C CNN
	1    2900 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5E3BC6C3
P 2650 6100
F 0 "R12" H 2720 6146 50  0000 L CNN
F 1 "10k" H 2720 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2580 6100 50  0001 C CNN
F 3 "~" H 2650 6100 50  0001 C CNN
	1    2650 6100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR037
U 1 1 5E3B0DE8
P 3050 5850
F 0 "#PWR037" H 3050 5700 50  0001 C CNN
F 1 "+3.3V" H 3065 6023 50  0000 C CNN
F 2 "" H 3050 5850 50  0001 C CNN
F 3 "" H 3050 5850 50  0001 C CNN
	1    3050 5850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 60D7A70E
P 8850 3900
F 0 "#PWR021" H 8850 3650 50  0001 C CNN
F 1 "GND" H 8855 3727 50  0000 C CNN
F 2 "" H 8850 3900 50  0001 C CNN
F 3 "" H 8850 3900 50  0001 C CNN
	1    8850 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 3900 8850 3700
Wire Wire Line
	8850 3700 8950 3700
Connection ~ 8850 3700
Connection ~ 8950 3700
Wire Wire Line
	8950 3700 9050 3700
Connection ~ 9050 3700
Wire Wire Line
	9050 3700 9150 3700
Connection ~ 9150 3700
Wire Wire Line
	9150 3700 9250 3700
Connection ~ 9250 3700
Wire Wire Line
	9250 3700 9350 3700
Connection ~ 9350 3700
Wire Wire Line
	9350 3700 9450 3700
Connection ~ 9450 3700
Wire Wire Line
	9450 3700 9550 3700
Connection ~ 9550 3700
Wire Wire Line
	9550 3700 9650 3700
Connection ~ 9650 3700
Wire Wire Line
	9650 3700 9750 3700
Connection ~ 9750 3700
Wire Wire Line
	9750 3700 9850 3700
Connection ~ 9850 3700
Wire Wire Line
	9850 3700 9950 3700
Connection ~ 9950 3700
Wire Wire Line
	9950 3700 10050 3700
Connection ~ 10050 3700
Wire Wire Line
	10050 3700 10150 3700
$Comp
L power:+3.3V #PWR011
U 1 1 5E40FAF5
P 4800 1950
F 0 "#PWR011" H 4800 1800 50  0001 C CNN
F 1 "+3.3V" H 4815 2123 50  0000 C CNN
F 2 "" H 4800 1950 50  0001 C CNN
F 3 "" H 4800 1950 50  0001 C CNN
	1    4800 1950
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 1950 4250 1950
$EndSCHEMATC
