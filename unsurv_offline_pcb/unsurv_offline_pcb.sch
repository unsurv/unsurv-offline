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
L Battery_Management:MCP73831-2-OT U1
U 1 1 5E2A0C54
P 1800 1850
F 0 "U1" H 1800 2331 50  0000 C CNN
F 1 "MCP73831-2-OT" H 1800 2240 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 1850 1600 50  0001 L CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001984g.pdf" H 1650 1800 50  0001 C CNN
	1    1800 1850
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:VP0610T Q1
U 1 1 5E2AF209
P 2000 3700
F 0 "Q1" H 2205 3746 50  0000 L CNN
F 1 "VP0610T" H 2205 3655 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 2200 3625 50  0001 L CIN
F 3 "http://www.vishay.com/docs/70209/70209.pdf" H 2000 3700 50  0001 L CNN
	1    2000 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 5E2B1B42
P 1850 4500
F 0 "D1" H 1850 4284 50  0000 C CNN
F 1 "D_Schottky" H 1850 4375 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323" H 1850 4500 50  0001 C CNN
F 3 "~" H 1850 4500 50  0001 C CNN
	1    1850 4500
	-1   0    0    1   
$EndComp
$Comp
L Connector:DF12-20DP CN1
U 1 1 5E2C8826
P 10300 3150
F 0 "CN1" H 10728 4546 50  0000 L CNN
F 1 "DF12-20DP" H 10728 4455 50  0000 L CNN
F 2 "unsurv_offline_footprints:DF-12-20DP" H 10250 6000 50  0001 C CNN
F 3 "" H 10250 6000 50  0001 C CNN
	1    10300 3150
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_Open_Dual SW1
U 1 1 5E2CB73F
P 3700 2250
F 0 "SW1" H 3700 2460 50  0000 C CNN
F 1 "SW_Push_Open_Dual" H 3700 2369 50  0000 C CNN
F 2 "unsurv_offline_footprints:2x2 Switch" H 3700 2450 50  0001 C CNN
F 3 "~" H 3700 2450 50  0001 C CNN
	1    3700 2250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push_Open_Dual SW2
U 1 1 5E2D1132
P 3700 3000
F 0 "SW2" H 3700 3210 50  0000 C CNN
F 1 "SW_Push_Open_Dual" H 3700 3119 50  0000 C CNN
F 2 "unsurv_offline_footprints:2x2 Switch" H 3700 3200 50  0001 C CNN
F 3 "~" H 3700 3200 50  0001 C CNN
	1    3700 3000
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:APX803L20 U3
U 1 1 5E303E7E
P 3500 250
F 0 "U3" H 3450 1325 50  0000 C CNN
F 1 "APX803L20" H 3450 1234 50  0000 C CNN
F 2 "digikey-footprints:SOT-23-3" H 3450 1250 50  0001 C CNN
F 3 "" H 3450 1250 50  0001 C CNN
	1    3500 250 
	-1   0    0    1   
$EndComp
$Comp
L dk_USB-DVI-HDMI-Connectors:10103594-0001LF J1
U 1 1 5E307692
P 900 3700
F 0 "J1" H 914 4423 50  0000 C CNN
F 1 "10103594-0001LF" H 914 4332 50  0000 C CNN
F 2 "digikey-footprints:USB_Micro_B_Female_10103594-0001LF" H 1100 3900 60  0001 L CNN
F 3 "https://cdn.amphenol-icc.com/media/wysiwyg/files/drawing/10103594.pdf" H 1100 4000 60  0001 L CNN
F 4 "609-4050-1-ND" H 1100 4100 60  0001 L CNN "Digi-Key_PN"
F 5 "10103594-0001LF" H 1100 4200 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 1100 4300 60  0001 L CNN "Category"
F 7 "USB, DVI, HDMI Connectors" H 1100 4400 60  0001 L CNN "Family"
F 8 "https://cdn.amphenol-icc.com/media/wysiwyg/files/drawing/10103594.pdf" H 1100 4500 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/amphenol-icc-fci/10103594-0001LF/609-4050-1-ND/2350357" H 1100 4600 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN RCPT USB2.0 MICRO B SMD R/A" H 1100 4700 60  0001 L CNN "Description"
F 11 "Amphenol ICC (FCI)" H 1100 4800 60  0001 L CNN "Manufacturer"
F 12 "Active" H 1100 4900 60  0001 L CNN "Status"
	1    900  3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5E30F837
P 1400 3900
F 0 "#PWR05" H 1400 3650 50  0001 C CNN
F 1 "GND" H 1405 3727 50  0000 C CNN
F 2 "" H 1400 3900 50  0001 C CNN
F 3 "" H 1400 3900 50  0001 C CNN
	1    1400 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 3900 1150 3900
$Comp
L Device:C C2
U 1 1 5E320DA1
P 2350 1150
F 0 "C2" V 2098 1150 50  0000 C CNN
F 1 "4.7yF" V 2189 1150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2388 1000 50  0001 C CNN
F 3 "~" H 2350 1150 50  0001 C CNN
	1    2350 1150
	0    1    1    0   
$EndComp
$Comp
L Device:C C1
U 1 1 5E32223A
P 850 1650
F 0 "C1" H 735 1604 50  0000 R CNN
F 1 "4.7yF" H 735 1695 50  0000 R CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 888 1500 50  0001 C CNN
F 3 "~" H 850 1650 50  0001 C CNN
	1    850  1650
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 5E328050
P 1100 2100
F 0 "R1" H 1170 2146 50  0000 L CNN
F 1 "2k" H 1170 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1030 2100 50  0001 C CNN
F 3 "~" H 1100 2100 50  0001 C CNN
	1    1100 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5E32973F
P 2400 2650
F 0 "R3" H 2470 2696 50  0000 L CNN
F 1 "1k" H 2470 2605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2330 2650 50  0001 C CNN
F 3 "~" H 2400 2650 50  0001 C CNN
	1    2400 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 5E32B05A
P 2400 2250
F 0 "D2" V 2347 2328 50  0000 L CNN
F 1 "LED" V 2438 2328 50  0000 L CNN
F 2 "digikey-footprints:LED_0603" H 2400 2250 50  0001 C CNN
F 3 "~" H 2400 2250 50  0001 C CNN
	1    2400 2250
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5E32E8C7
P 1800 2350
F 0 "#PWR09" H 1800 2100 50  0001 C CNN
F 1 "GND" H 1805 2177 50  0000 C CNN
F 2 "" H 1800 2350 50  0001 C CNN
F 3 "" H 1800 2350 50  0001 C CNN
	1    1800 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 2150 1800 2350
Wire Wire Line
	2400 2100 2400 1950
Wire Wire Line
	2400 1950 2200 1950
Wire Wire Line
	2400 2500 2400 2400
Wire Wire Line
	2400 2950 2400 2800
Wire Wire Line
	1100 1950 1400 1950
$Comp
L power:GND #PWR03
U 1 1 5E331B06
P 1100 2350
F 0 "#PWR03" H 1100 2100 50  0001 C CNN
F 1 "GND" H 1105 2177 50  0000 C CNN
F 2 "" H 1100 2350 50  0001 C CNN
F 3 "" H 1100 2350 50  0001 C CNN
	1    1100 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1100 2350 1100 2250
Wire Wire Line
	600  1500 850  1500
Connection ~ 850  1500
Wire Wire Line
	1800 1550 1800 1500
Wire Wire Line
	850  1500 1800 1500
$Comp
L power:GND #PWR02
U 1 1 5E3363DB
P 850 1950
F 0 "#PWR02" H 850 1700 50  0001 C CNN
F 1 "GND" H 855 1777 50  0000 C CNN
F 2 "" H 850 1950 50  0001 C CNN
F 3 "" H 850 1950 50  0001 C CNN
	1    850  1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  1950 850  1800
Wire Wire Line
	2200 1750 2200 1600
$Comp
L power:GND #PWR013
U 1 1 5E338243
P 2650 1150
F 0 "#PWR013" H 2650 900 50  0001 C CNN
F 1 "GND" H 2655 977 50  0000 C CNN
F 2 "" H 2650 1150 50  0001 C CNN
F 3 "" H 2650 1150 50  0001 C CNN
	1    2650 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1150 2500 1150
$Comp
L power:GND #PWR06
U 1 1 5E33995B
P 1450 950
F 0 "#PWR06" H 1450 700 50  0001 C CNN
F 1 "GND" H 1455 777 50  0000 C CNN
F 2 "" H 1450 950 50  0001 C CNN
F 3 "" H 1450 950 50  0001 C CNN
	1    1450 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR014
U 1 1 5E33AE8F
P 2650 1600
F 0 "#PWR014" H 2650 1450 50  0001 C CNN
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
L power:+BATT #PWR018
U 1 1 5E33F9F4
P 3000 1000
F 0 "#PWR018" H 3000 850 50  0001 C CNN
F 1 "+BATT" H 3015 1173 50  0000 C CNN
F 2 "" H 3000 1000 50  0001 C CNN
F 3 "" H 3000 1000 50  0001 C CNN
	1    3000 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1000 3300 1000
$Comp
L power:GND #PWR024
U 1 1 5E3410D6
P 4000 1100
F 0 "#PWR024" H 4000 850 50  0001 C CNN
F 1 "GND" H 4005 927 50  0000 C CNN
F 2 "" H 4000 1100 50  0001 C CNN
F 3 "" H 4000 1100 50  0001 C CNN
	1    4000 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 1100 3800 1100
Text GLabel 4650 900  2    50   Input ~ 0
BATTALARM
$Comp
L power:+BATT #PWR010
U 1 1 5E34D33B
P 2100 3400
F 0 "#PWR010" H 2100 3250 50  0001 C CNN
F 1 "+BATT" H 2115 3573 50  0000 C CNN
F 2 "" H 2100 3400 50  0001 C CNN
F 3 "" H 2100 3400 50  0001 C CNN
	1    2100 3400
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 5E34F77A
P 1350 3500
F 0 "#PWR04" H 1350 3350 50  0001 C CNN
F 1 "+5V" H 1365 3673 50  0000 C CNN
F 2 "" H 1350 3500 50  0001 C CNN
F 3 "" H 1350 3500 50  0001 C CNN
	1    1350 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1350 3500 1150 3500
$Comp
L power:+5V #PWR01
U 1 1 5E350CEC
P 600 1500
F 0 "#PWR01" H 600 1350 50  0001 C CNN
F 1 "+5V" H 615 1673 50  0000 C CNN
F 2 "" H 600 1500 50  0001 C CNN
F 3 "" H 600 1500 50  0001 C CNN
	1    600  1500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 5E351A65
P 2400 2950
F 0 "#PWR011" H 2400 2800 50  0001 C CNN
F 1 "+5V" H 2415 3123 50  0000 C CNN
F 2 "" H 2400 2950 50  0001 C CNN
F 3 "" H 2400 2950 50  0001 C CNN
	1    2400 2950
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 5E35363C
P 1600 3700
F 0 "#PWR07" H 1600 3550 50  0001 C CNN
F 1 "+5V" H 1615 3873 50  0000 C CNN
F 2 "" H 1600 3700 50  0001 C CNN
F 3 "" H 1600 3700 50  0001 C CNN
	1    1600 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 3500 2100 3400
Wire Wire Line
	1700 4500 1600 4500
Wire Wire Line
	2850 4500 2750 4500
Wire Wire Line
	2850 4400 2750 4400
Wire Wire Line
	2750 4400 2750 4500
Connection ~ 2750 4500
Wire Wire Line
	2750 4500 2600 4500
$Comp
L Device:R R12
U 1 1 5E35BA68
P 4300 1050
F 0 "R12" H 4370 1096 50  0000 L CNN
F 1 "100k" H 4370 1005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4230 1050 50  0001 C CNN
F 3 "~" H 4300 1050 50  0001 C CNN
	1    4300 1050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR026
U 1 1 5E35CBF5
P 4300 1350
F 0 "#PWR026" H 4300 1200 50  0001 C CNN
F 1 "+3.3V" H 4315 1523 50  0000 C CNN
F 2 "" H 4300 1350 50  0001 C CNN
F 3 "" H 4300 1350 50  0001 C CNN
	1    4300 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	4300 1350 4300 1200
Wire Wire Line
	3800 900  4300 900 
Wire Wire Line
	4650 900  4300 900 
Connection ~ 4300 900 
Wire Wire Line
	2850 4200 2600 4200
Wire Wire Line
	2600 4200 2600 4500
Connection ~ 2600 4500
Wire Wire Line
	2850 4300 2700 4300
Wire Wire Line
	2700 4300 2700 3850
$Comp
L power:GND #PWR020
U 1 1 5E365FB4
P 3200 3750
F 0 "#PWR020" H 3200 3500 50  0001 C CNN
F 1 "GND" H 3205 3577 50  0000 C CNN
F 2 "" H 3200 3750 50  0001 C CNN
F 3 "" H 3200 3750 50  0001 C CNN
	1    3200 3750
	-1   0    0    1   
$EndComp
Wire Wire Line
	3200 3750 3200 3850
Connection ~ 3200 3850
Wire Wire Line
	3200 3850 3200 3900
Wire Wire Line
	3550 4200 3550 3850
$Comp
L Device:R R11
U 1 1 5E36C1C0
P 3950 4350
F 0 "R11" H 4020 4396 50  0000 L CNN
F 1 "30.9k" H 4020 4305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3880 4350 50  0001 C CNN
F 3 "~" H 3950 4350 50  0001 C CNN
	1    3950 4350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5E36D408
P 3950 3950
F 0 "R10" H 4020 3996 50  0000 L CNN
F 1 "10k" H 4020 3905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3880 3950 50  0001 C CNN
F 3 "~" H 3950 3950 50  0001 C CNN
	1    3950 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3950 4200 3950 4150
Wire Wire Line
	3550 4300 3750 4300
Wire Wire Line
	3750 4300 3750 4150
Wire Wire Line
	3750 4150 3950 4150
Connection ~ 3950 4150
Wire Wire Line
	3950 4150 3950 4100
Wire Wire Line
	3550 4500 3700 4500
Wire Wire Line
	3550 4400 3700 4400
Wire Wire Line
	3700 4400 3700 4500
Connection ~ 3700 4500
Wire Wire Line
	3700 4500 3950 4500
$Comp
L Device:C C4
U 1 1 5E372397
P 4200 4700
F 0 "C4" H 4315 4746 50  0000 L CNN
F 1 "4.7yF" H 4315 4655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4238 4550 50  0001 C CNN
F 3 "~" H 4200 4700 50  0001 C CNN
	1    4200 4700
	1    0    0    -1  
$EndComp
Connection ~ 3950 4500
$Comp
L power:GND #PWR025
U 1 1 5E374B35
P 4200 4900
F 0 "#PWR025" H 4200 4650 50  0001 C CNN
F 1 "GND" H 4205 4727 50  0000 C CNN
F 2 "" H 4200 4900 50  0001 C CNN
F 3 "" H 4200 4900 50  0001 C CNN
	1    4200 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4950 4200 4900
$Comp
L power:+3.3V #PWR027
U 1 1 5E37759A
P 4450 4500
F 0 "#PWR027" H 4450 4350 50  0001 C CNN
F 1 "+3.3V" H 4465 4673 50  0000 C CNN
F 2 "" H 4450 4500 50  0001 C CNN
F 3 "" H 4450 4500 50  0001 C CNN
	1    4450 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR016
U 1 1 5E39E3AA
P 2900 2250
F 0 "#PWR016" H 2900 2100 50  0001 C CNN
F 1 "+3.3V" H 2915 2423 50  0000 C CNN
F 2 "" H 2900 2250 50  0001 C CNN
F 3 "" H 2900 2250 50  0001 C CNN
	1    2900 2250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR017
U 1 1 5E39F2C4
P 2900 3000
F 0 "#PWR017" H 2900 2850 50  0001 C CNN
F 1 "+3.3V" H 2915 3173 50  0000 C CNN
F 2 "" H 2900 3000 50  0001 C CNN
F 3 "" H 2900 3000 50  0001 C CNN
	1    2900 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5E3A00A1
P 3200 2450
F 0 "R7" V 2993 2450 50  0000 C CNN
F 1 "10k" V 3084 2450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3130 2450 50  0001 C CNN
F 3 "~" H 3200 2450 50  0001 C CNN
	1    3200 2450
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 5E3A103F
P 3200 3200
F 0 "R8" V 2993 3200 50  0000 C CNN
F 1 "10k" V 3084 3200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3130 3200 50  0001 C CNN
F 3 "~" H 3200 3200 50  0001 C CNN
	1    3200 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	3350 3200 3500 3200
Wire Wire Line
	3350 2450 3500 2450
Wire Wire Line
	2900 2250 2900 2450
Wire Wire Line
	2900 2450 3050 2450
Wire Wire Line
	2900 3200 3050 3200
Wire Wire Line
	2900 3000 2900 3200
Text GLabel 4150 3200 2    50   Input ~ 0
EN
Wire Wire Line
	4150 3200 3900 3200
$Comp
L power:+3.3V #PWR019
U 1 1 5E3B0DE8
P 3050 5850
F 0 "#PWR019" H 3050 5700 50  0001 C CNN
F 1 "+3.3V" H 3065 6023 50  0000 C CNN
F 2 "" H 3050 5850 50  0001 C CNN
F 3 "" H 3050 5850 50  0001 C CNN
	1    3050 5850
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
Wire Wire Line
	3550 6750 3400 6750
Wire Wire Line
	3550 6950 3150 6950
Wire Wire Line
	2350 7150 2900 7150
Wire Wire Line
	3550 7250 2650 7250
$Comp
L power:+3.3V #PWR022
U 1 1 5E3B9642
P 3850 7050
F 0 "#PWR022" H 3850 6900 50  0001 C CNN
F 1 "+3.3V" H 3865 7223 50  0000 C CNN
F 2 "" H 3850 7050 50  0001 C CNN
F 3 "" H 3850 7050 50  0001 C CNN
	1    3850 7050
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 7050 2350 7050
$Comp
L Device:R R4
U 1 1 5E3BC6C3
P 2650 6100
F 0 "R4" H 2720 6146 50  0000 L CNN
F 1 "10k" H 2720 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2580 6100 50  0001 C CNN
F 3 "~" H 2650 6100 50  0001 C CNN
	1    2650 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5E3BD7DA
P 2900 6100
F 0 "R5" H 2970 6146 50  0000 L CNN
F 1 "10k" H 2970 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2830 6100 50  0001 C CNN
F 3 "~" H 2900 6100 50  0001 C CNN
	1    2900 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5E3BDCF5
P 3150 6100
F 0 "R6" H 3220 6146 50  0000 L CNN
F 1 "10k" H 3220 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3080 6100 50  0001 C CNN
F 3 "~" H 3150 6100 50  0001 C CNN
	1    3150 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E3BE686
P 3400 6100
F 0 "R9" H 3470 6146 50  0000 L CNN
F 1 "10k" H 3470 6055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3330 6100 50  0001 C CNN
F 3 "~" H 3400 6100 50  0001 C CNN
	1    3400 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 5950 3150 5950
Wire Wire Line
	3050 5950 3050 5850
Connection ~ 3150 5950
Wire Wire Line
	3150 5950 3050 5950
Connection ~ 3050 5950
Connection ~ 2900 5950
Wire Wire Line
	2900 5950 3050 5950
Wire Wire Line
	2650 5950 2900 5950
Wire Wire Line
	2650 6250 2650 7250
Connection ~ 2650 7250
Wire Wire Line
	2650 7250 2350 7250
Wire Wire Line
	2900 6250 2900 7150
Connection ~ 2900 7150
Wire Wire Line
	2900 7150 3550 7150
Wire Wire Line
	3150 6250 3150 6950
Connection ~ 3150 6950
Wire Wire Line
	3150 6950 2350 6950
Wire Wire Line
	3400 6250 3400 6750
Connection ~ 3400 6750
Wire Wire Line
	3400 6750 2350 6750
$Comp
L power:GND #PWR021
U 1 1 5E3D06FF
P 3850 6850
F 0 "#PWR021" H 3850 6600 50  0001 C CNN
F 1 "GND" H 3855 6677 50  0000 C CNN
F 2 "" H 3850 6850 50  0001 C CNN
F 3 "" H 3850 6850 50  0001 C CNN
	1    3850 6850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 6850 2350 6850
$Comp
L power:GND #PWR012
U 1 1 5E3D4C21
P 2450 5850
F 0 "#PWR012" H 2450 5600 50  0001 C CNN
F 1 "GND" H 2455 5677 50  0000 C CNN
F 2 "" H 2450 5850 50  0001 C CNN
F 3 "" H 2450 5850 50  0001 C CNN
	1    2450 5850
	-1   0    0    1   
$EndComp
Text GLabel 7250 1550 2    50   Input ~ 0
BATTALARM
$Comp
L dk_Motion-Sensors-IMUs-Inertial-Measurement-Units:MPU-6050_NRND U4
U 1 1 5E3DF9A8
P 6050 5850
F 0 "U4" H 6100 5053 60  0000 C CNN
F 1 "MPU-6050_NRND" H 6100 4947 60  0000 C CNN
F 2 "digikey-footprints:QFN-24-1EP_4x4mm" H 6250 6050 60  0001 L CNN
F 3 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 6250 6150 60  0001 L CNN
F 4 "1428-1007-1-ND" H 6250 6250 60  0001 L CNN "Digi-Key_PN"
F 5 "MPU-6050" H 6250 6350 60  0001 L CNN "MPN"
F 6 "Sensors, Transducers" H 6250 6450 60  0001 L CNN "Category"
F 7 "Motion Sensors - IMUs (Inertial Measurement Units)" H 6250 6550 60  0001 L CNN "Family"
F 8 "https://store.invensense.com/datasheets/invensense/MPU-6050_DataSheet_V3%204.pdf" H 6250 6650 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/tdk-invensense/MPU-6050/1428-1007-1-ND/4038010" H 6250 6750 60  0001 L CNN "DK_Detail_Page"
F 10 "IMU ACCEL/GYRO 3-AXIS I2C 24QFN" H 6250 6850 60  0001 L CNN "Description"
F 11 "TDK InvenSense" H 6250 6950 60  0001 L CNN "Manufacturer"
F 12 "Not For New Designs" H 6250 7050 60  0001 L CNN "Status"
	1    6050 5850
	1    0    0    -1  
$EndComp
Text GLabel 7250 1750 2    50   Input ~ 0
4
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:22-23-2021 J3
U 1 1 5E3E7138
P 8650 1900
F 0 "J3" V 8741 1772 50  0000 R CNN
F 1 "TXRX" V 8650 1772 50  0000 R CNN
F 2 "digikey-footprints:PinHeader_1x2_P2.54mm_Drill1.02mm" H 8850 2100 60  0001 L CNN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8850 2200 60  0001 L CNN
F 4 "WM4200-ND" H 8850 2300 60  0001 L CNN "Digi-Key_PN"
F 5 "22-23-2021" H 8850 2400 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 8850 2500 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 8850 2600 60  0001 L CNN "Family"
F 8 "https://media.digikey.com/pdf/Data%20Sheets/Molex%20PDFs/A-6373-N_Series_Dwg_2010-12-03.pdf" H 8850 2700 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/molex/22-23-2021/WM4200-ND/26667" H 8850 2800 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER VERT 2POS 2.54MM" H 8850 2900 60  0001 L CNN "Description"
F 11 "Molex" H 8850 3000 60  0001 L CNN "Manufacturer"
F 12 "Active" H 8850 3100 60  0001 L CNN "Status"
	1    8650 1900
	0    -1   -1   0   
$EndComp
Text GLabel 7250 2750 2    50   Input ~ 0
SDA
Text GLabel 7250 2850 2    50   Input ~ 0
SCL
Text GLabel 7250 2550 2    50   Input ~ 0
18
Text GLabel 7250 2650 2    50   Input ~ 0
19
Text GLabel 7250 2950 2    50   Input ~ 0
23
Text GLabel 7250 3450 2    50   Input ~ 0
GRST
Text GLabel 7250 3050 2    50   Input ~ 0
ACC_INT
Text GLabel 7250 3350 2    50   Input ~ 0
32
Text GLabel 7250 3550 2    50   Input ~ 0
34
Text GLabel 7250 3650 2    50   Input ~ 0
35
$Comp
L power:GND #PWR034
U 1 1 5E401430
P 6650 4100
F 0 "#PWR034" H 6650 3850 50  0001 C CNN
F 1 "GND" H 6655 3927 50  0000 C CNN
F 2 "" H 6650 4100 50  0001 C CNN
F 3 "" H 6650 4100 50  0001 C CNN
	1    6650 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4100 6650 3950
Text GLabel 6050 2750 0    50   Input ~ 0
RX1
Text GLabel 6050 2850 0    50   Input ~ 0
TX1
$Comp
L power:+3.3V #PWR028
U 1 1 5E40FAF5
P 4850 3400
F 0 "#PWR028" H 4850 3250 50  0001 C CNN
F 1 "+3.3V" H 4865 3573 50  0000 C CNN
F 2 "" H 4850 3400 50  0001 C CNN
F 3 "" H 4850 3400 50  0001 C CNN
	1    4850 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5E41108F
P 4700 3650
F 0 "R13" H 4770 3696 50  0000 L CNN
F 1 "4.7k" H 4770 3605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4630 3650 50  0001 C CNN
F 3 "~" H 4700 3650 50  0001 C CNN
	1    4700 3650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 5E412365
P 5000 3650
F 0 "R14" H 5070 3696 50  0000 L CNN
F 1 "4.7k" H 5070 3605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4930 3650 50  0001 C CNN
F 3 "~" H 5000 3650 50  0001 C CNN
	1    5000 3650
	1    0    0    -1  
$EndComp
Text GLabel 4700 3800 3    50   Input ~ 0
SDA
Text GLabel 5000 3800 3    50   Input ~ 0
SCL
Text GLabel 6650 5750 2    50   Input ~ 0
SDA
Text GLabel 5550 5650 0    50   Input ~ 0
SCL
Text GLabel 6650 5650 2    50   Input ~ 0
ACC_INT
$Comp
L power:+3.3V #PWR032
U 1 1 5E420B2D
P 6050 4550
F 0 "#PWR032" H 6050 4400 50  0001 C CNN
F 1 "+3.3V" H 6065 4723 50  0000 C CNN
F 2 "" H 6050 4550 50  0001 C CNN
F 3 "" H 6050 4550 50  0001 C CNN
	1    6050 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5E42F2B5
P 6300 5000
F 0 "C7" V 6048 5000 50  0000 C CNN
F 1 "10nF" V 6139 5000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6338 4850 50  0001 C CNN
F 3 "~" H 6300 5000 50  0001 C CNN
	1    6300 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 5000 6150 5150
$Comp
L power:GND #PWR033
U 1 1 5E4337DC
P 6600 5000
F 0 "#PWR033" H 6600 4750 50  0001 C CNN
F 1 "GND" H 6605 4827 50  0000 C CNN
F 2 "" H 6600 5000 50  0001 C CNN
F 3 "" H 6600 5000 50  0001 C CNN
	1    6600 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5000 6450 5000
Wire Wire Line
	6150 5000 6050 5000
Connection ~ 6150 5000
Connection ~ 6050 5000
Wire Wire Line
	6050 5000 6050 5150
$Comp
L Device:C C9
U 1 1 5E43E1E6
P 7350 6100
F 0 "C9" H 7465 6146 50  0000 L CNN
F 1 "2.2nF" H 7465 6055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7388 5950 50  0001 C CNN
F 3 "~" H 7350 6100 50  0001 C CNN
	1    7350 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5E43EC24
P 6850 6100
F 0 "C8" H 6965 6146 50  0000 L CNN
F 1 "0.1yF" H 6965 6055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6888 5950 50  0001 C CNN
F 3 "~" H 6850 6100 50  0001 C CNN
	1    6850 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 5950 6850 5950
$Comp
L power:GND #PWR035
U 1 1 5E446A57
P 6850 6300
F 0 "#PWR035" H 6850 6050 50  0001 C CNN
F 1 "GND" H 6855 6127 50  0000 C CNN
F 2 "" H 6850 6300 50  0001 C CNN
F 3 "" H 6850 6300 50  0001 C CNN
	1    6850 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 6250 6850 6300
Wire Wire Line
	7350 6250 7350 6300
$Comp
L power:GND #PWR036
U 1 1 5E44845D
P 7350 6300
F 0 "#PWR036" H 7350 6050 50  0001 C CNN
F 1 "GND" H 7355 6127 50  0000 C CNN
F 2 "" H 7350 6300 50  0001 C CNN
F 3 "" H 7350 6300 50  0001 C CNN
	1    7350 6300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 5950 7350 5850
Wire Wire Line
	7350 5850 6650 5850
$Comp
L Device:C C6
U 1 1 5E46FA76
P 5900 4700
F 0 "C6" V 5648 4700 50  0000 C CNN
F 1 "0.1yF" V 5739 4700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5938 4550 50  0001 C CNN
F 3 "~" H 5900 4700 50  0001 C CNN
	1    5900 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	6050 4550 6050 4700
Wire Wire Line
	6050 4700 6050 5000
Connection ~ 6050 4700
$Comp
L power:GND #PWR031
U 1 1 5E47717F
P 5600 4700
F 0 "#PWR031" H 5600 4450 50  0001 C CNN
F 1 "GND" H 5605 4527 50  0000 C CNN
F 2 "" H 5600 4700 50  0001 C CNN
F 3 "" H 5600 4700 50  0001 C CNN
	1    5600 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 4700 5600 4700
$Comp
L power:GND #PWR029
U 1 1 5E47B235
P 5350 5350
F 0 "#PWR029" H 5350 5100 50  0001 C CNN
F 1 "GND" H 5355 5177 50  0000 C CNN
F 2 "" H 5350 5350 50  0001 C CNN
F 3 "" H 5350 5350 50  0001 C CNN
	1    5350 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 5350 5350 5350
$Comp
L power:GND #PWR030
U 1 1 5E482AB2
P 5450 6550
F 0 "#PWR030" H 5450 6300 50  0001 C CNN
F 1 "GND" H 5455 6377 50  0000 C CNN
F 2 "" H 5450 6550 50  0001 C CNN
F 3 "" H 5450 6550 50  0001 C CNN
	1    5450 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 6550 5450 6550
Text GLabel 10300 850  0    50   Input ~ 0
34
Text GLabel 10300 950  0    50   Input ~ 0
35
Text GLabel 10300 1050 0    50   Input ~ 0
32
Text GLabel 10300 1150 0    50   Input ~ 0
GRST
Text GLabel 10300 1250 0    50   Input ~ 0
RX1
Text GLabel 10300 1350 0    50   Input ~ 0
TX1
Text GLabel 10300 1450 0    50   Input ~ 0
SDA
Text GLabel 10300 1550 0    50   Input ~ 0
SCL
$Comp
L power:+3.3V #PWR043
U 1 1 5E48C288
P 9800 1650
F 0 "#PWR043" H 9800 1500 50  0001 C CNN
F 1 "+3.3V" H 9815 1823 50  0000 C CNN
F 2 "" H 9800 1650 50  0001 C CNN
F 3 "" H 9800 1650 50  0001 C CNN
	1    9800 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 1650 10050 1650
Wire Wire Line
	10300 1750 10050 1750
Wire Wire Line
	10050 1750 10050 1650
Connection ~ 10050 1650
Wire Wire Line
	10050 1650 10300 1650
$Comp
L power:GND #PWR045
U 1 1 5E4953F4
P 10100 2850
F 0 "#PWR045" H 10100 2600 50  0001 C CNN
F 1 "GND" H 10105 2677 50  0000 C CNN
F 2 "" H 10100 2850 50  0001 C CNN
F 3 "" H 10100 2850 50  0001 C CNN
	1    10100 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 1850 10100 1850
Wire Wire Line
	10100 1850 10100 1950
Wire Wire Line
	10300 1950 10100 1950
Connection ~ 10100 1950
Wire Wire Line
	10100 1950 10100 2050
Wire Wire Line
	10300 2050 10100 2050
Connection ~ 10100 2050
Wire Wire Line
	10100 2050 10100 2150
Wire Wire Line
	10300 2150 10100 2150
Connection ~ 10100 2150
Wire Wire Line
	10100 2150 10100 2250
Wire Wire Line
	10300 2250 10100 2250
Connection ~ 10100 2250
Wire Wire Line
	10100 2250 10100 2350
Wire Wire Line
	10300 2350 10100 2350
Connection ~ 10100 2350
Wire Wire Line
	10100 2350 10100 2450
Wire Wire Line
	10300 2450 10100 2450
Connection ~ 10100 2450
Wire Wire Line
	10100 2450 10100 2550
Wire Wire Line
	10300 2550 10100 2550
Connection ~ 10100 2550
Wire Wire Line
	10100 2550 10100 2650
Wire Wire Line
	10300 2650 10100 2650
Connection ~ 10100 2650
Wire Wire Line
	10100 2650 10100 2750
Wire Wire Line
	10300 2750 10100 2750
Connection ~ 10100 2750
Wire Wire Line
	10100 2750 10100 2850
$Comp
L Device:R R2
U 1 1 5E355803
P 1750 3250
F 0 "R2" H 1820 3296 50  0000 L CNN
F 1 "100k" H 1820 3205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1680 3250 50  0001 C CNN
F 3 "~" H 1750 3250 50  0001 C CNN
	1    1750 3250
	-1   0    0    1   
$EndComp
Wire Wire Line
	2000 4500 2100 4500
Wire Wire Line
	2100 3900 2100 4500
Connection ~ 2100 4500
Wire Wire Line
	2100 4500 2600 4500
$Comp
L power:GND #PWR08
U 1 1 5E3771A3
P 1750 3050
F 0 "#PWR08" H 1750 2800 50  0001 C CNN
F 1 "GND" H 1755 2877 50  0000 C CNN
F 2 "" H 1750 3050 50  0001 C CNN
F 3 "" H 1750 3050 50  0001 C CNN
	1    1750 3050
	-1   0    0    1   
$EndComp
Wire Wire Line
	1750 3050 1750 3100
$Comp
L power:GND #PWR023
U 1 1 5E86631E
P 3950 3700
F 0 "#PWR023" H 3950 3450 50  0001 C CNN
F 1 "GND" H 3955 3527 50  0000 C CNN
F 2 "" H 3950 3700 50  0001 C CNN
F 3 "" H 3950 3700 50  0001 C CNN
	1    3950 3700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5E868B5C
P 2700 3500
F 0 "#PWR015" H 2700 3250 50  0001 C CNN
F 1 "GND" H 2705 3327 50  0000 C CNN
F 2 "" H 2700 3500 50  0001 C CNN
F 3 "" H 2700 3500 50  0001 C CNN
	1    2700 3500
	-1   0    0    1   
$EndComp
Wire Wire Line
	3200 3850 3550 3850
$Comp
L Device:C C3
U 1 1 5E3637DE
P 2700 3700
F 0 "C3" V 2448 3700 50  0000 C CNN
F 1 "10nF" V 2539 3700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2738 3550 50  0001 C CNN
F 3 "~" H 2700 3700 50  0001 C CNN
	1    2700 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	3950 3700 3950 3800
Wire Wire Line
	2700 3500 2700 3550
Connection ~ 4200 4900
Wire Wire Line
	4200 4900 4200 4850
Wire Wire Line
	3950 4500 4200 4500
Wire Wire Line
	4200 4550 4200 4500
Connection ~ 4200 4500
Wire Wire Line
	4200 4500 4450 4500
$Comp
L unsurv_offline_symbols:NCP59800 U2
U 1 1 5E89DDF0
P 3400 4050
F 0 "U2" H 3600 4773 50  0000 C CNN
F 1 "NCP59800" H 3600 4864 50  0000 C CNN
F 2 "unsurv_offline_footprints:NCP59800" H 4100 4750 50  0001 C CNN
F 3 "" H 4100 4750 50  0001 C CNN
	1    3400 4050
	-1   0    0    1   
$EndComp
$Comp
L unsurv_offline_symbols:TF-101B CARD1
U 1 1 5E8A0B52
P 2250 7350
F 0 "CARD1" H 1633 8675 50  0000 C CNN
F 1 "TF-101B" H 1633 8584 50  0000 C CNN
F 2 "unsurv_offline_footprints:TF-SMD_TF-101B" H 2250 6950 50  0001 C CNN
F 3 "" H 2250 6950 50  0001 C CNN
	1    2250 7350
	1    0    0    -1  
$EndComp
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
Text GLabel 1250 3600 2    50   Input ~ 0
D-
Text GLabel 1250 3700 2    50   Input ~ 0
D+
Wire Wire Line
	1250 3600 1150 3600
Wire Wire Line
	1250 3700 1150 3700
$Comp
L Interface_USB:CH340G U6
U 1 1 5E8D4BE0
P 9800 5300
F 0 "U6" H 9800 4611 50  0000 C CNN
F 1 "CH340G" H 9800 4520 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 9850 4750 50  0001 L CNN
F 3 "http://www.datasheet5.com/pdf-local-2195953" H 9450 6100 50  0001 C CNN
	1    9800 5300
	1    0    0    -1  
$EndComp
$Comp
L unsurv_offline_symbols:UMH3N Q2
U 1 1 5E8E5B40
P 8100 5050
F 0 "Q2" H 8125 5375 50  0000 C CNN
F 1 "UMH3N" H 8125 5284 50  0000 C CNN
F 2 "unsurv_offline_footprints:UMH3N" H 8150 4800 50  0001 C CNN
F 3 "" H 8150 4800 50  0001 C CNN
	1    8100 5050
	1    0    0    -1  
$EndComp
Text GLabel 7250 1450 2    50   Input ~ 0
TX0
Text GLabel 7250 1650 2    50   Input ~ 0
RX0
Text GLabel 8550 1800 0    50   Input ~ 0
TX0
Text GLabel 8550 1900 0    50   Input ~ 0
RX0
$Comp
L Device:C C12
U 1 1 5E8F5D5D
P 8900 4200
F 0 "C12" H 9015 4246 50  0000 L CNN
F 1 "100nF" H 9015 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8938 4050 50  0001 C CNN
F 3 "~" H 8900 4200 50  0001 C CNN
	1    8900 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5E8FB387
P 9450 4200
F 0 "C13" H 9565 4246 50  0000 L CNN
F 1 "10yF" H 9565 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9488 4050 50  0001 C CNN
F 3 "~" H 9450 4200 50  0001 C CNN
	1    9450 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 3700 1750 3700
Wire Wire Line
	1600 3700 1600 4500
Connection ~ 1600 3700
Wire Wire Line
	1750 3400 1750 3700
Connection ~ 1750 3700
Wire Wire Line
	1750 3700 1800 3700
$Comp
L power:+3.3V #PWR040
U 1 1 5E93703A
P 8550 4050
F 0 "#PWR040" H 8550 3900 50  0001 C CNN
F 1 "+3.3V" H 8565 4223 50  0000 C CNN
F 2 "" H 8550 4050 50  0001 C CNN
F 3 "" H 8550 4050 50  0001 C CNN
	1    8550 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 4050 8900 4050
Wire Wire Line
	9450 4050 8900 4050
Connection ~ 8900 4050
Wire Wire Line
	9450 4050 9800 4050
Wire Wire Line
	9800 4050 9800 4600
Connection ~ 9450 4050
Wire Wire Line
	9700 4700 9700 4600
Wire Wire Line
	9700 4600 9800 4600
Connection ~ 9800 4600
Wire Wire Line
	9800 4600 9800 4700
$Comp
L power:GND #PWR041
U 1 1 5E950833
P 8900 4450
F 0 "#PWR041" H 8900 4200 50  0001 C CNN
F 1 "GND" H 8905 4277 50  0000 C CNN
F 2 "" H 8900 4450 50  0001 C CNN
F 3 "" H 8900 4450 50  0001 C CNN
	1    8900 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR042
U 1 1 5E9520A1
P 9450 4450
F 0 "#PWR042" H 9450 4200 50  0001 C CNN
F 1 "GND" H 9455 4277 50  0000 C CNN
F 2 "" H 9450 4450 50  0001 C CNN
F 3 "" H 9450 4450 50  0001 C CNN
	1    9450 4450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR044
U 1 1 5E9530ED
P 10050 5900
F 0 "#PWR044" H 10050 5650 50  0001 C CNN
F 1 "GND" H 10055 5727 50  0000 C CNN
F 2 "" H 10050 5900 50  0001 C CNN
F 3 "" H 10050 5900 50  0001 C CNN
	1    10050 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 5900 9800 5900
Wire Wire Line
	9450 4350 9450 4450
Wire Wire Line
	8900 4350 8900 4450
Text GLabel 10350 5700 2    50   Input ~ 0
RTS
Text GLabel 10350 5600 2    50   Input ~ 0
DTR
Text GLabel 10350 5000 2    50   Input ~ 0
RX0
Text GLabel 10350 4900 2    50   Input ~ 0
TX0
Text GLabel 9250 5200 0    50   Input ~ 0
D+
Text GLabel 9250 5300 0    50   Input ~ 0
D-
Text GLabel 6050 1350 0    50   Input ~ 0
EN
Text GLabel 7250 1350 2    50   Input ~ 0
0
Wire Wire Line
	10350 4900 10200 4900
Wire Wire Line
	10350 5000 10200 5000
Wire Wire Line
	10350 5600 10200 5600
Wire Wire Line
	10350 5700 10200 5700
Wire Wire Line
	9400 5200 9250 5200
Wire Wire Line
	9400 5300 9250 5300
Text GLabel 7750 4950 0    50   Input ~ 0
RTS
Text GLabel 7750 5050 0    50   Input ~ 0
DTR
Text GLabel 7750 5150 0    50   Input ~ 0
0
Text GLabel 8500 5150 2    50   Input ~ 0
DTR
Text GLabel 8500 5050 2    50   Input ~ 0
RTS
Text GLabel 8500 4950 2    50   Input ~ 0
EN
Wire Wire Line
	8500 4950 8350 4950
Wire Wire Line
	8500 5050 8350 5050
Wire Wire Line
	8350 5150 8500 5150
Wire Wire Line
	7900 4950 7750 4950
Wire Wire Line
	7900 5050 7750 5050
Wire Wire Line
	7900 5150 7750 5150
$Comp
L power:+3.3V #PWR039
U 1 1 5E9ED589
P 7100 850
F 0 "#PWR039" H 7100 700 50  0001 C CNN
F 1 "+3.3V" H 7115 1023 50  0000 C CNN
F 2 "" H 7100 850 50  0001 C CNN
F 3 "" H 7100 850 50  0001 C CNN
	1    7100 850 
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5EA4CFFD
P 4300 2450
F 0 "C5" V 4048 2450 50  0000 C CNN
F 1 "DNP" V 4139 2450 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4338 2300 50  0001 C CNN
F 3 "~" H 4300 2450 50  0001 C CNN
	1    4300 2450
	0    1    1    0   
$EndComp
Text GLabel 4600 2450 2    50   Input ~ 0
0
Wire Wire Line
	4150 2450 3900 2450
Wire Wire Line
	4450 2450 4600 2450
$Comp
L dk_Rectangular-Connectors-Headers-Male-Pins:S2B-PH-SM4-TB_LF__SN_ J2
U 1 1 5EA63059
P 2000 850
F 0 "J2" H 2178 904 50  0000 L CNN
F 1 "S2B-PH-SM4-TB_LF__SN_" H 2178 813 50  0000 L CNN
F 2 "digikey-footprints:PinHeader_2x1mm_P2mm_SMD_RA" H 2200 1050 60  0001 L CNN
F 3 "http://www.jst-mfg.com/product/pdf/eng/ePH.pdf" H 2200 1150 60  0001 L CNN
F 4 "455-1749-1-ND" H 2200 1250 60  0001 L CNN "Digi-Key_PN"
F 5 "S2B-PH-SM4-TB(LF)(SN)" H 2200 1350 60  0001 L CNN "MPN"
F 6 "Connectors, Interconnects" H 2200 1450 60  0001 L CNN "Category"
F 7 "Rectangular Connectors - Headers, Male Pins" H 2200 1550 60  0001 L CNN "Family"
F 8 "http://www.jst-mfg.com/product/pdf/eng/ePH.pdf" H 2200 1650 60  0001 L CNN "DK_Datasheet_Link"
F 9 "/product-detail/en/jst-sales-america-inc/S2B-PH-SM4-TB(LF)(SN)/455-1749-1-ND/926846" H 2200 1750 60  0001 L CNN "DK_Detail_Page"
F 10 "CONN HEADER SMD R/A 2POS 2MM" H 2200 1850 60  0001 L CNN "Description"
F 11 "JST Sales America Inc." H 2200 1950 60  0001 L CNN "Manufacturer"
F 12 "Active" H 2200 2050 60  0001 L CNN "Status"
	1    2000 850 
	-1   0    0    1   
$EndComp
Wire Wire Line
	2200 1150 2200 950 
Wire Wire Line
	2200 950  2000 950 
Connection ~ 2200 1150
Wire Wire Line
	1900 950  1450 950 
$Comp
L RF_Module:ESP32-WROOM-32 U5
U 1 1 5E2AB5E3
P 6650 2550
F 0 "U5" H 6650 4131 50  0000 C CNN
F 1 "ESP32-WROOM-32" H 6650 4040 50  0000 C CNN
F 2 "RF_Module:ESP32-WROOM-32" H 6650 1050 50  0001 C CNN
F 3 "https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf" H 6350 2600 50  0001 C CNN
	1    6650 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3500 4850 3500
Wire Wire Line
	4850 3400 4850 3500
Connection ~ 4850 3500
Wire Wire Line
	4850 3500 5000 3500
Wire Wire Line
	7100 850  7100 1150
Wire Wire Line
	7100 1150 6650 1150
$EndSCHEMATC
