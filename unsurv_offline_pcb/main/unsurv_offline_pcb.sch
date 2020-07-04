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
L Regulator_Linear:APX803L20 U2
U 1 1 5E303E7E
P 3500 250
F 0 "U2" H 3450 1325 50  0000 C CNN
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
L power:GND #PWR07
U 1 1 5E30F837
P 1400 3900
F 0 "#PWR07" H 1400 3650 50  0001 C CNN
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
L Device:R R2
U 1 1 5E328050
P 1100 2100
F 0 "R2" H 1170 2146 50  0000 L CNN
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
L power:GND #PWR010
U 1 1 5E32E8C7
P 1800 2350
F 0 "#PWR010" H 1800 2100 50  0001 C CNN
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
	1100 1950 1400 1950
$Comp
L power:GND #PWR05
U 1 1 5E331B06
P 1100 2350
F 0 "#PWR05" H 1100 2100 50  0001 C CNN
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
L power:GND #PWR03
U 1 1 5E3363DB
P 850 1950
F 0 "#PWR03" H 850 1700 50  0001 C CNN
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
L power:GND #PWR014
U 1 1 5E338243
P 2650 1150
F 0 "#PWR014" H 2650 900 50  0001 C CNN
F 1 "GND" H 2655 977 50  0000 C CNN
F 2 "" H 2650 1150 50  0001 C CNN
F 3 "" H 2650 1150 50  0001 C CNN
	1    2650 1150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 1150 2500 1150
$Comp
L power:GND #PWR08
U 1 1 5E33995B
P 1450 950
F 0 "#PWR08" H 1450 700 50  0001 C CNN
F 1 "GND" H 1455 777 50  0000 C CNN
F 2 "" H 1450 950 50  0001 C CNN
F 3 "" H 1450 950 50  0001 C CNN
	1    1450 950 
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR015
U 1 1 5E33AE8F
P 2650 1600
F 0 "#PWR015" H 2650 1450 50  0001 C CNN
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
L power:+BATT #PWR017
U 1 1 5E33F9F4
P 3000 1000
F 0 "#PWR017" H 3000 850 50  0001 C CNN
F 1 "+BATT" H 3015 1173 50  0000 C CNN
F 2 "" H 3000 1000 50  0001 C CNN
F 3 "" H 3000 1000 50  0001 C CNN
	1    3000 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1000 3300 1000
$Comp
L power:GND #PWR025
U 1 1 5E3410D6
P 4000 1100
F 0 "#PWR025" H 4000 850 50  0001 C CNN
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
L power:+5V #PWR06
U 1 1 5E34F77A
P 1350 3500
F 0 "#PWR06" H 1350 3350 50  0001 C CNN
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
L Device:R R8
U 1 1 5E35BA68
P 4300 1050
F 0 "R8" H 4370 1096 50  0000 L CNN
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
	3450 4300 3200 4300
Wire Wire Line
	3450 4400 3300 4400
Wire Wire Line
	3300 4400 3300 3950
$Comp
L power:GND #PWR022
U 1 1 5E365FB4
P 3800 3850
F 0 "#PWR022" H 3800 3600 50  0001 C CNN
F 1 "GND" H 3805 3677 50  0000 C CNN
F 2 "" H 3800 3850 50  0001 C CNN
F 3 "" H 3800 3850 50  0001 C CNN
	1    3800 3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3800 3850 3800 3950
Connection ~ 3800 3950
Wire Wire Line
	3800 3950 3800 4000
Wire Wire Line
	4150 4300 4150 3950
$Comp
L Device:R R10
U 1 1 5E36C1C0
P 4550 4450
F 0 "R10" H 4620 4496 50  0000 L CNN
F 1 "30.9k" H 4620 4405 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4480 4450 50  0001 C CNN
F 3 "~" H 4550 4450 50  0001 C CNN
	1    4550 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5E36D408
P 4550 4050
F 0 "R9" H 4620 4096 50  0000 L CNN
F 1 "10k" H 4620 4005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4480 4050 50  0001 C CNN
F 3 "~" H 4550 4050 50  0001 C CNN
	1    4550 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 4300 4550 4250
Wire Wire Line
	4150 4400 4350 4400
Wire Wire Line
	4350 4400 4350 4250
Wire Wire Line
	4350 4250 4550 4250
Connection ~ 4550 4250
Wire Wire Line
	4550 4250 4550 4200
Wire Wire Line
	4150 4600 4300 4600
Wire Wire Line
	4150 4500 4300 4500
Wire Wire Line
	4300 4500 4300 4600
Connection ~ 4300 4600
Wire Wire Line
	4300 4600 4550 4600
$Comp
L Device:C C6
U 1 1 5E372397
P 4800 4800
F 0 "C6" H 4915 4846 50  0000 L CNN
F 1 "4.7yF" H 4915 4755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4838 4650 50  0001 C CNN
F 3 "~" H 4800 4800 50  0001 C CNN
	1    4800 4800
	1    0    0    -1  
$EndComp
Connection ~ 4550 4600
$Comp
L power:GND #PWR028
U 1 1 5E374B35
P 4800 5000
F 0 "#PWR028" H 4800 4750 50  0001 C CNN
F 1 "GND" H 4805 4827 50  0000 C CNN
F 2 "" H 4800 5000 50  0001 C CNN
F 3 "" H 4800 5000 50  0001 C CNN
	1    4800 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5050 4800 5000
$Comp
L power:+3.3V #PWR030
U 1 1 5E37759A
P 5050 4600
F 0 "#PWR030" H 5050 4450 50  0001 C CNN
F 1 "+3.3V" H 5065 4773 50  0000 C CNN
F 2 "" H 5050 4600 50  0001 C CNN
F 3 "" H 5050 4600 50  0001 C CNN
	1    5050 4600
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR020
U 1 1 5E3B0DE8
P 3050 5850
F 0 "#PWR020" H 3050 5700 50  0001 C CNN
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
L power:+3.3V #PWR024
U 1 1 5E3B9642
P 3850 7050
F 0 "#PWR024" H 3850 6900 50  0001 C CNN
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
L Device:R R7
U 1 1 5E3BE686
P 3400 6100
F 0 "R7" H 3470 6146 50  0000 L CNN
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
L power:GND #PWR023
U 1 1 5E3D06FF
P 3850 6850
F 0 "#PWR023" H 3850 6600 50  0001 C CNN
F 1 "GND" H 3855 6677 50  0000 C CNN
F 2 "" H 3850 6850 50  0001 C CNN
F 3 "" H 3850 6850 50  0001 C CNN
	1    3850 6850
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 6850 2350 6850
$Comp
L power:GND #PWR013
U 1 1 5E3D4C21
P 2450 5850
F 0 "#PWR013" H 2450 5600 50  0001 C CNN
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
Text GLabel 7250 3050 2    50   Input ~ 0
ACC_INT
$Comp
L power:GND #PWR037
U 1 1 5E401430
P 6650 4100
F 0 "#PWR037" H 6650 3850 50  0001 C CNN
F 1 "GND" H 6655 3927 50  0000 C CNN
F 2 "" H 6650 4100 50  0001 C CNN
F 3 "" H 6650 4100 50  0001 C CNN
	1    6650 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4100 6650 3950
$Comp
L power:+3.3V #PWR029
U 1 1 5E40FAF5
P 4950 1650
F 0 "#PWR029" H 4950 1500 50  0001 C CNN
F 1 "+3.3V" H 4965 1823 50  0000 C CNN
F 2 "" H 4950 1650 50  0001 C CNN
F 3 "" H 4950 1650 50  0001 C CNN
	1    4950 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R11
U 1 1 5E41108F
P 4800 1900
F 0 "R11" H 4870 1946 50  0000 L CNN
F 1 "4.7k" H 4870 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4730 1900 50  0001 C CNN
F 3 "~" H 4800 1900 50  0001 C CNN
	1    4800 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5E412365
P 5100 1900
F 0 "R12" H 5170 1946 50  0000 L CNN
F 1 "4.7k" H 5170 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5030 1900 50  0001 C CNN
F 3 "~" H 5100 1900 50  0001 C CNN
	1    5100 1900
	1    0    0    -1  
$EndComp
Text GLabel 4800 2050 3    50   Input ~ 0
SDA
Text GLabel 5100 2050 3    50   Input ~ 0
SCL
Text GLabel 6650 5750 2    50   Input ~ 0
SDA
Text GLabel 5550 5650 0    50   Input ~ 0
SCL
Text GLabel 6650 5650 2    50   Input ~ 0
ACC_INT
$Comp
L power:+3.3V #PWR034
U 1 1 5E420B2D
P 6050 4550
F 0 "#PWR034" H 6050 4400 50  0001 C CNN
F 1 "+3.3V" H 6065 4723 50  0000 C CNN
F 2 "" H 6050 4550 50  0001 C CNN
F 3 "" H 6050 4550 50  0001 C CNN
	1    6050 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5E42F2B5
P 6300 5000
F 0 "C8" V 6048 5000 50  0000 C CNN
F 1 "10nF" V 6139 5000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6338 4850 50  0001 C CNN
F 3 "~" H 6300 5000 50  0001 C CNN
	1    6300 5000
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 5000 6150 5150
$Comp
L power:GND #PWR036
U 1 1 5E4337DC
P 6600 5000
F 0 "#PWR036" H 6600 4750 50  0001 C CNN
F 1 "GND" H 6605 4827 50  0000 C CNN
F 2 "" H 6600 5000 50  0001 C CNN
F 3 "" H 6600 5000 50  0001 C CNN
	1    6600 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5000 6450 5000
$Comp
L Device:C C10
U 1 1 5E43E1E6
P 7350 6100
F 0 "C10" H 7465 6146 50  0000 L CNN
F 1 "2.2nF" H 7465 6055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7388 5950 50  0001 C CNN
F 3 "~" H 7350 6100 50  0001 C CNN
	1    7350 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5E43EC24
P 6850 6100
F 0 "C9" H 6965 6146 50  0000 L CNN
F 1 "0.1yF" H 6965 6055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6888 5950 50  0001 C CNN
F 3 "~" H 6850 6100 50  0001 C CNN
	1    6850 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 5950 6850 5950
$Comp
L power:GND #PWR038
U 1 1 5E446A57
P 6850 6300
F 0 "#PWR038" H 6850 6050 50  0001 C CNN
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
L power:GND #PWR040
U 1 1 5E44845D
P 7350 6300
F 0 "#PWR040" H 7350 6050 50  0001 C CNN
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
L Device:C C7
U 1 1 5E46FA76
P 5900 4700
F 0 "C7" V 5648 4700 50  0000 C CNN
F 1 "0.1yF" V 5739 4700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5938 4550 50  0001 C CNN
F 3 "~" H 5900 4700 50  0001 C CNN
	1    5900 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	6050 4550 6050 4700
Connection ~ 6050 4700
$Comp
L power:GND #PWR033
U 1 1 5E47717F
P 5600 4700
F 0 "#PWR033" H 5600 4450 50  0001 C CNN
F 1 "GND" H 5605 4527 50  0000 C CNN
F 2 "" H 5600 4700 50  0001 C CNN
F 3 "" H 5600 4700 50  0001 C CNN
	1    5600 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 4700 5600 4700
$Comp
L power:GND #PWR031
U 1 1 5E47B235
P 5350 5350
F 0 "#PWR031" H 5350 5100 50  0001 C CNN
F 1 "GND" H 5355 5177 50  0000 C CNN
F 2 "" H 5350 5350 50  0001 C CNN
F 3 "" H 5350 5350 50  0001 C CNN
	1    5350 5350
	-1   0    0    1   
$EndComp
Wire Wire Line
	5550 5350 5350 5350
$Comp
L power:GND #PWR032
U 1 1 5E482AB2
P 5450 6550
F 0 "#PWR032" H 5450 6300 50  0001 C CNN
F 1 "GND" H 5455 6377 50  0000 C CNN
F 2 "" H 5450 6550 50  0001 C CNN
F 3 "" H 5450 6550 50  0001 C CNN
	1    5450 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 6550 5450 6550
$Comp
L power:GND #PWR027
U 1 1 5E86631E
P 4550 3800
F 0 "#PWR027" H 4550 3550 50  0001 C CNN
F 1 "GND" H 4555 3627 50  0000 C CNN
F 2 "" H 4550 3800 50  0001 C CNN
F 3 "" H 4550 3800 50  0001 C CNN
	1    4550 3800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5E868B5C
P 3300 3600
F 0 "#PWR021" H 3300 3350 50  0001 C CNN
F 1 "GND" H 3305 3427 50  0000 C CNN
F 2 "" H 3300 3600 50  0001 C CNN
F 3 "" H 3300 3600 50  0001 C CNN
	1    3300 3600
	-1   0    0    1   
$EndComp
Wire Wire Line
	3800 3950 4150 3950
$Comp
L Device:C C5
U 1 1 5E3637DE
P 3300 3800
F 0 "C5" V 3048 3800 50  0000 C CNN
F 1 "10nF" V 3139 3800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3338 3650 50  0001 C CNN
F 3 "~" H 3300 3800 50  0001 C CNN
	1    3300 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	4550 3800 4550 3900
Wire Wire Line
	3300 3600 3300 3650
Wire Wire Line
	4800 5000 4800 4950
Wire Wire Line
	4550 4600 4800 4600
Wire Wire Line
	4800 4650 4800 4600
Connection ~ 4800 4600
Wire Wire Line
	4800 4600 5050 4600
$Comp
L unsurv_offline_symbols:NCP59800 U3
U 1 1 5E89DDF0
P 4000 4150
F 0 "U3" H 4200 4873 50  0000 C CNN
F 1 "NCP59800" H 4200 4964 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:NCP59800" H 4700 4850 50  0001 C CNN
F 3 "" H 4700 4850 50  0001 C CNN
	1    4000 4150
	-1   0    0    1   
$EndComp
$Comp
L unsurv_offline_symbols:TF-101B CARD1
U 1 1 5E8A0B52
P 2250 7350
F 0 "CARD1" H 1633 8675 50  0000 C CNN
F 1 "TF-101B" H 1633 8584 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:TF-SMD_TF-101B" H 2250 6950 50  0001 C CNN
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
L unsurv_offline_symbols:UMH3N Q2
U 1 1 5E8E5B40
P 8500 5400
F 0 "Q2" H 8525 5725 50  0000 C CNN
F 1 "UMH3N" H 8525 5634 50  0000 C CNN
F 2 "unsurv_offline_pcb_footprints:UMH3N" H 8550 5150 50  0001 C CNN
F 3 "" H 8550 5150 50  0001 C CNN
	1    8500 5400
	1    0    0    -1  
$EndComp
Text GLabel 7250 1450 2    50   Input ~ 0
TX0
Text GLabel 7250 1650 2    50   Input ~ 0
RX0
$Comp
L power:+3.3V #PWR043
U 1 1 5E93703A
P 8950 4400
F 0 "#PWR043" H 8950 4250 50  0001 C CNN
F 1 "+3.3V" H 8965 4573 50  0000 C CNN
F 2 "" H 8950 4400 50  0001 C CNN
F 3 "" H 8950 4400 50  0001 C CNN
	1    8950 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 4400 10200 4950
Wire Wire Line
	10100 5050 10100 4950
Wire Wire Line
	10100 4950 10200 4950
Connection ~ 10200 4950
Wire Wire Line
	10200 4950 10200 5050
$Comp
L power:GND #PWR050
U 1 1 5E9530ED
P 10450 6250
F 0 "#PWR050" H 10450 6000 50  0001 C CNN
F 1 "GND" H 10455 6077 50  0000 C CNN
F 2 "" H 10450 6250 50  0001 C CNN
F 3 "" H 10450 6250 50  0001 C CNN
	1    10450 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10450 6250 10200 6250
Text GLabel 10750 6050 2    50   Input ~ 0
RTS
Text GLabel 10750 5950 2    50   Input ~ 0
DTR
Text GLabel 10750 5350 2    50   Input ~ 0
RX0
Text GLabel 10750 5250 2    50   Input ~ 0
TX0
Text GLabel 9650 5550 0    50   Input ~ 0
D+
Text GLabel 9650 5650 0    50   Input ~ 0
D-
Text GLabel 6050 1350 0    50   Input ~ 0
EN
Text GLabel 7250 1350 2    50   Input ~ 0
0
Wire Wire Line
	10750 5250 10600 5250
Wire Wire Line
	10750 5350 10600 5350
Wire Wire Line
	10750 5950 10600 5950
Wire Wire Line
	10750 6050 10600 6050
Wire Wire Line
	9800 5550 9650 5550
Wire Wire Line
	9800 5650 9650 5650
Text GLabel 8150 5300 0    50   Input ~ 0
RTS
Text GLabel 8150 5400 0    50   Input ~ 0
DTR
Text GLabel 8150 5500 0    50   Input ~ 0
0
Text GLabel 8900 5500 2    50   Input ~ 0
DTR
Text GLabel 8900 5400 2    50   Input ~ 0
RTS
Text GLabel 8900 5300 2    50   Input ~ 0
EN
Wire Wire Line
	8900 5300 8750 5300
Wire Wire Line
	8900 5400 8750 5400
Wire Wire Line
	8750 5500 8900 5500
Wire Wire Line
	8300 5300 8150 5300
Wire Wire Line
	8300 5400 8150 5400
Wire Wire Line
	8300 5500 8150 5500
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
	4800 1750 4950 1750
Wire Wire Line
	4950 1650 4950 1750
Connection ~ 4950 1750
Wire Wire Line
	4950 1750 5100 1750
Wire Wire Line
	7100 850  7100 1150
Wire Wire Line
	7100 1150 6650 1150
Wire Wire Line
	2400 2800 2400 3000
Wire Wire Line
	3200 4600 3300 4600
$Comp
L power:+5V #PWR011
U 1 1 5EE827C0
P 2400 3000
F 0 "#PWR011" H 2400 2850 50  0001 C CNN
F 1 "+5V" H 2415 3173 50  0000 C CNN
F 2 "" H 2400 3000 50  0001 C CNN
F 3 "" H 2400 3000 50  0001 C CNN
	1    2400 3000
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR09
U 1 1 5EE84493
P 1450 4500
F 0 "#PWR09" H 1450 4350 50  0001 C CNN
F 1 "+BATT" H 1465 4673 50  0000 C CNN
F 2 "" H 1450 4500 50  0001 C CNN
F 3 "" H 1450 4500 50  0001 C CNN
	1    1450 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 5E35363C
P 750 4850
F 0 "#PWR02" H 750 4700 50  0001 C CNN
F 1 "+5V" H 765 5023 50  0000 C CNN
F 2 "" H 750 4850 50  0001 C CNN
F 3 "" H 750 4850 50  0001 C CNN
	1    750  4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 4600 3300 4500
Wire Wire Line
	3300 4500 3450 4500
Connection ~ 3300 4600
Wire Wire Line
	3300 4600 3450 4600
Connection ~ 9850 4400
Wire Wire Line
	9850 4400 10200 4400
Connection ~ 9300 4400
Wire Wire Line
	9850 4400 9300 4400
Wire Wire Line
	8950 4400 9300 4400
Wire Wire Line
	9300 4700 9300 4800
Wire Wire Line
	9850 4700 9850 4800
$Comp
L power:GND #PWR048
U 1 1 5E9520A1
P 9850 4800
F 0 "#PWR048" H 9850 4550 50  0001 C CNN
F 1 "GND" H 9855 4627 50  0000 C CNN
F 2 "" H 9850 4800 50  0001 C CNN
F 3 "" H 9850 4800 50  0001 C CNN
	1    9850 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR046
U 1 1 5E950833
P 9300 4800
F 0 "#PWR046" H 9300 4550 50  0001 C CNN
F 1 "GND" H 9305 4627 50  0000 C CNN
F 2 "" H 9300 4800 50  0001 C CNN
F 3 "" H 9300 4800 50  0001 C CNN
	1    9300 4800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5E8FB387
P 9850 4550
F 0 "C12" H 9965 4596 50  0000 L CNN
F 1 "10yF" H 9965 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9888 4400 50  0001 C CNN
F 3 "~" H 9850 4550 50  0001 C CNN
	1    9850 4550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C11
U 1 1 5E8F5D5D
P 9300 4550
F 0 "C11" H 9415 4596 50  0000 L CNN
F 1 "100nF" H 9415 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9338 4400 50  0001 C CNN
F 3 "~" H 9300 4550 50  0001 C CNN
	1    9300 4550
	1    0    0    -1  
$EndComp
$Comp
L Interface_USB:CH340G U7
U 1 1 5E8D4BE0
P 10200 5650
F 0 "U7" H 10200 4961 50  0000 C CNN
F 1 "CH340G" H 10200 4870 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 10250 5100 50  0001 L CNN
F 3 "http://www.datasheet5.com/pdf-local-2195953" H 9850 6450 50  0001 C CNN
	1    10200 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5EFA7F2F
P 2850 4650
F 0 "C4" H 2965 4696 50  0000 L CNN
F 1 "1yF" H 2965 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2888 4500 50  0001 C CNN
F 3 "~" H 2850 4650 50  0001 C CNN
	1    2850 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5EFA7F35
P 2400 4650
F 0 "C3" H 2515 4696 50  0000 L CNN
F 1 "100nF" H 2515 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2438 4500 50  0001 C CNN
F 3 "~" H 2400 4650 50  0001 C CNN
	1    2400 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4300 3200 4500
Wire Wire Line
	2200 4500 2400 4500
Connection ~ 3200 4500
Wire Wire Line
	3200 4500 3200 4600
Connection ~ 2400 4500
Wire Wire Line
	2400 4500 2850 4500
Connection ~ 2850 4500
Wire Wire Line
	2850 4500 3200 4500
$Comp
L power:GND #PWR012
U 1 1 5F03CA35
P 2400 4850
F 0 "#PWR012" H 2400 4600 50  0001 C CNN
F 1 "GND" H 2405 4677 50  0000 C CNN
F 2 "" H 2400 4850 50  0001 C CNN
F 3 "" H 2400 4850 50  0001 C CNN
	1    2400 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5F044F49
P 2850 4850
F 0 "#PWR016" H 2850 4600 50  0001 C CNN
F 1 "GND" H 2855 4677 50  0000 C CNN
F 2 "" H 2850 4850 50  0001 C CNN
F 3 "" H 2850 4850 50  0001 C CNN
	1    2850 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 4800 2400 4850
Wire Wire Line
	2850 4800 2850 4850
Text GLabel 9400 1400 0    50   Input ~ 0
SCL
Text GLabel 9400 1500 0    50   Input ~ 0
SDA
$Comp
L power:+3.3V #PWR041
U 1 1 5EFBD731
P 8900 800
F 0 "#PWR041" H 8900 650 50  0001 C CNN
F 1 "+3.3V" H 8915 973 50  0000 C CNN
F 2 "" H 8900 800 50  0001 C CNN
F 3 "" H 8900 800 50  0001 C CNN
	1    8900 800 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 800  9400 800 
Text GLabel 9400 1000 0    50   Input ~ 0
12
Text GLabel 9400 1100 0    50   Input ~ 0
13
Text GLabel 9400 1200 0    50   Input ~ 0
14
Text GLabel 9400 1300 0    50   Input ~ 0
15
$Comp
L power:GND #PWR042
U 1 1 5EFF83B5
P 8900 900
F 0 "#PWR042" H 8900 650 50  0001 C CNN
F 1 "GND" H 8905 727 50  0000 C CNN
F 2 "" H 8900 900 50  0001 C CNN
F 3 "" H 8900 900 50  0001 C CNN
	1    8900 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 900  9400 900 
Connection ~ 4800 5000
$Comp
L unsurv_offline_symbols:1.27mm_female_connector J3
U 1 1 5F086F73
P 9600 1100
F 0 "J3" H 9628 1076 50  0000 L CNN
F 1 "1.27mm_female_connector" H 9628 985 50  0000 L CNN
F 2 "unsurv_offline_pcb_footprints:LPPB081NGCN-RC" H 9600 1100 50  0001 C CNN
F 3 "~" H 9600 1100 50  0001 C CNN
	1    9600 1100
	1    0    0    -1  
$EndComp
Text GLabel 7250 1950 2    50   Input ~ 0
12
Text GLabel 7250 2050 2    50   Input ~ 0
13
Text GLabel 7250 2150 2    50   Input ~ 0
14
Text GLabel 7250 2250 2    50   Input ~ 0
15
$Comp
L RF_GPS:ublox_SAM-M8Q U6
U 1 1 5F0D188A
P 9550 2850
F 0 "U6" H 9550 2261 50  0000 C CNN
F 1 "ublox_SAM-M8Q" H 9550 2170 50  0000 C CNN
F 2 "RF_GPS:ublox_SAM-M8Q" H 10050 2400 50  0001 C CNN
F 3 "https://www.u-blox.com/sites/default/files/SAM-M8Q_DataSheet_%28UBX-16012619%29.pdf" H 9550 2850 50  0001 C CNN
	1    9550 2850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR047
U 1 1 5F0E4E41
P 9450 2100
F 0 "#PWR047" H 9450 1950 50  0001 C CNN
F 1 "+3.3V" H 9465 2273 50  0000 C CNN
F 2 "" H 9450 2100 50  0001 C CNN
F 3 "" H 9450 2100 50  0001 C CNN
	1    9450 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 2350 9450 2250
Wire Wire Line
	9450 2250 9550 2250
Wire Wire Line
	9650 2250 9650 2350
Connection ~ 9450 2250
Wire Wire Line
	9450 2250 9450 2100
Text GLabel 10250 2950 2    50   Input ~ 0
SDA
Text GLabel 10250 3050 2    50   Input ~ 0
SCL
Wire Wire Line
	10050 2950 10250 2950
Wire Wire Line
	10050 3050 10250 3050
$Comp
L power:GND #PWR049
U 1 1 5F10DFA3
P 10050 3400
F 0 "#PWR049" H 10050 3150 50  0001 C CNN
F 1 "GND" H 10055 3227 50  0000 C CNN
F 2 "" H 10050 3400 50  0001 C CNN
F 3 "" H 10050 3400 50  0001 C CNN
	1    10050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	10050 3400 10050 3350
Wire Wire Line
	10050 3350 9550 3350
Text GLabel 8900 2750 0    50   Input ~ 0
16
Wire Wire Line
	8900 2750 9050 2750
Wire Wire Line
	9550 2350 9550 2250
Connection ~ 9550 2250
Wire Wire Line
	9550 2250 9650 2250
Text GLabel 7250 2350 2    50   Input ~ 0
16
Wire Wire Line
	6050 4700 6050 5150
$Comp
L power:+3.3V #PWR035
U 1 1 5F00156D
P 6200 4700
F 0 "#PWR035" H 6200 4550 50  0001 C CNN
F 1 "+3.3V" H 6215 4873 50  0000 C CNN
F 2 "" H 6200 4700 50  0001 C CNN
F 3 "" H 6200 4700 50  0001 C CNN
	1    6200 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 4700 6200 4850
Wire Wire Line
	6200 4850 6150 4850
Wire Wire Line
	6150 4850 6150 5000
Connection ~ 6150 5000
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5EFDDF31
P 3300 1750
F 0 "H1" H 3400 1799 50  0000 L CNN
F 1 "MountingHole_Pad" H 3400 1708 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad" H 3300 1750 50  0001 C CNN
F 3 "~" H 3300 1750 50  0001 C CNN
	1    3300 1750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5EFDF865
P 3300 2150
F 0 "H2" H 3400 2199 50  0000 L CNN
F 1 "MountingHole_Pad" H 3400 2108 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.2mm_M2_Pad" H 3300 2150 50  0001 C CNN
F 3 "~" H 3300 2150 50  0001 C CNN
	1    3300 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 5EFE62C4
P 3050 1850
F 0 "#PWR018" H 3050 1600 50  0001 C CNN
F 1 "GND" H 3055 1677 50  0000 C CNN
F 2 "" H 3050 1850 50  0001 C CNN
F 3 "" H 3050 1850 50  0001 C CNN
	1    3050 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 5EFEC240
P 3050 2250
F 0 "#PWR019" H 3050 2000 50  0001 C CNN
F 1 "GND" H 3055 2077 50  0000 C CNN
F 2 "" H 3050 2250 50  0001 C CNN
F 3 "" H 3050 2250 50  0001 C CNN
	1    3050 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 2250 3050 2250
Wire Wire Line
	3300 1850 3050 1850
$Comp
L Device:D_Schottky D1
U 1 1 5EF00E40
P 1450 5500
F 0 "D1" H 1450 5284 50  0000 C CNN
F 1 "D_Schottky" H 1450 5375 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-323" H 1450 5500 50  0001 C CNN
F 3 "~" H 1450 5500 50  0001 C CNN
	1    1450 5500
	-1   0    0    1   
$EndComp
Wire Wire Line
	750  4850 850  4850
$Comp
L Device:R R1
U 1 1 5F02E84F
P 850 5150
F 0 "R1" H 920 5196 50  0000 L CNN
F 1 "10k" H 920 5105 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 780 5150 50  0001 C CNN
F 3 "~" H 850 5150 50  0001 C CNN
	1    850  5150
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5F03AD41
P 850 5450
F 0 "#PWR04" H 850 5200 50  0001 C CNN
F 1 "GND" H 855 5277 50  0000 C CNN
F 2 "" H 850 5450 50  0001 C CNN
F 3 "" H 850 5450 50  0001 C CNN
	1    850  5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	850  5300 850  5450
Wire Wire Line
	850  4850 850  5000
Connection ~ 850  4850
Wire Wire Line
	850  4850 1000 4850
Wire Wire Line
	1450 4500 1450 4650
Wire Wire Line
	1450 5050 1450 5150
Wire Wire Line
	1450 5150 2000 5150
Wire Wire Line
	2200 4500 2200 5150
$Comp
L Device:Q_PMOS_GSD Q1
U 1 1 5EFD8503
P 1350 4850
F 0 "Q1" H 1555 4896 50  0000 L CNN
F 1 "Q_PMOS_GSD" H 1555 4805 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 1550 4950 50  0001 C CNN
F 3 "~" H 1350 4850 50  0001 C CNN
	1    1350 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 4850 1000 5500
Wire Wire Line
	1000 5500 1300 5500
Connection ~ 1000 4850
Wire Wire Line
	1000 4850 1150 4850
Wire Wire Line
	1600 5500 2000 5500
Wire Wire Line
	2000 5500 2000 5150
Connection ~ 2000 5150
Wire Wire Line
	2000 5150 2200 5150
$Comp
L Device:LED D3
U 1 1 5F0139A0
P 8350 1900
F 0 "D3" H 8343 2116 50  0000 C CNN
F 1 "LED" H 8343 2025 50  0000 C CNN
F 2 "digikey-footprints:LED_0603" H 8350 1900 50  0001 C CNN
F 3 "~" H 8350 1900 50  0001 C CNN
	1    8350 1900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR044
U 1 1 5F015824
P 9000 1900
F 0 "#PWR044" H 9000 1750 50  0001 C CNN
F 1 "+3.3V" H 9015 2073 50  0000 C CNN
F 2 "" H 9000 1900 50  0001 C CNN
F 3 "" H 9000 1900 50  0001 C CNN
	1    9000 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5F025A1B
P 8750 1900
F 0 "R13" H 8820 1946 50  0000 L CNN
F 1 "2k" H 8820 1855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8680 1900 50  0001 C CNN
F 3 "~" H 8750 1900 50  0001 C CNN
	1    8750 1900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9000 1900 8900 1900
Wire Wire Line
	8600 1900 8500 1900
Text GLabel 7250 1850 2    50   Input ~ 0
5
Text GLabel 8050 1900 0    50   Input ~ 0
26
Wire Wire Line
	8200 1900 8050 1900
Text GLabel 7250 3150 2    50   Input ~ 0
26
Text GLabel 7250 3250 2    50   Input ~ 0
27
$EndSCHEMATC
