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
Text Label 5750 4100 0    50   ~ 0
SCSRX_UART
Text Label 5750 3700 0    50   ~ 0
GND
Text Label 5750 3900 0    50   ~ 0
DOOROPEN
Text Notes 3600 2550 0    50   ~ 0
adafruit QT-PY (SAMD21)\n(with Analog Comparator)\n6 USD
Wire Notes Line
	6550 4000 7750 4000
Wire Notes Line
	7750 5300 6550 5300
Text Notes 6600 4150 0    50   ~ 0
voltage divider (5V → 3.3V)
Wire Notes Line
	7800 2250 7800 3350
Wire Notes Line
	6550 3350 6550 2250
Text Notes 6600 2350 0    50   ~ 0
level shifter (sparkfun 12009)
Text Label 8300 3800 0    50   ~ 0
BTICINO_SCSRX_5V
$Comp
L power:GND #PWR0102
U 1 1 602952B7
P 8450 4050
F 0 "#PWR0102" H 8450 3800 50  0001 C CNN
F 1 "GND" H 8455 3877 50  0000 C CNN
F 2 "" H 8450 4050 50  0001 C CNN
F 3 "" H 8450 4050 50  0001 C CNN
	1    8450 4050
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSS138 Q1
U 1 1 60295968
P 7150 3050
F 0 "Q1" V 7399 3050 50  0000 C CNN
F 1 "BSS138" V 7490 3050 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7350 2975 50  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/BSS138-D.PDF" H 7150 3050 50  0001 L CNN
	1    7150 3050
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 6029945E
P 7500 2600
F 0 "#PWR0103" H 7500 2450 50  0001 C CNN
F 1 "+5V" H 7515 2773 50  0000 C CNN
F 2 "" H 7500 2600 50  0001 C CNN
F 3 "" H 7500 2600 50  0001 C CNN
	1    7500 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R1
U 1 1 6029B315
P 6750 2850
F 0 "R1" H 6818 2896 50  0000 L CNN
F 1 "10K" H 6818 2805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 6750 2850 50  0001 C CNN
F 3 "~" H 6750 2850 50  0001 C CNN
	1    6750 2850
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R4
U 1 1 6029C267
P 7500 2850
F 0 "R4" H 7568 2896 50  0000 L CNN
F 1 "10K" H 7568 2805 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 7500 2850 50  0001 C CNN
F 3 "~" H 7500 2850 50  0001 C CNN
	1    7500 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 3150 7350 3150
Wire Wire Line
	7500 2750 7500 2600
Wire Wire Line
	6750 2950 6750 3150
Wire Wire Line
	6750 3150 6950 3150
Connection ~ 6750 3150
$Comp
L power:+3V3 #PWR0104
U 1 1 6029D53E
P 6750 2600
F 0 "#PWR0104" H 6750 2450 50  0001 C CNN
F 1 "+3V3" H 6765 2773 50  0000 C CNN
F 2 "" H 6750 2600 50  0001 C CNN
F 3 "" H 6750 2600 50  0001 C CNN
	1    6750 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2600 6750 2650
Wire Wire Line
	6750 2650 7150 2650
Wire Wire Line
	7150 2650 7150 2850
Connection ~ 6750 2650
Wire Wire Line
	6750 2650 6750 2750
$Comp
L Seeeduino-XIAO:SeeeduinoXIAO U1
U 1 1 6029EAD5
P 4000 4000
F 0 "U1" H 3975 3061 50  0000 C CNN
F 1 "SeeeduinoXIAO" H 3975 2970 50  0000 C CNN
F 2 "xiao:Seeeduino XIAO-MOUDLE14P-2.54-21X17.8MM" H 3650 4200 50  0001 C CNN
F 3 "" H 3650 4200 50  0001 C CNN
	1    4000 4000
	1    0    0    -1  
$EndComp
Wire Notes Line
	2650 2250 2650 5300
Wire Notes Line
	2650 5300 5400 5300
Wire Notes Line
	5400 5300 5400 2250
Wire Notes Line
	5400 2250 2650 2250
Wire Notes Line
	8100 2250 8100 5300
Wire Notes Line
	8100 5300 9200 5300
Wire Notes Line
	9200 5300 9200 2250
Wire Notes Line
	9200 2250 8100 2250
Text Notes 8300 2450 0    50   ~ 0
bTicino 344212\n(here be soldering)
Text Label 6500 4450 2    50   ~ 0
SCSRX_3V3
$Comp
L Device:R_Small_US R3
U 1 1 602AEB15
P 7150 4450
F 0 "R3" V 6945 4450 50  0000 C CNN
F 1 "3K3" V 7036 4450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 7150 4450 50  0001 C CNN
F 3 "~" H 7150 4450 50  0001 C CNN
	1    7150 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	8250 4450 7250 4450
Wire Wire Line
	4800 4450 6800 4450
$Comp
L Device:R_Small_US R2
U 1 1 602B041B
P 6800 4750
F 0 "R2" H 6868 4796 50  0000 L CNN
F 1 "6K8" H 6868 4705 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" H 6800 4750 50  0001 C CNN
F 3 "~" H 6800 4750 50  0001 C CNN
	1    6800 4750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 602B08A0
P 6800 5050
F 0 "#PWR0105" H 6800 4800 50  0001 C CNN
F 1 "GND" H 6805 4877 50  0000 C CNN
F 2 "" H 6800 5050 50  0001 C CNN
F 3 "" H 6800 5050 50  0001 C CNN
	1    6800 5050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6800 5050 6800 4850
Wire Wire Line
	6800 4650 6800 4450
Connection ~ 6800 4450
Wire Wire Line
	6800 4450 7050 4450
Wire Wire Line
	7500 2950 7500 3150
Connection ~ 7500 3150
Wire Wire Line
	4800 3850 5100 3850
Wire Wire Line
	5100 3850 5100 2650
Wire Wire Line
	5100 2650 6750 2650
Wire Notes Line
	5600 3550 6350 3550
Wire Notes Line
	6350 3550 6350 4250
Wire Notes Line
	6350 4250 5600 4250
Wire Notes Line
	5600 4250 5600 3550
Wire Wire Line
	5750 4300 5750 4100
Wire Wire Line
	4800 4300 5750 4300
Wire Wire Line
	5750 3900 6750 3900
Wire Wire Line
	6750 3900 6750 3700
Wire Wire Line
	4800 3700 5750 3700
Wire Notes Line
	6550 3350 7800 3350
Wire Notes Line
	6550 2250 7800 2250
Text Notes 5600 3500 0    50   ~ 0
external GPIO output
Wire Notes Line
	7750 4000 7750 5300
Wire Notes Line
	6550 4000 6550 5300
Text Notes 4250 5200 0    50   ~ 0
TODO: diode (welche?!)
Text Label 4900 3550 0    50   ~ 0
+5V
Wire Wire Line
	4900 3550 4800 3550
Text Label 8450 3900 0    50   ~ 0
GND
$Comp
L Connector:Conn_01x05_Male J1
U 1 1 602D7984
P 9500 3700
F 0 "J1" H 9472 3632 50  0000 R CNN
F 1 "Conn_01x05_Male" H 9472 3723 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Horizontal" H 9500 3700 50  0001 C CNN
F 3 "~" H 9500 3700 50  0001 C CNN
	1    9500 3700
	-1   0    0    1   
$EndComp
Wire Wire Line
	8250 3150 7500 3150
Wire Wire Line
	9300 3900 8450 3900
Wire Wire Line
	8450 4050 8450 3900
Wire Wire Line
	8250 4450 8250 3800
Wire Wire Line
	8250 3800 9300 3800
Text Label 8300 3600 0    50   ~ 0
BTICINO_DOOROPEN
Wire Wire Line
	8250 3600 9300 3600
Wire Wire Line
	8750 3500 8750 3350
Wire Wire Line
	8750 3500 9300 3500
Text Label 8750 3500 0    50   ~ 0
+5V
$Comp
L power:+5V #PWR0101
U 1 1 60294BDA
P 8750 3350
F 0 "#PWR0101" H 8750 3200 50  0001 C CNN
F 1 "+5V" H 8765 3523 50  0000 C CNN
F 2 "" H 8750 3350 50  0001 C CNN
F 3 "" H 8750 3350 50  0001 C CNN
	1    8750 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3150 8250 3600
Wire Wire Line
	9300 3700 6750 3700
Connection ~ 6750 3700
Wire Wire Line
	6750 3700 6750 3150
$EndSCHEMATC