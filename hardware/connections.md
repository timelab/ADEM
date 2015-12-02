# Connecting to the SparkFun ESP8266 Thing

ESP Thing	| External connection
---------------	| -------------------
GND		| PPD42NS pin 1
GND		| GPS GND
GND		| FTDI GND (only for programming and debugging)
3V3		| PPD42NS Pin 3 => 5V
3V3		| GPS VCC
12		| PPD42NS pin 2
13		| PPD42NS pin 2
RXI		| GPS TX (no need to disconnect while programming)
RXI		| FTDI TXO (only for programming and debugging)
TXO		| FTDI RXI (only for programming and debugging)

PPD42NS Pin	| Description
--------------- | -----------
1		| GND
2		| Output P2
3		| Vcc (5V, 90mA)
4		| Output P1
5		| Input (T1), threshold for P2

PPD60PV-TZ CN1	| Description
---------------	| -----------
1		| GND
2		| Analog Output (0-3.5V)
3		| Vin: 5V
4		| Digital Output (LoPulseOccaupance Time) 


PPD60PV-TZ CN2	| Description
---------------	| -----------
1		| GND
2		| NC
3		| Heater (36ohm) 5V DC/AC input, 140mA 

