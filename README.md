# Bodenfeuchte_ATtiny85_V2
Bodenfeuchte_ATtiny85_V2. Enabling Dallas 18B20 Sensor capability after deep-sleep.

#Hardware:
	433-TX   = D3   (DIP.2)
	BF-Power = D1   (DIP.6)
	BF_Puls  = D2   (INT0, DIP.7)
LED: replaced for SoftSerial = D4 (DIP.3)  with one TX-Port only RX is set to value 99 (Trick!) .

[SoftwareSerial for ATtiny] https://www.arduinoforum.de/arduino-Thread-Serial-Monitor-als-Debughilfe-f%C3%BCr-den-ATtiny85-und-andere-ATMEL-Mikrocontroller

[DS18B20] https://www.arduinoforum.de/arduino-Thread-ATTINY85-mit-DS18B20-Temperatursensor-und-echtem-Analogausgang