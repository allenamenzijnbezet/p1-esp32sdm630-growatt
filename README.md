# p1-esp32sdm630-growatt
ESP32 P1-to-SDM630 Emulator for Growatt Inverters

I tried to emulate an Eastron SDM630 meter for power export control of my Growatt MOD 4000 TL3-XH BP 3-phase PV-inverter.
My design uses an ESP32S3 devkit-c with a direct connection with my Dutch P1 DSMR5 meter. The ESP32 is connected to the Growatt via a generic MAX485 TTL to RS485 converter.
The ESP32S3 takes the P1 measurements and the code converts it to modbus messages for the Growatt following the SDM630 modbus spec.

It failed.

Well, essentially it works, but the P1 meter has an 1 second update rate. The Growatt inverter will poll the SDM630 8-10 times per second. This leads to an oscillation in the export control of the Growatt inverter. The Growatt inverter is very picky on the update rate of the SDM630 it seems. Responding once or twice every seconds leads to the 401 error on the display (Meter abnormal).
I tested with an Exponential Moving Average (EMA) filter but I could not counteract this behaviour. So I gave up.

My solution with Home Assistant and setting the export limit (modbus id 03) to a calculated value works, but the responds is > 1 < 5 seconds.

Some settings I used:
Modbus 02 set to 0 (use ram instead of flash memory)
Modbus 122 set to 1 (Enable 485 Export limit)
Modbus 123 set to 1-100 (Choose your export limit)
Modbus 533 set to 1 (Limit export with meter)
