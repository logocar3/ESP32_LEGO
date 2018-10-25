# Opozorila

 * Nikoli ne imeti poveyanega ESP32 na USB in 5V hkrati
 * Ni opozoril o prazni bateriji - pazi da boš imel zadosti polno baterijo!
 * Boljše vprašat kot pa cele dneve razbijat če kaj ne dela
 * Ko boste iskali za informacijami za programiranje, lahko iščete tudi pod gesli ESP8266, Arduino oz. C++
 * Za razhroščevanje uporabite Serial.write / writeln.

# Plan dela:

 1. Uspešna komunikacija do SD kartice preko SPI
 1. Prenos datoteke ESP32 -> računalnik preko Wifi. ESP32 uporabi kot http strežnik.
 2. Branje barvnega senzorja
 3. Krmiljenje motorja 
 4. Povezava vseh komponent
 5. Regulacija motorja 

# Examples

 * Dobri primeri na https://techtutorialsx.com/ na temo esp32
 * Shranjevanje na SD kartico https://randomnerdtutorials.com/esp32-data-logging-temperature-to-microsd-card/
 * Primer prenosa datoteke na https://github.com/G6EJD/ESP32-8266-File-Download
 * Komunikacija z barvnim senzorjem: https://lejosnews.wordpress.com/2014/05/31/arduino-library-for-ev3-uart-sensors/
