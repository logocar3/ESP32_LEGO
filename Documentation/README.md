# Opozorila

 * Nikoli ne imeti povezanega ESP32 na USB in 5V hkrati
 * Ni opozoril o prazni bateriji - pazi da boš imel zadosti polno baterijo!
 * Boljše vprašat kot pa cele dneve razbijat če kaj ne dela
 * Ko boste iskali za informacijami za programiranje, lahko iščete tudi pod gesli ESP8266, Arduino oz. C++
 * Za razhroščevanje uporabite Serial.write / writeln.
 * Naloži driver za USB! 
 * Naloži GIT
 * SD kartica potrebuje 5V in ne 3.3V

# Plan dela:
Prvi zagovor: prvi teden decembra

 1. Uspešna komunikacija do SD kartice preko SPI
 1. Cilj do 22.11.2018: Izvesti 5s eksperiment, kjer z 50 Hz vzorčimo jakost odbite svetlobe in meritve shranjujemo v datoteko - po koncu eksperimenta vzpostavimo http strežnik, na katerega se iz PC povežemo in naložimo datoteko meritev. To datoteko nato preberemo v MATLABu in izrišemo graf vrednosti senzorja v odvisnosti od časa. 
	 1. Prenos datoteke ESP32 -> računalnik preko Wifi. ESP32 uporabi kot http strežnik.
	 1. Branje barvnega senzorja ter zapisovanje meritev v .csv datoteko (vrednosti ločene z vejico, vzorci s entrom)
	 2. Odpiranje datoteke v matlabu
 3. Cilj do 6.12.2018: Dokončati kar ni bilo narejeno do sedaj PLUS:
	 1. Priprava predstavitve (shema povezav, demonstracija postopka, opis uporabljenih protokolov)
 3. Krmiljenje motorja 
 4. Povezava vseh komponent
 5. Regulacija motorja 

# Examples

 * Dobri primeri na: https://techtutorialsx.com/ na temo esp32
 * Shranjevanje na SD kartico https://randomnerdtutorials.com/esp32-data-logging-temperature-to-microsd-card/
 * Primer prenosa datoteke na https://github.com/G6EJD/ESP32-8266-File-Download
 * Komunikacija z barvnim senzorjem: https://lejosnews.wordpress.com/2014/05/31/arduino-library-for-ev3-uart-sensors/
 * Driver za ESP32 https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers
 * Navodila za instalacijo GIT http://codetunnel.com/installing-git-on-windows/
 * Protokol za komunikacijo z LEGO senzorji (UART): https://sourceforge.net/p/lejos/wiki/UART%20Sensor%20Protocol/