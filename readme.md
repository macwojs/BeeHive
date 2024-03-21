# Hive smart tracker - Inteligentny sensor dla ula

Initialy based on https://github.com/Christophe45/ESP32-Lorawan

## Wstęp
Obecnie projekt jest w fazie rozwojowej. Obecnie rozwijana jest wersja oparta na komunikacji przez LoraWAN (BeeHiveNodeLora) i bramkę sieci komunikującą się z serwerami TTN.

## Środowisko
Kompilacja projektu testowana była w środowisku Adruino IDE 2.x. Również dla tego środowiska podane sa nazwy bibliotek. 

## Układ
Cały projekt oparty jest na układzie ESP32.

W celu dodania odpowidniego układu do Arduino IDE należy dodać repozytorium modułów (https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json), a następnie wybrać moduł `ESP32 Dev Module`.


## Biblioteki
- HX711 -  HX711 Arduino Library by Bogdan w wersji 0.7.2
- lmic - MCCI LoRaWAN LMIC library by IBM w wersji 2.3.2
- Adafruit_Sensor - Adafruit Unified Sensor w wersji 1.0.2
- Adafruit_BME280 - Adafruit BME280 Library w wersji 1.0.8


## Problemy
Biblioteka do komunikacji przez Lora (lmic) wykorzystuje funkcję o nazwie hal_init(), która pokrywa sie nazwą z domyślną funkcją dla modułu ESP. W związku z tym do plików konfiguracyjnych biblioteki (`lmic_project_config.h`) należy dodać linię:


```#define hal_init LMICHAL_init```


Pliki konfiguracyjne powinny znajdować się w katalogu Arduino: 
`Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h`. 

Dla systemu linux będzie to: 
`~/Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h`