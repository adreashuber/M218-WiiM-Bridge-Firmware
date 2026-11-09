# Software anderer Autoren in der Firmware

| Bibliothek | Lizenz | Quelle |
|---|---|---|
| IRremote (Arduino-IRremote) | MIT | https://github.com/Arduino-IRremote/Arduino-IRremote |
| WiFiManager (tzapu) | MIT | https://github.com/tzapu/WiFiManager |
| Arduino-ESP32-Core (WiFi, WebServer, HTTPClient, HTTPUpdate, Update, Preferences, ESPmDNS u. a.) | LGPL-2.1 | https://github.com/espressif/arduino-esp32 |
| ESP-IDF (inkl. mDNS-Komponente) | Apache-2.0 | https://github.com/espressif/esp-idf |
| Mbed TLS | Apache-2.0 | https://github.com/Mbed-TLS/mbedtls |

Die vollständigen Lizenztexte stehen in den verlinkten Repositories.

## Hinweis zur LGPL-2.1

Die Firmware verwendet Bibliotheken des Arduino-ESP32-Cores unter der GNU Lesser General Public License 2.1. Auf Anfrage stellen wir die Objektdateien der Firmware zur Verfügung, damit sie mit geänderten Versionen dieser Bibliotheken neu gebaut werden kann. Anfragen bitte als **Issue** in diesem Repository.

Die Stammzertifikate in der Firmware (USERTrust, Sectigo, DigiCert, ISRG) stammen aus dem öffentlichen Mozilla-Zertifikatsspeicher.
