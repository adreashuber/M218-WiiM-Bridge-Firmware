/*
 * M218 -> WiiM Bridge  |  Version 3.12: Raumname, WiiM-Suche, Taster, Multiroom, Update ueber GitHub
 * -------------------------------------------------------------------------
 * Hardware : Platine v3.0 (Hammond 1593K) bzw. THT v2.x, ESP32-DevKit 30 Pin (USB-C, CP2102)
 *            IR-Out der M218 ueber Spannungsteiler 15k/30k (v3.2, aeltere Platinen 10k/18k) an D27 (GPIO27), Status-LED an D2.
 *            Mit DevKit-Aufbau (Draht an D15) stattdessen IR_PIN = 15 setzen.
 * Player   : WiiM Amp Ultra (HTTP-API), weitere WiiM/LinkPlay-Geraete ebenso
 * Libraries: - WiFiManager (tzapu)
 *            - IRremote    (Armin Joachimsmeyer, >= 4.x)
 *
 * Mehrere Raeume:
 *   - Jede Bridge bekommt einen Raumnamen, z.B. "Bad". Daraus werden
 *       Einstellungsseite  http://m218-bad.local
 *       Setup-WLAN         "M218-Bad-Setup"   (Passwort m218wiim)
 *   - Neue Bridge ohne Raumname: "M218-WiiM-Bridge-A1B2C3" bzw. m218-a1b2c3.local
 *     (A1B2C3 = letzte 6 Stellen der MAC-Adresse = Geraete-ID, steht auch auf der Einstellungsseite)
 *   - Der WiiM wird ueber seinen Namen aus der WiiM-App gewaehlt (Liste "WiiM suchen").
 *     Die Bridge sucht ihn bei jedem Start und wenn er nicht mehr antwortet neu im Netz,
 *     eine geaenderte IP-Adresse stoert also nicht. Ohne Namen gilt die eingetragene IP.
 *
 * Firmware-Update: Einstellungsseite -> Firmware -> "Nach Update suchen" / "Update installieren"
 *   (Releases im oeffentlichen Repository GITHUB_OWNER/GITHUB_REPO, Tag "v3.12" + angehaengte .bin-Datei;
 *   der Quellcode liegt im privaten Repository M218-WiiM-Bridge).
 *   Notloesung ohne Internet: http://<bridge>/update  (.bin-Datei hochladen).
 *   Arduino-IDE: Werkzeuge -> Partition Scheme -> "Minimal SPIFFS (1.9MB APP with OTA)".
 *
 * Uebersicht aller Bridges: http://m218.local  (Liste mit Raumnamen, antippen = Einstellungen)
 *   Jede Bridge zeigt die Liste auch unter ihrer eigenen Adresse (http://m218-<raum>.local/).
 *
 * Einstellungsseite "M218 Tasten":
 *   - Im laufenden Betrieb:  http://m218-<raum>.local  oder  http://<ESP32-IP>
 *   - Im Konfigurationsportal (Setup-WLAN): Menuepunkt "M218 Tasten"
 *   Dort einstellbar: Raumname, WiiM (Liste oder IP), Aktion je Quellentaste,
 *   Lautstaerke-Schritt, Max. Lautstaerke. Jede Quellentaste hat einen "Test"-Knopf.
 *
 * Multiroom: Quellentaste 2 s halten -> dieser Raum wird Master, alle WiiMs, die gerade spielen,
 *   treten der Gruppe bei. Off: Master loest die Gruppe auf, Slave verlaesst sie.
 *   Andere Quelle im Slave-Raum: verlaesst zuerst die Gruppe.
 * Off 2 s halten -> alle WiiMs im Netz aus (Gruppen aufloesen, stoppen).
 *
 * Feste Bedientasten (RC5, Adresse egal):
 *   0x21 Skip previous | 0x20 Skip next | 0x11 Vol - | 0x10 Vol + | 0x0C Off (Stop)
 *
 * Taster an der Front (D13 -> GND, Platine v3.1) oder BOOT-Taste am DevKit:
 *   LED: im Betrieb dunkel, ohne WLAN kurzer Blitz alle 2 s, Portal offen = Dauerlicht
 *   Taster druecken: LED blinkt langsam, ab 2 s mittel, ab 7 s schnell
 *   2-10 s halten und loslassen  -> Setup-Portal
 *   10 s halten                  -> Werksreset: WLAN, Raum, WiiM, Tasten loeschen, Neustart
 *   Beim Einstecken gedrueckt (nur Front-Taster): wie oben, 10 s = Werksreset, kuerzer = Portal
 *   Werksreset geht auch auf der Einstellungsseite (Knopf ganz unten).
 * Seriell (115200): 't' = WiiM-Status, 's' = WiiMs suchen, 'p' = Portal
 */

#define IR_INPUT_IS_ACTIVE_HIGH   // M218 IR-Out: Ruhe = LOW, Pulse = HIGH (per Oszi bestaetigt)
#define DECODE_RC5                // nur RC5 dekodieren

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <IRremote.hpp>
#include <esp_mac.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include <esp_ota_ops.h>

// =====================================================================
//  Firmware-Version und Update-Quelle (GitHub-Releases)
//  Neue Version: FW_VERSION erhoehen, kompilieren, .bin als Release mit Tag "v<Version>" hochladen.
// =====================================================================
const char* FW_VERSION   = "3.12";
const char* GITHUB_OWNER = "adreashuber";
const char* GITHUB_REPO  = "M218-WiiM-Bridge-Firmware";   // oeffentliches Repository nur mit Releases (.bin)
extern "C" bool verifyRollbackLater();   // Rueckfall: neue Firmware erst nach WLAN-Verbindung bestaetigen

// Gefundener WiiM (steht hier oben, weil die Arduino-IDE Funktions-Prototypen
// mit diesem Typ an den Anfang des Sketches setzt)
struct WiimInfo { char name[40]; char ip[16]; };
const int MAX_FOUND = 8;
// Kandidaten-Liste der Suche (IP-Adressen, ohne Doppelte)
const int MAX_CAND = 16;
struct CandList { String ip[MAX_CAND]; int n = 0; };
// Andere M218-Bridges im Netz (fuer die Uebersicht unter http://m218.local)
struct BridgeInfo { char room[24]; char host[40]; char ip[16]; char wiim[40]; char id[8]; };

// =====================================================================
//  Feste Einstellungen
// =====================================================================
const uint8_t  IR_PIN           = 27;          // D27 auf dem DevKit
const uint8_t  STATUS_LED_PIN   = 2;           // D2: Front-LED (5 mm, 82 Ohm) + blaue LED des DevKits
const uint8_t  CFG_BUTTON_PIN   = 0;           // BOOT-Taste am DevKit (nur im Betrieb nutzbar)
const uint8_t  SETUP_BUTTON_PIN = 13;          // Taster an der Front: D13 -> GND (interner Pull-up)
const uint32_t PORTAL_HOLD_MS   = 2000;        // so lange halten -> Portal
const uint32_t WARN_HOLD_MS     = 7000;        // ab hier blinkt die LED schnell (Warnung)
const uint32_t RESET_HOLD_MS    = 10000;       // so lange halten -> Werksreset
const uint32_t GROUP_HOLD_MS    = 2000;        // Quellentaste der M218 so lange halten -> Multiroom-Master
const uint32_t IR_RELEASE_MS    = 250;         // so lange keine Wiederholung -> Taste losgelassen
const char*    AP_PASSWORD      = "m218wiim";
const uint16_t PORTAL_TIMEOUT_S = 300;
const uint32_t VOL_RESYNC_MS    = 5000;
const uint32_t RESOLVE_PAUSE_MS = 30000;       // fruehestens so oft neu nach dem WiiM suchen
const char*    OFF_COMMAND      = "setPlayerCmd:stop";
const char*    TOGGLE_COMMAND   = "setPlayerCmd:onepause";
const uint32_t OTA_CHECK_MS     = 24UL * 60UL * 60UL * 1000UL;   // taeglich nach Updates fragen

// Stammzertifikate fuer api.github.com und die GitHub-Downloadserver (aus dem Mozilla-Zertifikatsspeicher)
static const char GITHUB_ROOT_CA[] = R"CA(
// USERTrust ECC Certification Authority
-----BEGIN CERTIFICATE-----
MIICjzCCAhWgAwIBAgIQXIuZxVqUxdJxVt7NiYDMJjAKBggqhkjOPQQDAzCBiDEL
MAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNl
eSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMT
JVVTRVJUcnVzdCBFQ0MgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAwMjAx
MDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNVBAgT
Ck5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVUaGUg
VVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBFQ0MgQ2VydGlm
aWNhdGlvbiBBdXRob3JpdHkwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAQarFRaqflo
I+d61SRvU8Za2EurxtW20eZzca7dnNYMYf3boIkDuAUU7FfO7l0/4iGzzvfUinng
o4N+LZfQYcTxmdwlkWOrfzCjtHDix6EznPO/LlxTsV+zfTJ/ijTjeXmjQjBAMB0G
A1UdDgQWBBQ64QmG1M8ZwpZ2dEl23OA1xmNjmjAOBgNVHQ8BAf8EBAMCAQYwDwYD
VR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAwNoADBlAjA2Z6EWCNzklwBBHU6+4WMB
zzuqQhFkoJ2UOQIReVx7Hfpkue4WQrO/isIJxOzksU0CMQDpKmFHjFJKS04YcPbW
RNZu9YO6bVi9JNlWSOrvxKJGgYhqOkbRqZtNyWHa0V1Xahg=
-----END CERTIFICATE-----
// USERTrust RSA Certification Authority
-----BEGIN CERTIFICATE-----
MIIF3jCCA8agAwIBAgIQAf1tMPyjylGoG7xkDjUDLTANBgkqhkiG9w0BAQwFADCB
iDELMAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0pl
cnNleSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNV
BAMTJVVTRVJUcnVzdCBSU0EgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAw
MjAxMDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNV
BAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVU
aGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBSU0EgQ2Vy
dGlmaWNhdGlvbiBBdXRob3JpdHkwggIiMA0GCSqGSIb3DQEBAQUAA4ICDwAwggIK
AoICAQCAEmUXNg7D2wiz0KxXDXbtzSfTTK1Qg2HiqiBNCS1kCdzOiZ/MPans9s/B
3PHTsdZ7NygRK0faOca8Ohm0X6a9fZ2jY0K2dvKpOyuR+OJv0OwWIJAJPuLodMkY
tJHUYmTbf6MG8YgYapAiPLz+E/CHFHv25B+O1ORRxhFnRghRy4YUVD+8M/5+bJz/
Fp0YvVGONaanZshyZ9shZrHUm3gDwFA66Mzw3LyeTP6vBZY1H1dat//O+T23LLb2
VN3I5xI6Ta5MirdcmrS3ID3KfyI0rn47aGYBROcBTkZTmzNg95S+UzeQc0PzMsNT
79uq/nROacdrjGCT3sTHDN/hMq7MkztReJVni+49Vv4M0GkPGw/zJSZrM233bkf6
c0Plfg6lZrEpfDKEY1WJxA3Bk1QwGROs0303p+tdOmw1XNtB1xLaqUkL39iAigmT
Yo61Zs8liM2EuLE/pDkP2QKe6xJMlXzzawWpXhaDzLhn4ugTncxbgtNMs+1b/97l
c6wjOy0AvzVVdAlJ2ElYGn+SNuZRkg7zJn0cTRe8yexDJtC/QV9AqURE9JnnV4ee
UB9XVKg+/XRjL7FQZQnmWEIuQxpMtPAlR1n6BB6T1CZGSlCBst6+eLf8ZxXhyVeE
Hg9j1uliutZfVS7qXMYoCAQlObgOK6nyTJccBz8NUvXt7y+CDwIDAQABo0IwQDAd
BgNVHQ4EFgQUU3m/WqorSs9UgOHYm8Cd8rIDZsswDgYDVR0PAQH/BAQDAgEGMA8G
A1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAFzUfA3P9wF9QZllDHPF
Up/L+M+ZBn8b2kMVn54CVVeWFPFSPCeHlCjtHzoBN6J2/FNQwISbxmtOuowhT6KO
VWKR82kV2LyI48SqC/3vqOlLVSoGIG1VeCkZ7l8wXEskEVX/JJpuXior7gtNn3/3
ATiUFJVDBwn7YKnuHKsSjKCaXqeYalltiz8I+8jRRa8YFWSQEg9zKC7F4iRO/Fjs
8PRF/iKz6y+O0tlFYQXBl2+odnKPi4w2r78NBc5xjeambx9spnFixdjQg3IM8WcR
iQycE0xyNN+81XHfqnHd4blsjDwSXWXavVcStkNr/+XeTWYRUc+ZruwXtuhxkYze
Sf7dNXGiFSeUHM9h4ya7b6NnJSFd5t0dCy5oGzuCr+yDZ4XUmFF0sbmZgIn/f3gZ
XHlKYC6SQK5MNyosycdiyA5d9zZbyuAlJQG03RoHnHcAP9Dc1ew91Pq7P8yF1m9/
qS3fuQL39ZeatTXaw2ewh0qpKJ4jjv9cJ2vhsE/zB+4ALtRZh8tSQZXq9EfX7mRB
VXyNWQKV3WKdwrnuWih0hKWbt5DHDAff9Yk2dDLWKMGwsAvgnEzDHNb842m1R0aB
L6KCq9NjRHDEjf8tM7qtj3u1cIiuPhnPQCjY/MiQu12ZIvVS5ljFH4gxQ+6IHdfG
jjxDah2nGN59PRbxYvnKkKj9
-----END CERTIFICATE-----
// Sectigo Public Server Authentication Root E46
-----BEGIN CERTIFICATE-----
MIICOjCCAcGgAwIBAgIQQvLM2htpN0RfFf51KBC49DAKBggqhkjOPQQDAzBfMQsw
CQYDVQQGEwJHQjEYMBYGA1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1T
ZWN0aWdvIFB1YmxpYyBTZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBFNDYwHhcN
MjEwMzIyMDAwMDAwWhcNNDYwMzIxMjM1OTU5WjBfMQswCQYDVQQGEwJHQjEYMBYG
A1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1TZWN0aWdvIFB1YmxpYyBT
ZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBFNDYwdjAQBgcqhkjOPQIBBgUrgQQA
IgNiAAR2+pmpbiDt+dd34wc7qNs9Xzjoq1WmVk/WSOrsfy2qw7LFeeyZYX8QeccC
WvkEN/U0NSt3zn8gj1KjAIns1aeibVvjS5KToID1AZTc8GgHHs3u/iVStSBDHBv+
6xnOQ6OjQjBAMB0GA1UdDgQWBBTRItpMWfFLXyY4qp3W7usNw/upYTAOBgNVHQ8B
Af8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAwNnADBkAjAn7qRa
qCG76UeXlImldCBteU/IvZNeWBj7LRoAasm4PdCkT0RHlAFWovgzJQxC36oCMB3q
4S6ILuH5px0CMk7yn2xVdOOurvulGu7t0vzCAxHrRVxgED1cf5kDW21USAGKcw==
-----END CERTIFICATE-----
// Sectigo Public Server Authentication Root R46
-----BEGIN CERTIFICATE-----
MIIFijCCA3KgAwIBAgIQdY39i658BwD6qSWn4cetFDANBgkqhkiG9w0BAQwFADBf
MQswCQYDVQQGEwJHQjEYMBYGA1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQD
Ey1TZWN0aWdvIFB1YmxpYyBTZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBSNDYw
HhcNMjEwMzIyMDAwMDAwWhcNNDYwMzIxMjM1OTU5WjBfMQswCQYDVQQGEwJHQjEY
MBYGA1UEChMPU2VjdGlnbyBMaW1pdGVkMTYwNAYDVQQDEy1TZWN0aWdvIFB1Ymxp
YyBTZXJ2ZXIgQXV0aGVudGljYXRpb24gUm9vdCBSNDYwggIiMA0GCSqGSIb3DQEB
AQUAA4ICDwAwggIKAoICAQCTvtU2UnXYASOgHEdCSe5jtrch/cSV1UgrJnwUUxDa
ef0rty2k1Cz66jLdScK5vQ9IPXtamFSvnl0xdE8H/FAh3aTPaE8bEmNtJZlMKpnz
SDBh+oF8HqcIStw+KxwfGExxqjWMrfhu6DtK2eWUAtaJhBOqbchPM8xQljeSM9xf
iOefVNlI8JhD1mb9nxc4Q8UBUQvX4yMPFF1bFOdLvt30yNoDN9HWOaEhUTCDsG3X
ME6WW5HwcCSrv0WBZEMNvSE6Lzzpng3LILVCJ8zab5vuZDCQOc2TZYEhMbUjUDM3
IuM47fgxMMxF/mL50V0yeUKH32rMVhlATc6qu/m1dkmU8Sf4kaWD5QazYw6A3OAS
VYCmO2a0OYctyPDQ0RTp5A1NDvZdV3LFOxxHVp3i1fuBYYzMTYCQNFu31xR13NgE
SJ/AwSiItOkcyqex8Va3e0lMWeUgFaiEAin6OJRpmkkGj80feRQXEgyDet4fsZfu
+Zd4KKTIRJLpfSYFplhym3kT2BFfrsU4YjRosoYwjviQYZ4ybPUHNs2iTG7sijbt
8uaZFURww3y8nDnAtOFr94MlI1fZEoDlSfB1D++N6xybVCi0ITz8fAr/73trdf+L
HaAZBav6+CuBQug4urv7qv094PPK306Xlynt8xhW6aWWrL3DkJiy4Pmi1KZHQ3xt
zwIDAQABo0IwQDAdBgNVHQ4EFgQUVnNYZJX5khqwEioEYnmhQBWIIUkwDgYDVR0P
AQH/BAQDAgGGMA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQEMBQADggIBAC9c
mTz8Bl6MlC5w6tIyMY208FHVvArzZJ8HXtXBc2hkeqK5Duj5XYUtqDdFqij0lgVQ
YKlJfp/imTYpE0RHap1VIDzYm/EDMrraQKFz6oOht0SmDpkBm+S8f74TlH7Kph52
gDY9hAaLMyZlbcp+nv4fjFg4exqDsQ+8FxG75gbMY/qB8oFM2gsQa6H61SilzwZA
Fv97fRheORKkU55+MkIQpiGRqRxOF3yEvJ+M0ejf5lG5Nkc/kLnHvALcWxxPDkjB
JYOcCj+esQMzEhonrPcibCTRAUH4WAP+JWgiH5paPHxsnnVI84HxZmduTILA7rpX
DhjvLpr3Etiga+kFpaHpaPi8TD8SHkXoUsCjvxInebnMMTzD9joiFgOgyY9mpFui
TdaBJQbpdqQACj7LzTWb4OE4y2BThihCQRxEV+ioratF4yUQvNs+ZUH7G6aXD+u5
dHn5HrwdVw1Hr8Mvn4dGp+smWg9WY7ViYG4A++MnESLn/pmPNPW56MORcr3Ywx65
LvKRRFHQV80MNNVIIb/bE/FmJUNS0nAiNs2fxBx1IK1jcmMGDw4nztJqDby1ORrp
0XZ60Vzk50lJLVU3aPAaOpg+VBeHVOmmJ1CJeyAvP/+/oYtKR5j/K3tJPsMpRmAY
QqszKbrAKbkTidOIijlBO8n9pu0f9GBj39ItVQGL
-----END CERTIFICATE-----
// DigiCert Global Root G2
-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----
// DigiCert Global Root G3
-----BEGIN CERTIFICATE-----
MIICPzCCAcWgAwIBAgIQBVVWvPJepDU1w6QP1atFcjAKBggqhkjOPQQDAzBhMQsw
CQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cu
ZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBHMzAe
Fw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVTMRUw
EwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5jb20x
IDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEczMHYwEAYHKoZIzj0CAQYF
K4EEACIDYgAE3afZu4q4C/sLfyHS8L6+c/MzXRq8NOrexpu80JX28MzQC7phW1FG
fp4tn+6OYwwX7Adw9c+ELkCDnOg/QW07rdOkFFk2eJ0DQ+4QE2xy3q6Ip6FrtUPO
Z9wj/wMco+I+o0IwQDAPBgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAd
BgNVHQ4EFgQUs9tIpPmhxdiuNkHMEWNpYim8S8YwCgYIKoZIzj0EAwMDaAAwZQIx
AK288mw/EkrRLTnDCgmXc/SINoyIJ7vmiI1Qhadj+Z4y3maTD/HMsQmP3Wyr+mt/
oAIwOWZbwmSNuJ5Q3KjVSaLtx9zRSX8XAbjIho9OjIgrqJqpisXRAL34VOKa5Vt8
sycX
-----END CERTIFICATE-----
// ISRG Root X1
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
// ISRG Root X2
-----BEGIN CERTIFICATE-----
MIICGzCCAaGgAwIBAgIQQdKd0XLq7qeAwSxs6S+HUjAKBggqhkjOPQQDAzBPMQsw
CQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJuZXQgU2VjdXJpdHkgUmVzZWFyY2gg
R3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBYMjAeFw0yMDA5MDQwMDAwMDBaFw00
MDA5MTcxNjAwMDBaME8xCzAJBgNVBAYTAlVTMSkwJwYDVQQKEyBJbnRlcm5ldCBT
ZWN1cml0eSBSZXNlYXJjaCBHcm91cDEVMBMGA1UEAxMMSVNSRyBSb290IFgyMHYw
EAYHKoZIzj0CAQYFK4EEACIDYgAEzZvVn4CDCuwJSvMWSj5cz3es3mcFDR0HttwW
+1qLFNvicWDEukWVEYmO6gbf9yoWHKS5xcUy4APgHoIYOIvXRdgKam7mAHf7AlF9
ItgKbppbd9/w+kHsOdx1ymgHDB/qo0IwQDAOBgNVHQ8BAf8EBAMCAQYwDwYDVR0T
AQH/BAUwAwEB/zAdBgNVHQ4EFgQUfEKWrt5LSDv6kviejM9ti6lyN5UwCgYIKoZI
zj0EAwMDaAAwZQIwe3lORlCEwkSHRhtFcP9Ymd70/aTSVaYgLXTWNLxBo1BfASdW
tL4ndQavEi51mI38AjEAi/V3bNTIZargCyzuFJ0nN6T5U6VR5CmD1/iQMVtCnwr1
/q4AaOeMSQ+2b1tbFfLn
-----END CERTIFICATE-----
)CA";

// RC5-Commands der M218
const uint8_t RC5_SOURCE   = 0x3F;
const uint8_t RC5_PREV     = 0x21;
const uint8_t RC5_NEXT     = 0x20;
const uint8_t RC5_VOL_DOWN = 0x11;
const uint8_t RC5_VOL_UP   = 0x10;
const uint8_t RC5_OFF      = 0x0C;

// =====================================================================
//  Auswaehlbare Aktionen (Dropdown)
// =====================================================================
struct ActionDef {
  const char* id;        // wird gespeichert
  const char* label;     // Anzeige im Dropdown
  const char* command;   // WiiM-API-Befehl
  bool        toggle;    // gleiche Taste nochmals = Play/Pause
};

const ActionDef ACTIONS[] = {
  { "P1",   "Preset 1",          "MCUKeyShortClick:1",                true  },
  { "P2",   "Preset 2",          "MCUKeyShortClick:2",                true  },
  { "P3",   "Preset 3",          "MCUKeyShortClick:3",                true  },
  { "P4",   "Preset 4",          "MCUKeyShortClick:4",                true  },
  { "P5",   "Preset 5",          "MCUKeyShortClick:5",                true  },
  { "P6",   "Preset 6",          "MCUKeyShortClick:6",                true  },
  { "P7",   "Preset 7",          "MCUKeyShortClick:7",                true  },
  { "P8",   "Preset 8",          "MCUKeyShortClick:8",                true  },
  { "P9",   "Preset 9",          "MCUKeyShortClick:9",                true  },
  { "P10",  "Preset 10",         "MCUKeyShortClick:10",               true  },
  { "LINE", "Eingang Line-In",   "setPlayerCmd:switchmode:line-in",   false },
  { "OPT",  "Eingang Optical",   "setPlayerCmd:switchmode:optical",   false },
  { "COAX", "Eingang Koaxial",   "setPlayerCmd:switchmode:co-axial",  false },
  { "HDMI", "Eingang HDMI (TV)", "setPlayerCmd:switchmode:HDMI",      false },
  { "BT",   "Bluetooth",         "setPlayerCmd:switchmode:bluetooth", false },
  { "WIFI", "Streaming (WLAN)",  "setPlayerCmd:switchmode:wifi",      true  },
  { "NONE", "keine Funktion",    "",                                  false },
};
const int ACTION_COUNT = sizeof(ACTIONS) / sizeof(ACTIONS[0]);

int findAction(const String& id) {
  for (int i = 0; i < ACTION_COUNT; i++) if (id == ACTIONS[i].id) return i;
  return ACTION_COUNT - 1;   // NONE
}

// =====================================================================
//  Quellentasten der M218 (Command 0x3F, Adresse = Taste)
// =====================================================================
struct SourceKey {
  uint8_t     address;
  const char* name;
  const char* prefKey;
  const char* defaultAction;
};

const SourceKey KEYS[] = {
  { 0x11, "Tuner",  "k_tuner",  "P1"   },
  { 0x14, "CD",     "k_cd",     "P2"   },
  { 0x07, "Server", "k_server", "P3"   },
  { 0x13, "Local",  "k_local",  "LINE" },
  { 0x15, "Aux",    "k_aux",    "P4"   },
};
const int KEY_COUNT = sizeof(KEYS) / sizeof(KEYS[0]);

// =====================================================================
//  Einstellungen (im Flash gespeichert)
// =====================================================================
Preferences prefs;
char roomName[24] = "";         // z.B. "Bad"  (leer = kein Raum)
char wiimName[40] = "";         // Name des WiiM wie in der WiiM-App (leer = nur IP)
char wiimIp[16]   = "";         // feste IP bzw. zuletzt gefundene IP
int  volStep      = 2;
int  maxVolume    = 70;
int  keyAction[KEY_COUNT];      // Index in ACTIONS je Taste

String hostName = "m218";       // mDNS-/Router-Name, aus dem Raumnamen
String apName   = "M218-WiiM-Setup";

// Raumname -> "bad", "wohnzimmer", "kinderzimmer-2" (Umlaute werden ae/oe/ue/ss)
String makeSlug(const char* s, bool lower) {
  String o;
  for (const unsigned char* p = (const unsigned char*)s; *p; p++) {
    unsigned char c = *p;
    if (c == 0xC3 && p[1]) {                       // UTF-8 Umlaute
      unsigned char d = *++p;
      const char* r = "";
      if      (d == 0xA4) r = "ae"; else if (d == 0x84) r = lower ? "ae" : "Ae";
      else if (d == 0xB6) r = "oe"; else if (d == 0x96) r = lower ? "oe" : "Oe";
      else if (d == 0xBC) r = "ue"; else if (d == 0x9C) r = lower ? "ue" : "Ue";
      else if (d == 0x9F) r = "ss";
      o += r;
    } else if (isalnum(c)) {
      o += (char)(lower ? tolower(c) : c);
    } else if (o.length() && !o.endsWith("-")) {
      o += '-';
    }
  }
  while (o.endsWith("-")) o.remove(o.length() - 1);
  if (o.length() > 20) o.remove(20);
  return o;
}

// Geraete-Kennung: letzte 6 Stellen der MAC-Adresse, z.B. "A1B2C3" (steht auch auf dem Aufkleber)
String deviceId() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char buf[7];
  snprintf(buf, sizeof(buf), "%02X%02X%02X", mac[3], mac[4], mac[5]);
  return String(buf);
}

void applyRoomName() {
  String lowerSlug = makeSlug(roomName, true);
  String caseSlug  = makeSlug(roomName, false);
  String id = deviceId();
  String idLower = id;
  idLower.toLowerCase();
  // Ohne Raumname: Kennung im Namen, damit mehrere neue Bridges unterscheidbar sind
  hostName = lowerSlug.length() ? "m218-" + lowerSlug : "m218-" + idLower;
  apName   = caseSlug.length()  ? "M218-" + caseSlug + "-Setup" : "M218-WiiM-Bridge-" + id;
}

void loadSettings() {
  prefs.begin("m218", true);
  prefs.getString("room", "").toCharArray(roomName, sizeof(roomName));
  prefs.getString("wiimName", "").toCharArray(wiimName, sizeof(wiimName));
  prefs.getString("wiimIp", "").toCharArray(wiimIp, sizeof(wiimIp));
  volStep   = prefs.getInt("volStep", 2);
  maxVolume = prefs.getInt("maxVol", 70);
  for (int i = 0; i < KEY_COUNT; i++) {
    keyAction[i] = findAction(prefs.getString(KEYS[i].prefKey, KEYS[i].defaultAction));
  }
  prefs.end();
  applyRoomName();
}

void saveSettings() {
  prefs.begin("m218", false);
  prefs.putString("room", roomName);
  prefs.putString("wiimName", wiimName);
  prefs.putString("wiimIp", wiimIp);
  prefs.putInt("volStep", volStep);
  prefs.putInt("maxVol", maxVolume);
  for (int i = 0; i < KEY_COUNT; i++) prefs.putString(KEYS[i].prefKey, ACTIONS[keyAction[i]].id);
  prefs.end();
  Serial.printf("[Config] Gespeichert: Raum '%s', WiiM '%s', IP %s, Schritt %d, Max %d\n",
                roomName, wiimName, strlen(wiimIp) ? wiimIp : "(keine)", volStep, maxVolume);
  for (int i = 0; i < KEY_COUNT; i++)
    Serial.printf("         %-6s -> %s\n", KEYS[i].name, ACTIONS[keyAction[i]].label);
}

void saveWiimIp() {
  prefs.begin("m218", false);
  prefs.putString("wiimIp", wiimIp);
  prefs.end();
}

bool setIpIfValid(const String& ip) {
  IPAddress test;
  if (ip.length() == 0 || test.fromString(ip)) {
    strlcpy(wiimIp, ip.c_str(), sizeof(wiimIp));
    return true;
  }
  return false;
}

bool hasSavedWifi() {
  prefs.begin("m218", true);
  bool ok = prefs.getBool("wifiOk", false);
  prefs.end();
  return ok;
}

void markWifiSaved() {
  prefs.begin("m218", false);
  prefs.putBool("wifiOk", true);
  prefs.end();
}

String htmlEscape(const String& s) {
  String o;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if      (c == '&')  o += F("&amp;");
    else if (c == '<')  o += F("&lt;");
    else if (c == '>')  o += F("&gt;");
    else if (c == '"')  o += F("&quot;");
    else if (c == '\'') o += F("&#39;");
    else o += c;
  }
  return o;
}

// =====================================================================
//  WiiM im Netz suchen (mDNS + Geraetename per getStatusEx)
// =====================================================================
WiimInfo found[MAX_FOUND];          // Ergebnis der letzten Suche (fuer die Webseite)
volatile int  foundCount = -1;      // -1 = noch nicht gesucht
volatile bool scanBusy   = false;

// Liest "key": "wert" aus einer JSON-Antwort (Gross/Klein egal, Leerzeichen erlaubt)
String jsonField(const String& r, const char* key) {
  String low = r;
  low.toLowerCase();
  String k = String("\"") + key + "\"";
  k.toLowerCase();
  int i = low.indexOf(k);
  while (i >= 0) {
    unsigned int p = i + k.length();
    while (p < r.length() && isspace((unsigned char)r[p])) p++;
    if (p < r.length() && r[p] == ':') {
      p++;
      while (p < r.length() && isspace((unsigned char)r[p])) p++;
      if (p < r.length() && r[p] == '"') {
        int e = r.indexOf('"', p + 1);
        if (e > (int)p) return r.substring(p + 1, e);
      }
      return "";
    }
    i = low.indexOf(k, i + 1);
  }
  return "";
}

// Fragt ein Geraet nach seinem Namen (erst HTTPS, dann HTTP). Leer = kein WiiM/LinkPlay-Geraet.
String queryDeviceName(const String& ip) {
  for (int secure = 1; secure >= 0; secure--) {
    WiFiClientSecure tls;
    WiFiClient plain;
    tls.setInsecure();
    HTTPClient h;
    h.setConnectTimeout(1500);
    h.setTimeout(2000);
    String url = String(secure ? "https://" : "http://") + ip + "/httpapi.asp?command=getStatusEx";
    bool ok = secure ? h.begin(tls, url) : h.begin(plain, url);
    if (!ok) continue;
    int code = h.GET();
    String body = (code == 200) ? h.getString() : "";
    h.end();
    String name;
    const char* keys[] = { "DeviceName", "device_name", "friendlyName", "ssid" };
    for (const char* k : keys) {
      name = jsonField(body, k);
      name.trim();
      if (name.length()) break;
    }
    // Antwort sieht nach WiiM/LinkPlay aus, aber ohne Namen -> IP als Namen verwenden
    if (!name.length() && (body.indexOf("uuid") >= 0 || body.indexOf("firmware") >= 0)) name = "WiiM " + ip;
    Serial.printf("[Suche]   %s %s -> HTTP %d, %d Zeichen, %s\n", secure ? "https" : "http ", ip.c_str(), code,
                  body.length(), name.length() ? ("Name '" + name + "'").c_str() : "(kein WiiM)");
    if (code == 200 && body.length()) Serial.println("[Suche]   Antwort: " + body.substring(0, 400));
    if (name.length()) return name;
    if (code > 0) return "";          // Geraet antwortet, ist aber kein WiiM
  }
  return "";
}

void addCandidate(CandList& cl, IPAddress ip, const char* via) {
  if (ip == IPAddress(0, 0, 0, 0) || ip == WiFi.localIP()) return;
  String s = ip.toString();
  for (int i = 0; i < cl.n; i++) if (cl.ip[i] == s) return;
  if (cl.n >= MAX_CAND) return;
  cl.ip[cl.n++] = s;
  Serial.printf("[Suche]   Kandidat %-15s (%s)\n", s.c_str(), via);
}

// mDNS-Dienst abfragen; fehlt die IPv4-Adresse, wird der Hostname aufgeloest
void mdnsCandidates(CandList& cl, const char* svc) {
  int c = MDNS.queryService(svc, "tcp");
  Serial.printf("[Suche] mDNS _%s._tcp: %d Antwort(en)\n", svc, c);
  String hosts[MAX_CAND];
  int hn = 0;
  for (int i = 0; i < c; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    IPAddress ip = MDNS.address(i);
#else
    IPAddress ip = MDNS.IP(i);
#endif
    if (ip != IPAddress(0, 0, 0, 0)) addCandidate(cl, ip, svc);
    else if (hn < MAX_CAND) hosts[hn++] = MDNS.hostname(i);
  }
  for (int i = 0; i < hn; i++) {
    if (!hosts[i].length()) continue;
    IPAddress ip = MDNS.queryHost(hosts[i], 1500);
    Serial.printf("[Suche]   Host %s -> %s\n", hosts[i].c_str(), ip.toString().c_str());
    addCandidate(cl, ip, svc);
  }
}

// UPnP/SSDP: WiiM meldet sich als MediaRenderer (funktioniert auch, wenn mDNS gesperrt ist)
void ssdpCandidates(CandList& cl) {
  WiFiUDP udp;
  if (!udp.begin(1901)) return;
  const char* req = "M-SEARCH * HTTP/1.1\r\nHOST: 239.255.255.250:1900\r\nMAN: \"ssdp:discover\"\r\n"
                    "MX: 2\r\nST: urn:schemas-upnp-org:device:MediaRenderer:1\r\n\r\n";
  for (int k = 0; k < 2; k++) {
    udp.beginPacket(IPAddress(239, 255, 255, 250), 1900);
    udp.print(req);
    udp.endPacket();
    delay(100);
  }
  int answers = 0;
  uint32_t t0 = millis();
  char buf[256];
  while (millis() - t0 < 3000) {
    if (udp.parsePacket()) {
      answers++;
      IPAddress ip = udp.remoteIP();
      while (udp.available()) udp.read(buf, sizeof(buf));
      addCandidate(cl, ip, "ssdp");
    } else {
      delay(20);
    }
  }
  udp.stop();
  Serial.printf("[Suche] SSDP MediaRenderer: %d Antwort(en)\n", answers);
}

int checkCandidates(CandList& cl, WiimInfo* out, int n, int maxN) {
  for (int i = 0; i < cl.n && n < maxN; i++) {
    bool dup = false;
    for (int j = 0; j < n; j++) if (cl.ip[i] == out[j].ip) dup = true;
    if (dup) continue;
    String name = queryDeviceName(cl.ip[i]);
    if (!name.length()) continue;
    strlcpy(out[n].name, name.c_str(), sizeof(out[n].name));
    strlcpy(out[n].ip, cl.ip[i].c_str(), sizeof(out[n].ip));
    n++;
  }
  return n;
}

// Sucht WiiM-Geraete: zuerst mDNS _linkplay._tcp; findet das nichts,
// dann AirPlay (_raop), Google Cast und UPnP/SSDP. Jedes Geraet wird per getStatusEx gefragt.
// Gespeicherte WiiM-Liste (wird beim Start und alle 10 min im Hintergrund erneuert)
const uint32_t CACHE_REFRESH_MS = 10UL * 60UL * 1000UL;
const uint32_t CACHE_MAX_AGE_MS = 30UL * 60UL * 1000UL;
WiimInfo wiimCache[MAX_FOUND];
int      wiimCacheCount = -1;
uint32_t wiimCacheTime  = 0;
volatile uint32_t lastIrActivity = 0;   // Hintergrundsuche nur, wenn niemand bedient
bool     overviewHostActive = false;    // diese Bridge beantwortet gerade m218.local
uint32_t bridgeTime = 0;                // letzte Suche nach anderen Bridges
const uint32_t BRIDGE_REFRESH_MS = 60000;
void refreshBridges();                  // weiter unten (Uebersicht)

int scanWiims(WiimInfo* out, int maxN) {
  if (WiFi.status() != WL_CONNECTED) return 0;
  Serial.println("[Suche] Starte WiiM-Suche ...");
  int n = 0;
  {
    CandList cl;
    mdnsCandidates(cl, "linkplay");
    n = checkCandidates(cl, out, n, maxN);
  }
  if (!n) {
    CandList cl;
    mdnsCandidates(cl, "raop");
    mdnsCandidates(cl, "googlecast");
    ssdpCandidates(cl);
    n = checkCandidates(cl, out, n, maxN);
  }
  for (int i = 0; i < n; i++) Serial.printf("[Suche] gefunden: %-20s %s\n", out[i].name, out[i].ip);
  if (!n) Serial.println("[Suche] Kein WiiM gefunden.");
  // Ergebnis merken: Gruppieren und "alle aus" nutzen diese Liste ohne neue Suche
  for (int i = 0; i < n; i++) wiimCache[i] = out[i];
  wiimCacheCount = n;
  wiimCacheTime  = millis();
  return n;
}

// WiiM-Liste aus dem Speicher (schnell) oder per neuer Suche (fresh = true oder Liste veraltet)
int wiimList(WiimInfo* out, bool fresh) {
  if (!fresh && wiimCacheCount > 0 && millis() - wiimCacheTime < CACHE_MAX_AGE_MS) {
    for (int i = 0; i < wiimCacheCount; i++) out[i] = wiimCache[i];
    Serial.printf("[Suche] %d WiiM(s) aus der gespeicherten Liste.\n", wiimCacheCount);
    return wiimCacheCount;
  }
  return scanWiims(out, MAX_FOUND);
}

// =====================================================================
//  WiiM-Kommunikation (eigener Task)
// =====================================================================
struct Cmd { char text[64]; };
QueueHandle_t cmdQueue;
portMUX_TYPE  volMux = portMUX_INITIALIZER_UNLOCKED;
volatile int  pendingVolDelta = 0;

WiFiClientSecure tlsClient;
HTTPClient       http;
int      currentVol  = -1;
uint32_t lastVolSync = 0;
uint32_t lastResolve = 0;
bool     resolvedOnce = false;

// Gespeicherten WiiM-Namen im Netz suchen und IP aktualisieren
bool resolveWiim() {
  if (!strlen(wiimName) || WiFi.status() != WL_CONNECTED) return false;
  if (resolvedOnce && millis() - lastResolve < RESOLVE_PAUSE_MS) return false;
  resolvedOnce = true;
  lastResolve  = millis();
  Serial.printf("[WiiM] Suche '%s' ...\n", wiimName);

  WiimInfo tmp[MAX_FOUND];
  int n = scanWiims(tmp, MAX_FOUND);
  for (int i = 0; i < n; i++) {
    if (strcasecmp(tmp[i].name, wiimName) != 0) continue;
    if (strcmp(tmp[i].ip, wiimIp) != 0) {
      http.end();
      tlsClient.stop();          // alte Verbindung verwerfen
      strlcpy(wiimIp, tmp[i].ip, sizeof(wiimIp));
      saveWiimIp();
      currentVol = -1;
    }
    Serial.printf("[WiiM] '%s' gefunden: %s\n", wiimName, wiimIp);
    return true;
  }
  Serial.printf("[WiiM] '%s' nicht gefunden, nutze %s\n", wiimName,
                strlen(wiimIp) ? wiimIp : "(keine IP)");
  return false;
}

String wiimRequest(const char* command, bool retry) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiiM] Kein WLAN.");
    return "";
  }
  if (strlen(wiimIp) == 0) {
    if (retry && resolveWiim()) return wiimRequest(command, false);
    Serial.println("[WiiM] Kein WiiM eingestellt bzw. nicht gefunden.");
    return "";
  }
  String url = String("https://") + wiimIp + "/httpapi.asp?command=" + command;
  int code = -1;
  String body;
  if (http.begin(tlsClient, url)) {
    code = http.GET();
    body = (code > 0) ? http.getString() : "";
    http.end();
  }
  Serial.printf("[WiiM] %s -> HTTP %d\n", command, code);

  // WiiM antwortet nicht -> evtl. neue IP: per Name neu suchen und einmal wiederholen
  if (code <= 0 && retry && strlen(wiimName)) {
    if (resolveWiim()) return wiimRequest(command, false);
  }
  return body;
}

String wiimGet(const char* command) { return wiimRequest(command, true); }

bool syncVolume() {
  String r = wiimGet("getPlayerStatus");
  int i = r.indexOf("\"vol\":\"");
  if (i < 0) return false;
  currentVol  = r.substring(i + 7).toInt();
  lastVolSync = millis();
  return true;
}

void applyVolumeDelta(int delta) {
  if (currentVol < 0 || millis() - lastVolSync > VOL_RESYNC_MS) {
    if (!syncVolume()) return;
  }
  int nv = currentVol + delta;
  if (delta > 0) nv = min(nv, max(maxVolume, currentVol));   // nie ueber Max hochregeln
  nv = max(nv, 0);
  if (nv == currentVol) return;

  char buf[32];
  snprintf(buf, sizeof(buf), "setPlayerCmd:vol:%d", nv);
  wiimGet(buf);
  currentVol  = nv;
  lastVolSync = millis();
  Serial.printf("[WiiM] Lautstaerke -> %d\n", nv);
}

// =====================================================================
//  Multiroom (WiiM/LinkPlay-Gruppen)
//  Befehle der LinkPlay-API (nicht offiziell dokumentiert, deshalb ausfuehrlich protokolliert):
//    Slave tritt bei:   ConnectMasterAp:JoinGroupMaster:eth<Master-IP>:wifi0.0.0.0   (an den Slave)
//    Slaves abfragen:   multiroom:getSlaveList                                       (an den Master)
//    Slave entfernen:   multiroom:SlaveKickout:<Slave-IP>                            (an den Master)
//    Gruppe aufloesen:  multiroom:Ungroup                                            (an den Master)
// =====================================================================
volatile uint32_t ledFlashUntil = 0;   // LED blinkt schnell bis zu diesem Zeitpunkt (Bestaetigung)

// Befehl an einen beliebigen WiiM (nicht nur den eigenen)
String wiimRequestTo(const String& ip, const String& command) {
  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient h;
  h.setConnectTimeout(2000);
  h.setTimeout(3000);
  String body;
  int code = -1;
  if (h.begin(c, "https://" + ip + "/httpapi.asp?command=" + command)) {
    code = h.GET();
    body = (code > 0) ? h.getString() : "";
    h.end();
  }
  Serial.printf("[Gruppe]   %s: %s -> HTTP %d %s\n", ip.c_str(), command.c_str(), code, body.substring(0, 120).c_str());
  return body;
}

int slaveCount(const String& r) {
  int i = r.indexOf("\"slaves\"");
  if (i < 0) return 0;
  i = r.indexOf(':', i);
  while (i >= 0 && i + 1 < (int)r.length() && (r[i + 1] == ' ' || r[i + 1] == '"')) i++;
  return (i < 0) ? 0 : r.substring(i + 1).toInt();
}

// Langer Tastendruck: eigener WiiM wird Master, alle anderen WiiMs, die gerade spielen, treten bei
void groupPlayingWiims() {
  if (!strlen(wiimIp)) resolveWiim();
  if (!strlen(wiimIp)) { Serial.println("[Gruppe] Eigener WiiM unbekannt."); return; }
  Serial.printf("[Gruppe] Bilde Gruppe, Master %s (%s) ...\n", wiimIp, strlen(wiimName) ? wiimName : "-");
  WiimInfo list[MAX_FOUND];
  int joined = 0;
  bool failed = false;
  for (int pass = 0; pass < 2; pass++) {
  int n = wiimList(list, pass == 1);
  failed = false;
  for (int i = 0; i < n; i++) {
    if (strcmp(list[i].ip, wiimIp) == 0) continue;
    String st = wiimRequestTo(list[i].ip, "getPlayerStatus");
    if (!st.length()) { failed = true; continue; }        // keine Antwort: Liste evtl. veraltet
    String status = jsonField(st, "status");
    Serial.printf("[Gruppe]   %-16s %s Status '%s'\n", list[i].name, list[i].ip, status.c_str());
    if (status != "play") continue;                       // nur Raeume, die gerade spielen
    wiimRequestTo(list[i].ip, String("ConnectMasterAp:JoinGroupMaster:eth") + wiimIp + ":wifi0.0.0.0");
    joined++;
  }
  if (!failed) break;
  Serial.println("[Gruppe] Ein WiiM hat nicht geantwortet - suche neu ...");
  }
  Serial.printf("[Gruppe] %d Raum/Raeume beigetreten.\n", joined);
  ledFlashUntil = millis() + (joined ? 1500 : 300);
}

// Off/andere Quelle: als Master Gruppe aufloesen, als Slave die Gruppe verlassen
void leaveGroup(bool alsoAsMaster) {
  if (!strlen(wiimIp)) return;
  // eigener WiiM ueber die bestehende Verbindung (schnell)
  if (alsoAsMaster && slaveCount(wiimGet("multiroom:getSlaveList")) > 0) {
    Serial.println("[Gruppe] Master: loese Gruppe auf.");
    wiimGet("multiroom:Ungroup");
    return;
  }
  // Slave? getStatusEx liefert bei LinkPlay "group":"1" fuer einen Slave
  String st = wiimGet("getStatusEx");
  String grp = jsonField(st, "group");
  String master = jsonField(st, "master_ip");
  if (grp != "1") return;
  Serial.printf("[Gruppe] Slave (Master '%s'): verlasse Gruppe.\n", master.c_str());
  if (master.length()) {
    wiimRequestTo(master, String("multiroom:SlaveKickout:") + wiimIp);
  } else {                                                // Master unbekannt: alle anderen bitten
    WiimInfo list[MAX_FOUND];
    int n = wiimList(list, false);
    for (int i = 0; i < n; i++)
      if (strcmp(list[i].ip, wiimIp) != 0) wiimRequestTo(list[i].ip, String("multiroom:SlaveKickout:") + wiimIp);
  }
}

// Langes Off: alle WiiMs im Netz ausschalten (Gruppen aufloesen, dann stoppen)
void allWiimsOff() {
  Serial.println("[Alle aus] Alle WiiMs ausschalten ...");
  WiimInfo list[MAX_FOUND];
  int n = 0, ok = 0;
  for (int pass = 0; pass < 2; pass++) {
    n = wiimList(list, pass == 1);
    bool failed = false;
    ok = 0;
    for (int i = 0; i < n; i++) {                         // zuerst Gruppen aufloesen
      String r = wiimRequestTo(list[i].ip, "multiroom:getSlaveList");
      if (!r.length()) { failed = true; continue; }
      if (slaveCount(r) > 0) wiimRequestTo(list[i].ip, "multiroom:Ungroup");
    }
    for (int i = 0; i < n; i++) if (wiimRequestTo(list[i].ip, OFF_COMMAND).length()) ok++;
    if (!failed) break;
    Serial.println("[Alle aus] Ein WiiM hat nicht geantwortet - suche neu ...");
  }
  Serial.printf("[Alle aus] %d von %d WiiM(s) gestoppt.\n", ok, n);
  ledFlashUntil = millis() + (n ? 1500 : 300);
}

// =====================================================================
//  Firmware-Update ueber das Internet (GitHub-Releases) und Rueckfall
// =====================================================================
char     otaLatest[24] = "";          // neueste Version auf GitHub, z.B. "3.12"
char     otaUrl[320]   = "";          // Download-Adresse der .bin
volatile int otaState  = 0;           // 0 = nicht geprueft, 1 = aktuell, 2 = Update da, 3 = Fehler, 4 = installiere
String   otaMsg        = "";
uint32_t otaLastCheck  = 0;
bool     otaValidated  = false;

// Arduino-Core: true = neue Firmware wird erst spaeter bestaetigt (sonst sofort)
bool verifyRollbackLater() { return true; }

// Nach erfolgreichem Start (WLAN verbunden oder 2 min ohne Absturz) die laufende Firmware bestaetigen.
// Stuerzt eine neue Firmware vorher ab, startet der ESP32 wieder die vorherige.
void otaMarkValid() {
  if (otaValidated) return;
  if (WiFi.status() != WL_CONNECTED && millis() < 120000) return;
  otaValidated = true;
  esp_ota_img_states_t st;
  const esp_partition_t* p = esp_ota_get_running_partition();
  if (esp_ota_get_state_partition(p, &st) == ESP_OK && st == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    Serial.printf("[Update] Firmware %s laeuft und ist bestaetigt.\n", FW_VERSION);
  }
}

// Versionsvergleich "3.11" vs "3.9" (Zahlen je Abschnitt), fuehrendes "v" wird ignoriert
int cmpVersion(const char* a, const char* b) {
  if (*a == 'v' || *a == 'V') a++;
  if (*b == 'v' || *b == 'V') b++;
  while (*a || *b) {
    long x = strtol(a, (char**)&a, 10), y = strtol(b, (char**)&b, 10);
    if (x != y) return x < y ? -1 : 1;
    if (*a == '.') a++;
    if (*b == '.') b++;
    if (!isdigit((unsigned char)*a) && !isdigit((unsigned char)*b)) break;
  }
  return 0;
}

void otaCheck() {
  otaLastCheck = millis();
  if (WiFi.status() != WL_CONNECTED) { otaState = 3; otaMsg = "kein WLAN"; return; }
  http.end(); tlsClient.stop();                         // Speicher fuer TLS freigeben
  WiFiClientSecure c;
  c.setCACert(GITHUB_ROOT_CA);
  HTTPClient h;
  h.setConnectTimeout(5000);
  h.setTimeout(8000);
  String url = String("https://api.github.com/repos/") + GITHUB_OWNER + "/" + GITHUB_REPO + "/releases/latest";
  Serial.println("[Update] Frage " + url);
  if (!h.begin(c, url)) { otaState = 3; otaMsg = "Verbindung fehlgeschlagen"; return; }
  h.addHeader("User-Agent", "M218-WiiM-Bridge");
  h.addHeader("Accept", "application/vnd.github+json");
  int code = h.GET();
  String body = (code == 200) ? h.getString() : "";
  h.end();
  if (code == 404) { otaState = 1; otaMsg = "noch kein Release auf GitHub"; Serial.println("[Update] Noch kein Release."); return; }
  if (code != 200) { otaState = 3; otaMsg = "GitHub antwortet nicht (HTTP " + String(code) + ")"; Serial.println("[Update] " + otaMsg); return; }
  String tag = jsonField(body, "tag_name");
  // erste .bin-Datei des Releases suchen
  String bin;
  int pos = 0;
  while ((pos = body.indexOf("\"browser_download_url\"", pos)) >= 0) {
    String u = jsonField(body.substring(pos), "browser_download_url");
    pos += 22;
    if (u.endsWith(".bin")) { bin = u; break; }
  }
  if (tag.startsWith("v") || tag.startsWith("V")) tag.remove(0, 1);
  strlcpy(otaLatest, tag.c_str(), sizeof(otaLatest));
  strlcpy(otaUrl, bin.c_str(), sizeof(otaUrl));
  if (!tag.length() || !bin.length()) { otaState = 3; otaMsg = "Release ohne .bin-Datei"; }
  else if (cmpVersion(otaLatest, FW_VERSION) > 0) { otaState = 2; otaMsg = String("Version ") + otaLatest + " verfuegbar"; }
  else { otaState = 1; otaMsg = "aktuell"; }
  Serial.printf("[Update] Installiert %s, GitHub %s -> %s\n", FW_VERSION, otaLatest, otaMsg.c_str());
}

void otaInstall() {
  if (otaState != 2 || !strlen(otaUrl)) { Serial.println("[Update] Kein Update vorgemerkt."); return; }
  otaState = 4;
  otaMsg = "Update wird installiert ...";
  Serial.printf("[Update] Lade %s ...\n", otaUrl);
  http.end(); tlsClient.stop();
  WiFiClientSecure c;
  c.setCACert(GITHUB_ROOT_CA);
  c.setTimeout(20);
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);   // GitHub leitet zum Downloadserver um
  httpUpdate.setLedPin(STATUS_LED_PIN, HIGH);
  httpUpdate.rebootOnUpdate(false);
  httpUpdate.onProgress([](int cur, int total) {
    static int last = -1;
    int pct = total > 0 ? cur * 100 / total : 0;
    if (pct / 10 != last) { last = pct / 10; Serial.printf("[Update] %d %%\n", pct); }
  });
  t_httpUpdate_return r = httpUpdate.update(c, otaUrl);
  if (r == HTTP_UPDATE_OK) {
    Serial.println("[Update] Fertig - Neustart.");
    delay(500);
    ESP.restart();
  }
  otaState = 3;
  otaMsg = "Update fehlgeschlagen: " + httpUpdate.getLastErrorString();
  Serial.println("[Update] " + otaMsg);
}

void wiimTask(void*) {
  tlsClient.setInsecure();     // WiiM nutzt ein selbstsigniertes Zertifikat
  http.setReuse(true);
  http.setTimeout(3000);
  http.setConnectTimeout(3000);

  Cmd m;
  for (;;) {
    if (xQueueReceive(cmdQueue, &m, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (strcmp(m.text, "#scan") == 0) {               // Liste fuer die Webseite
        scanBusy = true;
        foundCount = scanWiims(found, MAX_FOUND);
        scanBusy = false;
      } else if (strcmp(m.text, "#resolve") == 0) {     // gespeicherten Namen suchen
        resolvedOnce = false;
        resolveWiim();
      } else if (strcmp(m.text, "#group") == 0) {       // langer Tastendruck: Gruppe bilden
        groupPlayingWiims();
      } else if (strcmp(m.text, "#leave") == 0) {       // Off: aufloesen bzw. verlassen
        leaveGroup(true);
      } else if (strcmp(m.text, "#otacheck") == 0) {    // nach Firmware-Update fragen
        otaCheck();
      } else if (strcmp(m.text, "#otainstall") == 0) {  // Firmware-Update installieren
        otaInstall();
      } else if (strcmp(m.text, "#bridges") == 0) {     // Uebersicht aktualisieren
        refreshBridges();
      } else if (strcmp(m.text, "#alloff") == 0) {      // langes Off: alle WiiMs aus
        allWiimsOff();
      } else if (strcmp(m.text, "#leaveslave") == 0) {  // neue Quelle: nur als Slave verlassen
        leaveGroup(false);
      } else {
        String r = wiimGet(m.text);
        if (strncmp(m.text, "get", 3) == 0) Serial.println("[WiiM] Antwort: " + r.substring(0, 300));
      }
    }
    // Hintergrund: einmal taeglich (erstmals 2 min nach dem Start) nach einem Firmware-Update fragen.
    // Installiert wird nur auf Knopfdruck auf der Einstellungsseite.
    if (WiFi.status() == WL_CONNECTED && uxQueueMessagesWaiting(cmdQueue) == 0 && millis() > 120000 &&
        millis() - lastIrActivity > 30000 && (otaLastCheck == 0 || millis() - otaLastCheck > OTA_CHECK_MS)) {
      otaCheck();
    }
    // Hintergrund: andere Bridges suchen und m218.local vergeben (jede Minute, wenn 10 s niemand bedient)
    if (WiFi.status() == WL_CONNECTED && uxQueueMessagesWaiting(cmdQueue) == 0 &&
        millis() - lastIrActivity > 10000 && millis() > 8000 &&
        (bridgeTime == 0 || millis() - bridgeTime > BRIDGE_REFRESH_MS)) {
      refreshBridges();
    }
    // Hintergrund: WiiM-Liste erneuern (beim Start und alle 10 min, nur wenn 30 s niemand bedient)
    if (WiFi.status() == WL_CONNECTED && uxQueueMessagesWaiting(cmdQueue) == 0 &&
        millis() - lastIrActivity > 30000 && millis() > 15000 &&
        (wiimCacheCount < 0 || millis() - wiimCacheTime > CACHE_REFRESH_MS)) {
      WiimInfo tmp[MAX_FOUND];
      scanWiims(tmp, MAX_FOUND);
      if (wiimCacheCount < 0) wiimCacheCount = 0;
      wiimCacheTime = millis();
    }
    int d;
    portENTER_CRITICAL(&volMux);
    d = pendingVolDelta;
    pendingVolDelta = 0;
    portEXIT_CRITICAL(&volMux);
    if (d != 0) applyVolumeDelta(d);
  }
}

bool queueCmd(const char* c) {
  if (!c || !*c) return false;
  Cmd m;
  strlcpy(m.text, c, sizeof(m.text));
  if (xQueueSend(cmdQueue, &m, 0) == pdTRUE) return true;
  Serial.println("[WiiM] Warteschlange voll!");
  return false;
}

void addVolDelta(int d) {
  portENTER_CRITICAL(&volMux);
  pendingVolDelta += d;
  portEXIT_CRITICAL(&volMux);
}

// =====================================================================
//  Tastenlogik
// =====================================================================
int activeKey = -1;   // zuletzt gewaehlte Quellentaste, -1 = aus

void triggerKey(int k, bool allowToggle) {
  const ActionDef& a = ACTIONS[keyAction[k]];
  if (allowToggle && k == activeKey && a.toggle) {
    Serial.printf("[Taste] %s erneut -> Play/Pause\n", KEYS[k].name);
    queueCmd(TOGGLE_COMMAND);
    return;
  }
  Serial.printf("[Taste] %s -> %s\n", KEYS[k].name, a.label);
  queueCmd("#leaveslave");                 // war dieser Raum Slave einer Gruppe? dann zuerst raus
  queueCmd(a.command);
  activeKey = k;
}

// Langes Halten einer Quellentaste (Wiederholungs-Codes der M218)
int      holdKey      = -1;
uint32_t holdStart    = 0;
uint32_t holdLast     = 0;
bool     holdGrouped  = false;
bool     holdToggle   = false;      // gleiche Taste nochmals: Play/Pause erst beim Loslassen

void sourcePressed(int k, bool isRepeat) {
  uint32_t now = millis();
  if (!isRepeat || k != holdKey) {                     // neuer Tastendruck
    holdKey = k; holdStart = now; holdLast = now; holdGrouped = false;
    if (k == activeKey && ACTIONS[keyAction[k]].toggle) {
      holdToggle = true;                               // kurz = Play/Pause, lang = Gruppe
    } else {
      holdToggle = false;
      triggerKey(k, false);                            // Quelle sofort starten
    }
    return;
  }
  holdLast = now;                                      // Wiederholung: Taste wird gehalten
  if (!holdGrouped && now - holdStart >= GROUP_HOLD_MS) {
    holdGrouped = true;
    holdToggle = false;
    Serial.printf("[Taste] %s lange gehalten -> Multiroom-Gruppe bilden (dieser Raum = Master)\n", KEYS[k].name);
    ledFlashUntil = millis() + 400;
    queueCmd("#group");
  }
}

// Off-Taste: kurz = dieser Raum aus, 2 s halten = alle WiiMs aus
uint32_t offStart = 0, offLast = 0;
bool     offAllDone = false;

void offPressed(bool isRepeat) {
  uint32_t now = millis();
  if (!isRepeat || now - offLast > IR_RELEASE_MS) {     // neuer Tastendruck
    offStart = now; offAllDone = false;
    Serial.println("[Taste] Off");
    queueCmd("#leave");                  // Master: Gruppe aufloesen, Slave: Gruppe verlassen
    queueCmd(OFF_COMMAND);
    activeKey = -1;
  } else if (!offAllDone && now - offStart >= GROUP_HOLD_MS) {
    offAllDone = true;
    Serial.println("[Taste] Off lange gehalten -> alle WiiMs aus");
    ledFlashUntil = millis() + 400;
    queueCmd("#alloff");
  }
  offLast = now;
}

void checkHoldRelease() {
  if (holdKey < 0 || millis() - holdLast < IR_RELEASE_MS) return;
  if (holdToggle) {
    Serial.printf("[Taste] %s erneut -> Play/Pause\n", KEYS[holdKey].name);
    queueCmd(TOGGLE_COMMAND);
  }
  holdKey = -1;
  holdToggle = false;
}

void handleIr() {
  if (!IrReceiver.decode()) return;
  IRData d = IrReceiver.decodedIRData;
  IrReceiver.resume();
  lastIrActivity = millis();

  if (d.protocol != RC5) return;
  bool isRepeat = d.flags & IRDATA_FLAGS_IS_REPEAT;

  if (d.command == RC5_VOL_UP)   { addVolDelta(+volStep); if (!isRepeat) Serial.println("[Taste] Vol +"); return; }
  if (d.command == RC5_VOL_DOWN) { addVolDelta(-volStep); if (!isRepeat) Serial.println("[Taste] Vol -"); return; }
  if (d.command == RC5_SOURCE) {
    for (int k = 0; k < KEY_COUNT; k++) {
      if (KEYS[k].address == d.address) { sourcePressed(k, isRepeat); return; }
    }
    if (!isRepeat) Serial.printf("[Taste] Unbekannte Quelle, Adresse 0x%02X\n", d.address);
    return;
  }
  if (d.command == RC5_OFF) { offPressed(isRepeat); return; }
  if (isRepeat) return;

  switch (d.command) {
    case RC5_PREV: Serial.println("[Taste] Skip previous"); queueCmd("setPlayerCmd:prev"); break;
    case RC5_NEXT: Serial.println("[Taste] Skip next");     queueCmd("setPlayerCmd:next"); break;
    default:
      Serial.printf("[Taste] Unbekannt: Adresse 0x%02X Command 0x%02X\n", d.address, d.command);
  }
}

// =====================================================================
//  mDNS (Name m218-<raum>.local)
// =====================================================================
void restartMdns() {
  MDNS.end();
  overviewHostActive = false;
  bridgeTime = 0;                       // Bridges bald neu suchen und m218.local neu vergeben
  if (MDNS.begin(hostName.c_str())) {
    String inst = strlen(roomName) ? String("M218 ") + roomName : String("M218");
    MDNS.setInstanceName(inst.c_str());
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("m218", "tcp", 80);  // Kennung fuer die Bridge-Uebersicht
    MDNS.addServiceTxt("m218", "tcp", "room", (const char*)roomName);
    MDNS.addServiceTxt("m218", "tcp", "wiim", (const char*)wiimName);
    MDNS.addServiceTxt("m218", "tcp", "id", deviceId().c_str());
  }
}

// =====================================================================
//  Uebersicht aller Bridges: jede Bridge meldet sich als _m218._tcp (TXT room, wiim).
//  Die Bridge mit der kleinsten IP-Adresse uebernimmt zusaetzlich den Namen m218.local,
//  faellt sie aus, uebernimmt nach spaetestens 1 min die naechste.
// =====================================================================
const int MAX_BRIDGES = 16;
BridgeInfo bridges[MAX_BRIDGES];
volatile int bridgeCount = 0;

static uint32_t ipOrder(const char* ip) {     // IP als Zahl fuer den Vergleich (a.b.c.d)
  IPAddress a; a.fromString(ip);
  return ((uint32_t)a[0] << 24) | ((uint32_t)a[1] << 16) | ((uint32_t)a[2] << 8) | a[3];
}

void refreshBridges() {
  if (WiFi.status() != WL_CONNECTED) return;
  BridgeInfo tmp[MAX_BRIDGES];
  int n = 0;
  // diese Bridge selbst
  strlcpy(tmp[0].room, roomName, sizeof(tmp[0].room));
  strlcpy(tmp[0].host, hostName.c_str(), sizeof(tmp[0].host));
  strlcpy(tmp[0].ip, WiFi.localIP().toString().c_str(), sizeof(tmp[0].ip));
  strlcpy(tmp[0].wiim, wiimName, sizeof(tmp[0].wiim));
  strlcpy(tmp[0].id, deviceId().c_str(), sizeof(tmp[0].id));
  n = 1;
  bool otherHasM218Name = false;
  mdns_result_t* res = nullptr;
  if (mdns_query_ptr("_m218", "_tcp", 1500, 20, &res) == ESP_OK) {
    for (mdns_result_t* r = res; r && n < MAX_BRIDGES; r = r->next) {
      char ip[16] = "";
      for (mdns_ip_addr_t* a = r->addr; a; a = a->next)
        if (a->addr.type == ESP_IPADDR_TYPE_V4) { strlcpy(ip, IPAddress(a->addr.u_addr.ip4.addr).toString().c_str(), sizeof(ip)); break; }
      if (!ip[0]) continue;
      bool dup = false;
      for (int j = 0; j < n; j++) if (strcmp(tmp[j].ip, ip) == 0) dup = true;
      if (dup) continue;
      BridgeInfo& b = tmp[n];
      memset(&b, 0, sizeof(b));
      strlcpy(b.ip, ip, sizeof(b.ip));
      if (r->hostname) strlcpy(b.host, r->hostname, sizeof(b.host));
      for (size_t t = 0; t < r->txt_count; t++) {
        if (!r->txt[t].key || !r->txt[t].value) continue;
        if (strcmp(r->txt[t].key, "room") == 0) strlcpy(b.room, r->txt[t].value, sizeof(b.room));
        if (strcmp(r->txt[t].key, "id") == 0)   strlcpy(b.id, r->txt[t].value, sizeof(b.id));
        if (strcmp(r->txt[t].key, "wiim") == 0) strlcpy(b.wiim, r->txt[t].value, sizeof(b.wiim));
      }
      if (strcmp(b.host, "m218") == 0) otherHasM218Name = true;
      n++;
    }
    mdns_query_results_free(res);
  }
  for (int i = 0; i < n; i++) bridges[i] = tmp[i];
  bridgeCount = n;
  bridgeTime = millis();

  // Wahl: kleinste IP beantwortet m218.local (nicht, wenn eine Bridge ohne Raumname schon so heisst)
  bool winner = !otherHasM218Name && hostName != "m218";
  for (int i = 1; i < n && winner; i++) if (ipOrder(tmp[i].ip) < ipOrder(tmp[0].ip)) winner = false;
  mdns_ip_addr_t addr;
  memset(&addr, 0, sizeof(addr));
  addr.addr.type = ESP_IPADDR_TYPE_V4;
  addr.addr.u_addr.ip4.addr = (uint32_t)WiFi.localIP();
  if (winner) {
    esp_err_t e = overviewHostActive ? mdns_delegate_hostname_set_address("m218", &addr)
                                     : mdns_delegate_hostname_add("m218", &addr);
    if (!overviewHostActive && e == ESP_OK) Serial.println("[Uebersicht] Diese Bridge beantwortet jetzt http://m218.local");
    overviewHostActive = (e == ESP_OK) || overviewHostActive;
  } else if (overviewHostActive) {
    mdns_delegate_hostname_remove("m218");
    overviewHostActive = false;
    Serial.println("[Uebersicht] m218.local an eine andere Bridge abgegeben");
  }
  Serial.printf("[Uebersicht] %d Bridge(s) im Netz\n", n);
}

void sendOverviewPage(WebServer& s) {
  String h;
  h.reserve(4000);
  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>");
  if (s.hasArg("refresh")) h += F("<meta http-equiv='refresh' content='3;url=/'>");
  h += F("<title>M218 Bridges</title><style>"
         "body{font-family:-apple-system,Segoe UI,sans-serif;max-width:560px;margin:0 auto;padding:16px;background:#f4f4f4;color:#222}"
         "h2{margin:0 0 4px}.sub{color:#666;font-size:14px;margin:0 0 16px}"
         "a.card{display:block;background:#fff;border:1px solid #ccc;border-radius:8px;padding:12px 14px;margin:8px 0;color:#222;text-decoration:none}"
         "a.card b{font-size:18px}.me{border-color:#1f6feb!important}.meta{color:#666;font-size:13px;margin-top:4px}"
         ".btn{display:inline-block;margin-top:16px;padding:10px 14px;border-radius:6px;background:#ddd;color:#222;text-decoration:none}"
         "</style></head><body><h2>M218 &rarr; WiiM</h2>");
  h += "<p class='sub'>" + String((int)bridgeCount) + " Bridge(s) im Netz. Raum antippen f&uuml;r die Einstellungen.</p>";
  // nach Raumname sortiert ausgeben
  int n = bridgeCount;
  int order[MAX_BRIDGES];
  for (int i = 0; i < n; i++) order[i] = i;
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++)
      if (strcasecmp(bridges[order[j]].room, bridges[order[i]].room) < 0) { int t = order[i]; order[i] = order[j]; order[j] = t; }
  String myIp = WiFi.localIP().toString();
  for (int k = 0; k < n; k++) {
    const BridgeInfo& b = bridges[order[k]];
    bool me = (myIp == b.ip);
    String room = strlen(b.room) ? htmlEscape(b.room) : String("(ohne Raumname)");
    h += "<a class='card" + String(me ? " me" : "") + "' href='http://" + String(b.ip) + "/m218'><b>" + room + "</b>";
    if (me) h += F(" &middot; dieses Ger&auml;t");
    h += "<div class='meta'>" + (strlen(b.host) ? String(b.host) + ".local &middot; " : String("")) + b.ip;
    if (strlen(b.wiim)) h += " &middot; WiiM &laquo;" + htmlEscape(b.wiim) + "&raquo;";
    if (strlen(b.id))   h += " &middot; ID " + String(b.id);
    h += F("</div></a>");
  }
  h += F("<a class='btn' href='/refresh'>Liste aktualisieren</a> "
         "<a class='btn' href='/m218'>Einstellungen dieses Ger&auml;ts</a></body></html>");
  s.send(200, "text/html", h);
}

// =====================================================================
//  Einstellungsseite (gleiche Seite im Portal und im Betrieb)
// =====================================================================
void sendConfigPage(WebServer& s) {
  String title = strlen(roomName) ? htmlEscape(roomName) : String("M218 &rarr; WiiM");
  String h;
  h.reserve(8000);
  h += F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>");
  if (scanBusy || s.hasArg("scanning")) h += F("<meta http-equiv='refresh' content='4;url=/m218'>");
  h += "<title>M218 " + title + "</title>";
  h += F("<style>"
         "body{font-family:-apple-system,Segoe UI,sans-serif;max-width:480px;margin:0 auto;padding:16px;background:#f4f4f4;color:#222}"
         "h2{margin:0 0 4px}h3{margin:24px 0 8px;border-bottom:1px solid #ccc;padding-bottom:4px}"
         "label{display:block;margin:12px 0 4px;font-weight:600}"
         "select,input{width:100%;padding:10px;font-size:16px;box-sizing:border-box;border:1px solid #bbb;border-radius:6px;background:#fff}"
         ".row{display:flex;gap:8px}.row select{flex:1}"
         ".row a{padding:10px 14px;border-radius:6px;background:#ddd;color:#222;text-decoration:none;white-space:nowrap}"
         "button{margin-top:24px;width:100%;padding:14px;font-size:17px;border:0;border-radius:6px;background:#1f6feb;color:#fff}"
         ".ok{background:#d4f7d4;padding:10px;border-radius:6px;margin-top:12px}"
         ".info{background:#e8eefc;padding:10px;border-radius:6px;margin-top:12px}"
         ".sub{color:#666;font-size:14px;margin:0}"
         ".hint{font-size:13px;color:#666;margin-top:16px}"
         "</style></head><body>");
  h += "<h2>" + title + "</h2><p class='sub'>M218 &rarr; WiiM &middot; " + hostName + ".local &middot; ID " + deviceId() +
       " &middot; <a href='/m218/identify'>LED blinken lassen</a> &middot; <a href='/'>alle Bridges</a></p>";

  if (s.hasArg("saved")) h += F("<div class='ok'>Gespeichert.</div>");
  if (s.hasArg("tested")) h += "<div class='ok'>Test gesendet: " + String(KEYS[constrain(s.arg("tested").toInt(), 0, KEY_COUNT - 1)].name) + "</div>";
  if (scanBusy || s.hasArg("scanning")) h += F("<div class='info'>Suche WiiM-Ger&auml;te &hellip; die Seite l&auml;dt gleich neu.</div>");
  else if (foundCount == 0) h += F("<div class='info'>Kein WiiM gefunden. Ist der WiiM eingeschaltet und im gleichen WLAN (nicht im G&auml;ste-WLAN)? Details im seriellen Monitor. Bis dahin die IP-Adresse unten eintragen.</div>");

  h += F("<form method='post' action='/m218/save'>");

  // --- Raum ---
  h += F("<h3>Raum</h3><label>Raumname</label>");
  h += "<input name='room' maxlength='23' value='" + htmlEscape(roomName) + "' placeholder='z.B. Bad, Wohnzimmer'>";

  // --- WiiM ---
  h += F("<h3>WiiM</h3><label>WiiM-Ger&auml;t (Name aus der WiiM-App)</label><div class='row'><select name='wname'>");
  h += "<option value=''" + String(strlen(wiimName) ? "" : " selected") + ">&ndash; nur IP-Adresse verwenden &ndash;</option>";
  bool savedListed = false;
  int fc = scanBusy ? 0 : foundCount;
  for (int i = 0; i < fc; i++) {
    bool sel = strlen(wiimName) && strcasecmp(found[i].name, wiimName) == 0;
    if (sel) savedListed = true;
    h += "<option value='" + htmlEscape(found[i].name) + "'" + (sel ? " selected" : "") + ">" +
         htmlEscape(found[i].name) + " (" + found[i].ip + ")</option>";
  }
  if (strlen(wiimName) && !savedListed)
    h += "<option value='" + htmlEscape(wiimName) + "' selected>" + htmlEscape(wiimName) + " (gespeichert)</option>";
  h += F("</select><a href='/m218/scan'>Suchen</a></div>");

  h += F("<label>IP-Adresse (nur ohne Ger&auml;teauswahl n&ouml;tig)</label>");
  h += "<input name='ip' value='" + String(wiimIp) + "' placeholder='z.B. 192.168.1.50'>";
  if (strlen(wiimName))
    h += "<p class='sub'>Aktuell gefunden unter " + String(strlen(wiimIp) ? wiimIp : "&ndash;") +
         ". Die IP wird automatisch nachgef&uuml;hrt.</p>";

  // --- Tasten ---
  h += F("<h3>Quellentasten</h3>");
  for (int k = 0; k < KEY_COUNT; k++) {
    h += "<label>" + String(KEYS[k].name) + "</label><div class='row'><select name='k" + String(k) + "'>";
    for (int a = 0; a < ACTION_COUNT; a++) {
      h += "<option value='" + String(ACTIONS[a].id) + "'";
      if (a == keyAction[k]) h += " selected";
      h += ">" + String(ACTIONS[a].label) + "</option>";
    }
    h += "</select><a href='/m218/test?k=" + String(k) + "'>Test</a></div>";
  }

  // --- Lautstaerke ---
  h += F("<h3>Firmware</h3>");
  h += "<p class='sub'>Installiert: Version " + String(FW_VERSION);
  if (otaState == 1) h += " &middot; aktuell" + (otaMsg != "aktuell" ? " (" + htmlEscape(otaMsg) + ")" : String(""));
  if (otaState == 3) h += " &middot; " + htmlEscape(otaMsg);
  if (otaState == 4) h += F(" &middot; Update l&auml;uft, die Bridge startet gleich neu &hellip;");
  h += F("</p>");
  if (otaState == 2)
    h += "<div class='info'>Neue Version <b>" + htmlEscape(otaLatest) + "</b> verf&uuml;gbar.</div>"
         "<form method='post' action='/m218/otainstall' onsubmit=\"return confirm('Update installieren? "
         "Die Bridge startet danach neu, die Einstellungen bleiben erhalten.')\">"
         "<button type='submit'>Update auf " + htmlEscape(otaLatest) + " installieren</button></form>";
  h += F("<div class='row' style='margin-top:8px'><a href='/m218/otacheck'>Nach Update suchen</a>"
         "<a href='/update'>Datei hochladen</a></div>");
  h += F("<h3>Multiroom</h3><p class='sub'>Quellentaste an der M218 2 s halten: dieser Raum wird Master, "
         "alle WiiMs, die gerade spielen, h&ouml;ren mit. Off l&ouml;st die Gruppe auf.</p>"
         "<div class='row' style='margin-top:8px'><a href='/m218/group'>Gruppe bilden (Test)</a>"
         "<a href='/m218/ungroup'>Gruppe aufl&ouml;sen (Test)</a></div>"
         "<p class='sub'>Off 2 s halten: alle WiiMs aus.</p>"
         "<div class='row' style='margin-top:8px'><a href='/m218/alloff'>Alle WiiMs aus (Test)</a></div>");
  h += F("<h3>Lautst&auml;rke</h3>");
  h += "<label>Schritt pro Tastendruck (1&ndash;10)</label>"
       "<input type='number' name='step' min='1' max='10' value='" + String(volStep) + "'>";
  h += "<label>Maximale Lautst&auml;rke (0&ndash;100)</label>"
       "<input type='number' name='max' min='0' max='100' value='" + String(maxVolume) + "'>";
  h += F("<button type='submit'>Speichern</button></form>"
         "<p class='hint'>Den WiiMs in der WiiM-App eindeutige Namen geben (z.B. &laquo;Bad&raquo;). "
         "Presets in der WiiM-App belegen. Gleiche Taste nochmals dr&uuml;cken = Play/Pause "
         "(bei Presets und Streaming). Suchen und Test funktionieren nur mit WLAN-Verbindung, "
         "nicht im Setup-WLAN.</p>"
         "<form method='post' action='/m218/reset' onsubmit=\"return confirm('Wirklich alles l&ouml;schen? "
         "WLAN, Raum, WiiM und Tastenbelegung gehen verloren.')\">"
         "<button type='submit' style='background:#b42318'>Auf Werkseinstellungen zur&uuml;cksetzen</button></form>"
         "</body></html>");

  s.send(200, "text/html", h);
}

void handleSave(WebServer& s) {
  bool roomChanged = false, nameChanged = false;

  if (s.hasArg("room")) {
    String r = s.arg("room");
    r.trim();
    if (r != roomName) {
      strlcpy(roomName, r.c_str(), sizeof(roomName));
      applyRoomName();
      roomChanged = true;
    }
  }
  if (s.hasArg("wname")) {
    String n = s.arg("wname");
    if (n != wiimName) {
      strlcpy(wiimName, n.c_str(), sizeof(wiimName));
      nameChanged = true;
    }
  }
  if (strlen(wiimName)) {
    // IP aus der letzten Suche uebernehmen, sonst wird sie im Hintergrund gesucht
    for (int i = 0; i < foundCount; i++)
      if (strcasecmp(found[i].name, wiimName) == 0) strlcpy(wiimIp, found[i].ip, sizeof(wiimIp));
  } else if (s.hasArg("ip") && !setIpIfValid(s.arg("ip"))) {
    Serial.println("[Config] Ungueltige IP ignoriert.");
  }
  for (int k = 0; k < KEY_COUNT; k++) {
    String arg = "k" + String(k);
    if (s.hasArg(arg)) keyAction[k] = findAction(s.arg(arg));
  }
  if (s.hasArg("step")) volStep   = constrain(s.arg("step").toInt(), 1, 10);
  if (s.hasArg("max"))  maxVolume = constrain(s.arg("max").toInt(), 0, 100);
  saveSettings();
  activeKey  = -1;
  currentVol = -1;

  if (nameChanged && strlen(wiimName)) queueCmd("#resolve");
  if (nameChanged && !roomChanged && WiFi.status() == WL_CONNECTED) restartMdns();   // WiiM-Name in der Uebersicht
  if (roomChanged && WiFi.status() == WL_CONNECTED) {
    restartMdns();
    Serial.printf("[Config] Neuer Name: http://%s.local  (Setup-WLAN: %s)\n", hostName.c_str(), apName.c_str());
  }
  s.sendHeader("Location", "/m218?saved=1");
  s.send(303);
}

void handleTest(WebServer& s) {
  int k = constrain(s.arg("k").toInt(), 0, KEY_COUNT - 1);
  triggerKey(k, false);
  s.sendHeader("Location", "/m218?tested=" + String(k));
  s.send(303);
}

void handleScan(WebServer& s) {
  if (!scanBusy) {
    scanBusy = true;               // sofort setzen, damit die Seite "Suche ..." zeigt
    if (!queueCmd("#scan")) scanBusy = false;
  }
  s.sendHeader("Location", "/m218?scanning=1");
  s.send(303);
}

void factoryReset();

void handleReset(WebServer& s) {
  String h = F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'></head>"
         "<body style='font-family:sans-serif;padding:16px'><h2>Werkseinstellungen</h2>"
         "<p>Alles gel&ouml;scht. Die Bridge startet neu und &ouml;ffnet das Setup-WLAN "
         "<b>M218-WiiM-Bridge-");
  h += deviceId();
  h += F("</b> (Passwort m218wiim).</p></body></html>");
  s.send(200, "text/html", h);
  delay(500);
  factoryReset();
}

void registerConfigRoutes(WebServer& s) {
  s.on("/m218",      HTTP_GET,  [&s]() { sendConfigPage(s); });
  s.on("/m218/save", HTTP_POST, [&s]() { handleSave(s); });
  s.on("/m218/test", HTTP_GET,  [&s]() { handleTest(s); });
  s.on("/m218/scan", HTTP_GET,  [&s]() { handleScan(s); });
  s.on("/m218/reset", HTTP_POST, [&s]() { handleReset(s); });
  s.on("/m218/group", HTTP_GET, [&s]() { queueCmd("#group"); s.sendHeader("Location", "/m218"); s.send(303); });
  s.on("/m218/otacheck", HTTP_GET, [&s]() {
    queueCmd("#otacheck");
    s.sendHeader("Location", "/m218?scanning=1"); s.send(303);       // Seite laedt nach 4 s neu
  });
  s.on("/m218/otainstall", HTTP_POST, [&s]() {
    queueCmd("#otainstall");
    s.send(200, "text/html", F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='60;url=/m218'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'></head><body style='font-family:sans-serif;padding:16px'>"
           "<h2>Update l&auml;uft</h2><p>Die LED blinkt w&auml;hrend des Downloads. Danach startet die Bridge neu. "
           "Diese Seite l&auml;dt in einer Minute neu.</p></body></html>"));
  });
  // Notloesung ohne GitHub: .bin-Datei direkt vom PC hochladen
  s.on("/update", HTTP_GET, [&s]() {
    s.send(200, "text/html", F("<!DOCTYPE html><html><head><meta charset='utf-8'>"
           "<meta name='viewport' content='width=device-width,initial-scale=1'></head><body style='font-family:sans-serif;padding:16px;max-width:480px'>"
           "<h2>Firmware hochladen</h2><p>Datei <b>M218_WiiM_Bridge.ino.bin</b> w&auml;hlen (Arduino-IDE: Sketch &rarr; "
           "Kompilierte Bin&auml;rdatei exportieren).</p><form method='post' action='/update' enctype='multipart/form-data'>"
           "<input type='file' name='fw' accept='.bin'><br><br><button type='submit'>Hochladen und installieren</button></form>"
           "<p><a href='/m218'>zur&uuml;ck</a></p></body></html>"));
  });
  s.on("/update", HTTP_POST, [&s]() {
    bool ok = !Update.hasError();
    s.send(200, "text/html", ok ? F("<meta charset='utf-8'><meta http-equiv='refresh' content='20;url=/m218'>Update erfolgreich, Neustart &hellip;")
                                : F("<meta charset='utf-8'>Update fehlgeschlagen. <a href='/update'>Nochmals</a>"));
    delay(500);
    if (ok) ESP.restart();
  }, [&s]() {
    HTTPUpload& u = s.upload();
    if (u.status == UPLOAD_FILE_START) {
      Serial.printf("[Update] Datei %s wird hochgeladen ...\n", u.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (u.status == UPLOAD_FILE_WRITE) {
      if (Update.write(u.buf, u.currentSize) != u.currentSize) Update.printError(Serial);
      digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
    } else if (u.status == UPLOAD_FILE_END) {
      if (Update.end(true)) Serial.printf("[Update] %u Bytes geschrieben.\n", u.totalSize);
      else Update.printError(Serial);
    }
  });
  s.on("/m218/identify", HTTP_GET, [&s]() {          // LED 10 s blinken: welches Geraet ist das?
    ledFlashUntil = millis() + 10000;
    s.sendHeader("Location", "/m218"); s.send(303);
  });
  s.on("/m218/alloff", HTTP_GET, [&s]() { queueCmd("#alloff"); s.sendHeader("Location", "/m218"); s.send(303); });
  s.on("/m218/ungroup", HTTP_GET, [&s]() { queueCmd("#leave"); s.sendHeader("Location", "/m218"); s.send(303); });
}

// Webserver fuer den laufenden Betrieb
WebServer runtimeServer(80);

void startRuntimeServer() {
  runtimeServer.begin();
  restartMdns();
}

// =====================================================================
//  WLAN / Konfigurationsportal
// =====================================================================
void runWifiSetup(bool forcePortal) {
  runtimeServer.stop();   // Port 80 fuer das Portal freigeben

  char roomAtStart[sizeof(roomName)], nameAtStart[sizeof(wiimName)], ipAtStart[sizeof(wiimIp)];
  strlcpy(roomAtStart, roomName, sizeof(roomAtStart));
  strlcpy(nameAtStart, wiimName, sizeof(nameAtStart));
  strlcpy(ipAtStart, wiimIp, sizeof(ipAtStart));

  WiFiManager wm;
  WiFiManagerParameter pRoom("room", "Raumname (z.B. Bad)", roomName, sizeof(roomName) - 1);
  WiFiManagerParameter pName("wname", "WiiM-Name wie in der WiiM-App (leer = IP verwenden)", wiimName, sizeof(wiimName) - 1);
  WiFiManagerParameter pIp("wiimip", "WiiM IP-Adresse (nur ohne Name)", wiimIp, 15);
  wm.addParameter(&pRoom);
  wm.addParameter(&pName);
  wm.addParameter(&pIp);

  // Eigene Seite "M218 Tasten" ins Portal-Menue
  wm.setWebServerCallback([&wm]() { registerConfigRoutes(*wm.server); });
  std::vector<const char*> menu = { "wifi", "custom", "info", "exit" };
  wm.setMenu(menu);
  wm.setCustomMenuHTML("<form action='/m218' method='get'><button>M218 Tasten</button></form><br/>");

  wm.setHostname(hostName.c_str());
  wm.setConnectTimeout(20);
  wm.setConnectRetries(3);
  wm.setConfigPortalTimeout((!forcePortal && hasSavedWifi()) ? 60 : PORTAL_TIMEOUT_S);
  wm.setBreakAfterConfig(true);
  wm.setTitle(strlen(roomName) ? (String("M218 ") + roomName).c_str() : "M218 &rarr; WiiM");

  if (forcePortal) {
    Serial.printf("[WiFi] Konfigurationsportal geoeffnet: WLAN '%s'\n", apName.c_str());
    digitalWrite(STATUS_LED_PIN, HIGH);      // Dauerlicht = Portal offen
  }
  bool ok = forcePortal ? wm.startConfigPortal(apName.c_str(), AP_PASSWORD)
                        : wm.autoConnect(apName.c_str(), AP_PASSWORD);

  digitalWrite(STATUS_LED_PIN, LOW);

  // Werte aus dem WLAN-Formular nur uebernehmen, wenn dort geaendert
  bool changed = false;
  if (strcmp(pRoom.getValue(), roomAtStart) != 0) {
    String r = pRoom.getValue(); r.trim();
    strlcpy(roomName, r.c_str(), sizeof(roomName));
    applyRoomName();
    changed = true;
  }
  if (strcmp(pName.getValue(), nameAtStart) != 0) {
    String n = pName.getValue(); n.trim();
    strlcpy(wiimName, n.c_str(), sizeof(wiimName));
    changed = true;
  }
  if (strcmp(pIp.getValue(), ipAtStart) != 0) {
    if (setIpIfValid(pIp.getValue())) changed = true;
    else Serial.println("[Config] Ungueltige IP ignoriert.");
  }
  if (changed) saveSettings();

  if (ok && WiFi.status() == WL_CONNECTED) {
    markWifiSaved();
    Serial.printf("[WiFi] Verbunden mit '%s', Einstellungen: http://%s.local  oder  http://%s\n",
                  WiFi.SSID().c_str(), hostName.c_str(), WiFi.localIP().toString().c_str());
  } else {
    Serial.println("[WiFi] Keine Verbindung.");
  }

  startRuntimeServer();
  if (strlen(wiimName)) queueCmd("#resolve");
}

void checkWifi() {
  static uint32_t lostSince = 0;
  if ((int32_t)(ledFlashUntil - millis()) > 0) {           // Bestaetigung (z. B. Gruppe gebildet)
    digitalWrite(STATUS_LED_PIN, (millis() / 60) % 2);
    return;
  }
  if (WiFi.status() == WL_CONNECTED) { lostSince = 0; digitalWrite(STATUS_LED_PIN, LOW); return; }   // Normalbetrieb: dunkel
  digitalWrite(STATUS_LED_PIN, (millis() % 2000) < 100);   // ohne WLAN: kurzer Blitz alle 2 s
  if (!lostSince) {
    lostSince = millis();
    Serial.println("[WiFi] Verbindung verloren - versuche Reconnect ...");
  }
  if (millis() - lostSince > 5UL * 60UL * 1000UL) {
    Serial.println("[WiFi] Seit 5 Minuten kein WLAN - Neustart.");
    delay(500);
    ESP.restart();
  }
}

// =====================================================================
//  Taster: Setup-Portal / Werksreset
// =====================================================================
bool buttonPressed() {
  return digitalRead(SETUP_BUTTON_PIN) == LOW || digitalRead(CFG_BUTTON_PIN) == LOW;
}

void factoryReset() {
  Serial.println("[Reset] Werkseinstellungen: loesche WLAN, Raum, WiiM und Tasten ...");
  prefs.begin("m218", false);
  prefs.clear();
  prefs.end();
  WiFiManager wm;
  wm.resetSettings();                       // gespeicherte WLAN-Zugangsdaten loeschen
  digitalWrite(STATUS_LED_PIN, HIGH);       // 1 s Dauerlicht = Reset ausgefuehrt
  delay(1000);
  Serial.println("[Reset] Neustart.");
  ESP.restart();
}

// LED-Anzeige waehrend der Taster gehalten wird
void showHoldTime(uint32_t held) {
  if (held < PORTAL_HOLD_MS)     digitalWrite(STATUS_LED_PIN, (millis() / 500) % 2);  // langsam = Taster erkannt
  else if (held < WARN_HOLD_MS)  digitalWrite(STATUS_LED_PIN, (millis() / 200) % 2);  // mittel = loslassen -> Portal
  else                           digitalWrite(STATUS_LED_PIN, (millis() / 70) % 2);   // schnell = gleich Werksreset
}

// Im Betrieb: true, solange der Taster gehalten wird (dann steuert er die LED)
bool handleButton() {
  static uint32_t since = 0;
  if (buttonPressed()) {
    if (!since) {
      since = millis();
      Serial.printf("[Taster] gedrueckt (D13=%s, BOOT=%s): 2 s = Portal, 10 s = Werksreset\n",
                    digitalRead(SETUP_BUTTON_PIN) ? "HIGH" : "LOW", digitalRead(CFG_BUTTON_PIN) ? "HIGH" : "LOW");
    }
    uint32_t held = millis() - since;
    if (held >= RESET_HOLD_MS) factoryReset();
    showHoldTime(held);
    return true;
  }
  if (since) {
    uint32_t held = millis() - since;
    since = 0;
    if (held >= PORTAL_HOLD_MS) runWifiSetup(true);
    else if (held > 50) Serial.println("[Taster] zu kurz, nichts passiert");
  }
  return false;
}

// Beim Start: Front-Taster gehalten? true = Portal oeffnen (bei 10 s Werksreset)
// Nur D13: GPIO0 kann beim Start nicht gedrueckt sein (sonst Programmiermodus).
bool buttonAtBoot() {
  delay(50);                                         // Pull-up einschwingen lassen
  Serial.printf("[Taster] Pegel beim Start: D13=%s, BOOT(GPIO0)=%s\n",
                digitalRead(SETUP_BUTTON_PIN) ? "HIGH" : "LOW", digitalRead(CFG_BUTTON_PIN) ? "HIGH" : "LOW");
  for (int i = 0; i < 10; i++) {                     // 100 ms stabil LOW verlangen
    if (digitalRead(SETUP_BUTTON_PIN) == HIGH) return false;
    delay(10);
  }
  Serial.println("[Taster] beim Start gedrueckt: loslassen = Portal, 10 s halten = Werksreset");
  uint32_t t0 = millis();
  while (digitalRead(SETUP_BUTTON_PIN) == LOW) {
    uint32_t held = millis() - t0;
    if (held >= RESET_HOLD_MS) factoryReset();
    showHoldTime(held);
    delay(10);
  }
  return true;
}

// =====================================================================
//  Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("\n=== M218 -> WiiM Bridge (Version %s) ===\n", FW_VERSION);

  pinMode(CFG_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SETUP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  loadSettings();
  Serial.printf("[Config] Geraete-ID %s, Name %s.local, Setup-WLAN '%s'\n", deviceId().c_str(), hostName.c_str(), apName.c_str());
  Serial.printf("[Config] Raum '%s', WiiM '%s', IP %s\n", roomName, wiimName,
                strlen(wiimIp) ? wiimIp : "(keine)");

  cmdQueue = xQueueCreate(10, sizeof(Cmd));
  xTaskCreatePinnedToCore(wiimTask, "wiim", 16384, nullptr, 1, nullptr, 0);

  registerConfigRoutes(runtimeServer);
  runtimeServer.on("/", HTTP_GET, []() { sendOverviewPage(runtimeServer); });   // Uebersicht aller Bridges
  runtimeServer.on("/refresh", HTTP_GET, []() {
    queueCmd("#bridges");
    runtimeServer.sendHeader("Location", "/?refresh=1");
    runtimeServer.send(303);
  });

  bool forcePortal = buttonAtBoot();
  runWifiSetup(forcePortal);   // sucht danach den WiiM per Name (#resolve)

  // Router nach Stromausfall evtl. noch nicht bereit -> Neustart und erneut versuchen
  if (!forcePortal && WiFi.status() != WL_CONNECTED && hasSavedWifi()) {
    Serial.println("[WiFi] Gespeichertes WLAN nicht erreichbar - Neustart in 10 s ...");
    delay(10000);
    ESP.restart();
  }
  WiFi.setAutoReconnect(true);

  queueCmd("getStatusEx");

  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  Serial.printf("[IR] Bereit auf GPIO %d.  't' = Status, 's' = Suchen, 'p' = Portal\n", IR_PIN);
}

void loop() {
  handleIr();
  runtimeServer.handleClient();
  checkHoldRelease();
  otaMarkValid();
  if (!handleButton()) checkWifi();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't') queueCmd("getStatusEx");
    if (c == 's' && !scanBusy) { scanBusy = true; if (!queueCmd("#scan")) scanBusy = false; }
    if (c == 'p') runWifiSetup(true);
  }
}
