# 🤖 Lijnvolgende Robot met ESP32

> Eindwerk Thomas More 2024–2025  
> Door: Haico Luyckx & Jonathan De Geyter

Een autonome lijnvolgende robot die obstakels ontwijkt, communiceert via MQTT en een dashboard gebruikt op een Raspberry Pi 5. De robot werkt met IR-sensoren, een ultrasoon sensor op een servomotor, en toont batterijstatus via LED’s en MQTT.

---

## 📁 Mappenstructuur

| Map            | Inhoud                                                   |
|----------------|----------------------------------------------------------|
| `/code/`       | Arduino/ESP32-code (`.ino`-bestand)                      |
| `/3d-prints/`  | STL-bestanden voor geprinte onderdelen                   |
| `/pcb/`        | PCB-ontwerpen                                            |
| `/images/`     | Foto’s van het project + screenshots van dashboard       |
| `/videos/`     | Demonstratievideo’s van het eindresultaat                |
| `/datasheets/` | Datasheets van gebruikte componenten                     |

> 📄 Zie `Documentatie_eindwerk_HaicoJonathan.docx` voor volledige uitleg over hardware, software en opbouw.

---

## ⚙️ Hardwarecomponenten

- ESP-WROOM-32 Dev Board
- HC-SR04 ultrasoon sensor + servomotor
- 2x IR-lijnvolgsensoren
- L298N H-Bridge motorcontroller
- 7.2V NiMH batterij + spanningsdeler
- DC Buck Converter
- Raspberry Pi 5 (Node-RED + MQTT Broker)
- Startknop

---

## 🚦 Belangrijkste functies

- Volgt automatisch een zwarte lijn met IR-sensoren.
- Vermijdt obstakels via servo + HC-SR04.
- Batterijstatus wordt via MQTT gepubliceerd én weergegeven op dashboard.
- Robotstatus wordt gesynchroniseerd met dashboardknop.
- Automatische stop bij kritieke batterij (<10%).

---

## 🛰️ MQTT Topics

| Topic               | Payload Type | Voorbeeld                          |
|--------------------|--------------|------------------------------------|
| `robot/status`     | String       | `"RIJDEND"` / `"GESTOPT"` / ...    |
| `robot/battery`    | JSON         | `{ "voltage": 7.4, "pct": 65 }`    |
| `robot/remoteButton` | Boolean    | `"true"`                           |
| `robot/distance`   | JSON         | `{ "distance": 25, "unit": "cm" }` |

---

## 📊 Node-RED Dashboard

- Start/stop-knop voor robot op afstand
- Batterijstatus (percentage en waarschuwing)
- Robotstatus in realtime
- MQTT-verbindingsstatus

> Screenshot beschikbaar in `/images/dashboard.png`  
> Flowbestand voor import: `flow.json`

---

## 🔧 Installatie

1. **Clone deze repository:**

   ```bash
   git clone https://github.com/<jouw-gebruikersnaam>/<repo-naam>.git

2. **ESP32 configureren:**
   - Open `code/main.ino` in Arduino IDE.
   - Vul je WiFi-gegevens en IP van MQTT-broker in.
   - Upload naar ESP32.

3. **Raspberry Pi setup:**
   - Installeer Mosquitto & Node-RED.
   - Voeg dashboard nodes toe.
   - Start Node-RED (`http://<ip>:1880`).
   - Importeer `flow.json`.

---

## 🧪 Testen

- Start robot via fysieke knop of dashboard.
- Verifieer lijnvolging + obstakeldetectie.
- Bekijk live status en batterijgegevens op dashboard.
- Test automatische stop bij lage batterij.

---

## 📄 Documentatie

De volledige documentatie met:
- Hardwarebeschrijving
- Gedetailleerde uitleg over de code
- MQTT-configuratie
- Handleiding en foutenoplossing  
vind je in het bestand:

📘 `Documentatie_eindwerk_HaicoJonathan.docx`

---

## 📬 Contact

Voor vragen of opmerkingen:  
📧 haico.luyckx@student.thomasmore.be  
📧 jonathan.degeyter@student.thomasmore.be
