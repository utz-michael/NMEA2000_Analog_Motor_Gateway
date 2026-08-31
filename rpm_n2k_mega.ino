/*
 * ============================================================================
 *  Arduino Mega 2560 – Drehzahlmessung V8-Motor via Interrupt an D2
 *  Ausgabe als NMEA2000 (PGN 127488 - Engine Parameters, Rapid Update)
 *  CAN-Interface: MCP2515 (SPI)
 * ============================================================================
 *
 * Messprinzip:
 *  - Interrupt (FALLING) auf D2 bei jedem Zündimpuls
 *  - Zeit zwischen zwei Impulsen wird per micros() gemessen
 *  - RPM = 60 / (Periodendauer_s * Impulse_pro_Umdrehung)
 *
 * Benötigte Bibliotheken (über Bibliotheksverwalter installieren):
 *  - NMEA2000              (ttlappalainen/NMEA2000)
 *  - NMEA2000_mcp2515       (ttlappalainen/NMEA2000_mcp2515)
 *  - mcp_can                (coryjfowler/MCP_CAN_lib, Abhängigkeit der NMEA2000_mcp2515)
 *
 * Verkabelung MCP2515 <-> Mega 2560 (Hardware-SPI):
 *  MOSI -> Pin 51
 *  MISO -> Pin 50
 *  SCK  -> Pin 52
 *  CS   -> Pin 53 (frei wählbar, siehe MCP2515_CS_PIN)
 *  INT  -> Pin 21 (frei wählbar, siehe MCP2515_INT_PIN, muss interruptfähig sein)
 *  VCC  -> 5V (MCP2515-Module haben meist eigenen 5V->3.3V/5V passenden TJA1050 Transceiver)
 *  GND  -> GND
 *
 * Drehzahlsignal:
 *  D2 (INT0) <- Tachosignal (z.B. Zündspule Klemme 1/negativ über Vorwiderstand/Optokoppler,
 *               NIEMALS direkt Zündspannung an den Arduino!)
 *
 * ACHTUNG SICHERHEIT:
 *  Das Rohsignal von der Zündspule liegt im Bereich mehrerer hundert Volt.
 *  Es MUSS über eine geeignete Signalaufbereitung (z.B. Optokoppler-Schaltung
 *  oder fertiges Tacho-Signal-Interface) auf sauberen 5V/3.3V-Pegel gebracht
 *  werden, bevor es an D2 angeschlossen wird.
 * ============================================================================
 */

#include <Arduino.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <NMEA2000_mcp2515.h>
#include <N2kMessages.h>

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------
#define RPM_PIN             2     // D2 = INT0 auf dem Mega 2560
#define MCP2515_CS_PIN      53    // Chip-Select MCP2515
#define MCP2515_INT_PIN     21    // INT-Pin MCP2515

// Impulse pro Kurbelwellenumdrehung:
// V8 mit klassischem Zündspulen-Tachosignal (Zündfolge über 2 KW-Umdrehungen):
//   8 Zylinder / 2 = 4 Impulse pro Umdrehung
// Bei Kurbelwellensensor mit Geberrad: Zähnezahl des Rades eintragen (z.B. 36, 60-2 etc.)
#define PULSES_PER_REV       4.0

// Kein Impuls länger als diese Zeit -> Motor steht, RPM = 0
const unsigned long PULSE_TIMEOUT_US = 1500000UL;   // 1,5 s

// Entprellung: Impulse, die schneller aufeinanderfolgen, werden verworfen.
// 2000us Minimalabstand entspricht bei 4 Imp/U einer Grenze von ca. 7500 RPM.
// Bei Bedarf (höhere Maximaldrehzahl) verringern.
const unsigned long MIN_PULSE_INTERVAL_US = 2000UL;

const unsigned long SEND_INTERVAL_MS = 100;   // NMEA2000 Rapid-Update: alle 100ms senden

// ---------------------------------------------------------------------------
// Globale Variablen (von ISR beschrieben -> volatile)
// ---------------------------------------------------------------------------
volatile unsigned long lastPulseTime_us = 0;
volatile unsigned long lastPeriod_us    = 0;

// ---------------------------------------------------------------------------
// NMEA2000-Objekt
// ---------------------------------------------------------------------------
tNMEA2000_mcp2515 NMEA2000(MCP2515_CS_PIN, MCP2515_INT_PIN);

const unsigned long TransmitMessages[] PROGMEM = { 127488L, 0 };

const unsigned char EngineInstance = 0;   // 0 = einzige/erste Maschine

// ---------------------------------------------------------------------------
// Interrupt Service Routine
// ---------------------------------------------------------------------------
void ISR_RpmPulse() {
  unsigned long now   = micros();
  unsigned long delta = now - lastPulseTime_us;

  if (delta < MIN_PULSE_INTERVAL_US) {
    return;   // Prellen / Störimpuls ignorieren
  }

  lastPeriod_us    = delta;
  lastPulseTime_us = now;
}

// ---------------------------------------------------------------------------
// RPM aus letzter gemessener Periodendauer berechnen
// ---------------------------------------------------------------------------
double GetEngineRPM() {
  unsigned long period, lastTime;

  noInterrupts();
  period   = lastPeriod_us;
  lastTime = lastPulseTime_us;
  interrupts();

  if (period == 0) {
    return 0.0;   // noch nie ein gültiger Impuls
  }

  unsigned long sinceLast = micros() - lastTime;
  if (sinceLast > PULSE_TIMEOUT_US) {
    return 0.0;   // Motor steht / kein Signal mehr
  }

  double revsPerSecond = (1000000.0 / (double)period) / PULSES_PER_REV;
  return revsPerSecond * 60.0;
}

// ---------------------------------------------------------------------------
// NMEA2000 PGN 127488 senden
// ---------------------------------------------------------------------------
void SendN2kEngineRapid(double rpm) {
  tN2kMsg N2kMsg;
  SetN2kEngineParamRapid(N2kMsg, EngineInstance,
                          rpm,
                          N2kDoubleNA,   // EngineBoostPressure - hier nicht gemessen
                          N2kInt8NA);    // EngineTiltTrim - hier nicht gemessen
  NMEA2000.SendMsg(N2kMsg);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(RPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), ISR_RpmPulse, FALLING);

  NMEA2000.SetProductInformation(
    "00000001",               // Seriennummer (frei wählbar, eindeutig im Netz)
    100,                       // Produktcode
    "V8 RPM Sensor",           // Modellbezeichnung
    "1.0.0 (2026-08-31)",      // Software-Version
    "1.0.0"                    // Modellversion
  );

  NMEA2000.SetDeviceInformation(
    1,        // Eindeutige Geraetenummer im NMEA2000-Netz (bei mehreren eigenen Geraeten anpassen!)
    160,      // Geraetefunktion: Engine (Tachometer/Gauge waere 130, hier: Engine Gateway)
    50,       // Geraeteklasse: Propulsion
    2046      // Herstellercode (2046 = reserviert fuer Test/Eigenbau)
  );

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);   // Adresse 22 als Startwert, ClaimAdress regelt Rest
  NMEA2000.EnableForward(false);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();

  Serial.println(F("Setup abgeschlossen. Warte auf Impulse an D2..."));
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
unsigned long lastSend = 0;

void loop() {
  NMEA2000.ParseMessages();

  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;

    double rpm = GetEngineRPM();
    SendN2kEngineRapid(rpm);

    Serial.print(F("RPM: "));
    Serial.println(rpm, 1);
  }
}
