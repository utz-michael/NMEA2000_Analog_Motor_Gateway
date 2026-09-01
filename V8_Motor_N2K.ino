// ============================================================
// V8 Motor NMEA2000 Sensorik für Arduino Mega 2560 + MCP2515
// - RPM über Interrupt (Pin 2)          -> PGN 127488 (Rapid Engine Data)
// - Öldruck (PSI) + Kühlwassertemp (°F) -> PGN 127489 (Engine Dynamic Parameters)
// - Kraftstoff-Füllstand (Liter)        -> PGN 127505 (Fluid Level)
//
// Benötigte Library: ttlappalainen/NMEA2000 + ttlappalainen/CAN_BUS_Shield (mcp_can)
// Hinweis: N2kCZone-Dateien müssen aus der NMEA2000-Library entfernt sein
//          (AVR-Toolchain-Kompatibilität, siehe frühere Projektnotizen).
// ============================================================

#include <Arduino.h>
#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA2000_mcp.h>

// --- CAN / NMEA2000 Setup (MCP2515) ---
const int N2K_SPI_CS_PIN = 53;   // ggf. an dein Board anpassen
const int N2K_CAN_INT_PIN = 21;  // ggf. an dein Board anpassen
tNMEA2000_mcp NMEA2000(N2K_SPI_CS_PIN, MCP_8MHz, N2K_CAN_INT_PIN);

// --- RPM Interrupt (Pin 2) ---
const int PIN_RPM_INTERRUPT = 2;
volatile unsigned long rpmImpulsZaehler = 0;
unsigned long letzteRpmBerechnung = 0;
double aktuelleRPM = 0;
const int IMPULSE_PRO_UMDREHUNG = 1; // ggf. anpassen (z.B. 1 Impuls pro Zündung/Zylinder je nach Abgriff)

void rpmInterruptHandler() {
  rpmImpulsZaehler++;
}

// --- Analoge Sensor-Pins ---
const int PIN_OELDRUCK   = A0;
const int PIN_FUELLSTAND = A1;
const int PIN_TEMPERATUR = A2;

// --- Tankgröße für Füllstand-Prozentberechnung (PGN 127505 erwartet %) ---
const double TANK_KAPAZITAET_LITER = 200.0; // an echten Tank anpassen!

// --- Motor-/Tank-Instanzen (bei mehreren Motoren/Tanks hochzählen) ---
const unsigned char ENGINE_INSTANCE = 0;
const unsigned char FUEL_TANK_INSTANCE = 0;

// --- Sendeintervalle ---
unsigned long letzterRpmVersand = 0;
unsigned long letzterDynamicVersand = 0;
unsigned long letzterFluidVersand = 0;
const unsigned long INTERVALL_RPM_MS = 100;      // PGN127488: schnell
const unsigned long INTERVALL_DYNAMIC_MS = 500;  // PGN127489
const unsigned long INTERVALL_FLUID_MS = 2000;   // PGN127505: langsam reicht

// ============================================================
// Umrechnungsfunktionen Sensor-Rohwert -> physikalische Einheit
// ============================================================

// Öldruck in PSI (Kalibrierung: x=199->0psi, x=96->40psi, x=33->80psi)
double berechneOeldruckPSI(double x) {
  double druck = 0.0015 * x * x - 0.8265 * x + 105.66;
  if (druck < 0) druck = 0;
  return druck;
}

// Kraftstoff-Füllstand in Litern (x=199->0L, x=96->100L, x=33->200L)
double berechneFuellstandLiter(double x) {
  double fuellstand = 0.0037 * x * x - 2.0663 * x + 264.15;
  if (fuellstand < 0) fuellstand = 0;
  if (fuellstand > TANK_KAPAZITAET_LITER) fuellstand = TANK_KAPAZITAET_LITER;
  return fuellstand;
}

// Kühlwassertemperatur in °F (x=318->100F, x=92->175F, x=29->250F)
double berechneTemperaturF(double x) {
  if (x < 1) x = 1; // Schutz vor log(0)
  double temp = -62.61 * log(x) + 459.9;
  return temp;
}

// ============================================================
// Einheiten-Konvertierung für NMEA2000 (SI-Einheiten Pflicht!)
// ============================================================

// PSI -> Pascal (NMEA2000 verlangt Pascal für Öldruck)
double psiZuPascal(double psi) {
  return psi * 6894.757;
}

// Fahrenheit -> Kelvin (NMEA2000 verlangt Kelvin für Temperaturen)
double fahrenheitZuKelvin(double f) {
  double celsius = (f - 32.0) * 5.0 / 9.0;
  return celsius + 273.15;
}

// Liter -> Prozent der Tankkapazität (PGN127505 erwartet Level in %)
double literZuProzent(double liter, double kapazitaet) {
  if (kapazitaet <= 0) return 0;
  double prozent = (liter / kapazitaet) * 100.0;
  if (prozent > 100.0) prozent = 100.0;
  if (prozent < 0.0) prozent = 0.0;
  return prozent;
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);

  // RPM Interrupt
  pinMode(PIN_RPM_INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_INTERRUPT), rpmInterruptHandler, FALLING);

  // NMEA2000 Setup
  NMEA2000.SetProductInformation(
    "00000001",             // Seriennummer
    100,                    // Produkt-Code
    "V8 Engine Monitor",    // Modellbezeichnung
    "1.0.0.0",              // Software-Version
    "1.0.0.0"               // Modell-Version
  );

  NMEA2000.SetDeviceInformation(
    1,    // Eindeutige Nummer, bei mehreren Geräten hochzählen
    160,  // Function Code: Engine Gateway
    50,   // Device Class: Propulsion
    2046  // Hersteller-Code (2046 = "reserved / testing", ggf. anpassen)
  );

  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22); // Adresse 22, ggf. anpassen
  NMEA2000.EnableForward(false);
  NMEA2000.Open();

  Serial.println("NMEA2000 V8 Motor Monitor gestartet.");
}

// ============================================================
// RPM Berechnung aus Impulsen
// ============================================================
void aktualisiereRPM() {
  unsigned long jetzt = millis();
  unsigned long intervall = jetzt - letzteRpmBerechnung;

  if (intervall >= 1000) { // jede Sekunde RPM neu berechnen
    noInterrupts();
    unsigned long impulse = rpmImpulsZaehler;
    rpmImpulsZaehler = 0;
    interrupts();

    double umdrehungenProSekunde = (double)impulse / IMPULSE_PRO_UMDREHUNG / (intervall / 1000.0);
    aktuelleRPM = umdrehungenProSekunde * 60.0;

    letzteRpmBerechnung = jetzt;
  }
}

// ============================================================
// N2K Nachrichten senden
// ============================================================
void sendeRpmNachricht() {
  tN2kMsg N2kMsg;
  SetN2kEngineParamRapid(N2kMsg, ENGINE_INSTANCE, aktuelleRPM);
  NMEA2000.SendMsg(N2kMsg);
}

void sendeEngineDynamicNachricht(double oeldruckPSI, double temperaturF) {
  tN2kMsg N2kMsg;

  double oeldruckPascal = psiZuPascal(oeldruckPSI);
  double tempKelvin = fahrenheitZuKelvin(temperaturF);

  SetN2kEngineDynamicParam(
    N2kMsg,
    ENGINE_INSTANCE,
    oeldruckPascal,       // EngineOilPress (Pa)
    N2kDoubleNA,          // EngineOilTemp (nicht gemessen)
    tempKelvin,           // EngineCoolantTemp (K)
    N2kDoubleNA,          // AlternatorVoltage (nicht gemessen)
    N2kDoubleNA,          // FuelRate (nicht gemessen)
    N2kDoubleNA,          // EngineHours (nicht gemessen)
    N2kDoubleNA,          // EngineCoolantPress
    N2kDoubleNA,          // EngineFuelPress
    N2kInt8NA,            // EngineLoad
    N2kInt8NA             // EngineTorque
  );

  NMEA2000.SendMsg(N2kMsg);
}

void sendeFluidLevelNachricht(double fuellstandLiter) {
  tN2kMsg N2kMsg;
  double prozent = literZuProzent(fuellstandLiter, TANK_KAPAZITAET_LITER);

  SetN2kFluidLevel(
    N2kMsg,
    FUEL_TANK_INSTANCE,
    N2kft_Fuel,               // Flüssigkeitstyp: Kraftstoff (Benzin)
    prozent,                  // Füllstand in %
    TANK_KAPAZITAET_LITER     // Tankkapazität in Litern
  );

  NMEA2000.SendMsg(N2kMsg);
}

// ============================================================
// Loop
// ============================================================
void loop() {
  NMEA2000.ParseMessages();
  aktualisiereRPM();

  unsigned long jetzt = millis();

  // --- RPM senden (schnell) ---
  if (jetzt - letzterRpmVersand >= INTERVALL_RPM_MS) {
    sendeRpmNachricht();
    letzterRpmVersand = jetzt;
  }

  // --- Öldruck + Kühlwassertemp senden ---
  if (jetzt - letzterDynamicVersand >= INTERVALL_DYNAMIC_MS) {
    double rohOeldruck = analogRead(PIN_OELDRUCK);
    double rohTemperatur = analogRead(PIN_TEMPERATUR);

    double oeldruckPSI = berechneOeldruckPSI(rohOeldruck);
    double temperaturF = berechneTemperaturF(rohTemperatur);

    sendeEngineDynamicNachricht(oeldruckPSI, temperaturF);

    Serial.print("RPM: ");
    Serial.print(aktuelleRPM, 0);
    Serial.print("  Oeldruck: ");
    Serial.print(oeldruckPSI, 1);
    Serial.print(" PSI  Kuehlwasser: ");
    Serial.print(temperaturF, 1);
    Serial.println(" F");

    letzterDynamicVersand = jetzt;
  }

  // --- Füllstand senden (langsam) ---
  if (jetzt - letzterFluidVersand >= INTERVALL_FLUID_MS) {
    double rohFuellstand = analogRead(PIN_FUELLSTAND);
    double fuellstandLiter = berechneFuellstandLiter(rohFuellstand);

    sendeFluidLevelNachricht(fuellstandLiter);

    Serial.print("Fuellstand: ");
    Serial.print(fuellstandLiter, 1);
    Serial.println(" L");

    letzterFluidVersand = jetzt;
  }
}
