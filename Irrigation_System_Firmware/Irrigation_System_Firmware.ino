/*
  Irrigation_System_Firmware - A code to run an irrigation system
  Created by Sebastian G. Neuhausen, September 04, 2023.
*/

//Pindefinition von den Relais (outputs) (Preprocessor-Befehle)
#define PIN_RELAIS_1 11 //D11 (DIGITAL 11)
#define PIN_RELAIS_2 10 //D10 (DIGITAL 10)
#define PIN_RELAIS_3 9 //D9 (DIGITAL 9)
#define PIN_RELAIS_4 8 //D8 (DIGITAL 8)

//Pindefinition der Sensorik (inputs)
#define PIN_DURCHFLUSSSENSOR 2 //D2 (DIGITAL 2) - Interrupt-Funktion von Pin 2 wird genutzt
#define PIN_FEUCHTIGKEITSSENSOR_1 A0 //(ANALOG 0)
#define PIN_FEUCHTIGKEITSSENSOR_2 A1 //(ANALOG 1)
#define PIN_FEUCHTIGKEITSSENSOR_3 A2 //(ANALOG 2)
#define PIN_FEUCHTIGKEITSSENSOR_4 A3 //(ANALOG 3)

//Schwellenwertdefinitionen (allgemeingültig)
#define SCHWELLENWERT_TROCKEN 300 //Wertebereich: 0-1023; Ab diesem Messwert wird registriert, dass der Boden zu trocken ist und bewässert werden muss
#define SCHWELLENWERT_PUMPE_AUS 400 //Wertebereich: 0-1023; Ab diesem Messwert wird registriert, dass der Boden zu trocken ist und bewässert werden muss

//Zeitkonfigurationen (pro Pumpeinheit)
#define ZEIT_NACH_TROCKENHEIT_1 14400000 //Zeit in Millisekunden, die nach der Registrierung der Trockenheit vergehen soll, bis die Bewässerung aktiviert wird (Bei Pumpeinheit 1, 4h)
#define ZEIT_NACH_TROCKENHEIT_2 14400000 //Zeit in Millisekunden, die nach der Registrierung der Trockenheit vergehen soll, bis die Bewässerung aktiviert wird (Bei Pumpeinheit 2, 4h)
#define ZEIT_NACH_TROCKENHEIT_3 14400000 //Zeit in Millisekunden, die nach der Registrierung der Trockenheit vergehen soll, bis die Bewässerung aktiviert wird (Bei Pumpeinheit 3, 4h)
#define ZEIT_NACH_TROCKENHEIT_4 14400000 //Zeit in Millisekunden, die nach der Registrierung der Trockenheit vergehen soll, bis die Bewässerung aktiviert wird (Bei Pumpeinheit 4, 4h)
#define MESSINTERVAL_1 60000 //Messinterval in Millisekunden bei Pumpeinheit 1
#define MESSINTERVAL_2 60000 //Messinterval in Millisekunden bei Pumpeinheit 2
#define MESSINTERVAL_3 60000 //Messinterval in Millisekunden bei Pumpeinheit 3
#define MESSINTERVAL_4 60000 //Messinterval in Millisekunden bei Pumpeinheit 4

//Modis der State Machine
#define STATUS_MESSUNG 0
#define STATUS_WARTEN 1
#define STATUS_TROCKNEN 2
#define STATUS_PUMPEN 3

//Errormodis
#define ERROR_UNBEKANNT 5 //Unbekannter unerwarteter Fehler
#define ERROR_WASSERSTAND_LEER 2 //blinkt ein Mal kurz
#define ERROR_ZEITUEBERSCHREITUNG_PUMPE 3 //blinkt zwei Mal kurz

//Herzschlag-Definitionen
#define PIN_HERZ_LED 13 //Status-LED für den "Herzschlag" (falls das "Herz" nicht mehr schlägt, hat der Arduino einen Fehlerzustand)
#define HERZSCHLAGINTERVAL 750 //Zeit des Intervals in Millisekunden (0.5s = 500)

//Error-Definitionen
#define ERRORCOUNT 6 //Wenn mehr als diese Anzahl an Errors gemessen wurden -> in den Errorzustand gehen
#define PUMPE_TIMEOUT 30000 //Zeit in Millisekunden, nach welcher eine Pumpe ausgeschaltet werden soll, wenn die Sensoren zu lange trockenheit anzeigen (z.B. wenn etwas kaputt geht und die Pumpe ins leere pumpt)

unsigned long letzterZeitstempelBlink = 0; //Zeitstempel des letzten wechsels des LED-Lichts

struct pumpeinheit {
  unsigned short schwellenwertTrocken; //Wertebereich: 0-1023; Ab diesem Messwert wird registriert, dass der Boden zu trocken ist und bewässert werden muss
  unsigned short schwellenwertPumpeAus; //Wertebereich: 0-1023; Ab diesem Messwert wird registriert, dass der Boden zu trocken ist und bewässert werden muss
  unsigned long zeitNachTrockenheit; //Zeit in Sekunden, die nach der Registrierung der Trockenheit vergehen soll, bis die Bewässerung aktiviert wird 
  unsigned long messinterval; //Zeitinterval in Sekunden, in dem gemessen werden soll
  unsigned long letzteMessung; //Zeitstempel bei letzter Messung
  byte pinFeuchtigkeitssensor; //Pin des Feuchtigkeitssensors
  byte pinRelais; //Pin des Relais zur Pumpensteuerng
  byte prozessstatus; //Prozesstatus (State Machine)
  bool pumpeIstAn; //0, wenn diese Pumpstation nicht pumpt, 1, wenn die Pumpstation pumpt
};

struct pumpeinheit p1; //Initialisierung Pumpeinheit 1
struct pumpeinheit p2; //Initialisierung Pumpeinheit 2
struct pumpeinheit p3; //Initialisierung Pumpeinheit 3
struct pumpeinheit p4; //Initialisierung Pumpeinheit 4

volatile bool herzStatus = 0; //Status des Herzes (0 = kein Licht, 1 = Licht)
unsigned long letzterHerzschlag = 0; //Zeitstempel des letzten Herzschlags

volatile unsigned long durchflussImpulszaehler = 0; //Zaehlt die Anzahl der Pulse am Durchflusssensor
volatile unsigned long durchflussImpulszaehlerZwischenspeicher = 0; //Zaehlt die Anzahl der Pulse am Durchflusssensor
bool durchflusszaehlerAktiviert = 0; //Aktivieren und deaktivieren vom Zaehler des Durchflusssensors
bool pumpeIstAn = 0; //Status, ob eine Pumpe momentan im Einsatz ist (1 = ja, 0 = nein)
unsigned long zeitstempelPumpeAn = 0; //Zeitstempel der letzten Aktivierung der Pumpe

byte error = 0; //Wenn >0, dann herrscht ein Fehlerzustand
byte errorZaehler = 0; //Zaehler von Errorzustaenden waehrend Herzschlag

void setup() {
  //Konfiguration der Sensorpins (Analogpins) als Input
  pinMode(PIN_FEUCHTIGKEITSSENSOR_1, INPUT_PULLUP); //Wenn ein Sensor nicht funktionieren sollte oder die Verbindung getrennt wird, zeigt der Sensor immer an, dass die Erde nass ist, damit keine Pumpe gestartet wird
  pinMode(PIN_FEUCHTIGKEITSSENSOR_2, INPUT_PULLUP); //Wenn ein Sensor nicht funktionieren sollte oder die Verbindung getrennt wird, zeigt der Sensor immer an, dass die Erde nass ist, damit keine Pumpe gestartet wird
  pinMode(PIN_FEUCHTIGKEITSSENSOR_3, INPUT_PULLUP); //Wenn ein Sensor nicht funktionieren sollte oder die Verbindung getrennt wird, zeigt der Sensor immer an, dass die Erde nass ist, damit keine Pumpe gestartet wird
  pinMode(PIN_FEUCHTIGKEITSSENSOR_4, INPUT_PULLUP); //Wenn ein Sensor nicht funktionieren sollte oder die Verbindung getrennt wird, zeigt der Sensor immer an, dass die Erde nass ist, damit keine Pumpe gestartet wird

  //Konfiguration der Relaispins (Digitalpins) als Output
  pinMode(PIN_RELAIS_1, OUTPUT);
  pinMode(PIN_RELAIS_2, OUTPUT);
  pinMode(PIN_RELAIS_3, OUTPUT);
  pinMode(PIN_RELAIS_4, OUTPUT);
  
  //Interrupt für Durchflusssensor konfigurieren
  pinMode(PIN_DURCHFLUSSSENSOR, INPUT_PULLUP); //Wenn der Durchflusssensor nicht funktioniert oder keine Verbindung hat, werden Stoersignale vermieden
  attachInterrupt(digitalPinToInterrupt(PIN_DURCHFLUSSSENSOR), durchflussZaehlen, CHANGE); //Pin an Interrupt anschliessen (Code wird beim Signalwechsel von 0 zu 1 oder 1 zu 0 unterbrochen und ein Zähler wird aktiviert)
  
  //Konfiguration des Herzschlages
  pinMode(PIN_HERZ_LED, OUTPUT);

  //Werte für structs setzen 
  //Pumpeinheit 1
  p1.schwellenwertTrocken = SCHWELLENWERT_TROCKEN;
  p1.schwellenwertPumpeAus = SCHWELLENWERT_PUMPE_AUS;
  p1.zeitNachTrockenheit = ZEIT_NACH_TROCKENHEIT_1;
  p1.messinterval = MESSINTERVAL_1;
  p1.letzteMessung = 0;
  p1.pinFeuchtigkeitssensor = PIN_FEUCHTIGKEITSSENSOR_1;
  p1.pinRelais = PIN_RELAIS_1;
  p1.prozessstatus = 0;
  p1.pumpeIstAn = 0;

  //Pumpeinheit 2
  p2.schwellenwertTrocken = SCHWELLENWERT_TROCKEN;
  p2.schwellenwertPumpeAus = SCHWELLENWERT_PUMPE_AUS;
  p2.zeitNachTrockenheit = ZEIT_NACH_TROCKENHEIT_2;
  p2.messinterval = MESSINTERVAL_2;
  p2.letzteMessung = 0;
  p2.pinFeuchtigkeitssensor = PIN_FEUCHTIGKEITSSENSOR_2;
  p2.pinRelais = PIN_RELAIS_2;
  p2.prozessstatus = 0;
  p2.pumpeIstAn = 0;

  //Pumpeinheit 3
  p3.schwellenwertTrocken = SCHWELLENWERT_TROCKEN;
  p3.schwellenwertPumpeAus = SCHWELLENWERT_PUMPE_AUS;
  p3.zeitNachTrockenheit = ZEIT_NACH_TROCKENHEIT_3;
  p3.messinterval = MESSINTERVAL_3;
  p3.letzteMessung = 0;
  p3.pinFeuchtigkeitssensor = PIN_FEUCHTIGKEITSSENSOR_3;
  p3.pinRelais = PIN_RELAIS_3;
  p3.prozessstatus = 0;
  p3.pumpeIstAn = 0;

  //Pumpeinheit 4
  p4.schwellenwertTrocken = SCHWELLENWERT_TROCKEN;
  p4.schwellenwertPumpeAus = SCHWELLENWERT_PUMPE_AUS;
  p4.zeitNachTrockenheit = ZEIT_NACH_TROCKENHEIT_4;
  p4.messinterval = MESSINTERVAL_4;
  p4.letzteMessung = 0;
  p4.pinFeuchtigkeitssensor = PIN_FEUCHTIGKEITSSENSOR_4;
  p4.pinRelais = PIN_RELAIS_4;
  p4.prozessstatus = 0;
  p4.pumpeIstAn = 0;
  
  //Seriellen Monitor aktivieren
  Serial.begin(9600);
}

void loop() {
  herzschlag(); //Herzschlag abrufen
  pumpstationAusfuehren(&p1); //Ausfuehren von Pumpstation 1
  pumpstationAusfuehren(&p2); //Ausfuehren von Pumpstation 2
  pumpstationAusfuehren(&p3); //Ausfuehren von Pumpstation 3
  pumpstationAusfuehren(&p4); //Ausfuehren von Pumpstation 4
  //datenAusgeben(); //Daten ausgeben, wenn das Geraet per USB verbunden ist
  pruefeError(); //Wenn ein Error detektiert wird, wird hier unterbrochen
}

void datenAusgeben() { //Daten über die Serielle Schnittstelle zum seriellen Monitor ausgeben
  if (Serial) { //... Wenn das Geraet per USB-Kabel verbunden ist
    Serial.println("--------------------------------");
    Serial.print("Durchflusssensor:");
    Serial.println(digitalRead(PIN_DURCHFLUSSSENSOR));
    Serial.print("Feuchtigkeitssensor 1:");
    Serial.println(analogRead(p1.pinFeuchtigkeitssensor));
    Serial.print("Feuchtigkeitssensor 2:");
    Serial.println(analogRead(p2.pinFeuchtigkeitssensor));
    Serial.print("Feuchtigkeitssensor 3:");
    Serial.println(analogRead(p3.pinFeuchtigkeitssensor));
    Serial.print("Feuchtigkeitssensor 4:");
    Serial.println(analogRead(p4.pinFeuchtigkeitssensor));
    Serial.print("Error-Zaehler:");
    Serial.println(errorZaehler);
    Serial.println("--------------------------------");
    Serial.println();
  }
}

void pumpstationAusfuehren(struct pumpeinheit* p) { //Uebergeben der Pumpeinheit-Variable
  volatile short messwert = analogRead(p->pinFeuchtigkeitssensor);
  switch(p->prozessstatus) { //Je nach Status andere Aktionen ausfuehren
    case STATUS_MESSUNG:
      if (p->pumpeIstAn) { //Pruefen, ob die Pumpe an ist
        if (messwert >= p->schwellenwertPumpeAus) {
          pumpeAus(p->pinRelais);
          p->pumpeIstAn = 0;
        }
      } else {
        if (messwert <= p->schwellenwertTrocken) {
          p->letzteMessung = millis();
          p->prozessstatus = STATUS_TROCKNEN;
        } else {
          p->letzteMessung = millis();
          p->prozessstatus = STATUS_WARTEN;
        }
      }
      break;
    case STATUS_WARTEN: //Eine gewisse Zeit abwarten
      if (millis() >= p->letzteMessung + p->messinterval) {
        p->prozessstatus = STATUS_MESSUNG;
      }
      break;
    case STATUS_TROCKNEN: //Trocknen lassen
      if (millis() >= p->letzteMessung + p->zeitNachTrockenheit) {
        p->prozessstatus = STATUS_PUMPEN;
      }
      break;
    case STATUS_PUMPEN: //Pumpe einschalten
      if (!pumpeIstAn) {
        pumpeAn(p->pinRelais);
        p->prozessstatus = STATUS_MESSUNG;
        p->pumpeIstAn = 1;
      }
      break;
    default:
      p->prozessstatus = STATUS_MESSUNG;
      error = ERROR_UNBEKANNT;
      break;
  }
}

void durchflussZaehlen() {
  if (durchflusszaehlerAktiviert)
    durchflussImpulszaehler++;
}

void herzschlag() {
  unsigned long zeitstempel = millis(); //Aktuelle Zeit abrufen
  if (letzterHerzschlag + HERZSCHLAGINTERVAL <= zeitstempel) { //Wenn eine Zeit überschritten wird
    herzStatus = !herzStatus; //Herzstatus umschalten (0 zu 1 und 1 zu 0)
    digitalWrite(PIN_HERZ_LED, herzStatus); //Herzstatus schreiben

    //Pruefen, ob Zaehler funktioniert, wenn aktiv
    if (durchflusszaehlerAktiviert && durchflussImpulszaehler <= durchflussImpulszaehlerZwischenspeicher) { //Pruefen, ob sichd ie Anzahl der Impulse erhoeht hat
      errorZaehler++; //Error raufzaehlen
    } else {
      errorZaehler = 0;
    }
    durchflussImpulszaehlerZwischenspeicher = durchflussImpulszaehler; //Anzahl an Impulsen zwischenspeichern

    letzterHerzschlag = millis(); //Neuen Zeitstembel der aktuellen Zeit schreiben
  }
}

void pumpeAn(byte pin) {
  digitalWrite(pin, 0);
  durchflusszaehlerAktiviert = 1;
  durchflussImpulszaehlerZwischenspeicher = 0;
  durchflussImpulszaehler = 0;
  pumpeIstAn = 1;
  zeitstempelPumpeAn = millis();
}
void pumpeAus(byte pin) {
  digitalWrite(pin, 1);
  durchflusszaehlerAktiviert = 0;
  durchflussImpulszaehlerZwischenspeicher = 0;
  durchflussImpulszaehler = 0;
  pumpeIstAn = 0;
  zeitstempelPumpeAn = 0;
}

void pruefeError() {
  if (errorZaehler >= ERRORCOUNT) //Wenn zu viele Fehler gezaehlt wurden
    error = ERROR_WASSERSTAND_LEER;  //Errormodus aktivieren (Durchflusssensor zaehlt nicht, wahrscheinlich ist das Wasser leer)
  if (pumpeIstAn)
    if (millis() > zeitstempelPumpeAn+PUMPE_TIMEOUT)
      error = ERROR_ZEITUEBERSCHREITUNG_PUMPE; //Errormodus aktivieren (Pumpe pumpt zu lange, wahrscheinlich ist ein Schlauch getrennt oder nicht im Topf)
  if (error > 0) {
    pumpeAus(PIN_RELAIS_1); //Relais 1 ausschalten
    pumpeAus(PIN_RELAIS_2); //Relais 2 ausschalten
    pumpeAus(PIN_RELAIS_3); //Relais 3 ausschalten
    pumpeAus(PIN_RELAIS_4); //Relais 4 ausschalten
    digitalWrite(PIN_HERZ_LED, 1); //Herz bleibt stehen (leuchtet)
    pumpeIstAn = 0;
    while(true) { //Alles anhalten und errorcode anzeigen -> Neustart erforderlich
      for (byte i = 0; i < error; i++) {
        digitalWrite(PIN_HERZ_LED, 1);
        delay(200);
        digitalWrite(PIN_HERZ_LED, 0);
        delay(200);
      }
      delay(2800);
    }
  }
}

/*Zu pruefen:
Prozess von p1
Unangeschlossener Zustand
Errorcode 1
Errorcode 2
*/



