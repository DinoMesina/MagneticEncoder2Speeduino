/*
 *   Copyright (C) 2026 Dino del Favero <dino@mesina.net>
 *   
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 3 of the License, or
 *   (at your option) any later version.
 *   
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *   
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 */


#include <SPI.h>
#include <EEPROM.h>

/*

 ENCODER PIN_A = 2
 ENCODER PIN_B or PIN_Z = 3 (depends on the function called by the interrupt)

 OUTPUT PIN = 8
 
 Magnetic encoder: Magntek© MT6826S https://www.magntek.com.cn/en
 https://www.magntek.com.cn/upload/pdf/202407/MT6826S_Datasheet_Rev.1.1.pdf
 
 */

const int CS_PIN = 10;     // Chip Select SPI

// PARAMETRI DELL'ENCODER
const int CPR = 72;              // Encoder counts per revolution (impulsi ABZ totali in un giro)
const long SPI_MAX_VAL = 32768;  // L'MT6826 ha una risoluzione interna a 15 bit (2^15)

const int CPR_X2 = (CPR * 2);   
const int CPR_X4 = (CPR * 4);    // 72 CPR * 4 (quadrature/both edges)

// Con 72 impulsi a giro, abbiamo 288 cambi di stato (fronti) totali
// Ogni mezzo giro (180°) sono 144 fronti.
// Un "settore" della ruota fonica (5°) dura 4 fronti.
volatile int edge_counter = 0; // Counts both rising and falling edges

int value_to_add = 0;   // Il valore da aggiungere alla lettura dell'encoder per posizionare correttamente lo 0

// Per i dati nella EEPROM 
const int start_address = 10;   // Iniziamo dall'indirizzo 10 per sicurezza
const int magic_number = 0x45;  // Un numero magico per riconoscere se i nostri dati sono valido (salvato nella posizione 0)

bool print_debug = false;

// Comando per burst read angle registers (C3~C0 = 1010)
// + indirizzo 0x003 (A11~A0 = 0x003)
// Il comando a 16 bit è: | C3-C0 (4 bit) | A11-A0 (12 bit) |
// Quindi: 0xA (1010) seguito da 0x003 = 0xA003
#define BURST_READ_CMD 0xA003
// Polinomio CRC-8 secondo datasheet: x^8 + x^2 + x + 1
#define CRC8_POLY 0x07

// Comandi SPI (datasheet pagina 25)
// C3~C0 = 0011 per Read, C3~C0 = 0110 per Write
#define SPI_READ_CMD  0x3000  // OR con indirizzo
#define SPI_WRITE_CMD 0x6000  // OR con indirizzo

// Comando per programmare EEPROM (datasheet pagina 27)
// C3~C0 = 1100
#define SPI_PROG_EEPROM_CMD 0xC000

// Indirizzi dei registri (datasheet pagina 35)
#define REG_ABZ_RES_H  0x007  // ABZ_RES[11:4]
#define REG_ABZ_RES_L  0x008  // ABZ_RES[3:0] + altri bit riservati

// Valori validi per ABZ_RES (datasheet pagina 35)
// 0x001 = 1 PPR (pulse per revolution)
// 0x002 = 2 PPR
// ...
// 0xFFF = 4096 PPR (risoluzione massima)
#define MIN_ABZ_RES 1
#define MAX_ABZ_RES 4096

/**
 * Struttura per contenere i dati letti dal sensore
 */
struct AngleData {
  uint16_t angle;          // ANGLE[14:0] - valore angolo grezzo (15 bit)
  uint8_t status;          // STATUS[2:0] - warning flags
  uint8_t received_crc;    // CRC letto dal sensore (register 0x006)
  uint8_t calculated_crc;  // CRC calcolato sui dati ricevuti
  bool crc_valid;          // true se CRC corrisponde
} data;

void setup() {
  Serial.begin(115200);

  // Se la EEPROM contiene il dato lo leggo 
  uint8_t data_zero;
  EEPROM.get(0, data_zero);
  if (data_zero == magic_number) {
    EEPROM.get(start_address, value_to_add);
  }

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(CS_PIN, OUTPUT);
  pinMode(8, OUTPUT);

  // Inizializzazione SPI
  digitalWrite(CS_PIN, HIGH); // Deseleziona il chip all'inizio
  SPI.begin();
  // MT6826 solitamente supporta velocità elevate. Usiamo Mode 3 (o Mode 1 in base al datasheet)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  // Prova a leggere più volte in caso di CRC errato
  int error;
  int num_error = 0;
  do {
    error = readAngleWithCRC(&data);
    if (!error) {
      // Mappiamo i 15 bit dell'SPI sui fronti del giro
      edge_counter = map(data.angle, 0, SPI_MAX_VAL, 0, CPR_X4);
      // aggiungo la variazione dello zero
      edge_counter += value_to_add; 

      Serial.print("Initial position = ");
      Serial.println(edge_counter);
      Serial.print("Value to add = ");
      Serial.println(value_to_add);

    } else {
      Serial.print(++num_error);
      Serial.println(" Encoder reading error :(");
      delay(300);
    }
  } while (error);

  // Legge la risoluzione attuale dell'encoder 
  uint16_t current_res = getABZResolution();
  Serial.print("Current ABZ resolution: ");
  Serial.print(current_res);
  Serial.println(" CPR");

  attachInterrupt(digitalPinToInterrupt(2), manageEncoderAZ, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), manageEncoderAZ, CHANGE);

  Serial.println("? for valid commands");
}

void loop() {
  /* Valid commands:
   * ADD:num -> Valore da aggiungere per regolare lo zero
   * SAVE    -> Salva il valore da aggiuingere nella EEPROM dell'Arduino e nella EEPROM dell'encoder
   * CPR:num -> Valore da inviare all'encoder (ABZ_RES)
   * DEBUG:1 / DEBUG:0 Abilita o disabilita la stampa di debug
  */ 

  // Controlla se ci sono dati sulla seriale
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Legge fino all'invio
    input.trim(); // Rimuove spazi o caratteri nascosti (\r)

    // Controlla se la stringa inizia con "ADD:"
    if (input.startsWith("ADD:")) {
      // Estrae la parte numerica dopo i primi 4 caratteri ("ADD:")
      String valore_str = input.substring(4);
      value_to_add = valore_str.toInt();
      // sposto lo zero
      noInterrupts();                          // Disabilita gli interrupt
      edge_counter += value_to_add;  
      interrupts();                            // Riabilita gli interrupt

    } else if (input.startsWith("SAVE")) {
      // Salva il valore nella EEPROM dell'Arduino
      EEPROM.put(0, magic_number);
      EEPROM.put(start_address, value_to_add);
      Serial.print("Value to add ");
      Serial.print(value_to_add);
      Serial.println(" --> Successfully saved in Arduino EEPROM");

      // Salva i parametri nella EEPROM dell'encoder
      if (!programEEPROM()) {
        Serial.println("ERROR: encoder EEPROM programming failed :(");
      } else {
        Serial.print(getABZResolution());
        Serial.println(" CPR --> Successfully saved in encoder EEPROM");
      }
    } else if (input.startsWith("CPR:")) {
      // Estrae la parte numerica dopo i primi 4 caratteri ("CPR:")
      String valore_str = input.substring(4);
      long cpr = valore_str.toInt();
      if (setABZResolution(cpr)) {
        Serial.println("Setup completed successfully!");
      } else {
        Serial.println("ERROR: Setup failed!");
      }
    } else if (input.startsWith("CPR?")) {
      // Leggi e verifica il valore attuale
      uint16_t current_res = getABZResolution();
      Serial.print("Current ABZ resolution: ");
      Serial.print(current_res);
      Serial.println(" CPR");
    } else if (input.startsWith("DEBUG:")) {
      String valore_str = input.substring(6);
      print_debug = (valore_str.toInt() == 1);
      Serial.print("Debug ");
      if (print_debug) Serial.println("ON");
      else Serial.println("OFF");
    } else {
      Serial.println("Use ADD:number to change the value to be added for move the zero point");
      Serial.println("Use SAVE to save the permanently values into EEPROMs");
      Serial.println("Use CPR:number to change the encoder resolution");
      Serial.println("Use CPR? to see the current encoder resolution");
      Serial.println("DEBUG:1 / DEBUG:0 Enable or disable debug printing");
    }
  }
  if (print_debug) {
    Serial.print(edge_counter);
    Serial.print("\t");
    if (!readAngleWithCRC(&data)) {
      Serial.println(data.angle);
    } else {
      Serial.println("READ ERROR :(");
    }
  }
  delay(500);
}

// A B Non resetta mai e conta tutti i fronti quindi un giro completo corrisponde a CPR_X4 
void manageEncoderAB() {
  // Lettura rapida stato Pin 2 e 3
  byte pins = PIND;
  bool a = (pins >> 2) & 1;
  bool b = (pins >> 3) & 1;

  // Aggiornamento posizione (Quadratura 4x)
  static bool lastA, lastB;
  if (a != lastA || b != lastB) {
    if ((lastA ^ b) & 1) edge_counter++; 
    else edge_counter--;
    
    lastA = a;
    lastB = b;
  }

  // Normalizzazione da 0 a (CPR_X4 -1) (Un giro completo)
  if (edge_counter >= CPR_X4) edge_counter -= CPR_X4;
  //else if (edge_counter < 0) edge_counter += CPR_X4; // Serve solo se "giro all'indietro"

  // Logica Ruota Fonica (Mezzo giro = CPR_X2 fronti)
  // Usando il modulo CPR_X2, il controllo vale per ENTRAMBI i mezzi giri
  // int pos_rel = edge_counter % CPR_X2; // fare il modulo impiega ~150-250 cicli CPU
  // Il risultato è lo stesso di prima ma con pochi cicli CPU
  uint16_t pos_rel;
  if (edge_counter >= CPR_X2) pos_rel = edge_counter - CPR_X2;
  else pos_rel = edge_counter;

  if (pos_rel >= (CPR_X2 - 4)) {
    // ULTIMO SETTORE: Dente mancante (Pin LOW)
    PORTB &= ~(1 << 0); // Pin 8 LOW
  } else {
    // Generazione onda quadra
    // Ogni settore è lungo 4 fronti
    // Vogliamo l'uscita ALTA per i primi 2 fronti e BASSA per i restanti 2
    /* // fare il modulo impiega ~150-250 cicli CPU 
      if ((pos_rel % 4) < 2) {
        PORTB |= (1 << 0);  // Pin 8 HIGH
      } else {
        PORTB &= ~(1 << 0); // Pin 8 LOW
      }
    */
    // Invece del modulo (pos_rel % 4), uso l'operatore bitwise &
    // (pos_rel & 3) restituisce il resto della divisione per 4 (funziona solo con potenze di 2)
    if ((pos_rel & 3) < 2) {
      PORTB |= (1 << 0);
    } else {
      PORTB &= ~(1 << 0);
    }
  }
}

// A Z Legge solo i fronti di A e resetta ad ogni giro tramite il fronte di salita di Z, un giro corrisponde a CPR_X2 
void manageEncoderAZ() {
  /*
   When the magnet rotates counter-clock-wise (CCW)

   A __|‾‾|__|‾‾|__|‾‾|__|‾‾|_

   B _|‾‾|__|‾‾|__|‾‾|__|‾‾|__

   Z _____|‾|_________________
   
   The motor always rotates in the same direction so I can only read PIN A and Z,
   PIN A will indicate the angle and PIN Z will reset the count.
   In this way fewer resources are used and the synchronization is checked at each revolution.
   */

  // Lettura rapida stato Pin 2 e 3
  byte pins = PIND;
  bool a = (pins >> 2) & 1;
  bool z = (pins >> 3) & 1;

  // Aggiornamento posizione (Quadratura 4x)
  static bool lastA, lastZ;
  if (z && !lastZ) {
    edge_counter = value_to_add;
    lastA = a;
  } else if (a != lastA) {
    edge_counter++; 
    lastA = a;
  }
  lastZ = z;

  // Normalizzazione da 0 a (CPR_X2 -1) (Un giro completo)
  if (edge_counter >= CPR_X2) edge_counter -= CPR_X2;

  // Logica Ruota Fonica (Mezzo giro = CPR fronti)
  uint16_t pos_rel;
  if (edge_counter >= CPR) pos_rel = edge_counter - CPR;
  else pos_rel = edge_counter;

  if (pos_rel >= (CPR - 2)) {
    // ULTIMO SETTORE: Dente mancante (Pin LOW)
    PORTB &= ~(1 << 0); // Pin 8 LOW
  } else {
    // Onda quadra basata sul bit 0 (pari/dispari)
    // Se pos_rel è dispari (1, 3, 5...) -> LOW
    // Se pos_rel è pari (0, 2, 4...) -> HIGH
    if (pos_rel & 1) { 
      PORTB &= ~(1 << 0); // Pin 8 LOW
    } else {
      PORTB |= (1 << 0); // Pin 8 HIGH 
    }
  }
}


/*
 * Lettura posizione con verifica CRC
 * 
 * @param output Puntatore alla struttura dove salvare i dati
 * @return false se lettura OK e CRC valido, true altrimenti
 */
bool readAngleWithCRC(AngleData* output) {
  uint8_t buffer[4];  // 4 byte: reg 0x003, 0x004, 0x005, 0x006
  
  // Seleziona il sensore
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  
  // Invia comando burst read (16 bit)
  SPI.transfer16(BURST_READ_CMD);
  
  // Leggi i 4 byte di risposta
  for (int i = 0; i < 4; i++) {
    buffer[i] = SPI.transfer(0x00);
  }
  
  // Deseleziona
  digitalWrite(CS_PIN, HIGH);
  
  // Verifica che la lettura sia avvenuta (valori plausibili)
  // Se tutti i byte sono 0xFF o 0x00, probabilmente c'è un problema di comunicazione
  if (buffer[0] == 0xFF && buffer[1] == 0xFF && buffer[2] == 0xFF && buffer[3] == 0xFF) {
    output->crc_valid = false;
    return true;
  }
  if (buffer[0] == 0x00 && buffer[1] == 0x00 && buffer[2] == 0x00 && buffer[3] == 0x00) {
    output->crc_valid = false;
    return true;
  }
  
  // Estrai ANGLE[14:0] dai primi 2 byte (datasheet pagina 28)
  // buffer[0]: ANGLE[14:7]
  // buffer[1]: ANGLE[6:0] (bit7 = fixed 0)
  output->angle = ((uint16_t)buffer[0] << 7) | (buffer[1] & 0x7F);
  
  // Estrai STATUS[2:0] dal terzo byte (datasheet pagina 29)
  // buffer[2]: [bit7-3 = fixed 00000][bit2-0 = STATUS]
  output->status = buffer[2] & 0x07;

  if (output->status != 0) {
    if (output->status & 0x01) Serial.println("Rotation over speed");
    if (output->status & 0x02) Serial.println("Weak magnetic field");
    if (output->status & 0x04) Serial.println("Under voltage");
  }
  
  // Salva CRC ricevuto (quarto byte)
  output->received_crc = buffer[3];
  
  // Calcola CRC sui primi 3 byte (register 0x003, 0x004, 0x005)
  output->calculated_crc = calculateCRC8(buffer, 3);
  
  // Verifica se il CRC corrisponde
  output->crc_valid = (output->calculated_crc == output->received_crc);
  
  return(!output->crc_valid);
}

/*
 * Calcola CRC-8 secondo specifiche datasheet 
 * Polinomio: x^8 + x^2 + x + 1 (0x07)
 * 
 * @param data Array di 3 byte (register 0x003, 0x004, 0x005)
 * @param len Lunghezza dati (deve essere 3)
 * @return uint8_t CRC calcolato
 */
uint8_t calculateCRC8(uint8_t* data, uint8_t len) {
  uint8_t crc = 0x00;
  
  for (uint8_t byte_idx = 0; byte_idx < len; byte_idx++) {
    crc ^= data[byte_idx];
    
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ CRC8_POLY;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}


/**
 * Legge un byte da un registro SPI
 * 
 * @param address Indirizzo del registro (12 bit)
 * @return uint8_t Valore letto
 */
uint8_t spiReadByte(uint16_t address) {
  uint16_t command = SPI_READ_CMD | (address & 0x0FFF);
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  
  SPI.transfer16(command);
  uint8_t data = SPI.transfer(0x00);
  
  digitalWrite(CS_PIN, HIGH);
  
  return data;
}

/**
 * Scrive un byte in un registro SPI
 * 
 * @param address Indirizzo del registro (12 bit)
 * @param data Dato da scrivere (8 bit)
 */
void spiWriteByte(uint16_t address, uint8_t data) {
  uint16_t command = SPI_WRITE_CMD | (address & 0x0FFF);
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  
  SPI.transfer16(command);
  SPI.transfer(data);
  
  digitalWrite(CS_PIN, HIGH);
  
  delayMicroseconds(10);
}

/**
 * Imposta la risoluzione ABZ (Pulse Per Revolution)
 * 
 * @param resolution Risoluzione desiderata (1 ~ 4096 PPR)
 * @return true se impostata con successo, false altrimenti
 */
bool setABZResolution(uint16_t resolution) {
  // Validazione input
  if (resolution < MIN_ABZ_RES || resolution > MAX_ABZ_RES) {
    Serial.print("ERROR: Invalid resolution, must be from ");
    Serial.print(MIN_ABZ_RES);
    Serial.print(" and ");
    Serial.println(MAX_ABZ_RES);
    return false;
  }
  
  // Calcola i due byte da scrivere
  // ABZ_RES[11:0] è un valore a 12 bit
  uint8_t high_byte = (resolution >> 4) & 0xFF;      // Bit 11-4
  uint8_t low_byte = (resolution & 0x0F) << 4;       // Bit 3-0 vanno nei bit 7-4 del registro 0x008
  
  // Leggi il valore corrente del registro 0x008 per preservare i bit riservati (bits 3-0)
  uint8_t current_low_byte = spiReadByte(REG_ABZ_RES_L);
  
  // Maschera: preserva i bit riservati (bits 3-0), sostituisci solo ABZ_RES[3:0] (bits 7-4)
  uint8_t new_low_byte = (current_low_byte & 0x0F) | low_byte;
  
  // Scrivi i registri
  spiWriteByte(REG_ABZ_RES_H, high_byte);
  spiWriteByte(REG_ABZ_RES_L, new_low_byte);
  
  // Verifica la scrittura
  uint8_t verify_high = spiReadByte(REG_ABZ_RES_H);
  uint8_t verify_low = spiReadByte(REG_ABZ_RES_L);
  
  if (verify_high != high_byte || verify_low != new_low_byte) {
    Serial.println("ERROR: Writing verification failed!");
    return false;
  }
  
  Serial.print("ABZ resolution now is ");
  Serial.print(resolution);
  Serial.println(" PPR");
  
  return true;
}

/**
 * Legge la risoluzione ABZ corrente
 * 
 * @return uint16_t Risoluzione attuale in PPR
 */
uint16_t getABZResolution() {
  uint8_t high_byte = spiReadByte(REG_ABZ_RES_H);
  uint8_t low_byte = spiReadByte(REG_ABZ_RES_L);
  
  // Estrai ABZ_RES dai due registri
  uint16_t resolution = ((uint16_t)high_byte << 4) | ((low_byte >> 4) & 0x0F);
  
  return resolution;
}

/**
 * Programma l'EEPROM con i valori correnti dei registri (datasheet pag. 10)
 * 
 * @return true se programmazione riuscita, false altrimenti
 */
bool programEEPROM() {
  Serial.println("Programming the encoder EEPROM...");
  Serial.println("Do NOT TURN OFF the device!");
  
  uint16_t command = SPI_PROG_EEPROM_CMD;  // Comando + dati non necessari
  
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  
  // Invia comando di programmazione (16 bit)
  SPI.transfer16(command);
  
  // Leggi acknowledge bytes (datasheet: deve restituire 0x55)
  uint8_t ack1 = SPI.transfer(0x00);
  uint8_t ack2 = SPI.transfer(0x00);
  uint8_t ack3 = SPI.transfer(0x00);
  uint8_t ack4 = SPI.transfer(0x00);
  
  digitalWrite(CS_PIN, HIGH);
  
  // Verifica acknowledge (datasheet dice che deve tornare 0x55)
  // Nota: Il datasheet non specifica quanti byte di ack, controlliamo i primi
  if (ack1 == 0x55) {  // Semplificato: controlla solo il primo byte
    Serial.println("EEPROM programmed successfully!");
    
    // Attendi il completamento (datasheet non specifica un tempo,
    // ma consiglia di controllare il registro EE_DONE all'indirizzo 0x112[5])
    delay(500);  // Pausa prudenziale
    
    return true;
  } else {
    Serial.print("ERROR: EEPROM programming failed :(\nAck: 0x");
    Serial.println(ack1, HEX);
    return false;
  }
}
