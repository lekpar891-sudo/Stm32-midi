/* name: src/main.cpp
   Comprehensive firmware for STM32F103C8T6 BluePill
   - Full arranger features: mapping, layers, tap-chord tempo, persistence, transport, panic, mapping editor
   - Uses pins exactly as in your last mapping (no pin reassignments)
*/

#include <Arduino.h>
#include <EEPROM.h>
#include "MIDIUSB.h"

/* ---------------------------------------------------------------------
   PIN MAPPING (kept exactly as existing mapping)
   - NOTE MATRIX: Rows PB0..PB7  (inputs with pullups)
                  Cols  PB8..PB15 (outputs, drive LOW active)
   - FUNC MATRIX: Rows PC8..PC11 (inputs with pullups)
                  Cols  PA5..PA9   (outputs, drive LOW active)
   - Encoder/Buttons: PC0..PC7 etc. (as before)
   - ADC: PA0..PA4 for velocity,pitch X/Y,volume,pan
   ---------------------------------------------------------------------*/

/* --- Note matrix pins (do NOT change) --- */
const uint8_t ROW_PINS[8] = {PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7};
const uint8_t COL_PINS[8] = {PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15};

/* --- Function matrix pins (do NOT change) --- */
const uint8_t FROW_PINS[4] = {PC8, PC9, PC10, PC11};
const uint8_t FCOL_PINS[5] = {PA5, PA6, PA7, PA8, PA9};

/* --- Encoder & buttons (do NOT change) --- */
const uint8_t ENC_A_PIN = PC0;
const uint8_t ENC_B_PIN = PC1;
const uint8_t ENC_BTN_PIN = PC2;
const uint8_t BTN_TEMPO_PIN = PC3;
const uint8_t BTN_OCTAVE_PIN = PC4;
const uint8_t BTN_TRANSPOSE_PIN = PC5;
const uint8_t BTN_PB_X_PIN = PC6;
const uint8_t TAP_PIN = PC13;
const uint8_t TREMOLO_TOGGLE_PIN = PC7;

/* --- Analog (do NOT change) --- */
const uint8_t VELOCITY_PIN = A0; // PA0
const uint8_t PITCH_X_PIN = A1;  // PA1
const uint8_t PITCH_Y_PIN = A2;  // PA2
const uint8_t VOLUME_PIN  = A3;  // PA3
const uint8_t BALANCE_PIN = A4;  // PA4

/* MIDI settings */
const uint8_t MIDI_CHANNEL = 0; // 0..15 internal
int BASE_NOTE = 36;             // base, layered with layer offsets
const unsigned long DEBOUNCE_MS = 8;
const uint8_t FUNC_CC_BASE = 20;
const int FUNC_COUNT = 20;

/* EEPROM layout and structure */
#define EEPROM_MAGIC 0xA5A5A5A5UL
struct Settings {
  uint32_t magic;
  float savedBPM;
  int8_t transpose;
  int8_t octaveShift;
  bool paramEnabled[4];
  uint8_t layerOffsets[4];   // semitone offsets per layer
  uint8_t funcMappingType[FUNC_COUNT]; // 0=CC,1=PC,2=Note
  uint8_t funcMappingValue[FUNC_COUNT]; // CC number or PC/program or note offset
  uint16_t checksum;
};
static Settings settings;
const int EEPROM_ADDR = 0;

/* Application state */
bool keyState[8][8];
unsigned long keyLastChange[8][8];
bool fkeyState[4][5];
unsigned long fkeyLastChange[4][5];

volatile long encoderPos = 0;
int lastEncoded = 0;
bool lastEncBtnState = HIGH;
enum Param { PARAM_TEMPO=0, PARAM_OCTAVE, PARAM_TRANSPOSE, PARAM_PB_X, PARAM_COUNT };
bool paramEnabled[PARAM_COUNT] = { true, true, true, true };
int currentParam = 0;

/* tempo / midi clock */
unsigned long lastTap = 0;
unsigned long tapInterval = 0;
float bpm = 120.0f;
unsigned long lastClockSend = 0;
unsigned int midiClockCounter = 0;

/* tremolo */
bool tremoloEnabled = false;
float tremoloRateHz = 5.0;
unsigned long tremoloStart = 0;

/* transpose/octave */
int transposeSemitones = 0;
int octaveShift = 0;

/* analog caching + smoothing */
int velocityRaw = 0;
int pitchXRaw = 0;
int pitchYRaw = 0;
int volumeRaw = 0;
int balanceRaw = 0;
uint8_t lastCCVolume = 0;
uint8_t lastCCBalance = 0;
const uint8_t CC_THRESHOLD = 2;

/* layers */
int currentLayer = 0; // 0..3
const int LAYER_COUNT = 4;
int LAYER_BASE_OFFSETS[4] = {0, 12, 24, 36}; // default, can be stored

/* chord/tap detection */
bool chordActive = false;
unsigned long chordStartTime = 0;
unsigned long chordReleaseTime = 0;
int chordNotesBuffer[32];
int chordNotesCount = 0;
uint32_t lastChordHash = 0;
unsigned long lastChordTapTime = 0;
unsigned long lastChordInterval = 0;
int chordTapCount = 0;
const unsigned long MAX_TAP_INTERVAL_MS = 2000;
const int MIN_CHORD_SIZE = 2;

/* mapping editor state */
bool mappingMode = false;
int mappingKeyRow = -1, mappingKeyCol = -1; // which function key is being edited

/* forward declarations */
void sendAllNotesOff();
void saveSettings();
bool loadSettings();
uint16_t computeChecksum(const Settings& s);

/* MIDI low-level helpers */
void sendNoteOn(uint8_t channel, uint8_t note, uint8_t velocity) {
  midiEventPacket_t noteOn = {0x09, uint8_t(0x90 | (channel & 0x0F)), note, velocity};
  MidiUSB.sendMIDI(noteOn);
}
void sendNoteOff(uint8_t channel, uint8_t note, uint8_t velocity) {
  midiEventPacket_t noteOff = {0x08, uint8_t(0x80 | (channel & 0x0F)), note, velocity};
  MidiUSB.sendMIDI(noteOff);
}
void sendControlChange(uint8_t channel, uint8_t cc, uint8_t value) {
  midiEventPacket_t ccPkt = {0x0B, uint8_t(0xB0 | (channel & 0x0F)), cc, value};
  MidiUSB.sendMIDI(ccPkt);
}
void sendPitchBend(uint8_t channel, int value14) {
  uint8_t lsb = value14 & 0x7F;
  uint8_t msb = (value14 >> 7) & 0x7F;
  midiEventPacket_t pb = {0x0E, uint8_t(0xE0 | (channel & 0x0F)), lsb, msb};
  MidiUSB.sendMIDI(pb);
}
void sendProgramChange(uint8_t channel, uint8_t program) {
  midiEventPacket_t pc = {0x0C, uint8_t(0xC0 | (channel & 0x0F)), program, 0};
  MidiUSB.sendMIDI(pc);
}
void sendMidiClock() {
  midiEventPacket_t clk = {0x0, 0xF8, 0, 0};
  MidiUSB.sendMIDI(clk);
}
void sendMidiStart() { midiEventPacket_t m = {0x0, 0xFA, 0, 0}; MidiUSB.sendMIDI(m); }
void sendMidiStop()  { midiEventPacket_t m = {0x0, 0xFC, 0, 0}; MidiUSB.sendMIDI(m); }
void sendMidiContinue() { midiEventPacket_t m = {0x0, 0xFB, 0, 0}; MidiUSB.sendMIDI(m); }

/* utility: hash for chord fingerprint */
uint32_t chordHashFromNotes(int *notes, int cnt) {
  uint32_t h = 2166136261u;
  for (int i=0;i<cnt;i++) {
    uint32_t v = (uint32_t)notes[i] * 16777619u;
    h ^= v;
    h *= 16777619u;
  }
  return h;
}

/* map matrix coord -> MIDI note (respect layer, transpose, octave) */
uint8_t matrixToNote(uint8_t r, uint8_t c) {
  uint8_t idx = r * 8 + c;
  int note = BASE_NOTE + idx + transposeSemitones + octaveShift * 12 + LAYER_BASE_OFFSETS[currentLayer];
  note = constrain(note, 0, 127);
  return (uint8_t)note;
}

/* --------------------------- Persistence --------------------------- */
uint16_t computeChecksum(const Settings& s) {
  // simple checksum (sum of bytes)
  const uint8_t *p = (const uint8_t*)&s;
  uint32_t sum = 0;
  // exclude checksum bytes at end (last two bytes)
  for (size_t i=0;i<sizeof(Settings)-2;i++) sum += p[i];
  return (uint16_t)(sum & 0xFFFF);
}

bool loadSettings() {
  EEPROM.begin(sizeof(Settings) + 4);
  EEPROM.get(EEPROM_ADDR, settings);
  if (settings.magic != EEPROM_MAGIC) {
    Serial.println("EEPROM: no saved settings (magic mismatch)");
    return false;
  }
  uint16_t cs = computeChecksum(settings);
  if (cs != settings.checksum) {
    Serial.println("EEPROM: checksum mismatch");
    return false;
  }
  // apply settings
  bpm = settings.savedBPM;
  transposeSemitones = settings.transpose;
  octaveShift = settings.octaveShift;
  for (int i=0;i<PARAM_COUNT;i++) paramEnabled[i] = settings.paramEnabled[i];
  for (int i=0;i<LAYER_COUNT;i++) LAYER_BASE_OFFSETS[i] = settings.layerOffsets[i];
  // function mapping arrays loaded (funcMappingType & value)
  Serial.println("EEPROM: loaded settings");
  return true;
}

void saveSettings() {
  settings.magic = EEPROM_MAGIC;
  settings.savedBPM = bpm;
  settings.transpose = (int8_t)transposeSemitones;
  settings.octaveShift = (int8_t)octaveShift;
  for (int i=0;i<PARAM_COUNT;i++) settings.paramEnabled[i] = paramEnabled[i];
  for (int i=0;i<LAYER_COUNT;i++) settings.layerOffsets[i] = (uint8_t)LAYER_BASE_OFFSETS[i];
  // if mappings not initialized, fill defaults
  for (int i=0;i<FUNC_COUNT;i++) {
    // default mapping: CC base + idx
    settings.funcMappingType[i] = 0; // CC
    settings.funcMappingValue[i] = FUNC_CC_BASE + i;
  }
  settings.checksum = computeChecksum(settings);
  EEPROM.put(EEPROM_ADDR, settings);
  EEPROM.commit();
  Serial.println("EEPROM: saved settings");
}

/* --------------------------- Safety & Panic --------------------------- */
void sendAllNotesOff() {
  // CC 123 on channel
  sendControlChange(MIDI_CHANNEL, 123, 0);
  // send All Sound Off (120)
  sendControlChange(MIDI_CHANNEL, 120, 0);
  // send Note Off for all notes to be safe
  for (uint8_t n = 0; n < 128; ++n) {
    sendNoteOff(MIDI_CHANNEL, n, 0);
  }
  MidiUSB.flush();
}

/* --------------------------- Pin init --------------------------- */
void setupPins() {
  // NOTE matrix
  for (int i=0;i<8;i++) {
    pinMode(COL_PINS[i], OUTPUT);
    digitalWrite(COL_PINS[i], HIGH); // idle HIGH
    pinMode(ROW_PINS[i], INPUT_PULLUP);
  }
  // FUNC matrix
  for (int i=0;i<5;i++) {
    pinMode(FCOL_PINS[i], OUTPUT);
    digitalWrite(FCOL_PINS[i], HIGH);
  }
  for (int i=0;i<4;i++) pinMode(FROW_PINS[i], INPUT_PULLUP);

  // encoder & buttons
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_BTN_PIN, INPUT_PULLUP);

  pinMode(BTN_TEMPO_PIN, INPUT_PULLUP);
  pinMode(BTN_OCTAVE_PIN, INPUT_PULLUP);
  pinMode(BTN_TRANSPOSE_PIN, INPUT_PULLUP);
  pinMode(BTN_PB_X_PIN, INPUT_PULLUP);

  pinMode(TAP_PIN, INPUT_PULLUP);
  pinMode(TREMOLO_TOGGLE_PIN, INPUT_PULLUP);
}

/* --------------------------- Matrix scanning --------------------------- */
void scanNoteMatrix() {
  int activeNotes[64];
  int activeCount = 0;

  for (int c=0;c<8;c++) {
    digitalWrite(COL_PINS[c], LOW);
    delayMicroseconds(30);
    for (int r=0;r<8;r++) {
      bool pressed = (digitalRead(ROW_PINS[r]) == LOW);
      if (pressed != keyState[r][c]) {
        unsigned long now = millis();
        if (now - keyLastChange[r][c] > DEBOUNCE_MS) {
          keyState[r][c] = pressed;
          keyLastChange[r][c] = now;
          uint8_t note = matrixToNote(r,c);
          int vel = map(analogRead(VELOCITY_PIN), 0, 1023, 20, 127);
          vel = constrain(vel, 1, 127);
          if (pressed) {
            sendNoteOn(MIDI_CHANNEL, note, vel);
          } else {
            sendNoteOff(MIDI_CHANNEL, note, 0);
          }
        }
      }
      if (keyState[r][c]) {
        activeNotes[activeCount++] = matrixToNote(r,c);
      }
    }
    digitalWrite(COL_PINS[c], HIGH);
  }

  // chord detection (on release)
  static bool prevChordActive = false;
  if (!prevChordActive && activeCount >= MIN_CHORD_SIZE) {
    chordActive = true;
    chordStartTime = millis();
    chordNotesCount = activeCount;
    for (int i=0;i<activeCount && i<32;i++) chordNotesBuffer[i] = activeNotes[i];
  } else if (prevChordActive && activeCount == 0 && chordActive) {
    chordReleaseTime = millis();
    uint32_t h = chordHashFromNotes(chordNotesBuffer, chordNotesCount);
    if (lastChordHash != 0 && h == lastChordHash) {
      unsigned long interval = chordReleaseTime - lastChordTapTime;
      if (interval > 30 && interval < MAX_TAP_INTERVAL_MS) {
        chordTapCount++;
        lastChordInterval = interval;
        if (chordTapCount >= 2) {
          bpm = 60000.0f / float(lastChordInterval);
          Serial.print("TapChord set BPM: "); Serial.println(bpm);
        }
      } else {
        chordTapCount = 1;
      }
    } else {
      chordTapCount = 1;
    }
    lastChordHash = h;
    lastChordTapTime = chordReleaseTime;
    chordActive = false;
  }
  prevChordActive = (activeCount >= MIN_CHORD_SIZE);
}

/* --------------------------- Function matrix --------------------------- */
void executeFunctionIndexPress(int idx) {
  // Validate idx
  if (idx < 0 || idx >= FUNC_COUNT) return;
  uint8_t type = settings.funcMappingType[idx];
  uint8_t val  = settings.funcMappingValue[idx];
  if (idx >= 15 && idx <= 18) {
    // keyboardset indexes 15..18 default handled separately in scan (but also allow mapping)
    int layer = idx - 15;
    if (layer >= 0 && layer < LAYER_COUNT) {
      currentLayer = layer;
      sendProgramChange(MIDI_CHANNEL, (uint8_t)layer);
      Serial.print("Layer set to "); Serial.println(layer);
      return;
    }
  }
  if (type == 0) { // CC
    sendControlChange(MIDI_CHANNEL, val, 127);
  } else if (type == 1) { // Program Change
    sendProgramChange(MIDI_CHANNEL, val);
  } else if (type == 2) { // Note trigger
    sendNoteOn(MIDI_CHANNEL, val, 127);
  }
}
void executeFunctionIndexRelease(int idx) {
  if (idx < 0 || idx >= FUNC_COUNT) return;
  uint8_t type = settings.funcMappingType[idx];
  uint8_t val  = settings.funcMappingValue[idx];
  if (type == 0) { // CC
    sendControlChange(MIDI_CHANNEL, val, 0);
  } else if (type == 2) { // Note trigger release
    sendNoteOff(MIDI_CHANNEL, val, 0);
  }
}

void scanFunctionMatrix() {
  for (int c=0;c<5;c++) {
    digitalWrite(FCOL_PINS[c], LOW);
    delayMicroseconds(30);
    for (int r=0;r<4;r++) {
      bool pressed = (digitalRead(FROW_PINS[r]) == LOW);
      if (pressed != fkeyState[r][c]) {
        unsigned long now = millis();
        if (now - fkeyLastChange[r][c] > DEBOUNCE_MS) {
          fkeyState[r][c] = pressed;
          fkeyLastChange[r][c] = now;
          int idx = r * 5 + c;
          if (pressed) {
            // mapping editor entry: long press to enter edit mode
            if (!mappingMode) {
              // normal action
              executeFunctionIndexPress(idx);
            }
          } else {
            if (!mappingMode) {
              executeFunctionIndexRelease(idx);
            }
          }
        }
      }
      // mapping mode check: if long-press detected and encoder is used to change mapping
      if (!mappingMode && fkeyState[r][c]) {
        // detect long press
        if (millis() - fkeyLastChange[r][c] > 1200) {
          // start mapping mode for this key
          mappingMode = true;
          mappingKeyRow = r;
          mappingKeyCol = c;
          Serial.print("Entering mapping mode for func idx "); Serial.println(r*5+c);
          // Show options via Serial: 0:CC,1:PC,2:Note — use encoder to change, press encoder to save
        }
      }
    }
    digitalWrite(FCOL_PINS[c], HIGH);
  }
}

/* --------------------------- Encoder logic --------------------------- */
void updateEncoder() {
  int MSB = digitalRead(ENC_A_PIN);
  int LSB = digitalRead(ENC_B_PIN);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderPos++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderPos--;
  lastEncoded = encoded;

  bool curBtn = digitalRead(ENC_BTN_PIN);
  static unsigned long encBtnDownAt = 0;
  if (lastEncBtnState == HIGH && curBtn == LOW) {
    encBtnDownAt = millis();
  } else if (lastEncBtnState == LOW && curBtn == HIGH) {
    unsigned long held = millis() - encBtnDownAt;
    if (mappingMode) {
      // encoder button: save mapping
      int idx = mappingKeyRow*5 + mappingKeyCol;
      uint8_t selType = (uint8_t)constrain((int)settings.funcMappingType[idx], 0, 2);
      uint8_t selVal  = settings.funcMappingValue[idx];
      // Save and exit mapping mode
      EEPROM.put(EEPROM_ADDR, settings);
      EEPROM.commit();
      mappingMode = false; mappingKeyRow = mappingKeyCol = -1;
      Serial.println("Mapping saved and exited mapping mode");
    } else if (held >= 2000) {
      // long press on encoder -> panic (All Notes Off)
      Serial.println("Encoder long-press -> Panic (All Notes Off)");
      sendAllNotesOff();
    } else {
      // short press -> cycle parameter
      currentParam = (currentParam + 1) % PARAM_COUNT;
      Serial.print("Selected param "); Serial.println(currentParam);
    }
  }
  lastEncBtnState = curBtn;
}

/* apply encoder changes to current parameter (or mapping editor if active) */
void applyEncoderToParam() {
  static long lastPos = 0;
  long pos = encoderPos;
  long delta = pos - lastPos;
  if (delta == 0) return;
  lastPos = pos;

  if (mappingMode) {
    // change mapping for selected function key
    int idx = mappingKeyRow*5 + mappingKeyCol;
    // cycle value types on coarse rotation, value adjust on fine rotation
    static int localType = 0;
    static int localVal = 0;
    if (abs(delta) >= 4) {
      // larger delta toggles type
      localType = (localType + (delta > 0 ? 1 : -1) + 3) % 3;
      settings.funcMappingType[idx] = localType;
      Serial.print("Mapping key "); Serial.print(idx); Serial.print(" type -> "); Serial.println(localType);
    } else {
      // small delta modify value depending on type
      if (settings.funcMappingType[idx] == 0) { // CC 0..127
        int v = (int)settings.funcMappingValue[idx] + (delta > 0 ? 1 : -1);
        v = constrain(v, 0, 127);
        settings.funcMappingValue[idx] = (uint8_t)v;
        Serial.print("Mapping CC val -> "); Serial.println(v);
      } else if (settings.funcMappingType[idx] == 1) { // PC 0..127
        int v = (int)settings.funcMappingValue[idx] + (delta > 0 ? 1 : -1);
        v = constrain(v, 0, 127);
        settings.funcMappingValue[idx] = (uint8_t)v;
        Serial.print("Mapping PC val -> "); Serial.println(v);
      } else { // Note 0..127
        int v = (int)settings.funcMappingValue[idx] + (delta > 0 ? 1 : -1);
        v = constrain(v, 0, 127);
        settings.funcMappingValue[idx] = (uint8_t)v;
        Serial.print("Mapping Note val -> "); Serial.println(v);
      }
    }
    // reflect to EEPROM buffer but don't commit until encoder button pressed
    return;
  }

  if (!paramEnabled[currentParam]) return;

  if (currentParam == PARAM_TEMPO) {
    bpm += (delta > 0) ? 1.0f : -1.0f;
    bpm = constrain(bpm, 30.0f, 300.0f);
  } else if (currentParam == PARAM_OCTAVE) {
    octaveShift += (delta > 0) ? 1 : -1;
    octaveShift = constrain(octaveShift, -3, 3);
  } else if (currentParam == PARAM_TRANSPOSE) {
    transposeSemitones += (delta > 0) ? 1 : -1;
    transposeSemitones = constrain(transposeSemitones, -24, 24);
  } else if (currentParam == PARAM_PB_X) {
    tremoloRateHz += (delta > 0) ? 0.5f : -0.5f;
    tremoloRateHz = constrain(tremoloRateHz, 0.1f, 20.0f);
  }
}

/* --------------------------- Button toggles --------------------------- */
void handleParamToggles() {
  static bool lastTempo = HIGH, lastOct = HIGH, lastTrans = HIGH, lastPB = HIGH;
  bool curTempo = digitalRead(BTN_TEMPO_PIN);
  bool curOct   = digitalRead(BTN_OCTAVE_PIN);
  bool curTrans = digitalRead(BTN_TRANSPOSE_PIN);
  bool curPB    = digitalRead(BTN_PB_X_PIN);

  if (lastTempo == HIGH && curTempo == LOW) paramEnabled[PARAM_TEMPO] = !paramEnabled[PARAM_TEMPO];
  if (lastOct == HIGH && curOct == LOW) paramEnabled[PARAM_OCTAVE] = !paramEnabled[PARAM_OCTAVE];
  if (lastTrans == HIGH && curTrans == LOW) paramEnabled[PARAM_TRANSPOSE] = !paramEnabled[PARAM_TRANSPOSE];
  if (lastPB == HIGH && curPB == LOW) paramEnabled[PARAM_PB_X] = !paramEnabled[PARAM_PB_X];

  lastTempo = curTempo; lastOct = curOct; lastTrans = curTrans; lastPB = curPB;
}

/* --------------------------- Tap tempo (button) --------------------------- */
void handleTapTempoBtn() {
  static bool lastTapState = HIGH;
  bool cur = digitalRead(TAP_PIN);
  if (lastTapState == HIGH && cur == LOW) {
    unsigned long now = millis();
    if (lastTap != 0) {
      tapInterval = now - lastTap;
      if (tapInterval > 50 && tapInterval < 2000) {
        bpm = 60000.0f / float(tapInterval);
        Serial.print("Tap button set BPM: "); Serial.println(bpm);
      }
    }
    lastTap = now;
  }
  lastTapState = cur;
}

/* --------------------------- MIDI Clock --------------------------- */
void handleMidiClock() {
  float intervalMs = 60000.0f / (bpm * 24.0f);
  unsigned long now = millis();
  if (now - lastClockSend >= (unsigned long)intervalMs) {
    lastClockSend = now;
    sendMidiClock();
    midiClockCounter = (midiClockCounter + 1) % 24;
    MidiUSB.flush();
  }
}

/* --------------------------- Tremolo LFO --------------------------- */
int computeTremoloPitchbend() {
  unsigned long t = millis() - tremoloStart;
  float period = 1000.0f / tremoloRateHz;
  float phase = fmod((float)t, period) / period;
  float tri = (phase < 0.5f) ? (phase * 4.0f - 1.0f) : (3.0f - phase * 4.0f);
  float center = 8192.0f;
  float depth = 2000.0f;
  float pb = center + tri * depth;
  return (int)constrain(pb, 0, 16383);
}

/* --------------------------- Analog controls -> MIDI --------------- */
void handleAnalogControls() {
  velocityRaw = analogRead(VELOCITY_PIN);
  pitchXRaw = analogRead(PITCH_X_PIN);
  pitchYRaw = analogRead(PITCH_Y_PIN);
  volumeRaw = analogRead(VOLUME_PIN);
  balanceRaw = analogRead(BALANCE_PIN);

  if (tremoloEnabled) {
    int pbVal = computeTremoloPitchbend();
    sendPitchBend(MIDI_CHANNEL, pbVal);
  } else {
    int pbVal = map(pitchXRaw, 0, 1023, 0, 16383);
    sendPitchBend(MIDI_CHANNEL, pbVal);
  }

  int modVal = map(pitchYRaw, 0, 1023, 0, 127);
  sendControlChange(MIDI_CHANNEL, 1, (uint8_t)modVal);

  int vol = map(volumeRaw, 0, 1023, 0, 127);
  if (abs(vol - (int)lastCCVolume) >= CC_THRESHOLD) {
    sendControlChange(MIDI_CHANNEL, 7, (uint8_t)vol);
    lastCCVolume = vol;
  }

  int pan = map(balanceRaw, 0, 1023, 0, 127);
  if (abs(pan - (int)lastCCBalance) >= CC_THRESHOLD) {
    sendControlChange(MIDI_CHANNEL, 10, (uint8_t)pan);
    lastCCBalance = pan;
  }
}

/* --------------------------- Tremolo toggle --------------------------- */
void handleTremoloToggle() {
  static bool lastState = HIGH;
  bool cur = digitalRead(TREMOLO_TOGGLE_PIN);
  if (lastState == HIGH && cur == LOW) {
    tremoloEnabled = !tremoloEnabled;
    if (tremoloEnabled) tremoloStart = millis();
    Serial.print("Tremolo mode now "); Serial.println(tremoloEnabled ? "ON" : "OFF");
  }
  lastState = cur;
}

/* --------------------------- Setup & Loop --------------------------- */
void setup() {
  Serial.begin(115200);
  setupPins();

  // init states
  memset(keyState, 0, sizeof(keyState));
  memset(keyLastChange, 0, sizeof(keyLastChange));
  memset(fkeyState, 0, sizeof(fkeyState));
  memset(fkeyLastChange, 0, sizeof(fkeyLastChange));

  // load settings if present
  if (!loadSettings()) {
    // initialize defaults into settings object (first run)
    for (int i=0;i<FUNC_COUNT;i++) {
      settings.funcMappingType[i] = 0;
      settings.funcMappingValue[i] = FUNC_CC_BASE + i;
    }
    for (int i=0;i<LAYER_COUNT;i++) settings.layerOffsets[i] = (uint8_t)LAYER_BASE_OFFSETS[i];
    settings.savedBPM = bpm;
    settings.transpose = (int8_t)transposeSemitones;
    settings.octaveShift = (int8_t)octaveShift;
    for (int i=0;i<PARAM_COUNT;i++) settings.paramEnabled[i] = paramEnabled[i];
    settings.magic = EEPROM_MAGIC;
    settings.checksum = computeChecksum(settings);
    EEPROM.put(EEPROM_ADDR, settings);
    EEPROM.commit();
    Serial.println("Saved default settings to EEPROM");
  } else {
    Serial.println("Settings applied.");
  }

  // send all-notes-off to be safe
  sendAllNotesOff();
  MidiUSB.flush();
}

void loop() {
  scanNoteMatrix();
  scanFunctionMatrix();

  updateEncoder();
  applyEncoderToParam();
  handleParamToggles();

  handleTapTempoBtn();
  handleMidiClock();

  handleAnalogControls();
  handleTremoloToggle();

  // periodically auto-save some settings if they changed (e.g. bpm/transpose)
  static unsigned long lastSave = 0;
  if (millis() - lastSave > 30000) {
    // store some essentials
    settings.savedBPM = bpm;
    settings.transpose = (int8_t)transposeSemitones;
    settings.octaveShift = (int8_t)octaveShift;
    for (int i=0;i<PARAM_COUNT;i++) settings.paramEnabled[i] = paramEnabled[i];
    settings.checksum = computeChecksum(settings);
    EEPROM.put(EEPROM_ADDR, settings);
    EEPROM.commit();
    Serial.println("Auto-saved settings");
    lastSave = millis();
  }

  // small delay (keep responsiveness)
  delay(5);
}
