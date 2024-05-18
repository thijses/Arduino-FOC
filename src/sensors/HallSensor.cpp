#include "HallSensor.h"


/*
  HallSensor(int hallA, int hallB , int cpr, int index)
  - hallA, hallB, hallC    - HallSensor A, B and C pins
  - pp           - pole pairs
*/
HallSensor::HallSensor(int _hallA, int _hallB, int _hallC, int _pp) {

  // hardware pins
  pinA = _hallA;
  pinB = _hallB;
  pinC = _hallC;

  // hall has 6 segments per electrical revolution
  cpr = _pp * 6;

  // extern pullup as default
  pullup = Pullup::USE_EXTERN;
}

//  HallSensor interrupt callback functions
// A channel
void HallSensor::handleA() {
  A_active= digitalRead(pinA);
  updateState();
}
// B channel
void HallSensor::handleB() {
  B_active = digitalRead(pinB);
  updateState();
}

// C channel
void HallSensor::handleC() {
  C_active = digitalRead(pinC);
  updateState();
}

/**
 * Updates the state and sector following an interrupt
 */
void HallSensor::updateState() {
  long new_pulse_timestamp = _micros();

  if(_glitchFixTimestamp) { A_active=digitalRead(pinA);B_active=digitalRead(pinB);C_active=digitalRead(pinC); }
  int8_t new_hall_state = C_active + (B_active << 1) + (A_active << 2);
  //// TLD debug:
  static bool pin40State;
  if((new_hall_state == (0b000)) || (new_hall_state == (0b111))) { // TLD addition: if state is impossible
    // digitalWrite(40, pin40State=!pin40State); // toggle debug pin // DELETEME
    int8_t bad_state = new_hall_state; // record for debug
    A_active=digitalRead(pinA); B_active=digitalRead(pinB); C_active=digitalRead(pinC); // re-read all pins
    new_hall_state = C_active + (B_active << 1) + (A_active << 2); // attempt to recover
    if((new_hall_state == (0b000)) || (new_hall_state == (0b111))) { // if glitch persists
      log_d("%u bad state %u%u%u -> %u%u%u", pinA, (hall_state&0b100)>>2,(hall_state&0b010)>>1,hall_state&0b001, (new_hall_state&0b100)>>2,(new_hall_state&0b010)>>1,new_hall_state&0b001); // TLD debug
      if(!_glitchFixTimestamp) {_glitchFixTimestamp=millis();} // try to fix glitch at next interrupt or update()
      return; // glitch was not fixed, quit
    } else { // if glitch was fixed
      log_v("\t%u bad-state glitch fixed %u%u%u -> %u%u%u", pinA, (bad_state&0b100)>>2,(bad_state&0b010)>>1,bad_state&0b001, (new_hall_state&0b100)>>2,(new_hall_state&0b010)>>1,new_hall_state&0b001); // TLD debug, deleteme
    }
  } else if(new_hall_state == hall_state) { // glitch avoidance #1 - sometimes we get an interrupt but pins haven't changed
    // digitalWrite(40, pin40State=!pin40State); // toggle debug pin // DELETEME
    int8_t bad_state = new_hall_state; // record for debug
    A_active=digitalRead(pinA); B_active=digitalRead(pinB); C_active=digitalRead(pinC); // re-read all pins
    new_hall_state = C_active + (B_active << 1) + (A_active << 2); // attempt to recover
    if(new_hall_state == hall_state) { // if glitch persists
      log_d("%u no state change %u%u%u", pinA, (hall_state&0b100)>>2,(hall_state&0b010)>>1,hall_state&0b001); // TLD debug, deleteme
      if(!_glitchFixTimestamp) {_glitchFixTimestamp=millis();} // try to fix glitch at next interrupt or update()
      return; // glitch was not fixed, quit
    } else { // if glitch was fixed
      log_v("\t%u no-state-change glitch fixed %u%u%u -> %u%u%u", pinA, (bad_state&0b100)>>2,(bad_state&0b010)>>1,bad_state&0b001, (new_hall_state&0b100)>>2,(new_hall_state&0b010)>>1,new_hall_state&0b001); // TLD debug, deleteme
    }
  }
  hall_state = new_hall_state;
  _glitchFixTimestamp = 0; // no glitches here

  int8_t new_electric_sector = ELECTRIC_SECTORS[hall_state];
  //// TLD debug:
  if(abs(new_electric_sector-electric_sector) > 1) { // if step is weird (or just rollover)
    if(abs(new_electric_sector-electric_sector) != 5) { // if it's NOT a normal rollover
      if(electric_rotations != 0) { // an extra little patch to avoid this debug message when sensor is initialized
        // digitalWrite(40, pin40State=!pin40State); // toggle debug pin // DELETEME
        log_d("%u skipped state %u -> %u", pinA, electric_sector, new_electric_sector);
      }
    }
  }

  if((new_electric_sector - electric_sector) > 3) {
    //underflow
    direction = Direction::CCW;
    electric_rotations += direction;
  } else if((new_electric_sector - electric_sector) < (-3)) {
    //overflow
    direction = Direction::CW;
    electric_rotations += direction;
  } else {
    direction = (new_electric_sector > electric_sector)? Direction::CW : Direction::CCW;
  }
  if(direction != old_direction) { // TLD debug, deleteme
    static bool pinState; digitalWrite(42, pinState=!pinState); // toggle debug pin // DELETEME
    // log_v("%u dir change %u -> %d", pinA, electric_sector, new_electric_sector);
  }
  electric_sector = new_electric_sector;

  // glitch avoidance #2 changes in direction can cause velocity spikes.  Possible improvements needed in this area
  if(direction == old_direction) {
    // not oscilating or just changed direction
    pulse_diff = new_pulse_timestamp - pulse_timestamp;
  } else {
    pulse_diff = 0;
  }

  pulse_timestamp = new_pulse_timestamp;
  total_interrupts++;
  old_direction = direction;
  if(onSectorChange != nullptr) { onSectorChange(electric_sector); }
}

/**
 * Optionally set a function callback to be fired when sector changes
 * void onSectorChange(int sector) {
 *  ... // for debug or call driver directly?
 * }
 * sensor.attachSectorCallback(onSectorChange);
 */
void HallSensor::attachSectorCallback(void (*_onSectorChange)(int sector)) {
  onSectorChange = _onSectorChange;
}



// Sensor update function. Safely copy volatile interrupt variables into Sensor base class state variables.
void HallSensor::update() {
  // Copy volatile variables in minimal-duration blocking section to ensure no interrupts are missed
  // static bool pin40State, pin42State = false; // TLD debug, deleteme // DELETEME!
  // digitalWrite(40, pin40State=!pin40State); // toggle debug pin // DELETEME!
  #if !(defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)) // if ESP32 is NOT defined
    noInterrupts();
  #endif // interrupts are left enabled for the ESP32, as it runs a RTOS, and it's 32bit (so copying these volatile value is quick enough)
  if(_glitchFixTimestamp) {
    //// attempt to resolve glitch:
    updateState(); // this includes code that attempts to 'fix' the glitch
    if(_glitchFixTimestamp) { // if glitch is still not 'fixed'
      if((millis()-_glitchFixTimestamp) > _glitchFixTimeout) { _glitchFixTimestamp=0; } // stop trying
    }
  }
  angle_prev_ts = pulse_timestamp;
  long last_electric_rotations = electric_rotations;
  int8_t last_electric_sector = electric_sector;
  #if !(defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)) // if ESP32 is NOT defined
    interrupts();
  #endif
  // digitalWrite(42, pin42State=!pin42State); // toggle debug pin // DELETEME!
  angle_prev = ((float)((last_electric_rotations * 6 + last_electric_sector) % cpr) / (float)cpr) * _2PI ;
  full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / cpr);
}



/*
  Shaft angle calculation
  TODO: numerical precision issue here if the electrical rotation overflows the angle will be lost
*/
float HallSensor::getSensorAngle() {
  return(((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI);
}

/*
  Shaft velocity calculation
  function using mixed time and frequency measurement technique
*/
float HallSensor::getVelocity() {
  // Copy volatile variables in minimal-duration blocking section to ensure no interrupts are missed
  // static bool pin40State, pin42State = false; // TLD debug, deleteme // DELETEME!
  // digitalWrite(40, pin40State=!pin40State); // toggle debug pin // DELETEME!
  #if !(defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)) // if ESP32 is NOT defined
    noInterrupts();
  #endif // interrupts are left enabled for the ESP32, as it runs a RTOS, and it's 32bit (so copying these volatile value is quick enough)
  long last_pulse_timestamp = pulse_timestamp;
  long last_pulse_diff = pulse_diff;
  #if !(defined(ESP_H) && defined(ARDUINO_ARCH_ESP32)) // if ESP32 is NOT defined
    interrupts();
  #endif
  // digitalWrite(42, pin42State=!pin42State); // toggle debug pin // DELETEME!
  if((last_pulse_diff == 0) || ((long)(_micros() - last_pulse_timestamp) > last_pulse_diff*2) ) { // last velocity isn't accurate if too old // TLD order of operations check
    return(0);
  } else {
    return((direction * (_2PI / (float)cpr)) / (last_pulse_diff / 1000000.0f)); // TLD order of operations check
  }

}

// HallSensor initialisation of the hardware pins 
// and calculation variables
void HallSensor::init() {
  // initialise the electrical rotations to 0
  electric_rotations = 0;

  // HallSensor - check if pullup needed for your HallSensor
  if(pullup == Pullup::USE_INTERN) {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);
  } else {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinC, INPUT);
  }

    // init hall_state
  A_active= digitalRead(pinA);
  B_active = digitalRead(pinB);
  C_active = digitalRead(pinC);
  updateState();

  pulse_timestamp = _micros();

  // we don't call Sensor::init() here because init is handled in HallSensor class.
}

// function enabling hardware interrupts for the callback provided
// if callback is not provided then the interrupt is not enabled
void HallSensor::enableInterrupts(void (*doA)(), void(*doB)(), void(*doC)()) {
  // attach interrupt if functions provided

  // A, B and C callback
  if(doA != nullptr) { attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE); }
  if(doB != nullptr) { attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE); }
  if(doC != nullptr) { attachInterrupt(digitalPinToInterrupt(pinC), doC, CHANGE); }
}
