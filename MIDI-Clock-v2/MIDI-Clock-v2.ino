/**
 * MIDI Clock v2 - sends MIDI_CLOCK messages in MIDI Channel 1
 * this clock is adapted to a TAP time received by a tap-button.
 * time intervals between following taps are calculated and compared with CLOCK intervals
 * a PID controller adapts the new clock intervals to the received tap
 * TODO:
 * Control of the phase of the song. CLOCK control adapts bpms, but it is also necessary 
 * to adapt the song playing point to the place where it is the strong hit.
 * Strong hit detection could be made by another different TAP
 * 
 */

#include <TimerOne.h>

#define MIDI_ON
/*
 * FEATURE: TAP BPM INPUT
 */
#define TAP_PIN 4
//#define TAP_PIN_POLARITY RISING
#define TAP_PIN_POLARITY LOW


#define MINIMUM_TAPS 3
#define EXIT_MARGIN 150 // If no tap after 150% of last tap interval -> measure and set

/*
 * FEATURE: DIMMER BPM INPUT
 */
#define DIMMER_INPUT_PIN A0

#define DIMMER_CHANGE_MARGIN 20 // Big value to make sure this doesn't interfere. Tweak as needed.

/*
 * FEATURE: DIMMER BPM INCREASE/DECREASE
 * establece el cambio de bpm como velocidad de cambio
 */
//#define DIMMER_CHANGE_PIN A0
#define DEAD_ZONE 50
#define CHANGE_THRESHOLD 5000
#define RATE_DIVISOR 30

/*
 * FEATURE: BLINK TEMPO LED
 */
#define BLINK_OUTPUT_PIN 10
#define BLINK_PIN_POLARITY 255  // 0 = POSITIVE, 255 - NEGATIVE
#define BLINK_TIME 4 // How long to keep LED lit in CLOCK counts (so range is [0,24])

/*
 * FEATURE: SYNC PULSE OUTPUT
 */
#define SYNC_OUTPUT_PIN 9 // Can be used to drive sync analog sequencer (Korg Monotribe etc ...)
#define SYNC_PIN_POLARITY 0 // 0 = POSITIVE, 255 - NEGATIVE

/*
 * FEATURE: Send MIDI start/stop
 */
//#define START_STOP_INPUT_PIN A1
#define START_STOP_INPUT_PIN 3
#define START_STOP_PIN_POLARITY 0 // 0 = POSITIVE, 1024 = NEGATIVE

#define MIDI_START 0xFA
#define MIDI_STOP 0xFC

#define DEBOUNCE_INTERVAL 500L // Milliseconds

/*
 * FEATURE: EEPROM BPM storage
 */
#define EEPROM_ADDRESS 0 // Where to save BPM
#ifdef EEPROM_ADDRESS
#include <EEPROM.h>
#endif


/*
 * GENERAL PARAMETERS
 */
#define MIDI_TIMING_CLOCK 0xF8
#define CLOCKS_PER_BEAT 24
#define MINIMUM_BPM 400 // Used for debouncing
#define MAXIMUM_BPM 3000 // Used for debouncing

long intervalMicroSeconds;
int bpm = 1200;  // BPM in tenths of a BPM!!
long interval;
long intervalOld;

boolean initialized = false;
long minimumTapInterval = 60L * 1000 * 1000 * 10 / MAXIMUM_BPM;
long maximumTapInterval = 60L * 1000 * 1000 * 10 / MINIMUM_BPM;

volatile long lastTapTime = 0; // ultima vez que se pulso el TAP
volatile long lastTapTimeOld = 0; // penultima vez que se pulso el TAP
volatile long timesTapped = 0; 
volatile long lastClockTime = 0; // ultima vez que se enviaron CLOCKS_PER_BEAT mensajes

volatile int blinkCount = 0;

int lastDimmerValue = 0;

boolean playing = false;
long lastStartStopTime = 0;

#ifdef DIMMER_CHANGE_PIN
long changeValue = 0;
#endif
int tapValueOld = HIGH;
long intervalTap;
double Kp = 1.0;
double Kd = 1.0;
double Ki = 1.0;
long timeError; // error entre el ultimo tap y el ultimo clock. error de fase 
long timeErrorOld; 
long intervalError; // error entre el intervalo del clock y el intervalo de los taps. error de periodo
long intervalErrorOld; 

void setup() {
  #ifdef MIDI_ON
//    Serial.begin(31250);
    Serial.begin(9600);
  #else 
    Serial.begin(9600);
  #endif
  
  // Set pin modes
  pinMode(BLINK_OUTPUT_PIN, OUTPUT);
  analogWrite(BLINK_OUTPUT_PIN, HIGH );
  delay(500);
  analogWrite(BLINK_OUTPUT_PIN, LOW );

  pinMode(SYNC_OUTPUT_PIN, OUTPUT);
  analogWrite(SYNC_OUTPUT_PIN, HIGH );
  delay(500);
  analogWrite(SYNC_OUTPUT_PIN, LOW );

#ifdef START_STOP_INPUT_PIN
  pinMode(START_STOP_INPUT_PIN, INPUT_PULLUP);
#endif

  pinMode(TAP_PIN, INPUT_PULLUP);

#ifdef EEPROM_ADDRESS
  // Get the saved BPM value from 2 stored bytes: MSB LSB
  bpm = EEPROM.read(EEPROM_ADDRESS) << 8;
  bpm += EEPROM.read(EEPROM_ADDRESS + 1);
  if (bpm < MINIMUM_BPM || bpm > MAXIMUM_BPM) {
    bpm = 1200;
  }
#endif


  // Attach the interrupt to send the MIDI clock and start the timer
  #ifndef MIDI_ON
    Serial.print( "Begining Timer at: " );
    Serial.println( intervalMicroSeconds );
    Serial.print( "Begining Timer at bpm = " );
    Serial.print( bpm );
    Serial.print( "  interval = " );
    Serial.println( interval );
  #endif  
  Timer1.initialize( intervalMicroSeconds );
  interval = bpm2interval(bpm) ;
  Timer1.setPeriod( interval );
  Timer1.attachInterrupt( sendClockPulse );
  intervalOld = interval;

#ifdef DIMMER_INPUT_PIN
  // Initialize dimmer value
  lastDimmerValue = analogRead( DIMMER_INPUT_PIN );
#endif

}

void loop() {
  long now = micros();

  int tapValue = digitalRead( TAP_PIN );
  if( tapValue == LOW && tapValue!= tapValueOld) {
    if( tapInput() == 0) { // update lastTapTime
     timeError = lastTapTime - lastClockTime; // lastClockTime updated by Timer1 each CLOCK_PER_BEAT
     intervalError = intervalTap / CLOCKS_PER_BEAT - interval;
     /*
     Serial.println();
     Serial.print( "err = " );
     Serial.print( timeError);
     Serial.print( " Derr = " );
     Serial.print( timeError - timeErrorOld);
     Serial.print( " Ierr = " );
     Serial.println( timeError + timeErrorOld);
*/
  #ifndef MIDI_ON
     Serial.println();
     Serial.print( "err = " );
     Serial.print( intervalError);
     Serial.print( " Derr = " );
     Serial.print( intervalError - intervalErrorOld);
     Serial.print( " Ierr = " );
     Serial.println( intervalError + intervalErrorOld);
  #endif

     Kd = -0.1;     
     Ki = 0;     
//     interval = interval + Kp * timeError + Kd * (timeError - timeErrorOld);
     long intervalOld_10 = intervalOld / 10L;
     interval = interval + Kp * intervalError + Kd * (intervalError - intervalErrorOld) + Ki * (intervalError + intervalErrorOld);
     interval = constrain( interval, intervalOld - intervalOld_10, intervalOld + intervalOld_10 ); // para filtrar saltos bruscos
     updateBpm(interval);
     intervalOld = interval;
    }
  } else {
 //   timeError = 0;
  }
  tapValueOld = tapValue;
  
  noInterrupts();
  int blinkCopy = blinkCount;
  interrupts();
  if (blinkCopy % CLOCKS_PER_BEAT == 0) {
    // Update blinkCount to make sure LED blink matches tapped beat
    noInterrupts();
//    blinkCount = ((now - lastTapTime) * 24 / interval) % CLOCKS_PER_BEAT;
    blinkCount = 0;
    interrupts();
  }
  
  timeErrorOld = timeError;
  intervalErrorOld = intervalError;

//  delay(10);


#ifdef DIMMER_INPUT_PIN
  /*
   * Handle change of the dimmer input
   */
  int curDimValue = analogRead(DIMMER_INPUT_PIN);
  if (curDimValue > lastDimmerValue + DIMMER_CHANGE_MARGIN
      || curDimValue < lastDimmerValue - DIMMER_CHANGE_MARGIN) {
    // We've got movement!!
    bpm = map(curDimValue, 0, 1024, MINIMUM_BPM, MAXIMUM_BPM);

    updateBpm(now);
    lastDimmerValue = curDimValue;
  }
#endif

#ifdef DIMMER_CHANGE_PIN
  int curDimValue = analogRead(DIMMER_CHANGE_PIN);
  if (bpm > MINIMUM_BPM && curDimValue < (512 - DEAD_ZONE)) {
    int val = (512 - DEAD_ZONE - curDimValue) / RATE_DIVISOR;
    changeValue += val * val;
  } else if (bpm < MAXIMUM_BPM && curDimValue > (512 + DEAD_ZONE)) {
    int val = (curDimValue - 512 - DEAD_ZONE) / RATE_DIVISOR;
    changeValue += val * val;
  } else {
    changeValue = 0;
  }
  if (changeValue > CHANGE_THRESHOLD) {
    bpm += curDimValue < 512 ? -1 : 1;
    updateBpm(now);
    changeValue = 0;
  }
#endif

#ifdef START_STOP_INPUT_PIN
  /*
   * Check for start/stop button pressed
   */
//  int startStopTap =  digitalRead( START_STOP_PIN );
  boolean startStopPressed = digitalRead( START_STOP_INPUT_PIN );
//  if (startStopPressed && (lastStartStopTime + (DEBOUNCE_INTERVAL * 1000)) < now) {
  if (startStopPressed == LOW ) {
//    Serial.println("START/STOP");
    startOrStop();
    lastStartStopTime = now;
  }
#endif
}

int tapInput() {
  long now = micros();
  intervalTap = now - lastTapTime;
  if ( intervalTap < minimumTapInterval )
    return -1; // Debounce

  #ifndef MIDI_ON
    Serial.println();
    Serial.print("Tap OK: ");
    Serial.println(intervalTap);
  #endif
  lastTapTimeOld = lastTapTime;
  lastTapTime = now;
  if( intervalTap > maximumTapInterval ) 
    return -2;
  return(0);
}

void sendClockPulse() {
  // Write the timing clock byte
  #ifdef MIDI_ON
    Serial.write(MIDI_TIMING_CLOCK);
  #endif
//  Serial.print( blinkCount );
  #ifndef MIDI_ON
    Serial.print( "." );
  #endif
  blinkCount ++;
  if (blinkCount % CLOCKS_PER_BEAT == 0) {
    // Turn led on
  #ifndef MIDI_ON
    Serial.println( ">" );
  #endif
    analogWrite(SYNC_OUTPUT_PIN, 255 );
    lastClockTime = micros();
  } else {
    if (blinkCount % CLOCKS_PER_BEAT == BLINK_TIME) { // how long keep led lit
      // Turn led off
      analogWrite(SYNC_OUTPUT_PIN, 0 );
    }
  }
}

void updateBpm(long interval) {
  // Update the timer
  bpm = interval2bpm( interval );
  #ifndef MIDI_ON
    Serial.print( "Updating BPM: " );
    Serial.println( bpm );
  #endif
  Timer1.setPeriod( interval );

#ifdef EEPROM_ADDRESS
  // Save the BPM in 2 bytes, MSB LSB
  EEPROM.write(EEPROM_ADDRESS, bpm / 256);
  EEPROM.write(EEPROM_ADDRESS + 1, bpm % 256);
#endif

/*  Serial.print("Set BPM to: ");
  Serial.print(bpm / 10);
  Serial.print('.');
  Serial.println(bpm % 10);
*/
}

int interval2bpm(long interval) {
  // Take care about overflows!
  return 60L * 1000 * 1000 * 10 / interval / CLOCKS_PER_BEAT;
}

long bpm2interval(int bpm) {
  // Take care about overflows!
  return 60L * 1000 * 1000 * 10 / bpm / CLOCKS_PER_BEAT;
}

void startOrStop() {
  if (!playing) {
//    Serial.println("Start playing");
    Serial.write(MIDI_START);
  } else {
//    Serial.println("Stop playing");
    Serial.write(MIDI_STOP);
  }
  playing = !playing;
}

#ifdef TM1637_DISPLAY
void setDisplayValue(int value) {
  tm1637_data[0] = value >= 1000 ? display.encodeDigit(value / 1000) : 0x00;
  tm1637_data[1] = value >= 100 ? display.encodeDigit((value / 100) % 10) : 0x00;
  tm1637_data[2] = value >= 10 ? display.encodeDigit((value / 10) % 10) : 0x00;
  tm1637_data[3] = display.encodeDigit(value % 10);
  display.setSegments(tm1637_data);
}
#endif
