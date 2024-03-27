//* AnalysIR Firmware for ESP8266 via Serial & WiFi
//* Date: 12th January 2016
//* Release: 1.0.x
//* Note: Please use and read in conjunction with the README and the AnalysIR Getting Started Guide.
//* Licence: Free to use & modify without restriction or warranty for non-commercial uses.
//*          Free to use & modify without restriction or warranty for commercial uses with a registered copy of AnalysIR.
//*          For other uses please contact the Author directly.
//*          Please acknowledge AnalysIR as the Author in any derivative and include a link to http://www.AnalysIR.com in any publication.
//*
//* Tested with Arduino IDE  1.6.5
/*
ESP8266 NodeMCU GPIO pin mapping
D0   = 16;  //GPIO16
D1   = 5;   //GPIO5
D2   = 4;   //GPIO4
D3   = 0;   //GPIO0
D4   = 2;   //GPIO2 .... often configured for LED
D5   = 14;  //GPIO14
D6   = 12;  //GPIO12
D7   = 13;  //GPIO13
D8   = 15;  //GPIO15
D9   = 3;   //GPIO3
D10  = 1;   //GPIO1
NB: IMPORTANT - Please verify the pin mapping of your own device before making any connections or powering-up
 */


//set true for Micro-controller system being used, set all others to false
#include <Arduino.h>
#include <ESP8266WiFi.h>
#define ESP8266PLATFORM true    //set to true for Photon   


//Definitions for Buffers here...size RAM dependent
#define modPULSES 256 //increase until memory is used up, max 256, leave at 256.
#define byte unsigned char
#define boolean unsigned char
//General Definitions
void ICACHE_RAM_ATTR rxIR_Interrupt_Handler();
void ICACHE_RAM_ATTR rxIR3_Interrupt_Handler();
#if ESP8266PLATFORM
#define AVR8BITMCU false
#define IR_Rx_PIN D5   //Actually GPIO14 on ESP8266 (Dont use D4/GPIO2 as it often has the LED attached). 
#define IR_Mod_PIN D6  //Actually GPIO12 on ESP8266 (Dont use D4/GPIO2 as it often has the LED attached)
#define ledPin    2
#define LEDHIGH LOW
#define LEDLOW HIGH
#define pin2HIGH digitalRead(IR_Rx_PIN)
#define maxPULSES 1024 //More RAM is available on this Photon
#else
ERROR - you must define Photon as true above
#endif
//see *** note 1 in README
#define enableIRrx attachInterrupt(IR_Rx_PIN, rxIR_Interrupt_Handler, CHANGE) //set up interrupt handler for IR rx   on pin 2 - demodulated signal
#define enableIRrxMOD attachInterrupt(IR_Mod_PIN, rxIR3_Interrupt_Handler, FALLING) //Same for Pin3 - modulated signal
#define disableIRrx detachInterrupt(IR_Rx_PIN) //disable interrupt handler for IR rx   on pin 2 - demodulated signal
#define disableIRrxMOD detachInterrupt(IR_Mod_PIN) //Same for Pin3 - modulated signal

//Baud rate is now fixed to 115200 for all devices, to avoid issues with some platforms
#define BAUDRATE 115200

#define SIGNALGAP 125000 //determines gap between signals (typical range 100000->125000)
//...............................................................................................

//see *** note 3 in README
uint16_t pulseIR[maxPULSES]; //temp store for pulse durations (demodulated)
volatile byte modIR[modPULSES]; //temp store for modulation pulse durations - changed in interrupt

//General variables
//see *** note 5 in README
volatile byte state = 127; //defines initial value for state normally HIGH or LOW. Set in ISR.
byte oldState = 127; //set both the same to start. Used to test for change in state signalled from ISR
unsigned long usLoop, oldTime; //timer values in uSecs. usLoops stores the time value of each loop, oldTime remembers the last time a state change was received
volatile unsigned long newMicros;//passes the time a state change occurred from ISR to main loop
unsigned long oldMicros = 0; //passes the time a state change occurred from ISR to main loop
uint16_t countD = 0; // used as a pointer through the buffers for writing and reading (de-modulated signal)
volatile byte countM = 0; // used as a pointer through the buffers for writing and reading (modulated signal)
byte countM2 = 0; //used as a counter, for the number of modulation samples used in calculating the period.
uint16_t i = 0; //used to iterate in for loop...integer
byte j = 0; //used to iterate in for loop..byte
unsigned long sum = 0; //used in calculating Modulation frequency
byte sigLen = 0; //used when calculating the modulation period. Only a byte is required.
volatile boolean intDirection = true; //only used for Fubarino/ChipKit, can be removed for other platforms if neccessary

//Serial Tx buffer - uses Serial.write for faster execution
//see *** note 6 in README
byte txBuffer[5]; //Key(+-)/1,count/1,offset/4,CR/1   <= format of packet sent to AnalysIR over serial



//see *** note 12 in README
void  reportPulses() {
      yield(); //avoid watchdog issues
  for (i = 0; i < countD; i++) {
    //the following logic takes care of the inverted signal in the IR receiver
    if (pulseIR[i] & 0x01) txBuffer[0] = '+'; //Mark is sent as +...LSB bit of pulseIR is State(Mark or Space)
    else txBuffer[0] = '-';           //Space is sent as -
    txBuffer[1] = (byte) (i & 0xFF); //count
    txBuffer[3] = pulseIR[i] >> 8; //byte 1
    txBuffer[2] = pulseIR[i] & 0xFE; //LSB 0 ..remove lat bit as it was State
    Serial.write(txBuffer, 5);
    yield(); //avoid watchdog issues
  }
}

//see *** note 13 in README
void  reportPeriod() { //report period of modulation frequency in nano seconds for more accuracy
  yield(); //avoid watchdog issues
  sum = 0; // UL
  sigLen = 0; //byte
  countM2 = 0; //byte
  for (j = 1; j < (modPULSES - 1); j++) { //i is byte
    sigLen = (modIR[j] - modIR[j - 1]); //siglen is byte
    if (sigLen > 50 || sigLen < 10) continue; //this is the period range length exclude extraneous ones
    sum += sigLen; // sum is UL
    countM2++; //countM2 is byte
    modIR[j - 1] = 0; //finished with it so clear for next time
  }
  modIR[j - 1] = 0; //now clear last one, which was missed in loop

  if (countM2 == 0) return; //avoid div by zero = nothing to report
  sum =  sum * 1000 / countM2; //get it in nano secs
  // now send over serial using buffer
  txBuffer[0] = 'M'; //Modulation report is sent as 'M'
  txBuffer[1] = countM2; //number of samples used
  txBuffer[3] = sum >> 8 & 0xFF; //byte Period MSB
  txBuffer[2] = sum & 0xFF; //byte Period LSB
  Serial.write(txBuffer, 5);
  yield(); //avoid watchdog issues
  return;
}

//see *** note 14 in README
void rxIR_Interrupt_Handler() { //important to use few instruction cycles here
  //digital pin 2 on Arduino
  newMicros = micros(); //record time stamp for main loop
  state = pin2HIGH; //read changed state of interrupt pin 2 (return 4 or 0 for High/Low)
  digitalWrite(ledPin, !digitalRead(ledPin));

}

//see *** note 15 in README
void rxIR3_Interrupt_Handler() { //important to use few instruction cycles here
  //digital pin 3 on Arduino - FALLING edge only
  modIR[countM++] = micros(); //just continually record the time-stamp, will be mostly modulations
  //just save LSB as we are measuring values of 20-50 uSecs only - so only need a byte (LSB)
}




void setup() {

  Serial.begin(115200);//fixed at 115200 bps for all platforms
  // delay(500);//to avoid potential conflict with boot-loader on some systems
  //while(!Serial);
  txBuffer[4] = 13; //init ascii decimal value for CR in tx buffer

  pinMode(IR_Rx_PIN, INPUT);
  pinMode(IR_Mod_PIN, INPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(IR_Rx_PIN,LOW);
  digitalWrite(IR_Mod_PIN,LOW);

  // the following 4 lines are used to power teh A.IRrx shield and can be removed if not using this shield
  //  pinMode(4, OUTPUT);
  //  pinMode(5, OUTPUT);
  //  digitalWrite(4, LOW); //provide GND for A.IRrx Shield
  //  digitalWrite(5, HIGH); //provide Vcc/5V for A.IRrx Shield
  //

  digitalWrite(ledPin, LEDHIGH); //if LED stays on after reset, then serial not recognised PC
  delay(500);//time to see blink

  digitalWrite(ledPin, LEDLOW);

  Serial.println(); Serial.println("!AnalysIR!"); // HELLO STRING - ALL COMMENTS SENT IN !....! FORMAT
Serial.println(D0);Serial.println(D5);

  //Initialise State
  oldState = digitalRead(IR_Rx_PIN);
  state = oldState;

  //Initialise Times
  oldTime = 0; //init
  newMicros = micros(); //init
  oldMicros = newMicros;

  //following line not required - just reports free RAM on Arduino if there are problems
  //reportFreeRAM(0xFFFF);// report free ram to host always, use max UInt value of 0xFFFF. 8Bit AVRs only
  //

  //turn on interrupts and GO!
  enableIRrx;//set up interrupt handler for IR rx on pin 2 - demodulated signal
  enableIRrxMOD; //set up interrupt handler for modulated IR rx on pin 3 - full modulated signal
  yield(); //avoid watchdog issues
}

//see *** note 8 in README
void loop() {
  // while (true) { //avoid any extra stuff inserted by Arduino IDE at end of each normal Loop

  usLoop = micros(); //used once every loop rather than multiple calls
  //see *** note 9 in README
  if (oldState != state && countD < maxPULSES) {
    oldState = state;
    if (oldState) { //if the duration is longer than 0xFFFF(65535 uSecs) then multiple repeat pulses are stored, whith LSB used to signal mark or space
      sum = (newMicros - oldMicros); //re-use sum var here, to save RAM (normally used in reportperiod)
      while (sum > 0xFFFF && countD < (maxPULSES - 1) && countD) { //this allows for a mark/space of greater than 65535 uSecs (0xFFFF), ignore first signal
        sum -= 65535;//this assumes the length is not longer than 131070
        pulseIR[countD++] = 65535 | 0x0001; //store for later & include state
      }
      pulseIR[countD++] = sum | 0x0001; //store for later & include state
    }
    else {
      sum = (newMicros - oldMicros); //re-use sum var here, to save RAM (normally used in reportperiod)
      while (sum > 0xFFFF && countD < (maxPULSES - 1) && countD) { //this allows for a mark/space of greater than 65535 uSecs (0xFFFF), ignore first signal
        sum -= 65535;//this assumes the length is not longer than 131070
        pulseIR[countD++] = 65535 & 0xFFFE; //store for later & include state
      }
      pulseIR[countD++] = sum & 0xFFFE; //store for later & include state
    }
    oldMicros = newMicros; //remember for next time
    oldTime = usLoop; //last time IR was received
  }

  //see *** note 10 in README
  if (state && countD > 0 && (countD == maxPULSES ||  (usLoop - oldTime) > SIGNALGAP)) { //if we have received maximum pulses or its 100ms since last one
    disableIRrx;  //disable interrupt handler for IR rx   on pin 2 - demodulated signal
    disableIRrxMOD;  //Same for Pin3 - modulated signal
    reportPulses();
    reportPeriod();//reports modulation frequency to host over serial
    countD = 0; //reset value for next time
    //turn on interrupts and GO!
    enableIRrx;//set up interrupt handler for IR rx on pin 2 - demodulated signal
    enableIRrxMOD; //set up interrupt handler for modulated IR rx on pin 3 - full modulated signal

  }

  //see *** note 11 in README
  /*  this code only used, for debugging, if we are having problems with available RAM
   reportFreeRAM(200);//report freeram as comment to host if less than 200 bytes
   */
  //  } //while true

}
