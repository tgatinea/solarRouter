/*
  Solar router based upon the Linky TIC information to pilot a dimmer
  The dimmer is connected to an electrical water tank that is dedicated to consume the exceeded solar power production
  
  See Enedis-NOI-CPT_54E documentation for TIC data

  1) Linky standard mode: 
  ----------------------
  Define STANDARD_TIC to be in this mode.

  The baud rate is 9600.
  
  TIC frame structure with horodate:
  +------+-------+-------+----------+-------+------+-------+----------+------+
  ! <LF> ! Label ! <TAB> ! Horodate ! <TAB> ! Data ! <TAB> ! Checksum ! <CR> !
  ! 0x0A !       ! 0x09  !          ! 0x09  !      ! 0x09  !          ! 0x0D !
  +------+-------+-------+----------+-------+------+-------+----------+------+
         ! <-          Controlled by checksum           -> !
         +-------------------------------------------------+

  TIC frame structure without horodate:
  +------+-------+-------+------+-------+----------+------+
  ! <LF> ! Label ! <TAB> ! Data ! <TAB> ! Checksum ! <CR> !
  ! 0x0A !       ! 0x09  !      ! 0x09  !          ! 0x0D !
  +------+-------+-------+------+-------+----------+------+
         ! <- Controlled by checksum -> !
         +------------------------------+

  Checksum calculated = (Sum of controlled character & 0x3F) + 0x20
  
  The algorithm is based upon the SINSTS message that provides the instantaneous value of power consumption in VA.
  When SINSTS is 0, then the solar pannel provides probably more power than the electricity power consumption of the house.
  Note that:
  - SINSTI message would have been the killer data for such an algorithm, but this data is available only on Linky in production mode.
  - Linky is not configured into production mode in the use case of auto consumption with zero injection.

  2) Linky historic mode:
  ----------------------
  This is the default mode of the Linky if STANDARD_TIC is unset.

  The baud rate is 1200.
  
  TIC frame structure:
  +------+-------+------+---------------+------+----------+------+
  ! <LF> ! Label ! <SP> !      Data     ! <SP> ! Checksum ! <CR> !
  ! 0x0A !       ! 0x20 !               ! 0x20 !          ! 0x0D !
  +------+-------+------+---------------+------+----------+------+
         ! <- Controlled by checksum -> !
         +------------------------------+

  Checksum calculated = (Sum of controlled character & 0x3F) + 0x20
  
  The algorithm is based upon the PAPP message that provides the instantaneous value of power consumption in VA.
  When PAPP is 0, then the solar pannel provides probably more power than the electricity power consumption of the house.
  
*/

#include <AltSoftSerial.h>

// Undef STANDARD_TIC to be in Linky historic mode
#define STANDARD_TIC

// Only RX_PIN is involved with the Linky I1 and I2 pins
#define RX_PIN 8
#define TX_PIN 9

AltSoftSerial Linky(RX_PIN, TX_PIN, false);

//
// Variables of the dimmer management:
//

#define MAX_DIMMER_POWER 800
int dimmerPower = 0;

// Dimmer machine states
#define DIMMER_IDLE 0
#define DIMMER_INCREMENTATION 1
#define DIMMER_STABILIZED 2

int dimmerState = DIMMER_IDLE;

#define DIMMER_FAST_INCREMENT 100
#define DIMMER_SLOW_INCREMENT 10

//
// Variables used for Linky TIC data:
//

#define MAX_BUFFER_LENGTH 20
char buffer[MAX_BUFFER_LENGTH];
int index = 0;

// Machine state to recover Linky messages of instantaneous power consumption 
// e.g. SINSTS in case of Linky standard mode or PAPP in case of Linky historical mode
#define WAIT_LF 0
#define WAIT_C1 1
#define WAIT_C2 2
#define WAIT_C3 3
#define WAIT_C4 4
#define WAIT_C5 5
#define WAIT_C6 6
#define WAIT_SEPARATOR1 7
#define WAIT_D1 8
#define WAIT_D2 9
#define WAIT_D3 10
#define WAIT_D4 11
#define WAIT_D5 12
#define WAIT_SEPARATOR2 13
#define WAIT_CHECKSUM 14
#define WAIT_CR 15

int msgState = WAIT_LF;

// ASCII characters
#define NUL 0x00
#define LF  0x0A
#define CR  0x0D
#define TAB 0x09
#define SP  0x20

#ifdef STANDARD_TIC
// Linky standard mode: Field separator is a tabulation and the baud rate is 9600
#define FIELD_SEPARATOR TAB
#define TIC_BAUD_RATE 9600
#else 
// Linky standard mode: Field separator is a Space and the baud rate is 1200
#define FIELD_SEPARATOR SP
#define TIC_BAUD_RATE 1200
#endif

#ifdef STANDARD_TIC
// Message should be SINSTS in standard mode
#define C1 0x53 
#define C2 0x49
#define C3 0x4E
#define C4 0x53
#define C5 0x54
#define C6 0x53
#define c1 0x73 
#define c2 0x69
#define c3 0x6E
#define c4 0x73
#define c5 0x74
#define c6 0x73
#else
// Message should be PAPP in historic mode
#define C1 0x50 
#define C2 0x41
#define C3 0x50
#define C4 0x50
#define c1 0x71 
#define c2 0x61
#define c3 0x71
#define c4 0x71
#endif

/*
 * Feed buffer with SINSTS or PAPP message and return true when the message is received with a good checksum
 */
bool isNewMsgInBuffer(char c) {
  bool ret = false;
  
  if (c == NUL) {
    return ret;
  }
  
  // New frame starts with a line feed
  if ((msgState == WAIT_LF) && (c == LF)) { 
    msgState = WAIT_C1;
    index = 0;
  }
  else if (msgState == WAIT_C1) {
    if ((c == C1) || (c == c1)) {
      msgState = WAIT_C2;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_C2) {
    if ((c == C2) || (c == c2)) {
      msgState = WAIT_C3;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_C3) {
    if ((c == C3) || (c == c3)) {
      msgState = WAIT_C4;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_C4) {
    if ((c == C4) || (c == c4)) {
#ifdef STANDARD_TIC
      msgState = WAIT_C5;
#else
      msgState = WAIT_SEPARATOR1;
#endif
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
 #ifdef STANDARD_TIC 
  else if (msgState == WAIT_C5) {
    if ((c == C5) || (c == c5)) {
      msgState = WAIT_C6;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_C6) {
    if ((c == C6) || (c == c6)) {
      msgState = WAIT_SEPARATOR1;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
#endif
  else if (msgState == WAIT_SEPARATOR1) {
    if (c == FIELD_SEPARATOR) {
      msgState = WAIT_D1;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_D1) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D2;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_D2) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D3;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_D3) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D4;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_D4) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D5;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_D5) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_SEPARATOR2;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_SEPARATOR2) {
    if (c == FIELD_SEPARATOR) {
      msgState = WAIT_CHECKSUM;
      buffer[index] = c;
      index++;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }
  else if (msgState == WAIT_CHECKSUM) {
    msgState = WAIT_CR;
    buffer[index] = c;
    index++;
  }
  else if (msgState == WAIT_CR) {
    if (c == CR) {
      if (isChecksumGood()) {
        ret = true;
      }
      msgState = WAIT_LF;
      index = 0;
    }
    else {
      msgState = WAIT_LF;
      index = 0;
    }
  }

  return ret;
}

/*
 * Checksum calculation; Returns true if the checksum is correct, false otherwise
 */
bool isChecksumGood() {
  char checksum;
  unsigned int sum = 0;
  
  if (index == 0) {
    // Error: Empty frame
    return false;
  }
  
  checksum = buffer[index - 1];

#ifdef STANDARD_TIC
  for (int i = 0; i<(index - 1); i++) {
    sum += (unsigned int)buffer[i];
  }
#else
  for (int i = 0; i<(index - 2); i++) {
    sum += (unsigned int)buffer[i];
  }
#endif
  
  if (checksum != (char)((sum & 0x3F) + 0x20)) {
    return false; 
  }
  
  return true;
}

/*
  Returns the instantaneous power consumption in VA
 */
int getData() {
  int i;
  String data = "";

#ifdef STANDARD_TIC  
  for (i=7; i<12; i++) {
    data.concat(buffer[i]);
  }
#else
  for (i=5; i<10; i++) {
    data.concat(buffer[i]);
  }
#endif
  return data.toInt();
}

/*
 * Management of the dimmer power according to the instantaneous power consumption.
 * When the instantaneous power consumption is null, then it is probable that the solar pannel inject into the network.
 * The target is to increase quickly the dimmer power until instantaneous power consumption becames positive and then to
 * re-decrease the dimmer power such as instantaneous power consumption reaches again null.
 * When it is stabilized the target is to periodically retry to increase/decrease slowly the dimmer power to remains 
 * with instantaneous power consumption to null.
 * The power dimmer is limited to a maximum of 800W because I have only 2 solar pannels on a tracker.
 * This is a low power but the tracker increase the duration of production during the whole day.
 */
void processElectricalConsumption() {
  int electricalConsumption = getData();
  int delta = dimmerPower - electricalConsumption;
  int delayInMs = 1000;
  int nextDimmerPower = 0;

  if (dimmerState == DIMMER_IDLE) {
    if (electricalConsumption > 0) {
      delayInMs = 1000;
      nextDimmerPower = 0;
      dimmerState = DIMMER_IDLE;
    }
    else {
      delayInMs = 1000;
      nextDimmerPower = DIMMER_FAST_INCREMENT;
      dimmerState = DIMMER_INCREMENTATION;
    }
  }
  else if (dimmerState == DIMMER_INCREMENTATION) {
    if (electricalConsumption == 0) {
      delayInMs = 1000;
      nextDimmerPower = dimmerPower + DIMMER_FAST_INCREMENT;
      dimmerState = DIMMER_INCREMENTATION;
    }
    else if (dimmerPower > electricalConsumption) {
      delayInMs = 1000;
      nextDimmerPower = dimmerPower - electricalConsumption;
      dimmerState = DIMMER_STABILIZED;
    }
    else {
      delayInMs = 1000;
      nextDimmerPower = 0;
      dimmerState = DIMMER_IDLE;
    }
  }
  else if (dimmerState == DIMMER_STABILIZED) {
    if (electricalConsumption == 0) {
      delayInMs = 1000;
      nextDimmerPower = dimmerPower + DIMMER_SLOW_INCREMENT;
      dimmerState = DIMMER_STABILIZED;
    }
    else if (dimmerPower > electricalConsumption) {
      delayInMs = 1000;
      nextDimmerPower = dimmerPower - electricalConsumption;
      dimmerState = DIMMER_STABILIZED;
    }
    else {
      delayInMs = 1000;
      nextDimmerPower = 0;
      dimmerState = DIMMER_IDLE;
    }
    
  }
  else {
    // Unknown dimmer state
  }
  
  // Some protection 
  if (nextDimmerPower > MAX_DIMMER_POWER) {
    nextDimmerPower = MAX_DIMMER_POWER;
  }
  if (nextDimmerPower < 0) {
    nextDimmerPower = 0;
  }

  processDimmerPower(nextDimmerPower, delayInMs);
}

/*
 * Programation of the dimmer with new power
 */
void processDimmerPower(int nextDimmerPower, int delayInMs) {
  Serial.print("Previous dimmer ");
  Serial.print(dimmerPower);
  Serial.print("VA Next dimmer ");
  Serial.print(nextDimmerPower);
  Serial.print("VA Electrical power consumption ");
  Serial.print(getData());
  Serial.print("VA Delay ");
  Serial.print(delayInMs);
  Serial.print("ms\n");
  
  dimmerPower = nextDimmerPower;

  // Todo: Program hardware RobotDyn dimmer dimmer
  
  delay(delayInMs);
}

void setup() {
  Linky.begin(TIC_BAUD_RATE);
  Serial.begin(9600);
  // Todo: Dimmer initialization and LCD 
  Serial.print("End of setup...");
}

void loop() {
  char inChar;
  int val;
  
  while (Linky.available()) {
    // read the new char
    val = Linky.read();
    if (val != -1) {
      // Process instantaneous electical power when received from Linky   
      if (isNewMsgInBuffer((char)val & 0x7F)) {
        processElectricalConsumption();
      }
    }
  }
}
