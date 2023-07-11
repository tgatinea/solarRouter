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


  Tricks: 
  Note that the dimmer is programmed through a % 
  This means that if the electrical water tank can consumme 2000W while the solar pannel can produce a maximum of 800W then
  the % is in a range of 0% to 800/2000*100=40%

*/
// Serial hardware
#include <HardwareSerial.h>

// watchdog
#include <esp_task_wdt.h>

// Dimmer
#include <RBDdimmer.h>

// Undef STANDARD_TIC to be in Linky historic mode
#define STANDARD_TIC

// #define DEBUG


//
// Variables of the dimmer management:
//

/*
 * The table dimmerPwr provides a relation between a % of dimmer and a power consumption
 * This table needs to be calibrated according to the recepter (e.g. electric hot water tank, ...)
 * In the following, the values corresponds to an halogen lamp of 500W that was used for test
 */
int dimmerPwrIndex = 0;
#define MAX_DIMMER_PWR_INDEX 31
int dimmerPwr [MAX_DIMMER_PWR_INDEX] [2] = {
  // dimmer % ; Dimmer Watt
  {0, 0},
  {5, 32},
  {10, 49},
  {15, 72},
  {20, 102},
  {21, 108},
  {22, 117},
  {23, 121},
  {25, 136},
  {27, 151},
  {30, 170},
  {35, 211},
  {37, 230},
  {40, 245},
  {43, 270},
  {45, 283},
  {47, 300},
  {50, 317},
  {53, 349},
  {55, 353},
  {57, 370},
  {60, 385},
  {63, 411},
  {65, 419},
  {67, 432},
  {70, 440},
  {75, 466},
  {80, 479},
  {85, 491},
  {90, 494},
  {97, 494}
};

// Useful routed power in Wh
float cumulRoutedPowerWh = 0;
// Parasitic network power in Wh reached during the algorithm before stabilization
float cumulNetworkPowerWh = 0;

#define DIMMER_OUTPUT_PIN 27
#define DIMMER_ZERO_CROSS_PIN 39
dimmerLamp dimmer(DIMMER_OUTPUT_PIN, DIMMER_ZERO_CROSS_PIN); 

int dimmerPower = 0;

// Dimmer machine states
#define DIMMER_IDLE 0
#define DIMMER_INCREMENTATION 1
#define DIMMER_STABILIZED 2
#define DIMMER_WAIT 3
#define DIMMER_DECREMENTATION 4

int dimmerState = DIMMER_IDLE;

#define STABILIZED_TIMER 60000
unsigned long dimmerStabilized = 0;

//
// Watchdog variables
//

unsigned long previous = 0;
#define WDT_RESET 120000
#define WDT_RESET_24H 86400000
#define WDT_TIMER 8

//
// Variables used for Linky TIC data:
//

// Linky on UART 2
HardwareSerial linky(2);

#define MAX_BUFFER_LENGTH 20
char buffer[MAX_BUFFER_LENGTH];
int buff_index = 0;

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
    buff_index = 0;
  }
  else if (msgState == WAIT_C1) {
    if ((c == C1) || (c == c1)) {
      msgState = WAIT_C2;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_C2) {
    if ((c == C2) || (c == c2)) {
      msgState = WAIT_C3;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_C3) {
    if ((c == C3) || (c == c3)) {
      msgState = WAIT_C4;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_C4) {
    if ((c == C4) || (c == c4)) {
#ifdef STANDARD_TIC
      msgState = WAIT_C5;
#else
      msgState = WAIT_SEPARATOR1;
#endif
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
 #ifdef STANDARD_TIC 
  else if (msgState == WAIT_C5) {
    if ((c == C5) || (c == c5)) {
      msgState = WAIT_C6;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_C6) {
    if ((c == C6) || (c == c6)) {
      msgState = WAIT_SEPARATOR1;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
#endif
  else if (msgState == WAIT_SEPARATOR1) {
    if (c == FIELD_SEPARATOR) {
      msgState = WAIT_D1;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_D1) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D2;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_D2) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D3;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_D3) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D4;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_D4) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_D5;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_D5) {
    if ((c >= 0x30) && (c <= 0x39)) {
      msgState = WAIT_SEPARATOR2;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_SEPARATOR2) {
    if (c == FIELD_SEPARATOR) {
      msgState = WAIT_CHECKSUM;
      buffer[buff_index] = c;
      buff_index++;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
    }
  }
  else if (msgState == WAIT_CHECKSUM) {
    msgState = WAIT_CR;
    buffer[buff_index] = c;
    buff_index++;
  }
  else if (msgState == WAIT_CR) {
    if (c == CR) {
      if (isChecksumGood()) {
        ret = true;
      }
      msgState = WAIT_LF;
      buff_index = 0;
    }
    else {
      msgState = WAIT_LF;
      buff_index = 0;
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
  
  if (buff_index == 0) {
    // Error: Empty frame
    return false;
  }
  
  checksum = buffer[buff_index - 1];

#ifdef STANDARD_TIC
  for (int i = 0; i<(buff_index - 1); i++) {
    sum += (unsigned int)buffer[i];
  }
#else
  for (int i = 0; i<(buff_index - 2); i++) {
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
 * Replace processElectricalConsumption() with calibrate_processElectricalConsumption() for dimmer calibration
 * This function can be used to initialize 
 */
void calibrate_processElectricalConsumption() {
  String next = "";

  Serial.print("Conso : ");
  Serial.print(getData());
  Serial.print("VA\tDimmer power: ");
  Serial.print(dimmerPower);
  Serial.println("VA\tNext dimmer power: ?");
  while (Serial.available() == 0) {
    next = Serial.readString();
    if (next != "") {
      break;
    }
  }
  dimmer.setState(ON);
  dimmer.setPower(next.toInt());
    
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
  int nextDimmerPwrIndex = 0;

  if (dimmerState == DIMMER_IDLE) {
    if (electricalConsumption > 0) {
      nextDimmerPwrIndex = 0;
      dimmerState = DIMMER_IDLE;
    }
    else {
      nextDimmerPwrIndex = 1;
      dimmerState = DIMMER_INCREMENTATION;
    }
  }
  else if (dimmerState == DIMMER_INCREMENTATION) {
    if (electricalConsumption == 0) {
      nextDimmerPwrIndex = dimmerPwrIndex + 1;
      dimmerState = DIMMER_INCREMENTATION;
    }
    else if (dimmerPwr[dimmerPwrIndex][1] > electricalConsumption) {
      if (dimmerPwrIndex > 0) nextDimmerPwrIndex = dimmerPwrIndex - 1;
      dimmerState = DIMMER_WAIT;
    }
    else {
      nextDimmerPwrIndex = 0;
      dimmerState = DIMMER_IDLE;
    }
  }
  else if (dimmerState == DIMMER_WAIT) {
      nextDimmerPwrIndex = dimmerPwrIndex;
      dimmerState = DIMMER_DECREMENTATION;
  }
  else if (dimmerState == DIMMER_DECREMENTATION) {
    if (electricalConsumption == 0) {
      nextDimmerPwrIndex = dimmerPwrIndex;
      dimmerStabilized = millis();
      dimmerState = DIMMER_STABILIZED;
    }
    else if (dimmerPwr[dimmerPwrIndex][1] > electricalConsumption) {
      if (dimmerPwrIndex > 0) nextDimmerPwrIndex = dimmerPwrIndex - 1;
      dimmerState = DIMMER_WAIT;
    }
    else {
      nextDimmerPwrIndex = 0;
      dimmerState = DIMMER_IDLE;
    }
  }
  else if (dimmerState == DIMMER_STABILIZED) {
    if ((millis() - dimmerStabilized) > STABILIZED_TIMER) {
      nextDimmerPwrIndex = dimmerPwrIndex;
      dimmerState = DIMMER_INCREMENTATION;
    }
    else if (electricalConsumption == 0) {
      nextDimmerPwrIndex = dimmerPwrIndex;
      dimmerState = DIMMER_STABILIZED;
    }
    else if (dimmerPwr[dimmerPwrIndex][1] > electricalConsumption) {
      if (dimmerPwrIndex > 0) nextDimmerPwrIndex = dimmerPwrIndex - 1;
      dimmerState = DIMMER_WAIT;
    }
    else {
      nextDimmerPwrIndex = 0;
      dimmerState = DIMMER_IDLE;
    }
  }
  else {
    // Unknown dimmer state
  }
  
  // Some protection 
  if (nextDimmerPwrIndex >= MAX_DIMMER_PWR_INDEX) {
    nextDimmerPwrIndex = MAX_DIMMER_PWR_INDEX - 1;
  }

  processDimmerPwrIndex(nextDimmerPwrIndex);
}

/*
 * Programation of the dimmer with new power
 */
void processDimmerPwrIndex(int nextDimmerPwrIndex) {
  unsigned long now = millis();
  float pwr;
  
  if (getData() == 0) {
    cumulRoutedPowerWh += dimmerPwr[dimmerPwrIndex][1] * (float)(now - previous) / 3600000; 
  }
  else if (dimmerPwr[dimmerPwrIndex][1] > 0) {
    cumulNetworkPowerWh += getData() * (float)(now - previous) / 3600000;
  }

#ifdef DEBUG
  Serial.print(now - previous);
#endif 
  previous = now;
#ifdef DEBUG
  Serial.print("\tPrevious dimmer ");
  Serial.print(dimmerPwr[dimmerPwrIndex][1]);
  Serial.print("VA\tNext dimmer ");
  Serial.print(dimmerPwr[nextDimmerPwrIndex][1]);
  Serial.print("VA  \tPower consumption ");
  Serial.print(getData());
  Serial.print("VA\tDimmer ");
#endif 
  
  dimmerPwrIndex = nextDimmerPwrIndex;

  if (dimmerPwr[nextDimmerPwrIndex][1] == 0) {
    dimmer.setState(OFF);
  }
  else {
    dimmer.setState(ON);
  }
  
  if (dimmer.getPower() != dimmerPwr[nextDimmerPwrIndex][0]) {
    dimmer.setPower(dimmerPwr[nextDimmerPwrIndex][0]);
  }

#ifdef DEBUG
  Serial.print(dimmer.getPower());
  Serial.print("%\tRouted ");
  Serial.print(cumulRoutedPowerWh,2);
  Serial.print("Wh\tNetwork ");
  Serial.print(cumulNetworkPowerWh,2);
  Serial.println("Wh");
#endif
}

// Linky initialization
void setupLinky() {
  // Initialize the Linky serial link.
  linky.begin(TIC_BAUD_RATE,SERIAL_8N1, 16, 17);
}

// Watchdog initialization
void setupWatchdog() {
  // Initialize the watchdog to 8s.
  esp_task_wdt_init(WDT_TIMER, true);
  esp_task_wdt_add(NULL);
}

// Serial initialization
void setupSerial() {
  Serial.begin(115200); // connect serial 
}

// Dimmer initialization
void setupDimmer() {
  dimmer.begin(NORMAL_MODE, OFF);
}

// Reset the board
void resetBoard() {
  unsigned long t = millis();
  int i;

  // Reset the watchdog
  esp_task_wdt_reset();
  
  // Reset the board if no new message was received during WDT_RESET or every 24 hours...
  if (((t - previous) >= WDT_RESET) || (t > WDT_RESET_24H)) {
    // Let the watchdog reset the board
    while(true) {
      i++;
    }
  }
}

void setup() {
  setupLinky();
  setupWatchdog();
  setupSerial();
  setupDimmer();
  Serial.println("End of setup...");
}

void loop() {
  char inChar;
  int val;

  // Eventually reset the board if no message received from Linky during WDT_RESET ms
  resetBoard();
  
  while (linky.available()) {
    // read the new char
    val = linky.read();
    if (val != -1) {
      // Reset watchdog and process instantaneous electical power when received from Linky   
      if (isNewMsgInBuffer((char)val & 0x7F)) {
        processElectricalConsumption();
      }
    }
  } 
}
