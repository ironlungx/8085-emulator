#include <Adafruit_MCP23X17.h>
#include <Arduino.h>

/*
 * Components Used:
 *    - MCP23015    (Needs 10k pullup resistor between SDA & SCL)
 * https://www.makerguides.com/using-gpio-expander-mcp23017-with-arduino/
 *    - SN74LS373
 * https://www.alldatasheet.com/datasheet-pdf/download/51081/FAIRCHILD/74LS373.html
 *    - 8085
 *
 *
 *
 * */

#define READY 25
#define IOM 2
#define RESET 33

#define RD 32
#define WR 35

int32_t count = 0;

Adafruit_MCP23X17 mcp0;
Adafruit_MCP23X17 mcp1;

#define MEM_LEN 4096
byte memory8085[MEM_LEN] = {
  0x26, 0x01, 
  0x2E, 0x00,
  0x36, 0xAA,
  0x2C, 
  0xC3, 0x04, 0x00
};

byte readMemeory(uint16_t address) {
  if (address > MEM_LEN) {
    Serial.printf("Invalid readAddress 0x%.4X\n", address);
    return 0;
  } else {
    return memory8085[address];
  }
}

void writeMemory(uint16_t address, byte data) {
  if (address > MEM_LEN) {
    Serial.printf("Invalid writeAddress 0x%0.4X\n", address);
  } else {
    Serial.printf("Writing data: 0x%.2X\t at 0x%.4X", data, address);
    memory8085[address] = data;
  }
}

void writeDataBus(byte payload) {
  mcp1.digitalWrite(1, payload & 1);
  payload >>= 1;

  mcp1.digitalWrite(2, payload & 1);
  payload >>= 1;

  mcp1.digitalWrite(3, payload & 1);
  payload >>= 1;

  mcp1.digitalWrite(4, payload & 1);
  payload >>= 1;

  mcp0.digitalWrite(1, payload & 1);
  payload >>= 1;

  mcp0.digitalWrite(2, payload & 1);
  payload >>= 1;

  mcp0.digitalWrite(3, payload & 1);
  payload >>= 1;

  mcp0.digitalWrite(4, payload & 1);
  payload >>= 1;
}

uint16_t readAddressBus(bool debug = false) {
  uint16_t address = 0x00;

  byte valB0 = 0x7F & mcp0.readGPIOB();
  byte gpa0 = mcp0.digitalRead(0);

  byte valB1 = 0x7F & mcp1.readGPIOB();
  byte gpa1 = mcp1.digitalRead(0);

  address = uint16_t(valB0);
  address |= (uint16_t(gpa0) << 7);
  address |= (uint16_t(valB1) << 8);
  address |= (uint16_t(gpa1) << 15);

  if (debug)
    Serial.printf("valB0: 0x%.2x gpa0: %d\t valB1: 0x%.2x gpa1 %d          "
                  "address: 0x%.4X\n",
                  valB0, gpa0, valB1, gpa1, address);

  return address;
}

byte readDataBus(void) {
  byte payload = 0;

  payload |= (mcp1.digitalRead(1) << 0);
  payload |= (mcp1.digitalRead(2) << 1);
  payload |= (mcp1.digitalRead(3) << 2);
  payload |= (mcp1.digitalRead(4) << 3);

  payload |= (mcp0.digitalRead(1) << 4);
  payload |= (mcp0.digitalRead(2) << 5);
  payload |= (mcp0.digitalRead(3) << 6);
  payload |= (mcp0.digitalRead(4) << 7);

  return payload;
}

void reset() {
  digitalWrite(RESET, LOW);
  delay(1);
  digitalWrite(RESET, HIGH);
}

void changeDataMode(int mode) {
  for (int i = 1; i <= 4; i++) {
    mcp0.pinMode(i, mode);
    mcp1.pinMode(i, mode);
  }

  // delay(200);
}

void setup() {
  pinMode(READY, OUTPUT);
  digitalWrite(READY, LOW);

  Serial.begin(115200);

  if (!mcp1.begin_I2C(0x21)) {
    Serial.println("error in i2c   MCP1");
    while (true)
      ;
  }
  if (!mcp0.begin_I2C(0x20)) {
    Serial.println("error in i2c    MCP0");
    while (true)
      ;
  }

  Wire.setClock(400000); // 400 kHz   ->     18382 us to digitalRead 100

  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  pinMode(IOM, INPUT);

  // Magic number, no one knows how we got them
  pinMode(RD, INPUT);
  pinMode(WR, INPUT);

  // Pins 1 to 7
  for (int i = 8; i <= 14; i++) {
    mcp0.pinMode(i, INPUT);
    mcp1.pinMode(i, INPUT);
  }

  mcp0.pinMode(0, INPUT); // Pin 21
  mcp1.pinMode(0, INPUT); // Pin 21

  changeDataMode(OUTPUT);
  writeDataBus(0x00);
}

bool firstRun = true;
void loop() {
  if (count == 2 && firstRun) {
    reset();
    count = 0;
    firstRun = false;
  }

  changeDataMode(INPUT);

  uint16_t address = readAddressBus();

  changeDataMode(OUTPUT);

  if (digitalRead(RD) == LOW) { // Microprocessor is in read operation
    byte b = readMemeory(address);

    changeDataMode(OUTPUT);
    writeDataBus(b);
  }

  if (digitalRead(WR) == LOW) {
    byte b = readDataBus();
    writeMemory(address, b);
  }

  // Serial.printf("RD: %s\t WR: %s\n", digitalRead(RD) ? "HIGH" : "LOW", digitalRead(WR) ? "HIGH" : "LOW");

  // if (address == 0x00) {
  //   writeDataBus(0xC3); // Unconditional JMP
  //
  // } else if (address == 0x01) {
  //   writeDataBus(0x00);
  //
  // } else if (address == 0x02) {
  //   writeDataBus(0x10);
  //
  // } else {
  //   writeDataBus(0x00);
  // }

  Serial.printf("\ncount: 0x%.4X\t address: 0x%.4X\t", count, address);

  digitalWrite(READY, HIGH); //  Indicate memory is ready to 8085

  while (digitalRead(IOM) !=
         LOW) //    8085 indicates memory operation has finished
    ;

  digitalWrite(READY, LOW);

  delay(1000);
  count++;
}
