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

int32_t count = 0;

Adafruit_MCP23X17 mcp0;
Adafruit_MCP23X17 mcp1;

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

uint16_t readDataBus(void) {
  uint8_t payload = 0;

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
  reset();

  Serial.begin(115200);

  if (!mcp0.begin_I2C(0x20)) {
    Serial.println("error in i2c    MCP0");
    while (true)
      ;
  }

  if (!mcp1.begin_I2C(0x21)) {
    Serial.println("error in i2c   MCP1");
    while (true)
      ;
  }

  Wire.setClock(400000); // 400 kHz   ->     18382 us to digitalRead 100

  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);
  pinMode(IOM, INPUT);

  // Pins 1 to 7
  for (int i = 8; i <= 14; i++) {
    mcp0.pinMode(i, INPUT);
    mcp1.pinMode(i, INPUT);
  }

  mcp0.pinMode(0, INPUT); // Pin 21
  mcp1.pinMode(0, INPUT); // Pin 21

  // changeMode(OUTPUT);
  /*
    while (true) {
    mcp0.pinMode(1, OUTPUT);
      Serial.println("Writing HIGH");
      mcp0.digitalWrite(1, HIGH);
      delay(5000);

      mcp0.pinMode(1, INPUT);
      Serial.println("Waiting for LOW");
      while (mcp0.digitalRead(1) == HIGH);
      Serial.println("Detected LOW");
      delay(5000);
    } */
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

  uint16_t a = readAddressBus();

  changeDataMode(OUTPUT);

  // writeDataBus(0x00);

  if (a == 0x00) {
    writeDataBus(0xC3); // Unconditional JMP

  } else if (a == 0x01) {
    writeDataBus(0x00);

  } else if (a == 0x02) {
    writeDataBus(0x10);

  } else {
    writeDataBus(0x00);
  }


  Serial.printf("count: 0x%.4X\t address: 0x%.4X         Diff: %d\n", count, a,
                a - count);

  digitalWrite(READY, HIGH); //  Indicate memory is ready to 8085

  while (digitalRead(IOM) !=
         LOW) //    8085 indicates memory operation has finished
    ;

  digitalWrite(READY, LOW);

  delay(100);
  count++;
}
