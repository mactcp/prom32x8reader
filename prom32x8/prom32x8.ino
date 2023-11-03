#define _OE          2  // PD2
#define XA0         15  // PC1
#define XA1         16  // PC2
#define XA2         17  // PC3
#define XA3         18  // PC4
#define XA4         19  // PC5

#define Q0           3  // PD3
#define Q1           4  // PD4
#define Q2           5  // PD5
#define Q3           6  // PD6
#define Q4           7  // PD7
#define Q5           8  // PB0
#define Q6           9  // PB1
#define Q7          14  // PC0

#define R_LED       13  // PB3
#define G_LED       12   // PB4

#define BUS_SIZE     5

uint8_t portCMask;
uint8_t portCBits[1 << BUS_SIZE];
//uint8_t portDMask;
//uint8_t portDBits[1 << BUS_SIZE];

const uint8_t a_bus[BUS_SIZE] = {
  XA0, XA1, XA2, XA3, XA4
};
const uint8_t d_bus[8] = {
  Q0, Q1, Q2, Q3, Q4, Q5, Q6, Q7
};

void setupBus(void) {
  portCMask = 0xFF;
  //portDMask = 0xFF;
  //const uint8_t portCX = 3;//digitalPinToPort(PC4);
  //const uint8_t portDX = digitalPinToPort(PD2);
  //Serial.println(portCX, HEX);
  //Serial.println(portDX, HEX);
  unsigned i, j;
  for (i = 0; i < BUS_SIZE; i++) {
    const unsigned int pin = a_bus[i];
    const uint8_t bit = digitalPinToBitMask(pin);
    //const uint8_t port = digitalPinToPort(pin);
    //if (port == portCX) {
      portCMask &= ~bit;
    //} else if (port == portDX) {
      //portDMask &= ~bit;
    /*} else {
      Serial.print(i);
      Serial.print(" ");
      Serial.print(port, HEX);
      Serial.println(" failed port match (mask)");
    }*/
  }
  //Serial.println(portCMask, HEX);
  //Serial.println(portDMask, HEX);
  for (j = 0; j < (1<<BUS_SIZE); j++) {
    uint8_t portCValue = 0;
    //uint8_t portDValue = 0;
    for (i = 0; i < BUS_SIZE; i++) {
      if (j & (1 << i)) {
        const unsigned int pin = a_bus[i];
        const uint8_t bit = digitalPinToBitMask(pin);
        //const uint8_t port = digitalPinToPort(pin);
        //if (port == portCX) {
          portCValue |= bit;
        /*} else if (port == portDX) {
          portDValue |= bit;
        } else {
          Serial.print(i);
          Serial.println(" failed port match (value)");
        }*/
      }
    }
    portCBits[j] = portCValue;
    //portDBits[j] = portDValue;
    // Serial.print(j, HEX);
    // Serial.print(" ");
    // Serial.print(portCValue, HEX);
    // Serial.print(" ");
    // Serial.println(portDValue, HEX);
  }
}

inline void setBus(unsigned int a) {
	volatile uint8_t *out;
	out = portOutputRegister(3);
  *out = (*out & portCMask) | portCBits[a];
  //out = portOutputRegister(4);
  //*out = (*out & portDMask) | portDBits[a];
}

void setup() {
  unsigned i;

  Serial.begin(115200);
  while (!Serial)
    ; /* wait */

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println();
  Serial.println("PROM TESTER 32x8");

  digitalWrite(_OE, HIGH);
 
  for (i = 0; i < 8; i++) {
    pinMode(d_bus[i], INPUT);
  }
  for (i = 0; i < BUS_SIZE; i++) {
    pinMode(a_bus[i], OUTPUT);
  }

  setupBus();

  noInterrupts();
  pinMode(_OE, OUTPUT);
  setBus(0);
  digitalWrite(_OE, LOW);
  interrupts();
  digitalWrite(LED_BUILTIN, LOW);
}

uint8_t readByte(void) {
  unsigned i;
  uint8_t result = 0;

  for (i = 0;i < 8;i++) {
    if (digitalRead(d_bus[i]) != 0) {
      result |= 1 << i;
    }
  }
  return result;
}

void dumpPROM(void) {
  unsigned address;

  noInterrupts();
  for (address = 0;address < 32; address++) {
    digitalWrite(LED_BUILTIN, HIGH);
    setBus(address);
    uint8_t byte = readByte();
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print(byte >> 4, HEX);
    Serial.println(byte & 0x0F, HEX);
  }
  interrupts();
}

void loop() {
  dumpPROM();
  while (1)
    ;
}
