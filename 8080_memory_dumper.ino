#define ADDR_DISABLE 6
#define OUTPUT_DISABLE 7
#define MEM_W 5
#define CC_DISABLE 4

// DOut - Port C
// Add Low - Port A
// Add High - Port B
uint8_t code[] = {0x3E,0x00,0x06,0x01,0x2E,0xFF,0x4F,0x80,0xDA,0x00,0x00,0xAD,0xD3,0xFF,0xAD,0x41,0x67,0x11,0xFF,0xFF,0x1B,0x7A,0xB3,0xC2,0x14,0x00,0x7C,0xC3,0x06,0x00};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DDRC = 0;
  DDRA = 0;
  DDRB = 0;
  pinMode(ADDR_DISABLE, OUTPUT);
  pinMode(OUTPUT_DISABLE, OUTPUT);
  pinMode(MEM_W,OUTPUT);
  pinMode(CC_DISABLE, OUTPUT);

  digitalWrite(OUTPUT_DISABLE, LOW);
  digitalWrite(ADDR_DISABLE,LOW);
  digitalWrite(CC_DISABLE, LOW);
  delay(1);
  DDRC = 0xFF;
  DDRA = 0xFF;
  DDRB = 0xFF;
  delay(1);
  uint16_t addr = 0;
  for(addr; addr < 0x7FFF; addr++) {
    PORTA = addr & 0xFF;
    PORTB = (addr >> 8) & 0xFF;
//    delay(1);
//    Serial.print("Writing Addr ");
//    Serial.println(addr);
    PORTC = addr & 0xFF;
//    delay(1);
    digitalWrite(MEM_W, LOW);
//    delay(1);
    digitalWrite(MEM_W, HIGH);
//    delay(1);
  }
  delay(1);
  DDRA = 0;
  DDRB = 0;
  DDRC = 0;
  pinMode(ADDR_DISABLE, INPUT);
  pinMode(OUTPUT_DISABLE, INPUT);
  pinMode(MEM_W, INPUT);
  pinMode(CC_DISABLE, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
   
}
