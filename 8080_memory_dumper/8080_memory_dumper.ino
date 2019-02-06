#define ADDR_DISABLE 6
#define OUTPUT_DISABLE 7
#define MEM_W 5
#define CC_DISABLE 4

#define MEM_WRITE_DELAY 1
#define MEM_BUFFER_SIZE 256

// DOut - Port C
// Add Low - Port A
// Add High - Port B

uint8_t *data_buffer = nullptr;
bool has_bus_control = false;

void acquire_bus() {
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
  has_bus_control = true;
}

void release_bus() {
  has_bus_control = false;
  
  DDRA = 0;
  DDRB = 0;
  DDRC = 0;
  pinMode(ADDR_DISABLE, INPUT);
  pinMode(OUTPUT_DISABLE, INPUT);
  pinMode(MEM_W, INPUT);
  pinMode(CC_DISABLE, INPUT);

  delay(2);
}

void write_mem(uint16_t start_addr, uint8_t buf[], uintptr_t offset, uintptr_t size) {
  if (!has_bus_control) {
    Serial.println("Tried to write memory without acquiring bus!");
    return;
  }

  for(uintptr_t i = 0; i < size; i++) {
    uint16_t addr = (uint16_t)(start_addr + i);
    
    PORTA = addr & 0xFF;
    PORTB = (addr >> 8) & 0xFF;
#ifdef MEM_WRITE_DELAY
    delay(1);
#endif
    
    PORTC = buf[offset + i];

#ifdef MEM_WRITE_DELAY
    delay(1);
#endif

    digitalWrite(MEM_W, LOW);

#ifdef MEM_WRITE_DELAY
    delay(1);
#endif

    digitalWrite(MEM_W, HIGH);

#ifdef MEM_WRITE_DELAY
    delay(1);
#endif
  } 
}

void halt() {
  while (true) {
    delay(1000);    
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  data_buffer = (uint8_t *)malloc(MEM_BUFFER_SIZE);

  if (data_buffer == nullptr) {
    Serial.println("ERROR Failed to allocate memory buffer");
    halt();
  }

  Serial.println("====================================");
  Serial.println("|    *-* 8080_memory_dumper *-*    |");
  Serial.println("====================================");

#ifdef MEM_WRITE_DELAY
  Serial.println("| Using memory write delays: true  |");
#else
  Serial.println("| Using memory write delays: false |");
#endif
  
  Serial.println("====================================");
  
}

int read_robust() {
  unsigned long end_time = millis() + 10000;
  int cur_byte;
  do {
    cur_byte = Serial.read();
  } while(cur_byte == -1 && millis() < end_time);

  return cur_byte;
}

int read_hex_byte() {

  int high_digit = read_robust();
  if (high_digit == -1) {
    return -1;
  }

  int low_digit = read_robust();
  if (low_digit == -1) {
    return -1;
  }

  int value = 0;
  
  if (high_digit >= '0' && high_digit <= '9') {
    value = high_digit - '0';
  } else if (high_digit >= 'a' && high_digit <= 'f') {
    value = high_digit - 'a' + 10;
  } else if (high_digit >= 'A' && high_digit <= 'F') {
    value = high_digit - 'A' + 10;
  } else {
    return -2;
  }

  value = value << 4;

  if (low_digit >= '0' && low_digit <= '9') {
    value += low_digit - '0';
  } else if (low_digit >= 'a' && low_digit <= 'f') {
    value += low_digit - 'a' + 10;
  } else if (low_digit >= 'A' && low_digit <= 'Fx') {
    value += low_digit - 'A' + 10;
  } else {
    return -2;
  }

  return value;
}

void loop() {
  delay(100);

  if (Serial.available() > 0) {
    Serial.println("WARN Clearing unknown data from serial buffer.");
  }
  
  while(Serial.available() > 0) {
    Serial.read();
  }
  
  Serial.println("INFO Ready");

  // start of message.
  int cur_byte;
  do {
    cur_byte = Serial.read();
  } while (cur_byte == -1);

  if (cur_byte != ':') {
    Serial.println("ERROR Malformed start of message!, halting");
    halt(); 
  }

  int byte_count = read_hex_byte();
  if (byte_count == -1) {
    Serial.println("ERROR Didn't receive byte count, resetting");
    return;
  }
  if (byte_count == -2) {
    Serial.println("ERROR Received invalid hex for byte count, resetting");
    return;
  }

  int start_addr_high = read_hex_byte();
  if (start_addr_high == -1) {
    Serial.println("ERROR Didn't receive high address byte, resetting");
    return;
  }
  if (start_addr_high == -2) {
    Serial.println("ERROR Received invalid hex for high address byte, resetting");
    return;
  }

  int start_addr_low = read_hex_byte();
  if (start_addr_low == -1) {
    Serial.println("ERROR Didn't receive high address byte, resetting");
    return;
  }
  if (start_addr_low == -2) {
    Serial.println("ERROR Received invalid hex for high address byte, resetting");
    return;
  }

  uint16_t start_addr = ((start_addr_high & 0xFF) << 8) + (start_addr_low & 0xFF);

  int record_type = read_hex_byte();
  if (record_type == -1) {
    Serial.println("ERROR Didn't receive record type, resetting");
    return;
  }
  if (record_type == -2) {
    Serial.println("ERROR Received invalid hex for record type, resetting");
    return;
  }

  if (record_type != 0) {
    Serial.println("ERROR Only data record type is supported, resetting");
    return;
  }

  uint8_t checksum_total = 0;

  for (int i = 0; i < byte_count; i++) {
    int cur_byte = read_hex_byte();
     if (cur_byte == -1) {
      Serial.println("ERROR Didn't receive data byte, resetting");
      return;
    }
    if (cur_byte == -2) {
      Serial.println("ERROR Received invalid hex for data byte, resetting");
      return;
    }

    data_buffer[i] = (uint8_t)cur_byte;
    checksum_total += (uint8_t)cur_byte;
  }

  int checksum_byte = read_hex_byte();
  if (checksum_byte == -1) {
    Serial.println("ERROR Didn't receive checksum byte, resetting");
    return;
  }
  if (checksum_byte == -2) {
    Serial.println("ERROR Received invalid hex for checksum byte, resetting");
    return;
  }

  checksum_total += checksum_byte;

  if (checksum_total != 0) {
    Serial.print("ERROR Checksum invalid ");
    uint8_t checksum = checksum_total - checksum_byte;
    Serial.print(checksum, HEX);
    Serial.print(" should be ");
    Serial.print((uint8_t)(256 - checksum), HEX);
    Serial.println(", resetting");
    return;
  }

  Serial.println("INFO Writing memory");

  acquire_bus();
  write_mem(start_addr, data_buffer, 0, byte_count);
  release_bus();
  
  Serial.println("INFO Done writing.");
   
}
