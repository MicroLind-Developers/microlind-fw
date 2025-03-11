// ADDRESS LOGIC TESTER

// GAL inputs
int GAL_A0 = 22;
int GAL_A1 = 23;
int GAL_A2 = 24;
int GAL_A3 = 25;
int GAL_A4 = 26;
int GAL_A5 = 27;
int GAL_A6 = 28;
int GAL_A7 = 29;
int GAL_A8 = 30;
int GAL_A9 = 31;
int GAL_IOEN = 32;

// GAL outputs
int GAL_MEMEN = 34;
int GAL_IRQEN = 35;
int GAL_POWEN = 36;
int GAL_SEREN = 37;
int GAL_PAREN = 38;
int GAL_SNDLEN = 40;
int GAL_SNDREN = 41;
int GAL_VDCEN = 42;
int GAL_EXPEN = 43;

int address_port[10] = {GAL_A0, GAL_A1, GAL_A2, GAL_A3, GAL_A4, GAL_A5, GAL_A6, GAL_A7, GAL_A8, GAL_A9};
int enable_port[9] = {GAL_EXPEN, GAL_VDCEN, GAL_SNDREN, GAL_SNDLEN, GAL_PAREN, GAL_SEREN, GAL_POWEN, GAL_IRQEN, GAL_MEMEN};

void set_address(int address) {
  for (int i = 0; i < 10; ++i) {
    if ((address >> i) && 0x0001) {
      digitalWrite(address_port[i], HIGH);
    } else {
      digitalWrite(address_port[i], LOW);
    }
  }
}

int get_result() {
  int result = 0;
  for (int i = 0; i < 9; ++i) {
    if (digitalRead(enable_port[i])) {
      result = result + (1 << i);
    }
  }
  return result;
}

void setup() {
    pinMode(GAL_A0, OUTPUT);
    pinMode(GAL_A1, OUTPUT);
    pinMode(GAL_A2, OUTPUT);
    pinMode(GAL_A3, OUTPUT);
    pinMode(GAL_A4, OUTPUT);
    pinMode(GAL_A5, OUTPUT);
    pinMode(GAL_A6, OUTPUT);
    pinMode(GAL_A7, OUTPUT);
    pinMode(GAL_A8, OUTPUT);
    pinMode(GAL_A9, OUTPUT);
    pinMode(GAL_IOEN, OUTPUT);

    pinMode(GAL_MEMEN, INPUT_PULLUP);
    pinMode(GAL_IRQEN, INPUT_PULLUP);
    pinMode(GAL_POWEN, INPUT_PULLUP);
    pinMode(GAL_SEREN, INPUT_PULLUP);
    pinMode(GAL_PAREN, INPUT_PULLUP);
    pinMode(GAL_SNDLEN, INPUT_PULLUP);
    pinMode(GAL_SNDREN, INPUT_PULLUP);
    pinMode(GAL_VDCEN, INPUT_PULLUP);
    pinMode(GAL_EXPEN, INPUT_PULLUP);

  Serial.begin(115200);
}

void loop() {
  Serial.println("Arduino MEGA tester");

    Serial.println("microLind GAL tester");
    Serial.println("for Address chip\n\n");

    int en_val=0;
    for(int i = 0; i < 0x3FF; ++i){
    digitalWrite(GAL_IOEN, HIGH);
    Serial.print("Testing address 0x");
    Serial.print(i, HEX);
    Serial.print("... ");
    set_address(i);
    //en_val = get_result();
    //if(en_val != 0) {
    //  Serial.print("ERROR (enable set before IOEN (");
    //  Serial.print(en_val, BIN);
    //  Serial.println("))");
    //  continue;
    //}
    digitalWrite(GAL_IOEN, LOW);
    en_val = get_result();
    for(int n = 8; n>= 0; n--){
      Serial.print(bitRead(en_val,n));
    }
    Serial.println("");
    }
}
