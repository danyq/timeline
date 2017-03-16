
#define M1_PUL_PIN 11  // OC1A
#define M1_DIR_PIN 10
#define M1_VCC_PIN  9
#define M1_EN_PIN   8

#define M2_PUL_PIN 5  // OC3A
#define M2_DIR_PIN 4
#define M2_VCC_PIN 3
#define M2_EN_PIN  2

#define HX1_VCC_PIN 24
#define HX1_SCK_PIN 26
#define HX1_DT_PIN  28
#define HX1_GND_PIN 30

#define HX2_VCC_PIN 44
#define HX2_SCK_PIN 46
#define HX2_DT_PIN  48
#define HX2_GND_PIN 50

// SYS-STATE BIT POSITIONS
#define M1_EN       0
#define M2_EN       1
#define M1_FWD      2
#define M2_FWD      3
#define TIMING_FAIL 4

// TIMING
// ~50ms (needs to be tested)
#define MOTOR_RESET_PERIOD 65535
#define N_READ_WAITS   100000






class Hx711 {
 public:
  Hx711(uint8_t pin_din, uint8_t pin_slk);
  virtual ~Hx711();
  void getBytes(byte* data);
  char isReady();        
 private:
  const uint8_t _pin_dout;
  const uint8_t _pin_slk;
};

Hx711::Hx711(uint8_t pin_dout, uint8_t pin_slk) :
_pin_dout(pin_dout), _pin_slk(pin_slk) {
  pinMode(_pin_slk, OUTPUT);
  pinMode(_pin_dout, INPUT);
  
  digitalWrite(_pin_slk, HIGH);
  delayMicroseconds(100);
  digitalWrite(_pin_slk, LOW);
}

Hx711::~Hx711() { }

char Hx711::isReady() {
  return !digitalRead(_pin_dout);
}

void Hx711::getBytes(byte* data) {
  // Caller invariant: isReady() should be true before this is called.
  // while (digitalRead(_pin_dout));
  for (byte j = 0; j < 3; j++) {
    for (byte i = 0; i < 8; i++) {
      digitalWrite(_pin_slk, HIGH);
      bitWrite(data[2 - j], 7 - i, digitalRead(_pin_dout));
      digitalWrite(_pin_slk, LOW);
    }
  }
  digitalWrite(_pin_slk, HIGH);
  digitalWrite(_pin_slk, LOW);
}





struct Motor {
  unsigned int togglePeriod;
  long nstepsFwd;
  long nstepsBkwd;
  long *nsteps;
};


/* Initialize the togglePeriod to the minimum speed for safety... */
Motor motor1 = {.togglePeriod = MOTOR_RESET_PERIOD};
Motor motor2 = {.togglePeriod = MOTOR_RESET_PERIOD};

Hx711 scale1(HX1_DT_PIN, HX1_SCK_PIN);
Hx711 scale2(HX2_DT_PIN, HX2_SCK_PIN);

char sys_state = 0;
char nxt_sys_state = 0;

ISR(TIMER1_COMPA_vect) {
  // step pin (OC1A) is toggled on compare match
  (*motor1.nsteps) += 1;  // we've taken a step
  OCR1A = motor1.togglePeriod;
}

ISR(TIMER3_COMPA_vect) {
  // step pin (OC1A) is toggled on compare match
  (*motor2.nsteps) += 1;  // we've taken a step
  OCR3A = motor2.togglePeriod;
}

void enable_timer1() {
  TCCR1B |= _BV(CS10);  // enable clock
  sys_state |= (1 << M1_EN);
}
void enable_timer3() {
  TCCR3B |= _BV(CS30);  // enable clock
  sys_state |= (1 << M2_EN);
}
void disable_timer1() {
  TCCR1B = _BV(WGM12);  // clear all but CTC mode
  sys_state &= ~(1 << M1_EN);  
}
void disable_timer3() {
  TCCR3B = _BV(WGM32);  // clear all but CTC mode
  sys_state &= ~(1 << M2_EN);  
}

unsigned char inByte;
unsigned char lastForce1[3];
unsigned char lastForce2[3];

char f1ready;
char f2ready;

unsigned int nxt_p1;
unsigned int nxt_p2;

long rem_read_waits;

void compress_steps(Motor m) {
  // To avoid overflows, subtract the nstepsFwd and nstepsBkwd
  if(m.nstepsFwd > m.nstepsBkwd) {
    m.nstepsFwd -= m.nstepsBkwd;
    m.nstepsBkwd = 0;
  }
  else {
    m.nstepsBkwd -= m.nstepsFwd;
    m.nstepsFwd = 0;
  }
}

void motor1_fwd() {
  cli();
      
  digitalWrite(M1_DIR_PIN, HIGH);
  motor1.nsteps = &motor1.nstepsFwd;
  compress_steps(motor1);

  sei();

  sys_state |= (1 << M1_FWD);
}
void motor1_bkwd() {
  cli();
      
  digitalWrite(M1_DIR_PIN, LOW);
  motor1.nsteps = &motor1.nstepsBkwd;
  compress_steps(motor1);

  sei();

  sys_state &= ~(1 << M1_FWD);
}

void motor2_fwd() {
  cli();
      
  digitalWrite(M2_DIR_PIN, HIGH);
  motor2.nsteps = &motor2.nstepsFwd;
  compress_steps(motor2);

  sei();

  sys_state |= (1 << M2_FWD);
}
void motor2_bkwd() {
  cli();
      
  digitalWrite(M2_DIR_PIN, LOW);
  motor2.nsteps = &motor2.nstepsBkwd;
  compress_steps(motor2);

  sei();

  sys_state &= ~(1 << M2_FWD);
}

void reset() {
  // 1. Disable timers
  disable_timer1();
  disable_timer3();        

  // 2. Reset periods
  motor1.togglePeriod = MOTOR_RESET_PERIOD;
  motor2.togglePeriod = MOTOR_RESET_PERIOD;

  // 3. Reset directions
  motor1_bkwd();
  motor2_bkwd();  
}


char timecheck_serial() {
  while(!Serial.available()) {
    if(--rem_read_waits == 0) {
      return 0;
    }
  }
  return 1;
}

char eject_state() {
  // Eject all state - returns 1 if successful

  Serial.println("eject");

  Serial.write(sys_state);

  Serial.write(lastForce1[0]);
  Serial.write(lastForce1[1]);
  Serial.write(lastForce1[2]);

  Serial.write(lastForce2[0]);
  Serial.write(lastForce2[1]);
  Serial.write(lastForce2[2]);

  long steps1 = motor1.nstepsFwd - motor1.nstepsBkwd;
  long steps2 = motor2.nstepsFwd - motor2.nstepsBkwd;
  Serial.println(steps1);
  Serial.println(steps2);    

  Serial.println(motor1.togglePeriod);
  Serial.println(motor2.togglePeriod);    

  return 1;
}

char load_state() {
  Serial.println("load");

  if(Serial.parseInt() != 42) {
    // parity bit failed
    return 0;
  }

  if(!timecheck_serial()) return 0;
  nxt_sys_state = Serial.parseInt();

  if(!timecheck_serial()) { return 0; }  
  nxt_p1 = Serial.parseInt();
  if(!timecheck_serial()) { return 0; }  
  nxt_p2 = Serial.parseInt();
  
  return 1;
}

char apply_state() {
  // Enable/disable motors
  char bit_test = 1 << M1_EN;
      
  if((nxt_sys_state & bit_test) != (sys_state & bit_test)) {
    // Motor 1 has changed
    if(nxt_sys_state & bit_test)
      enable_timer1();
    else
      disable_timer1();
  }

  bit_test = 1 << M2_EN;
  if((nxt_sys_state & bit_test) != (sys_state & bit_test)) {
    if(nxt_sys_state & bit_test)
      enable_timer3();
    else
      disable_timer3();
  }

  bit_test = 1 << M1_FWD;
  if((nxt_sys_state & bit_test) != (sys_state & bit_test)) {
    // Motor 1 direction has changed
    if(nxt_sys_state & bit_test)
      motor1_fwd();
    else
      motor1_bkwd();
  }
  bit_test = 1 << M2_FWD;
  if((nxt_sys_state & bit_test) != (sys_state & bit_test)) {
    if(nxt_sys_state & bit_test)
      motor2_fwd();
    else
      motor2_bkwd();
  }

  // Set period
  // (need to disable interrupts here?)
  motor1.togglePeriod = nxt_p1;
  motor2.togglePeriod = nxt_p2;

  return 1;
}

void setup() {
  pinMode(M1_PUL_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M1_VCC_PIN, OUTPUT);
  pinMode(M1_EN_PIN, OUTPUT);

  pinMode(M2_PUL_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M2_VCC_PIN, OUTPUT);
  pinMode(M2_EN_PIN, OUTPUT);

  pinMode(HX1_VCC_PIN, OUTPUT);
  pinMode(HX2_VCC_PIN, OUTPUT);
  pinMode(HX1_GND_PIN, OUTPUT);
  pinMode(HX2_GND_PIN, OUTPUT);

  digitalWrite(M1_PUL_PIN, HIGH);
  digitalWrite(M1_DIR_PIN, HIGH);
  digitalWrite(M1_VCC_PIN, HIGH);
  digitalWrite(M1_EN_PIN, HIGH);

  digitalWrite(M2_PUL_PIN, HIGH);
  digitalWrite(M2_DIR_PIN, HIGH);
  digitalWrite(M2_VCC_PIN, HIGH);
  digitalWrite(M2_EN_PIN, HIGH);

  digitalWrite(HX1_VCC_PIN, HIGH);
  digitalWrite(HX2_VCC_PIN, HIGH);
  digitalWrite(HX1_GND_PIN, LOW);
  digitalWrite(HX2_GND_PIN, LOW);

  // initialize nsteps to be backwards (DIR_POS_PIN, LOW)
  motor1.nsteps = &motor1.nstepsBkwd;
  motor2.nsteps = &motor2.nstepsBkwd;

  Serial.begin(115200);

  cli();  // disable interrupts

  // set up timer1 for motor1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = motor1.togglePeriod;
  TCCR1A |= _BV(COM1A0);  // toggle OCR1A on compare match
  TCCR1B |= _BV(WGM12);   // CTC mode
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt

  // set up timer3 for motor2
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = motor2.togglePeriod;
  TCCR3A |= _BV(COM3A0);  // toggle OC3A on compare match
  TCCR3B |= _BV(WGM32);   // CTC mode
  TIMSK3 |= (1 << OCIE3A);      // enable compare interrupt

  sei();  // enable interrupts

  reset();
}

void loop() {
  if(scale1.isReady()) {
    scale1.getBytes(lastForce1);
    f1ready = 1;
  }
  if(scale2.isReady()) {
    scale2.getBytes(lastForce2);
    f2ready = 1;
  }
  if(f1ready && f2ready) {
    rem_read_waits = N_READ_WAITS;

    if(eject_state()) {
      if(load_state()) {
        if(apply_state()) {
          // Success!

          // Un-set timing fail
          sys_state &= ~_BV(TIMING_FAIL);

          f1ready = 0;
          f2ready = 0;

          return;
        }
      }
    }

    // Something went wrong.
    // Flush (?)
    reset();
    while (Serial.available())
      Serial.read();
    sys_state |= _BV(TIMING_FAIL);
  }
}
