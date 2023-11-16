/*** Leo Gabriel 2023-11-14                         ***
 *** Written because I accidentally cooked a nucleo ***
 *** and I need this functionality working tomorrow ***
 *** rtfm: https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 *** For ATMEGA328P but should work on any target   ***/

/************ NOTES ************
 *** delay() doesn't work because timer0 is used    ***
 *** delayMilliseconds() however does work fine     ***/

#include <avr/io.h>         // Library with register names
#include <avr/interrupt.h>  // Library to set up itnerrupts

#define MCU_FREQ_HZ 16000000    // 16 MHz
#define MCU_FREQ_MHZ 16         // 16 MHz
#define LOG_SIZE 40          // 2^16 (2^8 is too few, <1s of samples every 1ms)
#define LOG_TIMEBASE_US 100000  // sample every 1ms
#define CLOCKS_PER_LOG (LOG_TIMEBASE_US * MCU_FREQ_MHZ)


const uint16_t LOG_TIMEBASE_CLOCKS = MCU_FREQ_MHZ * LOG_TIMEBASE_US;

#define ACK 'Z'
#define BAD 'X'

uint8_t tx_ack[3] = { ACK, '\r', '\n' };
uint8_t tx_bad[3] = { BAD, '\r', '\n' };


/*** USED PINOUT ***/
#define SR_RX 0      // Input serial Rx. NC, serial over USB port
#define SR_TX 1      // Output serial Tx. NC, serial over USB port \
                     //d2 is an interrupt, may be useful. d3 is too
#define BRAKE_PIN 3  // Output signal
#define DISP_PIN 4   // Output signal
#define T1_PIN 5     // Input encoder output
#define OC0A_PIN 6   // Output physically wired to ICP1_PIN

#define ICP1_PIN 8    // Input trigger for counter1 input capture. Physically wired to OC0A_PIN
#define OPC1A_PIN 9   // Output set on OCR1A match. NC, only used internally.
#define OPC1B_PIN 10  // Output set on OCR1B match. NC, only used internally. \
                      // D11 and D12 free and unremarkable

#define PAN_STP 13   // Output pan step
#define PAN_DIR A0   // Output pan direction
#define PAN_EN A1    // Output pan enable
#define TILT_STP A2  // Output tilt step
#define TILT_DIR A3  // Output tilt direction
#define TILT_EN A4   // Output tilt enable

uint8_t rxBuffer[64];  // Serial receive buffer. Arbitrary but sufficiently large size

#define MOVING_AVG_LENGTH 256
uint16_t posLog[MOVING_AVG_LENGTH] = {0};
uint8_t log_position = 0;
uint16_t prev_pos = 0;
uint16_t next_next_pos = 0;
uint16_t timepoint_pos = 0;
uint16_t dispense_pos = 0;
uint16_t plunge_done_flag = 0;
uint32_t running_sum = 0;
double speed = 0;
uint32_t clocks_per_encoder_pulse = 0;
uint32_t dispense_delay_clocks = 0;
uint16_t clocks_to_disp = 0;
uint8_t disp_flag = 0;
uint8_t DEPOSITED = 0;
uint32_t pulseCount = 0;



uint16_t dispense_position = 5000;
uint16_t brake_pos = 10000;

/* Movement setup */
uint16_t tiltPos = 0;
uint16_t panPos = 0;
#define TILT_DEFAULT_DELAY 10
#define PAN_DEFAULT_DELAY 10

#define PULSES_TO_DEGREES 200 * 64

#define DIR_PAN_LEFT 1
#define DIR_PAN_RIGHT 0
#define DIR_TILT_UP 1
#define DIR_TILT_DOWN 0
#define PAN_SWITCH_POS 1000
#define TILT_SWITCH_POS 1000
#define PAN_DEG_TO_STEPS 200
#define TILT_DEG_TO_STEPS 200

#define ACK 'Z'
#define BAD 'X'

#define MOVE '1'
#define PLUNGE '2'
#define STATUS '3'
#define RELEASE '4'

#define UP '1'
#define DOWN '2'
#define LEFT '3'
#define RIGHT '4'

/* Timer/coutner helper cosntants */
#define COUNTER_OFF 0b00000101
#define PRESCALING_1024 0b00000101
#define PRESCALING_256 0b00000100
#define PRESCALING_64 0b00000011
#define PRESCALING_64 0b00000010
#define PRESCALING_NONE 0b00000001
#define FALLING_EDGE 0b00000110
#define RISING_EDGE 0b00000111
  

uint8_t captflag = 0;

void setup() {
  sei();  // Enable global interrupts

  // #define OPC1A_PIN 9    // Output set on OCR1A match. NC, only used internally.
  // #define OPC1B_PIN 10   // Output set on OCR1B match. NC, only used internally.
  //                         // D11 and D12 free and unremarkable


  pinMode(BRAKE_PIN, OUTPUT);
  pinMode(DISP_PIN, OUTPUT);
  //pinMode(ICP1_PIN, INPUT);
  //pinMode(5, INPUT); digitalWrite(5, HIGH);
  pinMode(T1_PIN, INPUT);
  pinMode(OC0A_PIN, OUTPUT);
  pinMode(DISP_PIN, OUTPUT);

  pinMode(PAN_STP, OUTPUT);
  pinMode(PAN_DIR, OUTPUT);
  pinMode(PAN_EN, OUTPUT);
  pinMode(TILT_STP, OUTPUT);
  pinMode(TILT_DIR, OUTPUT);
  pinMode(TILT_EN, OUTPUT);

  /* serial setup */
  Serial.begin(115200);

  GTCCR = 0;  // timer setup
  /*** TCNT1 will be used to count encoder pulses ***/
  TCCR1A = 0;
  TCCR1A |= (1 << COM1A0) | (1 << COM1A1);  // Enable output compare A for disp
  TCCR1A |= (1 << COM1B0) | (1 << COM1B0);  // Enable output compare B for brake
  TCCR1C = 0;                               // no nonsense
  TIFR1 = 0;                                // clear interrupt flags

  /*** TCNT0 will be used to generate logging timebase ***/
  TCCR0A |= (1 << COM0A0) | (1 << COM0A1);  // Enable compare on OC0A

  /* CTC mode (010). timer rolls and triggers over at OCR0A*/
  TCCR0A &= ~(1 << WGM00);
  TCCR0A |= (1 << WGM01);  // 1 for CTC
  TCCR0B &= ~(1 << WGM02);

  TIMSK0 = (1 << OCIE0A);  // enable A comapre interrupt

  /*          MHz            uS        scalar prescaler*/
  OCR0A = 0xFF;  //MCU_FREQ_MHZ * LOG_TIMEBASE_US / 64; // Triggers OC0A high when counter0 equals OCR0A
  /*^ if i want to go any slower i need to drop the prescaler again*/

  TCCR0B = 0;
  //TCCR0B = PRESCALING_64; // Statr clock with 1/64 prescaler

  /*** Configure timer 2 for final dispense ***/
  // /* set output compare register for TCNT1 *
  //  * enables compare set on OC0A*/
  // TCCR2A |= (1<<COM2A0);
  // TCCR2A |= (1<<COM2A1);

  // /* Normal mode */
  // TCCR2A &= ~(1<<WGM20);
  // TCCR2A &= ~(1<<WGM21);
  // TCCR2B &= ~(1<<WGM22);
  // TIMSK2 |= (1<<OCIE2A); // enable A comapre interrupt

  //read interrupts from TIFR1 : ICF1, OCF1B, OCF1A
  Serial.println("start!");
}


/*** ISRs ***/

/* Counter 1 A compare interrupt - Dispense */
ISR(TIMER1_COMPA_vect) {
  digitalWrite(DISP_PIN, 1);
  delayMicroseconds(10);
  digitalWrite(DISP_PIN, 0);
  TIMSK1 &= ~(1 << OCIE1A);  //disable this interrupt
}
uint8_t compb_flag = 0;

/* Counter 1 B compare interrupt - Brake */
ISR(TIMER1_COMPB_vect) {
  compb_flag = 1;
  //digitalWrite(BRAKE_PIN, 1);
  TIMSK1 &= ~(1 << OCIE1B);  //disable this interrupt
  /* end of plunge */
  TIFR1 = 0;
  TIMSK1 = 0;
  TCCR1B &= ~(0x00000111);  // Disable counter

  plunge_done_flag = 1;
}

uint16_t current_pos = 0;
uint16_t delta = 0;
uint8_t firstWrapFlag = 0;
/* Trigger data log */
ISR(TIMER0_COMPA_vect) {
/* all the fancy amth goes here if it can run */
  captflag = 1;
   posLog[log_position++] = TCNT1;
   running_sum += TCNT1 - prev_pos;
  
  uint8_t pos1 = log_position - MOVING_AVG_LENGTH + 1;  // using the overflow to create a ciruclar wrap
  uint8_t pos2 = log_position - MOVING_AVG_LENGTH;
  delta = (posLog[pos1] - posLog[pos2]);
  if(running_sum>=delta) running_sum -= delta;
  
  speed = (double)running_sum / (double)MOVING_AVG_LENGTH; // speed in pulses per log

  clocks_per_encoder_pulse = (CLOCKS_PER_LOG / speed); // a form of speed measurement
  next_next_pos = TCNT1 + 2*speed;	// predicted position 2 TIM5 updates from now. if this is beyond dispense_position we want to trigger TIM4

  dispense_pos = (timepoint_pos) - (speed * (dispense_delay_clocks / CLOCKS_PER_LOG)); // dispense_delay_clocks is based on distance and speed that the drop is shooting at
      
  if(next_next_pos > dispense_pos) { // dispense comes within the next timebase
  		digitalWrite(DISP_PIN, 1);
      /*disp_flag = 1;

  		clocks_to_disp = clocks_per_encoder_pulse*(dispense_pos-TCNT1);

      TIFR2 = 0;// Clear timer 2 interrupt flag
      TCCR2B |= ;     // Start timer 2, clock from internal, no prescaler (111)
      */
  	}
    
    prev_pos = TCNT1;

}


uint8_t default_flag = 0;
ISR(__vector_default) {
  default_flag = 1;
}


/*** MOTOR CONTROL FUNCTIONS ***/
void move_tilt_steps(uint32_t dly, uint8_t dir, uint32_t num_steps) {
  digitalWrite(TILT_EN, 0);
  digitalWrite(TILT_DIR, dir);
  for (int i = 0; i < num_steps; i++) {
    digitalWrite(TILT_STP, 1);
    delayMicroseconds(dly);
    digitalWrite(TILT_STP, 0);
    delayMicroseconds(dly);
  }

  tiltPos += num_steps * (1 - 2 * dir);  // + if dir is 0, else -
}

void move_tilt_deg(uint32_t degrees, uint8_t dir) {
  move_tilt_steps(TILT_DEFAULT_DELAY, dir, degrees * TILT_DEG_TO_STEPS);
}

void move_pan_steps(uint32_t dly, uint8_t dir, uint32_t num_steps) {
  digitalWrite(PAN_EN, 0);
  digitalWrite(PAN_DIR, dir);
  for (int i = 0; i < num_steps; i++) {
    digitalWrite(PAN_STP, 1);
    delayMicroseconds(dly);
    digitalWrite(PAN_STP, 0);
    delayMicroseconds(dly);
  }
  panPos += num_steps * (1 - 2 * dir);  // + if dir is 0, else -
}

void move_pan_deg(uint32_t degrees, uint8_t dir) {
  move_pan_steps(PAN_DEFAULT_DELAY, dir, degrees * PAN_DEG_TO_STEPS);
}

/*** USART Rx HANDLE ***/
void ack(void) {
  Serial.print((char)tx_ack);
}

void bad(void) {
  Serial.print((char)tx_bad);
}

void plungeFunc() {
  plunge_done_flag = 0;
  brake_pos = 0; timepoint_pos = 0;

  /* interpret brake and dispense positions set w/ command */
  for (int i = 1; i <= 6; i++) if (rxBuffer[i] >= '0' && rxBuffer[i] <= '9') brake_pos = (brake_pos * 10) + (rxBuffer[i] - '0');
  for (int i = 7; i <= 12; i++) if (rxBuffer[i] >= '0' && rxBuffer[i] <= '9') timepoint_pos = (timepoint_pos * 10) + (rxBuffer[i] - '0');

  /***TODO: figure out what angle to tilt to given timepoint_pos***/

  /* Reset tracking variables */
  log_position = 0;
  running_sum = 0;
  for(int i=0; i<MOVING_AVG_LENGTH; i++) posLog[i] = 0;
  Serial.println("intiialized psolog");
  DEPOSITED = 0;
  dispense_delay_clocks = 10000;

  digitalWrite(BRAKE_PIN, 0);  // Disengage brake

  /* Counter 1 start */
  TCCR1B &= ~(0x00000111);    // Disable counter
  TCNT1 = 0;                  // Reset count
  OCR1A = dispense_position;  // Set dispense position
  OCR1B = brake_pos;          // Set brake position
  Serial.println(brake_pos);
  Serial.println(dispense_position);

  TIMSK1 |= ((1 << OCIE1A) | (1 << OCIE1B) | (1 << ICIE1));  // enable all interrupts

  TIFR1 = 0;  // Clear the interrupt flags
  TCCR1B = 0;
  TCCR1B |= (1<<ICES1);  // capture compare on rising edge of ICP1_PIN. filtering on
  TCCR1B |= RISING_EDGE;  // Start counting on T1 rising edge. no pwm modes on



  /* Counter 0 start */
  TCNT0 = 0;                             //reset counter
  TIFR0 = 0;                             //clear interrupts
  TCCR0B = PRESCALING_64; // Statr clock with 1/64 prescaler
}


void rx_handle(void) {

  Serial.println('Z');

  if (rxBuffer[0] == 52) {
    Serial.println("RELEASE");
    digitalWrite(BRAKE_PIN, 0);  //disengage brake
  } else if (rxBuffer[0] == 53) {
    Serial.println("BRAKE");
    digitalWrite(BRAKE_PIN, 1);  //engage brake

  } else if (rxBuffer[0] == 50) {
    Serial.println("PLUNGE");
    plungeFunc();
  }
  // switch(int(rxBuffer[0])) {
  //   	case MOVE:
  //       uint32_t amount = (rxBuffer[2]-48) << 24 | (rxBuffer[3]-48) << 16 | (rxBuffer[4]-48) << 8 | (rxBuffer[5]-48);
  //       switch(rxBuffer[1]) {
  //         case UP: ;
  //           move_tilt_deg(amount, DIR_TILT_UP);
  //           ack();
  //           break;
  //         case DOWN: ;
  //           move_tilt_deg(amount, DIR_TILT_DOWN);
  //           ack();
  //           break;
  //         case LEFT: ;
  //           move_pan_deg(amount, DIR_PAN_LEFT);
  //           ack();
  //           break;
  //         case RIGHT: ;
  //           move_pan_deg(amount, DIR_PAN_RIGHT);
  //           ack();
  //           break;
  //         default: ;
  //           bad();
  //           break;
  //       }

  // 		break;


  //   }
}

uint8_t first = 1;
void loop() {
  if (Serial.available()) {
    Serial.readBytesUntil('\n', rxBuffer, 32);
    rx_handle();  // poll for serial rx flag
  }
  // if(plunge_done_flag) {
  //   TIMSK1 &= ~((1<<ICIE1)); // Stop data collection
  //   /* Transmit */
  // }
  //Serial.println("Loopyloop");
  //Serial.println(TCNT0);
  // Serial.println("xd");
  
  if(captflag) {
    Serial.println("captured");
    captflag = 0;
  }
  // Serial.print("runsum: ");
  // Serial.println(running_sum);
  //Serial.println(log_position);
  //if(compb_flag) Serial.println("hit compB");
  // if(default_flag) Serial.println("unhandled interrupt");
  // if(ovf1_flag) {
  //   Serial.println("tim1 overflow");
  //   ovf1_flag = 0;
  // }
  for(int i=0; i<40; i++) delayMicroseconds(10000);
}
