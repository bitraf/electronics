#define F_CPU 8000000UL // 8 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "font.h"

#define WIDTH 18
#define HEIGHT 5
#define COMPARE_VALUE 127
#define TICKS_PER_SECOND (1000000UL/COMPARE_VALUE)

void set_led(int x, int y);
void pixel();

volatile uint8_t x, y;
uint32_t buffer[HEIGHT];
volatile uint16_t tick;
uint32_t cast;
volatile int8_t seconds = 0;
volatile int8_t minutes = 2;
volatile int8_t button_status = 0;



ISR(TIMER0_COMPA_vect){
    pixel();
    tick++;
    TCNT0 = 0;
    button_status = (PINB & _BV(3)) | (PINA & _BV(0)) | (PINA & _BV(1));
}
  
void delay_ms_noninline(uint16_t ms){
    for(int i=0; i<ms; i++)
        _delay_ms(1);
}

void pixel(){
    cast = (uint32_t)1<<y;
    if((uint32_t)(buffer[x] & cast))
        set_led(x, 17-y);
    else
        set_led(5, 10);
    x++;
    y++;
    if(x > HEIGHT-1)
        x = 0;
    if(y > WIDTH-1)
        y = 0;
}

void set_sym(uint8_t c, uint8_t offset){
    uint32_t chr = pgm_read_dword(&symbols[c]);
    for(uint8_t i=0; i<5; i++){
        buffer[i] |= ((chr & ((0x7<<(3*i))))>>3*i)<<offset;
    }
}

void clear_buffer(){
    for(uint8_t i=0; i<5; i++){
        buffer[i] = 0;
    }
}

/*void test_pixels(){
    for(uint8_t y=0; y<5; y++){
        buffer[y] = 0xFFFFFFFF;
    }
}*/

void set_led(int x, int y){
    if(x == 5){
        DDRD = 0;
        PORTD = 0;
        DDRB &= ~0x7;
        PORTB &= ~0x7;
        return;
    }
    if(y == 6 || y == 7 || y == 8){
        DDRD = _BV(x);
        PORTD = 0;

        DDRB = _BV(y-6);
        PORTB = _BV(y-6);
    }
    else if(y >= 9 && y<14){
        if(x == 0){
            DDRD = _BV(y-9) | _BV(5);
            PORTD = _BV(y-9);

            DDRB &= ~0x7;
            PORTB &= ~0x7;
        }
        if(x == 1){
            DDRD = _BV(y-9) | _BV(6);
            PORTD = _BV(y-9);

            DDRB &= ~0x7;
            PORTB &= ~0x7;
        }
        if(x == 2){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(0);
            PORTB = 0;
        }
        if(x == 3){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(1);
            PORTB = 0;
        }
        if(x == 4){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(2);
            PORTB = 0;
        }
    }
    else if(y == 14){
        if(x == 0){
            y++;
            DDRD = _BV(y-9) | _BV(5);
            PORTD = _BV(y-9);

            DDRB = 0;
            PORTB = 0;
        }
        if(x == 1){
            DDRD = _BV(y-9) | _BV(6);
            PORTD = _BV(y-9);

            DDRB = 0;
            PORTB = 0;
        }
        if(x == 2){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(0);
            PORTB = 0;
        }
        if(x == 3){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(1);
            PORTB = 0;
        }
        if(x == 4){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(2);
            PORTB = 0;
        }
    }
    else if(y == 15){
        if(x == 0){
            DDRD =  _BV(5);
            PORTD = 0;

            DDRB = _BV(0);
            PORTB = _BV(0);
        }
        if(x == 1){
            DDRD =   _BV(6);
            PORTD = 0;

            DDRB = _BV(0);
            PORTB = _BV(0);
        }
        if(x == 2){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(0);
            PORTB = 0;
        }
        if(x == 3){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(1);
            PORTB = 0;
        }
        if(x == 4){
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB = _BV(2);
            PORTB = 0;
        }
    }
    else if(y == 16){
        if(x == 0){
            DDRD =  _BV(5);
            PORTD = 0;
            DDRB = _BV(1);
            PORTB = _BV(1);
        }
        if(x == 1){
            DDRD =   _BV(6);
            PORTD = 0;

            DDRB = _BV(1);
            PORTB = _BV(1);
        }
        if(x == 2){
            DDRD = 0;
            PORTD = 0;

            DDRB = _BV(0) | _BV(1);
            PORTB = _BV(1);
        }
        if(x == 3){
            DDRD = 0;
            PORTD = 0;

            DDRB = _BV(0) | _BV(1);
            PORTB = _BV(0);
        }
        if(x == 4){
            DDRD = 0;
            PORTD = 0;

            DDRB = _BV(0) | _BV(2);
            PORTB = _BV(0);
        }
    }
    else if(y == 17){
        if(x == 0){
            DDRD = _BV(5);
            PORTD = 0;
            DDRB = _BV(2);
            PORTB = _BV(2);
        }
        if(x == 1){
            DDRD = _BV(6);
            PORTD = 0;

            DDRB = _BV(2);
            PORTB = _BV(2);
        }
        if(x == 2){
            DDRD = 0;
            PORTD = 0;

            DDRB = _BV(0) | _BV(2);
            PORTB = _BV(2);
        }
        if(x == 3){
            DDRD = 0;
            PORTD = 0;

            DDRB = _BV(2) | _BV(1);
            PORTB = _BV(2);
        }
        if(x == 4){
            DDRD = 0;
            PORTD = 0;

            DDRB = _BV(1) | _BV(2);
            PORTB = _BV(1);
        }
    }
    else{
        if(y >= x){
            y++;
        }
        DDRD = _BV(y) | _BV(x);
        PORTD = _BV(y);
        DDRB &= ~0x7;
        PORTB &= ~0x7;
    }    
}


void delay_10_us(uint8_t us){
    for(uint8_t i=0; i<us; i++){
        _delay_us(10);
    }
}

void sound(uint8_t period, uint8_t length){
    DDRB |= _BV(4);
    for(uint16_t i=0; i<length; i++){
        PORTB |= _BV(4);
        delay_10_us(period);
        PORTB &= ~_BV(4); 
        delay_10_us(period);
    }
}

int main(void) {
    CLKPR = (1<<CLKPCE); // Enable change
    CLKPR = 0; // 8 MHz. Change done.
    
    TCCR0B = 1<<CS01; // set up timer with prescaler
    TCNT0 = 0; // initialize counter
    //TIMSK |= (1 << TOIE0); // enable overflow interrupt
    OCR0A = COMPARE_VALUE;
    TIMSK = (1 << OCIE0A); // compare A interrupt

    sei(); // enable global interrupts

    // Setup buttons
    DDRA &= ~(_BV(0) | ~_BV(1));
    PORTA |= _BV(0) | _BV(1);

    DDRB &= ~_BV(3);
    PORTB |= _BV(3);  //enable pull up resistor on button

    uint8_t running = 0;
    clear_buffer();
    set_sym(minutes/10, 15); 
    set_sym(minutes%10, 11);
    set_sym(10, 8);
    set_sym(seconds/10, 4);
    set_sym(seconds%10, 0);

    while(1){
        if(button_status & 0x01){
          button_status &= ~(0x01); // Clear bit
          if (!running) {
            seconds++;
            clear_buffer();
            set_sym(minutes/10, 15); 
            set_sym(minutes%10, 11);
            set_sym(10, 8);
            set_sym(seconds/10, 4);
            set_sym(seconds%10, 0);
          }
        }
        if(button_status & 0x02){
          button_status &= ~(0x02); // Clear bit
          if (!running) {
            minutes++;
            clear_buffer();
            set_sym(minutes/10, 15); 
            set_sym(minutes%10, 11);
            set_sym(10, 8);
            set_sym(seconds/10, 4);
            set_sym(seconds%10, 0);
          }
        }
        if(button_status & 0x04) {
          button_status &= ~(0x04); // Clear bit
          running = !running;
          delay_ms_noninline(50);
        }
        

        if(tick > (int)TICKS_PER_SECOND) {
            tick = 0;
            if (running) {
              seconds--;
              if(seconds == -1){
                  seconds = 59;
                  cli();
                  
                  // Sound until button 3 pushed:
                  uint8_t beeps = 0;
                  while (button_status & 0x04) {
                      button_status &= ~(0x04); // Clear bit
                      beeps++;
                      if (beeps <= 4) {
                        sound(10, 500);
                        delay_ms_noninline(50);
                      }
                      else if (beeps > 4 && beeps < 13) {
                        delay_ms_noninline(50);
                      }
                      else {
                        beeps = 0;
                      }
                  }
                  
                  // A short delay to avoid double-click
                  delay_ms_noninline(500);
                  
                  // Wait until button 3 pushed again before restarting
                  while (button_status & 0x04);
                  button_status &= ~(0x04); // Clear bit
                 
                  sei();                
              }
              clear_buffer();
              set_sym(minutes/10, 15); 
              set_sym(minutes%10, 11);
              set_sym(10, 8);
              set_sym(seconds/10, 4);
              set_sym(seconds%10, 0);
          }
        }
    }
}


