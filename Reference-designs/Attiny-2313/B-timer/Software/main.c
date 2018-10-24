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
volatile uint8_t current_buffer = 0;
uint32_t buffer[2][HEIGHT];
volatile uint16_t tick;
uint32_t cast;
volatile int8_t seconds = 0;
volatile int8_t minutes = 0;
volatile int8_t button_status = 0;

ISR(TIMER0_COMPA_vect){
    pixel();
    tick++;
    TCNT0 = 0;
    button_status = (PINB & _BV(3)) | (PINA & _BV(1)) | (PINA & _BV(0));
}
  
void delay_ms_noninline(uint16_t ms){
    for(int i=0; i<ms; i++)
        _delay_ms(1);
}

void pixel(){
    cast = (uint32_t)1<<y;
    if((uint32_t)(buffer[current_buffer][x] & cast))
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

void set_sym(uint8_t c, uint8_t offset, uint8_t bnr){
    uint32_t chr = pgm_read_dword(&symbols[c]);
    for(uint8_t i=0; i<5; i++){
        buffer[bnr][i] |= ((chr & ((0x7<<(3*i))))>>3*i)<<offset;
    }
}

void clear_buffer(uint8_t bnr){
    for(uint8_t i=0; i<5; i++){
        buffer[bnr][i] = 0;
    }
}

void test_pixels(){
    for(uint8_t y=0; y<5; y++){
        buffer[current_buffer][y] = 0xFFFFFFFF;
    }
}

void set_led(int x, int y){
    DDRB &= ~0x7;
    PORTB &= ~0x7;

    if(y == 6 || y == 7 || y == 8){
        DDRD = _BV(x);
        PORTD = 0;
        DDRB |= _BV(y-6);
        PORTB |= _BV(y-6);
    }
    else if(y >= 9 && y<=14){
        if(y == 14 && x == 0)
            y++;
        if(x <= 1){
            DDRD = _BV(y-9) | _BV(x+5);
            PORTD = _BV(y-9);
        }
        else{
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);
            DDRB |= _BV(x-2);
        }
    }
    else if(y == 15){
        if(x <= 1){
            DDRD =  _BV(x+5);
            PORTD = 0;

            DDRB |= _BV(0);
            PORTB |= _BV(0);
        }
        else{
            DDRD = _BV(y-9);
            PORTD = _BV(y-9);

            DDRB |= _BV(x-2);
        }
    }
    else if(y == 16){
        if(x <= 1){
            DDRD =  _BV(x+5);
            PORTD = 0;
            DDRB |= _BV(1);
            PORTB |= _BV(1);
        }
        else{
            DDRD = 0;
            PORTD = 0;
            if(x == 2){
                DDRB |= _BV(0) | _BV(1);
                PORTB |= _BV(1);
            }
            if(x == 3){
                DDRB |= _BV(0) | _BV(1);
                PORTB |= _BV(0);
            }
            if(x == 4){
                DDRB |= _BV(0) | _BV(2);
                PORTB |= _BV(0);
            }
        }
    }
    else if(y == 17){
        if(x <= 1){
            DDRD = _BV(x+5);
            PORTD = 0;
            DDRB |= _BV(2);
            PORTB |= _BV(2);
        }
        else{
            DDRD = 0;
            PORTD = 0;
            if(x == 2){
                DDRB |= _BV(0) | _BV(2);
                PORTB |= _BV(2);
            }
            if(x == 3){
                DDRB |= _BV(1) | _BV(2);
                PORTB |= _BV(2);
            }
            if(x == 4){
                DDRB |= _BV(1) | _BV(2);
                PORTB |= _BV(1);
            }
        }
    }
    else{
        if(y >= x){
            y++;
        }
        DDRD = _BV(y) | _BV(x);
        PORTD = _BV(y);
    }    
}

void delay_10_us(uint8_t us){
    for(uint8_t i=0; i<us; i++){
        _delay_us(10);
    }
}

void sound(uint8_t period, uint16_t length){
    DDRB |= _BV(4);
    for(uint16_t i=0; i<length; i++){
        PORTB |= _BV(4);
        delay_10_us(period);
        PORTB &= ~_BV(4); 
        delay_10_us(period);
    }
}

uint8_t button_1_pressed(){
    return !(button_status & _BV(0));
}

uint8_t button_2_pressed(){
    return !(button_status & _BV(1));
}

uint8_t button_3_pressed(){
    return !(button_status & _BV(3));
}

void alarm(){
    uint8_t beeps = 0;
    
    while(!button_3_pressed()) {
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
}

void update_time(){
    uint8_t next_buffer = (current_buffer == 0 ? 1 : 0);
    clear_buffer(next_buffer);
    set_sym(minutes/10, 14, next_buffer); 
    set_sym(minutes%10, 10,next_buffer);
    set_sym(10, 7, next_buffer);
    set_sym(seconds/10, 4, next_buffer);
    set_sym(seconds%10, 0, next_buffer);
    current_buffer = next_buffer;
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
    
    update_time();
    while(1){
        if(button_1_pressed()){
          while(button_1_pressed());
          if (!running) {
            seconds++;
            if(seconds > 59){
                seconds = 59;
            }
            update_time();
          }
        }
        if(button_2_pressed()){
          while(button_2_pressed());
          if (!running) {
            minutes++;
            if(minutes > 99){
                minutes = 99;
            }
            update_time();
          }
        }
        if(button_3_pressed()) {
          while(button_3_pressed());
          running = !running;
          update_time();
        }
        
        if(tick == (int)TICKS_PER_SECOND) {
            tick = 0;
            if (running){
              seconds--;
              if(seconds < 0){
                if(minutes > 0){
                    minutes--;
                    seconds = 59;
                }
                else{
                    seconds = 0;
                }
              }
            }
            update_time();
            if(running){    
              if(seconds == 0 && minutes == 0){
                  alarm();
              }              
            }
        }
    }
}


