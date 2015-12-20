/***************************************************************************\
 * hexclock.c                                                               *
 *                                                                          *
 * Created: 7/24/2015 4:02:13 AM                                            *
 *  Author: IRQ42                                                           *
 *                                                                          *
 * Version: 1.1                                                             *
 *                                                                          *
 * Hexclock is a user friendly implementation of binary time for            *
 * Atmel ATtiny2313a.                                                       *
 *                                                                          *
 * Added implementation to allow ordinary time as an option                 *
 * Changed display of '9' character to be a reflection of '6'               *
\***************************************************************************/
#define F_CPU 4096000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdint.h>
#include <stddef.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <avr/eeprom.h>

/* Keys */
#define DDR_SETK    DDRD
#define PORT_SETK   PORTD
#define PIN_SETK    PIND
#define SETK_MASK   _BV(PIND0)
#define DDR_PLUSK   DDRD
#define PORT_PLUSK  PORTD
#define PIN_PLUSK   PIND
#define PLUSK_MASK  _BV(PIND1)
#define DDR_SNOZK   DDRD
#define PORT_SNOZK  PORTD
#define PIN_SNOZK   PIND
#define SNOZK_MASK  _BV(PIND2)

/* Alarm Switch */
#define DDR_ALRMSW  DDRD
#define PORT_ALRMSW PORTD
#define PIN_ALRMSW  PIND
#define ALRMSW_MASK _BV(PIND3)

/* Piezo Buzzer */
#define DDR_BUZZ    DDRD
#define PORT_BUZZ   PORTD
#define BUZZ        _BV(PORTD4)

/* User Definable Snooze Jumper */
#define DDR_USNOZ   DDRD
#define PORT_USNOZ  PORTD
#define PIN_USNOZ   PIND
#define USNOZ_MASK  _BV(PIND5)

/* Shift Register */
#define DDR_SREG    DDRB
#define PORT_SREG   PORTB
#define SRCK        _BV(PORTB0)
#define RCK         _BV(PORTB1)
#define SER         _BV(PORTB2)
#define G           _BV(PORTB3)

/* Display Chip Select */
#define DDR_CS      DDRB
#define PORT_CS     PORTB
#define CS_1        _BV(PORTB4)
#define CS_2        _BV(PORTB5)
#define CS_3        _BV(PORTB6)
#define CS_4        _BV(PORTB7)
#define CS_ALL      (CS_1 | CS_2 | CS_3 | CS_4)

/* Display D.P. */
#define DISP_DP1    8
#define DISP_DP2    4
#define DISP_DP3    2
#define DISP_DP4    1

/* Timers */
#define OCR0A_VAL           127

#define OCR1A_BASEVAL       21092
#define OCR1A_VAL_ADJ       1
#define OCR1A_VAL           (OCR1A_BASEVAL - OCR1A_VAL_ADJ)
#define OCR1A_SLIP          11
#define OCR1A_SLIPCNT       16

/* TIM0 Interrupts per ms */
#define TIM0_INT_PER_MS     4

/* Display/alarm blink/strobe timing */
#define BLINK_MS    500
#define STROBE_MS   133
#define BUZZ_MS     333 
#define BLINK_CNT   (TIM0_INT_PER_MS * BLINK_MS) 
#define STROBE_CNT  (TIM0_INT_PER_MS * STROBE_MS)
#define BUZZ_CNT    (TIM0_INT_PER_MS * BUZZ_MS)

/* Time to hold keys to set user defined snooze interval */
#define USNOOZE_KEYS_MS     500    
#define USNOOZE_KEYS_CNT    (TIM0_INT_PER_MS * USNOOZE_KEYS_MS)

/* Define default snooze interval */
#define DEFAULT_SNOOZE_INT  0x16C   // Approx 8 min

/* User snooze interval EEPROM pointer */
#define USNOOZE_EEPROM_ADDR 0x0

enum alert_state {
    ALERT_OFF,
    ALERT_ON,
};

enum keyevent_type {
    KEYDOWN,
    KEYUP,
};

enum keysym {
    KEY_SET,
    KEY_PLUS,
    KEY_SNOOZE,
};

struct k_state {
    uint8_t set_state;
    uint8_t set_prevstate;
    uint8_t plus_state;
    uint8_t plus_prevstate;
    uint8_t snooze_state;
    uint8_t snooze_prevstate;
};

struct disp_attr {
    uint8_t blinkon;                        /* Blink feature enable     */
    uint8_t strobeon;                       /* Strobe feature enable    */
    uint8_t output_enab;                    /* Display shift register G */
    uint8_t strobe_flg;                     /* Display illum. state     */
};

struct keyevent {
    enum keyevent_type  type;               /* Type of event            */ 
    enum keysym         key;                /* Which key                */                
};

struct buzz_attr {
    uint8_t buzzon;                         /* Buzzer alert enable       */
    uint8_t buzz_flg;                       /* Buzzer chirp duty cycle   */
    uint8_t buzz_sig_level;                 /* Buzzer output signal      */
};

struct time {
    uint8_t hours;
    uint8_t minutes;
};

void init_timer(void);
void display_write(uint8_t data, uint8_t cursor);
uint8_t set_select(volatile uint16_t *time);
void user_snooze_set_select(uint16_t *interval);
void display_update_buffer(uint16_t val, uint8_t dp_flags);
void display_set_output_enab_hwstate(uint8_t output_enab_on);
void display_strober(uint8_t strobe_flg, volatile uint8_t *output_enab_on);
void alarm_set_alert(enum alert_state flag);
void buzzer_set_pin_level(uint8_t level);
void display_update_sequence(void);
uint8_t poll_keyevent(struct keyevent *event); 
void update_hhmm_time(void);
uint16_t hhmm_to_hex(struct time *hhmm);

volatile uint16_t           g_time;
volatile uint16_t           g_alarm;
volatile uint8_t            g_displaybuf[4];
volatile uint8_t            g_alarm_sw_state;
volatile uint8_t            g_user_snooze_on;
volatile uint8_t            g_user_snooze_set_flg;
volatile uint8_t            g_hhmm_time_on;
volatile uint16_t           g_hhmm_time;
volatile struct k_state     g_keystate;
volatile struct disp_attr   g_disp_attr;
volatile struct buzz_attr   g_buzz_attr;

int main(void)
{
    uint16_t alarm_time = g_alarm;
    uint8_t clock_is_set = 0;
    uint8_t snooze_on = 0;
    enum alert_state alert_state = ALERT_OFF;
    struct keyevent event;

    /* Setup */
    clock_prescale_set(clock_div_2);        // Set clock prescaler

    DDR_SREG |= (SRCK | RCK | SER | G);     // Config SREG Pins as Outputs
    PORT_SREG &= ~(SRCK | RCK | SER | G);   // Set Shift Register Pins Low
    
    DDR_BUZZ |= BUZZ;                       // Config buzzer pin as Output
    PORT_BUZZ &= ~BUZZ;                     // Initialize low
    
    DDR_CS |= CS_ALL;                       // Config Chip Select Pins Output
    PORT_CS |= CS_ALL;                      // Set Chip Select Pins High

    DDR_SETK &= ~SETK_MASK;                 // Config Key Pins as Input
    DDR_PLUSK &= ~PLUSK_MASK;
    DDR_SNOZK &= ~SNOZK_MASK;
    PORT_SETK |= SETK_MASK;                 // Enable Pullup Resistors
    PORT_PLUSK |= PLUSK_MASK;
    PORT_SNOZK |= SNOZK_MASK;
    DDR_ALRMSW &= ~ALRMSW_MASK;             // Config Switch Pin as Input
    PORT_ALRMSW |= ALRMSW_MASK;             // Enable Pullup Resistor

    DDR_USNOZ &= ~USNOZ_MASK;               // User Definable Snooze Jumper
    PORT_USNOZ |= USNOZ_MASK;               // Enable Pullup Resistor


    
    /* Initialize display attributes */
    g_disp_attr.output_enab = 1;
    init_timer();
    sei();
    
    for (;;) 
    {
        /* Enable blink effect if clock has not been set */
        g_disp_attr.blinkon = !clock_is_set;

        /* If snooze is not on, alarm_time gets g_alarm */
        if (!snooze_on)
           alarm_time = g_alarm;

        /* Enter user snooze set mode if flag is true */
        if (g_user_snooze_set_flg) {
            uint16_t usnooze_tmp_int;
           
            usnooze_tmp_int = eeprom_read_word((uint16_t*)USNOOZE_EEPROM_ADDR); 
            user_snooze_set_select(&usnooze_tmp_int);

            // write EEPROM only if value has changed
            eeprom_update_word((uint16_t*)USNOOZE_EEPROM_ADDR, usnooze_tmp_int);
        }

        /* Start/Stop alerting depending on alert_state var */
        alarm_set_alert(alert_state);
        
        /* Poll key event */
        if (poll_keyevent(&event))
           if (event.type == KEYDOWN)
               switch (event.key) {

                /* Listen for set key presses */
                case KEY_SET:
                   alarm_set_alert(ALERT_OFF);
                   display_set_output_enab_hwstate(g_disp_attr.output_enab = 1);
                   clock_is_set = set_select((g_alarm_sw_state) ? &g_alarm : &g_time);
                   break;

                /* Listen for snooze key presses */
                case KEY_SNOOZE:
                   // turn snooze on, push alarm time forward
                   if (alert_state == ALERT_ON) {
                       uint16_t snooze_interval;

                       /* Get time to snooze for */
                       if (g_user_snooze_on)
                           snooze_interval = eeprom_read_word((uint16_t*)USNOOZE_EEPROM_ADDR);
                       else
                           snooze_interval = DEFAULT_SNOOZE_INT;

                       snooze_on = 1;
                       alert_state = ALERT_OFF;
                       alarm_time = g_time + snooze_interval;
                   }
                   break;

                default:
                  break; 
               }

        /* Turn alarm alert on if clock is set, and alarm time reached */
        if (g_alarm_sw_state && clock_is_set && g_time == alarm_time) {
            alert_state = ALERT_ON;
        } else if (!g_alarm_sw_state) {
            /* When alarm switch is turned off, turn off alert, and set
             * snooze_on to 0, so that alarm_time gets reset to g_alarm */
            alert_state = ALERT_OFF;
            snooze_on = 0;
        }

        /* Handle display strobe/blink effect */
        if (g_disp_attr.strobeon || g_disp_attr.blinkon) {
            display_strober(g_disp_attr.strobe_flg, &g_disp_attr.output_enab);
        } else {
            g_disp_attr.output_enab = 1;
            g_disp_attr.strobe_flg = 0;
        }
        
        display_set_output_enab_hwstate(g_disp_attr.output_enab);
        display_update_buffer(g_time, (clock_is_set) ?
                0 : DISP_DP1 | DISP_DP2 | DISP_DP3 | DISP_DP4);
    }
}

/**
 * Sets up Timer/Counters
 */
void init_timer(void)
{
    /* -- Using 16 bit Timer1 -- */
    // Clear TCCR1A
    TCCR1A = 0;
    
    // Clock Select -- clk/256
    TCCR1B = _BV(CS12);
    
    // Clear on Compare Match (CTC)
    TCCR1B |= _BV(WGM12);
    
    // Set Compare Match A Register
    // Must slip clock every few matches to reduce drift
    OCR1A = OCR1A_VAL;  // (1.31...s per interrupt)

    /* -------------------------------------------- *\
     * Compare Match A interrupt is left disabled   *
     * until clock time is set initially. (Timer1)  *
    \* -------------------------------------------- */
    
    /* -- Using 8-bit Timer0 -- */
    // Clear on Compare Match (CTC)
    TCCR0A = _BV(WGM01);
    
    // Clock Select clk/8
    TCCR0B = _BV(CS01);
    
    // Set Compare Match A Register
    OCR0A = OCR0A_VAL;  // 0.25ms tick
    
    // Output Compare Match A interrupt enable (TIMER0/1)
    TIMSK = _BV(OCIE0A);
}

/** 
 * Updates display buffer with val. dp_flags can be used to set
 * the display's decimal points on, or off. Flags can be combined
 * using bitwise OR
 */
void display_update_buffer(uint16_t val, uint8_t dp_flags)
{
    static const uint8_t symtab[16] PROGMEM = {
        0xfc, 0x60, 0xda, 0xf2,
        0x66, 0xb6, 0xbe, 0xe0,
        0xfe, 0xf6, 0xee, 0x3e,
        0x9c, 0x7a, 0x9e, 0x8e
    };
    
    for (unsigned char i = 0; i < 4; ++i, val >>= 4, dp_flags >>= 1) {
        g_displaybuf[i] = pgm_read_byte(&symtab[val & 0xf]) | (dp_flags & 1);
    }
}

/**
 * Toggles output_enable line of display according to the value of strobe
 * flag
 */
void display_strober(uint8_t strobe_flg, volatile uint8_t *output_enab_on)
{
    *output_enab_on = strobe_flg;
}

/**
 * Sets the level of the buzzer pin
 */
void buzzer_set_pin_level(uint8_t level)
{
    if (level)
        PORT_BUZZ |= BUZZ;
    else
        PORT_BUZZ &= ~BUZZ;
}

/**
 * Controls alarm alert feature
 */
void alarm_set_alert(enum alert_state flag)
{
    switch (flag) {
        case ALERT_ON:
            g_disp_attr.strobeon = 1;
            g_buzz_attr.buzzon = 1;
            break;
        case ALERT_OFF:
            g_disp_attr.strobeon = 0;
            g_buzz_attr.buzzon = 0;
            break;
    }
}

/**
 * Sets state of G pin to display according to the current software display
 * attribute
 */
void display_set_output_enab_hwstate(uint8_t output_enab_on)
{
    if (output_enab_on) {
        PORT_SREG &= ~G;      
    } else {
        PORT_SREG |= G;
    }
}

/**
 * Sets time on clock in groups of 4 bits, returns status indicating if the
 * clock time had been previously set.
 */
uint8_t set_select(volatile uint16_t *time)
{
    static uint8_t  set;                    // has clock been initially set
    uint16_t        new_time;
    uint8_t         dp_cursor = DISP_DP1;   // use dp to indicate position
    uint8_t         alarm_sw_entry_state = g_alarm_sw_state;
    struct keyevent event;
    int8_t          i = 12;

    
    // new_time gets time
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        new_time = *time;
    }
    
    while (i >= 0) {
        uint8_t nibble = (new_time >> i) & 0x0f;

        /**
         * Clear flag so that keys held while in set_select mode will not cause
         * the device to enter mode for setting user snooze interval after return.
         * This makes the behavior more clear and predictable to the user. Keys 
         * must be held during normal operating mode, ie not in set_select mode 
         * This also takes care of two problems at once by ensuring that when we
         * actually are setting the user snooze, the mode will not be entered 
         * again over and over.
         */
        g_user_snooze_set_flg = 0;

        if (poll_keyevent(&event))
            if (event.type == KEYDOWN)
                switch (event.key) {
                    /* Set/Select Key cycles through 4-bit chunks of time */
                    case KEY_SET:
                        dp_cursor >>= 1;
                        i -= 4;
                        continue;

                    /* Presses of plus key increments time by one unit */
                    case KEY_PLUS:
                        
                        // Simulates a 4-bit register overflow behavior
                        nibble = (nibble < 0xf) ? nibble + 1 : 0;
                        break;
                    
                    /* Other Keys have no effect here */
                    default:
                        break;
                }

        // shift nibble back and insert into new_time
        new_time &= (~(0xf << i));
        new_time |= ((uint16_t) nibble << i);
        
        // update display
        display_update_buffer(new_time, dp_cursor);

       /**
        * If we are currently setting the time and the alarm mode switch
        * position is changed, then we will return immediately discarding the
        * users input in order to avoid the ambiguity of which time has been
        * or is being set.
        */
        if (alarm_sw_entry_state != g_alarm_sw_state)
            return set;
    }

    // time gets new time    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        *time = new_time;
    }

    // Clear Timer Value if g_time has been reset to ensure a full "1st second"
    if (time == &g_time) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            TCNT1 = 0;
        }

        // Output Compare Match A enable once g_time has been set
        if (!set) {
            TIMSK |= _BV(OCIE1A);

            // clock has been set 
            set = 1;

            /* ------------------------------------------------------------- *\
             * Workaround for annoying bug where setting g_time immediately
             * after the device has been reset causes the time to be set an
             * extra second ahead. Needs to be fixed proper, but this works.
            \* --------------------------------------------------------------*/
            --*time;
        }
    }

    return set;
}

/**
 * Set user snooze interval. Basically re-uses set_select, adding snooze
 * specific features for user-friendliness. When function is entered, a
 * message will appear on the display indicated that we are programming the
 * snooze interval. When set_select key is pressed, we will enter set_select
 * function as normal in order to set the snooze interval */
void user_snooze_set_select(uint16_t *interval)
{
    static const uint8_t user_msg_data[4] PROGMEM = {
        0xbe, 0xfc, 0x0a, 0xce
    };
    struct keyevent event;


    /* We don't have a function specifically for writing arbitrary symbols to
     * the display buffer and I don't see any use for one anywhere else so we
     * will simply poke the buffer directly */
    for (unsigned char i = 0; i < 4; ++i)
        g_displaybuf[i] = pgm_read_byte(&user_msg_data[i]);

    for (;;) {
        if (poll_keyevent(&event) && event.type == KEYDOWN)
            switch (event.key) {
                // user presses set key to continue to set/select
                case KEY_SET:
                    /* Setting here for consistency, even though it gets set
                     * in set_select */
                    g_user_snooze_set_flg = 0;
                    (void) set_select(interval);
                    return;

                // user can press snooze key to abort setting snooze interval
                case KEY_SNOOZE:
                    /* ---------------------------------------------------- *\
                     * This flag MUST be set here, else we get stuck in a   *
                     * loop where this function is immediately re-entered   *
                     * after we abort via snooze key since flag would still *
                     * be enabled !!                                        *
                    \* -----------------------------------------------------*/
                    g_user_snooze_set_flg = 0;
                    return;
                
                // all other keys have zero effect
                default:
                    break;
            }
    }
}

/**
 * When called the next position in the update sequence is shown on the
 * 7-segment display device.
 */
void display_update_sequence(void)
{
    static uint8_t cursor;

    cursor = (cursor < 3) ? cursor + 1 : 0;
    display_write(g_displaybuf[cursor], cursor);
}

void display_write(uint8_t data, uint8_t cursor)
{
    /* Set Chip Select High (blank) */
    PORT_CS |= CS_ALL;
        
    /* Bit-bang shift register */
    // latch low
    PORT_SREG &= ~RCK;
    for (char i = 0; i < 8; ++i) {
        
        // clock low
        PORT_SREG &= ~SRCK;
        if (data & 1)
            PORT_SREG |= SER;
        else
            PORT_SREG &= ~SER;
        
        // clock high
        PORT_SREG |= SRCK;
        data >>= 1;
    }
    
    // latch high
    PORT_SREG |= RCK;
    
    /** 
     * To Select and Enable 7-seg chips in sequence
     * Symbols are stored least significant first by numerical power
     */
    switch (cursor) {
        case 0:
            PORT_CS &= ~CS_4;
            break;
        case 1:
            PORT_CS &= ~CS_3;
            break;
        case 2:
            PORT_CS &= ~CS_2;
            break;
        case 3:
            PORT_CS &= ~CS_1;
            break;
    }
}

/**
 * Polls for a key event. Returns true if an event was detected, else false
 */
uint8_t poll_keyevent(struct keyevent *event)
{
    enum keyevent_type  type            = 0;
    enum keysym         key             = 0;

    if (g_keystate.set_state && !g_keystate.set_prevstate) {
        g_keystate.set_prevstate = 1;
        type = KEYDOWN;
        key = KEY_SET;
        goto event;
    } else if (!g_keystate.set_state && g_keystate.set_prevstate) {
        g_keystate.set_prevstate = 0;
        type = KEYUP;
        key = KEY_SET;
        goto event;
    }

    if (g_keystate.plus_state && !g_keystate.plus_prevstate) {
        g_keystate.plus_prevstate = 1;
        type = KEYDOWN;
        key = KEY_PLUS;
        goto event;
    } else if (!g_keystate.plus_state && g_keystate.plus_prevstate) {
        g_keystate.plus_prevstate = 0;
        type = KEYUP;
        key = KEY_PLUS;
        goto event;
    }

    if (g_keystate.snooze_state && !g_keystate.snooze_prevstate) {
        g_keystate.snooze_prevstate = 1;
        type = KEYDOWN;
        key = KEY_SNOOZE;
        goto event;
    } else if (!g_keystate.snooze_state && g_keystate.snooze_prevstate) {
        g_keystate.snooze_prevstate = 0;
        type = KEYUP;
        key = KEY_SNOOZE;
        goto event;
    }
    
    return 0;

event:
    // place event data in callers keyevent struct
    event->type = type;
    event->key  = key;
    
    return 1;
}

#ifdef ENABLE_HHMM_TIME
/* ------------------------------------------------------------------------ *\
 * The following has not been tested, and is not complete. It has not been  *
 * developed further due to lack of available program memory!               *
\* ------------------------------------------------------------------------ */

/**
 * Updates 24 hour time by converting the current hexadecimal time to 24hr
 * format
 */
void update_hhmm_time(void)
{
    uint32_t    seconds; 
    struct time hhmm;
    
    seconds = (uint32_t) (g_time * (86400.0 / 65536.0));
    hhmm.hours = seconds / 3600;
    hhmm.minutes = (seconds % 3600) / 60;

    g_hhmm_time = hhmm_to_hex(&hhmm);
}

uint16_t hhmm_to_hex(struct time *hhmm)
{
    uint8_t dec_digits[4];
    uint16_t hex = 0;

    /* Get digits for 24hr time */
    dec_digits[0] = hhmm->hours / 10;
    dec_digits[1] = hhmm->hours % 10;
    dec_digits[2] = hhmm->minutes / 10;
    dec_digits[3] = hhmm->minutes % 10;

    /* Now compose into a 16-bit hexadecimal number */
    for (uint8_t *p = dec_digits; p < 4 + dec_digits; ++p, hex <<= 4) {
        hex |= *p;
    }

    return hex;
}
#endif

/* The interrupt code here
 ****************************************************************************/
/**
 * Timer0 -- Compare Match on OCR0A every 0.25ms
 * Scan and update global key states
 * Update Multiplexed 7-segment display
 */
ISR(TIMER0_COMPA_vect, ISR_NOBLOCK)
{
    static uint8_t  tick_acc;
    static uint16_t blink_tick_cnt;
    static uint16_t strobe_tick_cnt;
    static uint16_t buzz_tick_cnt;
    static uint16_t user_snooze_tick_cnt;

    /* 250Hz full refresh rate */
    if (!(tick_acc & 3)) 
        display_update_sequence();
    
    /* Poll input keys/switch/jumper each 16ms */
    if (!(tick_acc & 63)) {
        // keys active low
        g_keystate.set_state = !(PIN_SETK & SETK_MASK);
        g_keystate.plus_state = !(PIN_PLUSK & PLUSK_MASK);
        g_keystate.snooze_state = !(PIN_SNOZK & SNOZK_MASK);

        // alarm switch is active high
        g_alarm_sw_state = PIN_ALRMSW & ALRMSW_MASK; 

        // user definable snooze jumper is active low
        g_user_snooze_on = !(PIN_USNOZ & USNOZ_MASK);
    }

    /* If combo of keys is held for enough time, we set a flag so that we
     * can set the user defined snooze interval in main */
    if (g_keystate.snooze_state && g_keystate.plus_state) {
        if (user_snooze_tick_cnt == USNOOZE_KEYS_CNT) {
            g_user_snooze_set_flg = 1;
            user_snooze_tick_cnt = 0;
        } else {
            ++user_snooze_tick_cnt;
        }
    } else {
        user_snooze_tick_cnt = 0;
    }

    /* Display strobe timer: Toggle strobe_flg if strobe effect is on */
    if (g_disp_attr.strobeon) {
        if (strobe_tick_cnt == STROBE_CNT) { 
            g_disp_attr.strobe_flg ^= 1;
            strobe_tick_cnt = 0;
        } else {
            ++strobe_tick_cnt;
        }
    }

    /* Display slow blink effect timer: Toggle strobe_flg if blink effect is on */
    if (g_disp_attr.blinkon) {
        if (blink_tick_cnt == BLINK_CNT) {
            g_disp_attr.strobe_flg ^= 1;
            blink_tick_cnt = 0;
        } else {
            ++blink_tick_cnt;
        }
    }

    /* Piezo Buzzer: Generate 2kHz sq. wave, Toggle buzz_flg if buzzer is on */
    if (g_buzz_attr.buzzon) {
        if (buzz_tick_cnt == BUZZ_CNT) {
            g_buzz_attr.buzz_flg ^= 1;
            buzz_tick_cnt = 0;
        } else {
            ++buzz_tick_cnt;
        }
        g_buzz_attr.buzz_sig_level ^= 1;
    }

    /* Buzzer chirp effect */
    buzzer_set_pin_level((g_buzz_attr.buzzon && g_buzz_attr.buzz_flg) ?
                g_buzz_attr.buzz_sig_level : 0);

   /* Increment tick accumulator */
    ++tick_acc;
}

/**
 * Timer1 -- Compare Match on OCR1A
 * 86400/65536s tick, or if g_24hr_time is one 1s
 */
ISR(TIMER1_COMPA_vect)
{
    static uint8_t tick_slip_cnt;
    static uint8_t slipped;

    /**
     * When tick_slip_cnt reaches OCR1A_SLIPCNT, we slip the compare register
     * value forward. This is to make up for the fractional part of the ideal
     * compare match value so that it doesn't become significant. On the next
     * tick, we reset the compare match register to the usual value.
     */
    if (tick_slip_cnt == OCR1A_SLIPCNT) {
        OCR1A += OCR1A_SLIP;
        slipped = 1;
    } else if (slipped) {
        OCR1A = OCR1A_VAL;
        slipped = 0;
    }

    // Roll tick slip counter
    tick_slip_cnt = (tick_slip_cnt < OCR1A_SLIPCNT) ? tick_slip_cnt + 1 : 0;

    // Increment global time
    ++g_time;

#ifdef ENABLE_HHMM_TIME
    if (g_hhmm_time_on) {
        update_hhmm_time();
    }
#endif
}
