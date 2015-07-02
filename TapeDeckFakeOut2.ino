//-------------------------
//    Fake tapedeck commands for
//    gm factory radio
//-------------------------

#include <SPI.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define DEBUG

//-------------------------
//    debounce stuff
//-------------------------
#define MAXCHECKS 15         //# of checks before a switch is considered debounced
byte debounced_state;        //debounced state of the switches
byte debounced_state_prev;   //previous debounced state of the switches
byte state[MAXCHECKS];       //array maintaining bounce status
byte index;                  //pointer into state
volatile boolean doDebounce;          //if true, call debounce
volatile byte debounceTimerCount;      //number of times the debounce timer has overflowed

//-------------------------
//    Output pins
//-------------------------
//these values are for building bitmasks
//so should be between 0 and 7 inclusive
#define CRO2     2               //digital pin 2
#define ANODE    3               //digital pin 3
#define PHO_TR   4               //digital pin 4
#define MODE     5               //digital pin 5
#define ST_BY    6               //digital pin 6
#define CA_IN    7               //digital pin 7
#define POWER    1               //digital pin 9, port B

//-------------------------
//    Tape State
//-------------------------
#define TAPE_INSERTED 4
#define TAPE_EJECTED  0
//is the tape in or out
byte TapeState;
//the number of times the photr timer has overflowed
volatile int PhoTr_OverflowCnt;
//whether the photr pin is high or low
volatile bool PhoTr_State;


byte ledstate=HIGH;


void setup()
{
	//-------------------------
	//   shutoff the adc....
	//-------------------------
	//23.8.2 ADCSRA – ADC Control and Status Register A
	//Bit 7 – ADEN: ADC Enable
	//Writing this bit to one enables the ADC. By writing it to zero, the ADC is turned off. Turning the
	//ADC off while a conversion is in progress, will terminate this conversion.
	//ADCSRA &= ~(1 << ADEN);
	ADCSRA = 0;
	
	//set port C (ADC) as inputs
	DDRC = 0x00;              //make port c (analog) inputs
	PORTC |= 0xFF;             //turn on port c internal pullups
	
	//set port B as outputs
	DDRB = 0b00111111;        //make port b as output The two high bits (6 & 7) map to the crystal pins and are not usable
        PORTB &= 0b11000000;
        

	//set port D as outputs
	DDRD = DDRD | 0b11111100;

	//init PORTD with values indicating empty tape deck
	PORTD = 0x00 | (
	(1<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(1<<ST_BY)
	|(0<<CA_IN)
	);
	
        //hold power high for ~2 seconds
        PORTB |= (1<<POWER);
        for(byte i=0;i<20;i++){delay(100);}
	PORTB &= ~(1<<POWER);

	//setup tape state vars
	TapeState = TAPE_EJECTED;
	PhoTr_OverflowCnt = 0;
	PhoTr_State= false;
	
	//setup debounce vars
	doDebounce = false;
	debounceTimerCount =0;
	debounced_state_prev=0;
	debounced_state=0;

	//init serial output
	#ifdef DEBUG
	Serial.begin(9600);
	#endif


	// set up timer with prescaler = 64 for timer2 (debounce timer)
	TCCR2B = (1 << CS22)|(0 << CS21)|(0 << CS20);
	// initialize counter for timer0 (debounce timer)
	TCNT2 = 0;
	// enable overflow interrupt for timer2 (debounce timer)
	TIMSK2 |= (1 << TOIE2);

	// enable interrupts
	sei();
	//goto sleep
	//sleep();
}

void loop()
{
        
        #ifdef DEBUG
	int prev;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		prev = PhoTr_OverflowCnt;
	}
	
	
	Serial.println(prev);
	#endif

        
	//Serial.println(debounced_state,BIN);

	if(doDebounce){
                
		debounce();
	}
	//Serial.println(debounced_state,BIN);
	if(debounced_state_prev != debounced_state){
		//key has been pressed or released

		if(GetKeysUp()!=0){
			
			if(TapeState==TAPE_INSERTED){
  
                                digitalWrite(13, LOW);  // 
				TapeEject();
				TapeState=TAPE_EJECTED;
				}else{
                                  digitalWrite(13, HIGH);  // 
				TapeInsert();
				TapeState=TAPE_INSERTED;
			}
			
			//sleep();
		}

	}//end if(key is pressed)


}

void TapeInsert(){
  
  
	
	PORTD = 0x00 | (
	(1<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(1<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(422);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(1<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(314);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(348);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(0<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(41);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(1<<PHO_TR)
	|(0<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(28);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(1<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(20);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(36);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(1<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(93);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(19);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(0<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(71);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(163);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(0<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(556);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(162);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(0<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(71);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(83);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(1<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(52);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(39);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(1<<PHO_TR)
	|(0<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(45);
	//-------------------------
	//    begin PhoTr transition
	//-------------------------
	//159ms off, 247ms on
	
	PhoTr_State=false;
	// set up timer1 with prescaler = 1024
	TCCR1B = (1 << CS12) | (1 << CS10);
	//dnit counter
	TCNT1 = 0;
	//enable interrupt
	TIMSK1  |= (1 << TOIE1);
	//init overflow counter
	PhoTr_OverflowCnt = 0;
	sei();         // turn on interrupts
	
	
	delay(250);//this is so sei will have something to execute...
	
	
}//end tape insert

//-------------------------
//    playback tape ejection
//-------------------------
void TapeEject(){
	
	//disable PhoTr pulse interrupt
	TIMSK1 &= ~(1 << TOIE1);
	TIFR1 &= ~(1 << TOV1);
	PhoTr_OverflowCnt=0;
	
	//begin eject signaling
	PORTD = 0x00 | (
	(0<<CRO2)
	|(0<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(0<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(170);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(0<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(1<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(251);
	//-------------------------
	//    anode goes high here
	//-------------------------
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(1<<ST_BY)
	|(1<<CA_IN)
	);
	
	delay(118);
	PORTD = 0x00 | (
	(0<<CRO2)
	|(1<<ANODE)
	|(0<<PHO_TR)
	|(1<<MODE)
	|(1<<ST_BY)
	|(0<<CA_IN)
	);
	
	delay(175);
	
}//end TapeEject

void PhoTrLow(){
	//-------------------------
	//    begin PhoTr transition
	//-------------------------
	//159ms off, 247ms on
	
	PORTD &= ~(1<<PHO_TR);
}

void PhoTrHigh(){
	//-------------------------
	//    begin PhoTr transition
	//-------------------------
	//159ms off, 247ms on
	
	PORTD |= (1<<PHO_TR);
}


void debounce(){
	
	byte i,j;
	state[index]=readkeys();
	++index;
	j=0xff;
	for(i=0;i<MAXCHECKS;i++){
		j = j & state[i];
	}
	debounced_state_prev=debounced_state;
	debounced_state=j;
	if(index >= MAXCHECKS){
		index =0;
	}
	doDebounce=false;//we just did a debounce, we dont need another one until the next timer
}

//-------------------------
//    read the keys into an byte
//-------------------------
byte readkeys(){
	byte ret=0x3F;   //3F (0x00111111)

	//pinc is the analog port. inverted because we are using active low
	ret &= ~PINC;

	return ret;
}

byte GetKeysUp(){
	//set bits that have changed to 1
	byte ret = debounced_state_prev ^ debounced_state;

	//set bits that have changed and are 1 in the current state
	ret = ret ^ debounced_state;
	return ret;
}

byte GetKeysDown(){

	//set bits that have changed to 1
	byte ret = debounced_state_prev ^ debounced_state;

	//set bits that have changed and are 1 in the current state
	ret = ret & debounced_state;
	return ret;
}

void sleep(){
	//debounce delays
	delay (50);

	// clear any outstanding interrupts
	PCIFR  |= _BV (PCIF0) | _BV (PCIF1) | _BV (PCIF2);
	//setup interrupts
	PCICR |= (1 << PCIE1);    // set PCIE1 to enable PCMSK1 scan

	// set PCINT13 PCINT12	PCINT11	PCINT10	PCINT9	PCINT8
	//to trigger an interrupt on state change
	PCMSK1 |= 0b111111;
	sei();         // turn on interrupts


	// disable ADC
	ADCSRA = 0;
	// turn off various modules
	//PRR = 0xFF;

	set_sleep_mode (SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_cpu ();

	// cancel sleep as a precaution
	//sleep_disable();
}

//-------------------------
//    the pin change interrupt handler
//-------------------------
ISR (PCINT1_vect, ISR_BLOCK)
{
	// cancel pin change interrupts
	PCICR = 0;

	// cancel sleep
	sleep_disable();
}

//-------------------------
//    handler for interrupt of PhoTr LOW timer
//-------------------------
ISR(TIMER1_OVF_vect){
	//-------------------------
	//    begin PhoTr transition
	//-------------------------
	//159ms off, 247ms on
	//prescaler = 64, time between overflows ~= 2ms
	//prescaler = 1024, time between overflows ~= 33 ms

	
	//disable overflow interrupt for timer2 (debounce timer)
	TIMSK2 &= ~(1 << TOIE2);
	// keep a track of number of overflows
	PhoTr_OverflowCnt++;
	if(5 < PhoTr_OverflowCnt && !PhoTr_State){
		//transition to high
		PORTD |= (1<<PHO_TR);
		PhoTr_State = true;
		}else if(12 < PhoTr_OverflowCnt && PhoTr_State){
		//transition to low
		PORTD &= ~(1<<PHO_TR);
		PhoTr_State = false;
		PhoTr_OverflowCnt=0;
	}
	//enable overflow interrupt for timer2 (debounce timer)
	TIMSK2 |= (1 << TOIE2);
	//sei();         // turn on interrupts
}

//-------------------------
//    handler for power/tape insert button press
//-------------------------
ISR(TIMER2_OVF_vect){

	//disable PhoTr pulse interrupt
	int PhoTr_prev= TIMSK1;
	TIMSK1 &= ~(1 << TOIE1);
	// keep a track of number of overflows
	debounceTimerCount++;
	if(2 <= debounceTimerCount){
		//set the doDebounce variable to true
		doDebounce=true;
		debounceTimerCount = 0;
	}
	//enable PhoTr pulse interrupt
	//TIMSK1 |= (1 << TOIE1);
	TIMSK1= PhoTr_prev;
	//sei();         // turn on interrupts
}
