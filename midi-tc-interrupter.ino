/*  MIDI Interrupter 1.0  for DRSSTC using several arduinos based on 328P (UNO/nano/micro pro...) 
    in cascade connection.

    Reads commands from MIDI connection on serial port and generates output pulses playing two notes
    per MCU using Timer1 on CTC mode.When 2 notes are playing simultaneously Busy signal goes high.
    This signal must be connected with enable input of next processor. 
    Enable pin of first processor must be pulled up (i.e. with 10k resistor).
    Not enabled processor simply ignores serial data, unless is playing a note, when it waits for the
    note_OFF command.

    Output signal can be used to control a DRSSTC via optic link (optic fiber or TTL-modulated laser beam)

    written by: Amador Alzaga - (c) 2014 - amadoralzaga.com

    NOT FOR COMERCIAL USE

*/

#define out_pin    2             // theese pins are handled together as PORTD bits.
#define busy_pin   3             // changing values here will not do any effect
#define enable_pin 4

  //   7.5us time resolution tables, period and t_on for each midi note.(in 7.5us ticks)
  //   Calculating values at runtime is not a valid option for speed reasons.
  //   Timer interrupt occurs every 7.5us so interrupt handling routine must be short and fast executable. 
  //   For this reason commands like digitalWrite (with several us of execution time) are out of rule.
  
static int period[127] = {15393,14529,13714,12944,12217,11532,10884,10274,9697,9153,8639,8154,7696,7264,6857,6472,
                          6109,5766,5442,5137,4848,4576,4319,4077,3848,3632,3428,3236,3054,2883,2721,2568,2424,2288,
                          2160,2039,1924,1816,1714,1618,1527,1441,1361,1284,1212,1144,1080,1019,962,908,857,809,
                          764,721,680,642,606,572,540,510,481,454,429,404,382,360,340,321,303,286,270,255,240,227,
                          214,202,191,180,170,161,152,143,135,127,120,114,107,101,95,90,85,80,76,72,67,64,60,57,54,
                          51,48,45,43,40,38,36,32,30,28,27,25,24,23,21,20,19,18,17,16,15,14,13,13,12,12,11,10};

static byte t_on[127] = {26,26,26,26,26,26,26,25,25,25,25,25,25,25,24,24,24,24,24,24,24,23,23,23,23,23,23,22,22,22,21,
                         21,21,20,20,20,20,20,19,19,19,19,19,19,18,18,18,18,18,18,18,17,17,17,16,16,16,15,15,15,14,14,
                         13,13,12,12,11,11,10,9,9,8,8,7,7,6,6,5,5,5,5,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,
                         2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

/* 10 us resolution table
static int period[127] = {11545,10897,10285,9708,9163,8649,8163,7705,7273,6865,6479,6116,5772,5448,5143,4854,4582,
                          4324,4082,3853,3636,3432,3240,3058,2886,2724,2571,2427,2291,2162,2041,1926,1818,1716,1620,
                          1529,1443,1362,1286,1213,1145,1081,1020,963,909,858,810,764,722,681,643,607,573,541,510,
                          482,455,429,405,382,361,341,321,303,286,270,255,241,227,215,202,191,180,170,161,152,143,
                          135,128,120,114,107,101,96,90,85,80,76,72,68,64,60,57,54,51,48,45,43,40,38,36,34,32,
                           30,28,27,25,24,23,21,20,19,18,17,16,15,14,13,13,12,11,11,10,9,9,8,8};
                           
static byte t_on[127] = {25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,
                         25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,24,22,21,20,19,18,17,16,15,14,13,13,12,11,11,
                         10,9,9,8,8,7,7,7,6,6,6,5,5,5,4,4,4,4,4,3,3,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
                         1,1,1,1,1,1,1,1,1,1,1,1,1,1};
*/
static int ticks1 = 10000;    // starting with overflow values
static int ticks2 = 10000;
boolean note1_on = false;     // no note is playing
boolean note2_on = false;
byte command;
byte pitch,actual_pitch1,actual_pitch2;
byte velocity;
byte channel;
byte actual_channel = 1;
int period1,period2;
byte t_on1,t_on2;



void setup() 
{

  pinMode(out_pin,OUTPUT);
  pinMode(busy_pin,OUTPUT);
  pinMode(enable_pin,INPUT);
  
  Serial.begin(31250);            // MIDI comunication is a 31250 serial link
  
  TCCR1A = 0;                     // set entire TCCR1A register to 0
  TCCR1B = 0;                     // same for TCCR1B
  TCNT1  = 0;                     //initialize counter value to 0
                                  // set compare match register for 7.5us time steps
  OCR1A = 120;                    // 120 sets time resolution to 7.5us (160 gives 10us ticks)
  TCCR1B |= (1 << WGM12);         // turn on CTC mode
  TCCR1B |= (1 << CS10);          // no prescaler. CS10 set to 1 to start running counter
  TIMSK1 |= (1 << OCIE1A);        // enable timer compare interrupt

}

void loop() 
{
 
 
  if (Serial.available())              // read a byte from serial (midi) 
  {
  int inByte = Serial.read();
  command = inByte / 16;               // command = most significative 4 bits
  channel = inByte % 16;               // channel = less significative 4 bits

  if( (digitalRead(enable_pin) == HIGH)  ||  ((command == 8) && (note1_on || note2_on)) )
 {   
 
  if(((command == 8) || (command == 9)) && (channel == actual_channel))    // command = noteON or noteOFF
   {
    while ((Serial.available() < 2)) {}
    pitch = Serial.read() - 12;            // read pitch and velocity values (velocity not used) and down 1 octave
    velocity = Serial.read();
   
   
    switch (command) 
    {
    case 8:                                         // command = noteOFF
      if((note1_on) && (actual_pitch1 == pitch))    // note1 stop
       {
        note1_on = false;
        PORTD = PORTD & B11111011;                  // no output
        ticks1 = 10000;                             // ticks overflowed
       }
      else 

      if((note2_on) && (actual_pitch2 == pitch))    // note2 stop
       {
        note2_on = false;
        PORTD = PORTD & B11111011;
        ticks2 = 10000;
       }
      break;
    case 9:
      if (!note1_on)                   // if note1 is not playing
       {
        period1 = period[pitch];       // Determinate period and T_on times from tables
        t_on1 = t_on[pitch];            
        actual_pitch1 = pitch;         // saves actual pitch of note which is playing on note1 "slot"
        note1_on = true;               // activate note1
       }
      else
      if (!note2_on)                  // same tasks for note2
       {
        period2 = period[pitch];
        t_on2 = t_on[pitch];
        actual_pitch2 = pitch; 
        note2_on = true;  
       }
      break;
    }

   }
   
 } 
  
 }

if(note1_on && note2_on)                     // set HIGH the busy flag (by direct asignation of PORTD value)
   PORTD = PORTD | B00001000;
 else
   PORTD = PORTD & B11110111;                // or not
 
if((!note1_on)&&(!note2_on))                 // no output if no note
   PORTD = PORTD & B11110011;

}

ISR(TIMER1_COMPA_vect)                             // Timer interrupt vector
{
 if(note1_on)                                      // note1 handling
 {
  if(ticks1>period1)                               // restart to 0 on ticks overflow 
  {
   ticks1 = 0;
   PORTD = PORTD | B00000100;                      // reset output, start ON state
  }
  if(ticks1==t_on1) PORTD = PORTD & B11111011;     // put down output after t_on
  ticks1++;
 }
   
 if(note2_on)                                      // note2 handling. Same treatment as note1
 {
  if(ticks2>period2)
  {
   ticks2 = 0;
   PORTD = PORTD | B00000100;                     // pulse ON
  }
  if(ticks2==t_on2) PORTD = PORTD & B11111011;    // pulse OFF
  ticks2++;
 }   
 
}
