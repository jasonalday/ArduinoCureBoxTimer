
#include <avr/pgmspace.h>

// SETUP:

#define RELAY_PIN 16

// Pins for Rotary Encoder
#define RE_CLK 0  // Input pin for Rotary Encoder CLK signal
#define RE_DT  1  // Input pin for Rotary Encoder DT signal
#define RE_SW  10  // Input pin for Rotary Encoder's swith
#define ENCODER_BUMP    5     // move it increments of 5 mins
#define ENCODER_MIN     ENCODER_BUMP     // min time in mins
#define ENCODER_MAX     240   // max time in mins
#define ENCODER_RESET   30    // default time in mins


enum ProgramState {
  TimerSetup   = 0,     // Setup
  TimerRunning = 1,     // Running
  TimerFinished = 2,    // Finished
  NumProgramStates = 3  // Count
};


// instantiate oled object for the SSD1306 OLED display

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif
#include <Fonts/ArialNarrow24_NUM.h> // custom font for remaining time

#define OLED_RESET 4
Adafruit_SSD1306 oled(OLED_RESET);

// variable for timer (in minutes) 
volatile int16_t timer_time = ENCODER_RESET;
// variable for the timer start time (device time, in ms)
uint32_t start_time;

 // current time
 volatile uint32_t current_time;
 volatile bool update_display_setup = false;
 volatile bool update_display_running = false;
 volatile bool update_display_finished = false;

volatile uint8_t min_left;
volatile uint8_t sec_left;

volatile ProgramState state = TimerSetup;

unsigned int lastReportedPos = 1;   // change management
static boolean rotating = false;    // debounce management

// interrupt service routine vars
boolean A_set = false;
boolean B_set = false;

void setup(){

  Serial.begin(9600);
  //while (!Serial) {}  // wait for serial to init

  Serial.println("Setup()");
  cli();//stop interrupts
  
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);


  
  pinMode(RELAY_PIN, OUTPUT); // high output lights the LED
  pinMode(RE_CLK, INPUT_PULLUP);  // sw1 is pulled-up and switched low
  pinMode(RE_DT, INPUT_PULLUP);  // sw2 is pulled-up and switched low

  // Setup pin change interrupts for rotary encoder's switch
  setupPinChangeInterrupt(RE_SW);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(RE_CLK), doEncoderCW, CHANGE);
  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(RE_DT), doEncoderCCW, CHANGE);

  sei();//allow interrupts
  
  // init the OLED display at I2C addr = 0x3C (defined by device)
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.setTextColor(WHITE);
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  //oled.display();
  //delay(2000);
  oled.clearDisplay();
  

   update_display_setup = true;
   updateDisplayTimerSetup();
   update_display_setup = true;
}    

// Setup Pin Change Interrupt
void setupPinChangeInterrupt(byte pin)
{
  pinMode(pin, INPUT_PULLUP);  // pull-up and switched low
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// Timer ISR
ISR(TIMER1_COMPA_vect)
{ 
  if (state == TimerRunning)
  {
    computeTimeLeft();
    update_display_running = true;
  }
}

volatile uint32_t progress_bar;


#define MAX_PROGRESS_COUNTER 10
#define PROGRESS_BAR_X_OFFSET 10
#define PROGRESS_BAR_Y_OFFSET 59
#define PROGRESS_BAR_WIDTH (128 - (PROGRESS_BAR_X_OFFSET * 2))
#define PROGRESS_BAR_HEIGHT 5

void computeTimeLeft()
{
  // get the current time
  uint32_t current_time = millis();
  // compute time left in milliseconds
  int32_t time_left = timer_time * 60000 - (current_time - start_time);
  if (time_left < 1) 
  { 
    time_left = 0;
    digitalWrite(RELAY_PIN, LOW);
    //Serial.println("TimerRunning -> TimerFinished"); 
    state = TimerFinished; // move to finished
    update_display_finished = true;
  }
  else
  {
    digitalWrite(RELAY_PIN, HIGH);
  }


  
  // convert time left to minutes and seconds for human readability
  min_left = time_left / 60000;
  sec_left = (time_left % 60000) / 1000;

  int32_t total_sec_left = (min_left * 60) + sec_left;
  int32_t total_timer_sec = timer_time * 60;
  int32_t numerator = (total_timer_sec - total_sec_left);
  int32_t denominator = total_timer_sec;  
  float progress_bar_float = ((float)numerator / (float)denominator) * PROGRESS_BAR_WIDTH;
  progress_bar = (uint32_t)progress_bar_float;
}

#define RE_DEBOUNCE_TIME 200
//// Pin change ISR for D0-D7
ISR (PCINT0_vect) // handle pin change interrupt for D0 to D7 here
{

  Serial.println("Got PCI ISR");  
  // Read the state of the clear button
  if (digitalRead(RE_SW) == LOW )  
  {
    //delay (1);
    static unsigned long last_interrupt_time = 0;
    unsigned long interrupt_time = millis();
    // If interrupts come faster than RE_DEBOUNCE_TIME, assume it's a bounce and ignore
    if (interrupt_time - last_interrupt_time < RE_DEBOUNCE_TIME) 
    {
      //Serial.println("Ignoring this interrupt");
      return;
    }
 
    // wait for it to go high (user could be holding it in)
    while (digitalRead(RE_SW) == LOW ) {}
    
    //Serial.println("Got PCI ISR");  
    switch (state)
    {
      case TimerRunning:
        timer_time = 0;
        computeTimeLeft();
        //Serial.println("TimerRunning -> TimerFinished"); 
        state = TimerFinished; // move to running
        update_display_finished = true;
        break;
 
      case TimerSetup:

        // Make sure are not at 0
        if (timer_time > 0)
        {
          // get the current time
          current_time = millis();
          start_time = current_time;  // save the new start time
          // Have a new time to display so update the display ASAP
          computeTimeLeft();
          //Serial.println("TimerSetup -> TimerRunning"); 
          state = TimerRunning; // move to running
          update_display_running = true;
        }
        break;
        
      case TimerFinished:
        timer_time = ENCODER_RESET;
        //Serial.println("TimerFinished -> TimerSetup"); 
        state = TimerSetup; // move back to setup
        update_display_setup = true;
        break;
    }

    last_interrupt_time = interrupt_time;
  }
}

#define OLED_X_OFFSET_1_DIGIT 27
#define OLED_X_OFFSET_2_DIGIT 17
#define OLED_X_OFFSET_3_DIGIT 6
#define OLED_Y_OFFSET 55

#define PUSH_COL_OFFSET 90
#define PUSH_ROW_OFFSET 0

#define STATE_COL_OFFSET 10
#define STATE_ROW_OFFSET 10

int8_t progress_counter = 0;
int8_t progress_counter_bump = 1;

void updateDisplayTimerRunning()
{
  if (update_display_running)
  {
    //Serial.println("updateDisplayTimerRunning");
    char msg_buf[32];
    // Udpdate the OLED
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setFont();
    oled.setCursor(PUSH_COL_OFFSET, PUSH_ROW_OFFSET);
    oled.println("\x1aStop");
    oled.setCursor(STATE_COL_OFFSET, STATE_ROW_OFFSET);
    oled.print("Curing");
     
    oled.drawRect(PROGRESS_BAR_X_OFFSET, PROGRESS_BAR_Y_OFFSET, PROGRESS_BAR_WIDTH, PROGRESS_BAR_HEIGHT, WHITE);
    oled.fillRect(PROGRESS_BAR_X_OFFSET, PROGRESS_BAR_Y_OFFSET, progress_bar, PROGRESS_BAR_HEIGHT, WHITE);
    
    oled.setFont(&Arial_Narrow_Bold24pt7b);
  
    int16_t x;
    int16_t y = OLED_Y_OFFSET;
  
     // Adjust X offset based on number of digits printing to give a nice centered look
    if (min_left < 10) // 1 digit
    {
      x = OLED_X_OFFSET_1_DIGIT;
    }
    else if (min_left < 100) // 2 digits
    {
      x = OLED_X_OFFSET_2_DIGIT;
    }
    else // 3 digits
    {
      x = OLED_X_OFFSET_3_DIGIT;
    }
    
    sprintf(msg_buf, "%1d:%02d", min_left, sec_left);
    oled.setCursor(x, y);
    oled.print(msg_buf);
    oled.display();
    update_display_running = false;
  }
}

void updateDisplayTimerSetup()
{

  if (update_display_setup)
  {
    //Serial.println("updateDisplayTimerSetup");
    
    char msg_buf[32];
    // Udpdate the OLED
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setFont();
    oled.setCursor(PUSH_COL_OFFSET, PUSH_ROW_OFFSET);
    oled.println("\x1aStart");
    oled.setCursor(STATE_COL_OFFSET, STATE_ROW_OFFSET);
    oled.println("Set time");
    oled.setFont(&Arial_Narrow_Bold24pt7b);
  
    int16_t x;
    int16_t y = OLED_Y_OFFSET;
  
    // convert time left to minutes and seconds for human readability
    uint8_t mins = timer_time;
    uint8_t secs = 0;
  
    //Serial.print("Timer_Time = ");
    //Serial.println(timer_time);
    
     // Adjust X offset based on number of digits printing to give a nice centered look
    if (mins < 10) // 1 digit
    {
      x = OLED_X_OFFSET_1_DIGIT;
    }
    else if (mins < 100) // 2 digits
    {
      x = OLED_X_OFFSET_2_DIGIT;
    }
    else // 3 digits
    {
      x = OLED_X_OFFSET_3_DIGIT;
    }
    
    sprintf(msg_buf, "%1d:%02d", mins, secs);
    oled.setCursor(x, y);
    oled.print(msg_buf);
    oled.display();
  
    update_display_setup = false;
  }
}

void updateDisplayTimerFinished()
{
  if (update_display_finished)
  {
    //Serial.println("updateDisplayTimerFinished");
    
    char msg_buf[32];
    // Udpdate the OLED
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setFont();
    oled.setCursor(PUSH_COL_OFFSET, PUSH_ROW_OFFSET);
    oled.println("\x1aReset");
    oled.setCursor(STATE_COL_OFFSET, STATE_ROW_OFFSET);
    oled.println("Finished");
    oled.setFont(&Arial_Narrow_Bold24pt7b);
    sprintf(msg_buf, "%1d:%02d", min_left, sec_left);
    oled.setCursor(OLED_X_OFFSET_1_DIGIT, OLED_Y_OFFSET);
    oled.print(msg_buf);
    
    oled.fillRect(PROGRESS_BAR_X_OFFSET, PROGRESS_BAR_Y_OFFSET, PROGRESS_BAR_WIDTH, PROGRESS_BAR_HEIGHT, WHITE);
    
    oled.display();
  
    update_display_finished = false;
  }
}


// Interrupt on A changing state
void doEncoderCW() 
{
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done

  // Test transition, did things really change?
  if ( digitalRead(RE_CLK) != A_set ) 
  { // debounce once more
    A_set = !A_set;

    // adjust counter + if A leads B
    if ( A_set && !B_set )
    {
      switch (state)
      {          
        case TimerSetup:
          timer_time += ENCODER_BUMP;
          if (timer_time > ENCODER_MAX)
          {
            timer_time = ENCODER_MAX;
          }
          update_display_setup = true;
          break;
        
        case TimerRunning:
        case TimerFinished:
          break;
      }
    }

    rotating = false;  // no more debouncing until loop() hits again
  }
}


// Interrupt on B changing state, same as A above
void doEncoderCCW() 
{
  
  if ( rotating ) delay (1);
  
  if ( digitalRead(RE_DT) != B_set ) 
  {
    B_set = !B_set;
    //  adjust counter - 1 if B leads A
    if ( B_set && !A_set )
    {
      switch (state)
      {
        case TimerSetup:
          timer_time -= ENCODER_BUMP;
          // Never go below min
          if (timer_time < ENCODER_MIN)
          {
            timer_time = ENCODER_MIN;
          }
          update_display_setup = true;
          break;
       
        case TimerRunning:
        case TimerFinished:
          break;
      }
    }

    rotating = false;
  }
}

void loop()
{
  rotating = true;  // reset the debouncer

  switch (state)
  {
    case TimerRunning:
      // If Timer ISR says it's time to update the display then do it
      if (update_display_running)
      {
        //Serial.println("loop TimerRunning");
        updateDisplayTimerRunning();
      }
      break;
    case TimerSetup:
      if (update_display_setup)
      {
        //Serial.println("TimerSetup");
        updateDisplayTimerSetup();
      }
      break;
    case TimerFinished:
      if (update_display_finished)
      {
        //Serial.println("TimerFinished");
        updateDisplayTimerFinished();
      }
      break;
  }    
} 

