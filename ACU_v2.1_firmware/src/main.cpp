#include <Arduino.h>
#include "definitions.h"
#define print_state 1

/* -------------------- STATE MACHINE DEFINITIONS -------------------- */
// VCU state enumeration
typedef enum
{
  STATE_INIT,                   
  STATE_INITIAL_SEQUENCE,
  STATE_READY,
  STATE_DRIVING,
  STATE_EBS_ERROR,      
  STATE_EMERGENCY            
} ACU_STATE_t;

typedef enum
{
  AS_STATE_OFF,
  AS_STATE_READY,                 
  AS_STATE_DRIVING,
  AS_STATE_EMERGENCY,
  AS_STATE_FINISHED          
} AS_STATE_t;


// State machine variables
ACU_STATE_t current_state = STATE_INIT;  // Current state of the VCU
ACU_STATE_t previous_state = STATE_INIT; // Previous state of the VCU

AS_STATE_t as_state = AS_STATE_OFF; // Autonomous system state

// State names for debug output
const char *state_names[] = {
    "STATE_INIT",
    "STATE_SHUTDOWN",
    "STATE_STANDBY",
    "STATE_PRECHARGE",
    "STATE_WAITING_FOR_R2D_MANUAL",
    "STATE_WAITING_FOR_R2D_AUTO",
    "STATE_READY_MANUAL",
    "STATE_READY_AUTONOMOUS",
    "STATE_AS_EMERGENCY"};



IntervalTimer PRESSURE_TIMER;

IntervalTimer CAN_TIMER;


//VARIABLES
unsigned long HeartBit = 0;





void UpdateState(void);
void HandleState(void);
void print_state_transition(ACU_STATE_t from, ACU_STATE_t to);

// Function prototypes
void led_heartbit();
void peripheral_init();
void Pressure_readings();
void send_can_msg();


void setup()
{

}

void loop()
{

  led_heartbit();

  UpdateState();

  /* Handle actions specific to the current state */
  HandleState();
}

/* -------------------- STATE MACHINE FUNCTIONS -------------------- */
/**
 * @brief Print state transition for debugging
 * @param from Previous state
 * @param to New state
 */
void print_state_transition(ACU_STATE_t from, ACU_STATE_t to)
{
  printf("\n\rState transition: %s -> %s\n", state_names[from], state_names[to]);
}

/**
 * @brief Update the state of the ACU based on inputs and conditions
 */
void UpdateState(void)
{
  // Store current state for change detection
  previous_state = current_state;

  // Process emergency condition with highest priority
  /*
    if (((as_system.state == 4) || (res.signal == 0)) && current_state != STATE_AS_EMERGENCY)
  {
    current_state = STATE_AS_EMERGENCY;
  }
  */


  // State transitions
  switch (current_state)
  {
  case STATE_INIT:
    current_state = STATE_INITIAL_SEQUENCE; 
    break;

  case STATE_INITIAL_SEQUENCE:

    break;

  case STATE_EBS_ERROR:

    break;

  case STATE_READY:

    break;

  case STATE_DRIVING:

    break;

  case STATE_EMERGENCY:

    break;
  }

  // Execute entry actions when state has changed
  if (current_state != previous_state)
  {
#ifdef print_state
    print_state_transition(previous_state, current_state);
#endif
    // State entry actions
    switch (current_state)
    {
    case STATE_INIT:
      break;
    case STATE_INITIAL_SEQUENCE:


      break;

    case STATE_EBS_ERROR:

      break;

    case STATE_READY:


      break;

    case STATE_DRIVING:


      break;

    case STATE_EMERGENCY:

      break;
    }
  }
}

/**
 * @brief Handle actions specific to the current state
 */
void HandleState(void)
{

  switch (current_state)
  {
  case STATE_INIT:
    peripheral_init();
    current_state = STATE_INITIAL_SEQUENCE; // Transition to initial sequence state
    break;
  case STATE_INITIAL_SEQUENCE:

    break;

  case STATE_EBS_ERROR:

    break;

  case STATE_READY:

    break;

  case STATE_DRIVING:

    break;

  case STATE_EMERGENCY:

    break;
  }
}




/**
 * @brief Initialize peripherals and pins
 */
void peripheral_init()
{
  pinMode(YELLOW_LEDS, OUTPUT);
  pinMode(BLUE_LEDS, OUTPUT);

  pinMode(MS_BUTTON1, INPUT_PULLUP);

  pinMode(MS_LED1, OUTPUT);
  pinMode(MS_LED2, OUTPUT);
  pinMode(MS_LED3, OUTPUT);
  pinMode(MS_LED4, OUTPUT);
  pinMode(MS_LED5, OUTPUT);
  pinMode(MS_LED6, OUTPUT);
  pinMode(MS_LED7, OUTPUT);
  digitalWrite(MS_LED1, 1);
  digitalWrite(MS_LED2, 1);
  digitalWrite(MS_LED3, 1);
  digitalWrite(MS_LED4, 1);
  digitalWrite(MS_LED5, 1);
  digitalWrite(MS_LED6, 1);
  digitalWrite(MS_LED7, 1);
  pinMode(AS_SW, INPUT);

  pinMode(HB_LED, OUTPUT);
  pinMode(Debug_LED2, OUTPUT);
  pinMode(Debug_LED3, OUTPUT);
  pinMode(Debug_LED4, OUTPUT);
  pinMode(Debug_LED5, OUTPUT);
  pinMode(Debug_LED6, OUTPUT);

  pinMode(EBS_TANK_PRESSURE_A, INPUT);
  pinMode(EBS_TANK_PRESSURE_B, INPUT);
  pinMode(EBS_VALLVE_A, INPUT);
  pinMode(EBS_VALLVE_B, INPUT);

  pinMode(R2D_PIN, INPUT);

  // pinMode(LED_PIN, OUTPUT);
  pinMode(WDT, OUTPUT);

  pinMode(ASMS, INPUT);
  pinMode(IGN_PIN, INPUT);

  pinMode(SOLENOID1, OUTPUT);
  pinMode(SOLENOID2, OUTPUT);

  digitalWrite(SOLENOID1, 1);
  digitalWrite(SOLENOID2, 1);

  Serial2.begin(115200);
  Serial.begin(115200);



  PRESSURE_TIMER.begin(Pressure_readings, 100000); // 100ms

  CAN_TIMER.begin(send_can_msg, 100000);      // 100ms
 
}

void led_heartbit() {
    if (HeartBit + 500 <= millis()) {
        digitalWrite(HB_LED, !digitalRead(HB_LED)); // Toggle LED state
        HeartBit = millis();
    }
}

/**
 * @brief Read pressure sensors and update system state
 */
void Pressure_readings() {
   
}


/**
 * @brief Send CAN messages based on system state
 */
void send_can_msg() {

}