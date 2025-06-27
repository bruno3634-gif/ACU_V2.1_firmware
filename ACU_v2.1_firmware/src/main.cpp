#include <Arduino.h>
#include "definitions.h"
#include "FlexCAN_T4_.h"
#include "IntervalTimer.h"
#include "autonomous_temporary.h"


#define print_state 1



#define PRESSURE_READINGS 8 // Number of pressure readings to average

/* -------------------- STATE MACHINE DEFINITIONS -------------------- */
// VCU state enumeration
typedef enum
{
  STATE_INIT, 
  STATE_MISSION_SELECCT,                  
  STATE_INITIAL_SEQUENCE,
  STATE_READY,
  STATE_DRIVING,
  STATE_EBS_ERROR,      
  STATE_EMERGENCY,   
  STATE_FINISHED         
} ACU_STATE_t;

typedef enum
{
  AS_STATE_OFF,
  AS_STATE_READY,                 
  AS_STATE_DRIVING,
  AS_STATE_EMERGENCY,
  AS_STATE_FINISHED          
} AS_STATE_t;


typedef enum{
  MANUAL,       // 0
  ACCELERATION, // 1
  SKIDPAD,      // 2
  AUTOCROSS,    // 3
  TRACKDRIVE,   // 4
  EBS_TEST,     // 5
  INSPECTION    // 6
}current_mission_t;



typedef enum
{
  WDT_TOOGLE_CHECK,
  WDT_STP_TOOGLE_CHECK,
  PNEUMATIC_CHECK,
  PRESSURE_CHECK1,
  IGNITON,
  PRESSURE_CHECK_FRONT,
  PRESSURE_CHECK_REAR,
  PRESSURE_CHECK2,
  ERROR
} INITIAL_SEQUENCE_STATE_t;


// State machine variables
ACU_STATE_t current_state = STATE_INIT;  // Current state of the VCU
ACU_STATE_t previous_state = STATE_INIT; // Previous state of the VCU

AS_STATE_t as_state = AS_STATE_OFF; // Autonomous system state

//INITIAL_SEQUENCE_STATE_t initial_sequence_state = WDT_TOOGLE_CHECK;
INITIAL_SEQUENCE_STATE_t initial_sequence_state = PNEUMATIC_CHECK;
current_mission_t current_mission = MANUAL; // Current mission state
current_mission_t jetson_mission = MANUAL; // Mission state from Jetson

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


FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> CAN;


//VARIABLES
unsigned long HeartBit = 0;

float EBS_TANK_PRESSURE_A_values[PRESSURE_READINGS], EBS_TANK_PRESSURE_B_values[PRESSURE_READINGS];
float TANK_PRESSURE_FRONT = 0, TANK_PRESSURE_REAR = 0; // Pressure values for tank A and B
float HYDRAULIC_PRESSURE_FRONT = 0, HYDRAULIC_PRESSURE_REAR = 0; // Pressure values for front and rear hydraulics

uint8_t adc_pointer = 0; // Pointer for pressure readings
bool update_median_flag = false; // Flag to indicate if median update is needed

uint8_t ignition_flag = 0; // Flag to indicate ignition signal
uint8_t ignition_vcu = 0; // Ignition signal state
uint8_t asms_flag = 0; // Current ignition signal state
uint8_t emergency_flag = 0; // Flag to indicate emergency state


uint8_t res_emergency = 0; // Emergency response from AS
volatile bool wdt_toogle_enable = true; // Flag to enable WDT toggle
unsigned long wdt_toogle_counter = 0; // Counter for WDT toggle
unsigned long wdt_relay_timout = 0; // Timeout for WDT relay
unsigned long pressure_check_delay = 0; // Delay for pressure check
uint8_t ignition_enable = 0; // Flag to enable ignition



unsigned long ASSI_YELLOW_time = 0, ASSI_BLUE_time = 0;


/***  For debounce */
static uint8_t last_ign_state = LOW;
static uint8_t debounced_ign_state = LOW;


void UpdateState(void);
void HandleState(void);
void print_state_transition(ACU_STATE_t from, ACU_STATE_t to);

// Function prototypes
void canISR(const CAN_message_t &msg);
void led_heartbit();
void peripheral_init();
void Pressure_readings();
void send_can_msg();
void median_pressures(); 
void initial_sequence();
void check_ignition();
void ASSI();
void led_heartbit();
void Mission_Indicator();


void setup()
{
  peripheral_init(); // Initialize peripherals and pins
}

void loop()
{


  UpdateState();

  /* Handle actions specific to the current state */
  HandleState();

  led_heartbit();

  Mission_Indicator();

  if(update_median_flag) {
    median_pressures(); // Read pressure values
    update_median_flag = false; // Reset flag after reading
  }

  ASSI(); // Update ASSI state

  if(wdt_toogle_enable) {
    if(millis() - wdt_toogle_counter >= 10) { // Check if 10 ms has passed
      digitalWrite(WDT, !digitalRead(WDT)); // Toggle WDT pin
      wdt_toogle_counter = millis(); // Reset counter
    }
  
  }
  check_ignition(); // Check ignition state

}

/* -------------------- STATE MACHINE FUNCTIONS -------------------- */
/**
 * @brief Print state transition for debugging
 * @param from Previous state
 * @param to New state
 */
void print_state_transition(ACU_STATE_t from, ACU_STATE_t to)
{
  Serial2.printf("\n\rState transition: %s -> %s\n", state_names[from], state_names[to]);
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
    as_state = AS_STATE_OFF; // Autonomous system state
    jetson_mission = MANUAL;
    break;
  case STATE_MISSION_SELECCT:
    break;
  case STATE_INITIAL_SEQUENCE:
    initial_sequence_state = WDT_TOOGLE_CHECK;
    break;
  case STATE_EBS_ERROR:
    break;

  case STATE_READY:
    as_state = AS_STATE_READY;
    break;

  case STATE_DRIVING:
    as_state = AS_STATE_DRIVING;
    break;

  case STATE_EMERGENCY:
    as_state = AS_STATE_EMERGENCY;
    break;
  case STATE_FINISHED:
    as_state = AS_STATE_FINISHED;
    // Handle finished state if needed
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
    case STATE_MISSION_SELECCT:
      break;
    case STATE_INITIAL_SEQUENCE:
      // reset all inital sequence variables
      break;

    case STATE_EBS_ERROR:
      digitalWrite(SOLENOID_REAR, LOW); // Deactivate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW);
      
      break;

    case STATE_READY:


      break;

    case STATE_DRIVING:
      digitalWrite(SOLENOID_REAR, HIGH); // Deactivate rear solenoid
      digitalWrite(SOLENOID_FRONT, HIGH); // Deactivate front solenoid
      
      break;

    case STATE_EMERGENCY:
      digitalWrite(SOLENOID_REAR, HIGH); // Activate rear solenoid
      digitalWrite(SOLENOID_FRONT, HIGH);
      break;
      case STATE_FINISHED:
      // Handle finished state actions if needed
      digitalWrite(SOLENOID_REAR, HIGH); // Activate rear solenoid
      digitalWrite(SOLENOID_FRONT, HIGH);
      break;
    }
  }
}

/**
 * @brief Handle actions specific to the current state
 */
void HandleState(void)
{

  if(res_emergency == 1){
    current_state = STATE_EMERGENCY;
  }

  switch (current_state)
  {
  case STATE_INIT:
    //peripheral_init();
      as_state = AS_STATE_OFF; // Autonomous system state
      ignition_enable = 0; // Reset ignition enable flag
      emergency_flag = 0; // Reset emergency flag
      ignition_flag = 0;
      asms_flag = 0;
      jetson_mission = MANUAL;
      res_emergency = 0;
      wdt_toogle_enable = true;

    
    current_state = STATE_INITIAL_SEQUENCE; // Transition to initial sequence state
    break;
  case STATE_MISSION_SELECCT:
    // Read mission select button with debounce while ASMS is off
    if (digitalRead(ASMS) == LOW) { // ASMS is off
      static uint8_t last_button_state = HIGH;
      static uint8_t debounced_button_state = HIGH;
      static unsigned long last_debounce_time = 0;
      const unsigned long debounce_delay = 30; // ms

      uint8_t current_button_state = digitalRead(MS_BUTTON1);

      if (current_button_state != last_button_state) {
        last_debounce_time = millis();
        last_button_state = current_button_state;
      }

      if ((millis() - last_debounce_time) > debounce_delay) {
        if (debounced_button_state != current_button_state) {
          debounced_button_state = current_button_state;
          if (debounced_button_state == LOW) {
            // Cycle to next mission
            current_mission = (current_mission_t)(((int)current_mission + 1) % 7);
          }
        }
      }
    }
    else{
      if(current_mission != MANUAL){
        current_state = STATE_INITIAL_SEQUENCE; // Transition to initial sequence state
      }
    }
    break;
  case STATE_INITIAL_SEQUENCE:
    initial_sequence(); // Execute initial sequence actions
    break;

  case STATE_EBS_ERROR:
    current_state = STATE_EMERGENCY;
    break;

  case STATE_READY:

    break;

  case STATE_DRIVING:

    break;

  case STATE_EMERGENCY:

    break;
    case STATE_FINISHED:
    // Handle finished state actions if needed
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

  pinMode(SOLENOID_FRONT, OUTPUT);
  pinMode(SOLENOID_REAR, OUTPUT);

  digitalWrite(SOLENOID_FRONT, 1);  // 1 equals braking on
  digitalWrite(SOLENOID_REAR, 1);

  Serial2.begin(115200);
  Serial.begin(115200);


  CAN.begin();
  CAN.setBaudRate(1000000); // Set CAN baud rate to 1 Mbps
  CAN.setMaxMB(16); // Set maximum number of mailboxes
  CAN.setMBFilter(REJECT_ALL);

  CAN.setMBFilter(MB0, AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID);
  CAN.setMBFilter(MB1, AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID);
  CAN.setMBFilter(MB2, AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID);
  CAN.setMBFilter(MB3, AUTONOMOUS_TEMPORARY_RES_FRAME_ID);

  CAN.onReceive(canISR);

  PRESSURE_TIMER.begin(Pressure_readings, 100000); // 100ms

  CAN_TIMER.begin(send_can_msg, 100000);      // 100ms

  Serial.println("Peripheral initialization complete");
  digitalWrite(Debug_LED2, 1); // Indicate initialization complete
 
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


  EBS_TANK_PRESSURE_A_values[adc_pointer] = analogRead(EBS_TANK_PRESSURE_A);
  EBS_TANK_PRESSURE_B_values[adc_pointer] = analogRead(EBS_TANK_PRESSURE_B);
  adc_pointer++;
  if (adc_pointer >= PRESSURE_READINGS) {
    adc_pointer = 0;
  }
  update_median_flag = true; // Set flag to update median values
}
   



/**
 * @brief Send CAN messages based on system state
 */
void send_can_msg() {

  uint8_t tx_buffer[8]; // Buffer for CAN message
    
    CAN_message_t tx_message;

    struct autonomous_temporary_acu_ign_t encoded_ign;    
    encoded_ign.ebs_pressure_rear = (uint8_t)(TANK_PRESSURE_FRONT ); // Convert to 0.1 bar scale
    encoded_ign.ebs_pressure_front = (uint8_t)(TANK_PRESSURE_REAR); // Convert to 0.1 bar scale
    encoded_ign.ign = ignition_flag; // Set ignition flag
    encoded_ign.asms = asms_flag; // Set ASMS flag
    encoded_ign.emergency = emergency_flag; // Set emergency flag

    autonomous_temporary_acu_ign_pack(tx_buffer, &encoded_ign, AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH);
    tx_message.id = AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID; // Set CAN ID
    tx_message.len = AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH; // Set message length
    memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH); // Copy data to CAN message buffer

    CAN.write(tx_message); // Send CAN message



    struct autonomous_temporary_acu_ms_t encoded_mission;
    encoded_mission.mission_select = (uint8_t)current_mission;
    autonomous_temporary_acu_ms_pack(tx_buffer, &encoded_mission, AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH);
    tx_message.id = AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID; // Set CAN ID
    tx_message.len = AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH; // Set message length
    memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH); // Copy data to CAN message buffer

    CAN.write(tx_message); // Send CAN message



    struct autonomous_temporary_rd_jetson_t RD_jetson_encode;
    RD_jetson_encode.rd = (as_state == AS_STATE_READY || as_state == AS_STATE_DRIVING) ? 1 : 0; // Set RD value based on current mission
    autonomous_temporary_rd_jetson_pack(tx_buffer, &RD_jetson_encode, AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH);
    tx_message.id = AUTONOMOUS_TEMPORARY_RD_JETSON_FRAME_ID;
    tx_message.len = AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH; //
    memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH); // Copy data to CAN message buffer

    CAN.write(tx_message); // Send CAN message
    
}


void median_pressures() {

    // Calculate median for tank pressure B
    float sum = 0;
    for (int i = 0; i < PRESSURE_READINGS; i++) {
      sum += EBS_TANK_PRESSURE_B_values[i];
    }
    TANK_PRESSURE_FRONT = sum / PRESSURE_READINGS;
    
    // Store raw voltage for debugging (convert ADC to voltage)
    float rawVoltage = TANK_PRESSURE_FRONT * 3.3 / 1023; // Read raw voltage from the analog pin
    
    // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
    float actualVoltage = rawVoltage / 0.66;
    

    TANK_PRESSURE_FRONT = (actualVoltage - 0.5) / 0.4;

    // tank pressure A

    sum = 0;
    for (int i = 0; i < PRESSURE_READINGS; i++) {
      sum += EBS_TANK_PRESSURE_A_values[i];
    }
    TANK_PRESSURE_REAR = sum / PRESSURE_READINGS;
    
    // Store raw voltage for debugging (convert ADC to voltage)
    rawVoltage = TANK_PRESSURE_REAR * 3.3 / 1023; // Read raw voltage from the analog pin
    
    // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
     actualVoltage = rawVoltage / 0.66;
    
    // Apply formula ONCE with corrected divider
    TANK_PRESSURE_REAR = (actualVoltage - 0.5) / 0.4;
  }



  void canISR(const CAN_message_t &msg){

    switch(msg.id) {
      case AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID:
        struct autonomous_temporary_jetson_ms_t decoded_jetson_ms_data;
        autonomous_temporary_jetson_ms_unpack(&decoded_jetson_ms_data,msg.buf,sizeof(decoded_jetson_ms_data));
        jetson_mission = (current_mission_t)decoded_jetson_ms_data.mission_select; // Update mission response from Jetson
        break;
      case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
        struct autonomous_temporary_res_t decoded_res_data;
        autonomous_temporary_res_unpack(&decoded_res_data, msg.buf, sizeof(decoded_res_data));
        res_emergency = (decoded_res_data.signal != 0) ? 0 : 1; // Update emergency response from AS
        break;
      case AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID:
        struct autonomous_temporary_vcu_hv_t decoded_vcu_hv_data;
        autonomous_temporary_vcu_hv_unpack(&decoded_vcu_hv_data, msg.buf, sizeof(decoded_vcu_hv_data));
        ignition_vcu = (decoded_vcu_hv_data.hv == AUTONOMOUS_TEMPORARY_VCU_HV_HV_HV_ON_CHOICE) ? 1 : 0; // Update ignition signal
        HYDRAULIC_PRESSURE_FRONT = decoded_vcu_hv_data.brake_pressure_front / 10.0; // Convert to bar
        HYDRAULIC_PRESSURE_REAR = decoded_vcu_hv_data.brake_pressure_rear / 10.0;
        break;
        case AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID:
        struct autonomous_temporary_as_state_t decoded_as_state_data;
        autonomous_temporary_as_state_unpack(&decoded_as_state_data, msg.buf, sizeof(decoded_as_state_data));
        as_state = (AS_STATE_t)decoded_as_state_data.state; // Update autonomous system state

      default:
        // Unknown message ID, ignore
        break;
    }
  }




  void initial_sequence() {
    switch (initial_sequence_state)
    {
    case WDT_TOOGLE_CHECK:
      if(digitalRead(WDT) == HIGH) {
        initial_sequence_state = WDT_STP_TOOGLE_CHECK;
        wdt_toogle_enable = false; // Disable WDT toggle for initial sequence
        wdt_relay_timout = millis(); // Reset WDT toggle counter
      }
      break;
    case WDT_STP_TOOGLE_CHECK:
      if(digitalRead(WDT) == LOW) {
        initial_sequence_state = PNEUMATIC_CHECK; 
        wdt_toogle_enable = true; 
      }else{
        if(millis() - wdt_relay_timout >= 500) { 
          initial_sequence_state = ERROR;
        }
      }
      break;
    case PNEUMATIC_CHECK:
      if (/*TANK_PRESSURE_REAR > 6.0 &&*/ TANK_PRESSURE_FRONT > 6.0)
      {
        if (/*TANK_PRESSURE_REAR < 10.0 &&*/ TANK_PRESSURE_FRONT < 10.0)
        {
          initial_sequence_state = PRESSURE_CHECK1; // Transition to pressure check state
        }else
        {
          initial_sequence_state = ERROR; // Transition to pressure check state
        }
      }else
      {
        initial_sequence_state = ERROR; // Transition to pressure check state
      }
      break;
    case PRESSURE_CHECK1: 
       if (HYDRAULIC_PRESSURE_FRONT >= 11.5 * TANK_PRESSURE_FRONT /*&& HYDRAULIC_PRESSURE_REAR >= 11.5 * TANK_PRESSURE_REAR*/)
       {
        initial_sequence_state = IGNITON; 
       }else{
        initial_sequence_state = ERROR;
       }
       
      break;
    case IGNITON:
        ignition_enable = 1; // Enable ignition
        if(ignition_vcu == 1 && ignition_flag == 1) {
          initial_sequence_state = PRESSURE_CHECK_FRONT; // Transition to pressure check state
          pressure_check_delay = millis(); // Reset pressure check delay
        }
      break;
    case PRESSURE_CHECK_REAR:
      digitalWrite(SOLENOID_REAR, LOW); // Deactivate rear solenoid
      digitalWrite(SOLENOID_FRONT, HIGH); // Activate front solenoid
      if(/*TANK_PRESSURE_REAR >= 11.5 * HYDRAULIC_PRESSURE_REAR &&*/ TANK_PRESSURE_FRONT <= 1) {
        initial_sequence_state = PRESSURE_CHECK2; 
        pressure_check_delay = millis(); // Reset pressure check delay
      }
      if(millis() - pressure_check_delay >= 500) { // Check if 500 ms has passed
        initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
      }
      break;
    case PRESSURE_CHECK_FRONT:
      digitalWrite(SOLENOID_REAR, HIGH); // Deactivate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid
      if(TANK_PRESSURE_FRONT > 11.5 * HYDRAULIC_PRESSURE_FRONT && TANK_PRESSURE_REAR <= 1) {
        //initial_sequence_state = PRESSURE_CHECK_REAR; 
        initial_sequence_state = PRESSURE_CHECK2; // Transition to pressure check state
        pressure_check_delay = millis(); // Reset pressure check delay
      }
      if(millis() - pressure_check_delay >= 500) { // Check if 500 ms has passed
        initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
      }
      break;
    case PRESSURE_CHECK2:
      digitalWrite(SOLENOID_REAR, LOW); // Deactivate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW); // Deactivate front solenoid
      if(/*TANK_PRESSURE_REAR >= 11.5 * HYDRAULIC_PRESSURE_REAR &&*/ TANK_PRESSURE_FRONT >= 11.5 * HYDRAULIC_PRESSURE_FRONT) {
        current_state = STATE_READY; // Transition to ready state
      }
      if(millis() - pressure_check_delay >= 1000) { // Check if 1000 ms has passed
        initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
      }
      break;
    case ERROR:
      current_state = STATE_EBS_ERROR; // Transition to EBS error state

    break;
    default:
      Serial.println("Unknown initial sequence state");
      break;
    }
  }



void check_ignition() {
  static unsigned long last_debounce_time = 0;
  const unsigned long debounce_delay = 30; // 30 ms debounce

  uint8_t current_state = digitalRead(IGN_PIN);

  if (current_state != last_ign_state) {
    last_debounce_time = millis();
    last_ign_state = current_state;
  }

  if ((millis() - last_debounce_time) > debounce_delay) {
    if (debounced_ign_state != current_state) {
      debounced_ign_state = current_state;
      ignition_flag = (debounced_ign_state == HIGH) ? 1 : 0;
    }
  }
  ignition_flag = ignition_flag && ignition_enable; // Ensure ignition flag is set only if ignition is enabled
}




void ASSI()
{

  switch (as_state)
  {
  case AS_STATE_OFF:
    digitalWrite(YELLOW_LEDS, HIGH);
    digitalWrite(BLUE_LEDS, LOW);

    break;
  case AS_STATE_READY:

    digitalWrite(YELLOW_LEDS, HIGH);
    digitalWrite(BLUE_LEDS, LOW);
    break;
  case AS_STATE_DRIVING:
    digitalWrite(BLUE_LEDS, LOW);
    if (millis() - ASSI_YELLOW_time >= 500)
    {
      ASSI_YELLOW_time = millis();
      digitalWrite(YELLOW_LEDS, !digitalRead(YELLOW_LEDS));
    }
    break;
  case AS_STATE_EMERGENCY:
    digitalWrite(YELLOW_LEDS, LOW);
    if (millis() - ASSI_BLUE_time >= 500)
    {
      ASSI_BLUE_time = millis();
      digitalWrite(BLUE_LEDS, !digitalRead(BLUE_LEDS));
    }
    break;
  case AS_STATE_FINISHED:
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, HIGH);
    break;

  default:
    digitalWrite(YELLOW_LEDS, LOW);
    digitalWrite(BLUE_LEDS, LOW);

    break;
  }


}


void Mission_Indicator() {
  switch (current_mission)
  {
  case MANUAL:
    digitalWrite(MS_LED1, 0);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case ACCELERATION:
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 0);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case SKIDPAD:
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 0);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case TRACKDRIVE:
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 0);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case EBS_TEST:
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 0);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  case INSPECTION:
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 0);
    digitalWrite(MS_LED7, 1);
    break;
  case AUTOCROSS:
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 0);
    break;
  default:
    digitalWrite(MS_LED1, 1);
    digitalWrite(MS_LED2, 1);
    digitalWrite(MS_LED3, 1);
    digitalWrite(MS_LED4, 1);
    digitalWrite(MS_LED5, 1);
    digitalWrite(MS_LED6, 1);
    digitalWrite(MS_LED7, 1);
    break;
  }
}
