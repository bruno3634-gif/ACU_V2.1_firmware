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


FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> CAN;


//VARIABLES
unsigned long HeartBit = 0;

float EBS_TANK_PRESSURE_A_values[PRESSURE_READINGS], EBS_TANK_PRESSURE_B_values[PRESSURE_READINGS];
float TANK_PRESSURE_B = 0, TANK_PRESSURE_A = 0; // Pressure values for tank A and B
uint8_t adc_pointer = 0; // Pointer for pressure readings
bool update_median_flag = false; // Flag to indicate if median update is needed

uint8_t ignition_flag = 0; // Flag to indicate ignition signal
uint8_t asms_flag = 0; // Current ignition signal state
uint8_t emergency_flag = 0; // Flag to indicate emergency state

uint8_t mission_response_jetson = 1; // Mission response from Jetson
uint8_t res_emergency = 0; // Emergency response from AS



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

void setup()
{

}

void loop()
{

  led_heartbit();

  if(update_median_flag) {
    median_pressures(); // Read pressure values
    update_median_flag = false; // Reset flag after reading
  }

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
      // reset all inital sequence variables


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
    // Handle EBS error state
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

    struct autonomous_temporary_acu_ign_t encoded_ign;    

    encoded_ign.ebs_pressure_rear = (uint8_t)(TANK_PRESSURE_B); // Convert to 0.1 bar scale
    encoded_ign.ebs_pressure_front = (uint8_t)(TANK_PRESSURE_A); // Convert to 0.1 bar scale
    encoded_ign.ign = ignition_flag; // Set ignition flag
    encoded_ign.asms = asms_flag; // Set ASMS flag
    encoded_ign.emergency = emergency_flag; // Set emergency flag

    autonomous_temporary_acu_ign_pack(tx_buffer, &encoded_ign, sizeof(encoded_ign));
    Serial.println("tx_buffer:");
    for (int i = 0; i < 8; i++) {
        Serial.print(tx_buffer[i], HEX);
        Serial.print(" ");
    }
}


void median_pressures() {

    // Calculate median for tank pressure B
    float sum = 0;
    for (int i = 0; i < PRESSURE_READINGS; i++) {
      sum += EBS_TANK_PRESSURE_B_values[i];
    }
    TANK_PRESSURE_B = sum / PRESSURE_READINGS;
    
    // Store raw voltage for debugging (convert ADC to voltage)
    float rawVoltage = TANK_PRESSURE_B * 3.3 / 1023; // Read raw voltage from the analog pin
    
    // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
    float actualVoltage = rawVoltage / 0.66;
    

    TANK_PRESSURE_B = (actualVoltage - 0.5) / 0.4;

    // tank pressure A

    sum = 0;
    for (int i = 0; i < PRESSURE_READINGS; i++) {
      sum += EBS_TANK_PRESSURE_A_values[i];
    }
    TANK_PRESSURE_A = sum / PRESSURE_READINGS;
    
    // Store raw voltage for debugging (convert ADC to voltage)
    rawVoltage = TANK_PRESSURE_A * 3.3 / 1023; // Read raw voltage from the analog pin
    
    // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
     actualVoltage = rawVoltage / 0.66;
    
    // Apply formula ONCE with corrected divider
    TANK_PRESSURE_A = (actualVoltage - 0.5) / 0.4;
  }



  void canISR(const CAN_message_t &msg){

    switch(msg.id) {
      case AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID:
        struct autonomous_temporary_jetson_ms_t decoded_jetson_ms_data;
        autonomous_temporary_jetson_ms_unpack(&decoded_jetson_ms_data,msg.buf,sizeof(decoded_jetson_ms_data));
        mission_response_jetson = decoded_jetson_ms_data.mission_select; // Update mission response from Jetson 
        break;
      case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
        struct autonomous_temporary_res_t decoded_res_data;
        autonomous_temporary_res_unpack(&decoded_res_data, msg.buf, sizeof(decoded_res_data));
        res_emergency = (decoded_res_data.signal != 0) ? 0 : 1; // Update emergency response from AS
        break;
      default:
        // Unknown message ID, ignore
        break;
    }
  }