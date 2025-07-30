/**
 * @file main.cpp
 * @brief Main firmware file for the Actuation Control Unit (ACU) V2.1.
 *
 * This file implements the main logic, state machine, and peripheral handling for the ACU,
 * which manages the actuation and safety logic for an autonomous vehicle.
 * The ACU interfaces with pressure sensors, solenoids, CAN bus, and various status indicators.
 *
 * Key features:
 * - Implements a robust state machine for ACU operation, including initialization, mission selection,
 *   initial safety checks, ready, driving, emergency, and finished states.
 * - Handles CAN communication for receiving commands and sending status updates.
 * - Reads and processes pressure sensor data with averaging and conversion to engineering units.
 * - Manages ignition and emergency logic with debounce and safety checks.
 * - Controls visual indicators (LEDs) for system and mission status.
 * - Provides detailed documentation for each function and state.
 *
 * @note Key global variables used throughout this file include:
 *   - current_state, previous_state: ACU state machine tracking.
 *   - as_state: Autonomous system state.
 *   - initial_sequence_state: State for initial safety sequence.
 *   - current_mission, jetson_mission: Mission selection tracking.
 *   - EBS_TANK_PRESSURE_A_values, EBS_TANK_PRESSURE_B_values: Pressure sensor readings.
 *   - TANK_PRESSURE_FRONT, TANK_PRESSURE_REAR: Calculated tank pressures.
 *   - HYDRAULIC_PRESSURE_FRONT, HYDRAULIC_PRESSURE_REAR: Hydraulic pressures.
 *   - ignition_flag, ignition_vcu, ignition_enable: Ignition logic.
 *   - asms_flag: Autonomous system master switch state.
 *   - emergency_flag, res_emergency, res_active: Emergency logic.
 *   - wdt_togle_enable, wdt_togle_counter, wdt_relay_timout: Watchdog timer logic.
 *   - pressure_check_delay: Timing for pressure checks.
 *   - emergency_timestamp: Timing for emergency state.
 *   - ASSI_YELLOW_time, ASSI_BLUE_time: Timing for LED indicators.
 *   - last_button_time_ms: Debounce for mission selection button.
 *   - last_ign_state, debounced_ign_state: Debounce for ignition input.
 *
 * @author (Bruno Vicente - LART)
 * @date (2025)
 */
#include <Arduino.h>
#include "definitions.h"
#include "FlexCAN_T4_.h"
#include "IntervalTimer.h"
#include "autonomous_temporary.h"

#define print_state 1

#define PRESSURE_READINGS 8 // Number of pressure readings to average

/* -------------------- STATE MACHINE DEFINITIONS -------------------- */

/**
 * @enum ACU_STATE_t
 * @brief Represents the various operational states of the ACU (Actuation Control Unit).
 *
 * This enumeration defines the possible states in which the ACU can exist during its lifecycle.
 * Each state corresponds to a specific phase or condition of the system.
 *
 * @var STATE_INIT
 *      The initial state after power-up or reset, where system initialization occurs.
 * @var STATE_MISSION_SELECT
 *      State where the mission or operational mode is selected.
 * @var STATE_INITIAL_SEQUENCE
 *      State for executing the initial sequence before becoming ready.
 * @var STATE_READY
 *      System is ready and awaiting further commands or actions.
 * @var STATE_DRIVING
 *      The ACU is actively controlling the vehicle or system in its driving mode.
 * @var STATE_EBS_ERROR
 *      An error has occurred in the Emergency Braking System (EBS).
 * @var STATE_EMERGENCY
 *      The system has entered an emergency state, requiring immediate attention.
 * @var STATE_FINISHED
 *      The mission or operation has completed, and the system is in a finished state.
 */
typedef enum
{
  STATE_INIT,
  STATE_MISSION_SELECT,
  STATE_JETSONWAITING,
  STATE_INITIAL_SEQUENCE,
  STATE_READY,
  STATE_DRIVING,
  STATE_EBS_ERROR,
  STATE_EMERGENCY,
  STATE_FINISHED,
  STATE_MANUAL,
} ACU_STATE_t;

// State names for debug output
const char *state_names[] = {
    "STATE_INIT",
    "STATE_Mission_Select",
    "STATE_JETSONWAITING",
    "STATE_INITIAL_SEQUENCE",
    "STATE_READY",
    "STATE_DRIVING",
    "STATE_EBS_ERROR",
    "STATE_EMERGENCY",
    "STATE_FINISHED",
    "STATE_MANUAL"};

/**
 * @enum AS_STATE_t
 * @brief Represents the different states of the Autonomous System (AS).
 * This enumeration defines the various operational states of the AS,
 * which can be used to manage the system's behavior during autonomous operations.
 * @var AS_STATE_OFF
 * ASSI OFF
 * @var AS_STATE_READY
 *    ASSI yellow
 *  @var AS_STATE_DRIVING
 *   ASSI Blinking yellow
 * @var AS_STATE_EMERGENCY
 *  ASSI blinking Blue
 * @var AS_STATE_FINISHED
 * ASSI Blue
 */
typedef enum
{
  AS_STATE_OFF = 1,       // 0
  AS_STATE_READY = 2,     // 1
  AS_STATE_DRIVING = 3,   // 2
  AS_STATE_EMERGENCY = 4, // 3
  AS_STATE_FINISHED = 5   // 4
} AS_STATE_t;

typedef enum
{
  MANUAL,       // 0
  ACCELERATION, // 1
  SKIDPAD,      // 2   // 3
  TRACKDRIVE,   // 4
  EBS_TEST,     // 5
  INSPECTION,   // 6
  AUTOCROSS     // 7
} current_mission_t;

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
volatile ACU_STATE_t current_state = STATE_INIT;  // Current state of the ACU
volatile ACU_STATE_t previous_state = STATE_INIT; // Previous state of the ACU

volatile AS_STATE_t as_state = AS_STATE_OFF; // Autonomous system state

// INITIAL_SEQUENCE_STATE_t initial_sequence_state = WDT_TOOGLE_CHECK;
// INITIAL_SEQUENCE_STATE_t initial_sequence_state = PNEUMATIC_CHECK;
INITIAL_SEQUENCE_STATE_t initial_sequence_state = WDT_TOOGLE_CHECK;
current_mission_t current_mission = MANUAL; // Current mission state
current_mission_t jetson_mission = MANUAL;  // Mission state from Jetson

IntervalTimer PRESSURE_TIMER;

IntervalTimer CAN_TIMER;

IntervalTimer HANDBOOK_MESSAGE_TIMER;

FlexCAN_T4<CAN2, RX_SIZE_1024, TX_SIZE_1024> CAN;

// VARIABLES
unsigned long HeartBit = 0;

/**
 * @brief Array storing recent pressure readings from EBS Tank A.
 *
 * This array holds the last PRESSURE_READINGS number of float values,
 * representing the sampled pressure values from the EBS (Emergency Braking System) Tank A.
 * Updated in Pressure_readings(), used in median_pressures() for filtering/averaging.
 *
 * @see EBS_TANK_PRESSURE_B_values
 * @see PRESSURE_READINGS
 */
float EBS_TANK_PRESSURE_A_values[PRESSURE_READINGS];

/**
 * @brief Array storing recent pressure readings from EBS Tank B.
 *
 * This array holds the last PRESSURE_READINGS number of float values,
 * representing the sampled pressure values from the EBS (Emergency Braking System) Tank B.
 * Updated in Pressure_readings(), used in median_pressures() for filtering/averaging.
 *
 * @see EBS_TANK_PRESSURE_A_values
 * @see PRESSURE_READINGS
 */
float EBS_TANK_PRESSURE_B_values[PRESSURE_READINGS];

/**
 * @brief Pressure value for front tank (in bar).
 *
 * Calculated in median_pressures() from EBS_TANK_PRESSURE_B_values.
 * Used in initial_sequence(), HandleState(), send_can_msg(), and for state transitions.
 */
float TANK_PRESSURE_FRONT = 0;

/**
 * @brief Pressure value for rear tank (in bar).
 *
 * Calculated in median_pressures() from EBS_TANK_PRESSURE_A_values.
 * Used in initial_sequence(), HandleState(), send_can_msg(), and for state transitions.
 */
float TANK_PRESSURE_REAR = 0;

/**
 * @brief Hydraulic pressure value for front brakes (in bar).
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID.
 * Used in initial_sequence() for pressure checks.
 */
float HYDRAULIC_PRESSURE_FRONT = 0;

/**
 * @brief Hydraulic pressure value for rear brakes (in bar).
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID.
 * Used in initial_sequence() for pressure checks.
 */
float HYDRAULIC_PRESSURE_REAR = 0;

/**
 * @brief Pointer for pressure readings buffer.
 *
 * Used in Pressure_readings() to cycle through EBS_TANK_PRESSURE_A_values and EBS_TANK_PRESSURE_B_values.
 */
uint8_t adc_pointer = 0;

/**
 * @brief Flag to indicate if median pressure update is needed.
 *
 * Set in Pressure_readings(), checked in loop() to call median_pressures().
 */
volatile bool update_median_flag = false;

/**
 * @brief Flag to indicate ignition signal.
 *
 * Updated in check_ignition(), used in send_can_msg(), HandleState(), and initial_sequence().
 */
uint8_t ignition_flag = 0;

/**
 * @brief Ignition signal state from VCU.
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID.
 * Used in initial_sequence().
 */
uint8_t ignition_vcu = 0;

/**
 * @brief Current ASMS (Autonomous System Master Switch) signal state.
 *
 * Updated in loop() from digitalRead(ASMS), used in send_can_msg().
 */
uint8_t asms_flag = 0;

/**
 * @brief Flag to indicate emergency state.
 *
 * Set in HandleState() and UpdateState(), used in send_can_msg().
 */
uint8_t emergency_flag = 0;

/**
 * @brief Emergency response from AS (Autonomous System).
 *
 * Updated in canISR() from CAN message AUTONOMOUS_TEMPORARY_RES_FRAME_ID.
 * Used in HandleState() and UpdateState().
 */
volatile uint8_t res_emergency = 0;

/**
 * @brief Flag to enable WDT (Watchdog Timer) toggle.
 *
 * Used in loop() and initial_sequence() to control WDT toggling.
 */
volatile bool wdt_togle_enable = true;

/**
 * @brief Counter for WDT toggle timing.
 *
 * Used in loop() and initial_sequence() to measure elapsed time for WDT toggling.
 */
unsigned long wdt_togle_counter = 0;

/**
 * @brief Timeout for WDT relay check.
 *
 * Used in initial_sequence() for timing WDT relay state.
 */
unsigned long wdt_relay_timout = 0;

/**
 * @brief Delay for pressure check timing.
 *
 * Used in initial_sequence() to measure elapsed time for pressure checks.
 */
unsigned long pressure_check_delay = 0;

/**
 * @brief Flag to enable ignition logic.
 *
 * Set in initial_sequence(), used in check_ignition().
 */
uint8_t ignition_enable = 0;

/**
 * @brief Flag to indicate if response from AS is active.
 *
 * Updated in canISR(), used in HandleState() and mission selection logic.
 */
volatile bool res_active = false;

/**
 * @brief Timestamp for entering emergency state.
 *
 * Set in UpdateState() when entering STATE_EMERGENCY, used in HandleState() for timeout.
 */
unsigned long emergency_timestamp = 0;

/**
 * @brief Timestamp for ASSI yellow LED blinking.
 *
 * Used in ASSI() for timing yellow LED blinking in AS_STATE_DRIVING.
 */
unsigned long ASSI_YELLOW_time = 0;

/**
 * @brief Timestamp for ASSI blue LED blinking.
 *
 * Used in ASSI() for timing blue LED blinking in AS_STATE_EMERGENCY.
 */
unsigned long ASSI_BLUE_time = 0;

/**
 * @brief Timestamp for mission LED blinking.
 *
 * Used in Mission_Indicator() for timing mission LED blinking at 1 Hz when state >= STATE_INITIAL_SEQUENCE.
 */
unsigned long mission_LED_time = 0;

/**
 * @brief Current state of mission LED for blinking.
 *
 * Used in Mission_Indicator() to track whether the LED should be ON or OFF during blinking.
 */
bool mission_LED_state = true;

/**
 * @brief Timestamp for initialization delay.
 *
 * Used in HandleState() for implementing 1-second delay before transitioning from STATE_INIT to STATE_MISSION_SELECT.
 */
unsigned long init_delay_time = 0;

/**
 * @brief Last button press time in milliseconds.
 *
 * Used in HandleState() for mission selection button debounce.
 */
unsigned long last_button_time_ms = 0;

/**
 * @brief Last raw state read from the ignition pin.
 *
 * Used in check_ignition() for debounce logic.
 */
static uint8_t last_ign_state = LOW;

/**
 * @brief Last debounced state of the ignition pin.
 *
 * Used in check_ignition() for debounce logic.
 */
static uint8_t debounced_ign_state = LOW;

int speed = 0;

/**
 * @brief Handbook variables that are sent to can bus -> DV driving dynamics 1
 * @showrefs FSG Handbook 2025  page 20
 * @param speed_actual Current speed of the vehicle 0.5 scale
 * @param speed_target Target speed of the vehicle 0.5 scale
 * @param steering_angle_actual Current steering angle of the vehicle 0.5 scale
 * @param steering_angle_target Target steering angle of the vehicle 0.5 scale
 * @param brake_hydr_actual Current hydraulic brake pressure 0.5 scale
 * @param brake_hydr_target Target hydraulic brake pressure 0.5 scale
 * @param motor_moment_actual Current motor moment 0.5 scale
 * @param motor_moment_target Target motor moment 0.5 scale
 * @note 0,5 scale
 * @note ID 0x500
 */

uint8_t Speed_actual = 0;
uint8_t Speed_target = 0;
int8_t Steering_angle_actual = 0;
int8_t Steering_angle_target = 0;
uint8_t Brake_hydr_actual = 0;
uint8_t Brake_hydr_target = 0;
uint8_t Motor_moment_actual = 0;
uint8_t Motor_moment_target = 0;

float wheel_speed_fl = 0;
float wheel_speed_fr = 0;
float wheel_speed_rl = 0;
float wheel_speed_rr = 0;

/**
 * @brief Handbook variables that are sent to can bus -> DV system status
 * @showrefs FSG Handbook 2025  page 20
 * @param as_status Autonomous system status (3 bits) -> @see AS_STATE_t
 * @param EBS_status Emergency Braking System status (2 bits)
 * @param AMI_status Autonomous Mission Indicator status (3 bits) -> @see current_mission
 * @param Steering_state Current steering state (1 bit) -> true if steering is engaged
 * @param ASB_redundancy Autonomous System Backup status (2 bits)
 * @param Lap_counter Current lap counter value (4 bits)
 * @param cones_count_actual Current count of cones detected 8 bits)
 * @param cones_count Total count of cones detected (16 bits)
 *
 * @note ID 0x501
 */

uint8_t as_status = 1;
uint8_t EBS_status = 0;
uint8_t AMI_status = 0;
bool Steering_state = false;
uint8_t ASB_redundancy = 0;
uint8_t Lap_counter = 0;
uint8_t cones_count_actual = 0;
uint16_t cones_count = 0;

uint8_t Brake_pressure_front = 0;
int8_t Brake_pressure_rear = 0;

int16_t dynamics_steering_angle = 0;
int16_t rpm_vcu = 0;

volatile int IGN_manual = 0; // Manual ignition control from Jetson

unsigned long RES_timeout = 0;
unsigned long JETSON_timeout = 0; // Last time a CAN message was received
unsigned long VCU_timeout = 0;    // Last time a CAN message was received
unsigned long MAXON_timeout = 0;  // Last time a CAN message was received

volatile int jetson_ready = 0;
volatile int sdc_signal = 1;

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
void continuous_monitoring();

void send_handbook_variables();

void setup()
{
  peripheral_init(); // Initialize peripherals and pins
  // delay(3000);
}

void loop()
{

  UpdateState();

  /* Handle actions specific to the current state */
  HandleState();

  led_heartbit();

  Mission_Indicator();

  if (update_median_flag)
  {
    median_pressures();         // Read pressure values
    update_median_flag = false; // Reset flag after reading
  }
  asms_flag = digitalRead(ASMS); // Read ASMS state
  ASSI();                        // Update ASSI state

  if (wdt_togle_enable)
  {
    if (millis() - wdt_togle_counter >= 10)
    {                                       // Check if 10 ms has passed
      digitalWrite(WDT, !digitalRead(WDT)); // Toggle WDT pin
      wdt_togle_counter = millis();         // Reset counter
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
  Serial.println("\n\rState transition: " + String(state_names[from]) + " -> " + String(state_names[to]));
}

/**
 * @brief Update the state of the ACU based on inputs and conditions
 */
void UpdateState(void)
{

  // State transitions
  switch (current_state)
  {
  case STATE_INIT:
    as_state = AS_STATE_OFF; // Autonomous system state
    jetson_mission = MANUAL;
    break;

  case STATE_MISSION_SELECT:
    as_state = AS_STATE_OFF; // Autonomous system state
    break;

  case STATE_JETSONWAITING:
    as_state = AS_STATE_OFF; // Autonomous system state
    break;

  case STATE_INITIAL_SEQUENCE:
    break;

  case STATE_EBS_ERROR:
    as_state = AS_STATE_EMERGENCY; // Autonomous system state
    break;

  case STATE_READY:

    break;
  case STATE_DRIVING:
    as_state = AS_STATE_DRIVING;        // Autonomous system state
    digitalWrite(SOLENOID_REAR, HIGH);  // Activate rear solenoid
    digitalWrite(SOLENOID_FRONT, HIGH); // Activate front solenoid
    break;

  case STATE_EMERGENCY:
    emergency_flag = 1; // Set emergency flag
    as_state = AS_STATE_EMERGENCY;
    digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
    digitalWrite(SOLENOID_FRONT, LOW);
    break;

  case STATE_FINISHED:
    // as_state = AS_STATE_FINISHED; // Autonomous system state
    // Handle finished state if needed
    break;

  case STATE_MANUAL:
    as_state = AS_STATE_OFF;  // Manual state uses OFF autonomous system state
    current_mission = MANUAL; // Set current mission to MANUAL
    // Keep brakes applied in manual mode
    digitalWrite(SOLENOID_REAR, HIGH);
    digitalWrite(SOLENOID_FRONT, HIGH);
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
      as_state = AS_STATE_OFF; // Autonomous system state
      wdt_togle_enable = true; // Enable WDT toggle
      break;

    case STATE_MISSION_SELECT:
      as_state = AS_STATE_OFF; // Autonomous system state
      wdt_togle_enable = true; // Enable WDT toggle
      break;

    case STATE_JETSONWAITING:
      as_state = AS_STATE_OFF; // Autonomous system state
      Serial.println("Waiting for Jetson response (AS_STATE_OFF)");
      break;

    case STATE_INITIAL_SEQUENCE:
      // initial_sequence_state = WDT_TOOGLE_CHECK;
      //  reset all inital sequence variables
      break;

    case STATE_EBS_ERROR:
      as_state = AS_STATE_EMERGENCY;    // Autonomous system state
      digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW);
      break;

    case STATE_READY:
      // as_state = AS_STATE_READY;
      /// digitalWrite(SOLENOID_REAR, LOW);  // Activate rear solenoid
      // digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid
      break;

    case STATE_DRIVING:
      as_state = AS_STATE_DRIVING;
      // digitalWrite(SOLENOID_REAR, HIGH);  // Deactivate rear solenoid
      // digitalWrite(SOLENOID_FRONT, HIGH); // Deactivate front solenoid
      break;

    case STATE_EMERGENCY:
      ignition_enable = 0;
      as_state = AS_STATE_EMERGENCY;
      digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
      digitalWrite(SOLENOID_FRONT, LOW);
      emergency_timestamp = millis(); // Record the time of entering emergency state
      break;

    case STATE_FINISHED:
      // Handle finished state actions if needed
      break;

    case STATE_MANUAL:
      as_state = AS_STATE_OFF;           // Set autonomous system state to OFF
      digitalWrite(SOLENOID_REAR, LOW);  // Apply brakes
      digitalWrite(SOLENOID_FRONT, LOW); // Apply brakes
      Serial.println("Entered MANUAL state via CAN ignition signal");
      break;
    }

    // Store current state for change detection
    previous_state = current_state;
  }
}

/**
 * @brief Handle actions specific to the current state
 */
void HandleState(void)
{

  /*if (asms_flag == LOW && current_state > STATE_MISSION_SELECT && current_state != STATE_EMERGENCY)
  {
    ignition_enable = 0;                  // Reset ignition enable flag
    current_state = STATE_MISSION_SELECT; // Transition to mission select state
  }*/

  if (asms_flag == LOW && current_state > STATE_MISSION_SELECT && current_state != STATE_MANUAL)
  {
    ignition_enable = 0;                  // Reset ignition enable flag
    current_state = STATE_MISSION_SELECT; // Transition to mission select state
    jetson_ready = 0;                     // Reset jetson ready flag
  }

  if (res_emergency == 1)
  {
    current_state = STATE_EMERGENCY;
    as_state = AS_STATE_EMERGENCY; // Autonomous system state
    return;
  }
  if (initial_sequence_state != WDT_STP_TOOGLE_CHECK && current_state >= STATE_INITIAL_SEQUENCE && current_state != STATE_MANUAL)
  {
    if (sdc_signal == 0)
    {
      current_state = STATE_EMERGENCY;
      emergency_flag = 1; // Set emergency flag
    }
  }

  if (current_state > STATE_INITIAL_SEQUENCE && current_state < STATE_EMERGENCY && current_state != STATE_MANUAL)
  {
    if (digitalRead(ASMS) == LOW && ignition_enable == 0)
    {
      ignition_enable = 1; // Enable ignition if ASMS is LOW
      Serial.println("ASMS is LOW, enabling ignition");
    }
  }
  if (current_state >= STATE_JETSONWAITING && current_state < STATE_EMERGENCY && current_state != STATE_MANUAL)
  {
    continuous_monitoring();
  }

  switch (current_state)
  {
  case STATE_INIT:
    // peripheral_init();
    as_state = AS_STATE_OFF; // Autonomous system state
    ignition_enable = 0;     // Reset ignition enable flag
    emergency_flag = 0;      // Reset emergency flag
    ignition_flag = 0;
    asms_flag = 0;
    jetson_mission = MANUAL;
    res_emergency = 0;
    wdt_togle_enable = true;

    // current_state = STATE_INITIAL_SEQUENCE; // Transition to initial sequence state
    current_state = STATE_MISSION_SELECT; // Transition to mission select state
                                          // current_state = STATE_INIT;
    break;

  case STATE_MISSION_SELECT:

    // Check for manual ignition from CAN
    emergency_flag = 0;
    if (IGN_manual == 1)
    {
      current_state = STATE_MANUAL;
      break;
    }

    static int last_button = 0;            // Start with LOW (pulldown default)
    static unsigned long ms_last_time = 0; // last timestamp

    if (digitalRead(ASMS) == LOW /*&& !res_active*/)
    {
      uint8_t current_button = digitalRead(MS_BUTTON1); // HIGH when pressed

      // 100 ms debunce
      if (last_button == LOW && current_button == HIGH && (millis() - ms_last_time) >= 400)
      {
        current_mission = (current_mission_t)(((int)current_mission + 1) % 7);
        ms_last_time = millis(); // start new lockout
        Serial.print("Mission changed to: ");
        Serial.println(current_mission);
      }

      last_button = current_button; // remember raw state
    }
    else
    {
      current_state = STATE_JETSONWAITING; // Transition to Jetson waiting state
      // initial_sequence_state = IGNITON; // Reset initial sequence state
      initial_sequence_state = WDT_TOOGLE_CHECK; // Reset initial sequence state
      // digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid
      // digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
      digitalWrite(Debug_LED4, HIGH); // Indicate mission selection
    }
    break;

  case STATE_JETSONWAITING:
    // Wait for Jetson to respond with AS_STATE_OFF on CAN ID 0x503
    // The transition to STATE_INITIAL_SEQUENCE is handled in canISR() when AS_STATE_OFF is received
    // If ASMS is removed while waiting, return to mission select
    if (digitalRead(ASMS) == LOW)
    {
      current_state = STATE_MISSION_SELECT;
      Serial.println("ASMS removed while waiting for Jetson, returning to mission select");
    }
    break;

  case STATE_INITIAL_SEQUENCE:

    initial_sequence(); // Execute initial sequence actions
    break;

  case STATE_EBS_ERROR:
    current_state = STATE_EMERGENCY;
    break;

  case STATE_READY:
    /***TODO: Destava o cao
     *  Solenoids a 0
     *
     * ***/
    if (jetson_ready)
    {
      as_state = AS_STATE_READY; // Autonomous system state
      digitalWrite(SOLENOID_REAR, LOW);
      digitalWrite(SOLENOID_FRONT, LOW);
    }

    break;

  case STATE_DRIVING:
    /**
     * TODO: Release brakes
     * Solenoids a 0
     * AS_state = AS_STATE_DRIVING;
     */

    digitalWrite(SOLENOID_FRONT, HIGH);
    digitalWrite(SOLENOID_REAR, HIGH);
    as_state = AS_STATE_DRIVING;
    break;

  case STATE_EMERGENCY:
    // Handle emergency actions
    digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid
    digitalWrite(SOLENOID_REAR, LOW);  // Activate rear solenoid
    emergency_flag = 1;                // Set emergency flag
    if (res_emergency == 0 && TANK_PRESSURE_FRONT < 1 && TANK_PRESSURE_REAR < 1 && ignition_flag == 0 && millis() - emergency_timestamp > 9000)
    {
      as_state = AS_STATE_OFF;
      current_state = STATE_INIT;
      Serial2.println("Emergency state timeout, returning to INIT state");
      emergency_flag = 0; // Reset emergency flag
    }
    else
    {
      Serial2.println("Emergency state active, waiting for AS response");
    }

    break;
  case STATE_FINISHED:
    // TODO: Make sue the ca is stopped
    static unsigned long rpm_zero_time = 0;
    static unsigned long state_change_time = 0;
    static volatile int flag_as_state = 0;

    if (rpm_vcu == 0)
    {
      if (rpm_zero_time == 0)
      {
        rpm_zero_time = millis();
      }

      if (millis() - rpm_zero_time > 1000)
      {
        digitalWrite(SOLENOID_REAR, LOW); // Activate rear solenoid
        digitalWrite(SOLENOID_FRONT, LOW);
        ignition_enable = 0; // Reset ignition enable flag
        if (flag_as_state == 0)
        {
          flag_as_state = 1;
          state_change_time = millis(); // Record the time of entering finished state
        }
        if (millis() - state_change_time > 5000)
        {
          if (HYDRAULIC_PRESSURE_FRONT >= 9 * TANK_PRESSURE_FRONT && HYDRAULIC_PRESSURE_REAR >= 3.8 * TANK_PRESSURE_REAR)
          {
             as_state = AS_STATE_FINISHED;
          }
          else
          {
            current_state = STATE_EMERGENCY;
          }
        }
      }
    }
    else
    {
      rpm_zero_time = 0;
      state_change_time = 0;
    }
    break;

  case STATE_MANUAL:
    // Handle manual state - return to mission select when IGN_manual becomes 0
    if (IGN_manual == 0)
    {
      current_state = STATE_MISSION_SELECT;
    }
    // In manual state, keep solenoids deactivated (brakes applied)
    digitalWrite(SOLENOID_FRONT, LOW);
    digitalWrite(SOLENOID_REAR, LOW);
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

  pinMode(MS_BUTTON1, INPUT);

  pinMode(MS_LED_TRACKD, OUTPUT);
  pinMode(MS_LED_ACCL, OUTPUT);
  pinMode(MS_LED_SKIDPAD, OUTPUT);
  pinMode(MS_LED_MANUEL, OUTPUT);
  pinMode(MS_LED_INSPCT, OUTPUT);
  pinMode(MS_LED_AUTOCRSS, OUTPUT);
  pinMode(MS_LED_EBS, OUTPUT);
  digitalWrite(MS_LED_TRACKD, 1);
  digitalWrite(MS_LED_ACCL, 1);
  digitalWrite(MS_LED_SKIDPAD, 1);
  digitalWrite(MS_LED_MANUEL, 1);
  digitalWrite(MS_LED_INSPCT, 1);
  digitalWrite(MS_LED_AUTOCRSS, 1);
  digitalWrite(MS_LED_EBS, 1);
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

  digitalWrite(SOLENOID_FRONT, 0); // 1 equals braking off
  digitalWrite(SOLENOID_REAR, 0);

  pinMode(SDC_FEEDBACK, INPUT);

  Serial2.begin(115200);
  Serial.begin(115200);

  CAN.begin();
  CAN.setBaudRate(1000000); // Set CAN baud rate to 1 Mbps
  CAN.setMaxMB(16);         // Set maximum number of mailboxes
  // CAN.setMB(MB4,RX,STD);
  // CAN.setMB(MB5,RX,STD);
  // CAN.setMBFilter(REJECT_ALL);

  /*CAN.setMBFilter(MB0, AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID);
  CAN.setMBFilter(MB1, AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID);
  CAN.setMBFilter(MB2, AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID);
  CAN.setMBFilter(MB3, AUTONOMOUS_TEMPORARY_RES_FRAME_ID);
  CAN.setMBFilter(MB4, 0x446);
  CAN.setMBFilter(MB5, 0x546);*/

  CAN.enableMBInterrupts(); // Enable mailbox interrupts added 10 july 2025
  CAN.onReceive(canISR);

  PRESSURE_TIMER.begin(Pressure_readings, 100000); // 100ms

  CAN_TIMER.begin(send_can_msg, 100000); // 100ms

  Serial.println("Peripheral initialization complete");
  digitalWrite(Debug_LED2, 1); // Indicate initialization complete

  // HANDBOOK_MESSAGE_TIMER.begin(send_handbook_variables, 100000); // 100ms
}

void led_heartbit()
{
  if (HeartBit + 500 <= millis())
  {
    digitalWrite(HB_LED, !digitalRead(HB_LED)); // Toggle LED state
    HeartBit = millis();
    Serial.print("current_state: ");
    Serial.println(current_state);
    Serial.print("as_state: ");
    Serial.println(as_state);
    Serial.print("initial_sequence_state: ");
    Serial.println(initial_sequence_state);
    Serial.print("SDC feedback: ");
    Serial.println(digitalRead(SDC_FEEDBACK));
    Serial.print("EBS Tank Pressure rear: ");
    Serial.println(TANK_PRESSURE_REAR);
    Serial.print("EBS Tank Pressure front: ");
    Serial.println(TANK_PRESSURE_FRONT);
    Serial.print("Hydraulic Pressure rear: ");
    Serial.println(HYDRAULIC_PRESSURE_REAR);
    Serial.print("Hydraulic Pressure front: ");
    Serial.println(HYDRAULIC_PRESSURE_FRONT);
  }
}

/**
 * @brief Read pressure sensors and update system state
 */
void Pressure_readings()
{

  EBS_TANK_PRESSURE_A_values[adc_pointer] = analogRead(EBS_TANK_PRESSURE_A);
  EBS_TANK_PRESSURE_B_values[adc_pointer] = analogRead(EBS_TANK_PRESSURE_B);
  adc_pointer++;
  if (adc_pointer >= PRESSURE_READINGS)
  {
    adc_pointer = 0;
  }
  update_median_flag = true; // Set flag to update median values
}

/**
 * @brief Send CAN messages based on system state
 */
void send_can_msg()
{

  uint8_t tx_buffer[8]; // Buffer for CAN message

  CAN_message_t tx_message;

  struct autonomous_temporary_acu_ign_t encoded_ign;
  encoded_ign.ebs_pressure_rear = (uint8_t)(TANK_PRESSURE_FRONT * 10); // Convert to 0.1 bar scale
  encoded_ign.ebs_pressure_front = (uint8_t)(TANK_PRESSURE_REAR * 10); // Convert to 0.1 bar scale
  encoded_ign.ign = ignition_flag;                                     // Set ignition flag
  encoded_ign.asms = asms_flag;                                        // Set ASMS flag
  encoded_ign.emergency = emergency_flag;                              // Set emergency flag

  autonomous_temporary_acu_ign_pack(tx_buffer, &encoded_ign, AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH);
  tx_message.id = AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID;                  // Set CAN ID
  tx_message.len = AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH;                   // Set message length
  memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_ACU_IGN_LENGTH); // Copy data to CAN message buffer

  CAN.write(tx_message); // Send CAN message

  struct autonomous_temporary_acu_ms_t encoded_mission;
  encoded_mission.mission_select = (uint8_t)current_mission;
  autonomous_temporary_acu_ms_pack(tx_buffer, &encoded_mission, AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH);
  tx_message.id = AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID;                  // Set CAN ID
  tx_message.len = AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH;                   // Set message length
  memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_ACU_MS_LENGTH); // Copy data to CAN message buffer

  CAN.write(tx_message); // Send CAN message

  struct autonomous_temporary_rd_jetson_t RD_jetson_encode;
  if (current_state == STATE_JETSONWAITING)
  {
    RD_jetson_encode.rd = 1; // Set RD value for Jetson waiting state
  }
  if (current_state == STATE_READY)
  {
    RD_jetson_encode.rd = 2; // Set RD value based on current mission
  }
  else if (current_state == STATE_DRIVING)
  {
    RD_jetson_encode.rd = 3; // Set RD value based on current mission
  }
  else if (current_state == STATE_EMERGENCY)
  {
    RD_jetson_encode.rd = 4; // Set RD value based on current mission
  }
  else
  {
    RD_jetson_encode.rd = 1; // Set RD value based on current mission
  }
  autonomous_temporary_rd_jetson_pack(tx_buffer, &RD_jetson_encode, AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH);
  tx_message.id = 0x513;
  tx_message.len = AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH;                   //
  memcpy(tx_message.buf, tx_buffer, AUTONOMOUS_TEMPORARY_RD_JETSON_LENGTH); // Copy data to CAN message buffer

  CAN.write(tx_message); // Send CAN message

  // Send ACU state on ID 0x700
  CAN_message_t acu_state_msg;
  acu_state_msg.id = 0x700;
  acu_state_msg.len = 8;
  acu_state_msg.buf[0] = (uint8_t)current_state; // Send current ACU state in first byte
  acu_state_msg.buf[1] = (uint8_t)as_state;      // Send current autonomous system state in second byte
  CAN.write(acu_state_msg);                      // Send ACU state message
}

void median_pressures()
{

  // Calculate median for tank pressure B
  float sum = 0;
  for (int i = 0; i < PRESSURE_READINGS; i++)
  {
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
  for (int i = 0; i < PRESSURE_READINGS; i++)
  {
    sum += EBS_TANK_PRESSURE_A_values[i];
  }
  TANK_PRESSURE_REAR = sum / PRESSURE_READINGS;

  // Store raw voltage for debugging (convert ADC to voltage)
  rawVoltage = TANK_PRESSURE_REAR * 3.3 / 1023; // Read raw voltage from the analog pin

  // Use corrected divider value (0.85 instead of 0.66) prev val 0.476
  actualVoltage = rawVoltage / 0.66;

  // Apply formula ONCE with corrected divider
  TANK_PRESSURE_REAR = (actualVoltage - 0.5) / 0.4;

  // Serial.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar");
  // Serial.println("Tank pressure rear: " + String(TANK_PRESSURE_REAR) + " bar");
}

/**
 * @brief CAN receive interrupt callback.
 *
 * When a can message is received, this function is called to process the message.
 * It decodes the message based on its ID and updates the system state accordingly.
 * @note This function is called from the FlexCAN_T4 library's interrupt handler.
 * @param msg The received CAN message
 */
void canISR(const CAN_message_t &msg)
{
  uint16_t aux_brake_p = 0;
  digitalWrite(Debug_LED3, !digitalRead(Debug_LED3)); // Indicate reception of RES message
  switch (msg.id)
  {
  case 0x446:
    dynamics_steering_angle = msg.buf[1] << 8 | msg.buf[0]; // Update steering angle actual
    Steering_angle_actual = (dynamics_steering_angle / 10);
    Serial2.println("Steering angle actual: " + String(Steering_angle_actual) + " degrees in isr");
    Serial2.println("Dynamics steering angle: " + String(dynamics_steering_angle / 10) + " degrees in isr");
    break;

  case 0x546:
    aux_brake_p = (msg.buf[1] << 8 | msg.buf[0]);
    HYDRAULIC_PRESSURE_REAR = aux_brake_p / 10; // Update rear brake pressure
    Brake_pressure_rear = (u_int8_t)HYDRAULIC_PRESSURE_REAR;
    Serial2.println("Rear brake pressure: " + String(Brake_pressure_rear) + " bar in isr");
    break;
    // TODO: Read id 0x556
    //  divide by 10 and store in float

  case AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID:
    jetson_mission = (current_mission_t)msg.buf[0]; // Update mission response from Jetson
    break;

  case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
    RES_timeout = millis();
    if (msg.buf[0] == 0)
    {
      res_emergency = 1; // Set emergency response flag
    }
    else
    {
      res_emergency = 0; // Reset emergency response flag
    }

    if (!res_active)
    {
      res_active = true; // Set response active flag
    }
    break;

  case AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID:
    VCU_timeout = millis(); // Update VCU timeout
    if (ignition_flag == 1)
    {
      ignition_vcu = (msg.buf[0] == 9) ? 1 : 0; // Update ignition signal from VCU
    }
    else
    {
      ignition_vcu = 0; // Reset ignition flag if ignition enable is not set
    }

    HYDRAULIC_PRESSURE_FRONT = msg.buf[1]; // Convert to bar
    // Serial2.println("Hydraulic pressure front: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar in isr");
    break;

  case 0x503:
    JETSON_timeout = millis(); // Update Jetson timeout
                               /* if (current_state != STATE_EMERGENCY || current_state != STATE_FINISHED)
                                {
                                  // Update autonomous system state
                                }*/

    if (msg.buf[0] == 5 || msg.buf[0] == 4)
    {
      if (msg.buf[0] == 5)
      {
        current_state = STATE_FINISHED; // Transition to FINISHED state
        Serial.println("Jetson responded with AS_STATE_FINISHED, transitioning to FINISHED state");
      }
      if (msg.buf[0] == AS_STATE_EMERGENCY)
      {
        current_state = STATE_EMERGENCY; // Transition to EMERGENCY state
        Serial.println("Jetson responded with AS_STATE_EMERGENCY, transitioning to EMERGENCY state");
      }
    }
    else
    {
      as_state = (AS_STATE_t)msg.buf[0];

      switch (as_state)
      {
      case AS_STATE_OFF:
        // If in JETSONWAITING state, transition to INITIAL_SEQUENCE when receiving AS_STATE_OFF
        if (current_state == STATE_JETSONWAITING)
        {

          current_state = STATE_INITIAL_SEQUENCE;
          initial_sequence_state = WDT_TOOGLE_CHECK; // Reset initial sequence state
          Serial.println("Jetson responded with AS_STATE_OFF, starting initial sequence");
        }
        jetson_ready = 0;
        // Don't transition to READY from other states - only after initial sequence completes
        break;

      case AS_STATE_READY:
        current_state = STATE_READY; // Transition to READY state
        jetson_ready = 1;
        break;

      case AS_STATE_DRIVING:
        current_state = STATE_DRIVING; // Transition to DRIVING state
        break;

      case AS_STATE_EMERGENCY:
        current_state = STATE_EMERGENCY; // Transition to EMERGENCY state
        break;

      case AS_STATE_FINISHED:
        current_state = STATE_FINISHED; // Transition to FINISHED state
        break;

      default:
        break;
      }
    }

    break;

  case 0x456: // Front wheels
    wheel_speed_fl = 0;
    wheel_speed_fr = 0;
    wheel_speed_fl = (uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8);
    wheel_speed_fl = wheel_speed_fl * 0.1;
    wheel_speed_fr = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    wheel_speed_fr = wheel_speed_fr * 0.1;
    Serial2.println("Wheel speed front: FL = " + String(wheel_speed_fl) + " | FR = " + String(wheel_speed_fr));
    break;

  case 0x556: // Rear wheels
    wheel_speed_rl = 0;
    wheel_speed_rr = 0;
    wheel_speed_rl = (uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8);
    wheel_speed_rl = wheel_speed_rl * 0.1;
    wheel_speed_rr = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    wheel_speed_rr = wheel_speed_rr * 0.1;
    // Serial.println("Wheel speed rear: RL = " + String(wheel_speed_rl) + " | RR = " + String(wheel_speed_rr));
    break;
  case AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID:
    rpm_vcu = ((msg.buf[1] << 8) | msg.buf[0]);
    break;
  case 0x600:
    IGN_manual = msg.buf[0]; // Read manual ignition state
    sdc_signal = msg.buf[4];
  default:
    // Unknown message ID, ignore
    break;
  }
}

void initial_sequence()
{
  switch (initial_sequence_state)
  {
  case WDT_TOOGLE_CHECK:
    if (digitalRead(SDC_FEEDBACK) == LOW)
    {
      initial_sequence_state = WDT_STP_TOOGLE_CHECK;
      wdt_togle_enable = false;    // Disable WDT toggle for initial sequence
      wdt_relay_timout = millis(); // Reset WDT toggle counter
    }
    break;

  case WDT_STP_TOOGLE_CHECK:
    if (SKIP_SDC_FEEDBACK)
    {
      initial_sequence_state = PNEUMATIC_CHECK; // Skip SDC feedback check
      wdt_togle_enable = true;                  // Enable WDT toggle
      break;
    }

    if (digitalRead(SDC_FEEDBACK) == HIGH)
    {
      initial_sequence_state = PNEUMATIC_CHECK;
      wdt_togle_enable = true;
      Serial.println("SDC feedback went HIGH, proceeding to PNEUMATIC_CHECK");
    }
    else
    {
      unsigned long elapsed = millis() - wdt_relay_timout;
      if (elapsed >= 5000)
      {
        Serial.println("SDC feedback timeout after " + String(elapsed) + "ms, going to ERROR");
        initial_sequence_state = ERROR;
      }
      else if (elapsed % 1000 == 0) // Print every second
      {
        Serial.println("Waiting for SDC feedback, elapsed: " + String(elapsed) + "ms, SDC: " + String(digitalRead(SDC_FEEDBACK)));
      }
    }
    break;

  case PNEUMATIC_CHECK:
    if (SKIP_PNEUMATIC_CHECK)
    {
      initial_sequence_state = PRESSURE_CHECK1;
      break;
    }

    if (TANK_PRESSURE_REAR > 6.0 && TANK_PRESSURE_FRONT > 6.0)
    {
      if (TANK_PRESSURE_REAR < 10.0 && TANK_PRESSURE_FRONT < 10.0)
      {
        initial_sequence_state = PRESSURE_CHECK1; // Transition to pressure check state
      }
      else
      {
        initial_sequence_state = ERROR; // Transition to pressure check state
      }
    }
    else
    {
      initial_sequence_state = ERROR; // Transition to pressure check state
    }
    break;

  case PRESSURE_CHECK1:
    if (SKIP_PRESSURE_CHECK1)
    {
      initial_sequence_state = IGNITON;
      break;
    }

    if (HYDRAULIC_PRESSURE_FRONT >= 9 * TANK_PRESSURE_FRONT && HYDRAULIC_PRESSURE_REAR >= 3.8 * TANK_PRESSURE_REAR)
    {
      initial_sequence_state = IGNITON;
    }
    else
    {
      Serial2.println("Pressure check failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial2.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar, Rear pressure: " + String(TANK_PRESSURE_REAR) + " bar");
      initial_sequence_state = ERROR;
    }

    break;

  case IGNITON:
    ignition_enable = 1; // Enable ignition
    if (SKIP_IGNITION_CHECK)
    {
      // current_state = STATE_READY;
      initial_sequence_state = PRESSURE_CHECK_FRONT;
      pressure_check_delay = millis();
      break;
    }

    if (ignition_vcu == 1 && ignition_flag == 1)
    {
      // current_state = STATE_READY; //no final da initial sequence
      initial_sequence_state = PRESSURE_CHECK_FRONT; // Transition to pressure check state
      pressure_check_delay = millis();               // Reset pressure check delay
    }
    break;

  case PRESSURE_CHECK_REAR:
    digitalWrite(SOLENOID_REAR, HIGH); // Deactivate rear solenoid
    digitalWrite(SOLENOID_FRONT, LOW); // Activate front solenoid

    if (SKIP_PRESSURE_REAR_CHECK)
    {
      initial_sequence_state = PRESSURE_CHECK2;
      pressure_check_delay = millis();
      break;
    }

    if (HYDRAULIC_PRESSURE_REAR >= TANK_PRESSURE_REAR * 3 && HYDRAULIC_PRESSURE_FRONT <= 1 && millis() - pressure_check_delay >= 1000)
    {
      initial_sequence_state = PRESSURE_CHECK2;
      pressure_check_delay = millis(); // Reset pressure check delay
    }
    if (millis() - pressure_check_delay >= 5000)
    {                                 // Check if 500 ms has passed
      initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
      Serial.println("Pressure check failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial.println("rear pressure check");
    }
    break;

  case PRESSURE_CHECK_FRONT:
    digitalWrite(SOLENOID_REAR, LOW);   // Deactivate rear solenoid
    digitalWrite(SOLENOID_FRONT, HIGH); // Activate front solenoid

    if (SKIP_PRESSURE_FRONT_CHECK)
    {
      initial_sequence_state = PRESSURE_CHECK_REAR;
      pressure_check_delay = millis();
      break;
    }

    if (HYDRAULIC_PRESSURE_FRONT >= TANK_PRESSURE_FRONT * 9 && HYDRAULIC_PRESSURE_REAR <= 1 && millis() - pressure_check_delay >= 1000)
    {
      // initial_sequence_state = PRESSURE_CHECK_REAR;
      initial_sequence_state = PRESSURE_CHECK_REAR; // Transition to pressure check state
      pressure_check_delay = millis();              // Reset pressure check delay
    }
    if (millis() - pressure_check_delay >= 5000)
    {                                 // Check if 500 ms has passed
      initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
      Serial.println("Pressure check failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar, Rear pressure: " + String(TANK_PRESSURE_REAR) + " bar");
      Serial.println("Front pressure check");
    }
    break;

  case PRESSURE_CHECK2:
    digitalWrite(SOLENOID_REAR, LOW);  // Deactivate rear solenoid
    digitalWrite(SOLENOID_FRONT, LOW); // Deactivate front solenoid

    if (SKIP_PRESSURE_CHECK2)
    {
      current_state = STATE_READY; // Transition to ready state
      // initial_sequence_state = STATE_READY; // Reset initial sequence state
      break;
    }

    if (HYDRAULIC_PRESSURE_REAR >= 3 * TANK_PRESSURE_REAR && HYDRAULIC_PRESSURE_FRONT >= 9 * TANK_PRESSURE_FRONT)
    {
      current_state = STATE_READY; // Transition to ready state
    }
    if (millis() - pressure_check_delay >= 5000)
    { // Check if 5000 ms has passed
      Serial2.println("Pressure check 2 failed: Front pressure: " + String(HYDRAULIC_PRESSURE_FRONT) + " bar, Rear pressure: " + String(HYDRAULIC_PRESSURE_REAR) + " bar");
      Serial2.println("Tank pressure front: " + String(TANK_PRESSURE_FRONT) + " bar, Rear pressure: " + String(TANK_PRESSURE_REAR) + " bar");
      Serial2.println("Initial sequence error: Pressure check 2 failed or timeout occurred");
      initial_sequence_state = ERROR; // Transition to error state if pressure check takes too long
    }
    break;

  case ERROR:
    current_state = STATE_EBS_ERROR; // Transition to EBS error state
    Serial2.println("Initial sequence error: Pressure check failed or timeout occurred");

    break;

  default:
    Serial.println("Unknown initial sequence state");
    break;
  }
}

/**
 * @brief Debounces and checks the ignition input signal.
 *
 * This function reads the current state of the ignition pin (IGN_PIN) and applies
 * a debounce algorithm to filter out spurious changes due to mechanical switch noise.
 * It updates the ignition_flag based on the debounced state and the ignition_enable flag.
 *
 *@note Variables used:
 *@note - last_debounce_time (static): Stores the last time the ignition input changed, used for debouncing.
 *@note - debounce_delay (const): The debounce interval in milliseconds.
 *@note - current_state: The current raw reading from the ignition pin.
 *@note - last_ign_state (external): The last raw state read from the ignition pin.
 *@note - debounced_ign_state (external): The last debounced state of the ignition pin.
 *@note - ignition_flag (external): Set to 1 if ignition is ON and enabled, otherwise 0.
 *@note - ignition_enable (external): Enables or disables the ignition logic.
 *
 * The function ensures that ignition_flag is set only if the ignition input is HIGH
 * and ignition_enable is true, providing reliable ignition state detection.
 */
void check_ignition()
{
  static unsigned long last_debounce_time = 0;
  const unsigned long debounce_delay = 30; // 30 ms debounce

  uint8_t current_state = digitalRead(IGN_PIN);

  if (current_state != last_ign_state)
  {
    last_debounce_time = millis();
    last_ign_state = current_state;
  }

  if ((millis() - last_debounce_time) > debounce_delay)
  {
    if (debounced_ign_state != current_state)
    {
      debounced_ign_state = current_state;
      ignition_flag = (debounced_ign_state == HIGH) ? 1 : 0;
    }
  }
  ignition_flag = ignition_flag && ignition_enable; // Ensure ignition flag is set only if ignition is enabled
}

/**
 * @brief Controls the state of the YELLOW_LEDS and BLUE_LEDS based on the current as_state.
 *
 * This function manages the visual indication of the system's state by toggling or setting
 * the YELLOW_LEDS and BLUE_LEDS according to the value of the as_state variable. The behavior
 * for each state is as follows:
 * - AS_STATE_OFF: Turns off both YELLOW_LEDS and BLUE_LEDS.
 * - AS_STATE_READY: Turns on YELLOW_LEDS and turns off BLUE_LEDS.
 * - AS_STATE_DRIVING: Toggles YELLOW_LEDS every 500 ms, keeps BLUE_LEDS off.
 * - AS_STATE_EMERGENCY: Toggles BLUE_LEDS every 500 ms, keeps YELLOW_LEDS off.
 * - AS_STATE_FINISHED: Turns off YELLOW_LEDS and turns on BLUE_LEDS.
 * - Default: Turns off both YELLOW_LEDS and BLUE_LEDS.
 *
 * Timing for toggling is managed using ASSI_YELLOW_time and ASSI_BLUE_time variables.
 *
 * @note This function assumes that as_state, ASSI_YELLOW_time, and ASSI_BLUE_time are
 *       defined and accessible in the current scope, and that digitalWrite, digitalRead,
 *       and millis functions are available (e.g., in an Arduino environment).
 */
void ASSI()
{

  switch (as_state)
  {
  case AS_STATE_OFF:
    digitalWrite(YELLOW_LEDS, LOW);
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

/**
 * @brief Updates the mission indicator LEDs based on the current mission state.
 *
 * This function sets the state of each mission status LED (MS_LED1 to MS_LED7)
 * to indicate the currently active mission. Each mission mode corresponds to a unique
 * LED pattern, where one LED is turned on to represent the active mission,
 * and the others are turned off (logic HIGH). If the mission state is not recognized,
 * all LEDs are turned off by default.
 *
 * When the current state is >= STATE_INITIAL_SEQUENCE, the active mission LED blinks
 * at 1 Hz frequency. When the state is below STATE_INITIAL_SEQUENCE, the LED stays solid.
 */
void Mission_Indicator()
{
  // Determine if we should blink (state >= STATE_INITIAL_SEQUENCE)
  bool should_blink = (current_state >= STATE_INITIAL_SEQUENCE);
  bool led_output = true; // Default to ON for solid state

  if (should_blink)
  {
    // Check if 500ms have passed (for 1 Hz blinking: 500ms ON, 500ms OFF)
    if (millis() - mission_LED_time >= 500)
    {
      mission_LED_time = millis();
      mission_LED_state = !mission_LED_state; // Toggle the LED state
    }
    led_output = mission_LED_state;
  }
  else
  {
    // For solid state, always ON and reset blinking state
    led_output = true;
    mission_LED_state = true;
    mission_LED_time = millis();
  }

  // First turn off all LEDs
  digitalWrite(MS_LED_TRACKD, 0);
  digitalWrite(MS_LED_ACCL, 0);
  digitalWrite(MS_LED_SKIDPAD, 0);
  digitalWrite(MS_LED_MANUEL, 0);
  digitalWrite(MS_LED_INSPCT, 0);
  digitalWrite(MS_LED_AUTOCRSS, 0);
  digitalWrite(MS_LED_EBS, 0);

  // Then turn on the appropriate LED based on current mission
  switch (current_mission)
  {
  case MANUAL:
    digitalWrite(MS_LED_MANUEL, led_output ? 1 : 0);
    break;
  case ACCELERATION:
    digitalWrite(MS_LED_ACCL, led_output ? 1 : 0);
    break;
  case SKIDPAD:
    digitalWrite(MS_LED_SKIDPAD, led_output ? 1 : 0);
    break;
  case TRACKDRIVE:
    digitalWrite(MS_LED_TRACKD, led_output ? 1 : 0);
    break;
  case EBS_TEST:
    digitalWrite(MS_LED_EBS, led_output ? 1 : 0);
    break;
  case INSPECTION:
    digitalWrite(MS_LED_INSPCT, led_output ? 1 : 0);
    break;
  case AUTOCROSS:
    digitalWrite(MS_LED_AUTOCRSS, led_output ? 1 : 0);
    break;
  default:
    // All LEDs already turned off above
    break;
  }
}

/**
 * @brief Encodes handbook variables for CAN bus transmission.
 * This function prepares the handbook variables related to driving dynamics
 */
void send_handbook_variables()
{

  uint8_t aux_buf = 0;

  // DV driving dynamics 1
  CAN_message_t msg;
  msg.buf[0] = Speed_actual;              // Scale speed actual to 0.5
  msg.buf[1] = Speed_target;              // Scale speed target to 0.5
  msg.buf[2] = Steering_angle_actual * 2; // Scale steering angle actual to 0.5
  msg.buf[3] = Steering_angle_target * 2; // Scale steering angle target to 0.5
  msg.buf[4] = Brake_hydr_actual;         // Scale brake hydraulic actual to 0.5
  msg.buf[5] = Brake_hydr_target;         // Scale brake hydraulic target to 0.5
  msg.buf[6] = Motor_moment_actual;       // Scale motor moment actual to 0.5
  msg.buf[7] = Motor_moment_target;       // Scale motor moment target to 0.5

  msg.id = 0x500; // Set CAN ID for driving dynamics 1
  msg.len = 8;    // Set message length to 8 bytes
  CAN.write(msg); // Send the CAN message

  // DV system status

  // byte 0
  CAN_message_t msg_dv;
  msg_dv.buf[0] = as_status && 0b00000111; // Autonomous system status
  aux_buf = EBS_status << 3;
  msg_dv.buf[0] = msg_dv.buf[0] || aux_buf; // Set EBS status in bits 3-4
  aux_buf = AMI_status << 5;                // set AMI status in bits 5-7
  msg_dv.buf[0] = msg_dv.buf[0] || aux_buf; // Set AMI status in bits 5-7

  // byte 1
  msg_dv.buf[1] = (Steering_state ? 1 : 0);
  aux_buf = ASB_redundancy << 1;            // Set ASB redundancy in bits 1-2
  msg_dv.buf[1] = msg_dv.buf[1] || aux_buf; // Set ASB redundancy in bits 1-2
  aux_buf = Lap_counter << 3;               // Set lap counter in bits 3-6
  msg_dv.buf[1] = msg_dv.buf[1] || aux_buf; // Set lap

  // byte 2
  msg_dv.buf[2] = cones_count_actual; // Set current cones count in byte 2
  // byte 3-4
  msg_dv.buf[3] = cones_count >> 8;
  msg_dv.buf[4] = (cones_count << 8) & 0xFF; // Set total cones count in bytes 3-4

  msg_dv.id = 0x502;
  msg_dv.len = 5;    // Set message length to 5 bytes
  CAN.write(msg_dv); // Send the CAN message

  // ASF signals

  CAN_message_t msg_asf;
  msg_asf.buf[0] = (uint8_t)(TANK_PRESSURE_FRONT * 10);
  msg_asf.buf[1] = (uint8_t)(TANK_PRESSURE_REAR * 10);
  msg_asf.buf[2] = Brake_pressure_front; // Convert to 0.1 bar scale
  msg_asf.buf[3] = Brake_pressure_rear;  // Convert to 0.1 bar scale

  msg_asf.len = 4;    // Set message length to 4 bytes
  msg_asf.id = 0x511; // Set CAN ID for ASF signals
  CAN.write(msg_asf); // Send the CAN message
}

void continuous_monitoring()
{
  bool sdc_opened = digitalRead(SDC_FEEDBACK) == HIGH;
  if (sdc_opened)
  {
    delay(50);
    if (!(HYDRAULIC_PRESSURE_FRONT >= 60 && HYDRAULIC_PRESSURE_FRONT <= 120) || !(HYDRAULIC_PRESSURE_REAR >= 60 && HYDRAULIC_PRESSURE_REAR <= 120))
    {
      current_state = STATE_EBS_ERROR;
    }
    // stop monitoring
    current_state = STATE_INIT;
  }
  else if (!(TANK_PRESSURE_FRONT >= 4 && TANK_PRESSURE_FRONT <= 10) || !(TANK_PRESSURE_REAR >= 4 && TANK_PRESSURE_REAR <= 10))
  {
    current_state = STATE_EBS_ERROR;
  }
  if (current_state == STATE_DRIVING)
  {
    if (millis() - RES_timeout > CAN_TIMEOUT_TIME && millis() - JETSON_timeout > CAN_TIMEOUT_TIME && millis() - VCU_timeout > CAN_TIMEOUT_TIME /*&& millis()- MAXON_timeout > CAN_TIMEOUT_TIME*/)
    {
      current_state = STATE_EMERGENCY;
    }
  }
}