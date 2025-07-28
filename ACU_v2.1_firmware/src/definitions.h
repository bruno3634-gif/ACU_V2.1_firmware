// SKIPS INITIAL SEQUENCE
#define SKIP_SDC_FEEDBACK true
#define SKIP_PNEUMATIC_CHECK false
#define SKIP_PRESSURE_CHECK1 false
#define SKIP_IGNITION_CHECK false
#define SKIP_PRESSURE_FRONT_CHECK false
#define SKIP_PRESSURE_REAR_CHECK false
#define SKIP_PRESSURE_CHECK2 false

#define SKIP_RES_EMMERGENCY false
#define SKIP_EBS_ERROR false
#define SKIP_CAN_AS_STATE_EMERGENCY false
#define SKIP_STATE_EMERGENCY false



#define WDT 39
#define EBS_TANK_PRESSURE_A A13
#define EBS_TANK_PRESSURE_B A12 
#define EBS_VALLVE_A A8
#define EBS_VALLVE_B 37
#define SDC_FEEDBACK A11



//#define IGN_PIN 34
#define R2D_PIN 10


// Mission Select

#define MS_BUTTON1 21
#define MS_LED_TRACKD 14
#define MS_LED_ACCL 15
#define MS_LED_SKIDPAD 16
#define MS_LED_MANUEL 17  
#define MS_LED_INSPCT 33
#define MS_LED_AUTOCRSS 13
#define MS_LED_EBS 20
#define AS_SW 38
// ASSI

//#define YELLOW_LEDS 24
//#define BLUE_LEDS 23
#define BLUE_LEDS 24
#define YELLOW_LEDS 23

// Solenoides

#define SOLENOID_FRONT 36
#define SOLENOID_REAR 10


// Debug Leds

#define HB_LED 28
#define Debug_LED2 29
#define Debug_LED3 34
#define Debug_LED4 35
#define Debug_LED5 30
#define Debug_LED6 31





#define TANK_PRESSURE_THRESHOLD 3.5

#define TANKS_INDEX_SIZE 6

#define ASMS 38
#define IGN_PIN 4
   

#define CAN_TIMEOUT_TIME 2000 // Timeout for CAN messages in milliseconds

#define MAX_RPM 6500