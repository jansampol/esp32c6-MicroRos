// 1 = internal, 2 = external 1, 3 = external 2
// #define NUM_I2C_MULTIPLEXERS 3 (ALWAYS 3)

// when esp is not connected to the controller board
#define ESP_ATTACHED true

// Select whether using the prototype board or the new PCB for C5
// (prototype board or newer c5 board with battery)
// TODO: Based on this variable configure the correct pinmappings, mainly for I2C and SPI pins.
#define PROTOTYPE_ESP true      

// Temporary runtime selector while PCB supports only one SPI path at a time.
// Change only this line to switch quickly between modes.
#define SPI_RUNTIME_MODE_SPI0_ONLY 0
#define SPI_RUNTIME_MODE_SPI1_ONLY 1
#define ACTIVE_SPI_RUNTIME_MODE SPI_RUNTIME_MODE_SPI1_ONLY

// Wifi setup
#define USING_WEB_SERVER true
// set false at home and add own credentials below (Dont push those to git)
#define USING_IOT_ROAM true

#if USING_IOT_ROAM
#define MAMRI_SSID "iotroam"
#define MAMRI_PASSWORD "L5PTb4mQVQ"
# else
#define MAMRI_SSID ""
#define MAMRI_PASSWORD ""
#endif

#define MAMRI_HOSTNAME "mamri-controller-esp-c5"

// Interrupt timer frequency [Hz]
#define INTERRUPT_TIMER_FREQUENCY 100 
// [micro seconds]
#define INTERRUPT_TIMER_INTERVAL_US (1000000 / INTERRUPT_TIMER_FREQUENCY)

// TFT Screen parameters
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

// This will be determined by the robot config later on
#define NUM_OF_FERRISWHEELS_ON_BOOT 0

// Motor frequency
#define MAX_MOTOR_HZ 80

// Inverse kinematics increment speed
#define IK_TRANSLATION_SPEED 0.1f
#define IK_ROTATION_SPEED 0.01f
