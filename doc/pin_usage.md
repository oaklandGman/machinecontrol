# esp32 pin usage

// IO pin assignments
const byte BIG_MOTOR_STEP      = 19;
const byte BIG_MOTOR_DIR       = 21;
const byte BIG_MOTOR_SLEEP     = 22;
const bool BIG_MOTOR_DIR_HI    = true; // drive not inveerted, direction counts up
const byte SMALL_MOTOR_STEP    = 25; // green
const byte SMALL_MOTOR_DIR     = 23; // blue
const byte SMALL_MOTOR_SLEEP   = 26; // brown
const bool SMALL_MOTOR_DIR_HI  = true; // drive not inverted, direction counts up
const byte EMERGENCY_STOP_PIN  = 35;
const byte LIMIT_SW1           = 33;
const byte LIMIT_SW2           = 27;

const byte VSPI_MOSI  = 23;
const byte VSPI_MISO  = 19;
const byte VSPI_CLK   = 18;
const byte VSPI_CS    = 5;

const byte HSPI_MISO  = 12;
const byte HSPI_MOSI  = 13;
const byte HSPI_CLK   = 14;
const byte HSPI_CS    = 15;
