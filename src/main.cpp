#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include <FastAccelStepper.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFSEditor.h>
#include "my_passwords.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncEventSource events("/events");

#ifndef PASSWORD_SET
const char* ssid = "xxxx";
const char* password = "xxxx";
const char* http_username = "xxxx";
const char* http_password = "xxxx";
#endif
const char* PARAM_MESSAGE = "Machine Control";
const char* hostName = "machine"; // network hostname

const char* fncMotor = "motor"; // string for motor function messages
const char* fncInfo = "info"; // string for info function messages

const byte DEBOUNCETIME = 10; // debounce time in millis 

// IO pin assignments
const byte BIG_MOTOR_STEP     = 19;
const byte BIG_MOTOR_DIR      = 21;
const byte BIG_MOTOR_SLEEP    = 22;
const byte SMALL_MOTOR_STEP   = 25;
const byte SMALL_MOTOR_DIR    = 23;
const byte SMALL_MOTOR_SLEEP  = 26;
const byte EMERGENCY_STOP_PIN = 35;
const byte LIMIT_SW1          = 33;
const byte LIMIT_SW2          = 27;

// Speed settings
const unsigned int MAX_ACCEL         = 4294967295;
const unsigned int BIG_MOTOR_HZ      = 9000;
const unsigned int BIG_MOTOR_ACCEL   = 40000;
const unsigned int SMALL_MOTOR_HZ    = 400;
const unsigned int SMALL_MOTOR_ACCEL = 400; // infinite acceleration

struct WSmsg // structure for msgpack messages to send/recv over websocket
{
  uint8_t msgId;
  char msgArray[201]; // 201 byte array for a message, hope that's enough!
};
struct MOTcmd // structure to define commands for stepper motor control
{
  char cmd[16]; // command string
  char txt[32]; // text string, for filename, etc
  int dat; // four bytes of data
};

struct MotorConfig // motor configuration parameters to store
{
  int bigmotorSpeed; // speed setting for big motor
  int bigmotorAccel; // accelleration for big motor
  int smallmotorSpeed; // speed setting for small motor
  int lubeAmt; // amount dispensed per request
  int lubeFreq; // frequnecy of dispensing
};

TaskHandle_t taskStepper; // handle for stepper task
TaskHandle_t taskMSG; // handle for websocket message handling 
TaskHandle_t taskSW1; // handle for task that watching digital input
TaskHandle_t taskSW2; // handle for task that watching digital input
TaskHandle_t taskESTOP; // handle for task watching digital input

QueueHandle_t motQueue; // handle for motor command queue
QueueHandle_t wsmsgQueue; // handle for websocket message queue
QueueHandle_t wsoutQueue; // queue for outbound websocket messages

SemaphoreHandle_t syncSW1; // for sw1 isr
SemaphoreHandle_t syncSW2; // for sw2 isr
SemaphoreHandle_t syncESTOP; // for estop isr

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *big_motor   = NULL;
FastAccelStepper *small_motor = NULL;

void IRAM_ATTR handleSW1() // super simple ISR to debounce switch
{
    xSemaphoreGiveFromISR(syncSW1, NULL);
}

void IRAM_ATTR handleSW2() // super simple ISR to debounce switch
{
    xSemaphoreGiveFromISR(syncSW2, NULL);
}

void IRAM_ATTR handleESTOP() // super simple ISR to debounce switch
{
    xSemaphoreGiveFromISR(syncESTOP, NULL);
}

void watchSW1(void * parameter) // task to watch limit switch 1 input
{
  volatile uint32_t sw1Last = 0; // last triggered time for switch
  struct MOTcmd motor; // buffer for motor related commands

  pinMode(LIMIT_SW1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW1), handleSW1, FALLING);

  for (;;) { // loop forever
    xSemaphoreTake(syncSW1, portMAX_DELAY);

    if (millis() - sw1Last > DEBOUNCETIME) {
      // input triggered
      strcpy(motor.cmd, "sw1");
      motor.dat = true;
      // xQueueSend(motQueue, &motor, 5 / portMAX_DELAY); // put info on motor control queue
      sw1Last = millis();
    }
  }
}

void watchSW2(void * parameter) // task to switch limit switch 2 input
{
  volatile uint32_t sw2Last = 0; // last triggered time for switch
  struct MOTcmd motor; // buffer for motor control message

  pinMode(LIMIT_SW2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW2), handleSW1, FALLING);

  for (;;) { // loop forever
    xSemaphoreTake(syncSW2, portMAX_DELAY);
  
    if (millis() - sw2Last > DEBOUNCETIME) {
      // input triggered
      strcpy(motor.cmd, "sw2");
      motor.dat = true;
      // xQueueSend(motQueue, &motor, 5 / portMAX_DELAY); // put info on motor control queue
      sw2Last = millis();
    }
  }
}

void watchESTOP(void * parameter) // task to watch ESTOP input
{
  volatile uint32_t estopLast = 0; // last triggered time for switch
  struct MOTcmd motor; // buffer for motor control message

  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP); // digital input, internal pull-up resistors on
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), handleESTOP, FALLING); // setup interrupt on falling edge 

  for (;;) { // loop forever
    xSemaphoreTake(syncESTOP, portMAX_DELAY); // block task until interrupt triggers
  
    if (millis() - estopLast > DEBOUNCETIME) { // check elapsed time between interrupts
      // input triggered, send message to motor control task 
      strcpy(motor.cmd, "estop");
      motor.dat = true;
      // xQueueSend(motQueue, &motor, 5 / portMAX_DELAY); // put info on motor control queue
      estopLast = millis();
    }
  }
}

void wsMsgtask(void * parameter) // task to handle sending and receiving websocket messages
{
  struct WSmsg data; // buffer for the message data
  struct MOTcmd motor; // buffer for motor control message

  unsigned long lastMillis = millis();

  for (;;) { // infinite loop
    // check receive queue every loop
    if ( uxQueueMessagesWaiting(wsmsgQueue) ) { // message in the queue, dump it to debug console
      xQueueReceive(wsmsgQueue, &data, 5 / portMAX_DELAY); // grab message from queue
      // Serial.printf("Received msg from client %u: ", data.msgId);
      // for (size_t i=0; i < sizeof(data.msgArray) - 1; i++ ) {
      //  Serial.printf("%02x ", data.msgArray[i]);
      // }
      // Serial.println();

      StaticJsonDocument<100> doc;
    
      DeserializationError error = deserializeMsgPack(doc, (char *) data.msgArray);
    
      if (error) {
        ws.printfAll("Deserialization failed error %s", error.c_str());
        //Serial.print("Deserialization failed with error: ");
        //Serial.println(error.c_str());
      } else { // deserialize successful, we have json
        const char* fnc = doc["fnc"]; // read function
        // ws.printfAll("Processing command for fuction %s", fnc);
        if (strcmp("motor", fnc)==0) {
          // if (fnc == 0x4d) { // M for motor
          strcpy(motor.cmd, doc["cmd"]); // copy json string value into buffer
          const char* txt = doc["txt"]; // attempt to read key which may not be set
          if (txt) strcpy(motor.txt, txt); // copy text string to buffer if it is set
          motor.dat = doc["dat"]; // assign integer value to buffer

          xQueueSend(motQueue, &motor, 5 / portMAX_DELAY); // put command on motor control queue
        } else if (strcmp("info", fnc)==0) {
          uint32_t result = uxTaskGetStackHighWaterMark(NULL);
          sprintf(data.msgArray, "Message task high water mark %u", result);
          xQueueSend(wsoutQueue, &data, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
        }
      }

      // pretty print to serial console
      // serializeJsonPretty(doc, Serial);
        
    }  // end receive queue check

    if (millis() - lastMillis >= 500) { // check transmit queue every 500ms
      if ( uxQueueMessagesWaiting(wsoutQueue) ) { // message in the queue, dump it to debug console
        // StaticJsonDocument<200> doc;
        xQueueReceive(wsoutQueue, &data, 5 / portMAX_DELAY); // grab message from queue

        ws.textAll(data.msgArray);

        lastMillis = millis();
      }
    } // end transmit queue check

  }
}

void runStepper(void * parameter) // task to handle motor related commands
{
  DynamicJsonDocument program(3072); // setup memory for big json program data

  bool runTest = false; // flag for motor self test routine
  bool bigmotorDir = false; // flag used for motor direction reversal in self test
  bool autoLube = false; // automatically dispense lubrication
  bool autoStroke = false; // automatically cycle motor
  bool shootLube = false; // flag to dispense lubrication
  bool bigmotorCal = false; // flag that automatica calibration is complete
  bool runCal = false; // automatic calibration phase 1
  bool runCal2 = false; // automatic calibration phase 2
  bool doDelay = false; // flag to trigger delay without motor motion
  bool updateClient = false; // flag to send parameters to ws clients
  bool initMotors = false; // flag to set stepper engine with config parameters
  bool initComplete = false; // flag indicating motors configured
  bool motEnabled = false; // flag indicating big motor is enabled
  bool updateDepth = false; // flag to update stroke depth number
  bool updateSpeed = false; // flag to update big motor speed
  bool updateStroke = false; // flag to upload big motor stroke length
  bool configLoaded = false; // flag indicating config loaded
  bool progLoaded = false; // flag indicating program has been loaded
  bool progRunning = false; // flag indicating program is running
  bool progNextStep = false; // flag indicating program ready for next step
  bool progLoop = false; // flag indicating program is running in a loop
  bool debugMotor = false; // flag to print diagnostics for big motor
  bool dualSpeed = false; // flag indicating different extend / retract speeds
  bool listFiles = false; // flag to send list of files to client


  unsigned long previousMillis = 0; // last time on the clock
  unsigned long currentMillis = millis();

  int bigmotorSpeed = 400; // speed of big motor
  int bigmotorOut = BIG_MOTOR_HZ; // extend speed
  int bigmotorIn = BIG_MOTOR_HZ; // retract speed
  int bigmotorAccel = BIG_MOTOR_ACCEL; // accelleration of big motor
  int bigmotorMove = 200; // stroke length
  int bigmotorDepth = 0; // stroke depth (offset from 0)
  int localMove = 0; // used in selftest routine
  int smallmotorSpeed = SMALL_MOTOR_HZ; // speed of small motor
  int lubeAmt = 400; // amount of lube dispensed per request
  int sw1Pos = 0; // step count for limit switch 1
  int sw2Pos = 0; // step count for limit switch 2
  int delayMillis = 0; // milliseconds for delay

  uint8_t strokeCnt   = 0; // number of strokes since last lube
  uint8_t lubeFreq    = 10; // number of strokes between lube
  uint8_t progPointer = 0; // where are we at in the program
  uint8_t progReps    = 0; // number of reps for current program command
  uint8_t progRepCnt  = 0; // counter for number of reps completed
  uint8_t progSteps   = 0; // number of steps in program
  uint8_t progLoopCnt = 0; // number of times program has looped

  const char* configFile = "/config.json";

  struct MOTcmd command; // buffer for incoming commands
  struct WSmsg tmpBuffer; // buffer for outgoing ws messages

  // loop forever
  for (;;) { 
    currentMillis = millis(); // update current count on clock

    if (uxQueueMessagesWaiting(motQueue)) // check queue for new WS commands
    {
      // queue not empty, grab a message and process it
      xQueueReceive(motQueue, &command, 5 / portMAX_DELAY);

      const char* cmd = command.cmd; // copy command from buffer to a local variable
      int dat = command.dat; // copy data from buffer to local variable
      
      // ws.printfAll("Received command %s data %i", cmd, dat); // print debug message

      if (strcmp("estop", cmd) == 0 ) { 
        // handle estop
      }

      if (strcmp("listfiles", cmd) == 0 ) { // set flag to send file list to client
        listFiles = true;
      }

      if (strcmp("loadprog", cmd) == 0 ) // open json program command file
      {
        const char* filename = command.txt; // copy filename from buffer to local variable
        if (strlen(filename)>0) { // make sure we have a filename
          File progFile = SPIFFS.open(filename, "r"); // open file on SPIFFS
        
          if (!progFile) {
            ws.printfAll("Program error: failed to open %s for reading.", filename); // failed to open, abort
          } else { // successfully opened, proceed
            // Deserialize the JSON document
            DeserializationError error = deserializeJson(program, progFile);
            if (error) {
              ws.textAll("Program error: unable to deserialize JSON.");
            } else {
              progLoaded = true; // set flag
              progRunning = false; // clear flag
              initMotors = true; // set flag, get things ready
              progLoopCnt = 0; // zero loop counter

              const char* progName = program["program"]["name"]; // name of program
              progSteps = program["program"]["steps"]; // number of program steps
              progPointer = 0; // reset pointer
              ws.printfAll("Program loaded: %s with %u steps.", progName, progSteps + 1);
              // serializeJsonPretty(program, Serial);
              progFile.close();
            }
          }
        }
      }

      if (strcmp("stopprog", cmd) == 0 ) // stop executing loaded program
      { 
        progRunning = false;
        autoStroke = false;
      }

      if (strcmp("execprog", cmd) == 0 ) // execute loaded program
      { 
        if (progLoaded && progSteps > 0) {
          progPointer = 0; // reset step pointer
          progRunning = true; // set flag to begin program
          progNextStep = true; // ready for next step
        } else {
          ws.textAll("Program error: no program loaded.");
        }
      }

      if (strcmp("saveconfig", cmd) == 0 ) // save motor parameters to json config file
      { 
        File file = SPIFFS.open(configFile, "w"); // open file on SPIFFS
        if (!file) {
          ws.textAll("Config error: failed to open config.json for writing."); // failed to open, abort
        } else {
          // successfully opened, proceed
          StaticJsonDocument<500> config;

          config["motspeed"]  = bigmotorSpeed;
          config["speedout"]  = bigmotorOut;
          config["speedin"]   = bigmotorIn;
          config["motaccel"]  = bigmotorAccel;
          config["lubeamt"]   = lubeAmt;
          config["lubefreq"]  = lubeFreq;
          config["lubespeed"] = smallmotorSpeed;

          if (serializeJsonPretty(config, file) == 0) { // write contents to file
            ws.textAll("Config error: failed to write to config.json");
          } else {
              ws.textAll("Config file saved.");
          }

          file.close(); // close file handle
        }
      }

      if (strcmp("loadconfig", cmd) == 0 ) // load motor parameters from json config file
      { 
        File file = SPIFFS.open(configFile, "r"); // open file on SPIFFS
      
        if (!file) {
          ws.textAll("Config error: failed to open config.json for reading."); // failed to open, abort
        } else {
          // successfully opened, proceed
          StaticJsonDocument<500> config;

          // Deserialize the JSON document
          DeserializationError error = deserializeJson(config, file);
          if (error) {
            ws.textAll("Config error: unable to deserialize JSON.");
          } else {
            bigmotorSpeed   = config["motspeed"]  | BIG_MOTOR_HZ;
            bigmotorOut     = config["speedout"]  | BIG_MOTOR_HZ;
            bigmotorIn      = config["speedin"]   | BIG_MOTOR_HZ;
            bigmotorAccel   = config["motaccel"]  | BIG_MOTOR_ACCEL;
            lubeAmt         = config["lubeamt"]   | 200;
            lubeFreq        = config["lubefreq"]  | 10;
            smallmotorSpeed = config["lubespeed"] | SMALL_MOTOR_HZ;

            initMotors = true; // activate some of the parameter changes
            configLoaded = true; // set flag that we've loaded config successfuy
            updateClient = true; // set flag to push update to clients

            ws.textAll("Config file loaded.");
          }

          file.close(); // close file handle
        }
      }

      if (strcmp("runtest", cmd) == 0 ) { // positioning test on big motor
        if (dat == 1) { 
          runTest = true;
          bigmotorSpeed = BIG_MOTOR_HZ;
          bigmotorMove = 200;
        } else {
          runTest = false;
        }
      }

      if (runTest == false) { // motortest disables all other commands
        if (strcmp("autostroke", cmd) == 0 ) { // command "autostroke"
          if (dat == 1) {
            strokeCnt = 0;
            autoStroke = true;
          } else {
            autoStroke = false;
          }
        } else if (strcmp("autolube", cmd) == 0 ) { // command "autolube"
          if (dat == 1) autoLube = true;
          else autoLube = false;
        } else if (strcmp("shootlube", cmd) == 0 ) { // command "shootlube" 
          shootLube = true;
        } else if (strcmp("dualspeed", cmd) == 0 ) { // command "dualspeed" 
          if (dat == 1) dualSpeed = true;
          else dualSpeed = false;
        } else if (strcmp("calibrate", cmd) == 0 ) { // command "calibrate" 
          runCal = true;
        } else if (strcmp("strokelen", cmd) == 0 ) { // command "strokelen"
          bigmotorMove = dat;
          updateStroke = true;
        } else if (strcmp("strokedep", cmd) == 0 ) { // command "strokedep" 
          bigmotorDepth = dat;
          updateDepth = true;
        } else if (strcmp("speedout", cmd) == 0 ) { // command "speedout" 
          bigmotorOut = dat;
        } else if (strcmp("speedin", cmd) == 0 ) { // command "speedin" 
          bigmotorIn = dat;
        } else if (strcmp("motspeed", cmd) == 0 ) { // command "motspeed" 
          bigmotorSpeed = dat;
          updateSpeed = true;
        } else if (strcmp("motaccel", cmd) == 0 ) { 
          bigmotorAccel = dat; // update big motor accelleration 
        } else if (strcmp("lubeamt", cmd) == 0 ) { 
          lubeAmt = dat; // update number of steps lube motor will run
        } else if (strcmp("lubefreq", cmd) == 0 ) {
          lubeFreq = dat;
        } else if (strcmp("update", cmd) == 0 ) {
          updateClient = true;
        } else if (strcmp("debugmot", cmd) == 0 ) {
          debugMotor = true;
        } else if (strcmp("motenabled", cmd) == 0 ) {
          if (dat == 1) {
            strcpy(tmpBuffer.msgArray, "Motors enabled.");
            xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
            digitalWrite(BIG_MOTOR_SLEEP, HIGH); // enable motor driver
            motEnabled = true;
          }
          else {
            strcpy(tmpBuffer.msgArray, "Motors disabled.");
            xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
            digitalWrite(BIG_MOTOR_SLEEP, LOW); // disable motor driver
            motEnabled = false;
          }
        } else if (strcmp("sw1", cmd) == 0 ) { // handle limit switch
          if (runCal) {
            sw1Pos = big_motor->getCurrentPosition();
            big_motor->stopMove();
            sprintf(tmpBuffer.msgArray, "Triggered SW1 at %i", sw1Pos);
            xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
          }
        } else if (strcmp("sw2", cmd) == 0 ) { // handle limit switch
          if (runCal) {
            sw2Pos = big_motor->getCurrentPosition();
            big_motor->stopMove();
            sprintf(tmpBuffer.msgArray, "Triggered SW2 at %i", sw2Pos);
            xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
          }
        }
      }
    } // end message queue check
    
    if (progRunning && progNextStep) // process JSON program commands
    {
      progNextStep = false; // clear flag, we're not ready for next step

      char key[5];
      sprintf(key,"%u",progPointer); // convert numeric pointer to char array
      
      // ws.printfAll("Program step %u of %u", progPointer, progSteps);

      if (progPointer <= progSteps) { // load next step
        progReps = program[key]["reps"]; // how many times to repeat this command, only for strokes
        const char* progFnc = program[key]["fnc"]; // read function from json
        const char* progDesc = program[key]["desc"]; // text description of this step
        ws.printfAll("Exec (%u/%u) %s reps %u", progPointer + 1, progSteps + 1, progDesc, progReps); // alert user

        if (strcmp("lube", progFnc) == 0 ) { // dispense lube only, automatic delay afterwards
          lubeAmt = program[key]["lubeamt"]; // how much
          shootLube = true; // set flag
          delayMillis = 5000; // 5 sec delay
          previousMillis = millis(); // starting time for delay
          doDelay = true;
        } else if (strcmp("stroke", progFnc) == 0 ) { // setup auto stroke
          autoStroke = true; // always set autostroke for this command
          updateStroke = true; // set flag to force position update
          progRepCnt = 0; // zero out counter
          strokeCnt = 0; // reset counter
          bigmotorSpeed = program[key]["motspeed"];
          bigmotorAccel = program[key]["motaccel"];
          bigmotorMove = program[key]["strokelen"];
          bigmotorDepth = program[key]["strokedep"];
          lubeAmt = program[key]["lubeamt"];
          lubeFreq = program[key]["lubefreq"];
          autoLube = program[key]["autolube"];
          dualSpeed = program[key]["dualspeed"] | false;
          bigmotorOut = program[key]["speedout"] | bigmotorSpeed;
          bigmotorIn = program[key]["speedin"] | bigmotorSpeed;
          big_motor->setSpeedInHz(bigmotorSpeed); // update speed 
        } else if (strcmp("depth", progFnc) == 0 ) { // set depth only
          bigmotorSpeed = program[key]["motspeed"] | BIG_MOTOR_HZ;
          bigmotorDepth = program[key]["strokedep"]; // set depth offset from home position
          big_motor->setSpeedInHz(bigmotorSpeed); // update speed
          autoStroke = false; // clear flag
          updateDepth = true; // set flag
        } else if (strcmp("loop", progFnc) == 0 ) { // loop back in program
          progNextStep = true; // set flag for next step, or jumping back

          if (progLoop) { // already in a loop, are we done yet?
            if (progLoopCnt++ < progReps) { // loop multiple times based on reps
              progPointer = program[key]["jumpto"] | 0; // reset pointer
              progRepCnt = 0; // reset counter
              strokeCnt = 0; // reset counter
              autoStroke = false; // clear flag
              ws.printfAll("Exec loop jumpto=%u progPointer=%u", program[key]["jumpto"], progPointer);
              // ws.printfAll("Program looping again to step %u", progPointer + 1);
            } else { // number of loops completed, don't reset pointer
              ws.textAll("Program loop completed");
            }
          } else { // ooh, first time loop
            progLoopCnt = 0; // reset counter
            progRepCnt = 0; // reset counter
            strokeCnt = 0; // reset counter
            autoStroke = false; // clear flag
            progPointer = program[key]["jumpto"] | 0; // reset pointer
            progLoop = true; // set flag for next time around
            ws.printfAll("Exec loop jumpto=%u progPointer=%u", program[key]["jumpto"], progPointer);
          }
        } else if (strcmp("delay", progFnc) == 0 ) { // set delay only
          delayMillis = program[key]["delay"]; // milliseconds
          doDelay = true;
          ws.printfAll("Exec delay for %ums", delayMillis);
          previousMillis = millis(); // starting time for delay
        } // end command selection

        progPointer++; // increment program step pointer
      } else if (progPointer > progSteps) { // end of program
        progRunning = false; // clear flag
        autoStroke = false;
        autoLube = false;
        ws.textAll("Program complete!");
      }
    } // end program runner

    if (runCal && motEnabled) // calibrate positioning
    {
      if (!runCal2) { // first calibration pass
        if ((sw1Pos != 0) && (sw2Pos != 0)) {
          big_motor->setSpeedInHz(60); // creeping speed
          big_motor->setAcceleration(MAX_ACCEL); // maximum acceleration
          big_motor->forceStopAndNewPosition(0); // force motor to stop, zero out position
          big_motor->moveTo(9999); // move in one direction
        } 
      }
    }

    if (initMotors)  // setup motors
    {
      initMotors = false;
      initComplete = true;
      big_motor->setSpeedInHz(bigmotorSpeed); // setup inital speed
      big_motor->setAcceleration(bigmotorAccel); // setup default acceleration
      small_motor->setSpeedInHz(smallmotorSpeed); // setup lube pump speed
      small_motor->setAutoEnable(true); // auto enable lube pump driver
      digitalWrite(BIG_MOTOR_SLEEP, HIGH); // enable driver
      ws.textAll("Motors initialized.");
      motEnabled = true; // enable motors
    }
    
    if (runTest) // run a self test on the big motor
    {
      if (!big_motor->isMotorRunning()) { // test to see if motor is done moving
        big_motor->setSpeedInHz(bigmotorSpeed);
        bigmotorDir = bigmotorDir ^ 1; // flip direction
        if (bigmotorDir) {
          localMove = bigmotorMove;
        } else {
          bigmotorSpeed = bigmotorSpeed + 100; // increase speed
          if (bigmotorSpeed > BIG_MOTOR_HZ) { // reset speed
            bigmotorSpeed = 100;
          }
          localMove = bigmotorMove * (-1);
          bigmotorMove = bigmotorMove + 50; // increase travel
          if (bigmotorMove > 1000) { // reset travel
            bigmotorMove = 50;
          }
        }
        
        big_motor->move(localMove);
        sprintf(tmpBuffer.msgArray, "speed %u moving %i", bigmotorSpeed, localMove);
        xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
        // ws.printfAll("speed %u moving %i", bigmotorSpeed, localMove);
      } 
    } // end of runtest

    if (listFiles) // send list of SPIFFs files to client
    {
      listFiles = false; // clear flag

      File root = SPIFFS.open("/");
    
      File file = root.openNextFile();
    
      while(file){
          const char* fName = file.name();
          ws.printfAll("Program file %s", fName);
    
          file = root.openNextFile();
      }
      
      file.close();
      root.close();
    }

    if (updateClient) // send some parameters to client on request, crashes ESP if run too often
    {
      updateClient = false;

      const char* mySwitches = "{\"switches\":{\"runtest\":%i,\"motenabled\":%i,\"autostroke\":%i,\"autolube\":%i,\"dualspeed\":%i}}";
      const char* myConfig = "{\"speedin\":%u,\"speedout\":%u,\"lubefreq\":%u,\"lubeamt\":%u,\"motaccel\":%u,\"strokedep\":%u,\"motspeed\":%u,\"strokelen\":%i}";
      // const char* myProg = "Program running %i step %u/%u";

      sprintf(tmpBuffer.msgArray, myConfig, bigmotorIn, bigmotorOut, lubeFreq, lubeAmt, bigmotorAccel, bigmotorDepth, bigmotorSpeed, bigmotorMove);
      xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
      
      sprintf(tmpBuffer.msgArray, mySwitches, runTest, motEnabled, autoStroke, autoLube, dualSpeed);
      xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     

      // sprintf(tmpBuffer.msgArray, myProg, progRunning, progPointer, progSteps);
      // xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     

      // debugging, print task stack utilization
      // uint32_t result = uxTaskGetStackHighWaterMark(NULL);
      // sprintf(tmpBuffer.msgArray, "Motor task high water mark %u", result);
      // xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
    }

    if (debugMotor) // send diagnostic info on motor speed, position
    {
      debugMotor = false; // clear flag
      const char* bigMotor = "Big motor running %i speed %i position %i target %i";
      const char* smallMotor = "Small motor running %i speed %i position %i target %i";

      ws.printfAll(bigMotor, big_motor->isRunning(), big_motor->getSpeedInUs(), big_motor->getCurrentPosition(), big_motor->targetPos());
      ws.printfAll(smallMotor, small_motor->isRunning(), small_motor->getSpeedInUs(), small_motor->getCurrentPosition(), small_motor->targetPos());
      // sprintf(tmpBuffer.msgArray, bigMotor, big_motor->isRunning(), big_motor->getSpeedInUs(), big_motor->getCurrentPosition(), big_motor->targetPos());
      // xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     

      // sprintf(tmpBuffer.msgArray, smallMotor, small_motor->isRunning(), small_motor->getSpeedInUs(), small_motor->getCurrentPosition(), small_motor->targetPos());
      // xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
    }

    if ((updateDepth == true) && (autoStroke != true) && (runTest != true)) // update depth position on big motor
    {
      updateDepth = false;
      if (progRunning) progNextStep = true; // set flag for next step as needed
      big_motor->moveTo(bigmotorDepth);
      sprintf(tmpBuffer.msgArray, "Moving to depth: %i", bigmotorDepth);
      xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
      // ws.printfAll("Moving to depth %i", bigmotorDepth);
    }

    if (updateSpeed) // update RPMs for big motor
    {
      updateSpeed = false;
      big_motor->setSpeedInHz(bigmotorSpeed);
      sprintf(tmpBuffer.msgArray, "Updated motor speed: %u", bigmotorSpeed);
      xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
    }

    // if (big_motor->isMotorRunning()) {
    //   Serial.printf("Motor position %i\n", big_motor->getCurrentPosition());
    // }

    if (autoStroke && doDelay==false) // reciprocate big motor 
    {
      if (initComplete && motEnabled) { // check if motors are setup first
        updateDepth = false;
        if (big_motor->getCurrentPosition() >= bigmotorMove + bigmotorDepth) {
          if (dualSpeed) big_motor->setSpeedInHz(bigmotorIn);
          big_motor->moveTo(bigmotorDepth); // return to home position
          sprintf(tmpBuffer.msgArray, "Stroke %u rep %u dual %i", strokeCnt, progRepCnt, dualSpeed);
          xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // add message to the transmit queue     
          if (!big_motor->isRunning()) { // set motor parameters if it's not running
            big_motor->setAcceleration(bigmotorAccel); // update acceleration
          }
          if (progRepCnt++>=progReps) { // done enough reps, time for next command
            progRepCnt=0; // reset counter
            progNextStep=true; // flag next command
          }
            // ws.printfAll("stroke %u complete", strokeCnt);
        } else if (big_motor->getCurrentPosition() <= bigmotorDepth) {
          if (dualSpeed) big_motor->setSpeedInHz(bigmotorOut);
          big_motor->moveTo(bigmotorMove + bigmotorDepth); // move to extended position
          strokeCnt++; // increment stroke counter
          if ((strokeCnt >= lubeFreq) && (autoLube)) {
            shootLube = true; // request lube
            strokeCnt = 0; // reset counter
          }
        } else if (updateStroke) { // update to new longer stroke length
          if (dualSpeed) big_motor->setSpeedInHz(bigmotorIn);
          big_motor->moveTo(bigmotorMove + bigmotorDepth); // move to extended position
          strokeCnt++; // increment stroke counter
          updateStroke = false;
        }
      } else if (!initComplete && motEnabled) { // setup motors first
        initMotors = true;
      }
    }

    if (shootLube) // dispense lubrication on demand
    {
      if (initComplete) { 
        shootLube = false;
        if (!small_motor->isRunning()) { // only run motor if it's not running already
          small_motor->setCurrentPosition(0); // zero out position
          small_motor->move(lubeAmt); // run motor so many steps
          sprintf(tmpBuffer.msgArray, "Dispensing lubricant %u", lubeAmt);
          xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
        }
      } else {
        initMotors = true;
      }
    }

    if (doDelay) // disable autostroke for a time period as requested by program
    {
      currentMillis = millis(); // update clock
      autoStroke = false; // disable autostroke
      if (currentMillis - previousMillis > delayMillis) { // delay is over
        doDelay = false;
        progNextStep = true; // ready for next command
        sprintf(tmpBuffer.msgArray, "Delay complete %ums", delayMillis);
        xQueueSend(wsoutQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the transmit queue     
      }
    }
    
    delay(5); // probably not needed?

  } 
}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    //client connected
    // Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    // Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
 
  } else if(type == WS_EVT_ERROR){
    //error was received from the other end
   //  Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } else if(type == WS_EVT_DATA){
    //data packet
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len){
      // the whole message is in a single frame and we got all of it's data
      // Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);
      if(info->opcode == WS_TEXT){ // dump text message to debug console
        data[len] = 0;
        // Serial.printf("%s\n", (char*)data);
      } else { // dump binary message to debug console
        for(size_t i=0; i < info->len; i++){
          // Serial.printf("%02x ", data[i]);
        }
        // Serial.printf("\n");
      }
      if(info->opcode == WS_TEXT) { // acknowledge text message
        client->text("I got your text message");
      } else { // acknowledge binary message
        // client->text("I got your binary message"); // send ack to client
        struct WSmsg tmpBuffer;
        if (sizeof(data) <= sizeof(tmpBuffer.msgArray)) { // array small enough to fit in queue
          for (size_t i=0; i < info->len; i++){
            tmpBuffer.msgArray[i] = data[i]; // copy received array into buffer
          }
          tmpBuffer.msgId = client->id(); // grab client id, just for giggles
          xQueueSend(wsmsgQueue, &tmpBuffer, (5 / portTICK_PERIOD_MS)); // pass pointer for the message to the queue
        }
      } 
    }
  }  
}

void setup(){
  Serial.begin(115200);
  Serial.println("Booting");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
    delay(5000);
  }

  ArduinoOTA
    .onStart([]() {
      String type;

      ws.textAll("Shutting down for OTA update");
      vTaskSuspend(taskStepper); // shutdown stepper motion task
      vTaskSuspend(taskMSG); // suspend message handler task

      big_motor->forceStopAndNewPosition(0);
      small_motor->forceStopAndNewPosition(0);

      server.end(); // shutdown webserver

      digitalWrite(BIG_MOTOR_SLEEP, LOW); // turn off driver
      digitalWrite(SMALL_MOTOR_SLEEP, LOW); // turn off driver

      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_SPIFFS 
        type = "filesystem";
        SPIFFS.end(); // unmount filesystem
    }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      // Serial.println("OTA Update: " + type);
    })
    .onEnd([]() {
      // Serial.println("\nEnd");
      ESP.restart();
      delay(5000);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      // Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      ESP.restart();
      delay(5000);
    });

  ArduinoOTA.setHostname(hostName);
  ArduinoOTA.begin();

  MDNS.addService("http","tcp",80);

  SPIFFS.begin();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  events.onConnect([](AsyncEventSourceClient *client){
    client->send("hello!",NULL,millis(),1000);
  });

  server.addHandler(&events);

  server.addHandler(new SPIFFSEditor(SPIFFS, http_username, http_password));

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.onNotFound([](AsyncWebServerRequest *request){
    // handle not found
    request->send(404, "text/plain", "Not found");
  });

  server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
   // if(!index)
      // Serial.printf("UploadStart: %s\n", filename.c_str());
    // Serial.printf("%s", (const char*)data);
    // if(final)
      // Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
  });

  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    // handle it?
  });
  
  server.begin();
  
  // Serial.println("Ready");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  engine.init();
  big_motor = engine.stepperConnectToPin(BIG_MOTOR_STEP);
  small_motor = engine.stepperConnectToPin(SMALL_MOTOR_STEP);

  if (big_motor) { // set up drive motor
    big_motor->setDirectionPin(BIG_MOTOR_DIR, false); // inverted direction pin
    // big_motor->setEnablePin(BIG_MOTOR_SLEEP); // commented out, will manually control sleep
    // big_motor->setAutoEnable(false);

    big_motor->setSpeedInHz(BIG_MOTOR_HZ);      
    big_motor->setAcceleration(BIG_MOTOR_ACCEL);   
  } 

  if (small_motor) { // set up small motor
    small_motor->setEnablePin(SMALL_MOTOR_SLEEP);
    small_motor->setAutoEnable(true);

    small_motor->setSpeedInHz(SMALL_MOTOR_HZ);      
    small_motor->setAcceleration(SMALL_MOTOR_ACCEL);   
  }

  digitalWrite(SMALL_MOTOR_SLEEP, LOW); // turn off driver

  // small_motor->move(1000); // test motor
  // big_motor->move(1000); // test motor

  motQueue = xQueueCreate(3, sizeof(struct MOTcmd) );

  wsmsgQueue = xQueueCreate(3, sizeof(struct WSmsg));

  wsoutQueue = xQueueCreate(2, sizeof(struct WSmsg));

  syncSW1   = xSemaphoreCreateBinary();
  syncSW2   = xSemaphoreCreateBinary();
  syncESTOP = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(
    runStepper, /* Function to implement the task */
    "taskStepper", /* Name of the task */
    5000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &taskStepper,  /* Task handle. */
    0); /* Core where the task should run */

   xTaskCreatePinnedToCore(
     wsMsgtask, /* Function to implement the task */
     "taskMSG", /* Name of the task */
     5000,  /* Stack size in words */
     NULL,  /* Task input parameter */
     0,  /* Priority of the task */
     &taskMSG,  /* Task handle. */
     1); /* Core where the task should run */

    xTaskCreatePinnedToCore(
     watchSW1, /* Function to implement the task */
     "taskSW1", /* Name of the task */
     1000,  /* Stack size in words */
     NULL,  /* Task input parameter */
     0,  /* Priority of the task */
     &taskSW1,  /* Task handle. */
     1); /* Core where the task should run */

    xTaskCreatePinnedToCore(
     watchSW2, /* Function to implement the task */
     "taskSW2", /* Name of the task */
     1000,  /* Stack size in words */
     NULL,  /* Task input parameter */
     0,  /* Priority of the task */
     &taskSW2,  /* Task handle. */
     1); /* Core where the task should run */

    xTaskCreatePinnedToCore(
     watchESTOP, /* Function to implement the task */
     "taskESTOP", /* Name of the task */
     1000,  /* Stack size in words */
     NULL,  /* Task input parameter */
     0,  /* Priority of the task */
     &taskESTOP,  /* Task handle. */
     1); /* Core where the task should run */
}

void loop() // main task loop, used for housekeeping and ota
{
  ArduinoOTA.handle();
  ws.cleanupClients(); // clean up any disconnected clients

  delay(10);
}

