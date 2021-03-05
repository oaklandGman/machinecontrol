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
const char* hostName = "machine";

// IO pin assignments
const int BIG_MOTOR_STEP = 19;
const int BIG_MOTOR_DIR = 21;
const int BIG_MOTOR_SLEEP = 22;
const int SMALL_MOTOR_STEP = 25;
const int SMALL_MOTOR_DIR = 23;
const int SMALL_MOTOR_SLEEP = 26;
const int EMERGENCY_STOP_PIN = 35; //define the IO pin the emergency stop switch is connected to
const int LIMIT_SW1 = 33;
const int LIMIT_SW2 = 27;

// Speed settings
const unsigned int BIG_MOTOR_HZ = 9000;
const unsigned int BIG_MOTOR_ACCEL = 40000;
const unsigned int SMALL_MOTOR_HZ = 200;
const unsigned int SMALL_MOTOR_ACCEL = 4294967295;

unsigned int smallmotorSpeed = SMALL_MOTOR_HZ;
struct WSmsg // structure for msgpack messages to send/recv over websocket
{
  uint8_t msgId;
  char msgArray[201]; // 201 byte array for a message, hope that's enough!
};
struct MOTcmd // structure to define commands for stepper motor control
{
  char cmd[16]; // command string
  int dat; // four bytes of data
};

TaskHandle_t taskStepper; // handle for stepper task
TaskHandle_t taskMSG; // handle for websocket message handling 

QueueHandle_t motQueue; // handle for motor command queue
QueueHandle_t wsmsgQueue; // handle for websocket message queue
QueueHandle_t wsoutQueue;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *big_motor = NULL;
FastAccelStepper *small_motor = NULL;

void wsMsgtask(void * parameter) {
  struct WSmsg data; // buffer for the message data
  struct MOTcmd motor; // buffer for motor control message
  struct WSmsg msgout; // buffer for outgoing messages

  unsigned long lastMillis = millis();

  for (;;) { // infinite loop
    // check receive queue every cycle
    if ( uxQueueMessagesWaiting(wsmsgQueue) ) { // message in the queue, dump it to debug console
      xQueueReceive(wsmsgQueue, &data, portMAX_DELAY); // grab message from queue
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
        ws.printfAll("Processing command for fuction %s", fnc);
        if (strcmp("motor", fnc)==0) {
          // if (fnc == 0x4d) { // M for motor
          strcpy(motor.cmd, doc["cmd"]); // copy json string value into buffer
          motor.dat = doc["dat"]; // assign integer value

          xQueueSend(motQueue, &motor, portMAX_DELAY); // put command on motor control queue
        } 
      }

      // pretty print to serial console
      serializeJsonPretty(doc, Serial);
        
    }  // end receive queue check

    if (millis() - lastMillis >= 500) { // check transmit queue every 500ms
      if ( uxQueueMessagesWaiting(wsoutQueue) ) { // message in the queue, dump it to debug console
        // StaticJsonDocument<200> doc;
        xQueueReceive(wsoutQueue, &data, portMAX_DELAY); // grab message from queue

        ws.textAll(data.msgArray);

        lastMillis = millis();
      }
    } // end transmit queue check

  }
}

void runStepper(void * parameter) {
  bool runTest = false;
  bool bigmotorDir = false;
  bool autoLube = false;
  bool autoStroke = false;
  bool shootLube = false;
  bool bigmotorCal = false;
  bool runCal = false;

  int bigmotorSpeed = BIG_MOTOR_HZ; // default value
  int bigmotorAccel = BIG_MOTOR_ACCEL; // default value
  int bigmotorMove = 200;
  int bigmotorDepth = 0;
  int lubeAmt = SMALL_MOTOR_HZ;

  uint localMove = 0;
  struct MOTcmd command;
  struct WSmsg tmpBuffer;

  // loop forever
  for (;;) { 
    // currentMillis = millis();  
    
    // check queue for new commands
    if (uxQueueMessagesWaiting(motQueue)) {
      // queue not empty, see if anything in here is for me
      xQueueReceive(motQueue, &command, portMAX_DELAY);

      const char* cmd = command.cmd;
      int dat = command.dat;
      
      ws.printfAll("Received command %s data %i", cmd, dat);

      if (strcmp("motortest", cmd) == 0 ) { // command "test"
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
          // another command
        } else if (strcmp("autolube", cmd) == 0 ) { // command "autolube"
          if (dat == 1) autoLube = true;
          else autoLube = false;
        } else if (strcmp("shootlube", cmd) == 0 ) { // command "shootlube" 
          // another command
        } else if (strcmp("calibrate", cmd) == 0 ) { // command "calibrate" 
          // another command
        } else if (strcmp("strokelen", cmd) == 0 ) { // command "strokelen" 
          // another command
        } else if (strcmp("strokedep", cmd) == 0 ) { // command "strokedep" 
          // another command
        } else if (strcmp("motspeed", cmd) == 0 ) { // command "motspeed" 
          // another command
        } else if (strcmp("motaccel", cmd) == 0 ) { 
          bigmotorAccel = dat; // update big motor accelleration 
          // another command
        } else if (strcmp("lubeamt", cmd) == 0 ) { 
          lubeAmt = dat; // update number of steps lube motor will run
        }
      }
    } // end message queue check
    
    if (runTest) {
      if (!big_motor->isMotorRunning()) { // test to see if motor is done moving
        // Serial.println("Big motor done!");
        // ws.textAll("Big motor done!");

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
        // Serial.printf("speed %u moving %i\n", bigmotorSpeed, localMove);
        sprintf(tmpBuffer.msgArray, "{\"lubeamt\":%u,\"motaccel\":%u,\"strokedep\":%u,\"motspeed\":%u,\"strokelen\":%i}", lubeAmt, bigmotorAccel, bigmotorDepth, bigmotorSpeed, localMove);
        xQueueSend(wsoutQueue, &tmpBuffer, (portTICK_PERIOD_MS * 5)); // pass pointer for the message to the transmit queue
      } 
    } // end of runtest


    // delay(5);

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
          xQueueSend(wsmsgQueue, &tmpBuffer, (portTICK_PERIOD_MS * 5)); // pass pointer for the message to the queue
        }
      } 
    }
  }  
}

void setup(){
  // Serial.begin(115200);
  // Serial.println("Booting");
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
    big_motor->setDirectionPin(BIG_MOTOR_DIR);
    big_motor->setEnablePin(BIG_MOTOR_SLEEP);
    big_motor->setAutoEnable(false);

    big_motor->setSpeedInHz(BIG_MOTOR_HZ);      
    big_motor->setAcceleration(BIG_MOTOR_ACCEL);   
  } 

  if (small_motor) { // set up small motor
    small_motor->setDirectionPin(SMALL_MOTOR_DIR);
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

  xTaskCreatePinnedToCore(
    runStepper, /* Function to implement the task */
    "taskStepper", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &taskStepper,  /* Task handle. */
    0); /* Core where the task should run */

   xTaskCreatePinnedToCore(
     wsMsgtask, /* Function to implement the task */
     "taskMSG", /* Name of the task */
     10000,  /* Stack size in words */
     NULL,  /* Task input parameter */
     0,  /* Priority of the task */
     &taskMSG,  /* Task handle. */
     1); /* Core where the task should run */
}

void loop(){
  ArduinoOTA.handle();
  ws.cleanupClients(); // clean up any disconnected ws clients

  delay(10);
}

