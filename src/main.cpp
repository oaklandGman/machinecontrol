#include <Arduino.h>
#include <ArduinoOTA.h>
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
const char * hostName = "machine";
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

unsigned int bigmotorSpeed = 100;
unsigned int smallmotorSpeed = SMALL_MOTOR_HZ;
unsigned int bigmotorMove = 50;
bool bigmotorDir = true;
uint8_t bigmotorTest = 0; // no test by default

TaskHandle_t taskStepper;
TaskHandle_t taskOTA;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *big_motor = NULL;
FastAccelStepper *small_motor = NULL;

void runStepper(void * parameter) {
  unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  uint localMove = 0;

  for (;;) { // loop forever
    currentMillis = millis();  
    
    if (currentMillis - previousMillis >= 500) {
      previousMillis = millis();
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
        Serial.printf("speed %u moving %i\n", bigmotorSpeed, localMove);
        ws.printfAll("speed %u moving %i", bigmotorSpeed, localMove);
      } else {
        // Serial.println("M");
        // ws.textAll("M");
      }

      // if (!small_motor->isMotorRunning()) {
      //   // Serial.println("Small motor done!");
      //   Serial.println("Small motor moving 1000 steps.");
      //   small_motor->move(2000);
      // } else {
      //   Serial.println("L");
      // }
    }

    delay(5);

  } 
}

void runOTA(void * parameter) {
}


void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    //client connected
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Hello Client %u :)", client->id());
    client->ping();
  } else if(type == WS_EVT_DISCONNECT){
    Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
 
  } else if(type == WS_EVT_ERROR){
    //error was received from the other end
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
  } 
}

void setup(){
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  server.on("/hello", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Hello World");
  });
   
  ArduinoOTA
    .onStart([]() {
      String type;

      vTaskSuspend(taskStepper); // shutdown stepper motion task

      big_motor->forceStopAndNewPosition(0);
      small_motor->forceStopAndNewPosition(0);

      ws.textAll("Shutting down for OTA update");
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
      Serial.println("OTA Update: " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      ESP.restart();
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
      ESP.restart();
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

  server.addHandler(new SPIFFSEditor(SPIFFS, http_username,http_password));

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.onNotFound([](AsyncWebServerRequest *request){
    Serial.printf("NOT_FOUND: ");
    if(request->method() == HTTP_GET)
      Serial.printf("GET");
    else if(request->method() == HTTP_POST)
      Serial.printf("POST");
    else if(request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if(request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if(request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if(request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if(request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if(request->contentLength()){
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for(i=0;i<headers;i++){
      AsyncWebHeader* h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for(i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isFile()){
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if(p->isPost()){
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    request->send(404);
  });
  server.onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index)
      Serial.printf("UploadStart: %s\n", filename.c_str());
    Serial.printf("%s", (const char*)data);
    if(final)
      Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
  });
  server.onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if(!index)
      Serial.printf("BodyStart: %u\n", total);
    Serial.printf("%s", (const char*)data);
    if(index + len == total)
      Serial.printf("BodyEnd: %u\n", total);
  });
  
  server.begin();
  
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

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

  xTaskCreatePinnedToCore(
    runStepper, /* Function to implement the task */
    "taskStepper", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &taskStepper,  /* Task handle. */
    0); /* Core where the task should run */

  //  xTaskCreatePinnedToCore(
  //    runOTA, /* Function to implement the task */
  //    "taskOTA", /* Name of the task */
  //    10000,  /* Stack size in words */
  //    NULL,  /* Task input parameter */
  //    0,  /* Priority of the task */
  //    &taskOTA,  /* Task handle. */
  //    0); /* Core where the task should run */
}

void loop(){
  ArduinoOTA.handle();
  ws.cleanupClients(); // clean up any disconnected ws clients
  
  delay(10);
}

