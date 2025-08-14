/*  ========================================================================================================================================
    MrDIY Audio Notifier is a cloud-free media notifier that can play MP3s, stream icecast radios, read text 
    and play RTTTL ringtones. It is controller over MQTT.
	Notifier Ergänzung durch pedrosalino :Version  "Gollino 3LED Notifier 2e2"   3LED Blink bei Sprachwiedergabe mp3 Gollino oder
	RTTTl und mp3 Musik ("song" am Anfang des Dateinamens). Zusätzlich mqtt für led1,2,3, zusätzlich mqtt für Led4,5
	5 Scripte über IObroker Blockly:     1 Pausenmusik, 2 Gollino Texte bei Motion
	                                     3 Motion umschalten   4 LED für Pausenmusik  5 LED für Gollino Texte

      Option 1) Connect your audio jack to the Rx and GN pins. Then connect the audio jack to an external amplifier. 
      Option 2) Use an extenal DAC (like MAX98357A) and uncomment "#define USE_I2S" in the code.


    + MQTT COMMANDS:  ( 'your_custom_mqtt_topic' is the MQTT prefix defined during the setup)

      - Play an MP3             MQTT topic: "your_custom_mqtt_topic/play"
                                MQTT load: http://url-to-the-mp3-file/file.mp3 
                                PS: supports HTTP only, no HTTPS.  

      - Play an Icecast Stream  MQTT topic: "your_custom_mqtt_topic/stream"
                                MQTT load: http://url-to-the-icecast-stream/file.mp3, example: http://22203.live.streamtheworld.com/WHTAFM.mp3

      - Play a Ringtone         MQTT topic: "your_custom_mqtt_topic/tone"
                                MQTT load: RTTTL formated text, example: Soap:d=8,o=5,b=125:g,a,c6,p,a,4c6,4p,a,g,e,c,4p,4g,a

      - Say Text                MQTT topic: "your_custom_mqtt_topic/say"
                                MQTT load: Text to be read, example: Hello There. How. Are. You?

      - Stop Playing            MQTT topic: "your_custom_mqtt_topic/stop"
      
      - Change the Volume       MQTT topic: "your_custom_mqtt_topic/volume"
                                MQTT load: a double between 0.00 and 1.00, example: 0.7


    + STATUS UPDATES:

     - The notifier sends status updates on MQTT topic: "your_custom_mqtt_topic/status" with these values:

                  "playing"       either paying an mp3, a ringtone or saying a text
                  "idle"          idle and waiting for a command
                 // "error"         error when receiving a command: example: MP3 file URL can't be loaded"
                  
     - The LWT MQTT topic: "your_custom_topic/LWT" with values:
                   "online"       
                   "offline"    
                                                      
     - At boot, the notifier plays a 2 second audio chime when it is successfully connected to Wifi & MQTT


    + SETUP:
     
      To Upload to an ESP8266 module or board:
      
        - Set CPU Frequency to 160MHz ( Arduino IDE: Tools > CPU Frequency )
        - Set IwIP to V2 Higher Bandwidth ( Arduino IDE: Tools > IwIP Variant )
        - Press "Upload"


    + DEPENDENCIES:

     - ESP8266          https://github.com/esp8266/Arduino
     - ESP8266Audio     https://github.com/earlephilhower/ESP8266Audio
     - ESP8266SAM       https://github.com/earlephilhower/ESP8266SAM
     - IotWebConf       https://github.com/prampec/IotWebConf
     - PubSubClient     https://github.com/knolleary/pubsubclient


    Many thanks to all the authors and contributors to the above libraries - you have done an amazing job!

    For more info, please watch my instruction video at https://youtu.be/SPa9SMyPU58
    MrDIY.ca
    
  ============================================================================================================== */

//#define DEBUG_FLAG              // uncomment to enable debug mode & Serial output
#define USE_I2S                 // uncomment to use extenral I2S DAC 

#include "Arduino.h"
#include "boot_sound.h"
#include "ESP8266WiFi.h"
#include <PubSubClient.h>

bool lastMotionState = false;  // global definieren
WiFiClient espClient;
PubSubClient client(espClient);  // Das ist dein MQTT-Client

#include "AudioFileSourceHTTPStream.h"
#include "AudioFileSourceICYStream.h"
#include "AudioFileSourcePROGMEM.h"
#include "AudioFileSourceBuffer.h"
#include "AudioGeneratorMP3.h"
#include "AudioGeneratorWAV.h"
#include "AudioGeneratorRTTTL.h"
#ifdef USE_I2S
#include "AudioOutputI2S.h"
#else
#include "AudioOutputI2SNoDAC.h"
#endif
#include "ESP8266SAM.h"
#include "IotWebConf.h"
#include "IotWebConfUsing.h"

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

AudioGeneratorMP3         *mp3 = NULL;
AudioGeneratorWAV         *wav = NULL;
AudioGeneratorRTTTL       *rtttl = NULL;
AudioFileSourceHTTPStream *file_http = NULL;
AudioFileSourcePROGMEM    *file_progmem = NULL;
AudioFileSourceICYStream  *file_icy = NULL;
AudioFileSourceBuffer     *buff = NULL;
#ifdef USE_I2S
AudioOutputI2S            *out = NULL;
#else
AudioOutputI2SNoDAC       *out = NULL;
#endif

WiFiClient                wifiClient;
PubSubClient              mqttClient(wifiClient);
#define  port             1883
#define  MQTT_MSG_SIZE    256
const char* willTopic     = "LWT";
const char* willMessage   = "offline";
boolean willRetain        = false;
byte willQoS              = 0;

// AudioRelated ---------------------------
float volume_level              = 0.8;
String playing_status;
const int preallocateBufferSize = 2048;
void *preallocateBuffer = NULL;

// WifiManager -----------------------------
#ifdef ESP8266
String ChipId = String(ESP.getChipId(), HEX);
#elif ESP32
String ChipId = String((uint32_t) ESP.getEfuseMac(), HEX);
#endif
String thingName = String("MrDIY Notifier - ") + ChipId;
#define wifiInitialApPassword "mrdiy.ca"
char mqttServer[16];
char mqttUserName[32];
char mqttUserPassword[32];
char mqttTopicPrefix[32];
char mqttTopic[MQTT_MSG_SIZE];

DNSServer             dnsServer;
WebServer             server(80);
IotWebConf            iotWebConf(thingName.c_str(), &dnsServer, &server, wifiInitialApPassword, "MrD3");
iotwebconf::ParameterGroup mqttgroup = iotwebconf::ParameterGroup("mqttgroup", "");
iotwebconf::TextParameter mqttServerParam = iotwebconf::TextParameter("MQTT server", "mqttServer", mqttServer, sizeof(mqttServer));
iotwebconf::TextParameter mqttUserNameParam = iotwebconf::TextParameter("MQTT username", "mqttUser", mqttUserName, sizeof(mqttUserName));
iotwebconf::PasswordParameter mqttUserPasswordParam = iotwebconf::PasswordParameter("MQTT password", "mqttPass", mqttUserPassword, sizeof(mqttUserPassword), "password");
iotwebconf::TextParameter mqttTopicParam = iotwebconf::TextParameter("MQTT Topic", "mqttTopic", mqttTopicPrefix, sizeof(mqttTopicPrefix));

#define PIR_PIN D1


const int LED1_Pin = D5;  // z. B. GPIO14
const int LED2_Pin = D6;  // z. B. GPIO12
const int LED3_Pin = D7;  // z. B. GPIO13
const int LED4_Pin = D0;  // z. B. GPIO16
const int LED5_Pin = D2;  // z. B. GPIO4


unsigned long lastBlink1 = 0;
unsigned long lastBlink2 = 0;
unsigned long lastBlink3 = 0;

bool ledState1 = false; // deklaration ledstate(x) boulean
bool ledState2 = false;
bool ledState3 = false;

bool shouldBlink = false;
bool isMP3Playing = false;

String currentMp3Filename = "";

String extractFilename(String url) {
  int lastSlash = url.lastIndexOf('/');
  if (lastSlash == -1) return url;
  return url.substring(lastSlash + 1);
}

void callback(char* topic, byte* payload, unsigned int length) {
  // 1. Payload in char-Array kopieren und terminieren
  char msg[length + 1];
  strncpy(msg, (char*)payload, length);
  msg[length] = '\0';

  // 2. Debug-Ausgabe
  Serial.printf("Received MQTT: [%s] %s\n", topic, msg);

  // 3. LED Control Handling
  if (strcmp(topic, mqttFullTopic("led1")) == 0) {
    bool state = (strcmp(msg, "1") == 0 || strcmp(msg, "on") == 0);
    digitalWrite(LED1_Pin, state);
    Serial.printf("Set LED1: %s\n", state ? "ON" : "OFF");
    mqttClient.publish(mqttFullTopic("led1/status"), msg, true);
    return;
  }
  else if (strcmp(topic, mqttFullTopic("led2")) == 0) {
    bool state = (strcmp(msg, "1") == 0 || strcmp(msg, "on") == 0);
    digitalWrite(LED2_Pin, state);
    Serial.printf("Set LED2: %s\n", state ? "ON" : "OFF");
    mqttClient.publish(mqttFullTopic("led2/status"), msg, true);
    return;
  }
  else if (strcmp(topic, mqttFullTopic("led3")) == 0) {
    bool state = (strcmp(msg, "1") == 0 || strcmp(msg, "on") == 0);
    digitalWrite(LED3_Pin, state);
    Serial.printf("Set LED3: %s\n", state ? "ON" : "OFF");
    mqttClient.publish(mqttFullTopic("led3/status"), msg, true);
    return;
  }
 else if (strcmp(topic, mqttFullTopic("led4")) == 0) {
    bool state = (strcmp(msg, "1") == 0 || strcmp(msg, "on") == 0);
    digitalWrite(LED4_Pin, state);
    Serial.printf("Set LED4: %s\n", state ? "ON" : "OFF");
    mqttClient.publish(mqttFullTopic("led4/status"), msg, true);
    return;
  }
 else if (strcmp(topic, mqttFullTopic("led5")) == 0) {
    bool state = (strcmp(msg, "1") == 0 || strcmp(msg, "on") == 0);
    digitalWrite(LED5_Pin, state);
    Serial.printf("Set LED5: %s\n", state ? "ON" : "OFF");
    mqttClient.publish(mqttFullTopic("led5/status"), msg, true);
    return;
  }
  // 4. Andere MQTT-Nachrichten weiterreichen (z.B. Audio)
  Serial.println("Passing to audio handler");
  onMqttMessage(topic, payload, length);
}
/* ################################## Setup ############################################# */

void setup() {

  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);

  pinMode(LED1_Pin, OUTPUT);  // D1
  pinMode(LED2_Pin, OUTPUT);  // D5
  pinMode(LED3_Pin, OUTPUT);  // D6
  pinMode(LED4_Pin, OUTPUT);  // D0
  pinMode(LED5_Pin, OUTPUT);  // D2
  digitalWrite(LED1_Pin, LOW);
  digitalWrite(LED2_Pin, LOW);
  digitalWrite(LED3_Pin, LOW);  
  digitalWrite(LED4_Pin, LOW);
  digitalWrite(LED5_Pin, LOW);


  mqttClient.setCallback(callback);

  mqttgroup.addItem(&mqttServerParam);
  mqttgroup.addItem(&mqttUserNameParam);
  mqttgroup.addItem(&mqttUserPasswordParam);
  mqttgroup.addItem(&mqttTopicParam);
  iotWebConf.addParameterGroup(&mqttgroup);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);
  iotWebConf.setFormValidator(&formValidator);
#ifdef LED_Pin
  iotWebConf.setStatusPin(LED_Pin);
#endif
  iotWebConf.skipApStartup();

  boolean validConfig = iotWebConf.init();
  if (!validConfig) {
    mqttServer[0] = '\0';
    mqttUserName[0] = '\0';
    mqttUserPassword[0] = '\0';
  }

  server.on("/", [] { iotWebConf.handleConfig(); });
  server.onNotFound([] { iotWebConf.handleNotFound(); });

#ifdef USE_I2S
  out = new AudioOutputI2S();
  Serial.println("Using I2S output");
#else
  out = new AudioOutputI2SNoDAC();
  Serial.println("Using No DAC - using Serial port Rx pin");
#endif
  out->SetGain(volume_level);
}

/* ##################################### Loop ############################################# */

void loop() {
  // 1. PIR-Sensor auslesen
  bool motion = digitalRead(PIR_PIN);

  // 2. Bei Zustandsänderung MQTT-Nachricht senden
  if (motion != lastMotionState) {
    lastMotionState = motion;
    Serial.print("PIR-Status geändert: ");
    Serial.println(motion);
    
    // Nur EIN MQTT-Publish mit Retain-Flag
    mqttClient.publish(mqttFullTopic("motion"), motion ? "1" : "0", true);
    
    // Debug-Ausgabe
    Serial.println(motion ? "Bewegung!" : "Keine Bewegung");
    delay(500);  // Entprellung
  }

  // MQTT-Verbindung aufrechterhalten
  if (!mqttClient.connected()) {
    mqttReconnect();
  }
  mqttClient.loop();

  // Audio-Verarbeitung
  if (!mp3) iotWebConf.doLoop();
  if (mp3 && !mp3->loop()) stopPlaying();
  if (wav && !wav->loop()) stopPlaying();
  if (rtttl && !rtttl->loop()) stopPlaying();

  // LED-Blinksteuerung
  unsigned long currentMillis = millis();
  if (mp3 && mp3->isRunning() && shouldBlink) {
    if (currentMillis - lastBlink1 >= 100) {
      lastBlink1 = currentMillis;
      ledState1 = !ledState1;
      digitalWrite(LED1_Pin, ledState1);
    }

    if (currentMillis - lastBlink2 >= 200) {
      lastBlink2 = currentMillis;
      ledState2 = !ledState2;
      digitalWrite(LED2_Pin, ledState2);
    }

    if (currentMillis - lastBlink3 >= 300) {
      lastBlink3 = currentMillis;
      ledState3 = !ledState3;
      digitalWrite(LED3_Pin, ledState3);
    }
  } else {
    digitalWrite(LED1_Pin, LOW);
    digitalWrite(LED2_Pin, LOW);
    digitalWrite(LED3_Pin, LOW);
  }

#ifdef DEBUG_FLAG
  if (mp3 && mp3->isRunning()) {
    static int unsigned long lastms = 0;
    if (millis() - lastms > 1000) {
      lastms = millis();
      Serial.print(F("Free: "));
      Serial.print(ESP.getFreeHeap(), DEC);
      Serial.print(F("  ("));
      Serial.print(ESP.getFreeHeap() - ESP.getMaxFreeBlockSize(), DEC);
      Serial.print(F(" lost)"));
      Serial.print(F("  Fragmentation: "));
      Serial.print(ESP.getHeapFragmentation(), DEC);
      Serial.print(F("%"));
      if (ESP.getHeapFragmentation() > 40) Serial.print(F("  ----------- "));
      Serial.println();
    }
  }
#endif
}

/* ############################### Audio ############################################ */

void playBootSound() {

  file_progmem = new AudioFileSourcePROGMEM(boot_sound, sizeof(boot_sound));
  wav = new AudioGeneratorWAV();
  wav->begin(file_progmem, out);
}

void stopPlaying() {

  if (mp3) {
#ifdef DEBUG_FLAG
    Serial.print(F("...#"));
    Serial.println(F("Interrupted!"));
    Serial.println();
#endif
    mp3->stop();
    delete mp3;
    mp3 = NULL;
    currentMp3Filename = "";
  }
  if (wav) {
    wav->stop();
    delete wav;
    wav = NULL;
  }
  if (rtttl) {
    rtttl->stop();
    delete rtttl;
    rtttl = NULL;
  }
  if (buff) {
    buff->close();
    delete buff;
    buff = NULL;
  }
  if (file_http) {
    file_http->close();
    delete file_http;
    file_http = NULL;
  }
  if (file_progmem) {
    file_progmem->close();
    delete file_progmem;
    file_progmem = NULL;
  }
  if (file_icy) {
    file_icy->close();
    delete file_icy;
    file_icy = NULL;
  }
  broadcastStatus("status", "idle");
  updateLEDBrightness(100);
}

/* ################################## MQTT ############################################### */

void onMqttMessage(char* topic, byte* payload, unsigned int mlength)  {
 
 char newMsg[MQTT_MSG_SIZE];

if (mlength > 0) {
  memset(newMsg, '\0', sizeof(newMsg));     // Alles nullen
  memcpy(newMsg, payload, mlength);         // Bytes kopieren
  newMsg[mlength] = '\0';                   // Sicherheitshalber nullterminieren!
}

#ifdef DEBUG_FLAG
    Serial.println();
    Serial.print(F("Requested ["));
    Serial.print(topic);
    Serial.print(F("] "));
    Serial.println(newMsg);
#endif
    // got a new URL to play ------------------------------------------------
if (!strcmp(topic, mqttFullTopic("play"))) {
  stopPlaying();
  file_http = new AudioFileSourceHTTPStream();
  if (file_http->open(newMsg)) {
    currentMp3Filename = extractFilename(String(newMsg)); // hier speichern
    shouldBlink = !currentMp3Filename.substring(0, 4).equalsIgnoreCase("song");
    
    // 🔽 Dateiname prüfen
    if (currentMp3Filename.startsWith("song")) {
      shouldBlink = false;
    } else {
      shouldBlink = true;
    }

    broadcastStatus("status", "playing");
    updateLEDBrightness(50);

    buff = new AudioFileSourceBuffer(file_http, preallocateBuffer, preallocateBufferSize);
    mp3 = new AudioGeneratorMP3();
    mp3->begin(buff, out);
  } else {
    stopPlaying();
    currentMp3Filename = "";
    shouldBlink = false;  // sicherheitshalber
    broadcastStatus("status", "error");
    broadcastStatus("status", "idle");
  }
}


if (!strcmp(topic, mqttFullTopic("stream"))) {
  stopPlaying();
  file_icy = new AudioFileSourceICYStream();
  if (file_icy->open(newMsg)) {
    currentMp3Filename = extractFilename(String(newMsg)); // speichern
    broadcastStatus("status", "playing");
    updateLEDBrightness(50);
    buff = new AudioFileSourceBuffer(file_icy, preallocateBuffer, preallocateBufferSize);
    mp3 = new AudioGeneratorMP3();
    mp3->begin(buff, out);
  } else {
    stopPlaying();
    currentMp3Filename = "";
    broadcastStatus("status", "error");
    broadcastStatus("status", "idle");
  }
}


// Bei tone (RTTTL) kein LED Blinken
if ( !strcmp(topic, mqttFullTopic("tone") ) ) {
  stopPlaying();
  isMP3Playing = false;
  broadcastStatus("status", "playing");
  updateLEDBrightness(50);
  file_progmem = new AudioFileSourcePROGMEM( newMsg, sizeof(newMsg) );
  rtttl = new AudioGeneratorRTTTL();
  rtttl->begin(file_progmem, out);
  broadcastStatus("status", "idle");
}

 // TTS mit LED-Blinken während Say
if (!strcmp(topic, mqttFullTopic("say"))) {
  stopPlaying();
  broadcastStatus("status", "playing");
  updateLEDBrightness(50);

  shouldBlink = true;  // aktivieren, falls loop() noch was tun kann (optional)

  ESP8266SAM *sam = new ESP8266SAM;

  // LED-Blinken manuell während TTS
  unsigned long duration = strlen(newMsg) * 100;  // z. B. 100 ms pro Zeichen
  unsigned long ttsStart = millis();
  unsigned long blinkLast = 0;
  bool ledState = false;

  // Start Ausgabe (blockiert!)
  sam->Say(out, newMsg);

  // Simulierter Blink während Say (da Say blockiert!)
  while (millis() - ttsStart < duration) {
    if (millis() - blinkLast > 100) {
      blinkLast = millis();
      ledState = !ledState;
      digitalWrite(LED1_Pin, ledState);
      digitalWrite(LED2_Pin, ledState);
      digitalWrite(LED3_Pin, ledState);
    }
    delay(10);  // Mini-Pause, damit WiFi nicht stirbt
  }

  // LEDs aus
  digitalWrite(LED1_Pin, LOW);
  digitalWrite(LED2_Pin, LOW);
  digitalWrite(LED3_Pin, LOW);

  delete sam;

  shouldBlink = false;
  stopPlaying();
  broadcastStatus("status", "idle");
}

// Bei stop kein LED Blinken
if ( !strcmp(topic, mqttFullTopic("stop"))) {
  stopPlaying();
  isMP3Playing = false;
}

    // got a volume request, expecting double [0.0,1.0] ---------------------
    if ( !strcmp(topic, mqttFullTopic("volume"))) {
      volume_level = atof(newMsg);
      if ( volume_level < 0.0 ) volume_level = 0;
      if ( volume_level > 1.0 ) volume_level = 0.7;
      out->SetGain(volume_level);
    }

    // got a stop request  --------------------------------------------------
    if ( !strcmp(topic, mqttFullTopic("stop"))) {
      stopPlaying();
  }
}

void broadcastStatus(const char topic[], String msg) {

  if ( playing_status != msg) {
    char charBuf[msg.length() + 1];
    msg.toCharArray(charBuf, msg.length() + 1);
    mqttClient.publish(mqttFullTopic(topic), charBuf);
    playing_status = msg;
#ifdef DEBUG_FLAG
    Serial.print(F("[MQTT] "));
    Serial.print(mqttTopicPrefix);
    Serial.print(F("/"));
    Serial.print(topic);
    Serial.print(F("\t\t"));
    Serial.println(msg);
#endif
  }
}


void updateLEDBrightness(int brightness_percentage) {
#ifdef LED_Pin
  analogWrite(LED_Pin, (int) brightness_percentage * 255 / 100);
#endif
}

void mqttReconnect() {

  if (!mqttClient.connected()) {

    updateLEDBrightness(0);

    if (mqttClient.connect(thingName.c_str(), mqttUserName, mqttUserPassword, mqttFullTopic(willTopic), willQoS, willRetain, willMessage)) {
      mqttClient.subscribe(mqttFullTopic("play"));
      mqttClient.subscribe(mqttFullTopic("stream"));
      mqttClient.subscribe(mqttFullTopic("tone"));
      mqttClient.subscribe(mqttFullTopic("say"));
      mqttClient.subscribe(mqttFullTopic("stop"));
      mqttClient.subscribe(mqttFullTopic("volume"));
      mqttClient.subscribe(mqttFullTopic("led1"));
      mqttClient.subscribe(mqttFullTopic("led2"));
      mqttClient.subscribe(mqttFullTopic("led3"));
      mqttClient.subscribe(mqttFullTopic("led4"));
      mqttClient.subscribe(mqttFullTopic("led5"));

#ifdef DEBUG_FLAG
      Serial.println(F("Connected to MQTT"));
#endif
      broadcastStatus("LWT", "online");
      broadcastStatus("ThingName", thingName.c_str());
      broadcastStatus("IPAddress", WiFi.localIP().toString());
      broadcastStatus("status", "idle");
      updateLEDBrightness(100);
    }
  }
}
/* ############################ WifiManager ############################################# */

void wifiConnected() {

#ifdef DEBUG_FLAG
  Serial.println();
  Serial.println(F("=================================================================="));
  Serial.println(F("  MrDIY Notifier"));
  Serial.println(F("=================================================================="));
  Serial.println();
  Serial.print(F("Connected to Wifi \t["));
  Serial.print(WiFi.localIP());
  Serial.println(F("]"));
#endif
  playBootSound();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setBufferSize(MQTT_MSG_SIZE);
  mqttReconnect();
}

bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper)
{
  boolean valid = true;
  int l = server.arg(mqttServerParam.getId()).length();
  if (l == 0) {
    mqttServerParam.errorMessage = "Please provide an MQTT server";
    valid = false;
  }
  return valid;
}

char* mqttFullTopic(const char action[]) {
  strcpy (mqttTopic, mqttTopicPrefix);
  strcat (mqttTopic, "/");
  strcat (mqttTopic, action);
  return mqttTopic;
}
