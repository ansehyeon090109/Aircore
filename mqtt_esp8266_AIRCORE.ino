/*
 Basic ESP8266 MQTT example
 This sketch demonstrates the capabilities of the pubsub library in combination
 with the ESP8266 board/library.
 It connects to an MQTT server then:
  - publishes "hello world" to the topic "outTopic" every two seconds
  - subscribes to the topic "inTopic", printing out any messages
    it receives. NB - it assumes the received payloads are strings not binary
  - If the first character of the topic "inTopic" is an 1, switch ON the ESP Led,
    else switch it off
 It will reconnect to the server if the connection is lost using a blocking
 reconnect function. See the 'mqtt_reconnect_nonblocking' example for how to
 achieve the same result without blocking the main loop.
 To install the ESP8266 board, (using Arduino 1.6.4+):
  - Add the following 3rd party board manager under "File -> Preferences -> Additional Boards Manager URLs":
       http://arduino.esp8266.com/stable/package_esp8266com_index.json
  - Open the "Tools -> Board -> Board Manager" and click install for the ESP8266"
  - Select your ESP8266 in "Tools -> Board"
*/

#include <WiFi.h>
#include <PubSubClient.h>
#define btn1 25
#define btn2 26


// Update these with values suitable for your network.

const char* ssid = "bssm_free";
const char* password = "bssm_free";
const char* mqtt_server = "broker.mqtt-dashboard.com";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
//ESP32의 메시지 수신부
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}
//MQTT서버와 접속이 끊어지면 재접속하는 부분
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      //만약에 ESP32에서 뭔가를 받는다면 구독 토픽을 등록한다.
      client.subscribe("bssm/nockanda2");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {    // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  pinMode(btn1, INPUT);
  pinMode(btn2, INPUT);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
}

void loop() {
  //ESP32가 서버와 접속을 유지하는 부분
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  if(digitalRead(btn1) == HIGH) {
    Serial.println("버튼1눌려짐");
    client.publish("bssm/kibou", "1");
    delay(100);
  }
  if(digitalRead(btn2) == HIGH) {
    Serial.println("버튼2눌려짐");
    client.publish("bssm/kibou", "2");
    delay(100);
  }
  /*unsigned long now = millis();
  if (now - lastMsg > 2000) {
     lastMsg = now;
     
     float h = dht.readHumidity();
     float t = dht.readTemperature();

     String mydata = String(t) + "," + String(h)
  
    Serial.println(mydata);
    client.publish("bssm/kibou", mydata.c_str());
  }*/
}
