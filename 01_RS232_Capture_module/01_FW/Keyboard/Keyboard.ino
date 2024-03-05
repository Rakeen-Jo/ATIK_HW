#include <WiFi.h> 
#include <WiFiClient.h> 
#include <WebServer.h> 
#include <BleKeyboard.h> // 블루투스 키보드 이름, 변경해서 사용 가능

BleKeyboard bleKeyboard("ESP32 Keyboard", "ESP32Compny", 80); 
const char* ssid PROGMEM = "ATIK"; 
const char* password PROGMEM = "atikorea907"; 
WebServer server(80); // ESP32 보드의 2번 LED 입니다. 

const int led = 2; 

void handleRoot() 
{ server.send(200, F("text/plain"), F("hello from esp8266!")); } 

void handleNotFound() { server.send(404, F("text/plain"), F("Not Found")); } 

void setup(void) { 
  bleKeyboard.begin(); 
  pinMode(led, OUTPUT); 
  digitalWrite(led, 0); 
  Serial.begin(115200); 
  WiFi.mode(WIFI_STA); 
  WiFi.begin(ssid, password); 
  Serial.println(F("")); // Wait for connection 
  while (WiFi.status() != WL_CONNECTED) 
  { delay(500); Serial.print(F(".")); } 
  
  Serial.println(F("")); 
  Serial.print(F("Connected to ")); 
  Serial.println(F(ssid)); 
  Serial.print(F("IP address: ")); 
  Serial.println(WiFi.localIP()); 
  server.on(F("/"), handleRoot); 
  server.on(F("/inline"), []() { server.send(200, F("text/plain"), F("this works as well")); 
  }); 
  
  // ESP32 보드 IP 주소의 input path URL 요청시 작동하는 부분 
  // EX) http://192.168.0.2/input 요청하면 보드에서 아래 코드가 실행 
  server.on(F("/input"), []() { server.send(200, F("text/plain"), F("Input ok")); 
  
  // 블루투스 연결되었는지 여부 
  if(bleKeyboard.isConnected()) { 
  // ESP32 보드의 2번 LED 를 켠다 
  digitalWrite(led, 1); 
  // 키보드의 Scroll Lock 키를 누른다. 
  bleKeyboard.write(207); 
  // ESP32 보드의 2번 LED 를 끈다 
  digitalWrite(led, 0); } }); 
  
  server.onNotFound(handleNotFound); 
  server.begin(); 
  Serial.println(F("HTTP server started")); 
} 

void loop(void) { server.handleClient(); }
