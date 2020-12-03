#include <Arduino.h>
#include <IRSend.h>
#include <IRRecv.h>
#include <Wire.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WIFI_SSID       "VETORIAL_231_202"
#define WIFI_PASS       "querocafe"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "testecontpeople"
#define AIO_KEY         "aio_cWqO57at4fgs6Aw4EMhVSY62S36q"


WiFiClient client;
// Configuração adafruit
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish infraver = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/infraver");


const uint16_t pinLedA = 2;
const uint16_t pinLedB = 4;
const uint16_t pinSensorA = 19;
const uint16_t pinSensorB = 5;
uint32_t senhaA = 0xE1411111;
uint32_t senhaB = 0xE1499999;


IRSend ledA(RMT_CHANNEL_0);
IRSend ledB(RMT_CHANNEL_1);
IRRecv receptorA(RMT_CHANNEL_2);
IRRecv receptorB(RMT_CHANNEL_3);

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}

void ConnectWifi(){
   WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("IP: ");
    Serial.println(WiFi.localIP());
}


void setup() {

  Serial.begin(115200);
  delay(500);
  Serial.println("Start->");
  ledA.start(pinLedA);
  ledB.start(pinLedB);
  receptorA.start(pinSensorA);
  receptorB.start(pinSensorB);
  ConnectWifi();
}

boolean recebeu(IRRecv *recv, IRSend *led, uint32_t senha){
  led->send(senha);
  boolean res  = false;
  while(recv->available()){
    char* rcvGroup;
    uint32_t result = recv->read(rcvGroup);
    if (result == senha) {
        res = true;
    }
  }  
  return res;
}

struct Estado{
  boolean agora; 
  boolean anterior;
};

Estado alguemNaFrenteA;
Estado alguemNaFrenteB;


boolean trocouEstado(Estado *estado){
  if(estado->agora != estado->anterior){
    estado->anterior=estado->agora;
    return true;
  }
  return false;
}

void printEstado(Estado est){
  Serial.print("anterior:");
  Serial.print(est.anterior);
  Serial.print("Agora:");
  Serial.println(est.agora);
}

int contPessoas=0, A=0, B=0;
uint32_t naoRecebeuACont = 0, naoRecebeuBCont = 0;
boolean atualizouEstado=true;

void printa(){
  Serial.printf("A: %d, B: %d | cont: %d\n", A, B, contPessoas);
}

boolean precisaPublicar=true;
uint64_t ultimaPublicacao=0;
// Na conta gratis o máximo de publicações é 30 por minuto
void publicaAdafruit(){
  uint64_t agora = millis();
  if(precisaPublicar && agora > ultimaPublicacao + 2000){
    precisaPublicar = false;
    infraver.publish(contPessoas);
    ultimaPublicacao = millis();
  }
}

boolean ePar(uint32_t num){
  return num % 2 == 0;
}

void loop() {
  if(recebeu(&receptorA, &ledA, senhaA)){
    naoRecebeuACont=0;
    alguemNaFrenteA.agora = false;
  }else{
    naoRecebeuACont++;
    // Conta somente se não recebeu sinal por duas vezes consecutivas
    if(naoRecebeuACont>3){
      alguemNaFrenteA.agora = true;
    }
  }

  if(recebeu(&receptorB, &ledB, senhaB)){
    naoRecebeuBCont=0;
    alguemNaFrenteB.agora = false;
  }else{
    naoRecebeuBCont++;
    if(naoRecebeuBCont>3){
      alguemNaFrenteB.agora = true;
    }
  }

  if(trocouEstado(&alguemNaFrenteA)){
    A++;
    if(ePar(A)){
      contPessoas++;
    }
    atualizouEstado=true;
  }
  if(trocouEstado(&alguemNaFrenteB)){
    B++;
    
    if(ePar(B)){
      contPessoas--;
    }
    atualizouEstado=true;
  }
  MQTT_connect();
  if(atualizouEstado){
    atualizouEstado =false;
    printa();
    precisaPublicar = true;
  }
  publicaAdafruit();
  //printEstado(alguemNaFrenteA);
  //delay(100);
}

