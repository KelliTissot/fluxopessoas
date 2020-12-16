#include <Arduino.h>
#include <IRSend.h>
#include <IRRecv.h>
#include <Wire.h>
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Adafruit_MLX90614.h>


#define WIFI_SSID       "VETORIAL_231_202"
#define WIFI_PASS       "querocafe"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                
#define AIO_USERNAME    "testecontpeople"
#define AIO_KEY         "aio_odud255iLf6uQuB1M2niarQG5qCq"


WiFiClient client;
// Configuração adafruit
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish infraver = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/infraver");
Adafruit_MQTT_Publish temperatura = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatura");

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
const uint16_t pinLedA = 4;
const uint16_t pinLedB = 19;
const uint16_t pinSensorA = 5;
const uint16_t pinSensorB = 17;
uint32_t senhaA = 0xE1411111;
uint32_t senhaB = 0xE1499999;


IRSend ledA(RMT_CHANNEL_0);
IRSend ledB(RMT_CHANNEL_1);
IRRecv receptorA(RMT_CHANNEL_2);
IRRecv receptorB(RMT_CHANNEL_3);

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { 
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);
       retries--;
       if (retries == 0) {
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
  mlx.begin();  
}

boolean recebeu(IRRecv *recv, uint32_t senha){
  
  boolean res  = false;
  while(recv->available()){
    char* rcvGroup;
    uint32_t result = recv->read(rcvGroup);
    if (result == senha) {
        res = true;
    }else if(result!=0){
      Serial.println(String("Senha errada:") + result);
    }
  }  
  return res;
}

struct Estado{
  boolean agora; 
  boolean anterior;
};

enum Ordem{
  E1,
  E2,
  E3,
  E4,
};

Estado alguemNaFrenteA;
Estado alguemNaFrenteB;
int pessoasDentro=0;
float temperaturaPessoa=0;
Ordem vet[] = {E1, E1, E1, E1, E1};


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
  Serial.print("*C\tCorporal: "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");
  Serial.println();
}

int A=0, B=0;
uint32_t naoRecebeuACont = 0, naoRecebeuBCont = 0;
boolean atualizouEstado=true;

void printa(){
  Serial.printf("A: %d, B: %d | pessoas: %d\n", A, B, pessoasDentro);
}

boolean precisaPublicar=true;
uint64_t ultimaPublicacao=0;
// Na conta gratis o máximo de publicações é 30 por minuto
void publicaAdafruit(){
  uint64_t agora = millis();
  if(precisaPublicar && agora > ultimaPublicacao + 2000){
    precisaPublicar = false;
    infraver.publish(pessoasDentro);
    temperatura.publish(mlx.readObjectTempC());
    ultimaPublicacao = millis();
  }
}

Ordem ordem=E1;

const char* stringOrdem(Ordem ord){
  switch (ord)
  {
  case E1:
    return "E1";
  case E2:
    return "E2";
  case E3:
    return "E3";
  case E4:
    return "E4";
  }
}

void checaOrdem(){
  boolean XA = alguemNaFrenteA.agora;
  boolean XB = alguemNaFrenteB.agora;
  Ordem anterior = ordem;
  switch (ordem)
  {
  case E1:
      if(XA==true){
        ordem=E2;
      }else if(XB==true){
        ordem=E3;
      };
    break;
  case E2:
    if(XA==false){
      ordem=E1;
    }else if(XB==true){
      ordem=E4;
    }
    break;
  case E3:
    if(XA==true){
      ordem=E4;
    }else if(XB==false){
      ordem=E1;
    }
    break;
  case E4:
    if(XA==false){
      ordem=E3;
    }else if(XB==false){
      ordem=E2;
    }
    break;
  }
  if(ordem != anterior){
    int i;
    // Salva ordem atual no vet  na ultima posição
    for(i=0; i<4; i++){
      vet[i]=vet[i+1];
    }
    vet[4]=ordem;

    if(vet[0]==E1 && vet[1]==E2 && vet[2]==E4  && vet[3]==E3  && vet[4]==E1 ){
      Serial.println("Entrou!");
      pessoasDentro++;
    }else if(vet[0]==E1 && vet[1]==E3 && vet[2]==E4  && vet[3]==E2  && vet[4]==E1 ){
      Serial.println("Saiu!");
      pessoasDentro--;
    }

    Serial.println(String("Ordem ") + stringOrdem(anterior) + " -> " + stringOrdem(ordem));
  }
}

void loop() {
  ledA.send(senhaA);
  ledA.send(senhaA);
  ledB.send(senhaB);
  ledB.send(senhaB);

  
  

  if(recebeu(&receptorA, senhaA)){
    naoRecebeuACont=0;
    alguemNaFrenteA.agora = false;
  }else{
    naoRecebeuACont++;
    // Conta somente se não recebeu sinal por duas vezes consecutivas
    if(naoRecebeuACont>1){
      alguemNaFrenteA.agora = true;
    }
  }

  if(recebeu(&receptorB, senhaB)){
    naoRecebeuBCont=0;
    alguemNaFrenteB.agora = false;
  }else{
    naoRecebeuBCont++;
    if(naoRecebeuBCont>1){
      alguemNaFrenteB.agora = true;
    }
  }

  if(trocouEstado(&alguemNaFrenteA)){
    A++;
    
    atualizouEstado=true;
  }
  if(trocouEstado(&alguemNaFrenteB)){
    B++;

    atualizouEstado=true;
  }
  MQTT_connect();
  checaOrdem();
  if(atualizouEstado){
    atualizouEstado =false;
    printa();
    precisaPublicar = true;
  }
  publicaAdafruit();
  
  //printEstado(alguemNaFrenteA);
  //delay(100);
}

