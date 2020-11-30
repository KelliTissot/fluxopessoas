#include <Arduino.h>
#include <assert.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRremoteESP8266.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>
#include <Wire.h>


const uint16_t pinLedA = 2;
const uint16_t pinLedB = 5;
const uint16_t pinSensorA = 19;
const uint16_t pinSensorB = 4;


#define LEGACY_TIMING_INFO false

IRrecv sensorA = IRrecv(pinSensorA, 1024, 15, true);
IRrecv sensorB = IRrecv(pinSensorB, 1024, 15, true);

decode_results results;  
IRsend ledA = IRsend(pinLedA);
IRsend ledB = IRsend(pinLedB);

void setup() {

  Serial.begin(115200);

  assert(irutils::lowLevelSanityCheck() == 0);

  
  receptorA.ESP32_IRrecvPIN(pinSensorA,0);
  receptorB.ESP32_IRrecvPIN(pinSensorB,0);

  ledA.begin();
  ledB.begin();
}

boolean recebeu(IRrecv *recv, IRsend *send, uint64_t senha){
  recv->enableIRIn();
  
  send->sendNEC(senha);
  if (recv->decode(&results)) {
    if(results.value == senha){
      recv->disableIRIn();
      return true;
    }
  }
  recv->disableIRIn();
  return false;
}

struct Estado{
  boolean agora; 
  boolean antes;
};

Estado alguemNaFrenteA;
Estado alguemNaFrenteB;


boolean trocouEstado(Estado *estado){
  if(estado->agora != estado->antes){
    estado->antes=estado->agora;
    return true;
  }
  return false;
}

void printEstado(Estado est){
  Serial.print("Antes:");
  Serial.print(est.antes);
  Serial.print("Agora:");
  Serial.println(est.agora);
}

int A=0, B=0;

void printa(){
  Serial.printf("A: %d, B: %d\n", A, B);
}

void loop() {
  alguemNaFrenteA.agora = !recebeu(&sensorA, &ledA, 0x1234567);
  alguemNaFrenteB.agora = !recebeu(&sensorB, &ledB, 0x1234567);
  if(trocouEstado(&alguemNaFrenteA)){
    A++;
    printa();
  }
  if(trocouEstado(&alguemNaFrenteB)){
    B++;
    printa();
  }
  //printEstado(alguemNaFrenteA);
  delay(100);
}

