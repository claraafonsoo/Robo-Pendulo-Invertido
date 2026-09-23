#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// Pinos I2C no NodeMCU / ESP8266
#define PIN_SDA D2
#define PIN_SCL D1
#define SAMPLE_TIME 10 // tempo de amostragem
#define MAX_ANGLE 18.0 // angulo maximo de segurança

uint8_t FIFOBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool mpuReady = false;
uint32_t lastHeartbeat = 0;
float pitchOffset = 0.0;
float pitchRAW = 0.0;
float actualPITCH = 0.0;
float u = 0.0; // esforço de controle
bool motorAtivo;
float integralError = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); // 100kHz para evitar falhas nos cabos jumper

  Serial.println("\nIniciando MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("ERRO: MPU6050 nao encontrado. Verifique as conexoes!");
    return;
  }

  // Inicializa o processador de movimento interno (DMP)
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    mpuReady = true;
    Serial.println("MPU6050 pronto para leitura!");
  } else {
    Serial.println("ERRO: Falha ao inicializar o DMP.");
  }
}

void loop() {
  if (!mpuReady) return;

  if (millis() - lastHeartbeat >= SAMPLE_TIME) {
    // Lê o pacote mais recente do sensor
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      pitchRAW = ypr[1] * 180 / M_PI;
      actualPITCH = pitchRAW - pitchOffset; 

      if (abs(actualPITCH >= MAX_ANGLE)) {
        u = 0.0;
        motorAtivo = false;
        integralError = 0.0;
      } else {
        motorAtivo = true; 
      }
      // Exibe os valores convertidos para graus
      Serial.print("Yaw: ");   Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\tPitch: "); Serial.print(pitchRAW);
      Serial.print("\tRoll: ");  Serial.println(ypr[2] * 180 / M_PI);
      Serial.print("\tActualPitch: ");  Serial.println(actualPITCH);
    }
    lastHeartbeat = millis();
  }

  
}