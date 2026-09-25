#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// Pinos I2C no NodeMCU / ESP8266
#define PIN_SDA D2
#define PIN_SCL D1
#define SAMPLE_TIME_MS 10  // tempo de amostragem
#define MAX_ANGLE 18.0    // angulo maximo de segurança

// Parâmetros da bateria
const float V_BAT       = 6.0;    // Bateria
const float V_DEADBAND  = 1.80;   // Zona morta
const float V_MIN_MOVER = 1.80;   // Corte de corrente mínima

// Ganhos PID Turner
const float Kp = 30.42;
const float Ki = 86.57;
const float Kd = 1.66;
const float dt = SAMPLE_TIME_MS / 1000.0; // 0.01s

// Variáveis do MPU
uint8_t FIFOBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool mpuReady = false;

// Variáveis de Controle
float referenciaDeg    = 0.0; //angulo de referencia
float pitchOffset      = 0.0;
float pitchRAW         = 0.0;
float actualPITCH      = 0.0;
float integralErrorRad = 0.0;
float erroAnteriorRad  = 0.0;
bool primeiroPasso     = true;
bool motorAtivo        = false;
uint32_t lastHeartbeat = 0;

void setup() {
  Serial.begin(115200);
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

  if (millis() - lastHeartbeat >= SAMPLE_TIME_MS) {
    lastHeartbeat = millis();
    // Lê o pacote mais recente do sensor
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      pitchRAW = ypr[1] * 180.0 / M_PI;
      actualPITCH = pitchRAW - pitchOffset; 

      float u_pid = 0.0;
      float u_motor = 0.0;
      int pwm_esp = 0;

      // Trava de seguranca
      if (abs(actualPITCH) >= MAX_ANGLE) {
        u_pid = 0.0;
        u_motor = 0.0;
        pwm_esp = 0;
        motorAtivo = false;
        integralErrorRad = 0.0;
        primeiroPasso = true;
      } 
      else {
        motorAtivo = true;

        // Erro para radianos 
        float referenciaRad = referenciaDeg * (M_PI / 180.0);
        float pitchRad = actualPITCH * (M_PI / 180.0);
        float erroRad  = referenciaRad - pitchRad;

        if (primeiroPasso) {
          erroAnteriorRad = erroRad;
          primeiroPasso = false;
        }

        // P e D
        float P = Kp * erroRad;
        float D = Kd * ((erroRad - erroAnteriorRad) / dt);
        erroAnteriorRad = erroRad;

        // ANTI-WINDUP
        float novaIntegral = integralErrorRad + (erroRad * dt);
        float u_teste = P + (Ki * novaIntegral) + D;

        if (abs(u_teste) <= V_BAT) {
          integralErrorRad = novaIntegral;
        }

        u_pid = P + (Ki * integralErrorRad) + D;

        // Zona morta e saturacao
        if (abs(u_pid) > 0.001) {
          float u_tentativa = u_pid + ((u_pid > 0) ? V_DEADBAND : -V_DEADBAND);

          if (abs(u_tentativa) >= V_MIN_MOVER) {
            u_motor = u_tentativa;

            if (u_motor > V_BAT)  u_motor = V_BAT;
            if (u_motor < -V_BAT) u_motor = -V_BAT;

            // Escala PWM (0-255)
            pwm_esp = (int)((abs(u_motor) / V_BAT) * 255.0);
          } else {
            u_motor = 0.0;
            pwm_esp = 0;
          }
        }
      }

      // Diagnóstico
      Serial.print("Pitch: ");    Serial.print(actualPITCH, 2); Serial.print("° | ");
      Serial.print("u_PID: ");    Serial.print(u_pid, 2);       Serial.print("V | ");
      Serial.print("u_Motor: ");  Serial.print(u_motor, 2);     Serial.print("V | ");
      Serial.print("PWM: ");      Serial.print(pwm_esp);        Serial.print("/255 | ");
      Serial.println(motorAtivo ? "STATUS: ATIVO" : "STATUS: DESLIGADO (QUEDA)");
    }
  }
}