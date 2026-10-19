#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

// PINOS (NodeMCU / ESP8266)
// I2C do MPU6050
#define PIN_SDA D2
#define PIN_SCL D1

// Ponte H L298N - Motor A
#define PIN_ENA D5   // PWM motor A (GPIO14)
#define PIN_IN1 D6   // Direção motor A (GPIO12)
#define PIN_IN2 D7   // Direção motor A (GPIO13)

// Ponte H L298N - Motor B
#define PIN_ENB D8   // PWM motor B (GPIO15)
#define PIN_IN3 D0   // Direção motor B (GPIO16)
#define PIN_IN4 D4   // Direção motor B (GPIO2)

// PWM 10 BITS
#define PWM_RANGE 1023     // 10 bits: 0 a 1023
#define PWM_FREQ  1000     // 1 kHz

#define DIRECAO_MOTOR_A  1
#define DIRECAO_MOTOR_B -1

// PARÂMETROS GERAIS
#define SAMPLE_TIME_MS 10      // tempo de amostragem
#define MAX_ANGLE 12.0        
#define ANGULO_VERTICAL 90.0  

// Parâmetros da bateria
const float V_BAT       = 6.0;    // Tensão da bateria 
const float V_DEADBAND  = 0.9;   // Zona morta
const float V_MIN_MOVER = 0.9;   // Corte de corrente mínima

// Ganhos PID Turner
const float Kp = 30.42;
const float Ki = 86.57;
const float Kd = 1.66;

// Variáveis do MPU
uint8_t FIFOBuffer[64];
Quaternion q;
VectorFloat gravity;
bool mpuReady = false;

// Variáveis de Controle
float referenciaDeg    = ANGULO_VERTICAL; // ângulo de referência
float pitchOffset      = 0.0;             // ajuste fino de calibração
float pitchRAW         = 0.0;
float actualPITCH      = 0.0;
float integralErrorRad = 0.0;
float erroAnteriorRad  = 0.0;
bool primeiroPasso     = true;
bool motorAtivo        = false;
uint32_t lastHeartbeat = 0;
uint32_t lastPrint     = 0;
uint32_t lastMicros    = 0;

// MOTORES 
void setarSentido(uint8_t pinA, uint8_t pinB, int sentido) {
  if (sentido > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  }
}

// pwm: 0 a 1023 | sentido: +1 frente, -1 trás, 0 parado
void acionarMotores(int pwm, int sentido) {
  if (pwm <= 0 || sentido == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
    analogWrite(PIN_ENA, 0);
    analogWrite(PIN_ENB, 0);
    return;
  }

  setarSentido(PIN_IN1, PIN_IN2, sentido * DIRECAO_MOTOR_A);
  setarSentido(PIN_IN3, PIN_IN4, sentido * DIRECAO_MOTOR_B);

  if (pwm > PWM_RANGE) pwm = PWM_RANGE;
  analogWrite(PIN_ENA, pwm);
  analogWrite(PIN_ENB, pwm);
}

void setup() {
  Serial.begin(115200);

  // pinos ponte H e PWM 
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  analogWriteRange(PWM_RANGE);
  analogWriteFreq(PWM_FREQ);
  acionarMotores(0, 0);

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
    lastMicros = micros();
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

      pitchRAW = atan2(gravity.x, gravity.z) * 180.0 / M_PI;
      actualPITCH = pitchRAW - pitchOffset;

      // dt
      uint32_t agora = micros();
      float dt = (agora - lastMicros) / 1000000.0;
      lastMicros = agora;
      if (dt <= 0.0 || dt > 0.1) dt = SAMPLE_TIME_MS / 1000.0;

      float u_pid = 0.0;
      float u_motor = 0.0;
      int pwm_esp = 0;

      // Trava de segurança
      if (fabs(actualPITCH - referenciaDeg) >= MAX_ANGLE) {
        u_pid = 0.0;
        u_motor = 0.0;
        pwm_esp = 0;
        motorAtivo = false;
        integralErrorRad = 0.0;
        primeiroPasso = true;
      }
      else {
        motorAtivo = true;

        // Erro em radianos
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

        if (fabs(u_teste) <= V_BAT) {
          integralErrorRad = novaIntegral;
        }

        u_pid = P + (Ki * integralErrorRad) + D;

        // Zona morta e saturação
        if (fabs(u_pid) > 0.001) {
          float u_tentativa = u_pid + ((u_pid > 0) ? V_DEADBAND : -V_DEADBAND);

          if (fabs(u_tentativa) >= V_MIN_MOVER) {
            u_motor = u_tentativa;

            if (u_motor > V_BAT)  u_motor = V_BAT;
            if (u_motor < -V_BAT) u_motor = -V_BAT;

            // Escala PWM de 10 bits (0-1023)
            pwm_esp = (int)((fabs(u_motor) / V_BAT) * (float)PWM_RANGE);
          } else {
            u_motor = 0.0;
            pwm_esp = 0;
          }
        }
      }

      // Envia o sinal para a ponte H (L298N)
      if (motorAtivo && pwm_esp > 0) {
        acionarMotores(pwm_esp, (u_motor > 0) ? 1 : -1);
      } else {
        acionarMotores(0, 0);
      }

      // Diagnóstico a cada 50 ms
      if (millis() - lastPrint >= 50) {
        lastPrint = millis();
        Serial.print("Angulo: ");   Serial.print(actualPITCH, 2); Serial.print("° | ");
        Serial.print("u_PID: ");    Serial.print(u_pid, 2);       Serial.print("V | ");
        Serial.print("u_Motor: ");  Serial.print(u_motor, 2);     Serial.print("V | ");
        Serial.print("PWM: ");      Serial.print(pwm_esp);        Serial.print("/1023 | ");
        Serial.println(motorAtivo ? "STATUS: ATIVO" : "STATUS: DESLIGADO (QUEDA)");
      }
    }
  }
}
