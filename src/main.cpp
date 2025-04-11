#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AiEsp32RotaryEncoder.h>

// ----- PID -----
float setpoint = 3.0;       // Maintenant variable dynamique
#define DT 10               // Intervalle en ms
#define AFFICHAGE (1000 / DT)
#define K0 6.65
#define KP 0.01

int compteur = 0;

// ----- MPU -----
Adafruit_MPU6050 mpu;

// ----- ROTARY ENCODER -----
#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 18
#define ROTARY_ENCODER_BUTTON_PIN 19
#define ROTARY_ENCODER_VCC_PIN -1
#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(
  ROTARY_ENCODER_A_PIN,
  ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN,
  ROTARY_ENCODER_VCC_PIN,
  ROTARY_ENCODER_STEPS
);

// ----- PWM -----
#define PWM_PIN 13
#define PWM_CHANNEL 0
#define PWM_FREQ 50
#define PWM_RESOLUTION 12

// ----- ISR -----
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

// ----- SETUP -----
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // MPU INIT
  if (!mpu.begin()) {
    Serial.println("MPU6050 non détecté");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 prêt");

  // PWM INIT
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  // ENCODER INIT
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(-45, 45, false); // plage du setpoint
  rotaryEncoder.setEncoderValue((int)setpoint); // valeur initiale
  rotaryEncoder.disableAcceleration();

  Serial.println("Setup terminé");
  ledcWrite(PWM_CHANNEL, 200.655); // PWM à 0 au démarrage
  delay(5000); // Laisser le temps au moteur de se stabiliser
}

// ----- LOOP -----
void loop() {
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= DT) {
    lastTime = currentTime;
    compteur++;

    // 1. Lire le setpoint de l’encodeur
    if (rotaryEncoder.encoderChanged()) {
      setpoint = (float)rotaryEncoder.readEncoder(); // en degrés
      Serial.print("Setpoint modifié: ");
      Serial.println(setpoint);
    }

    // 2. Lire angle du capteur
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float angle = -atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;

    // 3. Contrôle P
    float erreur = setpoint - angle;
    float sortie = K0 + KP * erreur;
    sortie = sortie > 6.9 ? 6.9 : sortie; // Limiter la sortie à 6.9 pour éviter le dépassement
    sortie = sortie < 6.4 ? 6.4 : sortie; // Limiter la sortie à 0.0 pour éviter le dépassement

    int pwmValue = (int)((sortie / 100.0) * 4095.0);
    ledcWrite(PWM_CHANNEL, pwmValue);

    // 4. Affichage toutes les x itérations
    if (compteur >= AFFICHAGE) {
      compteur = 0;
      Serial.print(">");
      Serial.print("var1: ");
      Serial.print(angle);
      Serial.print(",");
      Serial.print("var2: ");
      Serial.print(setpoint);
      Serial.println("");
    }
  }

  // Optional: bouton sur encodeur
  if (rotaryEncoder.isEncoderButtonClicked()) {
    Serial.println("Bouton cliqué (action future?)");
  }
}