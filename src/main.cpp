#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AiEsp32RotaryEncoder.h>

// ----- PID -----
float setPoint_deg = 3.0;       // Maintenant variable dynamique
#define DT_ms 10               // Intervalle en ms
#define K0 7.2
#define KP 0.01
#define KI 0.0
#define KD 0.0

int compteur_affichage = 0;

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
  rotaryEncoder.setBoundaries(-45, 45, false); // plage du setPoint_deg
  rotaryEncoder.setEncoderValue((int)setPoint_deg); // valeur initiale
  rotaryEncoder.disableAcceleration();

  Serial.println("Setup terminé");
  ledcWrite(PWM_CHANNEL, 200.655); // PWM à 4.9% au démarrage
  delay(5000); // Laisser le temps au moteur de se stabiliser
}

// ----- LOOP -----
void loop() {
  static unsigned long lastTime_ms = 0;
  unsigned long currentTime_ms = millis();

  if (currentTime_ms - lastTime_ms >= DT_ms) {
    lastTime_ms = currentTime_ms;
    compteur_affichage++;

    // 1. Lire le setPoint_deg de l’encodeur
    if (rotaryEncoder.encoderChanged()) {
      setPoint_deg = (float)rotaryEncoder.readEncoder(); // en degrés
      Serial.print("SetPoint modifié: ");
      Serial.println(setPoint_deg);
    }

    // 2. Lire angle du capteur
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float angle_deg = -atan2(a.acceleration.x, a.acceleration.z) * 180.0 / PI;

    // 3. Contrôle P
    float erreur = setPoint_deg - angle_deg;
    float integrale_erreur = integrale_erreur + (erreur * (DT_ms / 1000)); // Erreur intégrée sur 10ms

    float sortie = K0 + KP * erreur;
    
    sortie = sortie > 7.3 ? 7.3 : sortie; // Limiter la sortie à 6.9 pour éviter le dépassement
    sortie = sortie < 7 ? 7 : sortie; // Limiter la sortie à 0.0 pour éviter le dépassement

    int pwmValue = (int)((sortie / 100.0) * 4095.0); // Convertir en valeur PWM (0-4095 pour 12 bits)
    ledcWrite(PWM_CHANNEL, pwmValue);

    // 4. Affichage toutes les x itérations
    if (compteur_affichage >= 10) {
      compteur_affichage = 0;
      Serial.print(">");
      Serial.print("angle: ");
      Serial.print(angle_deg);
      Serial.print(",");
      Serial.print("setPoint: ");
      Serial.print(setPoint_deg);
      Serial.print(",");
      Serial.print("erreur: ");
      Serial.print(erreur);
      Serial.print(",");
      Serial.print("sortie: ");
      Serial.print(sortie);
      Serial.println("");
    }
  }

  // Optional: bouton sur encodeur
  if (rotaryEncoder.isEncoderButtonClicked()) {
    Serial.println("Bouton cliqué (action future?)");
  }
}