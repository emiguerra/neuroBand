´´´ino
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>

// ---------- NeoPixel ----------
#define PIN_NEOPIXEL 6
#define NUM_LEDS     32
#define BRILLO       80

Adafruit_NeoPixel ring(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ---------- MPU6050 ----------
const uint8_t MPU_ADDR = 0x68;

// Botones
#define PIN_BOTON_POWER 2   // ON/OFF sistema
#define PIN_BOTON_CAL   8   // Recalibrar

// LED indicador
#define PIN_LED         13

// Estado del sistema
bool sistemaActivo   = true;
bool lastPowerState  = HIGH;
bool lastCalState    = HIGH;

// Offsets de calibración
int16_t axOffset = 0;
int16_t ayOffset = 0;
int16_t azOffset = 0;

// Filtros y umbrales
float baseLP  = 0.0f;
float deltaLP = 0.0f;

float aNormal     = 0.05f;   // movimiento controlado (verde)
float aPreventivo = 0.15f;   // movimiento brusco (amarillo)
float aPeligroso  = 0.30f;   // movimiento peligroso (rojo)

unsigned long lastEvent    = 0;
unsigned long refractoryMs = 100;

// ---------- NeoPixel ----------
void ringClear() {
  ring.clear();
  ring.show();
}

void ringFill(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    ring.setPixelColor(i, ring.Color(r, g, b));
  }
  ring.show();
}

// ---------- Lectura MPU cruda ----------
bool readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;

  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() < 6) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  return true;
}

// ---------- Calibración ----------
void calibrarOffsets() {
  Serial.println("Calibrando MPU... mantén el casco quieto ~3s.");

  // Azul mientras calibra
  ringFill(0, 0, 80);

  const int N = 500;
  long sumAx = 0;
  long sumAy = 0;
  long sumAz = 0;
  int lecturasValidas = 0;

  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az;
    if (readAccelRaw(ax, ay, az)) {
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      lecturasValidas++;
    }
    delay(5);
  }

  if (lecturasValidas > 0) {
    axOffset = sumAx / lecturasValidas;
    ayOffset = sumAy / lecturasValidas;
    azOffset = sumAz / lecturasValidas;

    Serial.print("Offsets -> ax: ");
    Serial.print(axOffset);
    Serial.print("  ay: ");
    Serial.print(ayOffset);
    Serial.print("  az: ");
    Serial.println(azOffset);
    Serial.println("Calibración completa.");
  } else {
    Serial.println("No se pudieron obtener lecturas válidas para calibrar.");
  }

  // Reiniciar filtros
  baseLP  = 0.0f;
  deltaLP = 0.0f;

  // Rosado al terminar
  ringFill(255, 80, 140);
  delay(800);
  ringClear();
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  // Despertar MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // NeoPixel
  ring.begin();
  ring.setBrightness(BRILLO);
  ringClear();

  // Botones y LED
  pinMode(PIN_BOTON_POWER, INPUT_PULLUP);
  pinMode(PIN_BOTON_CAL,   INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_LED, HIGH);
  Serial.println("Sistema iniciado (ON).");

  // Calibración inicial
  calibrarOffsets();
}

// ================= LOOP =================
void loop() {
  // ----- Botón POWER -----
  bool powerState = digitalRead(PIN_BOTON_POWER);
  if (lastPowerState == HIGH && powerState == LOW) {
    sistemaActivo = !sistemaActivo;

    if (sistemaActivo) {
      Serial.println(">>> SISTEMA ENCENDIDO");
      digitalWrite(PIN_LED, HIGH);
    } else {
      Serial.println(">>> SISTEMA APAGADO");
      digitalWrite(PIN_LED, LOW);
      ringClear();
    }
  }
  lastPowerState = powerState;

  // Si está apagado, no hacemos nada más
  if (!sistemaActivo) {
    delay(20);
    return;
  }

  // ----- Botón CAL -----
  bool calState = digitalRead(PIN_BOTON_CAL);
  if (lastCalState == HIGH && calState == LOW) {
    Serial.println(">>> Botón CAL presionado: recalibrando...");
    calibrarOffsets();
  }
  lastCalState = calState;

  // ----- Lectura MPU con offsets -----
  int16_t axRaw, ayRaw, azRaw;
  if (!readAccelRaw(axRaw, ayRaw, azRaw)) {
    Serial.println("Error leyendo MPU6050");
    delay(10);
    return;
  }

  int16_t axCal = axRaw - axOffset;
  int16_t ayCal = ayRaw - ayOffset;
  int16_t azCal = azRaw - azOffset;

  // Pasar a "g"
  float ax_g = axCal / 16384.0f;  // adelante / atrás
  float ay_g = ayCal / 16384.0f;  // izquierda / derecha

  // Movimiento combinado en plano X–Y
  float mov = sqrt(ax_g * ax_g + ay_g * ay_g);

  // Filtro paso bajo sobre el movimiento combinado
  baseLP = 0.05f * mov + 0.95f * baseLP;
  float delta = fabs(mov - baseLP);
  deltaLP = 0.3f * delta + 0.7f * deltaLP;

  // Debug
  Serial.print("Delta g combinado (lat + frente/atrás): ");
  Serial.println(deltaLP, 3);

  // ----- Niveles / NeoPixel -----
  unsigned long now = millis();
  if (now - lastEvent > refractoryMs) {
    if (deltaLP > aPeligroso) {
      ringFill(255, 0, 0);        // Rojo
      Serial.println("⚠ Movimiento PELIGROSO");
    }
    else if (deltaLP > aPreventivo) {
      ringFill(255, 180, 0);      // Amarillo
      Serial.println("Movimiento preventivo");
    }
    else if (deltaLP > aNormal) {
      ringFill(0, 255, 0);        // Verde
      Serial.println("Movimiento normal");
    }
    else {
      ringClear();                // Sin movimiento
    }

    lastEvent = now;
  }

  delay(10);

  ´´´ino
}
