// Declaración de pines
#define pin_INT_Throttle 12 // Pin Throttle del mando RC
#define pin_INT_Yaw 27      // Pin Yaw del mando RC
#define pin_INT_Pitch 14    // Pin Pitch del mando RC
#define pin_INT_Roll 13     // Pin Roll del mando RC

float RC_Throttle_consigna, RC_Pitch_consigna, RC_Roll_consigna, RC_Yaw_consigna;

// AJUSTE MANDO RC - THROTLLE
const int us_max_Throttle_adj = 2000;
const int us_min_Throttle_adj = 950;
const float us_max_Throttle_raw = 2014;
const float us_min_Throttle_raw = 989;

// AJUSTE MANDO RC - PITCH
const float us_max_Pitch_raw = 2014;
const float us_min_Pitch_raw = 989;
const int us_max_Pitch_adj = -30;   // <-- Si teneis la entrada Pitch invertido sustituid este valor
const int us_min_Pitch_adj = 30;

// AJUSTE MANDO RC - ROLL
const float us_max_Roll_raw = 2014;
const float us_min_Roll_raw = 989;
const int us_max_Roll_adj = 30;     // <-- Si teneis la entrada Roll invertido sustituid este valor
const int us_min_Roll_adj = -30;

// AJUSTE MANDO RC - YAW
const float us_max_Yaw_raw = 2014;
const float us_min_Yaw_raw = 989;
const int us_max_Yaw_adj = 30;      // <-- Si teneis la entrada Yaw invertido sustituid este valor
const int us_min_Yaw_adj = -30; 

volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;

volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;

volatile long Roll_HIGH_us;
volatile int RC_Roll_raw;

volatile long Yaw_HIGH_us;
volatile int RC_Yaw_raw;

long loop_timer, tiempo_ejecucion;

// INTERRUPCIÓN MANDO RC --> THROTTLE
void INT_Throttle() {
  if (digitalRead(pin_INT_Throttle) == HIGH)
    Throttle_HIGH_us = micros();
  else
    RC_Throttle_raw = micros() - Throttle_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> PITCH
void INT_Pitch() {
  if (digitalRead(pin_INT_Pitch) == HIGH)
    Pitch_HIGH_us = micros();
  else
    RC_Pitch_raw = micros() - Pitch_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> ROLL
void INT_Roll() {
  if (digitalRead(pin_INT_Roll) == HIGH)
    Roll_HIGH_us = micros();
  else
    RC_Roll_raw = micros() - Roll_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> YAW
void INT_Yaw() {
  if (digitalRead(pin_INT_Yaw) == HIGH)
    Yaw_HIGH_us = micros();
  else
    RC_Yaw_raw = micros() - Yaw_HIGH_us;
}

void setup() {
  // Declaración de pines como entrada
  pinMode(pin_INT_Yaw, INPUT_PULLUP);
  pinMode(pin_INT_Throttle, INPUT_PULLUP);
  pinMode(pin_INT_Pitch, INPUT_PULLUP);
  pinMode(pin_INT_Roll, INPUT_PULLUP);

  // Configuración de interrupciones con attachInterrupt
  attachInterrupt(digitalPinToInterrupt(pin_INT_Yaw), INT_Yaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Throttle), INT_Throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Pitch), INT_Pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Roll), INT_Roll, CHANGE);

  // Inicializar comunicación serial
  Serial.begin(115200);
  loop_timer = micros(); // Inicializar temporizador
}

void loop() {
  // Esperar un intervalo fijo (10 ms)
  while (micros() - loop_timer < 10000);
  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

   // Ecuaciones de procesamiento
  RC_Throttle_consigna = map(RC_Throttle_raw, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  RC_Pitch_consigna    = map(RC_Pitch_raw, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
  RC_Roll_consigna     = map(RC_Roll_raw, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
  RC_Yaw_consigna      = map(RC_Yaw_raw, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);

  // Monitor Serie
  Serial.print(RC_Throttle_consigna);
  Serial.print("\t");
  Serial.print(RC_Pitch_consigna);
  Serial.print("\t");
  Serial.print(RC_Roll_consigna);
  Serial.print("\t");
  Serial.println(RC_Yaw_consigna);
}
