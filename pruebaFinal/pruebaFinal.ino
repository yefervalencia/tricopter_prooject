#include <ESP32Servo.h>
#include <Wire.h>

Servo servoMotor;
Servo motor1;
Servo motor2;
Servo motor3;

int servo_position;
int max_servo_position = 110;
int min_servo_position = 50;

// MPU6050
#define MPU6050_adress 0x68
float gyro_Z, gyro_X, gyro_Y, temperature, gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
int gx, gy, gz, cal_int;
float acc_total_vector, ax, ay, az;
bool set_gyro_angles, accCalibOK  = false;
float acc_X_cal, acc_Y_cal, acc_Z_cal, angulo_pitch_acc, angulo_roll_acc, angulo_pitch, angulo_roll;

#define usCiclo 5000

//definicion de pines
#define pin_red 19
#define pin_green 18
#define pin_blue 5
#define pin_motor1 33
#define pin_motor2 25
#define pin_motor3 26
#define pin_servoMotor 23
#define pin_INT_Throttle 12 // Pin Throttle del mando RC
#define pin_INT_Yaw 27      // Pin Yaw del mando RC
#define pin_INT_Pitch 14    // Pin Pitch del mando RC
#define pin_INT_Roll 13     // Pin Roll del mando RC

//variables mando rc
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

float  tension_bateria, lectura_bat = 0.00;

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando configuración...");
  //Wire.begin();
  ledStart();
  mandoRCStart();
  motorStart();
  //MPU6050Start();
  //MPU6050Calibrate();

  
  loop_timer = micros(); // Inicializar temporizador
}

void loop() {
  led("ok");
  MPU6050Eject();
  bateryRead();
  mandoRCRead();
  //MPU6050Eject();
  motortest();


  Serial.print("tension_bateria:");
  Serial.print("\t");
  Serial.print(tension_bateria);
  Serial.print("\t");
  Serial.print("servo_position:");
  Serial.print("\t");
  Serial.print(servo_position);
  Serial.print("\t");
  Serial.println("angulo_pitch:");
  //Serial.print("\t");
  //Serial.print(angulo_pitch);
  //Serial.print("\t");
  //Serial.print("angulo_roll:");
  //Serial.print("\t");
  //Serial.println(angulo_roll);


}

void MPU6050Eject(){
  // Nuevo ciclo
  while (micros() - loop_timer < usCiclo);
  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  MPU6050_leer();     // Leer sensor MPU6050
  MPU6050_procesar(); // Procesar datos del sensor MPU6050
}

void MPU6050Start(){
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);                          // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);                          // 00000000 para activar
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);                          // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);                          // 00001000: 500dps
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);                          // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);                          // 00010000: +/- 8g
  Wire.endTransmission();
    Serial.println("se inicio el sensor mpu6050");

}

// Leer sensor MPU6050
void MPU6050_leer() {
  // Los datos del giroscopio y el acelerómetro se encuentran de la dirección 3B a la 14
  Wire.beginTransmission(MPU6050_adress);       // Empezamos comunicación
  Wire.write(0x3B);                             // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_adress, 14);         // Solicitar un total de 14 registros
  while (Wire.available() < 14);                // Esperamos hasta recibir los 14 bytes

  ax = Wire.read() << 8 | Wire.read();          // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read();          // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();          // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();          // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();          // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();          // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

// Cálculo de velocidad angular (º/s) y ángulo (º)
void MPU6050_procesar() {

  // Restar valores de calibración del acelerómetro
  ax -= acc_X_cal;
  ay -= acc_Y_cal;
  az -= acc_Z_cal;
  az  = az + 4096;

  // Restar valores de calibración del giroscopio y calcular
  // velocidad angular en º/s. Leer 65.5 en raw equivale a 1º/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calcular ángulo de inclinación con datos del giroscopio
  // 0.000000266 = tiempo_ejecucion / 1000 / 65.5 * PI / 180
  angulo_pitch += gyro_X * tiempo_ejecucion / 1000;
  angulo_roll  += gyro_Y * tiempo_ejecucion / 1000;
  angulo_pitch += angulo_roll  * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);
  angulo_roll  -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion  * 0.000000266);

  // Calcular vector de aceleración
  // 57.2958 = Conversion de radianes a grados 180/PI
  acc_total_vector  = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angulo_pitch_acc  = asin((float)ay / acc_total_vector) * 57.2958;
  angulo_roll_acc   = asin((float)ax / acc_total_vector) * -57.2958;

  // Filtro complementario
  if (set_gyro_angles) {
    angulo_pitch = angulo_pitch * 0.99 + angulo_pitch_acc * 0.01;
    angulo_roll  = angulo_roll  * 0.99 + angulo_roll_acc  * 0.01;
  }
  else {
    angulo_pitch = angulo_pitch_acc;
    angulo_roll  = angulo_roll_acc;
    set_gyro_angles = true;
  }
}

void MPU6050Calibrate(){
    // Calibrar giroscopio y acelerómetro. El sensor tiene que estar inmovil y en una supercifie plana.
  // Leer los datos del MPU6050 3000 veces y calcular el valor medio
  for (cal_int = 0; cal_int < 3000 ; cal_int ++) {
    MPU6050_leer();   // Leer sensor MPU6050
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal  += ax;
    acc_Y_cal  += ay;
    acc_Z_cal  += az;
    delayMicroseconds(50);
  }
  gyro_X_cal = gyro_X_cal / 3000;
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;
  acc_X_cal  = acc_X_cal  / 3000;
  acc_Y_cal  = acc_Y_cal  / 3000;
  acc_Z_cal  = acc_Z_cal  / 3000;
  accCalibOK = true;
  Serial.println("se calibro el sensor");
}

void motortest(){
  if(RC_Throttle_consigna>=1000){
    motor1.writeMicroseconds(RC_Throttle_consigna);
    motor2.writeMicroseconds(RC_Throttle_consigna);
    motor3.writeMicroseconds(RC_Throttle_consigna);
  }
  if(true){
    servo_position = map(RC_Yaw_consigna,us_min_Yaw_adj, us_max_Yaw_adj,min_servo_position,max_servo_position);
    servoMotor.write(servo_position);
  }
}

void motorStart(){
  servoMotor.attach(pin_servoMotor);
  motor1.attach(pin_motor1);
  motor2.attach(pin_motor2);
  motor3.attach(pin_motor3);

  servoMotor.write(min_servo_position);
  delay(2000);
  servoMotor.write(min_servo_position);
  delay(2000);
  servoMotor.write(80);
  delay(2000);
  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  delay(2000);
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);


  delay(5000);
  Serial.println("se iniciaron los motores");


}

void mandoRCStart(){
  // Declaración de pines como entrada
  Serial.println("se inicio el mando");
  pinMode(pin_INT_Yaw, INPUT_PULLUP);
  pinMode(pin_INT_Throttle, INPUT_PULLUP);
  pinMode(pin_INT_Pitch, INPUT_PULLUP);
  pinMode(pin_INT_Roll, INPUT_PULLUP);

  // Configuración de interrupciones con attachInterrupt
  attachInterrupt(digitalPinToInterrupt(pin_INT_Yaw), INT_Yaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Throttle), INT_Throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Pitch), INT_Pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Roll), INT_Roll, CHANGE);
  Serial.println("se inicio el mando");
}

void mandoRCRead(){
    // Esperar un intervalo fijo (10 ms)
  while (micros() - loop_timer < 10000);
  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

   // Ecuaciones de procesamiento
  RC_Throttle_consigna = map(RC_Throttle_raw, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  RC_Pitch_consigna    = map(RC_Pitch_raw, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
  RC_Roll_consigna     = map(RC_Roll_raw, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
  RC_Yaw_consigna      = map(RC_Yaw_raw, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);
}

void bateryRead(){
  while (micros() - loop_timer < usCiclo);  // Hacemos una lectura cada 'usCiclo' microsegundos
  loop_timer = micros();

  lectura_bat = analogRead(36);
  tension_bateria =  5.4* (lectura_bat * 3.3  / 4095);
}

void color(int R,int G, int B){
  R=255-R;
  G=255-G;
  B=255-B;
  analogWrite(pin_red,R);
  analogWrite(pin_green,G);
  analogWrite(pin_blue,B);
}

void ledStart(){
  pinMode(pin_red, OUTPUT);
    pinMode(pin_green, OUTPUT);
    pinMode(pin_blue, OUTPUT);
    color(0,0,255);
    delay(500);
    color(0,0,0);
    delay(500);
    color(0,0,255);
    delay(500);
    color(0,0,0);
    delay(500);
    color(0,0,255);
    delay(500);
    color(0,0,0);
    Serial.println("se iniciaron los leds");
}

void led(String state){
  if (state =="ok"){
    color(255,0,0);
    delay(500);
    color(0,255,0);
    delay(500);
    color(0,0,255);
    delay(500);
  }
  
}
