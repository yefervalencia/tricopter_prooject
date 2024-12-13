#include <ESP32Servo.h>
#include <Wire.h>
#include "Simple_MPU6050.h"	
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

bool visu = 1;         // Visualizar variables por canal serie. En vuelo a 0!!
int visu_select = 999;   // 0: mando RC, 1: bateria, 2: giro, 3: ang, 4: motores consigna
bool MODO_vuelo = 1;   // 0: Modo acrobatico, 1: Modo estable (por defecto MODO_vuelo = 1)
String dron_estado;


#define usCiclo 6000   // Ciclo de ejecucion del software en microsegundos

#define pin_motor1 25        // Pin motor 1 motor delantero izquierdo
#define pin_motor2 33        // Pin motor 2 motor delantero derecho
#define pin_motor3 26        // Pin motor 3 motor trasero
#define pin_servoMotor 23        // Pin servo motor servomotor
#define pin_INT_Throttle 12 // Pin Throttle del mando RC
#define pin_INT_Yaw 27       // Pin Yaw del mando RC
#define pin_INT_Pitch 14    // Pin Pitch del mando RC
#define pin_INT_Roll 13      // Pin Roll del mando RC
#define pin_LED_rojo 19    // Pin LED rojo 1      
#define pin_LED_verde 18    // Pin LED rojo 2    
#define pin_LED_azul 5     // Pin LED azul     
// --------------------------------------------------



Servo servoMotor;
int SERVO_CONSIGNA = 80;
int led_rojo_state = false;
int led_verde_state = false;
int led_azul_state = false;

int velocidad=245;

int pwm_channel_motor1 = 0;
int pwm_channel_motor2 = 1;
int pwm_channel_motor3 = 2;

int MOTOR1_CONSIGNA=245;
int MOTOR2_CONSIGNA=245;
int MOTOR3_CONSIGNA=245;

const int pwmFreq = 60;      // Frecuencia PWM para servos (50 Hz)
const int pwmResolution = 12; // Resolución del PWM (16 bits)

Servo motor1;
Servo motor2;
Servo motor3;

// Variables de tiempo
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned long blinkInterval = 1000; // 1 segundo encendido, 1 segundo apagado

// Variables de control
int blinkCount = 0; 

//ajuste pid's
float Roll_ang_Kp  = 0.5, Roll_ang_Ki  = 0.05, Roll_ang_Kd  = 10;
float Pitch_ang_Kp = 0.5, Pitch_ang_Ki = 0.05, Pitch_ang_Kd = 10;
float Pitch_W_Kp   = 2,   Pitch_W_Ki   = 0.02, Pitch_W_Kd   = 0;
float Roll_W_Kp    = 2,   Roll_W_Ki    = 0.02, Roll_W_Kd    = 0;
float Yaw_W_Kp     = 1,   Yaw_W_Ki     = 0.05, Yaw_W_Kd     = 0;

int PID_W_sat1   = 150;  // Limitar parte integral PID velocidad
int PID_W_sat2   = 150;  // Limitar salida del PID velocidad
int PID_ang_sat1 = 130;  // Limitar parte integral PID ángulo
int PID_ang_sat2 = 130;  // Limitar salida del PID ángulo
int PID_W_servo = 10;  //mapeo salida PID servomotor minima
//int PID_max_servo = 110;  //mapeo salida PID servomotor maxima

float PID_ang_Pitch_error, PID_ang_Pitch_P, PID_ang_Pitch_I, PID_ang_Pitch_D, PID_ang_Pitch_OUT;
float PID_ang_Roll_error, PID_ang_Roll_P, PID_ang_Roll_I, PID_ang_Roll_D, PID_ang_Roll_OUT;
float PID_ang_Yaw_error, PID_ang_Yaw_P, PID_ang_Yaw_I, PID_ang_Yaw_D, PID_ang_Yaw_OUT;
float PID_W_Pitch_error, PID_W_Pitch_P, PID_W_Pitch_I, PID_W_Pitch_D, PID_W_Pitch_OUT;
float PID_W_Roll_error, PID_W_Roll_P, PID_W_Roll_I, PID_W_Roll_D, PID_W_Roll_OUT;
float PID_W_Yaw_error, PID_W_Yaw_P, PID_W_Yaw_I, PID_W_Yaw_D, PID_W_Yaw_OUT;
float PID_W_Pitch_consigna, PID_W_Roll_consigna;

// AJUSTE MANDO RC - THROTLLE
const int us_max_Throttle_adj = 2000;
const int us_min_Throttle_adj = 950;
const float us_max_Throttle_raw = 2014;
const float us_min_Throttle_raw = 989;

// AJUSTE MANDO RC - PITCH
const float us_max_Pitch_raw = 2014;
const float us_min_Pitch_raw = 989;
const int us_max_Pitch_adj = 30;
const int us_min_Pitch_adj = -30;

// AJUSTE MANDO RC - ROLL
const float us_max_Roll_raw = 2014;
const float us_min_Roll_raw = 989;
const int us_max_Roll_adj = 30;
const int us_min_Roll_adj = -30;

// AJUSTE MANDO RC - YAW
const float us_max_Yaw_raw = 2014;
const float us_min_Yaw_raw = 989;
const int us_max_Yaw_adj = 30;
const int us_min_Yaw_adj = -30; 

// MPU6050
#define MPU6050_ADDRESS_AD0_LOW     0x68			// direccion I2C con AD0 en LOW o sin conexion
#define MPU6050_ADDRESS_AD0_HIGH    0x69			// direccion I2C con AD0 en HIGH
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW	// por defecto AD0 en LOW

Simple_MPU6050 mpu;				// crea objeto con nombre mpu
// ENABLE_MPU_OVERFLOW_PROTECTION();		// activa proteccion, ya no se requiere

#define OFFSETS  4294964334,   832,   1122,   16,   19,   4294967286  // Colocar valores personalizados

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
// spamtimer funcion para generar demora al escribir en monitor serie sin usar delay()

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);
// printfloatx funcion para mostrar en monitor serie datos para evitar el uso se multiples print()
float angulo_pitch, angulo_roll, angulo_yaw;
float angulo_pitch_ant, angulo_roll_ant, angulo_yaw_ant;
float gyro_Z, gyro_X, gyro_Y, gyro_X_ant, gyro_Y_ant, gyro_Z_ant;

const float GYRO_SCALE_FACTOR = 65.5;
// mostrar_valores funcion que es llamada cada vez que hay datos disponibles desde el sensor

void leerGiroscopio() {
  int16_t raw_X, raw_Y, raw_Z;

  // Leer 6 bytes del giroscopio (registro 0x43 al 0x48)
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(0x43); // Dirección del registro GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, 6);

  // Unir los bytes para cada eje
  raw_X = (Wire.read() << 8) | Wire.read(); // GYRO_XOUT_H y GYRO_XOUT_L
  raw_Y = (Wire.read() << 8) | Wire.read(); // GYRO_YOUT_H y GYRO_YOUT_L
  raw_Z = (Wire.read() << 8) | Wire.read(); // GYRO_ZOUT_H y GYRO_ZOUT_L

  // Convertir los valores RAW a velocidad angular en °/s
  gyro_X = raw_X / GYRO_SCALE_FACTOR;
  gyro_Y = raw_Y / GYRO_SCALE_FACTOR;
  gyro_Z = raw_Z / GYRO_SCALE_FACTOR;
}

void mostrar_valores (int16_t *gyro, int16_t *accel, int32_t *quat){//), uint32_t *timestamp) {	
  uint8_t SpamDelay = 100;			// demora para escribir en monitor serie de 100 mseg
  Quaternion q;					// variable necesaria para calculos posteriores
  VectorFloat gravity;				// variable necesaria para calculos posteriores
  float ypr[3] = { 0, 0, 0 };			// array para almacenar valores de yaw, pitch, roll
  float xyz[3] = { 0, 0, 0 };			// array para almacenar valores convertidos a grados de yaw, pitch, roll
  spamtimer(SpamDelay) {			// si han transcurrido al menos 100 mseg entonces proceder
    mpu.GetQuaternion(&q, quat);		// funcion para obtener valor para calculo posterior
    mpu.GetGravity(&gravity, &q);		// funcion para obtener valor para calculo posterior
    mpu.GetYawPitchRoll(ypr, &q, &gravity);	// funcion obtiene valores de yaw, ptich, roll
    mpu.ConvertToDegrees(ypr, xyz);		// funcion convierte a grados sexagesimales
    angulo_pitch=xyz[2];
    angulo_roll=xyz[1];
    angulo_yaw=xyz[0];
    leerGiroscopio();
  }
}



// TIEMPOS
long loop_timer, loop_timer1, tiempo_motores_start, tiempo_ON, tiempo_1, tiempo_2;

// LECTURA TENSIÓN DE BATERÍA
bool LOW_BAT_WARING = false;
float tension_bateria, lectura_ADC;
int LOW_BAT_WARING_cont;

// OTROS
int LED_contador;

// MANDO RC
float RC_Throttle_filt, RC_Pitch_filt, RC_Yaw_filt, RC_Roll_filt;
float RC_Throttle_consigna, RC_Pitch_consigna, RC_Roll_consigna, RC_Yaw_consigna;

/// SEÑALES PWM
float ESC1_us, ESC2_us, ESC3_us, SERVO_us;

volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;

volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;

volatile long Roll_HIGH_us;
volatile int RC_Roll_raw;

volatile long Yaw_HIGH_us;
volatile int RC_Yaw_raw;

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
  uint8_t val;
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE	// activacion de bus I2C a 400 Khz
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  
  Serial.begin(115200);			// inicializacion de monitor serie a 115200 bps
  SerialBT.begin("ESP32_LABVIEW");
  while (!Serial); 			// espera a enumeracion en caso de modelos con USB nativo
  Serial.println(F("Inicio:"));		// muestra texto estatico
  SerialBT.println(F("Inicio:"));		// muestra texto estatico
#ifdef OFFSETS								// si existen OFFSETS
  Serial.println(F("Usando Offsets predefinidos"));			// texto estatico
  SerialBT.println(F("Usando Offsets predefinidos"));			// texto estatico
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS);	// inicializacion de sensor

#else										// sin no existen OFFSETS
  Serial.println(F(" No se establecieron Offsets, haremos unos nuevos.\n"	// muestra texto estatico
                   " Colocar el sensor en un superficie plana y esperar unos segundos\n"
                   " Colocar los nuevos Offsets en #define OFFSETS\n"
                   " para saltar la calibracion inicial \n"
                   " \t\tPresionar cualquier tecla y ENTER"));
  SerialBT.println(F(" No se establecieron Offsets, haremos unos nuevos.\n"	// muestra texto estatico
                   " Colocar el sensor en un superficie plana y esperar unos segundos\n"
                   " Colocar los nuevos Offsets en #define OFFSETS\n"
                   " para saltar la calibracion inicial \n"
                   " \t\tPresionar cualquier tecla y ENTER"));
  while (Serial.available() && Serial.read());		// lectura de monitor serie
  while (!Serial.available());   			// si no hay espera              
  while (Serial.available() && Serial.read()); 		// lecyura de monitor serie
  mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();	// inicializacion de sensor
#endif
  mpu.on_FIFO(mostrar_valores);	

  pinMode(pin_LED_rojo, OUTPUT); // Led naranja --> Batería baja
  pinMode(pin_LED_verde, OUTPUT);    // Led azul    --> Ciclo (parpadeo)
  pinMode(pin_LED_azul, OUTPUT);   // Led rojo 1  --> Error MPU6050

  //mando rc declaracion de onterrupciones
  pinMode(pin_INT_Yaw, INPUT_PULLUP);
  pinMode(pin_INT_Throttle, INPUT_PULLUP);
  pinMode(pin_INT_Pitch, INPUT_PULLUP);
  pinMode(pin_INT_Roll, INPUT_PULLUP);

  // Configuración de interrupciones con attachInterrupt
  attachInterrupt(digitalPinToInterrupt(pin_INT_Yaw), INT_Yaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Throttle), INT_Throttle, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Pitch), INT_Pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_INT_Roll), INT_Roll, CHANGE);

  // Declaración de los pines de los motores
  /*pinMode(pin_motor1, OUTPUT);  //Motor 1
  pinMode(pin_motor2, OUTPUT);  //Motor 2
  pinMode(pin_motor3, OUTPUT);  //Motor 3*/
  /*ledcAttachChannel(pin_motor1, pwmFreq, pwmResolution, pwm_channel_motor1);
  ledcAttachChannel(pin_motor2, pwmFreq, pwmResolution, pwm_channel_motor2);
  ledcAttachChannel(pin_motor3, pwmFreq, pwmResolution, pwm_channel_motor3);*/

  motor1.attach(pin_motor1); // Pin en el que funciona
  delay(1000);
  motor2.attach(pin_motor2); // Pin en el que funciona
  delay(1000);
  motor3.attach(pin_motor3); // Pin en el que funciona
  delay(1000);
  servoMotor.attach(pin_servoMotor,500,2400);
  delay(1000);

  //Activar el motor
  motor1.writeMicroseconds(2000); //2000 = 1ms
  delay(3000);
  motor2.writeMicroseconds(2000); //2000 = 1ms
  delay(3000);
  motor3.writeMicroseconds(2000); //2000 = 1ms
  delay(3000); 
  servoMotor.write(60);
  delay(3000);
  servoMotor.write(100);
  delay(3000);
  servoMotor.write(80);
  delay(3000);
  // Forzar los pines a estado LOW
  /*digitalWrite(pin_motor1, LOW);
  digitalWrite(pin_motor2, LOW);
  digitalWrite(pin_motor3, LOW);
  ledcWriteChannel(pwm_channel_motor1,velocidad);
  ledcWriteChannel(pwm_channel_motor2,velocidad);
  ledcWriteChannel(pwm_channel_motor3,velocidad);*/
  

  //
  
  Serial.println("Encender mando");
  SerialBT.println("Encender mando");

  if (MODO_vuelo == 1){
    Serial.println("-MODO Estable-");
    SerialBT.println("-MODO Estable-");
  } else{
    Serial.println("-MODO Acro-");
    SerialBT.println("-MODO Acro-");
  } 
  while (RC_Throttle_consigna < 950 || RC_Throttle_consigna > 1050) {
    RC_procesar();
    dron_estado="error";
    avisos(dron_estado);
  }
  //dron_estado="funcionando";
  //avisos(dron_estado);

  //MPU6050_iniciar();         // Iniciar sensor MPU6050
  //MPU6050_calibrar();        // Calibrar sensor MPU6050
  //Lectura_tension_bateria(); // Leer tension de batería

  /*Serial.println("ROLL ------>");
  while (RC_Roll_consigna < 10){
    RC_procesar();
  }*/
  
  loop_timer = micros();
  dron_estado="okStart";
  avisos(dron_estado);
  Serial.println("se inicio correctamente");
  SerialBT.println("se inicio correctamente");
}

void loop() {
  // Si se supera el tiempo de ciclo, se activa el LED rojo 2
  if (micros() - loop_timer > usCiclo + 50){
    dron_estado="error";
  }else if(LOW_BAT_WARING == true){
    dron_estado="lowBattery";
  }else{
    dron_estado="normal";
  }
  // Comienzo de un nuevo ciclo
  while (micros() - loop_timer < usCiclo);
  // Registrar instante de comienzo del ciclo
  loop_timer = micros();
  currentMillis = millis();

  avisos(dron_estado);

                             // Generar señales PWM para los motores
  mpu.dmp_read_fifo();
  //MPU6050_leer();                  // Leer sensor MPU6050
  //MPU6050_procesar();              // Procesar datos del sensor MPU6050  
  if (MODO_vuelo == 1)PID_ang();   // Obtener salida de los PID de inclinación
  PID_w();                         // Obtener salida de los PID de velocidad
  Modulador();                      // Modulador o generador de señales para PWM
  PWM();                  
  //servoMotor.write(SERVO_CONSIGNA);
  // Guardamos las lecturas del sensor MPU6050 para el siguiente ciclo (necesario para los PID)
  angulo_pitch_ant = angulo_pitch;
  angulo_roll_ant  = angulo_roll;
  angulo_yaw_ant   = angulo_yaw;
  gyro_X_ant = gyro_X; // Pitch
  gyro_Y_ant = gyro_Y; // Roll
  gyro_Z_ant = gyro_Z; // Yaw
  
  
  if (Serial.available() > 0){
    String command = Serial.readStringUntil('\n');
    command.toLowerCase();
    visu_select = command.toInt();
  }
  if (SerialBT.available() > 0){
    String command = SerialBT.readStringUntil('\n');
    command.toLowerCase();
    visu_select = command.toInt();
  }

  // Visualización de variables
  if (visu == 1)Visualizaciones();
}

void color(int R,int G, int B){
  R=255-R;
  G=255-G;
  B=255-B;
  analogWrite(pin_LED_rojo,R);
  analogWrite(pin_LED_verde,G);
  analogWrite(pin_LED_azul,B);
}

// Función para controlar las luces RGB
void avisos(String estado) {
  if (estado == "okStart") {
    color(0,255,0);
    delay(10000);
    color(0,0,0);
  } else if (estado == "lowBattery") {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      led_azul_state = !led_azul_state;
       led_azul_state ? color(255,0,0) : color(0,0,0);
    }
  } else if (estado == "error") {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      if (led_rojo_state) {
        led_rojo_state = false;
        led_azul_state = true;
      } else {
        led_rojo_state = true;
        led_azul_state = false;
      }
      led_rojo_state ? color(255,0,0) : color(0,0,0);
    }
  } else if (estado == "RCerror") {
    color(255,0,0);
    /*if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      if (led_rojo_state) {
        led_rojo_state = false;
        led_verde_state = true;
      } else {
        led_rojo_state = true;
        led_verde_state = false;
      }
      digitalWrite(pin_LED_rojo, led_rojo_state ? 0 : 255);
      digitalWrite(pin_LED_azul, led_azul_state ? 0 : 255);
    }*/
  }else if (estado == "normal") {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      led_verde_state = !led_verde_state;
      led_verde_state ? color(0,255,0) : color(0,0,0);
    }
  } else if (estado == "funcionando") {
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      led_rojo_state = !led_rojo_state;
      led_verde_state = !led_verde_state;
      led_azul_state = !led_azul_state;
      digitalWrite(pin_LED_rojo, led_rojo_state ? 0 : 255);
      digitalWrite(pin_LED_verde, led_verde_state ? 0 : 255);
      digitalWrite(pin_LED_azul, led_azul_state ? 0 : 255);
    }
  }
}

// PID ángulo
void PID_ang() {
  // PID ángulo - PITCH
  PID_ang_Pitch_error = RC_Pitch_consigna - angulo_pitch;                        // Error entre lectura y consigna
  PID_ang_Pitch_P  = Pitch_ang_Kp  * PID_ang_Pitch_error;                        // Parte proporcional
  PID_ang_Pitch_I += (Pitch_ang_Ki * PID_ang_Pitch_error);                       // Parte integral (sumatorio del error en el tiempo)
  PID_ang_Pitch_I  = constrain(PID_ang_Pitch_I, -PID_ang_sat1, PID_ang_sat1);    // Limitar parte integral
  PID_ang_Pitch_D  = Pitch_ang_Kd * (angulo_pitch - angulo_pitch_ant);           // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_ang_Pitch_OUT =  PID_ang_Pitch_P + PID_ang_Pitch_I + PID_ang_Pitch_D;      // Salida PID
  PID_ang_Pitch_OUT = constrain(PID_ang_Pitch_OUT, -PID_ang_sat2, PID_ang_sat2); // Limitar salida del PID

  // PID ángulo - ROLL
  PID_ang_Roll_error = RC_Roll_consigna - angulo_roll;                           // Error entre lectura y consigna
  PID_ang_Roll_P  = Roll_ang_Kp  * PID_ang_Roll_error;                           // Parte proporcional
  PID_ang_Roll_I += (Roll_ang_Ki * PID_ang_Roll_error);                          // Parte integral (sumatorio del error en el tiempo)
  PID_ang_Roll_I  = constrain(PID_ang_Roll_I, -PID_ang_sat1, PID_ang_sat1);      // Limitar parte integral
  PID_ang_Roll_D  = Roll_ang_Kd * (angulo_roll - angulo_roll_ant);               // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_ang_Roll_OUT = PID_ang_Roll_P + PID_ang_Roll_I + PID_ang_Roll_D;           // Salida PID
  PID_ang_Roll_OUT = constrain(PID_ang_Roll_OUT, -PID_ang_sat2, PID_ang_sat2);   // Limitar salida del PID
}

// PID velocidad angular
void PID_w() {
  // En funcion del modo de vuelo que hayamos seleccionado, las consignas de los PID serán diferentes
  if (MODO_vuelo == 0) {
    // En modo acrobático solo controlamos la velocidad de cada eje (un PID por eje). La consigna del PID se da en º/s
    // y viene directamente del mando RC
    PID_W_Pitch_consigna = RC_Pitch_consigna;
    PID_W_Roll_consigna  = RC_Roll_consigna;
  }
  else {
    // En modo estable las consignas de los PID de velocidad vienen de las salidas de los PID de ángulo
    PID_W_Pitch_consigna = PID_ang_Pitch_OUT;
    PID_W_Roll_consigna  = PID_ang_Roll_OUT;
  }

  // PID velocidad - PITCH
  PID_W_Pitch_error = PID_W_Pitch_consigna - gyro_X;                       // Error entre lectura y consigna
  PID_W_Pitch_P  = Pitch_W_Kp  * PID_W_Pitch_error;                        // Parte proporcional
  PID_W_Pitch_I += (Pitch_W_Ki * PID_W_Pitch_error);                       // Parte integral (sumatorio del error en el tiempo)
  PID_W_Pitch_I  = constrain(PID_W_Pitch_I, -PID_W_sat1, PID_W_sat1);      // Limitar parte integral
  PID_W_Pitch_D  = Pitch_W_Kd * (gyro_X - gyro_X_ant);                     // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_W_Pitch_OUT = PID_W_Pitch_P + PID_W_Pitch_I + PID_W_Pitch_D;         // Salida PID
  PID_W_Pitch_OUT = constrain(PID_W_Pitch_OUT, -PID_W_sat2, PID_W_sat2);   // Limitar salida del PID

  // PID velocidad - ROLL
  PID_W_Roll_error = PID_W_Roll_consigna - gyro_Y;                         // Error entre lectura y consigna
  PID_W_Roll_P  = Roll_W_Kp  * PID_W_Roll_error;                           // Parte proporcional
  PID_W_Roll_I += (Roll_W_Ki * PID_W_Roll_error);                          // Parte integral (sumatorio del error en el tiempo)
  PID_W_Roll_I  = constrain(PID_W_Roll_I, -PID_W_sat1, PID_W_sat1);        // Limitar parte integral
  PID_W_Roll_D  = Roll_W_Kd * (gyro_Y - gyro_Y_ant);                       // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_W_Roll_OUT = PID_W_Roll_P + PID_W_Roll_I + PID_W_Roll_D;             // Salida PID
  PID_W_Roll_OUT = constrain(PID_W_Roll_OUT, -PID_W_sat2, PID_W_sat2);     // Limitar salida del PID

  // PID velocidad - YAW
  PID_W_Yaw_error = RC_Yaw_consigna - gyro_Z;                              // Error entre lectura y consigna
  PID_W_Yaw_P  = Yaw_W_Kp  * PID_W_Yaw_error;                              // Parte proporcional
  PID_W_Yaw_I += (Yaw_W_Ki * PID_W_Yaw_error);                             // Parte integral (sumatorio del error en el tiempo)
  PID_W_Yaw_I  = constrain(PID_W_Yaw_I, -PID_W_servo, PID_W_servo);          // Limitar parte integral
  PID_W_Yaw_D  = Yaw_W_Kd * (gyro_Z - gyro_Z_ant);                         // Parte derivativa (diferencia entre el error actual y el anterior)

  PID_W_Yaw_OUT = PID_W_Yaw_P + PID_W_Yaw_I + PID_W_Yaw_D;                 // Salida PID
  PID_W_Yaw_OUT = constrain(PID_W_Yaw_OUT, -PID_W_servo, PID_W_servo);       // Limitar salida del PID
}

void Modulador() {
  // Si el Throttle es menos a 1300us, el control de estabilidad se desactiva. La parte integral
  // de los controladores PID se fuerza a 0.
  if (RC_Throttle_consigna <= 1300) {
    PID_W_Pitch_I = 0;
    PID_W_Roll_I = 0;
    PID_W_Yaw_I  = 0;
    PID_ang_Pitch_I = 0;
    PID_ang_Roll_I = 0;

    ESC1_us = RC_Throttle_consigna;
    ESC2_us = RC_Throttle_consigna;
    ESC3_us = RC_Throttle_consigna;
    SERVO_us = 80 + RC_Yaw_consigna;
    //SERVO_CONSIGNA = round(abs(SERVO_us)) * (SERVO_us < 0 ? -1 : 1);

    // Si lo motores giran con el stick de Throttle al mínimo, recudir el valor de 950us
    //if (ESC1_us < 1000) ESC1_us = 950;
    //if (ESC2_us < 1000) ESC2_us = 950;
    //if (ESC3_us < 1000) ESC3_us = 950;
    //if(SERVO_CONSIGNA > 110) SERVO_CONSIGNA = 110;
    if(SERVO_CONSIGNA < 50) SERVO_CONSIGNA = 50;
  }

  // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
  else {
    // Limitar throttle a 1800 para dejar margen a los PID
    if (RC_Throttle_consigna > 1800)RC_Throttle_consigna = 1800;
    if (RC_Yaw_consigna > 20)RC_Yaw_consigna=20;
    if (RC_Yaw_consigna < -20)RC_Yaw_consigna=-20;

    // Modulador
    ESC1_us = 1000 + ((RC_Throttle_consigna-1000)/2) + PID_W_Pitch_OUT + PID_W_Roll_OUT ; // Motor 1- PID_W_Yaw_OUT
    ESC2_us = 1000 + ((RC_Throttle_consigna-1000)/2) - PID_W_Roll_OUT ; // Motor 2+ PID_W_Yaw_OUT
    ESC3_us = RC_Throttle_consigna - PID_W_Pitch_OUT ; // Motor 3- PID_W_Yaw_OUT+ PID_W_Roll_OUT 
    //ESC4_us = RC_Throttle_consigna - PID_W_Pitch_OUT - PID_W_Roll_OUT + PID_W_Yaw_OUT; // Motor 4
    //    ESC1_us = RC_Throttle_filt; // Solo para testeos
    //    ESC2_us = RC_Throttle_filt;
    //    ESC3_us = RC_Throttle_filt;
    //    ESC4_us = RC_Throttle_filt;
    SERVO_us = 80 + RC_Yaw_consigna - PID_W_Yaw_OUT;
    


    // Evitamos que alguno de los motores de detenga completamente en pleno vuelo
    if (ESC1_us < 1200) ESC1_us = 1200;
    if (ESC2_us < 1200) ESC2_us = 1200;
    if (ESC3_us < 1200) ESC3_us = 1200;
    if(SERVO_CONSIGNA < 50) SERVO_CONSIGNA = 50;

    // Evitamos mandar consignas mayores a 2000us a los motores
    if (ESC1_us > 2000) ESC1_us = 2000;
    if (ESC2_us > 2000) ESC2_us = 2000;
    if (ESC3_us > 2000) ESC3_us = 2000;
    if(SERVO_CONSIGNA > 110) SERVO_CONSIGNA = 110;
    
  }
}

void PWM() {
  /*MOTOR1_CONSIGNA = map(ESC1_us, 1000,2000,245,492);
  MOTOR2_CONSIGNA = map(ESC2_us, 1000,2000,245,492);
  MOTOR3_CONSIGNA = map(ESC3_us, 1000,2000,245,492);*/
  SERVO_CONSIGNA = round(abs(SERVO_us)) * (SERVO_us < 0 ? -1 : 1);

  /*ledcWriteChannel(pwm_channel_motor1,MOTOR1_CONSIGNA);
  ledcWriteChannel(pwm_channel_motor2,MOTOR2_CONSIGNA);
  ledcWriteChannel(pwm_channel_motor3,MOTOR3_CONSIGNA);*/

  MOTOR1_CONSIGNA = ESC1_us;
  MOTOR2_CONSIGNA = ESC2_us;
  MOTOR3_CONSIGNA = ESC3_us;

  motor1.writeMicroseconds(MOTOR1_CONSIGNA);
  motor2.writeMicroseconds(MOTOR2_CONSIGNA);
  motor3.writeMicroseconds(MOTOR3_CONSIGNA);
  servoMotor.write(SERVO_CONSIGNA);
  
  RC_procesar();             // Leer mando RC
  Lectura_tension_bateria();
  
  tiempo_motores_start = micros();

  // ------------------ ¡¡1ms max!! ------------------
  tiempo_1 = micros();

   // Leer Vbat

  // Si la duracion entre tiempo_1 y tiempo_2 ha sido mayor de 900us, encender LED de aviso.
  // Nunca hay que sobrepasar 1ms de tiempo en estado HIGH.
  tiempo_2 = micros();
  tiempo_ON = tiempo_2 - tiempo_1;
  if (tiempo_ON > 900) dron_estado="cicloError";
  // ------------------ ¡¡1ms max!! ------------------
  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  /*digitalWrite(pin_motor1, HIGH);
  digitalWrite(pin_motor2, HIGH);
  digitalWrite(pin_motor3, HIGH);

  
  

  // Pasamos las señales PWM a estado LOW cuando haya transcurrido el tiempo definido en las variables ESCx_us
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_servoMotor) == HIGH) {
    if (tiempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW);
    if (tiempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW);
    if (tiempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW);
    //if (tiempo_motores_start + SERVO_us <= micros()) digitalWrite(pin_servoMotor, LOW);
  }*/
}

void Lectura_tension_bateria() {
  // La tensión de batería solo se lee con Throttle inferior a 1100us
  if (RC_Throttle_consigna < 1100) {
    // Leer entrada analógica
    lectura_ADC = analogRead(36);
    tension_bateria = 6.33* (lectura_ADC * 3.3  / 4095);

    // Si la tension de batería es inferior a 14V durante un número de ciclos consecutivos
    // se activa la señal de batería baja
    if (tension_bateria < 14.4 && LOW_BAT_WARING == false) {
      LOW_BAT_WARING_cont++;
      if (LOW_BAT_WARING_cont > 30)LOW_BAT_WARING = true;
    }
    else LOW_BAT_WARING_cont = 0;
  }
}

void RC_procesar() {
  //  Filtrado de lecturas raw del mando RC
  /*RC_Throttle_filt = RC_Throttle_filt * 0.9 + RC_Throttle_raw * 0.1;
  RC_Pitch_filt    = RC_Pitch_filt * 0.9 + RC_Pitch_raw * 0.1;
  RC_Roll_filt     = RC_Roll_filt  * 0.9 + RC_Roll_raw  * 0.1;
  RC_Yaw_filt      = RC_Yaw_filt   * 0.9 + RC_Yaw_raw   * 0.1;

  // Mapeo de señales del mando RC
  RC_Throttle_consigna = map(RC_Throttle_filt, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  RC_Pitch_consigna    = map(RC_Pitch_filt, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
  RC_Roll_consigna     = map(RC_Roll_filt, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
  RC_Yaw_consigna      = map(RC_Yaw_filt, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);*/

  // Ecuaciones de procesamiento
  RC_Throttle_consigna = map(RC_Throttle_raw, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  RC_Pitch_consigna    = map(RC_Pitch_raw, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
  RC_Roll_consigna     = map(RC_Roll_raw, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
  RC_Yaw_consigna      = map(RC_Yaw_raw, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);

  // Si las lecturas son cercanas a 0, las forzamos a 0 para evitar inclinar el drone por error
  if (RC_Pitch_consigna < 3 && RC_Pitch_consigna > -3)RC_Pitch_consigna = 0;
  if (RC_Roll_consigna  < 3 && RC_Roll_consigna  > -3)RC_Roll_consigna  = 0;
  if (RC_Yaw_consigna   < 3 && RC_Yaw_consigna   > -3)RC_Yaw_consigna   = 0;
}

void Visualizaciones() {

  // Visualizar variables por canal serie
  // Hay que seleccionar qué variable visualizar con visu_select
  if (visu == 1) {
    if (visu_select == 0) {
      Serial.print(RC_Pitch_consigna);
      SerialBT.print(RC_Pitch_consigna);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.print(RC_Roll_consigna);
      SerialBT.print(RC_Roll_consigna);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.print(RC_Yaw_consigna);
      SerialBT.print(RC_Yaw_consigna);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.println(RC_Throttle_consigna);
      SerialBT.println(RC_Throttle_consigna);
    }

    if (visu_select == 1) {
      Serial.print("potencia bateria:");
      SerialBT.print("potencia bateria:");
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.println(tension_bateria);
      SerialBT.println(tension_bateria);
    }

    if (visu_select == 2) {
      Serial.print(gyro_Z);
      SerialBT.print(gyro_Z);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.print(gyro_X);
      SerialBT.print(gyro_X);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.println(gyro_Y);
      SerialBT.println(gyro_Y);
    }

    if (visu_select == 3) {
      Serial.print(angulo_yaw);
      SerialBT.print(angulo_yaw);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.print(angulo_pitch);
      SerialBT.print(angulo_pitch);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.println(angulo_roll);
      SerialBT.println(angulo_roll);
    }

    if (visu_select == 4) {
      Serial.print(MOTOR1_CONSIGNA);
      SerialBT.print(MOTOR1_CONSIGNA);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.print(MOTOR2_CONSIGNA);
      SerialBT.print(MOTOR2_CONSIGNA);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.print(MOTOR3_CONSIGNA);
      SerialBT.print(MOTOR3_CONSIGNA);
      Serial.print("\t");
      SerialBT.print("\t");
      Serial.println(SERVO_CONSIGNA);
      SerialBT.println(SERVO_CONSIGNA);
    }

    if (visu_select == 999) {
    }
  }
}
