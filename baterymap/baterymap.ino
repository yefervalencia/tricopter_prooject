#define usCiclo 50000  // Ciclo de ejecución de software en microsegundos

long  loop_timer;
float tension_bateria, lectura_bat = 0.00;

void setup() {
  Serial.begin(115200);
}

void loop() {

  while (micros() - loop_timer < usCiclo);  // Hacemos una lectura cada 'usCiclo' microsegundos
  loop_timer = micros();

  lectura_bat = analogRead(36);
  tension_bateria =  6.33* (lectura_bat * 3.3  / 4095);
  Serial.print(tension_bateria);
  Serial.print("\t");
  Serial.println(lectura_bat);

}