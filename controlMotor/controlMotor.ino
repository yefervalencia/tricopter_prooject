#include <ESP32Servo.h>




#define pin_motor1 25        // Pin motor 1 motor delantero izquierdo
#define pin_motor2 33        // Pin motor 2 motor delantero derecho
#define pin_motor3 26 


Servo motor1;
Servo motor2;
Servo motor3;


int velocidad = 1000; // Amplitud minima de pulso para tu ESC

void setup() {

  motor1.attach(pin_motor1); // Pin en el que funciona
  delay(1000);
  motor2.attach(pin_motor2); // Pin en el que funciona
  delay(1000);
  motor3.attach(pin_motor3); // Pin en el que funciona
  delay(1000);

  //Activar el motor
  motor1.writeMicroseconds(2000); //2000 = 1ms
  delay(3000);
  motor2.writeMicroseconds(2000); //2000 = 1ms
  delay(3000);
  motor3.writeMicroseconds(2000); //2000 = 1ms
  delay(3000);


  motor1.writeMicroseconds(1000); //1000 = 1ms
  delay(3000);
  motor2.writeMicroseconds(1000); //1000 = 1ms
  delay(3000);
  motor3.writeMicroseconds(950); //1000 = 1ms
  delay(3000);
  //Cambia el 1000 anterior por 2000 si
  //tu ESC se activa con un pulso de 2ms   

  Serial.begin(115200);
  Serial.setTimeout(10);

}

void loop() {
  if (Serial.available() > 0){
    String command = Serial.readStringUntil('\n');
    command.toLowerCase();
    velocidad = command.toInt();
  }
  motor1.writeMicroseconds(velocidad);
  motor2.writeMicroseconds(velocidad);
  motor3.writeMicroseconds(velocidad);

  Serial.println(velocidad);

}
