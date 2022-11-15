#include <dummy.h>

#include <WiFi.h>
#include <ESP32Servo.h>

#define SSID "Eu tenho internet"
#define PASSWD "batatabatata"
#define ERRO -1
#define SUCESS 0
#define EXIT 1
#define READY 2

Servo servo1, servo2;
int servoPin1 = 25;
int servoPin2 = 27;
const uint16_t port = 8890;
const char *host = "192.168.0.2";
int theta1, theta2, flag;

void setup() {
  Serial.begin(9600);

  WiFi.begin(SSID, PASSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

  // Prints own IP
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  servo1.attach(servoPin1, 500, 2400);
  servo2.attach(servoPin2, 500, 2400);
}

void loop() {
  WiFiClient client;
  String msg;
  // Connect to server
  while (!client.connected()) {
    Serial.println("Tentando conectar com IP: 192.168.0.115");
    delay(2000);
    if (!client.connect(host, port)) {
      Serial.println("Falha de conexao");
      delay(1000);
      return;
    }
  }
  Serial.println("Conectado!");
  // Session
  while( 1 ) {
    delay(200);
    flag = READY;
    Serial.println("Esperando nova posicao...");
    // Listens until 10sec idle or 'exit' message
    for (int i = 0; i < 100 && flag!=SUCESS; i++ ) {
      if (client.available() > 0) {
        msg = client.readStringUntil('\n');
        flag = readMessage(msg);
        if (flag != READY && flag != SUCESS) break;
      }
      delay(100);
    }
    switch (flag) {
      case READY :
        Serial.println("REINICIANDO POR INATIVIDADE!");
      case ERRO :
        Serial.println("MENSAGEM MAL FORMATADA!");
        Serial.println("Tamanho: " + msg.length());
        Serial.println( "(" + msg + ")" );
      case EXIT : 
        Serial.println("ENCERRANDO SISTEMA!");
        delay(2000);
      case SUCESS :
        Serial.println(theta1);
        Serial.println(theta2);
        break;
      default :
        client.stop();
        return;
    }
    /*
      Control servo motors
    */
    if (theta1 <= 2) servo1.write(5);
    else servo1.write(theta1);
    if (theta2 <= 2) servo1.write(5);
    else servo2.write(theta2);
    if (theta1 >= 178) servo1.write(175);
    else servo1.write(theta1);
    if (theta2 >= 178) servo1.write(175);
    else servo2.write(theta2);
  }
}
/*
  Handle data from socket and set angles
*/
int readMessage(String msg) {
  // Exit case <- "exit"
  if (msg.equals("exit")) return EXIT;
  // Check size
  if (msg.length() != 6) return ERRO;
  // Check data
  for( int i = 0; i < msg.length(); i++ )
    if ( msg[i] < 48 && msg[i] > 57 ) 
      return ERRO;
  theta1 = (int)msg.substring(0, 3).toInt();
  theta2 = (int)msg.substring(3).toInt();
  return SUCESS;
}