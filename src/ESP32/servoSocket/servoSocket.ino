#include <WiFi.h>
// #include <Servo.h>   // For Calvin's PC
#include <ESP32Servo.h> // For Kuka's lab

#define SSID "megamente"
#define PASSWD "guisoares10"
#define ERRO -1
#define SUCESS 0
#define EXIT 1
#define READY 2

Servo servo1, servo2;
int servoPin1 = 25;
int servoPin2 = 27;
const uint16_t port = 8890;
const char *host = "192.168.154.31";
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

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
}

void loop() {
  WiFiClient client;
  String msg = "";
  int out = 0;
  // Connect to server
  while (!client.connected()) {
    Serial.println("Tentando conectar com IP: 192.168.154.31");
    delay(2000);
    if (!client.connect(host, port)) {
      Serial.println("Falha de conexao");
      delay(1000);
      return;
    }
  }
  Serial.println("Conectado!");
  delay(200);
  // Session
  while( 1 ) {
    flag = READY;
    Serial.println("Esperando nova posicao...");
    // Listens until 10sec idle or 'exit' message
    for (int i = 0; i < 10000 && flag!=SUCESS; i++ ) {
      if (client.available() > 0) {
        msg = client.readStringUntil('\n');
        flag = readMessage(msg);
        if (flag != READY && flag != SUCESS) break;
      }
      delay(1);
    }
    switch (flag) {
      case READY :
        Serial.println("REINICIANDO POR INATIVIDADE!");
        out = 1;
        break;
      case ERRO :
        Serial.println("MENSAGEM MAL FORMATADA!");
        Serial.println("Tamanho: " + msg.length());
        Serial.println( "(" + msg + ")" );
        out = 1;
        break;
      case EXIT : 
        Serial.println("ENCERRANDO SISTEMA!");
        delay(2000);
        out = 1;
        break;
      case SUCESS :
        Serial.println(theta1);
        Serial.println(theta2);
        break;

      if ( out == 1 ) {
        client.stop();
        return;
      }
    }
    /*
      Control servo motors
    */
    if (theta1 <= 2) servo1.write(5);
    else servo1.write(theta1);
    if (theta2 <= 2) servo2.write(5);
    else servo2.write(theta2);
    if (theta1 >= 178) servo1.write(175);
    else servo1.write(theta1);
    if (theta2 >= 178) servo2.write(175);
    else servo2.write(theta2);

    delay(1);
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