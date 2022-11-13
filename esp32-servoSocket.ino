#include <WiFi.h>
 
#define SSID "Eu tenho internet"
#define PASSWD "batatabatata"
#define ERRO -1

const uint16_t port = 8890;
const char * host = "192.168.0.115";
int theta1, theta2;

 
void setup(){
    Serial.begin(9600);
 
    WiFi.begin( SSID, PASSWD );
    while ( WiFi.status() != WL_CONNECTED ) {
      delay(100);
    }

    Serial.print("IP: ");
    Serial.println( WiFi.localIP() );
}
 
void loop(){   
 
    WiFiClient client;

    if (!client.connect(host, port)) {
      Serial.println("Falha de conexao");
      delay(1000);
      return;
    }
 
    Serial.println("Conectado!\nEsperando mensagem...");

    delay(100);  // Must wait to prevent read errors
    
    /*
      Handle data from socket and return angles
    */
    if (client.available() > 0)  {
      String msg = client.readStringUntil('\n');
      int flag = readMessage( msg );
      if (flag == ERRO) return;
    }    

    Serial.println(theta1);
    Serial.println(theta2);
    /*
      Control servo motors
    */
          
    client.stop();

    Serial.println("Fim da conexao");
}

int readMessage( String msg ) {
  // Verify message format
  if ( msg.length() != 6 ) {
    Serial.println("ERRO MENSAGEM: Tamanho diferente de 6!");
    Serial.println(msg.length());
    return ERRO;
  }
  theta1 = (int) msg.substring(0,3).toInt();
  theta2 = (int) msg.substring(3).toInt();  
  return 0;
}