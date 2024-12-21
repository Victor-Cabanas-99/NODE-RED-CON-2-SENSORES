# NODE-RED-CON-2-SENSORES
Practica con un sensor DHT de temperatura y humedad como un sensor ULTRASONICO, en una  ESP32 para una comunicación en Node red, creando una Interfaz HMI
## MATERIAL A UTILIZAR
- [WOKWI](https://wokwi.com/projects/new/esp32)
- TARJET ESP32
- SENSOR ULTRASONICO HC-SR04
- SENSOR DHT22
- NODE RED
- CDM
  
## INSTRUCCIONES
Insertar el codigo dado de la practica, se realizara la misma actividad que la practica anterior mostrando datos de quien esta programando:

```
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define BUILTIN_LED 2
#include "DHTesp.h"
const int DHT_PIN = 15;
DHTesp dhtSensor;
// Update these with values suitable for your network.

const int Trigger = 4;   //Pin digital 2 para el Trigger del sensor
const int Echo = 16;   //Pin digital 3 para el Echo del sensor
const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "3.76.79.164";
String username_mqtt="VictorC";
String password_mqtt="12345678";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   
    // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  
    // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), username_mqtt.c_str() , password_mqtt.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}

void loop() {

 long t; //timepo que demora en llegar el eco
  long d; //distancia en centimetros

  digitalWrite(Trigger, HIGH);
  delayMicroseconds(10);          //Enviamos un pulso de 10us
  digitalWrite(Trigger, LOW);
  
  t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
  d = t/59;             //escalamos el tiempo a una distancia en cm
  
  Serial.print("Distancia: ");
  Serial.print(d);      //Enviamos serialmente el valor de la distancia
  Serial.print("cm");
  Serial.println();
  delay(2000);          //Hacemos una pausa de 100ms
  
delay(1000);
TempAndHumidity  data = dhtSensor.getTempAndHumidity();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    //++value;
    //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);

    StaticJsonDocument<128> doc;

    doc["DEVICE"] = "ESP32";
    //doc["Anho"] = 2022;
    //doc["Empresa"] = "Educatronicos";
    doc["TEMPERATURA"] = String(data.temperature, 1);
    doc["HUMEDAD"] = String(data.humidity, 1);
    doc["DISTANCIA"] = String(d);
   doc["NOMBRE"] = "VICTOR CABAÑAS";

    String output;
    
    serializeJson(doc, output);

    Serial.print("Publish message: ");
    Serial.println(output);
    Serial.println(output.c_str());
    client.publish("GOTEAM", output.c_str());
  }
}
```
2. Instalar la libreria de **ArduinoJson**, **PubSubClient** y **DHT sensor library for ESPx** como se muestra en la siguente imagen.

![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20LIBRERIA.PNG?raw=true)

3. Hacer la conexion del **HC-SR04 ULTRASONIC DISTANCE SENSOR** y el **DHT sensor library for ESPx** con la **ESP32** como se muestra en la siguente imagen.

![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20CONEXCION.PNG?raw=true)

4.- Abrimos la CMD como administrador, para ejecutar el Programa Node-red debemos escribir en el CMD: npm install -g --unsafe-perm node-red node-red

5.- Para abrir la aplicación nos vamos algun explorador y colocamos el siguente link: localhost:1880

6.- ejecutamos el CMD como administrador escribimos nslookup broker.emqx.io y copiamos el Address que nos genera.

7.- una vez en node-red colocamos un bloque **mqqtt in** y configuramos el bloque con el puerto mqtt con el ip 3.76.79.164 como se muestra en la imagen.
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUE%20MQTT.PNG?raw=true)
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUE%20MQTT%20CONFIGURADO.PNG?raw=true)

8.- Colocar el bloque **json** y configurarlo como se muestra en la imagen.
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUE%20JSON.PNG?raw=true)
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUE%20JSON%20CONFIGURADO.PNG?raw=true)

9.- Colocamos un bloque **Debug** para la comunicacion serial
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUE%20DEBUG.PNG?raw=true)

10.- Colocamos tres bloques function (TEMPERATURA, HUMEDAD y DISTANCIA) y los configuramos con los siguentes codigos.
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUES%20DE%20FUNCION.PNG?raw=true)
**TEMPERATURA**
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUES%20DE%20FUNCION%20confg.%201.PNG?raw=true)
**HUMEDAD**
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUES%20DE%20FUNCION%20confg.%202.PNG?raw=true)
**DISTANCIA**
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUES%20DE%20FUNCION%20confg.%203.PNG?raw=true)

11.- Añadimo en el DASHBOARD tablas y 2 grupos para GRAFICAS e iNDICADORES 
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20TABLAS.PNG?raw=true)

12.- Colocamos un bloque de chart y gauge para cada funcion para poder visualisar grafica como indicadores.
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20BLOQUES%20GRAFICAS.PNG?raw=true)

13.- Colocamos la relciones en el NODE RED
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20ACOMODO.PNG?raw=true)

### Instrucciónes de operación
1.-Seleccionamos DEPLOY en node-red para iniciar la comunicacion.
2.- Seleccionamos DASHBOARD en node-red.
3.- Abrimos el localhost en el dashboard.
4. Iniciar simulador.
5. Visualizar los datos en el monitor serial del localhost para comprobar la conexcion.
6. Colocar la distancia *doble click* al sensor **HC-SR04 ULTRASONIC DISTANCE SENSOR**
7. Colocar la temperatura y humedad *doble click* al sensor **DHT sensor library for ESPx**
8.-Visualizamos las graficas como los parametros.
## Resultados

Cuando haya funcionado, verás los valores en le localhost como se muestra en la siguente imagen..
![](https://github.com/Victor-Cabanas-99/NODE-RED-CON-2-SENSORES/blob/main/NODE%20RED%202%20SENESORES%20Graficos.PNG?raw=true)
## Librerías

1. **PubSubClient**
2. **ArduinoJson**
3. **DHT sensor library for ESPx**
