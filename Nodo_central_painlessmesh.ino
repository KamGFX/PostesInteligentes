#include <HardwareSerial.h>
#include <painlessMesh.h>
#include <TimeLib.h>

// Configuración UART
HardwareSerial SerialUART(2); // UART2 (pines 16 y 17)

// Configuración PainlessMesh
#define MESH_PREFIX "AlumbradoMesh"
#define MESH_PASSWORD "Alumbrado123"
#define MESH_PORT 5555

// Configuración PIR
#define PIR_PIN 27
#define POSTE_ID "POSTE_CENTRAL"

// Temporizador para envío sin detección
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 5000; // 15 segundos


// VARIABLES GLOBALES
int EstadoAnteriorPIR = LOW;
const unsigned long TiempoEncendido = 3000;       // Tiempo en milisegundos
unsigned long TiempoUltMov = 0;                   // Almacena el momento de la última detección


painlessMesh mesh;
bool isTimeSynced = false;
unsigned long epochTime = 0;

void setup() {
  Serial.begin(115200);
  SerialUART.begin(115200, SERIAL_8N1, 16, 17); // UART con ESP32_Internet
  pinMode(PIR_PIN, INPUT);

  // Configurar PainlessMesh
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
}

void loop() {
  mesh.update();

  // Esperar sincronización inicial
  if (!isTimeSynced && SerialUART.available()) {
    String timeStamp = SerialUART.readStringUntil('\n');
    epochTime = timeStamp.toInt();
    if (epochTime > 0) {
      setTime(epochTime); // Sincronizar reloj interno
      isTimeSynced = true;
      Serial.println("Tiempo sincronizado: " + String(epochTime));
    }
  }

  if (isTimeSynced) {
    // Leer sensor PIR
    bool pirState = digitalRead(PIR_PIN);
    unsigned long currentTime = now(); // Obtener tiempo actual
    // Detectar cambios en el estado del sensor
    if (pirState != EstadoAnteriorPIR) {
    
      if (pirState == HIGH) {
      
        // Cuando el PIR detecta movimiento
        Serial.println("¡Movimiento detectado!");
        sendData(currentTime, POSTE_ID, "DETECCION");
        lastSendTime = millis(); // Reiniciar temporizador

      } else if (millis() - lastSendTime >= SEND_INTERVAL){
        // Cuando el PIR no detecta movimiento
        Serial.println("Sin movimiento.");
        // Sin detección, enviar cada 15 segundos
        sendData(currentTime, POSTE_ID, "SIN_DETECCION");
        lastSendTime = millis();
      }
      EstadoAnteriorPIR = pirState;          // Actualizar el estado anterior
    }    
  }
}

void sendData(unsigned long timestamp, String posteId, String state) {
  String data = String(timestamp) + "," + posteId + "," + state;
  SerialUART.println(data);
  Serial.println("Enviado por UART: " + data);
}

void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("Mensaje recibido de %u: %s\n", from, msg.c_str());
  // Procesar mensajes de los nodos (por ejemplo, almacenar o reenviar)
  SerialUART.println(msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("Nuevo nodo conectado: %u\n", nodeId);
  // Enviar timestamp NTP al nuevo nodo
  String msg = "TIME:" + String(epochTime);
  mesh.sendSingle(nodeId, msg);
}