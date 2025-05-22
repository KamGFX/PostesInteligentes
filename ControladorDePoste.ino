/*
Codigo de POSTE NODO CENTRAL
Universidad del Cauca
Proyecto Integrador
2025-1
*/

//_____________________________________________________ DEFINICION DE LIBRERIAS ______________________________________________________________

  #include <HardwareSerial.h>
  #include <painlessMesh.h>
  #include <TimeLib.h>

//

//_______________________________________________________ DEFINICION DE PINES ________________________________________________________________

  // Sensor PIR y Led de Control
  const int Pin_PIR = 32;               // Pin GPIO 32 para el sensor PIR (Lectura Digital)
  const int ledPin = 2;                 // Pin GPIO 2 para el LED (Salida led ESP)
  const int ledPostePin = 13;           // Pin GPIO 13 para el LED externo (Salida de Voltaje)

  // Controlador Puente H
  const int ENA = 14;                   // Pin 14 PWM para controlar brillo Enable A (ENA - PuenteH)
  const int IN1 = 26;                   // Pin 26 para controlar dirección/polaridad (IN1 - PuenteH)
  const int IN2 = 27;                   // Pin 27 para controlar dirección/polaridad (IN2 - PuenteH)

  // Configuración UART
  HardwareSerial SerialUART(2);         // UART2 (pines 16 y 17)

//

//_________________________________________________________ DEFINICIONES _____________________________________________________________________

  // Configuración PainlessMesh
  #define MESH_PREFIX "AlumbradoMesh"
  #define MESH_PASSWORD "Alumbrado123"
  #define MESH_PORT 5555

  #define POSTE_ID "POSTE_CENTRAL"  // Cambiar por ID único para cada poste

//

//_______________________________________________________ VARIABLES GLOBALES _________________________________________________________________

  int EstadoAnteriorPIR = LOW;
  const unsigned long TiempoEncendido = 6000;    // Tiempo en milisegundos
  unsigned long TiempoUltMov = 0;                // Última detección local
  unsigned long TiempoUltMovExterno = 0;         // Última detección externa
  unsigned long lastSendTime = 0;                // Temporizador para envío periódico
  const unsigned long SEND_INTERVAL = 15000;     // Enviar cada 15 segundos (ajustado para menos redundancia)
  painlessMesh mesh;
  bool isTimeSynced = false;
  unsigned long epochTime = 0;
  unsigned long lastTimeSyncAttempt = 0;         // Control de reintentos de sincronización
  
  const unsigned long TIME_SYNC_INTERVAL_1 = 5000;
//

//___________________________________________________ DECLARACION DE Void Setup () ____________________________________________________________
  void setup() {

    // Iniciar comunicación serial
      Serial.begin(115200);
      SerialUART.begin(115200, SERIAL_8N1, 16, 17);         // UART con ESP32_Internet
    //

    // Configuracion los pines
      pinMode(Pin_PIR, INPUT);                        // Pin del sensor PIR como entrada
      pinMode(ledPin, OUTPUT);                        // Pin del LED como salida (opcional)
      pinMode(ledPostePin, OUTPUT);                   // Pin del LED como salida (opcional)
    //

    // Configurar pines del L298N como salida
      pinMode(IN1, OUTPUT);
      pinMode(IN2, OUTPUT);
      pinMode(ENA, OUTPUT);
    //

    // Configurar PWM con ledc
    ledcSetup(0, 5000, 8);  // Canal 0, 5000 Hz, resolución 8 bits
    ledcAttachPin(ENA, 0);  // Asocia GPIO 14 al canal 0

    // Estado inicial: bombillo apagado
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      ledcWrite(0, 0);
    //

    // Configuracion Painless Mesh
      mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // Configurar PainlessMesh
      mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT);
      mesh.onReceive(&receivedCallback);
      mesh.onNewConnection(&newConnectionCallback);

      Serial.println("Sistema iniciado. Escaneando movimiento...");
    //
  }
//

//____________________________________________________ DECLARACION DE Void Loop () ____________________________________________________________

  void loop() {
    mesh.update();

  // Procesar datos recibidos por UART
  if (SerialUART.available()) {
    String receivedData = SerialUART.readStringUntil('\n');
    Serial.print("Datos recibidos por UART: ");
    Serial.println(receivedData);
  }
  // Intentar sincronización de tiempo si no está sincronizado
  if (!isTimeSynced && SerialUART.available()) {
    String timeStamp = SerialUART.readStringUntil('\n');
    epochTime = timeStamp.toInt();
    if (epochTime > 0) {
      setTime(epochTime);
      isTimeSynced = true;
      Serial.println("Tiempo sincronizado: " + String(epochTime));
    }
  }
  //
  // Reintentar sincronización cada 60 segundos si falla
  if (!isTimeSynced && millis() - lastTimeSyncAttempt >= TIME_SYNC_INTERVAL_1) {
    Serial.println("Reintentando sincronización de tiempo...");
    lastTimeSyncAttempt = millis();
  }

    // Manejo de PIR, control de temporizador y envio de datos
    if (isTimeSynced) {

      // Leer sensor PIR
      bool pirState = digitalRead(Pin_PIR);
      unsigned long currentTime = now();

      // Detectar cambios en el estado del sensor
      if (pirState != EstadoAnteriorPIR) {  
        if (pirState == HIGH) {
          // Cuando el PIR detecta movimiento
          Serial.println("¡Movimiento detectado!");
          TiempoUltMov = millis();
          sendData(currentTime, POSTE_ID, "DETECCION");
          lastSendTime = millis(); // Reiniciar temporizador
        } 
      // Actualizar el estado anterior
      EstadoAnteriorPIR = pirState;
      
      }
      if (pirState == LOW && millis() - lastSendTime >= SEND_INTERVAL){
        // Cuando el PIR no detecta movimiento
        Serial.println("Sin movimiento.");
        // Sin detección, enviar cada 15 segundos
        sendData(currentTime, POSTE_ID, "SIN_DETECCION");
        lastSendTime = millis();
      }    

    // Mantener el LED encendido mientras no hayan pasado TiempoEncendido milisegundos
      if (millis() - TiempoUltMov < TiempoEncendido) {
        digitalWrite(ledPostePin, HIGH);
        digitalWrite(IN1, HIGH);  // Configurar polaridad para encender
        digitalWrite(IN2, LOW);
        ledcWrite(0, 255);;  // Máxima intensidad (100%)
      } else {
        digitalWrite(ledPostePin, LOW);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        ledcWrite(0, 64);  // Ahorro de intensidad (25%)
      }
    if (millis() - TiempoUltMovExterno < TiempoEncendido) {
    digitalWrite(ledPostePin, HIGH);
    digitalWrite(IN1, HIGH);  // Configurar polaridad para encender
    digitalWrite(IN2, LOW);
    ledcWrite(0, 255);  // Máxima intensidad (100%)
    }
  }  
}
//

//_____________________________________________________ DECLARACION DE FUNCIONES ______________________________________________________________

  void sendData(unsigned long timestamp, String posteId, String state) {
    String data = String(timestamp) + "," + posteId + "," + state;
    mesh.sendBroadcast(data);
    SerialUART.println(data);
    Serial.println("Enviado por UART: " + data);
  }

  void receivedCallback(uint32_t from, String &msg) {
    Serial.printf("Mensaje recibido de %u: %s\n", from, msg.c_str());
    // Parsear el mensaje (formato: timestamp,POSTE_ID,estado)
    int comma1 = msg.indexOf(',');
    int comma2 = msg.lastIndexOf(',');
    if (comma1 != -1 && comma2 != -1 && comma2 > comma1) {
      String state = msg.substring(comma2 + 1);
      if (state == "DETECCION") {
        Serial.println("Detección recibida de poste adyacente.");
        TiempoUltMovExterno = millis();  // Activar temporizador para detección externa
      } else if (state == "SIN_DETECCION") {
        Serial.println("Sin detección recibida de poste adyacente (sin acción).");
      }
      // Reenviar mensaje a ESP32_Internet
      SerialUART.println(msg);
    }else {
    Serial.println("Error: Formato de mensaje inválido");
    }
  }

  void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("Nuevo nodo conectado: %u\n", nodeId);
    if (isTimeSynced) {
      String msg = "TIME:" + String(epochTime);
      mesh.sendSingle(nodeId, msg);
      Serial.println("Enviado timestamp NTP al nodo: " + String(nodeId));
    }
  }
//