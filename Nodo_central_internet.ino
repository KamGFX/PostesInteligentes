#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include "time.h"

// Configuración de WiFi
const char* ssid = "A55Avocado";
const char* password = "juanjo200402";

// Configuración de InfluxDB
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "QvnD4zP1Kh9FBB4hdFZ2_vMF0XF21kaMG3frRw34AC-2IloS_flT53gemC0RcOxWDx_qqFACkOsKZlbmH9TIFg=="
#define INFLUXDB_ORG "cbe2cf2d9d7e7691"
#define INFLUXDB_BUCKET "Alumbrado_inteligente"

// Cliente de InfluxDB
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Point iluminacion("iluminacion");

// Configuración de NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -18000, 60000); // UTC-5, actualización cada 60s

// Configuración de UART
HardwareSerial SerialUART(2); // UART2 (pines 16 y 17)

// Variables
unsigned long lastTimeSync = 0;
const unsigned long TIME_SYNC_INTERVAL_1 = 10000;         
void setup() {
  Serial.begin(115200);
  SerialUART.begin(115200, SERIAL_8N1, 16, 17);

  // Conectar a WiFi
  connectWiFi();

  // Iniciar cliente NTP
  timeClient.begin();

  // Validar conexión a InfluxDB
  connectInfluxDB();
}

void loop() {
  // Verificar conexión WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi desconectado. Reconectando...");
    connectWiFi();
  }

  // Verificar conexión a InfluxDB
  if (!client.validateConnection()) {
    Serial.println("InfluxDB desconectado. Reconectando...");
    connectInfluxDB();
  }

  // Actualizar y enviar timestamp NTP periódicamente
  if (millis() - lastTimeSync >= TIME_SYNC_INTERVAL_1) {
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();
    if (epochTime > 0) {
      SerialUART.println(epochTime);
      Serial.print("Enviado timestamp por UART: ");
      Serial.println(epochTime);
      lastTimeSync = millis();
    }
  }

  // Procesar datos recibidos por UART
  if (SerialUART.available()) {
    String receivedData = SerialUART.readStringUntil('\n');
    Serial.print("Datos recibidos por UART: ");
    Serial.println(receivedData);

    // Parsear datos: timestamp,POSTE_ID,estado++7
    int firstComma = receivedData.indexOf(',');
    int secondComma = receivedData.indexOf(',', firstComma + 1);
    if (firstComma != -1 && secondComma != -1) {
      String timestampStr = receivedData.substring(0, firstComma);
      String posteId = receivedData.substring(firstComma + 1, secondComma);
      String estado = receivedData.substring(secondComma + 1);

      // Validar timestamp
      unsigned long timestamp = timestampStr.toInt();
      if (timestamp == 0) {
        Serial.println("Error: Timestamp inválido");
        return;
      }
      // Convertir timestamp a formato legible para depuración
      // Convertir timestamp a formato legible para depuración
      time_t rawTime = timestamp;
      struct tm *timeInfo = localtime(&rawTime);
      char timeStr[20];
      strftime(timeStr, sizeof(timeStr), "%d-%m-%Y %H:%M:%S", timeInfo);
      Serial.print("Timestamp legible: ");
      Serial.println(timeStr);

      // Preparar datos para InfluxDB
      iluminacion.clearFields();
      iluminacion.clearTags();
      iluminacion.addTag("poste_id", posteId);
      iluminacion.addTag("device", "esp32-internet");
      iluminacion.addField("movimiento", (estado == "DETECCION" ? 1 : 0));
      iluminacion.setTime(timestamp * 1000000000LL); // Timestamp en nanosegundos

      // Enviar a InfluxDB
      if (!client.writePoint(iluminacion)) {
        Serial.print("❌ Error al enviar a InfluxDB: ");
        Serial.println(client.getLastErrorMessage());
      } else {
        Serial.println("✔ Datos enviados a InfluxDB: poste_id=" + posteId + ", movimiento=" + String(estado == "DETECCION" ? 1 : 0) + ", timestamp=" + timeStr);
      }
    } else {
      Serial.println("Error: Formato de datos UART inválido");
    }
  }
}

// Función para conectar a WiFi
void connectWiFi() {
  Serial.println("Conectando a WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado a WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// Función para conectar a InfluxDB
void connectInfluxDB() {
  iluminacion.addTag("device", "esp32-internet");
  while (!client.validateConnection()) {
    Serial.print("Error conectando a InfluxDB ❌: ");
    Serial.println(client.getLastErrorMessage());
    delay(5000); // Esperar antes de reintentar
  }
  Serial.println("Conectado a InfluxDB ✔");
}