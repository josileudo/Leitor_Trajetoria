#include <Arduino.h>
#include <Adafruit_sensor.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <WiFi.h>
#include <TinyGPS++.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <iostream>
#include <WebSocketsServer.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>

//Conectado na minha rede:
const char *ssid = "brisa-248210";
const char *password = "4vo56255";

//Roteando o ESP32
//const char* ssid = "ESP32-Acess-Point";
//const char* password = "123456789";

IPAddress ip(192, 168, 1, 8);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

Adafruit_BMP280 bmp;

static const int RXPin = 2, TXPin = 4;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

float latitude, longitude, hora, minutos, segundos;
String latStr, lngStr, dateStr, dayStr, timeStr, monthStr, yearStr, tempStr, altStr, presStr;
int hour, minute, second, pm;

WiFiServer server(80);

void setup()
{

  Serial.begin(115200);
  gpsSerial.begin(GPSBaud);
  Serial.println(F("BMP280 Test"));

  Serial.print("Conectando...");
  Serial.println(ssid);
  WiFi.begin(ssid, password); // Utilizando o WiFi
  WiFi.config(ip, gateway, subnet);
  //WiFi.softAP(ssid, password); // ESP32 roteando

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  Serial.print("");
  Serial.print("WIFI Conectado");
  Serial.print("Endereço de IP: ");
  Serial.print(WiFi.localIP()); // Endereço do WIFI

  Serial.print(ssid);
  server.begin();

  /*Conectando ao IP do ESP32
  IPAddress IP = WiFi.softAPIP();
  Serial.println(IP);
  server.begin();*/

  //conexão com BMP280
  if (!bmp.begin(0x76))
  {
    Serial.println(F("Sensor BMP280 Não encontrado..."));
    while (true)
      ;
  }

  bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_500);
}

void loop()
{

  // -> Configurando GPS <-
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
    {
      Serial.print(F("Localização: "));

      if (gps.location.isValid())
      {
        latitude = gps.location.lat();
        longitude = gps.location.lng();

        latStr = String(latitude, 6); //String(latitude, 6);
        lngStr = String(longitude, 6);

        Serial.print(gps.location.lat(), 6);
        Serial.print(F(","));
        Serial.println(longitude, 6);
      }
      else
      {
        Serial.println(F("INVÁLIDO"));
      }

      Serial.print(F("Data = "));
      if (gps.date.isValid())
      {
        dayStr = String(gps.date.day());
        monthStr = String(gps.date.month());
        yearStr = String(gps.date.year());

        dateStr = "";
        if (gps.date.day() < 10)
          dateStr += "0";
        dateStr += dayStr;
        dateStr += "/";

        if (gps.date.month() < 10)
          dateStr += "0";
        dateStr += monthStr;
        dateStr += "/";

        dateStr += yearStr;

        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.println(gps.date.year());
      }
      else
      {
        Serial.print(F(" DATA INVÁLIDA"));
      }

      Serial.print(F("Tempo = "));
      if (gps.time.isValid())
      {
        hour = gps.time.hour();
        minute = gps.time.minute();
        second = gps.time.second();

        timeStr = "";
        minute = (minute + 0);
        if (minute > 59)
        {
          minute -= 60;
          hour += 1;
        }
        hour = (hour + 21);
        if (hour > 23)
        {
          hour = 23;
          hour -= 24;
        }

        if (hour < 10)
          timeStr += "0";
        timeStr += String(hour);
        timeStr += (":");

        if (minute < 10)
          timeStr += "0";
        timeStr += String(minute);
        timeStr += (":");

        if (second < 10)
          timeStr += "0";
        timeStr += String(second);

        Serial.print(timeStr);
      }
      else
      {
        Serial.print("TEMPO INVÁLIDO");
      }

      if (millis() > 5000 && gps.charsProcessed() < 10)
      {
        Serial.println(F("GPS não detectado: Por favor, checar..."));
        while (true)
          ;
      }
    }
  // BARÔMETRO
  Serial.print(F("Temperatura = "));
  Serial.println(bmp.readTemperature());
  tempStr = String(bmp.readTemperature());

  Serial.print(F("Pressão = "));
  Serial.println(bmp.readPressure());
  presStr = (bmp.readPressure());

  Serial.print(F("Altitude = "));
  Serial.println(bmp.readAltitude());
  altStr = (bmp.readAltitude());

  Serial.println();
  delay(2000);

  // -> Enviando dados para o cliente <-
  WiFiClient client = server.available();
  if (!client)
  {
    return;
  }

  /*Serial.println("Novo cliente se conectou!");
  while (!client.available())
  {
    delay(1);
  }*/
  const size_t capacity = JSON_ARRAY_SIZE(6) + JSON_OBJECT_SIZE(6) + 60;
  DynamicJsonDocument root(capacity);

  String s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n <!DOCTYPE html><html><head><title>Leitor de Trajetorias ESP32</title><style>";
  s += ("Access-Control-Allow-Origin: *");
  s += ("Access-Control-Allow-Methods: GET");
  root["Hora"] = timeStr;
  root["Data"] = dateStr;
  root["Temperatura"] = tempStr;
  root["Pressao"] = presStr;
  root["Altitude"] = altStr;

  JsonArray location = root.createNestedArray("Location");
  location.add(latStr);
  location.add(lngStr);

  serializeJson(root, client);
  Serial.println("Client desconectado");
  Serial.println("");

  delay(1000);
}
