#include <Arduino.h>
#include <Wire.h>

// Pantalla ILI9488
#include "SPI.h"
#include <Adafruit_GFX.h>
#include <ILI9488.h>

// KeyPad 3x4
#include <Keypad_I2C.h>
#include <Keypad.h>
#include <PCF8574_HD44780_I2C.h>

// DHT22
#include <DHT.h>

// WIFI
#include <WiFi.h>

// Server
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Definiciones de pines
#define TFT_RESET 16
#define TFT_CS 12
#define TFT_DC 14
#define TFT_MOSI 15
#define TFT_SCK 18

#define KEYPAD_SDA 21
#define KEYPAD_SCL 22

#define OA_F_1 32
#define OA_F_2 33
#define OA_F_3 34
#define OA_F_4 35

#define PREAMP_UD 25
#define PREAMP_INC 26
#define PREAMP_CS 27

#define AMP_UD 21
#define AMP_INC 22
#define AMP_CS 23

#define DATA_SENSOR_TEMP_AND_HR 0
#define DHTTYPE DHT22

#define MIC_SIGNAL_OUT 2

#define BUZZER_PIN 5

// Variables globales para datos capturados
volatile float temperatura = 0.0;
volatile float humedad = 0.0;
volatile int micValue = 0;

// Credenciales WIFI
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"

// Credenciales Server
#define MQTT_SERVER "mqtt_server"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt_user"
#define MQTT_PASSWORD "mqtt_password"

// Configuración del cliente MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Instancia del Sensor DHT22
DHT dht(DATA_SENSOR_TEMP_AND_HR, DHTTYPE);

// Inicialización de la pantalla TFT
Adafruit_ILI9488 tft = Adafruit_ILI9488(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RESET);

// Configuración del teclado matricial
const byte ROWS = 4; // Cuatro filas
const byte COLS = 3; // Tres columnas
char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};
byte rowPins[ROWS] = {0, 1, 2, 3}; // Pines de las filas conectados al PCF8574
byte colPins[COLS] = {4, 5, 6};    // Pines de las columnas conectados al PCF8574

Keypad_I2C kpd(makeKeymap(keys), rowPins, colPins, ROWS, COLS, KEYPAD_SDA, KEYPAD_SCL, PCF8574_ADDRESS);

void setup()
{
  // Inicialización del Serial
  Serial.begin(115200);

  // Inicialización de la comunicación SPI
  SPI.begin();

  // Inicialización de la comunicación I2C
  Wire.begin();

  // Inicialización de los sensores
  initSensors();

  // Configuración del cliente MQTT
  setupMQTT();

  // Inicialización de la pantalla TFT
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9488_BLACK);
  tft.setTextColor(ILI9488_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Inicializando...");

  // Inicialización del teclado matricial
  Wire.begin(KEYPAD_SDA, KEYPAD_SCL);
  kpd.begin();

  // Configuración de los pines de las teclas de función como entrada
  pinMode(OA_F_1, INPUT_PULLUP);
  pinMode(OA_F_2, INPUT_PULLUP);
  pinMode(OA_F_3, INPUT_PULLUP);
  pinMode(OA_F_4, INPUT_PULLUP);

  // Configuración de las resistencias digitales variables:
  // Preamplificador (Mod_Sensibilidad)
  pinMode(PREAMP_UD, OUTPUT);
  pinMode(PREAMP_INC, OUTPUT);
  pinMode(PREAMP_CS, OUTPUT);
  // Amplificador (Mod_Ganancia)
  pinMode(AMP_UD, OUTPUT);
  pinMode(AMP_INC, OUTPUT);
  pinMode(AMP_CS, OUTPUT);

  // Configuración de los pines del sensor de temperatura-humedad y del micrófono
  pinMode(DATA_SENSOR_TEMP_AND_HR, INPUT);
  pinMode(MIC_SIGNAL_OUT, INPUT);

  // Configuración de pines para lecturas de interrupción
  attachInterrupt(digitalPinToInterrupt(DATA_SENSOR_TEMP_AND_HR), leerTempHumInterrupcion, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MIC_SIGNAL_OUT), leerMicrofonoInterrupcion, CHANGE);

  // Configuración del buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Verificación de errores de lectura inicial
  if (checkReadErrors())
  {
    // Sección de ERROR: si hay errores de lectura, activar el buzzer y detener el programa
    Serial.println("Error de lectura inicial.");
    while (true)
    {
      digitalWrite(BUZZER_PIN, HIGH);
      delay(1000);
      digitalWrite(BUZZER_PIN, LOW);
      delay(1000);
    }
  }

  // Verificación de conexión Wi-Fi
  if (!checkWiFiConnection())
  {
    // Solicitar Modo sin Conexión
    Serial.println("No se pudo conectar al Wi-Fi. Desea continuar en modo sin conexión? (y/n)"); // F1 = Yes , F2 = No
    while (!Serial.available())
      ;
    char response = Serial.read();
    if (digitalRead(OA_F_1) == HIGH)
    {
      // Continuar en modo sin conexión
      Serial.println("Continuando en modo sin conexión...");
      // Ir a HOME
      homeScreen(); // Aquí deberías implementar la lógica para mostrar la pantalla HOME sin conexión
      break;
    }
    else if (digitalRead(OA_F_2) == HIGH)
    {
      // Ir a CONFI (configuración)
      Serial.println("Dirigiendo a configuración...");
      break;
    }
    else
    {
      Serial.println("Conexión Wi-Fi exitosa.");
      // Continuar al siguiente punto
    }

    // Intentar conectar al servidor remoto
    if (!connectMQTT())
    {
      // Sección de ERROR: si hay errores de conexión, activar el buzzer y detener el programa
      Serial.println("Error de conexión con el servidor remoto.");
      while (true)
      {
        digitalWrite(BUZZER_PIN, HIGH);
        delay(1000);
        digitalWrite(BUZZER_PIN, LOW);
        delay(1000);
      }
    }

    // Verificar si hay tareas pendientes
    if (checkPendingTasks())
    {
      Serial.println("Tareas pendientes encontradas.");
      // Aquí se leería la información de la tarea pendiente
      // Esperar confirmación del usuario para continuar
      Serial.println("Desea continuar con la tarea? (F1: Continuar, F2: Cancelar)");

      while (!Serial.available())

        char response = Serial.read();

      if (response = '1' && (OA_F_1) == HIGH)
      {
        // Cargar Información de la Tarea
        Serial.println("Cargando información de la tarea...");
        // Monitor de Tarea
        break;
      }
      else if (digitalRead(OA_F_2) == HIGH)
      {
        // Ir a HOME
        Serial.println("Cancelando tarea. Dirigiendo a HOME...");
        homeScreen(); // Implementar la lógica para mostrar la pantalla HOME
        break;
      }
    }
    else
    {
      Serial.println("No hay tareas pendientes.");
      // Ir a HOME
      homeScreen(); // Implementar la lógica para mostrar la pantalla HOME
    }
  }
}

void loop()
{
  // Verificar si el usuario ha seleccionado alguna función
  checkSelectedFunction();
  // Aquí se implementaría el resto del código para el funcionamiento del sistema
}

// Función para inicializar el Sensor y Microfono
void initSensors()
{
  // Configuración del pin del micrófono como entrada
  pinMode(MIC_SIGNAL_OUT, INPUT);
  // Inicialización del sensor DHT22
  dht.begin();
}

void setupMQTT()
{
  // Configuración del servidor MQTT
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

// Función para establecer la conexión al servidor MQTT
bool connectMQTT()
{
  // Intentar conectar al broker MQTT
  if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD))
  {
    Serial.println("Conexión con el Broker MQTT exitosa.");
    return true;
  }
  return false;
}

// Función para verificar errores de lectura
bool checkReadErrors()
{
  // Lectura de la temperatura del sensor DHT22
  float temperatura = dht.readTemperature();
  // Lectura de la humedad del sensor DHT22
  float humedad = dht.readHumidity();
  // Lectura del valor del micrófono
  int micValue = analogRead(MIC_SIGNAL_OUT);

  // Verificación de si alguna lectura falló
  if (isnan(temperatura) || isnan(humedad) || micValue < 0)
  {
    return true;
  }
  return false;
}

// Función para verificar la conexión Wi-Fi
bool checkWiFiConnection()
{
  // Inicio de la conexión Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  // Esperar a que se establezca la conexión, con un máximo de 10 intentos
  while (WiFi.status() != WL_CONNECTED && attempts < 10)
  {
    delay(1000);
    attempts++;
  }
  // Retornar verdadero si la conexión fue exitosa
  return WiFi.status() == WL_CONNECTED;
}

// Función para verificar tareas pendientes
bool checkPendingTasks()
{
  // Aquí se leerán y verificarán los datos remotos utilizando MQTT
  // Esta función deberá retornar true si hay tareas pendientes, false en caso contrario
  return false; // Ejemplo: sin tareas pendientes
}

// Función para verificar si el usuario ha seleccionado alguna función
void checkSelectedFunction()
{
  // Verificación de botones de función
  if (digitalRead(OA_F_1) == HIGH)
  {
    // Si el usuario presionó OA_F_1 (F1), implementar acción
    // Aquí se debe implementar la lógica correspondiente
  }
  else if (digitalRead(OA_F_2) == HIGH)
  {
    // Si el usuario presionó OA_F_2 (F2), implementar acción
    // Aquí deberías implementar la lógica correspondiente
  }
  else if (digitalRead(OA_F_3) == HIGH)
  {
    // Si el usuario presionó OA_F_3 (F3), implementar acción
    // Aquí deberías implementar la lógica correspondiente
  }
  else if (digitalRead(OA_F_4) == HIGH)
  {
    // Si el usuario presionó OA_F_4 (F4), implementar acción
    // Aquí deberías implementar la lógica correspondiente
  }
}

// Función para cargar la pantalla principal (HOME)
void homeScreen()
{
  // Implementar aquí la lógica para cargar y mostrar la pantalla HOME en la TFT

  // Cargar Barra de Estados del Equipo
  loadEquipmentStatusbar();

  // Cargar Indicadores en la Barra de Estados
  loadStatusIndicators();

  // Cargar Iconos de Funciones en la Parte Inferior de la Pantalla
  loadFunctionIcons();

  // Cargar Información de la Sección HOME en la Pantalla TFT LCD SPI ILI9488
  loadHomeSection();
}

void cicloMonitoreo()
{
  tiempoInicioTarea = millis();

  while (millis() - tiempoInicioTarea < DURACION_TAREA)
  {
    unsigned long tiempoActual = millis();

    // Proceso de lectura de ruido
    if (tiempoActual - tiempoRuidoAnterior >= 5000)
    {
      ruidos[indiceRuido] = leerRuido();
      indiceRuido = (indiceRuido + 1) % 12; // Circular buffer
      tiempoRuidoAnterior = tiempoActual;

      // Mostrar valores actualizados en pantalla
      mostrarEnPantalla(promedioTemperatura, promedioHumedad, ruidos[indiceRuido]);
    }

    // Proceso de lectura de temperatura y humedad
    if (tiempoActual - tiempoTempHumAnterior >= 60000)
    {
      temperaturas[indiceTempHum] = leerTemperatura();
      humedades[indiceTempHum] = leerHumedad();
      indiceTempHum = (indiceTempHum + 1) % 10; // Circular buffer

      promedioTemperatura = calcularPromedio(temperaturas, 10);
      promedioHumedad = calcularPromedio(humedades, 10);

      tiempoTempHumAnterior = tiempoActual;

      // Guardar valores anteriores para comparación
      temperaturaAnterior = promedioTemperatura;
      humedadAnterior = promedioHumedad;

      // Mostrar valores actualizados en pantalla
      mostrarEnPantalla(promedioTemperatura, promedioHumedad, ruidos[indiceRuido]);
    }

    // Verificar cambio de temperatura
    if (abs(promedioTemperatura - temperaturaAnterior) > 0.5)
    {
      ajustarGananciaMicrofono(promedioTemperatura, promedioHumedad);
      ajustarSensibilidadMicrofono(promedioTemperatura, promedioHumedad);
    }

    // Verificar estado de conexión y publicar datos acumulados
    if (modoSinConexion)
    {
      almacenarDatos(temperaturas, humedades, ruidos);
    }
    else
    {
      if (tiempoActual - tiempoUltimaPublicacion >= INTERVALO_PUBLICACION)
      {
        publicarDatosMQTT(temperaturas, humedades, ruidos);
        tiempoUltimaPublicacion = tiempoActual;
      }
    }
  }

  // Tarea completada
  if (modoSinConexion)
  {
    mostrarMensajeTareaCompleta(true);
  }
  else
  {
    mostrarMensajeTareaCompleta(false);
  }
}

void ajustarGananciaMicrofono(float promedioTemperatura, float promedioHumedad)
{
  // Código para ajustar la resistencia digital X9C103
}

void ajustarSensibilidadMicrofono(float promedioTemperatura, float promedioHumedad)
{
  // Código para ajustar la resistencia digital X9C503
}