/*"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""|
|                Equipo Electrónico Microcontrolado para la Medición del Ruido Ambiental de Fuentes Fijas                |
|                                                                                                                        |
|   Autor: Hengers Emmanuel Rosario Morales                                                                              |
|   Matricula: 2-12-1786                                                                                                 |
|   Fecha: 11-07-2024                                                                                                    |
|                                                                                                                        |
|   Descripción: Equipo Electrónico Microcontrolado para la Medición del Ruido Ambiental,                                |
|                para el Monitoreo y Medición de Ruido de Fuentes Fijas con Visualización Local                          |
|                de Ruido de Fuentes Fijas con Visualización Local y Remota, Utilizando el Protocolo MQTT y Gestionado   |
|                por aplicación Móvil de React Native.                                                                   |
|                                                                                                                        |
|   Versión: 1.0                                                                                                         |
|_______________________________________________________________________________________________________________________*/

/*-----------------------------------------------------------------------------------------------------------------------/
/                                                Librerias Utilizadas                                                    /
/-----------------------------------------------------------------------------------------------------------------------*/

#include <Arduino.h> // Librería principal de Arduino, necesaria para todas las aplicaciones de Arduino.
#include <Wire.h>    // Comunicación I2C entre el microcontrolador y dispositivos periféricos.

// Pantalla ILI9488
#include <SPI.h>          // Comunicación SPI, esencial para la transferencia de datos con la pantalla.
#include <Adafruit_GFX.h> // Funciones gráficas básicas para dibujar formas, texto, etc.
#include <ILI9488.h>      // Controlador específico para la pantalla ILI9488.

// Teclado Matricial 3x4
#include <Keypad_I2C.h>          // Manejo de teclados matriciales conectados a través de I2C.
#include <PCF8574_HD44780_I2C.h> // Maneja el expansor de puertos PCF8574 para el teclado matricial.

// DHT22
#include <DHT.h> // Controla el sensor de temperatura y humedad DHT22.

// WIFI
#include <WiFi.h>             // Conexión a redes WiFi.
#include <WiFiClientSecure.h> // Comunicación segura a través de WiFi.
#include <PubSubClient.h>     // Cliente para el protocolo MQTT, utilizado para la comunicación con servidores MQTT.

// Memoria EEPROM
#include <EEPROM.h> // Manejo de la memoria EEPROM para almacenamiento de datos no volátil.

/*-----------------------------------------------------------------------------------------------------------------------/
/                                        Definicion y Configuración de las GPIO                                          /
/-----------------------------------------------------------------------------------------------------------------------*/

// Definiciones de pines para la pantalla TFT
#define TFT_RESET 16 // Pin de reset de la pantalla TFT.
#define TFT_CS 12    // Pin de selección de chip (CS) de la pantalla TFT.
#define TFT_DC 14    // Pin de datos/comando (DC) de la pantalla TFT.
#define TFT_MOSI 15  // Pin de datos SPI MOSI para la pantalla TFT.
#define TFT_SCK 18   // Pin de reloj SPI SCK para la pantalla TFT.

// Definiciones de pines para el teclado matricial I2C
#define KEYPAD_SDA 21 // Pin SDA para el teclado matricial I2C.
#define KEYPAD_SCL 22 // Pin SCL para el teclado matricial I2C.

// Definiciones de pines para optoacopladores
#define OA_F_1 32 // Pin de control del primer optoacoplador.
#define OA_F_2 33 // Pin de control del segundo optoacoplador.
#define OA_F_3 34 // Pin de control del tercer optoacoplador.
#define OA_F_4 35 // Pin de control del cuarto optoacoplador.

// Definiciones de pines para el preamplificador
#define PREAMP_U_D 25 // Pin de control U/D del preamplificador.
#define PREAMP_INC 26 // Pin de incremento del preamplificador.
#define PREAMP_CS 27  // Pin de selección de chip (CS) del preamplificador.

// Definiciones de pines para el amplificador
#define AMP_U_D 21 // Pin de control U/D del amplificador.
#define AMP_INC 22 // Pin de incremento del amplificador.
#define AMP_CS 23  // Pin de selección de chip (CS) del amplificador.

// Definición de pin para el sensor de temperatura y humedad
#define DATA_SENSOR_TEMP_AND_HR 0 // Pin de datos para el sensor DHT22.

// Definición de pin para la salida de señal del micrófono
#define MIC_SIGNAL_OUT 2 // Pin de salida de señal del micrófono.

// Definición de pin para el buzzer
#define BUZZER_PIN 5 // Pin de control del buzzer.

/*-----------------------------------------------------------------------------------------------------------------------/
-                                Definiciones de las credenciales para WIFI y el SERVER                                  -
/-----------------------------------------------------------------------------------------------------------------------*/

// Credenciales WiFi
#define WIFI_SSID "SSID"         // Nombre de la red WiFi.
#define WIFI_PASSWORD "PASSWORD" // Contraseña de la red WiFi.

// Credenciales del servidor MQTT
#define MQTT_SERVER "mqtt_server"     // Dirección del servidor MQTT.
#define MQTT_PORT 0000                // Puerto del servidor MQTT.
#define MQTT_USER "mqtt_user"         // Nombre de usuario para el servidor MQTT.
#define MQTT_PASSWORD "mqtt_password" // Contraseña para el servidor MQTT.

// Configuración del cliente MQTT
WiFiClientSecure espClient;     // Cliente WiFi seguro.
PubSubClient client(espClient); // Cliente MQTT utilizando el cliente WiFi seguro.

// Definiciones de direcciones de memoria en EEPROM
#define EEPROM_SIZE 512        // Tamaño total de la EEPROM.
#define TEMP_START_ADDR 0      // Dirección inicial para almacenar datos de temperatura.
#define HUM_START_ADDR 40      // Dirección inicial para almacenar datos de humedad.
#define NOISE_START_ADDR 80    // Dirección inicial para almacenar datos de ruido.
#define DATA_SIZE 40           // Tamaño de cada conjunto de datos almacenados.
#define TASK_START_ADDR 120    // Dirección inicial para almacenar datos de tareas.
#define TASK_SIZE sizeof(Task) // Tamaño de la estructura de datos Task.
#define CONFI_START_ADDR 160   // Dirección inicial para almacenar configuraciones

// Definiciones de dirección de memoria del teclado matricial 3x4
#define PCF8574_ADDRESS 0x20 // Dirección I2C del expansor de puertos PCF8574.

// Configuración del sensor DHT22
#define DHTTYPE DHT22                      // Tipo de sensor DHT (DHT22).
DHT dht(DATA_SENSOR_TEMP_AND_HR, DHTTYPE); // Inicialización del sensor DHT22 con el pin de datos.

// Inicialización de la pantalla TFT
Adafruit_ILI9488 tft = Adafruit_ILI9488(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RESET); // Inicializa la pantalla ILI9488 con los pines correspondientes.

// Configuración del teclado matricial
const byte ROWS = 4;      // Cuatro filas.
const byte COLS = 3;      // Tres columnas.
char keys[ROWS][COLS] = { // Mapa de teclas del teclado matricial.
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};
byte rowPins[ROWS] = {0, 1, 2, 3}; // Pines de las filas conectados al PCF8574.
byte colPins[COLS] = {4, 5, 6};    // Pines de las columnas conectados al PCF8574.

Keypad_I2C kpd(makeKeymap(keys), rowPins, colPins, ROWS, COLS, KEYPAD_SDA, KEYPAD_SCL, PCF8574_ADDRESS); // Inicializa el teclado matricial I2C con el mapa de teclas y pines correspondientes.

// Variables para manejar las banderas de interrupción
volatile bool leerTempHumFlag = false;   // Bandera para indicar si se debe leer temperatura y humedad.
volatile bool leerMicrofonoFlag = false; // Bandera para indicar si se debe leer el micrófono.

// Variables globales
volatile float temperatura = 0.0; // Variable para almacenar la temperatura actual.
volatile float humedad = 0.0;     // Variable para almacenar la humedad actual.
volatile int micValue = 0;        // Variable para almacenar el valor de la señal del micrófono.

volatile unsigned long tiempoRuidoAnterior = 0;   // Tiempo de la última lectura de ruido.
volatile unsigned long tiempoTempHumAnterior = 0; // Tiempo de la última lectura de temperatura y humedad.

volatile int ruidos[12] = {0};           // Arreglo para almacenar las lecturas de ruido.
volatile float temperaturas[10] = {0.0}; // Arreglo para almacenar las lecturas de temperatura.
volatile float humedades[10] = {0.0};    // Arreglo para almacenar las lecturas de humedad.

volatile int indiceRuido = 0;   // Índice actual para almacenar la siguiente lectura de ruido.
volatile int indiceTempHum = 0; // Índice actual para almacenar la siguiente lectura de temperatura y humedad.

float promedioTemperatura = 0.0; // Promedio de las lecturas de temperatura.
float promedioHumedad = 0.0;     // Promedio de las lecturas de humedad.

float temperaturaAnterior = 0.0; // Última temperatura leída.
float humedadAnterior = 0.0;     // Última humedad leída.

bool modoSinConexion = false; // Modo sin conexión (indica si no hay conexión a internet).

unsigned long tiempoInicioTarea = 0;   // Tiempo de inicio de la tarea actual.
const unsigned long duracionTarea = 0; // Duración de la tarea actual.

unsigned long tiempoUltimaPublicacion = 0;    // Tiempo de la última publicación de datos.
const unsigned long intervaloPublicacion = 0; // Intervalo de tiempo entre publicaciones.

int tareaSeleccionada = -1; // Tarea seleccionada actualmente.
int cursorPos = 0;          // Posición del cursor en la interfaz de usuario.

// Definición de la estructura de una tarea
struct Task
{
  char areaClass;     // Clasificación del área.
  int maxNoise;       // Rango máximo de ruido en dB.
  int minNoise;       // Rango mínimo de ruido en dB.
  int duration;       // Duración de la tarea en minutos.
  bool notifications; // Permitir o no notificaciones.
};

// Estructura para almacenar la configuración
struct Config
{
  bool wifiConfigured;
  bool serverConfigured;
  bool notificationsConfigured;
};

/*-----------------------------------------------------------------------------------------------------------------------/
/                                                Funciones de Inicialización                                             /
/-----------------------------------------------------------------------------------------------------------------------*/

// Inicializa el bus SPI
void initializeSPI()
{
  SPI.begin(); // Inicializa el bus SPI.
}

// Inicializa el bus I2C
void initializeI2C()
{
  Wire.begin(); // Inicializa el bus I2C.
}

// Inicializa el teclado matricial I2C
void initializeKeypad()
{
  kpd.begin(); // Inicializa el teclado matricial I2C.
}

// Inicializa la memoria EEPROM
void initializeEEPROM()
{
  EEPROM.begin(EEPROM_SIZE); // Inicializa la EEPROM con el tamaño definido.
}

// Función para inicializar el Sensor y Micrófono
void initializeMicDht()
{
  // Configuración del pin del micrófono como entrada
  pinMode(MIC_SIGNAL_OUT, INPUT); // Configura el pin del micrófono como entrada.
  // Inicialización del sensor DHT22
  dht.begin(); // Inicializa el sensor DHT22.
}

// Inicializa la pantalla TFT
void initializeTFT()
{
  tft.begin();                     // Inicializa la pantalla TFT.
  tft.setRotation(3);              // Establece la rotación de la pantalla.
  tft.fillScreen(ILI9488_BLACK);   // Rellena la pantalla con color negro.
  tft.setTextColor(ILI9488_WHITE); // Establece el color del texto a blanco.
  tft.setTextSize(2);              // Establece el tamaño del texto.
  tft.setCursor(10, 10);           // Establece la posición del cursor.
  tft.println("Inicializando..."); // Muestra "Inicializando..." en la pantalla.
}

// Configura los pines GPIO
void initializeGPIO()
{
  pinMode(OA_F_1, INPUT_PULLUP); // Configura el pin OA_F_1 como entrada con resistencia pull-up.
  pinMode(OA_F_2, INPUT_PULLUP); // Configura el pin OA_F_2 como entrada con resistencia pull-up.
  pinMode(OA_F_3, INPUT_PULLUP); // Configura el pin OA_F_3 como entrada con resistencia pull-up.
  pinMode(OA_F_4, INPUT_PULLUP); // Configura el pin OA_F_4 como entrada con resistencia pull-up.

  pinMode(PREAMP_U_D, OUTPUT); // Configura el pin PREAMP_U_D como salida.
  pinMode(PREAMP_INC, OUTPUT); // Configura el pin PREAMP_INC como salida.
  pinMode(PREAMP_CS, OUTPUT);  // Configura el pin PREAMP_CS como salida.

  pinMode(AMP_U_D, OUTPUT); // Configura el pin AMP_U_D como salida.
  pinMode(AMP_INC, OUTPUT); // Configura el pin AMP_INC como salida.
  pinMode(AMP_CS, OUTPUT);  // Configura el pin AMP_CS como salida.

  pinMode(DATA_SENSOR_TEMP_AND_HR, INPUT); // Configura el pin del sensor de temperatura y humedad como entrada.
  pinMode(MIC_SIGNAL_OUT, INPUT);          // Configura el pin de salida de señal del micrófono como entrada.

  pinMode(BUZZER_PIN, OUTPUT); // Configura el pin del buzzer como salida.
}

// Configura las interrupciones
void initializeInterrupts()
{
  attachInterrupt(digitalPinToInterrupt(DATA_SENSOR_TEMP_AND_HR), leerTempHumInterrupcion, CHANGE); // Configura interrupción para cambios en el pin del sensor de temperatura y humedad.
  attachInterrupt(digitalPinToInterrupt(MIC_SIGNAL_OUT), leerMicrofonoInterrupcion, CHANGE);        // Configura interrupción para cambios en el pin de salida del micrófono.
}

// Configura el cliente MQTT
void setupMQTT()
{
  client.setServer(MQTT_SERVER, MQTT_PORT); // Establece el servidor MQTT y su puerto.
  client.setCallback(callback);             // Establece la función de callback para el cliente MQTT.
}

/*-----------------------------------------------------------------------------------------------------------------------/
/                                                Funciones Principales                                                   /
/-----------------------------------------------------------------------------------------------------------------------*/

// Función principal de configuración
void setup()
{
  // Inicialización del Serial
  Serial.begin(115200); // Inicializa la comunicación serial a 115200 bps.

  // Inicialización de componentes
  initializeSPI();    // Inicializa el bus SPI.
  initializeI2C();    // Inicializa el bus I2C.
  initializeKeypad(); // Inicializa el teclado matricial I2C.
  initializeEEPROM(); // Inicializa la memoria EEPROM.
  initializeMicDht(); // Inicializa el sensor DHT22 y el micrófono.
  initializeTFT();    // Inicializa la pantalla TFT.
  initializeGPIO();   // Configura los pines GPIO.

  initializeInterrupts(); // Configura las interrupciones.

  setupTimer(); // Configura el temporizador para las interrupciones.
  setupMQTT();  // Configura el cliente MQTT.

  // Verificación de errores de lectura inicial
  if (checkReadErrors())
  {
    handleError(); // Maneja los errores de lectura inicial.
  }

  // Verificación de conexión Wi-Fi
  if (!checkWiFiConnection())
  {
    handleNoWiFiConnection(); // Maneja la falta de conexión Wi-Fi.
  }

  // Intentar conectar al servidor remoto
  if (!connectMQTT())
  {
    handleMQTTConnectionError(); // Maneja los errores de conexión MQTT.
  }

  // Verificar si hay tareas pendientes
  if (checkPendingTasks())
  {
    handlePendingTasks(); // Maneja las tareas pendientes.
  }
  else
  {
    homeScreen(); // Implementa la lógica para mostrar la pantalla HOME.
  }
}

void loop()
{
  // Verificar si el usuario ha seleccionado alguna función
  checkSelectedFunction();

  // Verificar la conexión WiFi y MQTT
  if (!checkWiFiConnection())
  {
    // Reintentar la conexión
    reconnect();

    // Gestionar los eventos MQTT si hay una conexión activa
    if (client.connected())
    {
      client.loop();
    }
  }

  // Otras tareas del bucle principal
  // Aquí se implementaría el resto del código para el funcionamiento del sistema
}

/*-----------------------------------------------------------------------------------------------------------------------/
/                                           Funciones para Manejo de Errores                                             /
/-----------------------------------------------------------------------------------------------------------------------*/

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
    // Si alguna lectura no es un número (falló) o el valor del micrófono es negativo, retorna verdadero (error).
    return true;
  }
  // Si todas las lecturas fueron exitosas, retorna falso (sin errores).
  return false;
}

// Función para manejar errores
void handleError()
{
  // Imprime un mensaje de error en el monitor serial
  Serial.println("¡ERROR!");

  // Limpia la pantalla del TFT y muestra un mensaje de inicialización
  tft.fillScreen(ILI9488_BLACK);
  tft.setCursor(10, 10);
  tft.println("Inicializando...");

  // Bucle infinito para manejar el estado de error
  while (true)
  {
    // Enciende el buzzer para alertar del error
    digitalWrite(BUZZER_PIN, HIGH);
    delay(1000); // Espera 1 segundo

    // Apaga el buzzer
    digitalWrite(BUZZER_PIN, LOW);
    delay(1000); // Espera 1 segundo

    // Verifica si se ha presionado la tecla F1 (para reiniciar)
    if (digitalRead(OA_F_1) == HIGH)
    {
      // Imprime un mensaje de reinicio en el monitor serial
      Serial.println("Reiniciando dispositivo por tecla F1...");
      // Limpia la pantalla del TFT y muestra un mensaje de reinicio
      tft.fillScreen(ILI9488_BLACK);
      tft.setCursor(10, 10);
      tft.println("Reiniciando dispositivo...");

      delay(3000); // Espera 3 segundos antes de reiniciar

      // Reinicia el dispositivo
      ESP.restart();
    }
    // Verifica si se ha presionado la tecla F2 (para omitir el error)
    else if (digitalRead(OA_F_2) == HIGH)
    {
      // Imprime un mensaje de omisión en el monitor serial
      Serial.println("Omitiendo error y continuando por tecla F2...");

      // Limpia la pantalla del TFT y muestra un mensaje de omisión
      tft.fillScreen(ILI9488_BLACK);
      tft.setCursor(10, 10);
      tft.println("Omitiendo error...");

      delay(1000); // Espera 1 segundo

      // Muestra opciones de omisión en el TFT
      tft.setCursor(10, 30);
      tft.println("Desea Omitir el Error y Continuar?");
      tft.setCursor(10, 50);
      tft.println("F1: OK");
      tft.setCursor(10, 70);
      tft.println("F2: CANCEL");

      // Bucle infinito para manejar las opciones de omisión
      while (true)
      {
        // Verifica si se ha presionado la tecla F1 para continuar
        if (digitalRead(OA_F_1) == HIGH)
        {
          // Imprime un mensaje de continuación en el monitor serial
          Serial.println("Omitiendo error y continuando...");

          // Limpia la pantalla del TFT y muestra un mensaje de continuación
          tft.fillScreen(ILI9488_BLACK);
          tft.setCursor(10, 10);
          tft.println("Continuando...");

          delay(1000); // Espera 1 segundo
          break;       // Salir del bucle y continuar con la ejecución normal
        }
        // Verifica si se ha presionado la tecla F2 para cancelar
        else if (digitalRead(OA_F_2) == HIGH)
        {
          // Imprime un mensaje de cancelación en el monitor serial
          Serial.println("Cancelando operación...");

          // Limpia la pantalla del TFT y muestra un mensaje de cancelación
          tft.fillScreen(ILI9488_BLACK);
          tft.setCursor(10, 10);
          tft.println("Cancelando operación...");

          delay(1000); // Espera 1 segundo

          // Llama nuevamente a la función handleError para reiniciar el proceso
          handleError();
        }
      }
      break; // Salir del bucle de error y continuar
    }
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                                       Verificación de Conexión Wi-Fi                                               /
/------------------------------------------------------------------------------------------------------------------*/

// Función para verificar la conexión Wi-Fi
bool checkWiFiConnection()
{
  // Inicio de la conexión Wi-Fi utilizando las credenciales definidas
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int attempts = 0;
  // Espera hasta que se establezca la conexión, con un máximo de 10 intentos
  while (WiFi.status() != WL_CONNECTED && attempts < 10)
  {
    // Espera 1 segundo entre intentos
    delay(1000);
    // Incrementa el contador de intentos
    attempts++;
  }
  // Retorna verdadero si la conexión fue exitosa, falso en caso contrario
  return WiFi.status() == WL_CONNECTED;
}

// Función para manejar la falta de conexión Wi-Fi
void handleNoWiFiConnection()
{
  // Imprime un mensaje solicitando la acción del usuario
  Serial.println("No se pudo conectar al Wi-Fi. ¿Desea continuar en modo sin conexión? (F1: Sí, F2: No)");
  // Entra en un bucle infinito a la espera de la respuesta del usuario
  while (true)
  {
    // Si el usuario presiona el botón F1 (sí)
    if (digitalRead(OA_F_1) == HIGH)
    {
      // Imprime un mensaje en el monitor serial
      Serial.println("Continuando en modo sin conexión...");
      // Activa el modo sin conexión
      modoSinConexion = true;
      // Muestra la pantalla de inicio
      homeScreen();
      // Sale del bucle
      break;
    }
    // Si el usuario presiona el botón F2 (no)
    else if (digitalRead(OA_F_2) == HIGH)
    {
      // Imprime un mensaje en el monitor serial
      Serial.println("Dirigiendo a configuración...");
      // Implementar función de configuración si es necesario
      break;
    }
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                                         Establece Conexión con el Server                                          /
/------------------------------------------------------------------------------------------------------------------*/

// Función para establecer la conexión al servidor MQTT
bool connectMQTT()
{
  // Intentar conectar al broker MQTT
  if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD))
  {
    // Imprimir mensaje en el monitor serial indicando éxito en la conexión
    Serial.println("Conexión con el Broker MQTT exitosa.");
    return true; // Retorna true si la conexión es exitosa
  }
  // Retorna false si la conexión falla
  return false;
}

// Función para manejar errores de conexión MQTT
void handleMQTTConnectionError()
{
  // Llama a una función de manejo de errores
  handleError();
}

/*------------------------------------------------------------------------------------------------------------------/
/                                              Reconexión (WIFI) - (MQTT)                                           /
/------------------------------------------------------------------------------------------------------------------*/

// Función para reconectar al WiFi y MQTT si se pierde la conexión
void reconnect()
{
  // Bucle para reconectar Wi-Fi
  while (WiFi.status() != WL_CONNECTED)
  {
    // Imprime el SSID al cual se está intentando conectar
    Serial.print("Intentando conectar a SSID: ");
    Serial.println(WIFI_SSID);
    // Inicia la conexión Wi-Fi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(5000); // Espera 5 segundos antes de intentar nuevamente
  }

  // Bucle para reconectar MQTT
  while (!client.connected())
  {
    Serial.println("Intentando conexión MQTT...");
    // Intentar conectar al broker MQTT
    if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD))
    {
      // Imprime mensaje en el monitor serial indicando éxito en la conexión
      Serial.println("Conectado al servidor MQTT.");
      // Suscribirse a un tópico MQTT
      client.subscribe("esp32/output");
    }
    else
    {
      // Imprime el código de error y mensaje de reintento
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos.");
      delay(5000); // Espera 5 segundos antes de intentar nuevamente
    }
  }
}

// Callback para manejar mensajes MQTT recibidos
void callback(char *topic, byte *payload, unsigned int length)
{
  // Lógica para manejar mensajes recibidos a través de MQTT
  // Aquí se puede agregar el procesamiento de mensajes según el tópico y el payload
}

/*------------------------------------------------------------------------------------------------------------------/
/                                        Verificación de Tareas Pendientes                                          /
/------------------------------------------------------------------------------------------------------------------*/

// Función para verificar tareas pendientes
bool checkPendingTasks()
{
  // Aquí se leerán y verificarán los datos remotos utilizando MQTT
  // Esta función deberá retornar true si hay tareas pendientes, false en caso contrario
  return false; // Ejemplo: sin tareas pendientes
}

// Función para manejar tareas pendientes
void handlePendingTasks()
{
  // Imprime mensaje en el monitor serial indicando que se encontraron tareas pendientes
  Serial.println("Tareas pendientes encontradas.");
  Serial.println("¿Desea continuar con la tarea? (F1: Continuar, F2: Cancelar)");

  // Bucle infinito para esperar la decisión del usuario
  while (true)
  {
    // Verifica si la tecla F1 ha sido presionada para continuar con la tarea
    if (digitalRead(OA_F_1) == HIGH)
    {
      // Imprime mensaje en el monitor serial indicando que se está cargando la información de la tarea
      Serial.println("Cargando información de la tarea...");
      // Llama a la función para monitorear la tarea
      taskMonitor();
      break; // Sale del bucle
    }
    // Verifica si la tecla F2 ha sido presionada para cancelar la tarea
    else if (digitalRead(OA_F_2) == HIGH)
    {
      // Imprime mensaje en el monitor serial indicando que se está cancelando la tarea y dirigiendo a la pantalla principal
      Serial.println("Cancelando tarea. Dirigiendo a HOME...");
      // Llama a la función para mostrar la pantalla principal
      homeScreen();
      break; // Sale del bucle
    }
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                         Función para esperar y manejar los Eventos del Teclado Matricial                          /
/------------------------------------------------------------------------------------------------------------------*/

// Función para manejar eventos de teclas validas
void waitForValidKeypadEvent()
{
  char key;
  do
  {
    key = kpd.getKey();
  } while (key == NO_KEY);
  return key;
}

/*------------------------------------------------------------------------------------------------------------------/
/                             Función de Verificación de teclas de funciones Principales                            /
/------------------------------------------------------------------------------------------------------------------*/

// Función para verificar si el usuario ha seleccionado alguna función
void checkSelectedFunction()
{
  // Verificación de botones de función
  if (digitalRead(OA_F_1) == HIGH)
  {
    // Si el usuario presionó OA_F_1 (F1), implementar acción de ir a Pantalla Principal (HOME)
    // Aquí se debe implementar la lógica correspondiente a esta tecla de Función
    homeScreen();
  }
  else if (digitalRead(OA_F_2) == HIGH)
  {
    // Si el usuario presionó OA_F_2 (F2), implementar acción de ir a la Sección de Tareas
    // Aquí se debe implementar la lógica correspondiente a esta tecla de Función
    showTaskMenu();
  }
  else if (digitalRead(OA_F_3) == HIGH)
  {
    // Si el usuario presionó OA_F_3 (F3), implementar acción de ir a la Pantalla Notificaciones
    // Aquí se debe implementar la lógica correspondiente a esta tecla de Función
    handleNotifications();
  }
  else if (digitalRead(OA_F_4) == HIGH)
  {
    // Si el usuario presionó OA_F_4 (F4), implementar acción de ir a la Sección de Configuración
    // Aquí se debe implementar la lógica correspondiente a esta tecla de Función
    handleConfiguracion();
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                                            HOME (Pantalla Principal)                                              /
/------------------------------------------------------------------------------------------------------------------*/

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

  // Codigo para esperar la Función Oprimida
}

/*------------------------------------------------------------------------------------------------------------------/
/                                                     Menú de Tareas                                                /
/------------------------------------------------------------------------------------------------------------------*/

// Función para mostrar el menú principal de tareas
void showTaskMenu()
{
  // Llena la pantalla del TFT con color negro
  tft.fillScreen(ILI9488_BLACK);

  // Establece el cursor en la posición (10, 10)
  tft.setCursor(10, 10);

  // Establece el color del texto en blanco
  tft.setTextColor(ILI9488_WHITE);

  // Imprime el título "Lista de Tareas" en la pantalla
  tft.print("Lista de Tareas");

  int taskIndex = 0; // Índice de la tarea seleccionada inicialmente

  // Bucle infinito para navegar por el menú de tareas
  while (true)
  {
    // Muestra las tareas disponibles en la pantalla
    displayTasks(taskIndex);

    // Espera una tecla válida del teclado matricial
    char key = waitForValidKeypadEvent();

    // Maneja la tecla presionada
    if (key == '2')
    {
      taskIndex = max(0, taskIndex - 1);
    }
    else if (key == '8')
    {
      taskIndex = min(getTaskCount() - 1, taskIndex + 1);
    }
    else if (key == '#')
    {
      if (taskIndex < getTaskCount())
      {
        executeTask(taskIndex);
      }
      else
      {
        createNewTask();
      }
    }
    delay(200); // Espera 200 ms antes de la siguiente iteración del bucle
  }
}

// Función para mostrar las tareas en pantalla
void displayTasks(int selectedIndex)
{
  // Llena la región de la pantalla donde se muestran las tareas con color negro
  tft.fillRect(0, 40, tft.width(), tft.height() - 40, ILI9488_BLACK);

  int taskCount = getTaskCount(); // Obtiene el número de tareas almacenadas

  // Bucle para mostrar cada tarea
  for (int i = 0; i < taskCount; i++)
  {
    Task task; // Variable para almacenar una tarea temporalmente

    // Lee la tarea desde la EEPROM
    EEPROM.get(TASK_START_ADDR + i * TASK_SIZE, task);

    // Establece el cursor en la posición adecuada para la tarea actual
    tft.setCursor(10, 40 + i * 20);

    // Si la tarea actual es la seleccionada, muestra el texto en color rojo
    if (i == selectedIndex)
    {
      tft.setTextColor(ILI9488_RED);
    }
    // Si no es la tarea seleccionada, muestra el texto en color blanco
    else
    {
      tft.setTextColor(ILI9488_WHITE);
    }

    // Imprime la información de la tarea en la pantalla
    tft.printf("Tarea %d: Area %c, Max %d dB, Min %d dB, Dur %d min", i + 1, task.areaClass, task.maxNoise, task.minNoise, task.duration);
  }
}

// Función para contar la cantidad de tareas almacenadas
int getTaskCount()
{
  int count = 0; // Inicializa el contador de tareas

  // Bucle para recorrer la EEPROM desde la dirección de inicio de tareas hasta el final de la EEPROM
  for (int i = TASK_START_ADDR; i < EEPROM_SIZE; i += TASK_SIZE)
  {
    Task task; // Variable para almacenar una tarea temporalmente

    // Lee la tarea desde la EEPROM
    EEPROM.get(i, task);

    // Si la clasificación de área de la tarea es 0, significa que no hay más tareas
    if (task.areaClass == 0)
    {
      break;
    }
    count++; // Incrementa el contador de tareas
  }
  return count; // Retorna el número total de tareas
}

// Función para ejecutar una tarea
void executeTask(int taskIndex)
{
  Task task; // Variable para almacenar una tarea temporalmente

  // Lee la tarea desde la EEPROM utilizando el índice de la tarea
  EEPROM.get(TASK_START_ADDR + taskIndex * TASK_SIZE, task);

  // Implementa la lógica para ejecutar la tarea (esto se puede personalizar)
  taskMonitor();
}

/*------------------------------------------------------------------------------------------------------------------/
/                                                    Crear Tarea                                                    /
/------------------------------------------------------------------------------------------------------------------*/

// Función para crear una nueva tarea
void createNewTask()
{
  Task newTask; // Variable para almacenar la nueva tarea

  // Obtiene la clasificación de área para la nueva tarea
  newTask.areaClass = getAreaClass();

  // Obtiene el valor máximo de ruido para la nueva tarea
  newTask.maxNoise = getNoiseValue("Maximo");

  // Obtiene el valor mínimo de ruido para la nueva tarea
  newTask.minNoise = getNoiseValue("Minimo");

  // Obtiene la duración para la nueva tarea
  newTask.duration = getDuration();

  // Obtiene la configuración de notificaciones para la nueva tarea
  newTask.notifications = getNotifications();

  // Obtiene el número de tareas almacenadas
  int taskCount = getTaskCount();

  // Guarda la nueva tarea en la EEPROM
  EEPROM.put(TASK_START_ADDR + taskCount * TASK_SIZE, newTask);

  // Realiza el commit para asegurar que los cambios se guarden en la EEPROM
  EEPROM.commit();

  // Llena la pantalla del TFT con color negro
  tft.fillScreen(ILI9488_BLACK);

  // Establece el cursor en la posición (10, 10)
  tft.setCursor(10, 10);

  // Establece el color del texto en verde
  tft.setTextColor(ILI9488_GREEN);

  // Imprime el mensaje "Tarea Creada" en la pantalla
  tft.print("Tarea Creada");

  // Espera 1 segundo antes de continuar
  delay(1000);
}

// Función para obtener la clasificación de área
char getAreaClass()
{
  // Llena la pantalla del TFT con color negro
  tft.fillScreen(ILI9488_BLACK);

  // Establece el cursor en la posición (10, 10)
  tft.setCursor(10, 10);

  // Establece el color del texto en blanco
  tft.setTextColor(ILI9488_WHITE);

  // Imprime las instrucciones para la clasificación de área en la pantalla
  tft.print("Clasificacion de Area: A,B,C,D");

  // Espera una tecla válida del teclado matricial
  return waitForValidKey();
}

// Función para obtener el valor de ruido (máximo o mínimo)
int getNoiseValue(const char *type)
{
  // Llena la pantalla del TFT con color negro
  tft.fillScreen(ILI9488_BLACK);

  // Establece el cursor en la posición (10, 10)
  tft.setCursor(10, 10);

  // Establece el color del texto en blanco
  tft.setTextColor(ILI9488_WHITE);

  // Imprime las instrucciones para el rango de ruido en la pantalla
  tft.printf("Rango de Ruido %s: ", type);

  String noiseValue = ""; // Variable para almacenar el valor de ruido como cadena

  // Bucle infinito para esperar la entrada del usuario
  while (true)
  {
    // Espera una tecla válida del teclado matricial
    char key = waitForValidKeypadEvent();

    // Maneja la tecla presionada
    if (key >= '0' && key <= '9')
    {
      noiseValue += key;
      tft.print(key);
    }
    else if (key == '#')
    {
      return noiseValue.toInt();
    }
    else if (key == '*')
    {
      noiseValue = "";
      tft.fillRect(10, 40, tft.width(), 20, ILI9488_BLACK);
      tft.setCursor(10, 40);
    }
    delay(200); // Espera 200 ms antes de la siguiente iteración del bucle
  }
}

// Función para obtener la duración de la tarea
int getDuration()
{
  // Llena la pantalla del TFT con color negro
  tft.fillScreen(ILI9488_BLACK);

  // Establece el cursor en la posición (10, 10)
  tft.setCursor(10, 10);

  // Establece el color del texto en blanco
  tft.print("Duracion de la Tarea (min): ");

  String durationValue = ""; // Variable para almacenar la duración de la tarea como cadena

  // Bucle infinito para esperar la entrada del usuario
  while (true)
  {
    // Espera una tecla válida del teclado matricial
    char key = waitForValidKeypadEvent();

    // Maneja la tecla presionada
    if (key >= '0' && key <= '9')
    {
      durationValue += key;
      tft.print(key);
    }
    else if (key == '#')
    {
      return durationValue.toInt();
    }
    else if (key == '*')
    {
      durationValue = "";
      tft.fillRect(10, 40, tft.width(), 20, ILI9488_BLACK);
      tft.setCursor(10, 40);
    }
    delay(200); // Espera 200 ms antes de la siguiente iteración del bucle
  }
}

// Función para obtener la configuración de notificaciones
bool getNotifications()
{
  // Llena la pantalla del TFT con color negro
  tft.fillScreen(ILI9488_BLACK);

  // Establece el cursor en la posición (10, 10)
  tft.setCursor(10, 10);

  // Establece el color del texto en blanco
  tft.print("Notificaciones (1=Si, 0=No): ");

  // Bucle infinito para esperar la entrada del usuario
  while (true)
  {
    // Espera una tecla válida del teclado matricial
    char key = waitForValidKeypadEvent();

    // Maneja la tecla presionada
    if (key == '1')
    {
      return true;
    }
    else if (key == '0')
    {
      return false;
    }
    delay(200); // Espera 200 ms antes de la siguiente iteración del bucle
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                                               Monitor de Tarea                                                    /
/------------------------------------------------------------------------------------------------------------------*/

void taskMonitor()
{
  tiempoInicioTarea = millis(); // Marca el tiempo de inicio de la tarea

  while (millis() - tiempoInicioTarea < duracionTarea)
  {
    unsigned long tiempoActual = millis(); // Tiempo actual del sistema

    // Proceso de lectura de ruido manejado por interrupciones
    if (leerMicrofonoFlag)
    {
      noInterrupts();            // Desactiva las interrupciones temporalmente
      leerMicrofonoFlag = false; // Reinicia el flag de lectura de micrófono
      interrupts();              // Activa las interrupciones nuevamente

      ruidos[indiceRuido] = leerRuido();    // Lee el valor actual del ruido y lo almacena en un buffer circular
      indiceRuido = (indiceRuido + 1) % 12; // Este es un Buffer circular para almacenar el indice del Ruido capturado.
      /* Circular buffer: es una estructura de datos que utiliza una sola cola de tamaño fijo como si fuera conectada de
         extremo a extremo, lo cual es muy útil para almacenar datos en tiempo real sin necesidad de desplazar elementos
         en la memoria.    */

      // Muestra los valores actualizados en la pantalla
      showOnScreen(promedioTemperatura, promedioHumedad, ruidos[indiceRuido]);
    }

    // Proceso de lectura de temperatura y humedad manejado por interrupciones
    if (leerTempHumFlag)
    {
      noInterrupts();          // Desactiva las interrupciones temporalmente
      leerTempHumFlag = false; // Reinicia el flag de lectura de temperatura y humedad
      interrupts();            // Activa las interrupciones nuevamente

      temperaturas[indiceTempHum] = leerTemperatura(); // Lee la temperatura actual y la almacena en un buffer circular
      humedades[indiceTempHum] = leerHumedad();        // Lee la humedad actual y la almacena en un buffer circular
      indiceTempHum = (indiceTempHum + 1) % 10;        // Este es un Buffer circular para almacenar el indice de la Temperatura y Humedad capturada.

      // Calcula el promedio de temperatura y humedad
      promedioTemperatura = calcularPromedio(temperaturas, 10);
      promedioHumedad = calcularPromedio(humedades, 10);

      // Guarda los valores anteriores para comparación
      temperaturaAnterior = promedioTemperatura;
      humedadAnterior = promedioHumedad;

      // Muestra los valores actualizados en la pantalla
      showOnScreen(promedioTemperatura, promedioHumedad, ruidos[indiceRuido]);
    }

    // Ajusta la ganancia y sensibilidad según cambios en la temperatura promedio
    if (abs(promedioTemperatura - temperaturaAnterior) > 0.5)
    {
      ajustarGanancia(promedioTemperatura, promedioHumedad);
      ajustarSensibilidad(promedioTemperatura, promedioHumedad);
    }

    // Verifica el estado de conexión y gestiona la publicación de datos
    if (modoSinConexion)
    {
      almacenarDatos(temperaturas, humedades, ruidos); // Almacena localmente si no hay conexión
    }
    else
    {
      if (millis() - tiempoUltimaPublicacion >= intervaloPublicacion)
      {
        publicarDatosMQTT(temperaturas, humedades, ruidos); // Publica los datos si hay conexión MQTT
        tiempoUltimaPublicacion = millis();
      }
    }
  }

  /*------------------------------------------------------------------------------------------------------------------/
  /                               Obtener el Promedio para la Temperatura y Humedad                                   /
  /------------------------------------------------------------------------------------------------------------------*/

  float calcularPromedio(float datos[], int cantMed)
  {
    float suma = 0.0; // Inicializa la variable suma para acumular los valores del arreglo

    // Itera sobre el arreglo para sumar todos los elementos
    for (int i = 0; i < cantMed; i++)
    {
      suma += datos[i]; // Agrega cada elemento del arreglo a la suma total
    }

    return suma / cantMed; // Calcula y devuelve el promedio dividiendo la suma por la cantidad de elementos
  }

  /*------------------------------------------------------------------------------------------------------------------/
  /                                    Calibrarción del Sistema de Captura de Ruido                                   /
  /------------------------------------------------------------------------------------------------------------------*/

  // Función para ajustar la sensibilidad del micrófono en función de la temperatura y la humedad promedio
  void ajustarSensibilidad(float promedioTemperatura, float promedioHumedad)
  {
    // Inicia la comunicación con la resistencia digital X9C503 para ajustar la sensibilidad del micrófono
    digitalWrite(PREAMP_CS, LOW); // Selecciona la resistencia X9C503 (activa el chip select)
    delay(1);                     // Pequeña pausa para asegurar la comunicación

    // Determina si se debe incrementar o disminuir la sensibilidad del micrófono
    // Basado en la humedad promedio
    if (promedioHumedad > 80.0)
    {
      // Incrementa la sensibilidad del micrófono si la humedad promedio es mayor a 80%
      digitalWrite(PREAMP_U_D, HIGH);
    }
    else
    {
      // Decrementa la sensibilidad del micrófono si la humedad promedio es 80% o menor
      digitalWrite(PREAMP_U_D, LOW);
    }
    delay(1); // Pequeña pausa para asegurar la configuración

    // Genera pulsos para incrementar o decrementar la sensibilidad
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(PREAMP_INC, LOW); // Pulso bajo
      delay(1);
      digitalWrite(PREAMP_INC, HIGH); // Pulso alto
      delay(1);
    }

    // Termina la comunicación con la resistencia digital X9C503
    digitalWrite(PREAMP_CS, HIGH); // Desactiva el chip select
  }

  // Función para ajustar la ganancia del micrófono en función de la temperatura y la humedad promedio
  void ajustarGanancia(float promedioTemperatura, float promedioHumedad)
  {
    // Inicia la comunicación con la resistencia digital X9C103 para ajustar la ganancia del micrófono
    digitalWrite(AMP_CS, LOW); // Selecciona la resistencia X9C103 (activa el chip select)
    delay(1);                  // Pequeña pausa para asegurar la comunicación

    // Determina si se debe incrementar o disminuir la ganancia del micrófono
    // Basado en la temperatura promedio
    if (promedioTemperatura > 27.5)
    {
      // Incrementa la ganancia del micrófono si la temperatura promedio es mayor a 27.5°C
      digitalWrite(AMP_U_D, HIGH);
    }
    else
    {
      // Decrementa la ganancia del micrófono si la temperatura promedio es 27.5°C o menor
      digitalWrite(AMP_U_D, LOW);
    }
    delay(1); // Pequeña pausa para asegurar la configuración

    // Genera pulsos para incrementar o decrementar la ganancia
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(AMP_INC, LOW); // Pulso bajo
      delay(1);
      digitalWrite(AMP_INC, HIGH); // Pulso alto
      delay(1);
    }

    // Termina la comunicación con la resistencia digital X9C103
    digitalWrite(AMP_CS, HIGH); // Desactiva el chip select
  }

  /*------------------------------------------------------------------------------------------------------------------/
  /                                           Verificación de Modo de Operación                                       /
  /------------------------------------------------------------------------------------------------------------------*/

  // Verifica si la tarea está operando en modo sin conexión
  if (modoSinConexion())
  {
    mostrarMensajeTareaCompleta(true); // Muestra un mensaje indicando que la tarea está completada
  }
  else
  {
    mostrarMensajeTareaCompleta(false); // Muestra un mensaje indicando que la tarea no está completada
  }

  /*------------------------------------------------------------------------------------------------------------------/
  /                                           Publicación  de Datos Recolectados                                      /
  /------------------------------------------------------------------------------------------------------------------*/

  // Función para publicar datos a través de MQTT
  void publicarDatosMQTT(float *temperaturas, float *humedades, int *ruidos)
  {
    // Lógica para publicar datos a través de MQTT
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                                            Lectura del Sensor DHT22                                               /
/------------------------------------------------------------------------------------------------------------------*/

// Función para leer la temperatura del sensor DHT22
float leerTemperatura()
{
  // Lee la temperatura del sensor DHT22
  float temp = dht.readTemperature();

  // Verifica si la lectura fue exitosa (si no es un valor NaN)
  if (isnan(temp))
  {
    Serial.println("Error al leer la temperatura!"); // Imprime un mensaje de error si la lectura falla
    return -1;                                       // Retorna -1 para indicar un error en la lectura
  }
  return temp; // Retorna el valor de la temperatura leída
}

// Función para leer la humedad del sensor DHT22
float leerHumedad()
{
  // Lee la humedad del sensor DHT22
  float humedad = dht.readHumidity();

  // Verifica si la lectura fue exitosa (si no es un valor NaN)
  if (isnan(humedad))
  {
    Serial.println("Error al leer la humedad!"); // Imprime un mensaje de error si la lectura falla
    return -1;                                   // Retorna -1 para indicar un error en la lectura
  }
  return humedad; // Retorna el valor de la humedad leída
}

/*------------------------------------------------------------------------------------------------------------------/
/                                               Lectura de Ruido Capturado                                          /
/------------------------------------------------------------------------------------------------------------------*/

// Función para leer el valor de ruido del micrófono
int leerRuido()
{
  // Lee el valor de ruido del micrófono
  int micValue = analogRead(MIC_SIGNAL_OUT);

  // Verifica si la lectura está dentro de los valores esperados (0 a 1024)
  if (micValue < 0 || micValue > 1024)
  {
    Serial.println("Error al leer el ruido!"); // Imprime un mensaje de error si la lectura falla
    return -1;                                 // Retorna -1 para indicar un error en la lectura
  }
  return micValue; // Retorna el valor del ruido leído
}
/*------------------------------------------------------------------------------------------------------------------/
/                                            MONITOR (Mostrar en Pantalla)                                         /
/------------------------------------------------------------------------------------------------------------------*/

void showOnScreen(float temperatura, float humedad, int ruido)
{
  // Limpiar la pantalla antes de actualizar los valores
  tft.fillScreen(ILI9488_BLACK);

  // Configurar el cursor y las propiedades del texto
  tft.setTextColor(ILI9488_WHITE);
  tft.setTextSize(2);

  // Mostrar la temperatura
  tft.setCursor(10, 30);
  tft.print("Temperatura: ");
  tft.print(temperatura);
  tft.println(" C");

  // Mostrar la humedad
  tft.setCursor(10, 60);
  tft.print("Humedad: ");
  tft.print(humedad);
  tft.println(" %");

  // Mostrar el ruido
  tft.setCursor(10, 90);
  tft.print("Ruido: ");
  tft.print(ruido);
  tft.println(" dB");

  // Puedes añadir más información según sea necesario
  // Por ejemplo, el estado de conexión o la hora
}

/*------------------------------------------------------------------------------------------------------------------/
/                                          Almacenar y Leer Datos Locales                                           /
/------------------------------------------------------------------------------------------------------------------*/

// Función para almacenar datos en la memoria EEPROM
void almacenarDatos(float temperaturas[], float humedades[], int ruidos[])
{
  // Inicializar la EEPROM con el tamaño definido
  EEPROM.begin(EEPROM_SIZE);

  // Almacenar los valores de temperatura en la EEPROM
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = TEMP_START_ADDR + i * sizeof(float); // Calcular la dirección de memoria para cada temperatura
    float temp = temperaturas[i];                   // Obtener el valor de temperatura actual
    EEPROM.put(addr, temp);                         // Guardar el valor de temperatura en la dirección calculada
  }

  // Almacenar los valores de humedad en la EEPROM
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = HUM_START_ADDR + i * sizeof(float); // Calcular la dirección de memoria para cada humedad
    float hum = humedades[i];                      // Obtener el valor de humedad actual
    EEPROM.put(addr, hum);                         // Guardar el valor de humedad en la dirección calculada
  }

  // Almacenar los niveles de ruido en la EEPROM
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = NOISE_START_ADDR + i * sizeof(int); // Calcular la dirección de memoria para cada nivel de ruido
    int noise = ruidos[i];                         // Obtener el valor de ruido actual
    EEPROM.put(addr, noise);                       // Guardar el valor de ruido en la dirección calculada
  }

  // Finalizar la escritura en la EEPROM para asegurar que todos los datos se guarden
  EEPROM.commit();
}

// Función para leer los datos almacenados en la EEPROM (por si necesitas leerlos más tarde)
void leerDatosAlmacenados(float temperaturas[], float humedades[], int ruidos[])
{
  // Inicializar la EEPROM con el tamaño definido
  EEPROM.begin(EEPROM_SIZE);

  // Leer los valores de temperatura desde la EEPROM
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = TEMP_START_ADDR + i * sizeof(float); // Calcular la dirección de memoria para cada temperatura
    EEPROM.get(addr, temperaturas[i]);              // Leer el valor de temperatura desde la dirección calculada
  }

  // Leer los valores de humedad desde la EEPROM
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = HUM_START_ADDR + i * sizeof(float); // Calcular la dirección de memoria para cada humedad
    EEPROM.get(addr, humedades[i]);                // Leer el valor de humedad desde la dirección calculada
  }

  // Leer los niveles de ruido desde la EEPROM
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = NOISE_START_ADDR + i * sizeof(int); // Calcular la dirección de memoria para cada nivel de ruido
    EEPROM.get(addr, ruidos[i]);                   // Leer el valor de ruido desde la dirección calculada
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                                               Timer y Interrupciones                                              /
/------------------------------------------------------------------------------------------------------------------*/

// Manejador de interrupción del timer
void IRAM_ATTR onTimer()
{
  static unsigned long contadorRuido = 0;
  static unsigned long contadorTempHum = 0;

  contadorRuido++;
  contadorTempHum++;

  // Establece los flags para las lecturas periódicas de ruido y temperatura/humedad
  if (contadorRuido >= INTERVALO_RUIDO)
  {
    leerMicrofonoFlag = true;
    contadorRuido = 0;
  }

  if (contadorTempHum >= INTERVALO_TEMP_HUM)
  {
    leerTempHumFlag = true;
    contadorTempHum = 0;
  }
}

void setupTimer()
{
  noInterrupts(); // Desactiva interrupciones durante la configuración

  // Configuración del timer para interrupciones cada 1 ms
  timer = timerBegin(0, 80, true);             // Usa el timer 0, con prescaler 80 (1 tick = 1 microsegundo)
  timerAttachInterrupt(timer, &onTimer, true); // Adjunta la función de interrupción
  timerAlarmWrite(timer, 1000, true);          // Configura el período del timer (1 ms)
  timerAlarmEnable(timer);                     // Habilita la alarma del timer

  interrupts(); // Activa interrupciones
}

/*------------------------------------------------------------------------------------------------------------------/
/                                               Notificaciones                                                     /
/------------------------------------------------------------------------------------------------------------------*/

void handleNotifications()
{
  tft.fillScreen(ILI9488_BLACK);
  tft.setTextColor(ILI9488_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Cargando Notificaciones...");

  delay(1000); // Simular carga

  std::vector<String> notifications;
  // Aquí se debe implementar la lógica real para obtener las notificaciones
  notifications.push_back("Error de conexion WiFi")

      if (notifications.empty())
  {
    showNoNotificationsMessage();
  }
  else
  {
    showNotificationsList(notifications);
  }
}

void showNotificationsList(const std::vector<String> &notifications)
{
  int selectedIndex = 0;
  while (true)
  {
    tft.fillScreen(ILI9488_BLACK);
    tft.setCursor(10, 10);
    tft.println("Notificaciones:");

    for (int i = 0; i < notifications.size(); i++)
    {
      if (i == selectedIndex)
      {
        tft.setTextColor(ILI9488_YELLOW);
      }
      else
      {
        tft.setTextColor(ILI9488_WHITE);
      }
      tft.println(notifications[i]);
    }

    tft.setTextColor(ILI9488_WHITE);
    tft.println("");
    tft.println("2: Arriba, 8: Abajo");
    tft.println("#: Seleccionar");
    tft.println("*: Volver a HOME");

    char key = waitForValidKeypadEvent();
    if (key == '2' && selectedIndex > 0)
    {
      selectedIndex--;
    }
    else if (key == '8' && selectedIndex < notifications.size() - 1)
    {
      selectedIndex++;
    }
    else if (key == '#')
    {
      showSelectedNotification(notifications[selectedIndex]);
    }
    else if (key == '*')
    {
      return; // Volver a HOME
    }
  }
}

/*------------------------------------------------------------------------------------------------------------------/
/                                               Configuraciones                                                     /
/------------------------------------------------------------------------------------------------------------------*/

// Función principal del menú de configuración
void handleConfiguracion()
{
  int cursorPos = 0;
  while (true)
  {
    cargarInformacionPantalla();
    moverCursor(cursorPos, 2); // Inicialmente mueve el cursor arriba

    while (true)
    {
      char key = waitForValidKeypadEvent();
      if (key == '2')
        moverCursor(cursorPos, 2);
      else if (key == '8')
        moverCursor(cursorPos, 8);
      else if (key == '#')
      {
        switch (cursorPos)
        {
        case 0:
          configurarWiFi();
          break;
        case 1:
          configurarConexionServidor();
          break;
        case 2:
          configurarNotificaciones();
          break;
        case 3:
          if (volverHome())
            return;
        }
        break; // Sale del bucle interno para recargar la pantalla
      }
      delay(100);
    }
  }
}

// Función para configurar Wi-Fi
void configurarWiFi()
{
  tft.fillScreen(ILI9488_BLACK);
  tft.setCursor(0, 0);
  tft.println("Configurando Wi-Fi...");

  // Aqui se va a implementar la lógica para configurar Wi-Fi
  // Por ejemplo, solicitar SSID y contraseña
  Config config;
  EEPROM.get(CONFI_START_ADDR, config);
  config.wifiConfigured = true;
  EEPROM.put(CONFI_START_ADDR, config);
  EEPROM.commit();
}

// Función para configurar conexión con el servidor
void configurarConexionServidor()
{
  tft.fillScreen(ILI9488_BLACK);
  tft.setCursor(0, 0);
  tft.println("Configurando conexion con el servidor...");

  // Aqui se va a implementar la lógica para configurar la conexión con el servidor
  Config config;
  EEPROM.get(CONFI_START_ADDR, config);
  config.serverConfigured = true;
  EEPROM.put(CONFI_START_ADDR, config);
  EEPROM.commit();
}

// Función para configurar notificaciones
void configurarNotificaciones()
{
  tft.fillScreen(ILI9488_BLACK);
  tft.setCursor(0, 0);
  tft.println("Configurando notificaciones...");

  // Aqui se va a implementar la lógica para configurar notificaciones
  Config config;
  EEPROM.get(CONFI_START_ADDR, config);
  config.notificationsConfigured = true;
  EEPROM.put(CONFI_START_ADDR, config);
  EEPROM.commit();
}

// Función para volver a HOME
bool volverHome()
{
  tft.fillScreen(ILI9488_BLACK);
  tft.setCursor(0, 0);
  tft.println("Volviendo a HOME...");

  delay(1000);

  homeScreen();

  return true;
}

// Función para mover el cursor
void moverCursor(int &cursorPos, int direccion)
{
  if (direccion == 2 && cursorPos > 0)
    cursorPos--;
  if (direccion == 8 && cursorPos < 3)
    cursorPos++;

  tft.fillRect(0, 20, 20, tft.height(), ILI9488_BLACK);
  tft.setCursor(0, 20 + cursorPos * 20);
  tft.print(">");
}
