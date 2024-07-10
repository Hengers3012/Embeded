#include <Arduino.h>
#include <Wire.h>

// Pantalla ILI9488
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <ILI9488.h>

// Teclado Matricial 3x4
#include <Keypad_I2C.h>
#include <PCF8574_HD44780_I2C.h>

// DHT22
#include <DHT.h>

// WIFI
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// Memory EEPROM
#include <EEPROM.h>

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

#define PREAMP_U_D 25
#define PREAMP_INC 26
#define PREAMP_CS 27

#define AMP_U_D 21
#define AMP_INC 22
#define AMP_CS 23

#define DATA_SENSOR_TEMP_AND_HR 0
#define DHTTYPE DHT22

#define MIC_SIGNAL_OUT 2

#define BUZZER_PIN 5

// Credenciales WIFI
#define WIFI_SSID "SSID"
#define WIFI_PASSWORD "PASSWORD"

// Credenciales Server MQTT
#define MQTT_SERVER "mqtt_server"
#define MQTT_PORT 0000
#define MQTT_USER "mqtt_user"
#define MQTT_PASSWORD "mqtt_password"

// Configuración del cliente MQTT
WiFiClientSecure espClient;
PubSubClient client(espClient);

// Definiciones de direcciones de memoria en EEPROM
#define EEPROM_SIZE 512
#define TEMP_START_ADDR 0
#define HUM_START_ADDR 40
#define NOISE_START_ADDR 80
#define DATA_SIZE 40

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

// Variables para manejar las banderas de interrupción
volatile bool leerTempHumFlag = false;
volatile bool leerMicrofonoFlag = false;

// Variables globales
volatile float temperatura = 0.0;
volatile float humedad = 0.0;
volatile int micValue = 0;

volatile unsigned long tiempoRuidoAnterior = 0;
volatile unsigned long tiempoTempHumAnterior = 0;

volatile int ruidos[12] = {0};
volatile float temperaturas[10] = {0.0};
volatile float humedades[10] = {0.0};

volatile int indiceRuido = 0;
volatile int indiceTempHum = 0;

float promedioTemperatura = 0.0;
float promedioHumedad = 0.0;

float temperaturaAnterior = 0.0;
float humedadAnterior = 0.0;

bool modoSinConexion = false;

unsigned long tiempoInicioTarea = 0;
const unsigned long duracionTarea = 0;

unsigned long tiempoUltimaPublicacion = 0;
const unsigned long intervaloPublicacion = 0;

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
  kpd.begin();

  // Configuración de los pines de las teclas de función como entrada
  pinMode(OA_F_1, INPUT_PULLUP);
  pinMode(OA_F_2, INPUT_PULLUP);
  pinMode(OA_F_3, INPUT_PULLUP);
  pinMode(OA_F_4, INPUT_PULLUP);

  // Configuración de las resistencias digitales variables:
  // Preamplificador (Mod_Sensibilidad)
  pinMode(PREAMP_U_D, OUTPUT);
  pinMode(PREAMP_INC, OUTPUT);
  pinMode(PREAMP_CS, OUTPUT);
  // Amplificador (Mod_Ganancia)
  pinMode(AMP_U_D, OUTPUT);
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

  // Verificar la conexión WiFi y MQTT
  if (WiFi.status() != WL_CONNECTED || !client.connected())
  {
    // Reintentar la conexión
    reconnect();
  }

  // Gestionar los eventos MQTT si hay una conexión activa
  if (client.connected())
  {
    client.loop();
  }

  // Manejar interrupciones
  handleInterrupts();

  // Verificar si es necesario publicar datos
  if (millis() - tiempoUltimaPublicacion >= intervaloPublicacion)
  {
    publishData();
  }

  // Gestionar eventos del teclado matricial
  char key = kpd.getKey();
  if (key)
  {
    handleKeypadEvent(key);
  }

  // Otras tareas del bucle principal
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
  client.setCallback(callback);
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

// Función para manejar eventos del teclado matricial
void handleKeypadEvent(char key)
{
  // Lógica para manejar eventos del teclado matricial
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

// Función para manejar la reconexión a Wi-Fi y MQTT
void reconnect()
{
  // Reconexión Wi-Fi
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Intentando conectar a SSID: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(5000);
  }

  // Reconexión MQTT
  while (!client.connected())
  {
    Serial.println("Intentando conexión MQTT...");
    if (client.connect("ESP32Client", MQTT_USER, MQTT_PASSWORD))
    {
      Serial.println("Conectado al servidor MQTT.");
      client.subscribe("esp32/output");
    }
    else
    {
      Serial.print("Fallo, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos.");
      delay(5000);
    }
  }
}

// Función de devolución de llamada para el cliente MQTT
void callback(char *topic, byte *payload, unsigned int length)
{
  // Lógica para manejar mensajes recibidos a través de MQTT
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

  while (millis() - tiempoInicioTarea < duracionTarea)
  {
    unsigned long tiempoActual = millis();

    // Proceso de lectura de ruido
    if (tiempoActual - tiempoRuidoAnterior >= 5000)
    {
      ruidos[indiceRuido] = leerRuido();
      indiceRuido = (indiceRuido + 1) % 12; // Circular buffer
      tiempoRuidoAnterior = tiempoActual;

      // Mostrar valores actualizados en pantalla
      showOnScreen(promedioTemperatura, promedioHumedad, ruidos[indiceRuido]);
    }

    // Proceso de lectura de temperatura y humedad
    if (tiempoActual - tiempoTempHumAnterior >= 60000)
    {
      temperaturas[indiceTempHum] = leerTemperatura();
      humedades[indiceTempHum] = leerHumedad();
      indiceTempHum = (indiceTempHum + 1) % 10; /* Circular buffer: es una estructura de datos que utiliza una sola cola de tamaño fijo como si fuera conectada de extremo a extremo,
                                                                    lo cual es muy útil para almacenar datos en tiempo real sin necesidad de desplazar elementos en la memoria.    */

      promedioTemperatura = calcularPromedio(temperaturas, 10);
      promedioHumedad = calcularPromedio(humedades, 10);

      tiempoTempHumAnterior = tiempoActual;

      // Guardar valores anteriores para la comparación
      temperaturaAnterior = promedioTemperatura;
      humedadAnterior = promedioHumedad;

      // Mostrar valores actualizados en pantalla
      showOnScreen(promedioTemperatura, promedioHumedad, ruidos[indiceRuido]);
    }

    // Verificar cambio de temperatura
    if (abs(promedioTemperatura - temperaturaAnterior) > 0.5)
    {
      ajustarGanancia(promedioTemperatura, promedioHumedad);
      ajustarSensibilidad(promedioTemperatura, promedioHumedad);
    }

    // Verificar estado de conexión y publicar datos acumulados
    if (modoSinConexion)
    {
      almacenarDatos(temperaturas, humedades, ruidos);
    }
    else
    {
      if (tiempoActual - tiempoUltimaPublicacion >= intervaloPublicacion)
      {
        publicarDatosMQTT(temperaturas, humedades, ruidos);
        tiempoUltimaPublicacion = tiempoActual;
      }
    }
  }

  float calcularPromedio(float datos[], int cantMed)
  {
    float suma = 0.0;

    for (int i = 0; i < cantMed; i++)
    {
      suma += datos[i];
    }

    return suma / cantMed;
  }

  void ajustarSensibilidad(float promedioTemperatura, float promedioHumedad)
  {
    // Ajustar la resistencia digital X9C503 para la sensibilidad del micrófono
    digitalWrite(PREAMP_CS, LOW); // Seleccionar la resistencia X9C503
    delay(1);

    // La lógica para ajustar la sensibilidad.
    // Podría aplicar algo simple como aumentar o disminuir los pasos en función de la temperatura y la humedad.
    if (promedioHumedad > 80.0)
    {
      // Incrementar la sensibilidad
      digitalWrite(PREAMP_U_D, HIGH);
    }
    else
    {
      // Decrementar la sensibilidad
      digitalWrite(PREAMP_U_D, LOW);
    }
    delay(1);

    // Generar pulsos para incrementar/decrementar
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(PREAMP_INC, LOW);
      delay(1);
      digitalWrite(PREAMP_INC, HIGH);
      delay(1);
    }

    digitalWrite(PREAMP_CS, HIGH); // Desseleccionar la resistencia X9C503
  }

  void ajustarGanancia(float promedioTemperatura, float promedioHumedad)
  {
    // Ajustar la resistencia digital X9C103 para la ganancia del micrófono
    digitalWrite(AMP_CS, LOW); // Seleccionar la resistencia X9C103
    delay(1);

    // La lógica para ajustar la ganancia.
    // Podría aplicar algo simple como aumentar o disminuir los pasos en función de la temperatura y la humedad.
    if (promedioTemperatura > 27.5)
    {
      // Incrementar la ganancia
      digitalWrite(AMP_UD, HIGH);
    }
    else
    {
      // Decrementar la ganancia
      digitalWrite(AMP_UD, LOW);
    }
    delay(1);

    // Generar pulsos para incrementar/decrementar
    for (int i = 0; i < 5; i++)
    {
      digitalWrite(AMP_INC, LOW);
      delay(1);
      digitalWrite(AMP_INC, HIGH);
      delay(1);
    }

    digitalWrite(AMP_CS, HIGH); // Desseleccionar la resistencia X9C103
  }

  // Función para publicar datos a través de MQTT
  void publicarDatosMQTT(float *temperaturas, float *humedades, int *ruidos)
  {
    // Lógica para publicar datos a través de MQTT
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

// Función para almacenar datos en la memoria EEPROM
void almacenarDatos(float temperaturas[], float humedades[], int ruidos[])
{
  // Inicializar EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Almacenar temperaturas
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = TEMP_START_ADDR + i * sizeof(float);
    float temp = temperaturas[i];
    EEPROM.put(addr, temp);
  }

  // Almacenar humedades
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = HUM_START_ADDR + i * sizeof(float);
    float hum = humedades[i];
    EEPROM.put(addr, hum);
  }

  // Almacenar niveles de ruido
  for (int i = 0; i < DATA_SIZE; i++)
  {
    int addr = NOISE_START_ADDR + i * sizeof(int);
    int noise = ruidos[i];
    EEPROM.put(addr, noise);
  }

  // Finalizar la escritura en EEPROM
  EEPROM.commit();
}

float leerTemperatura()
{
  // Leer la temperatura del sensor DHT
  float temp = dht.readTemperature();
  // Verificar si la lectura fue exitosa
  if (isnan(temp))
  {
    Serial.println("Error al leer la temperatura!");
    return -1; // Valor para identificación de error
  }
  return temp;
}

float leerHumedad()
{
  // Leer la humedad del sensor DHT
  float humedad = dht.readHumidity();
  // Verificar si la lectura fue exitosa
  if (isnan(humedad))
  {
    Serial.println("Error al leer la humedad!");
    return -1; // Valor para identificación de error
  }
  return humedad;
}

int leerRuido()
{
  // Leer el valor del micrófono
  int micValue = analogRead(MIC_SIGNAL_OUT);
  // Verificar si la lectura fue exitosa (esto dependerá de cómo manejes los valores esperados)
  if (micValue < 0 || micValue > 1024)
  {
    Serial.println("Error al leer el ruido!");
    return -1; // Valor para identificación de error
  }
  return micValue;
}

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

void almacenarDatos(float temperaturas[], float humedades[], int ruidos[])
{
  // Inicializar EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Almacenar temperaturas

  // Almacenar humedades

  // Almacenar niveles de ruido

  // Finalizar la escritura en EEPROM
  EEPROM.commit();
}

// Función para leer los datos almacenados en EEPROM (por si necesitas leerlos más tarde)
void leerDatosAlmacenados(float temperaturas[], float humedades[], int ruidos[])
{
  // Inicializar EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Leer temperaturas

  // Leer humedades

  // Leer niveles de ruido
}

//--------------------------------------------------------------------------------------------------------------------------------------------------
// Si quiero realizar las mediciones por interrupción será ideal usar banderas para ejecutarlas en el momento especifico,
// ya que las funciones de interrupción deben ser lo más breves posible y no deben realizar lecturas de sensores que podrían tardar en completarse.

// Función para manejar las interrupciones
void handleInterrupts()
{
  // Manejo de interrupción para leer temperatura y humedad
  if (leerTempHumFlag)
  {
    // Lógica para manejar la interrupción de lectura de temperatura y humedad
    leerTempHumFlag = false;
  }

  // Manejo de interrupción para leer micrófono
  if (leerMicrofonoFlag)
  {
    // Lógica para manejar la interrupción de lectura de micrófono
    leerMicrofonoFlag = false;
  }
}

/*
  void leerTempHumInterrupcion()
  {
    leerTempHumFlag = true;
  }

  void leerMicrofonoInterrupcion()
  {
    leerMicrofonoFlag = true;
  }
*/