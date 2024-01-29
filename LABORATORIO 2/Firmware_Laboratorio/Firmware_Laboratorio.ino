/***********************************************************************************************************
*                                                                                                          *
*  Firmware para el  Laboratorio #1 del curso Diseño de Dispositivos IoT de la Maestría en Ingeniería      *
*                                                                                                          *
*  Presentado por:                                                                                         *
*  - Geradeyra Acevedo Sierra                                                                              *
*  - Juan Manuel Calvo Duque                                                                               *
*  - Lina Marcela García Palacio                                                                           *
***********************************************************************************************************/

/***********************************************************************************************************
*                              Librerías utilizadas                                                         *
************************************************************************************************************/
#include "RMaker.h" 
#include "WiFi.h" 
#include "WiFiProv.h" 
#include "DHT.h" 
#include <SimpleTimer.h> 
#include <wifi_provisioning/manager.h>

/***********************************************************************************************************
*                              Función de control                                                          *
************************************************************************************************************/
void control_LED(int Elem, int PIN_ELEM, boolean &estado){
  String Elemento;
  estado = !estado;
  digitalWrite(PIN_ELEM, estado);
  String text = (estado)? "ON" : "OFF";
  switch(Elem){
    case 1:
      Elemento = "Humidicador";
      break;
    case 2: 
      Elemento = "LED2";
      break;
  }
  Serial.println(Elemento+" es "+text);
}

/***********************************************************************************************************
*                              DEFINICIÓN DE VARIAABLES, CONSTANTES Y OBJETOS                              *
************************************************************************************************************/
#define LIMITE_POR_DEFECTO 20    
float Limite = LIMITE_POR_DEFECTO;
//Credenciales para la conexión a la red WiFi
const char *SSID = "JuanMCD";
const char *SSIDpass = "1234567890";

//definición de pines GPIO
static uint8_t gpio_reset = 0;
static uint8_t DHT_PIN = 4;
static uint8_t PIN_HUMIDIFICADOR = 18;
static uint8_t PIN_VENTILADOR = 19;
static uint8_t LED1 = 2;
static uint8_t LED2 = 4;

bool ESTADO_LED1 = false;
bool ESTADO_LED2 = false;
bool ESTADO_HUMIDIFICADOR = false;
bool CONECTADO_A_WIFI = false;

DHT dht(DHT_PIN, DHT11);

//definición de un objeto de tipo SimpleTimer
SimpleTimer Temporizador;

//Nombres de los dispositivos
char Temperatura[] = "Temperatura";
char Humedad[] = "Humedad";
char Humidificador[] = "Humidificador";
char LED[] = "LED2";
char Selector[] = "Selector";

//definición de dispositivos
static TemperatureSensor temperature(Temperatura);
static TemperatureSensor humidity(Humedad);
static Switch humidificador(Humidificador, &PIN_HUMIDIFICADOR);
static Switch my_switch2(LED, &LED2);
static Switch selector(Selector, &LED1);

/***********************************************************************************************************
*                            CONEXIÓN A WIFI                                                               *
************************************************************************************************************/
void sysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id)
  {
  case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
    Serial.printf("\nProvisioning Started with name \"%s\" and SSIDpass \"%s\" on BLE\n", SSID, SSIDpass);
    printQR(SSID, SSIDpass, "ble");
#else
    Serial.printf("\nProvisioning Started with name \"%s\" and SSIDpass \"%s\" on SoftAP\n", SSID, SSIDpass);
    printQR(SSID, SSIDpass, "softap");
#endif
    break;
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    Serial.printf("\nConnected to Wi-Fi!\n");
    CONECTADO_A_WIFI = true;
    delay(500);
    break;
  case ARDUINO_EVENT_PROV_CRED_RECV:
  {
    Serial.println("\nReceived Wi-Fi credentials");
    Serial.print("\tSSID : ");
    Serial.println((const char *)sys_event->event_info.prov_cred_recv.ssid);
    Serial.print("\tPassword : ");
    Serial.println((char const *)sys_event->event_info.prov_cred_recv.password);
    break;
  }
  case ARDUINO_EVENT_PROV_INIT:
    wifi_prov_mgr_disable_auto_stop(10000);
  case ARDUINO_EVENT_PROV_CRED_SUCCESS:
    wifi_prov_mgr_stop_provisioning();
    break;
  }
}
/***********************************************************************************************************
*                            FUNCIÓN CALLBACK                                                              *
************************************************************************************************************/
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx){
  const char *device_name = device->getDeviceName();
  Serial.println(device_name);
  const char *param_name = param->getParamName();

  if (strcmp(device_name, "LED2") == 0){
    if (strcmp(param_name, "Power") == 0){
      Serial.printf("Received value = %s for %s - %s\n", val.val.f ? "true" : "false", device_name, param_name);
      ESTADO_LED2 = val.val.b;
      (ESTADO_LED2 == false) ? digitalWrite(LED2, LOW) : digitalWrite(LED2, HIGH);
      param->updateAndReport(val);
    }
  }
  //__________________________________________________________________________________________
  else if(strcmp(device_name, "Selector") == 0) {
    if(strcmp(param_name, "Power") == 0) {
    }else if (strcmp(param_name, "Level") == 0) {
      int LimiteActualizado = val.val.i;
      Serial.printf("\nReceived value = %d for %s - %s\n", LimiteActualizado, device_name, param_name);
      Limite = LimiteActualizado; 
      delay(15); 
      }
    }
  //__________________________________________________________________________________________
  else if (strcmp(device_name, "Humidificador") == 0){
    if (strcmp(param_name, "Power") == 0){
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      ESTADO_HUMIDIFICADOR = val.val.b;
      (ESTADO_HUMIDIFICADOR == false) ? digitalWrite(PIN_HUMIDIFICADOR, LOW) : digitalWrite(PIN_HUMIDIFICADOR, HIGH);
      if(ESTADO_HUMIDIFICADOR == true){
        Serial.println("Humidificador Activado");
      }else{
        Serial.println("Humidificador Desactivado");
      }
      param->updateAndReport(val);
    }
  }
}
/***********************************************************************************************************
*                            FUNCIÓN SETUP                                                                 *
************************************************************************************************************/
void setup()
{
  Serial.begin(115200);
  pinMode(gpio_reset, INPUT);
  pinMode(PIN_HUMIDIFICADOR, OUTPUT);
  pinMode(PIN_VENTILADOR, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  dht.begin();
  Node Laboratorio1;
  Laboratorio1 = RMaker.initNode("Laboratorio 1. Diseño de Dispositivos IoT");

//Crear parámetro de nivel mediante un slider asociado el dispositivo Selector
  Param level_param("Level", "custom.param.level", value(LIMITE_POR_DEFECTO)
                    , PROP_FLAG_READ | PROP_FLAG_WRITE);
  level_param.addBounds(value(13), value(35), value(1));
  level_param.addUIType(ESP_RMAKER_UI_SLIDER);
  selector.addParam(level_param);

  humidificador.addCb(write_callback);
  my_switch2.addCb(write_callback);
  selector.addCb(write_callback);

  //Añadir Dispositivos
  Laboratorio1.addDevice(temperature);
  Laboratorio1.addDevice(humidity);
  Laboratorio1.addDevice(humidificador);
  Laboratorio1.addDevice(my_switch2);
  Laboratorio1.addDevice(selector);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();
  Temporizador.setInterval(3000);
  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, SSIDpass, SSID);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, SSIDpass, SSID);
#endif
}

/***********************************************************************************************************
*                            FUNCIÓN LOOP                                                                  *
************************************************************************************************************/

void loop()
{
  if (Temporizador.isReady() && CONECTADO_A_WIFI)
  {
    enviarTemperaturayHumedad(); 
    Temporizador.reset();
  }

  if (digitalRead(gpio_reset) == LOW)
  { 
    Serial.printf("Reset Button Pressed!\n");
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW)
      delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000)
    {
      Serial.printf("Reset to factory.\n");
      CONECTADO_A_WIFI = false;
      RMakerFactoryReset(2);
    }
    else if ((endTime - startTime) > 3000)
    {
      Serial.printf("Reset Wi-Fi.\n");
      CONECTADO_A_WIFI = false;
      RMakerWiFiReset(2);
    }
  }
  delay(100);
}

/***********************************************************************************************************
*                            FUNCIÓN LOOP                                                                  *
************************************************************************************************************/

//Con esta función se miden los valores de temperatura y humedad, se envían y se estable la acción de control del ventilador
void enviarTemperaturayHumedad(){
  //float temperaturaMedida = dht.readTemperature();
  //float humedadMedida = dht.readHumidity();
  float temperaturaMedida = random(21,23);
  float humedadMedida = random(80,82);
  Serial.print("Temperatura:");
  Serial.print(temperaturaMedida);
  Serial.print(" | Humedad - ");
  Serial.print(humedadMedida);
  Serial.print(" | Limite para accionar el Ventilador: ");
  Serial.println(Limite);

  temperature.updateAndReportParam("Temperature", temperaturaMedida);
  humidity.updateAndReportParam("Temperature", humedadMedida);

  if (temperaturaMedida > Limite){
    digitalWrite(LED2, HIGH);
    digitalWrite(PIN_VENTILADOR, HIGH);
    Serial.println("******Ventilador Activado******");
  }
  else
  {
    digitalWrite(LED2, LOW);
    digitalWrite(PIN_VENTILADOR, LOW);
    Serial.println("******Ventilador Desactivado******");
  }
}