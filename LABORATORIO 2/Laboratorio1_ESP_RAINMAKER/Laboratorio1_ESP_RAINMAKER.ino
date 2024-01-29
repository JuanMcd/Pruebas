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

//Control de LED y Humidificador
void control_LED(int Elem, int PIN_ELEM, boolean &estado){
  String Elemento;
  estado = !estado;
  digitalWrite(PIN_ELEM, estado);
  String text = (estado)? "ON" : "OFF";
  switch(Elem){
    case 1:
      Elemento = "LED1";
      break;
    case 2: 
      Elemento = "LED2";
      break;
    case 3: 
      Elemento = "Humidificador";
      break;
  }
  Serial.println(Elemento+" es "+text);
}

/***********************************************************************************************************
*                              DEFINICIÓN DE VARIAABLES, CONSTANTES Y OBJETOS                              *
************************************************************************************************************/

//Credenciales para WiFi
const char *SSID = "JuanMCD"; 
const char *SSIDPass = "1234567890";

//Nombres de los dispositivos
char dispositivo1[] = "LED1";
char dispositivo2[] = "LED2";
char sensorTemperatura[] = "Temperatura";
char sensorHumedad[] = "Humedad";
char humidificador[] = "Humidificador";

//Definición de pines GPIO
static uint8_t LED1 = 23;  
static uint8_t LED2 = 22;  
static uint8_t VENT = 18;
static uint8_t HUMID = 19;
static uint8_t DHT_PIN = 4;
static uint8_t WIFI_LED = 2;   
static uint8_t gpio_reset = 0;

//Valores iniciales
bool ESTADO_LED1 = LOW;
bool ESTADO_LED2 = LOW;
bool ESTADO_VENT = LOW;
bool ESTADO_HUMID = LOW;
bool CONECTADO_A_WIFI = false;
float LimiteTemperatura = 23;
//Para el temporizador
SimpleTimer Temporizador;

//Para el DHT11
DHT dht(DHT_PIN, DHT11);
/***********************************************************************************************************
*                            CREACIÓN DE DISPOSITIVOS                                                      *
************************************************************************************************************/

static Switch my_switch1(dispositivo1, &LED1);
static Switch my_switch2(dispositivo2, &LED2);
static TemperatureSensor temperature(sensorTemperatura);  
static TemperatureSensor humidity(sensorHumedad);
static Switch humidif(humidificador, &HUMID);

/***********************************************************************************************************
*                            CONEXIÓN A WIFI                                                               *
************************************************************************************************************/

void sysProvEvent(arduino_event_t *sys_event)
{
  //switch para diferentes tipos de eventos
  switch (sys_event->event_id)
  {
  //inicio del aprovisamiento
  case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
    Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", SSID, SSIDPass);
    printQR(SSID, SSIDPass, "ble");
#else
    Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", SSID, SSIDPass);
    printQR(SSID, SSIDPass, "softap");
#endif
    break;
    //si la conexión a la red es exitosa
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    //se imprime un mensaje de que es exitosa
    Serial.printf("\nConnected to Wi-Fi!\n");
    CONECTADO_A_WIFI = true;
    delay(500);
    break;
  //caso para cuando hay recepción de credenciales  durante el aprovisamiento
  case ARDUINO_EVENT_PROV_CRED_RECV:
  {
    //impresión del mensaje que inidca que se han recibido credenciales
    Serial.println("\nReceived Wi-Fi credentials");
    Serial.print("\tSSID : ");
    Serial.println((const char *)sys_event->event_info.prov_cred_recv.ssid);
    Serial.print("\tPassword : ");
    Serial.println((char const *)sys_event->event_info.prov_cred_recv.password);
    break;
  }
  //caso para la inicialización del aprovisamiento
  case ARDUINO_EVENT_PROV_INIT:
    wifi_prov_mgr_disable_auto_stop(10000);
    break;
  //caso del aprovisamiento de credenciales
  case ARDUINO_EVENT_PROV_CRED_SUCCESS:
    wifi_prov_mgr_stop_provisioning();
    break;
  }
}
/***********************************************************************************************************
*                            FUNCIÓN CALLBACK                                                              *
************************************************************************************************************/
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx){
  //obtiene el nombre del dispositivo
  const char *device_name = device->getDeviceName();
  //imprime el nombre del dispositivo en la consola
  Serial.println(device_name);
  //se obtiene el nombre del parametro
  const char *param_name = param->getParamName();
  
  if(strcmp(device_name, dispositivo1) == 0) {
    Serial.printf("Lightbulb1 = %s\n", val.val.b? "true" : "false");
    //comprueba si el parametro es power
    if(strcmp(param_name, "Power") == 0){
      //actualiza el estado del bombillo y y se llama la función que lo controla
      ESTADO_LED1 = val.val.b;
      ESTADO_LED1 = !ESTADO_LED1;
      control_LED(1, LED1, ESTADO_LED1);
    }
  }
   
  //----------------------------------------------------------------------------------
  else if(strcmp(device_name, dispositivo2) == 0) {
      
    Serial.printf("Valor del pulsador = %s\n", val.val.b? "true" : "false");

    if(strcmp(param_name, "Power") == 0) {
      ESTADO_LED2 = val.val.b;
      ESTADO_LED2 = !ESTADO_LED2;
      control_LED(2, LED2, ESTADO_LED2);
    }
  }
  else if(strcmp(device_name, humidificador) == 0) {
      
    //Serial.printf("Valor del pulsador = %s\n", val.val.b? "true" : "false");
    if(strcmp(param_name, "Power") == 0) {
      //ESTADO_HUMID = val.val.b;
      //ESTADO_HUMID = !ESTADO_HUMID;
      //control_LED(3, HUMID, ESTADO_HUMID);
    }else if (strcmp(param_name, "Level") == 0){
      float nuevoLimiteTemperatura = val.val.i;
      Serial.print(nuevoLimiteTemperatura);
      Serial.printf("\nReceived value = %d for %s - %s\n", nuevoLimiteTemperatura, device_name, param_name);
      LimiteTemperatura = nuevoLimiteTemperatura; 
      delay(15);
    }
  }

}


void setup(){
  uint32_t chipId = 0;
  Serial.begin(115200);

  //Seleccionar modo de funcionamiento de los pines GPIO
  pinMode(LED1, OUTPUT);   
  pinMode(LED2, OUTPUT);  
  pinMode(VENT, OUTPUT);
  pinMode(HUMID, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);   
  pinMode(gpio_reset, INPUT);

  //Iniciar Sensor DHT11
  dht.begin();
  //------------------------------------------------------------------------------
  // Estados GPIO iniciales 
  digitalWrite(LED1, !ESTADO_LED1);
  digitalWrite(LED2, !ESTADO_LED2);
  //------------------------------------------------------------------------------
  Node Laboratorio1;    
  Laboratorio1 = RMaker.initNode("Laboratorio #1. Diseño de Dispositivos IoT");

  //--------------Creación de parámetros y agregación de dispositivos-----------------------------
  Param level_param("Level", "custom.param.level", value(23), PROP_FLAG_READ | PROP_FLAG_WRITE);
  level_param.addBounds(value(14), value(30), value(1)); //sart_value, end_value, interval
  level_param.addUIType(ESP_RMAKER_UI_SLIDER);
  //se añade al parametro al dispositivo de temperatura
  humidif.addParam(level_param);

  my_switch1.addCb(write_callback);
  my_switch2.addCb(write_callback);
  humidif.addCb(write_callback);

  //Agregar dispositivos
  Laboratorio1.addDevice(my_switch1);
  Laboratorio1.addDevice(my_switch2);
  Laboratorio1.addDevice(temperature);
  Laboratorio1.addDevice(humidity);
  Laboratorio1.addDevice(humidif);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();
  //------------------------------------------------------------------------------
  //Nombre de servicio
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.printf("\nChip ID:  %d Nombre de Servicio: %s\n", chipId, SSID);
  //------------------------------------------------------------------------------
  Serial.printf("\nIniciando ESP-RainMaker\n");
  RMaker.start();
  //Configurar intervalo del temporizador
  Temporizador.setInterval(5000);
  //------------------------------------------------------------------------------
  WiFi.onEvent(sysProvEvent);
  #if CONFIG_IDF_TARGET_ESP32
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, SSIDPass, SSID);
  #else
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, SSIDPass, SSID);
  #endif
  //------------------------------------------------------------------------------
  digitalWrite(LED1, ESTADO_LED1);
  digitalWrite(LED2, ESTADO_LED2);
  
  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, ESTADO_LED1);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, ESTADO_LED2);

  Serial.printf("El estado del LED 1 es %s \n", ESTADO_LED1? "ON" : "OFF");
  Serial.printf("El estado del LED 2 es %s \n", ESTADO_LED2? "ON" : "OFF");  
}

void loop(){
  if(Temporizador.isReady() && CONECTADO_A_WIFI){
    enviarTemperaturayHumedad();
    Temporizador.reset();
  }
  if(digitalRead(gpio_reset) == LOW) { //Pulsador RESET presionado
    Serial.printf("Pulsador Reset activado!\n");
    // Manejo de antirrebote
    delay(100);
    int tiempoInicial = millis();
    while(digitalRead(gpio_reset) == LOW) delay(50);
    int tiempoFinal = millis();
    //_______________________________________________________________________
    if ((tiempoFinal - tiempoInicial) > 10000) {
      // Si está presionado RESET por más de 10 segundos, eliminar todo
      CONECTADO_A_WIFI = false;
      Serial.printf("Restauración de fábrica.\n");
      RMakerFactoryReset(2);
    } 
    //_______________________________________________________________________
    else if ((tiempoFinal - tiempoInicial) > 3000) {
      CONECTADO_A_WIFI = false;
      Serial.printf("Reset Wi-Fi.\n");
      // Si está presionado RESET por más de 3 segundos pero menos de 10 segundos, reinicia Wi-Fi
      RMakerWiFiReset(2);
    }
  }
}


void enviarTemperaturayHumedad(){
  //Leer los datos del sensor
  //float temp = dht.readTemperature();
  //float humed = dht.readHumidity();
  float temp = random(13,45);
  float humed = random(0,100);
  Serial.print("Temperatura: ");
  Serial.print(temp);
  Serial.print("|");
  Serial.print("Humedad: ");
  Serial.println(humed);
  Serial.print("Set ");
  Serial.println(LimiteTemperatura);

  //Enviar valores a la APP
  temperature.updateAndReportParam("Temperature", temp); 
  humidity.updateAndReportParam("Temperature", humed);

  //Control del ventilador según la temperatura

  if(temp>LimiteTemperatura){
    digitalWrite(VENT,HIGH);
    Serial.println("Ventilador encendido");
  }else{
    digitalWrite(VENT,LOW);
  }
}

