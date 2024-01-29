/***********************************************************************************************************
*                                                                                                          *
*  Firmware base para el Laboratorio #1 del curso Diseño de Dispositivos IoT de la Maestría en Ingeniería  *
*                                                                                                          *
***********************************************************************************************************/

#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include "DHT.h"
const char *SSID = "JuanMCD";
const char *SSIDPass = "1234567890";

//---------------------------------------------------
/*****************************************************************************************************
 *                                Función control_LED                                                *
*****************************************************************************************************/
void control_LED(int LED_no, int LED_pin, boolean &estado){
  estado = !estado;
  digitalWrite(LED_pin, estado);
  String text = (estado)? "ON" : "OFF";
  Serial.println("LED"+String(LED_no)+" es "+text);
} 
//-------------------------------------------------------
// Definir nombres de los dispositivos the Device Names
char dispositivo1[] = "Boton1";
char dispositivo2[] = "Boton2";
char dispositivo3[] = "Ventilador";
char slider[] = "Selector";

//-------------------------------------------------------
// Definir los GPIO conectados con los LED, al sensor, al MOSFET y al TIP122 y un bool para la conexión a WiFI
static uint8_t LED1 = 23;  //D23
static uint8_t LED2 = 22;  //D22
static uint8_t PIN_DHT= 4; 
static uint8_t PIN_Ventilador = 18;
static uint8_t PIN_Humidificador = 19;

bool conectado_a_wifi = 0;
//-------------------------------------------------------
static uint8_t WIFI_LED    = 2;   //D2
static uint8_t gpio_reset = 0;    // Reset de la ESP32
//-------------------------------------------------------
/* Variables para leer los estados de los pines */
// Estado LED 
bool ESTADO_LED1 = LOW; //Define el estado del LED 1
bool ESTADO_LED2 = LOW; //Define el estado del LED 2
bool ESTADO_Ventilador = false;
bool ESTADO_Humidificador = false;
//---------------------------------------------------
//El framework proporciona algunos tipos de dispositivos estándar 
// como switch, lightbulb, fan, temperature sensor.
static Switch my_switch1(dispositivo1, &LED1);
static Switch my_switch2(dispositivo2, &LED2);
static TemperatureSensor Temperatura("Temperatura");
static TemperatureSensor Humedad("Humedad");
static Swtich Ventilador("Ventilador", &PIN_Ventilador);
//---------------------------------------------------
/* Crear objeto tipo DHT */
DHT dht(PIN_DHT, DHT11);
/****************************************************************************************************
 * sysProvEvent Function
*****************************************************************************************************/
void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {      
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
        Serial.printf("\nAprovisionamiento iniciado con la red llamada \"%s\" y SSIDPass \"%s\" en BLE\n", SSID, SSIDPass);
        printQR(SSID, SSIDPass, "ble");
#else
        Serial.printf("\nAprovisionamiento iniciado con la red llamada \"%s\" y SSIDPass \"%s\" en SoftAP\n", SSID, SSIDPass);
        printQR(SSID, SSIDPass, "softap");
#endif        
        break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.printf("\nConectado a Wi-Fi!\n");
        digitalWrite(WIFI_LED, HIGH);
        conectado_a_wifi = 1;
        break;
    }
}

/****************************************************************************************************
 * Función write_callback 
*****************************************************************************************************/
void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *nombre_dispositivo = device->getDeviceName();
    const char *nombre_parametro = param->getParamName();
    //----------------------------------------------------------------------------------
    if(strcmp(nombre_dispositivo, dispositivo1) == 0) {
      
      Serial.printf("Lightbulb1 = %s\n", val.val.b? "true" : "false", nombre_dispositivo, nombre_parametro);
      
      if(strcmp(nombre_parametro, "Power") == 0) {
        ESTADO_LED1 = val.val.b;
        ESTADO_LED1 = !ESTADO_LED1;
        control_LED(1, LED1, ESTADO_LED1);
      }
    }
   
    //----------------------------------------------------------------------------------
    else if(strcmp(nombre_dispositivo, dispositivo2) == 0) {
      
      Serial.printf("Valor del pulsador = %s\n", val.val.b? "true" : "false", nombre_dispositivo, nombre_parametro);

      if(strcmp(nombre_parametro, "Power") == 0) {
        ESTADO_LED2 = val.val.b;
        ESTADO_LED2 = !ESTADO_LED2;
        control_LED(2, LED2, ESTADO_LED2);
      }
    }

     else if (strcmp(device_name, dispositivo3) == 0)
    {
      if (strcmp(param_name, "Power") == 0)
      {
        Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
        ESTADO_Ventilador = val.val.b;
        (ESTADO_Ventilador == false) ? digitalWrite(PIN_Ventilador, LOW) : digitalWrite(PIN_Ventilador, HIGH);
        param->updateAndReport(val);
      }
    }
    //----------------------------------------------------------------------------------   
    else if (strcmp(device_name, slider) == 0) //aca no se si es este o temperatureSENSOR
    {
      if (strcmp(param_name, "Level") == 0)
      {
        float new_setpoint = val.val.f;
        setPoint = new_setpoint; 
        temperature.updateAndReportParam("Setpoint", new_setpoint);
      }
    }   
}



/****************************************************************************************************
 * Función setup
*****************************************************************************************************/
void setup(){
  //------------------------------------------------------------------------------
  uint32_t chipId = 0;
  Serial.begin(115200);
  // Configuración de GPIO como salida
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(gpio_reset, INPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(PIN_Ventilador, OUTPUT);
  pinMode(PIN_Humidificador, OUTPUT);
  digitalWrite(WIFI_LED, LOW);
  //Inicializar DHT11
  dht.begin();
  //------------------------------------------------------------------------------
  // Estados GPIO iniciales 
  digitalWrite(LED1, !ESTADO_LED1);
  digitalWrite(LED2, !ESTADO_LED2);
  //------------------------------------------------------------------------------
  //Node DispLab1;    
  //DispLab1 = RMaker.initNode("Laboratorio #1. Diseño de Dispositivos IoT");
  
  Node TareaLab1;
  TareaLab1 = RMaker.initNode("Laboratorio #1. Diseño de Dispositivos IoT");
  //------------------------------------------------------------------------------
  // Dispositivo interruptor estándar
  my_switch1.addCb(write_callback);
  my_switch2.addCb(write_callback);
  Ventilador.addCb(write_callback);
  //rango temperatura->slide
  Param level_param("Level", "custom.param.level", value(Default_Temp), PROP_FLAG_READ | PROP_FLAG_WRITE);
  level_param.addBounds(value(20), value(24), value(1)); //sart_value, end_value, interval
  level_param.addUIType(ESP_RMAKER_UI_SLIDER);
  //se añade al parametro al dispositivo de temperatura
  Temperatura.addParam(level_param);
  //------------------------------------------------------------------------------

  //------------------------------------------------------------------------------
  // Añadir un dispositivo de conmutación al nodo   
  TareaLab1.addDevice(my_switch1);
  TareaLab1.addDevice(my_switch2);
  TareaLab1.addDevice(Temperatura);
  TareaLab1.addDevice(Humedad);

  //------------------------------------------------------------------------------
  //Opcional
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
  //------------------------------------------------------------------------------
}

/****************************************************************************************************
 * Función cíclica
*****************************************************************************************************/
void loop()
{
  if(conectado_a_wifi){
    Enviar_TyH();
  }
  //------------------------------------------------------------------------------
  // Leer GPIO0, pulsador externo para reiniciar
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
      Serial.printf("Restauración de fábrica.\n");
      RMakerFactoryReset(2);
      conectado_a_wifi = 0;
    } 
    //_______________________________________________________________________
    else if ((tiempoFinal - tiempoInicial) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      // Si está presionado RESET por más de 3 segundos pero menos de 10 segundos, reinicia Wi-Fi
      RMakerWiFiReset(2);
      conectado_a_wifi = 0;
    }
    //_______________________________________________________________________
  }
  //------------------------------------------------------------------------------
  delay(100);
  
  if (WiFi.status() != WL_CONNECTED){
    //Serial.println("WiFi No Conectado");
    digitalWrite(WIFI_LED, LOW);
  }
  else{
    //Serial.println("WiFi Conectado");
    digitalWrite(WIFI_LED, HIGH);
  }
}

/****************************************************************************************************
 * Enviar datos de temperatura y humedad
*****************************************************************************************************/
void Enviar_TyH(){
  //float temp = dht.readTemperature();
  //float hum = dht.readHumidity();
  float temp = random(20,45);
  float hum = random(0,100);
  Serial.print("Temperatura: ");Serial.print(temp);Serial.print("°C -- ");
  Serial.print("Humedad: ");Serial.print(hum);Serial.println("RH");
  
  //Enviar datos
  Temperatura.updateAndReportParam("Temperatura", temp);
  Humedad.updateAndReportParam("Humedad", hum);
  if (temp > setPoint)
    {
      //se activa ventilador
      digitalWrite(PIN_Ventilador, HIGH);
      //se actualizan los parametros
      Ventilador.updateAndReportParam("Power", HIGH);
    }
  else
    {
      //desactiva el ventilador
      digitalWrite(PIN_Ventilador, LOW);
      //actualización del parametro
      Ventilador.updateAndReportParam("Power", LOW);
  }
  
}