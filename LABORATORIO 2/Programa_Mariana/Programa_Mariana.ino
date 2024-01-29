#include "RMaker.h" //libreria para el deesarrollo de la aplicación de ESP RainMaker
#include "WiFi.h" //libreria que permite la conectidad WiFi con la tarejta ESP32
#include "WiFiProv.h" //libreria para el aprovisamiento de WiFi
#include "DHT.h" //libreria para el manejo del sensor DHT11
#include <SimpleTimer.h> //libreria para el manejo de temporizadores
#include <wifi_provisioning/manager.h> //libreria para la gestión de credenciales

//valores por defecto de las variables
#define DEFAULT_RELAY_MODE false
#define DEFAULT_Temperature 0
#define DEFAULT_Humidity 0
#define DEFAULT_SET 20

float setPoint = DEFAULT_SET;

//credenciales BLE para la conexión WiFi
const char *service_name = "JuanMCD";
const char *pop = "1234567890";

//definición de pines GPIO
static uint8_t gpio_reset = 0;
static uint8_t DHTPIN = 19;
static uint8_t relePin = 18;
static uint8_t ventilador_pin = 33;
static uint8_t LED1 = 2;
static uint8_t LED2 = 4;
static uint8_t LED3 = 15;

//definición de estados iniciales
bool led1_state = false;
bool led2_state = false;
bool led3_state = false;
bool vent_state = false;
bool wifi_connected = 0;
bool bomb_state = 0;

//definición de un objeto de tipo DHT
DHT dht(DHTPIN, DHT11);

//definición de un objeto de tipo timer
SimpleTimer Timer;

//definición de dispositivos
static TemperatureSensor temperature("Temperature");
static TemperatureSensor humidity("Humidity");
static Switch my_switch("Bombillo", &relePin);
static Switch my_switch1("ledTemp", &LED1);
static Switch my_switch2("ledHum", &LED2);
static Switch my_switch3("ledAux", &LED3);
static Switch ventilador("Vent", &ventilador_pin);

//función que permite el control del bomillo
void control_BOMBILLO(int bombillo_no, int rele_pin, boolean &estado){
  estado = !estado;
  if(estado){
    digitalWrite(rele_pin,HIGH);
  }else{
    digitalWrite(rele_pin,LOW);
  }
  String text = (estado)? "ON" : "OFF";
  Serial.println("BOMBILLO"+String(bombillo_no)+" es "+text);
} 

//función que permite el control del LED
void control_LED(int LED_no, int LED_pin, boolean &estado)
{
  estado = !estado;
  digitalWrite(LED_pin, estado);
  String text = (estado) ? "ON" : "OFF";
  Serial.println("LED" + String(LED_no) + " es " + text);
}



//evento para  la conexión WIFI
void sysProvEvent(arduino_event_t *sys_event)
{
  //switch para diferentes tipos de eventos
  switch (sys_event->event_id)
  {
  //inicio del aprovisamiento
  case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32
    Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
    printQR(service_name, pop, "ble");
#else
    Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
    printQR(service_name, pop, "softap");
#endif
    break;
    //si la conexión a la red es exitosa
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    //se imprime un mensaje de que es exitosa
    Serial.printf("\nConnected to Wi-Fi!\n");
    wifi_connected = 1;
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

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx){
  //obtiene el nombre del dispositivo
  const char *device_name = device->getDeviceName();
  //imprime el nombre del dispositivo en la consola
  Serial.println(device_name);
  //se obtiene el nombre del parametro
  const char *param_name = param->getParamName();
  
  //caso en que el dispositivo sea bombillo
  if(strcmp(device_name, "bombillo") == 0) {
    Serial.printf("Lightbulb1 = %s\n", val.val.b? "true" : "false");
    //comprueba si el parametro es power
    if(strcmp(param_name, "Power") == 0){
      //actualiza el estado del bombillo y y se llama la función que lo controla
      bomb_state = val.val.b;
      bomb_state = !bomb_state;
      control_BOMBILLO(1, relePin, bomb_state);
    }
  }
   
  //----------------------------------------------------------------------------------
  //caso en el que el dispositivo es "ledTemp"
  else if (strcmp(device_name, "ledTemp") == 0)
  {
    //compruebas si el parametro es power
    if (strcmp(param_name, "Power") == 0)
    {
      //se actualiza el estado del LED 
      Serial.printf("Received value = %s for %s - %s\n", val.val.f ? "true" : "false", device_name, param_name);
      led1_state = val.val.b;
      (led1_state == false) ? digitalWrite(LED1, LOW) : digitalWrite(LED1, HIGH);
      //se informa sobre el cambio de parametro
      param->updateAndReport(val);
    }
  }

  //----------------------------------------------------------------------------------
  //caso en el que el dispositivo es "ledHum"
  else if (strcmp(device_name, "ledHum") == 0)
  {
    //comprueba si el parametro es power
    if (strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      led2_state = val.val.b;
      (led2_state == false) ? digitalWrite(LED2, LOW) : digitalWrite(LED2, HIGH);
      //se informa sobre el cambio de parametro
      param->updateAndReport(val);
    }
  }

  else if(strcmp(device_name, "ledAux") == 0) {
    //Serial.printf("Fan = %s\n", val.val.b? "true" : "false");
    if(strcmp(param_name, "Power") == 0) {
      //int fan_on_off_value = val.val.b;
      //Serial.printf("\nReceived value = %d for %s - %s\n", fan_on_off_value, device_name, param_name);
      //param->updateAndReport(val);
    }else if (strcmp(param_name, "Level") == 0) {
      int new_setpoint = val.val.i;
      Serial.printf("\nReceived value = %d for %s - %s\n", new_setpoint, device_name, param_name);
      //led2_state = val.val.b;
      //(led2_state == false) ? digitalWrite(LED2, LOW) : digitalWrite(LED2, HIGH);
      setPoint = new_setpoint; // Actualiza la variable global setPoint con el nuevo valor del setpoint
      delay(15); // waits 15ms for the servo to reach the position
      //param->updateAndReport(val);
      }
    }

  //----------------------------------------------------------------------------------
  //caso en el que el dispositivo es "Vent"
  else if (strcmp(device_name, "Vent") == 0)
  {
    //comprueba si el parametros es power
    if (strcmp(param_name, "Power") == 0)
    {
      Serial.printf("Received value = %s for %s - %s\n", val.val.b ? "true" : "false", device_name, param_name);
      vent_state = val.val.b;
      (vent_state == false) ? digitalWrite(ventilador_pin, LOW) : digitalWrite(ventilador_pin, HIGH);
      //se informa sobre el cambio de parametro
      param->updateAndReport(val);
    }
  }

  //---------------------------------------------------------------------------------
  //caso en el que el dispositivo es "Temperature"
  else if (strcmp(device_name, "Temperature") == 0)
  {
    //comprueba si el parametro es "temperature"
    if (strcmp(param_name, "Level") == 0)
    {
      float new_setpoint = val.val.f;
      setPoint = new_setpoint; // Actualiza la variable global setPoint con el nuevo valor del setpoint
      //actualiza y reporta el parametro "SetPoint"
      //se informa sobre el cambio del parametro
      temperature.updateAndReportParam("Level", new_setpoint);
    }
  }
}

void setup()
{

  Serial.begin(115200);

  //configuracion de los pines GPIO
  pinMode(gpio_reset, INPUT);
  pinMode(relePin,OUTPUT);
  pinMode(ventilador_pin, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Estados GPIO iniciales 
  digitalWrite(relePin, bomb_state);

  // Beginning Sensor
  dht.begin();

  //declaración del nodo
  Node my_node;
  //inicializa y nombra el nodo
  my_node = RMaker.initNode("Monitoreo de ambiente y control de dispositivos");

  
  

  //creación del parametro slide para establecer la temperatura, se establece un rango entre 17 y 25
  Param level_param("Level", "custom.param.level", value(DEFAULT_SET), PROP_FLAG_READ | PROP_FLAG_WRITE);
  level_param.addBounds(value(17), value(25), value(1)); //sart_value, end_value, interval
  level_param.addUIType(ESP_RMAKER_UI_SLIDER);
  //se añade al parametro al dispositivo de temperatura
  my_switch3.addParam(level_param);

  my_switch.addCb(write_callback);
  my_switch1.addCb(write_callback);
  my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback);
  ventilador.addCb(write_callback);
  //temperature.addCb(write_callback);

 
  //se añaden todos los dispositivos al nodo
  my_node.addDevice(humidity);
  my_node.addDevice(my_switch);
  my_node.addDevice(my_switch1);
  my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);
  my_node.addDevice(ventilador);
  my_node.addDevice(temperature);

  //habilitar la actualizacion OTA para el dispositivo
  RMaker.enableOTA(OTA_USING_PARAMS);
  //habilitar el servicio de zona horaria
  RMaker.enableTZService();
  //habilita la funcionalidad del programa en ESP_Rainmaker
  RMaker.enableSchedule();

  //impreme en pantalla 
  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  // Temporizador para el envío de datos del sensor
  Timer.setInterval(3000);
  
  //manejo de eventos WiFi
  WiFi.onEvent(sysProvEvent);

//identificar el tipo del dispositivo 
#if CONFIG_IDF_TARGET_ESP32
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif
  //configura el estado del pin asociado al bombillo
  digitalWrite(relePin, bomb_state);
  //actualizar y reportar el parametro asociado a power
  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, bomb_state);
  //imprime el estado del bombillo en el puerto serial
  Serial.printf("El estado del bombillo es %s \n", bomb_state? "ON" : "OFF");
}

void loop()
{
  // si el temporizador está listo se envían los datos del sensor
  if (Timer.isReady() && wifi_connected)
  {
    Serial.println("Sending Sensor's Data");
    Send_Sensor(); // se llama a la función que envía los parámetros del sensor
    Timer.reset();
  }

  // lectura del pin GPIO0 (botón externo para reiniciar el dispositivo)
  if (digitalRead(gpio_reset) == LOW)
  { //se identifica que se presiono el boton
    Serial.printf("Reset Button Pressed!\n");
    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW)
      delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000)
    {
      //si se presiona mas de 10 segundos se resetea todo
      Serial.printf("Reset to factory.\n");
      wifi_connected = 0;
      RMakerFactoryReset(2);
    }
    else if ((endTime - startTime) > 3000)
    {
      Serial.printf("Reset Wi-Fi.\n");
      wifi_connected = 0;
      // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
      RMakerWiFiReset(2);
    }
  }
  delay(100);
}

//función que envia los valores del sensor
void Send_Sensor(){
  //lectura de la humedad
  //float h = dht.readHumidity();
  //lectura de la temperatura
  //float t = dht.readTemperature();
  float h = random(0,100);
  float t = random(18,45);
  //impresión en el puerto serial de los valores establecidos
  Serial.print("Temperature - ");
  Serial.println(t);
  Serial.print("Humidity - ");
  Serial.println(h);
  Serial.print("Set Point - ");
  Serial.println(setPoint);

  //se reporta y cambia el nuevo parametro de temperatura
  temperature.updateAndReportParam("Temperature", t);
  //se reporta y cambia el nuevo parametro de humedad
  humidity.updateAndReportParam("Temperature", h);

  //establecer acciones para el control en base a la sensorica
  if (t > setPoint)
  {
    //activa led temperatura
    digitalWrite(LED1, HIGH);
    //activa ventilador
    digitalWrite(ventilador_pin, HIGH);
    //se actualizan los parametros
    my_switch1.updateAndReportParam("Power", HIGH);
    ventilador.updateAndReportParam("Power", HIGH);
  }
  else
  {
    //desactiva led temperatura
    digitalWrite(LED1, LOW);
    //desactiva el ventilador
    digitalWrite(ventilador_pin, LOW);
    //actualización del parametro
    my_switch1.updateAndReportParam("Power", LOW);
  }

  if (h > 65)
  {
    //se activa el led de humedad
    digitalWrite(LED2, HIGH);
    my_switch2.updateAndReportParam("Power", HIGH);
  }
  else
  {
    //se desactiva el led de humedad
    digitalWrite(LED2, LOW);
    my_switch2.updateAndReportParam("Power", LOW);
  }
}