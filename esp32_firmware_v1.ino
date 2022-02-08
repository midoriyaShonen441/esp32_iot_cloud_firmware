/**
 * @file esp32_firmware_v1.ino
 * @author Wasin Wongkum (kao.wongkum@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-02-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <EEPROM.h>
#include <ArduinoJson.h>      //! version to 6.19.1
#include <esp_task_wdt.h>

#include "time.h"
#include "PubSubClient.h"
#include "ETT_ModbusRTU.h"
#include "Adafruit_PWMServoDriver.h"


// Replace the next variables with your SSID/Password combination
// const char* ssid = "Redmi_WiFi";
// const char* password = "zxc12345";

const char* ssid = "WiFi_481972";
const char* password = "50481972";

// MQTT Broker
const char* mqtt_name = "plant_001";

const char* mqtt_broker = "test.mosquitto.org";
const char* mqtt_topic = "plant_001/command";

const char* mqtt_username = "emqx";
const char* mqtt_password = "public";
const int mqtt_port = 1883;

const char* serverName = "http://httpbin.org/anything";



float sensor_sht31_temperature_float_value;
float sensor_sht31_humidity_float_value;
float sensor_bh1750_light_float_value;


// define tasks
#include "Plant_Controller.h"

// the setup function runs once when you press reset or power the board
void setup() {

  const int  WDT_TIMEOUT_S = 600;
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  Serial.begin(115200);
  Serial.println("System start");

  espClient_xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(Controller_handle, "Controller_handle", 1024 * 32, NULL, 2, NULL, RUNNING_CORE);
  xTaskCreatePinnedToCore(PCA9685_handle, "PCA9685_handle", 1024* 16, NULL, 1, NULL, RUNNING_CORE);  //! ??
  xTaskCreatePinnedToCore(MCP23017_handle, "MCP23017_handle", 1024 * 16, NULL, 1, NULL, RUNNING_CORE);  // ok
  xTaskCreatePinnedToCore(Modbus_handle, "Modbus_handle", 1024 * 16, NULL, 1, NULL, RUNNING_CORE); // ok
  xTaskCreatePinnedToCore(MQTT_handle, "MQTT_handle", 1024 * 48, NULL, 1, NULL, RUNNING_CORE); // ok
  xTaskCreatePinnedToCore(HTTP_handle, "HTTP_handle", 1024 * 64, NULL, 1, NULL, RUNNING_CORE); // ok
  xTaskCreatePinnedToCore(WDT_handle, "WDT_handle", 1024 * 40, NULL, 1, NULL, RUNNING_CORE); // ok

}
void loop()
{

}

/*---------------------- Tasks ---------------------*/

void Controller_handle(void* pvParameters) {
  (void)pvParameters;
  vTaskDelay(1000);

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Controller_handle start");
  TickType_t PreviousWakeTime = xTaskGetTickCount();
  for (;;) // A Task shall never return or exit.
  {
    xTaskDelayUntil(&PreviousWakeTime, 100);

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay(1000);  // one tick delay (15ms) in between reads for stability

    Serial.print("Sensor SHT31 Temperature Float Value  = ");
    Serial.println(sensor_sht31_temperature_float_value);
    Serial.print("Sensor SHT31 Humidity Float Value     = ");
    Serial.println(sensor_sht31_humidity_float_value);
    Serial.print("Sensor BH1750 Light Float Value  = ");
    Serial.println(sensor_bh1750_light_float_value);
    Serial.println("");
    vTaskDelay(1000);

    Serial.print("Read Input_status  = ");
    Serial.println(Input_status, BIN);
    vTaskDelay(1000);

    Serial.println("test all leds on");
    for (int i = 0; i < 16; i++)
    {
      LED_Power[i] = 50;
    }
    vTaskDelay(5000);
    Serial.println("test all leds off");
    for (int i = 0; i < 16; i++)
    {
      LED_Power[i] = 0;
    }
    vTaskDelay(1000);

    Serial.println("test all relays on");
    Relay_status = 0xFF;
    vTaskDelay(5000);
    Serial.println("test all relays off");
    Relay_status = 0x00;
    vTaskDelay(1000);
  }
}

