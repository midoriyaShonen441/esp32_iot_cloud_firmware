/**
 * @file Plant_Controller.h
 * @author Wasin Wongkum (kao.wongkum@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-02-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#define RUNNING_CORE 1 

#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define SerialRS485           Serial2      

#define I2C_SDA               4
#define I2C_SCL               5

#define I2C1_SDA              21
#define I2C1_SCL              22

#define LED_BUILTIN           16

#define MCP23017_ADDR         0x20

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

uint16_t InputRegister[13];

const char json_format[] = "{\"uid\":\"\",\"timestamp\":\"\",\"sensorData\":{\"temperatureSensor\":[{\"sensorID\":\"\",\"positionID\":\"\",\"value\":0,\"status\":-1}],\"humiditySensor\":[{\"sensorID\":\"\",\"positionID\":\"\",\"value\":0,\"status\":-1}],\"lightSensor\":[{\"sensorID\":\"\",\"positionID\":\"\",\"value\":0,\"status\":-1}]},\"actuatorData\":{\"solenoid\":[{\"actuatorID\":\"\",\"positionID\":\"\",\"isOn\":false,\"status\":-1}],\"fan\":[{\"actuatorID\":\"\",\"positionID\":\"\",\"isOn\":false,\"status\":-1}],\"pump\":[{\"actuatorID\":\"\",\"positionID\":\"\",\"isOn\":false,\"status\":-1}]}}";

WiFiClient espClient;
PubSubClient mqtt(espClient);

long lastMsg = 0;
char msg[50];
int value = 0;

StaticJsonDocument<10240> doc;
StaticJsonDocument<1024> mqtt_doc;

Modbus master(0, SerialRS485);                                                        // RS485 Modbus
modbus_t telegram[1];                                                                 // 2-Modbus Frame Service

uint8_t wdt_status = 0;
SemaphoreHandle_t  espClient_xMutex;

uint8_t Relay_status;
uint8_t Input_status;
float LED_Power[16] = { 0 };
uint16_t sensor_sht31_temperature_signed_value;
uint16_t sensor_sht31_humidity_signed_value;
uint16_t sensor_bh1750_light_signed_value;

const char* ntpServer = "asia.pool.ntp.org";
const long  gmtOffset_sec = 25200;
const int   daylightOffset_sec = 0;

void Controller_handle(void* pvParameters);
void MCP23017_handle(void* pvParameters);
void PCA9685_handle(void* pvParameters);
void Modbus_handle(void* pvParameters);
void MQTT_handle(void* pvParameters);
void HTTP_handle(void* pvParameters);
void WDT_handle(void* pvParameters);
void setup_wifi();
void callback(char* topic, byte* message, unsigned int length);
void reconnect();


void Modbus_handle(void* pvParameters) {
    (void)pvParameters;
    vTaskDelay(1000);

    telegram[0].u8id = 247;                                                                         // Slave Address
    telegram[0].u8fct = 4;                                                                          // Function 0x04(Read Input Register)
    telegram[0].u16RegAdd = 0;                                                                      // Start Address Read(0x0000)
    telegram[0].u16CoilsNo = 13;                                                                    // Number of Register to Read(13 Input Register)
    telegram[0].au16reg = InputRegister;

    SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
    while (!SerialRS485);
    master.begin(SerialRS485);                                                                      // Mosbus Interface
    master.setTimeOut(2000);                                                                        // if there is no answer in 2000 ms, roll over

    Serial.println("Modbus_handle start");
    TickType_t PreviousWakeTime = xTaskGetTickCount();

    for (;;)
    {
        xTaskDelayUntil(&PreviousWakeTime, 5000);
        master.query(telegram[0]);
        vTaskDelay(100);

        if (master.poll())
        {

            sensor_sht31_temperature_signed_value = InputRegister[1];
            sensor_sht31_humidity_signed_value = InputRegister[2];
            sensor_bh1750_light_signed_value = InputRegister[4];

            u_var.u_int16[1] = InputRegister[5];
            u_var.u_int16[0] = InputRegister[6];
            sensor_sht31_temperature_float_value = u_var.u_float;

            u_var.u_int16[1] = InputRegister[7];
            u_var.u_int16[0] = InputRegister[8];
            sensor_sht31_humidity_float_value = u_var.u_float;

            u_var.u_int16[1] = InputRegister[11];
            u_var.u_int16[0] = InputRegister[12];
            sensor_bh1750_light_float_value = u_var.u_float;

        }
    }
}

void MCP23017_handle(void* pvParameters) {
    (void)pvParameters;
    vTaskDelay(1000);


    Wire.begin(I2C_SDA, I2C_SCL);

    Wire.beginTransmission(MCP23017_ADDR);
    Wire.write(0x0A);
    Wire.write(0b00100000);
    Wire.endTransmission();

    Wire.beginTransmission(MCP23017_ADDR);
    Wire.write(0x0C);
    Wire.write(0xff);
    Wire.endTransmission();

    // set I/O pins to input
    Wire.beginTransmission(MCP23017_ADDR);
    Wire.write(0x00); // IODIRA register
    Wire.write(0xff); // set all of port A to input
    Wire.endTransmission();

    // set I/O pins to outputs
    Wire.beginTransmission(MCP23017_ADDR);
    Wire.write(0x01); // IODIRB register
    Wire.write(0x00); // set all of port B to outputs
    Wire.endTransmission();

    Serial.println("MCP23017_handle start");
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    for (;;)
    {
        xTaskDelayUntil(&PreviousWakeTime, 100);

        Wire.beginTransmission(MCP23017_ADDR);
        Wire.write(0x13); // IODIRB register
        Wire.write(Relay_status);
        Wire.endTransmission();

        Wire.beginTransmission(MCP23017_ADDR);
        Wire.write(0x12);// IODIRA register
        Wire.endTransmission();
        Wire.requestFrom(MCP23017_ADDR, (int)1);

        Input_status = Wire.read();

    }
}

void PCA9685_handle(void* pvParameters) {
    (void)pvParameters;
    vTaskDelay(1000);

    Wire1.begin(I2C1_SDA, I2C1_SCL);
    Wire1.setClock(100000);

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(1600);

    Serial.println("PCA9685_handle start");
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    for (;;)
    {
        xTaskDelayUntil(&PreviousWakeTime, 100);
        for (int i = 0; i < 16; i++)
        {
            LED_Power[i] = constrain(LED_Power[i], 0, 100);
            pwm.setPWM(i, 0, int(LED_Power[i] * 40.96f) % 4096);
        }
    }
}

void MQTT_handle(void* pvParameters) {
    (void)pvParameters;
    vTaskDelay(1000);
    EEPROM.begin(1);
    xSemaphoreTake(espClient_xMutex, portMAX_DELAY);
    setup_wifi();
    xSemaphoreGive(espClient_xMutex);
    Serial.println("MQTT_handle start");
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    for (;;)
    {
        xTaskDelayUntil(&PreviousWakeTime, 1000);

        xSemaphoreTake(espClient_xMutex, portMAX_DELAY);
        if (WiFi.status() != WL_CONNECTED) {
            setup_wifi();
        }
        if (!mqtt.connected()) {
            reconnect();
        }
        mqtt.loop();
        xSemaphoreGive(espClient_xMutex);
    }
}

void WDT_handle(void* pvParameters) {
    (void)pvParameters;
    vTaskDelay(1000);

    TickType_t PreviousWakeTime = xTaskGetTickCount();
    for (;;) // A Task shall never return or exit.
    {

        xTaskDelayUntil(&PreviousWakeTime, 1000);

        wdt_status = 0xff;
        if (wdt_status == 0xff) {
            wdt_status = 0;
            esp_task_wdt_reset();
        }

        // xSemaphoreTake(espClient_xMutex, portMAX_DELAY);
        // mqtt_doc.clear();
        // mqtt_doc["ID"] = 1; // massage id (0-99)
        // JsonArray Relay_data = mqtt_doc.createNestedArray("RELAY");
        // Relay_data.add(1);
        // Relay_data.add(2);
        // Relay_data.add(3);
        // Relay_data.add(4);
        // Relay_data.add(5);
        // Relay_data.add(6);
        // Relay_data.add(7);
        // Relay_data.add(8);
        // JsonArray LED_data = mqtt_doc.createNestedArray("LED");
        // LED_data.add(100);
        // LED_data.add(100);
        // LED_data.add(100);
        // LED_data.add(100);
        // LED_data.add(100);
        // LED_data.add(100);
        // LED_data.add(100);
        // LED_data.add(100);

        // char out[256];
        // int b = serializeJson(mqtt_doc, out);
        // Serial.print("bytes = ");
        // Serial.println(b, DEC);
        // Serial.println(out);
        // mqtt.publish(mqtt_topic, out, b);
        // xSemaphoreGive(espClient_xMutex);

    }
}

void HTTP_handle(void* pvParameters) {
    (void)pvParameters;
    do
    {
        vTaskDelay(1000);
    } while (WiFi.status() != WL_CONNECTED);

    Serial.println("HTTP_handle start");
    TickType_t PreviousWakeTime = xTaskGetTickCount();
    for (;;)
    {
        xSemaphoreTake(espClient_xMutex, portMAX_DELAY);
        if (WiFi.status() == WL_CONNECTED) {


            struct tm timeinfo;
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
            if (!getLocalTime(&timeinfo)) {
                Serial.println("Failed to obtain time");
                return;
            }
            char timeStringBuff[24] = { 0 };
            strftime(timeStringBuff, sizeof(timeStringBuff), "%FT%T", &timeinfo);
            Serial.println(timeStringBuff);
            String asString(timeStringBuff);

            DeserializationError error = deserializeJson(doc, json_format);

            if (error) {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
                return;
            }

            doc["uid"] = String(WiFi.macAddress());
            doc["timestamp"] = asString;

            doc["sensorData"]["temperatureSensor"][0]["sensorID"] = String(001);
            doc["sensorData"]["temperatureSensor"][0]["positionID"] = String(001);
            doc["sensorData"]["temperatureSensor"][0]["value"] = (float)001;
            doc["sensorData"]["temperatureSensor"][0]["status"] = (uint8_t)001;

            doc["sensorData"]["humiditySensor"][0]["sensorID"] = String(001);
            doc["sensorData"]["humiditySensor"][0]["positionID"] = String(001);
            doc["sensorData"]["humiditySensor"][0]["value"] = (float)001;
            doc["sensorData"]["humiditySensor"][0]["status"] = (uint8_t)001;

            doc["sensorData"]["lightSensor"][0]["sensorID"] = String(001);
            doc["sensorData"]["lightSensor"][0]["positionID"] = String(001);
            doc["sensorData"]["lightSensor"][0]["value"] = (float)001;
            doc["sensorData"]["lightSensor"][0]["status"] = (uint8_t)001;

            doc["actuatorData"]["solenoid"][0]["actuatorID"] = String(001);
            doc["actuatorData"]["solenoid"][0]["positionID"] = String(001);
            doc["actuatorData"]["solenoid"][0]["isOn"] = (bool)false;
            doc["actuatorData"]["solenoid"][0]["status"] = (uint8_t)001;

            doc["actuatorData"]["fan"][0]["actuatorID"] = String(001);
            doc["actuatorData"]["fan"][0]["positionID"] = String(001);
            doc["actuatorData"]["fan"][0]["isOn"] = (bool)false;
            doc["actuatorData"]["fan"][0]["status"] = (uint8_t)001;

            doc["actuatorData"]["pump"][0]["actuatorID"] = String(001);
            doc["actuatorData"]["pump"][0]["positionID"] = String(001);
            doc["actuatorData"]["pump"][0]["isOn"] = (bool)false;
            doc["actuatorData"]["pump"][0]["status"] = (uint8_t)001;


            // serializeJsonPretty(doc, Serial);

            String httpRequestData = "";
            HTTPClient http;
            http.begin(espClient, serverName);
            http.addHeader("Content-Type", "application/json");

            int httpResponseCode = http.POST(httpRequestData);

            Serial.print("httpResponseCode : ");

            if (httpResponseCode > 0) {
                Serial.println(httpResponseCode);
            }
            else {
                Serial.printf("Error occurred while sending HTTP POST \n\n");
            }
            http.end();
        }
        else {
            setup_wifi();
        }
        xSemaphoreGive(espClient_xMutex);
        xTaskDelayUntil(&PreviousWakeTime, 60000);
    }
}

void setup_wifi() {
    vTaskDelay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.disconnect();
    WiFi.begin(ssid, password);
    uint8_t wait_time = 100;
    while (WiFi.status() != WL_CONNECTED && wait_time > 0) {
        vTaskDelay(500);
        wait_time--;
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    mqtt.setServer(mqtt_broker, mqtt_port);
    mqtt.setCallback(callback);
}

void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;

    for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
    }
    Serial.println("");

    if (String(topic) == mqtt_topic) {
        mqtt_doc.clear();
        deserializeJson(mqtt_doc, message);
        uint8_t prev_id = EEPROM.read(0);
        if (prev_id != mqtt_doc["ID"]) {
            prev_id = mqtt_doc["ID"];
            EEPROM.write(0, prev_id);
            EEPROM.commit();

            Serial.print("RELAY");
            uint8_t relay[8];
            for (int i = 0; i < 8; i++)
            {
                relay[i] = mqtt_doc["RELAY"][i];
                Serial.print(" ");
                Serial.print(relay[i]);
            }
            Serial.println("");

            Serial.print("LED");
            uint8_t led[8];
            for (int i = 0; i < 8; i++)
            {
                led[i] = mqtt_doc["LED"][i];
                Serial.print(" ");
                Serial.print(led[i]);
            }
            Serial.println("");
        }

    }
}

void reconnect() {
    if (!mqtt.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (mqtt.connect(mqtt_name)) {
            // if (mqtt.connect(mqtt_name, mqtt_username, mqtt_password)) {
            Serial.println("connected");
            mqtt.subscribe(mqtt_topic);
        }
        else {
            Serial.print("failed, rc=");
            Serial.print(mqtt.state());
            Serial.println(" try again in 5 seconds");
        }
    }
}

