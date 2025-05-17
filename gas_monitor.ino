#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <HTTPClient.h>

// WiFi Credentials
const char* ssid = "Realme";  // Replace with your Wi-Fi SSID
const char* password = "1234567890";  // Replace with your Wi-Fi password

// Pin Connections
// temp - 18
// mq4 - 32
// mq6 -34
// mq7 - 35

// Temperature Sensor
// Dallas DS18B20
#define TEMP_SENSOR_PIN 18
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature DS18B20(&oneWire);
float tempC; // Temperature in Celsius
float tempF; // Temperature in Fahrenheit

// MQ-4 Sensor
int mq4_sensorPin = 32;
int mq4_sensorData;

// MQ-6 Sensor
byte MQ6_Pin = 34;          /* Define A0 for MQ Sensor Pin */
float Referance_V = 3300.0; /* ESP32 Referance Voltage in mV */
float RL = 1.0;             /* In Module RL value is 1k Ohm */
float Ro = 10.0;            /* The Ro value is 10k Ohm */
float mVolt = 0.0;
const float Ro_clean_air_factor = 10.0;
int mq6_sensorData;

// MQ-7 Sensor
int mq7_sensorPin = 35;
int mq7_sensorData;


// Global variable for Rs (to make it accessible across functions)
float Rs = 0.0;  // Declare Rs globally so it can be accessed anywhere in the code

void setup() {
  Serial.begin(9600);  // Initialize serial monitor
  DS18B20.begin();     // Initialize the DS18B20 sensor

  // Connect to Wi-Fi
  WiFi.begin(ssid, password); // Connect to the Wi-Fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  // Print ESP32 IP address

  // Initialize MQ-6 Sensor
  pinMode(MQ6_Pin, INPUT);  /* Define A0 as a INPUT Pin */
  delay(500);
  Serial.println("Wait for 30 sec warmup");
  delay(30000);             /* Set the warmup delay wich is 30 Sec */
  Serial.println("Warmup Complete");

  for(int i=0; i<30; i++){
    mVolt += Get_mVolt(MQ6_Pin);
  }
  mVolt = mVolt /30.0;      /* Get the volatage in mV for 30 Samples */
  Ro = Calculate_Rs(mVolt) / Ro_clean_air_factor;
  mVolt = 0.0;

  // Initialize MQ-7 Sensor
  pinMode(mq7_sensorPin, INPUT);

  // Initialize MQ-4 Sensor
  pinMode(mq4_sensorPin, INPUT);     
}

void loop() {
  Temp_Sensor();
  MQ4_Sensor();
  MQ6_Sensor();
  MQ7_Sensor();
  

  // Send data to local server
  sendDataToServer(mq4_sensorData, mq7_sensorData, LPG_PPM(Rs / Ro));

  Serial.println(" ");
  delay(1000);
}

void Temp_Sensor(void) {
  DS18B20.requestTemperatures();  // Request temperature from DS18B20
  tempC = DS18B20.getTempCByIndex(0);  // Read temperature in Celsius
  tempF = tempC * 9 / 5 + 32;  // Convert Celsius to Fahrenheit

  Serial.print("Temperature: ");
  Serial.print(tempC);  // Print temperature in Celsius
  Serial.println("°C");
}

//----------------------------------------------------------------------

// MQ6 Function
void MQ6_Sensor(void) {
  for(int i=0; i<500; i++){
    mVolt += Get_mVolt(MQ6_Pin);
  }
  mVolt = mVolt/500.0;      /* Get the volatage in mV for 500 Samples */
  // Serial.print("Voltage at A0 Pin = ");
  // Serial.print(mVolt);      /* Print the mV in Serial Monitor */
  // Serial.println(" mV");

  float Rs = Calculate_Rs(mVolt);
  // Serial.print("Rs = ");
  // Serial.println(Rs);       /* Print the Rs value in Serial Monitor */
  float Ratio_RsRo = Rs/Ro;

  // Serial.print("RsRo = ");
  // Serial.println(Ratio_RsRo);

  Serial.print("MQ6 Sensor Value: ");
  unsigned int LPG_ppm = LPG_PPM(Ratio_RsRo);
  mq6_sensorData=LPG_ppm;
  Serial.print(LPG_ppm);   /* Print the Gas PPM value in Serial Monitor */
  Serial.print(" PPM");
  Serial.println("");
  mVolt = 0.0;              /* Set the mVolt variable to 0 */
}

float Calculate_Rs(float Vo) {
/* 
 *  Calculate the Rs value
 *  The equation Rs = (Vc - Vo)*(RL/Vo)
 */
  float Rs = (Referance_V - Vo) * (RL / Vo); 
  return Rs;
}


unsigned int LPG_PPM(float RsRo_ratio) {
/*
 * Calculate the PPM using below equation
 * LPG ppm = [(Rs/Ro)/18.446]^(1/-0.421)
 */
  float ppm;
  ppm = pow((RsRo_ratio/18.446), (1/-0.421));
  return (unsigned int) ppm;
}


float Get_mVolt(byte AnalogPin) {
/* Calculate the ADC Voltage using below equation
 *  mVolt = ADC_Count * (ADC_Referance_Voltage / ADC_Resolution)
 */
  int ADC_Value = analogRead(AnalogPin); 
  delay(1);
  float mVolt = ADC_Value * (Referance_V / 4096.0);
  return mVolt;
}

// ----------------------------------------------------------------------

void MQ7_Sensor(void) {
  mq7_sensorData = analogRead(mq7_sensorPin);       
  Serial.print("MQ7 Sensor Value : ");
  Serial.println(mq7_sensorData);
}

void MQ4_Sensor(void) {
  mq4_sensorData = analogRead(mq4_sensorPin);       
  Serial.print("MQ4 Sensor value: ");
  Serial.println(mq4_sensorData);
}

// Function to send data to the server
void sendDataToServer(int mq4Value, int mq7Value, unsigned int lpgValue) {
  Serial.print("WiFi Status: ");
  Serial.println(WiFi.status());
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    // Construct the request body (JSON format)
    // String requestBody = "{\"mq4\":" + String(mq4Value) + ",\"mq6\":" + String(lpgValue) + ",\"mq7\":" + String(mq7Value) +  ",\"Temperature\":" + String(tempC) + "}";
  String requestBody = "{\"username\":\"yuvan\",\"temp\":" + String(tempC) + ",\"mq4\":" + String(mq4Value) + ",\"mq6\":" + String(mq6_sensorData) + ",\"mq7\":" + String(mq7Value) + "}";


    // Use the correct IP address of the machine running the server
    String serverURL = "http://192.168.87.95:3500/gasmonitor";  // Replace with your server's IP address
    http.begin(serverURL);  // Initialize HTTP request
    http.addHeader("Content-Type", "application/json");  // Specify content type as JSON

    // Send POST request
    int httpResponseCode = http.POST(requestBody);

    if (httpResponseCode > 0) {
      String response = http.getString();  // Get the response
      Serial.println("HTTP Response Code: " + String(httpResponseCode));
      Serial.println("Response: " + response);
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }

    http.end();  // Close the HTTP connection
  } else {
    Serial.println("WiFi Disconnected");
  }
}