  /*
  WiFiAccessPoint.ino creates a WiFi access point and provides a web server on it.

  Steps:
  1. Connect to the access point "yourAp"
  2. Point your web browser to http://192.168.4.1/H to turn the LED on or http://192.168.4.1/L to turn it off
     OR
     Run raw TCP "GET /H" and "GET /L" on PuTTY terminal with 192.168.4.1 as IP address and 80 as port

  Created for arduino-esp32 on 04 July, 2018
  by Elochukwu Ifediora (fedy0)
*/

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

//for bmp
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define ledPin 2   // Set the GPIO pin where you connected your test LED or comment this line out if your dev board has a built-in LED
#define feed 0  // setting the water motor pin

#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       A3


//for bmp
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP280 bmp; // I2C

//for water level
#define POWER_PIN  17 // ESP32 pin GIOP17 connected to sensor's VCC pin
#define SIGNAL_PIN A0 // ESP32 pin GIOP36 (ADC0) connected to sensor's signal pin
int waterlevel = 0; // variable to store the sensor value


// Set these to your desired credentials.
const char *ssid = "fish";
const char *password = "12345678";


WiFiServer server(80);


void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(feed, OUTPUT);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // You can remove the password parameter if you want the AP to be open.
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();

  Serial.println("Server started");


  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bmp.begin(0x76);  

  //for water level
   pinMode(POWER_PIN, OUTPUT);   // configure pin as an OUTPUT
  digitalWrite(POWER_PIN, LOW); // turn the sensor OFF

}

void loop() {
    // get the ADC value from the temperature sensor
  int adcVal = analogRead(PIN_LM35);
  // convert the ADC value to voltage in millivolt
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  // convert the voltage to the temperature in Celsius
  float tempC = milliVolt / 10;

  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> to feed Food.<br><br><br><br>");
            client.print("Click <a href=\"/L\">here</a> to stop Food.<br><br><br>");
            client.print("Temperature of Fish Pond: ");
            client.print(tempC);
            client.print("°C");
            client.print("<br><br><br>");
            

            client.print("Atmospheric Pressure: ");
            //client.print(pressP);
            client.print("Pressure = ");
            client.print(bmp.readPressure() / 100.0F);
            client.println(" hPa");
            client.print("<br><br><br>");


            client.print("Ambient Temperature = ");
            client.print(1.8 * bmp.readTemperature() + 32);
            client.println(" *F");
            client.print("<br><br><br>");

            
            client.print("Approx. Altitude = ");
            client.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
            client.println(" m");
            client.print("<br><br><br>");


            //for water level
            digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
            delay(100);                      // wait 10 milliseconds
            waterlevel = analogRead(SIGNAL_PIN); // read the analog value from sensor
            digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF

            client.print("Water Sensor value: ");
            client.println(waterlevel);

            // water level Motor Run if(waterlevel < 2500 then ) then G2 = High
            
           if(waterlevel < 2000) {
                digitalWrite(ledPin, HIGH);  }             
           if(waterlevel > 3000) {
                digitalWrite(ledPin, LOW);  }             
          
          

            
            //<h1>Sensor Value:<span id="tempC">0</span></h1><br>


            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(feed, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(feed, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}