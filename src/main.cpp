#include <Adafruit_BME280.h>
#include <Arduino.h>
#include <Wire.h>

#define SER Serial // Serial: USB Serial, Serial1: Hardware Serial
#define SER_BAUD (9600)
#define INTERVAL_MS (1000 * 60) // Sensor Read Interval (1m)
#define DEBUG

Adafruit_BME280 bme; // BME280 Humidity, Pressure, Temperature Sensor (via I2C)

void setup() {
  // Serial
  SER.begin(SER_BAUD);

  // LED
  pinMode(LED_BUILTIN, OUTPUT);

  // BME280
  while (!bme.begin(0x77, &Wire)) {
    delay(1000);
  } // IC2 Addr: 0x77

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF);
}

void loop() {

  digitalWrite(LED_BUILTIN, HIGH);

  bme.takeForcedMeasurement();

  float temperature = bme.readTemperature();
  float pressure = bme.readPressure() / 100.0 + 4.14 * 12;
  float humidity = bme.readHumidity();

#ifdef DEBUG
  SER.print("Temperature: ");
  SER.print(temperature);
  SER.print(" *C ");

  SER.print("Pressure: ");
  SER.print(pressure);
  SER.print(" hPa ");

  SER.print("Humidity: ");
  SER.print(humidity);
  SER.print(" %");

  SER.println();
#else
  SER.print(temperature);
  SER.print(",");
  SER.print(pressure);
  SER.print(",");
  SER.print(humidity);
  SER.print("\n");
#endif

  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  delay(INTERVAL_MS);
}
