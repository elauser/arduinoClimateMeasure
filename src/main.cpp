#include <Arduino.h>
#include <LiquidCrystal.h>
#include <DHT.h>

#define DHTPIN 22
#define DHTTYPE DHT11

#define HC_ECHO 8
#define HC_TRIG 9

#define BUTTON_PIN 24

DHT dht(DHTPIN, DHTTYPE);

enum Modes
{
  TEMP_HUMIDITY,
  DISTANCE,
  END,
};

struct Mode
{
  void nextMode()
  {
    if (mode + 1 == Modes::END)
      mode = static_cast<Modes>(0);
    else
      mode = static_cast<Modes>(mode + 1);
  }
  Modes getMode()
  {
    return mode;
  }

private:
  Modes mode = static_cast<Modes>(0);
};

struct DistanceSensor
{
  static float getDistance()
  {
    float distances[10];
    noInterrupts();
    for (int i = 0; i < 10; i++)
    {
      float pulseTime = 0;
      float distance_in_cm = 0;

      digitalWrite(HC_TRIG, LOW);
      delayMicroseconds(3);

      digitalWrite(HC_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(HC_TRIG, LOW);

      pulseTime = pulseIn(HC_ECHO, HIGH);
      distance_in_cm = (pulseTime / 2) / 29.1;

      distances[i] = distance_in_cm;
    }
    interrupts();

    float sum = 0;
    for (float distance : distances)
    {
      sum += distance;
    }

    float avg_distances = sum / 10;
    return avg_distances;
  }
};

struct Sensor
{
  Sensor()
  {
    lcd.begin(16, 2);
    displayMeasurement();
  }
  void nextMode()
  {
    mode.nextMode();
  }
  void displayMeasurement()
  {
    switch (mode.getMode())
    {
    case TEMP_HUMIDITY:
      displayHumidityTemperature();
      break;
    case DISTANCE:
      displayDistance();
      break;
    default:
      displayError();
      break;
    }
  }

private:
  Mode mode{};
  const uint8_t rs = 12, en = 11, d4 = 45, d5 = 44, d6 = 43, d7 = 42;
  LiquidCrystal lcd{rs, en, d4, d5, d6, d7};

  void displayMode()
  {
    lcd.print(String("Mode: ") + String(mode.getMode()));
  }
  void displayHumidityTemperature()
  {
    lcd.clear();
    displayMode();

    lcd.setCursor(0, 1);
    lcd.print(String("H:") + String(dht.readHumidity()) + String("  T:") + String(dht.readTemperature()));
  }
  void displayDistance()
  {
    lcd.clear();
    displayMode();

    lcd.setCursor(0, 1);
    lcd.print(String("Distance:") + String(DistanceSensor::getDistance()));
  }
  void displayError()
  {
    lcd.clear();
    displayMode();

    lcd.setCursor(0, 1);
    lcd.print(String("Mode unavailable"));
  }
};

int buttonTriggered()
{
  if (digitalRead(BUTTON_PIN))
  {
    while (digitalRead(BUTTON_PIN))
    {
      delay(50);
    }
    return 1;
  }
  else
  {
    return 0;
  }
}

int red_light_pin = 6;
int green_light_pin = 5;
int blue_light_pin = 3;

int sensor_red = A0;
int sensor_green = A1;
int sensor_blue = A2;

void rgb_color(int red_light_value, int green_light_value, int blue_light_value)
{
  analogWrite(red_light_pin, constrain(map(red_light_value, 600, 1023, 0, 255), 0, 255));
  analogWrite(green_light_pin, constrain(map(green_light_value, 600, 1023, 0, 255), 0, 255));
  analogWrite(blue_light_pin, constrain(map(blue_light_value, 600, 1023, 0, 255), 0, 255));
}

int buzz_pin = 30;
void buzz(int duration_miliseconds)
{
  int wave_time = 2000;
  int rotations = duration_miliseconds / (wave_time / 1000);
  for (int i = 0; i < rotations; i++)
  {
    digitalWrite(buzz_pin, HIGH);
    delayMicroseconds(wave_time / 2);
    digitalWrite(buzz_pin, LOW);
    delayMicroseconds(wave_time / 2);
  }
}

void setup()
{
  pinMode(BUTTON_PIN, INPUT);
  pinMode(HC_TRIG, OUTPUT);
  pinMode(HC_ECHO, INPUT);
  digitalWrite(HC_TRIG, HIGH);
  dht.begin();

  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);

  pinMode(buzz_pin, OUTPUT);
  Serial.begin(9600);
}

Sensor sensor{};
void loop()
{
  sensor.displayMeasurement();
  if (buttonTriggered())
  {
    sensor.nextMode();
  }

  rgb_color(analogRead(sensor_red),
            analogRead(sensor_green),
            analogRead(sensor_blue)); // Blue

  delay(500);
}