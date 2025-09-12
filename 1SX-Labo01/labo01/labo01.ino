#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define TRIG_PIN 12
#define ECHO_PIN 11

unsigned long currentTime;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 non trouvé..."));
    for (;;)
      ;
  }
}

void loop() {
  currentTime = millis();
  int distance = distanceTask(currentTime);

  printDistanceTask(currentTime, distance);
  display.clearDisplay();
  screenTask();
  sunTask(distance);
  display.display();
}

int distanceTask(unsigned long ct) {
  static unsigned long lastTime = 0;
  unsigned long rate = 50;
  static int lastResult = -1;
  int result = -1;

  long duration;
  int distance;

  if (ct - lastTime < rate) {
    return lastResult;
  }

  lastTime = ct;

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Activer le trigPin 10 microsecondes
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Lire l'écho
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculer la distance
  distance = duration * 0.034 / 2;  // Vitesse du son / 2

  if (distance >= 2 && distance <= 400) {
    result = distance;
    lastResult = result;
  }

  return lastResult;
}

void printDistanceTask(unsigned long ct, int distance) {
  static unsigned long lastTime = 0;
  unsigned long rate = 500;

  if (ct - lastTime < rate) return;
  lastTime = ct;

  if (distance >= 2 && distance <= 400) {
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("Hors de portée");
  }
}

void screenTask() {
  display.drawRect(6, 10, 30, 20, SSD1306_WHITE);             // Mur
  display.drawRect(28, 2, 4, 4, SSD1306_WHITE);               // Cheminée
  display.drawTriangle(1, 10, 40, 10, 20, 1, SSD1306_WHITE);  // Toit
  display.drawRect(12, 20, 6, 10, SSD1306_WHITE);             // Porte

  display.drawRect(23, 15, 8, 8, SSD1306_WHITE);  // Fenêtre
  display.drawLine(24, 18, 30, 18, SSD1306_WHITE);
  display.drawLine(27, 15, 27, 22, SSD1306_WHITE);

  display.setTextSize(2);
  display.setTextColor(SSD1306_INVERSE);
  display.setCursor(1, 50);
  display.println("Brim");
}

void sunTask(int distance) {
  int sunY;
  int cercle = 360;
  int rayon1 = 8;
  int rayon2 = 10;
  int demiCercle = cercle / 2;
  int distanceMin = 10;
  int distanceMax = 50;
  int distanceGauche = 110;

  if (distance >= distanceMin && distance <= distanceMax) {
    sunY = map(distance, distanceMin, distanceMax, distanceMax, distanceMin);
  } else if (distance < distanceMin) {
    sunY = distanceMax;
  } else {
    sunY = distanceMin;
  }

  // Cercle
  display.fillCircle(distanceGauche, sunY, rayon1, SSD1306_WHITE);

  // Rayons
  for (int i = 0; i < cercle; i += 45) {
    int x1 = distanceGauche + cos(i * PI / demiCercle) * rayon2;
    int y1 = sunY + sin(i * PI / demiCercle) * rayon2;
    int x2 = distanceGauche + cos(i * PI / demiCercle) * 14;
    int y2 = sunY + sin(i * PI / demiCercle) * 14;
    display.drawLine(x1, y1, x2, y2, SSD1306_WHITE);
  }
}
