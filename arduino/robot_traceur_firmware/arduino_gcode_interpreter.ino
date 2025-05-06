#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Définir les moteurs
#define MotorInterfaceType 1
#define motorXStepPin 2
#define motorXDirPin 5
#define motorZStepPin 3  // Connecté au motorYStepPin, mais on le renomme pour clarifier
#define motorZDirPin 6   // Connecté au motorYDirPin, mais on le renomme pour clarifier
#define ENABLE_PIN 8
// Servo pour le stylo
#define servoPin 11 // lié à SPN EN de CNC Shield

// Broche pour la LED ou le buzzer d'erreur
#define errorPin 13

// Configurer l'écran LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

AccelStepper motorX(MotorInterfaceType, motorXStepPin, motorXDirPin);
AccelStepper motorZ(MotorInterfaceType, motorZStepPin, motorZDirPin);  // Renommé de motorY à motorZ
Servo penServo;

// Paramètres des moteurs
float motorXNormalSpeed = 500; 
float motorZNormalSpeed = 1000;  // Renommé
float motorXFastSpeed = 1000; // Vitesse rapide
float motorZFastSpeed = 3000; // Vitesse rapide, renommé
float motorXAcceleration = 1000;
float motorZAcceleration = 3000;  // Renommé

float currentFeedrate = 1000; // Feedrate initiale en mm/min à régler

// **Étapes par millimètre**
float stepsPerMMX = 5.0; // Steps par millimètre pour l'axe X
float stepsPerMMZ = 5.0; // Steps par millimètre pour l'axe Z (était Y)

// Limites physiques de la zone de travail
float maxX = 210; // Limite maximale pour X (en mm)
float maxZ = 297; // Limite maximale pour Z (en mm), modifié pour A4

// Position actuelle du stylo (en millimètres)
float currentX = 0;
float currentZ = 0;  // Renommé de currentY à currentZ
bool penDown = false;

// Variables pour gérer le mouvement non bloquant du stylo
unsigned long penMoveStartTime = 0;
bool isPenMoving = false;
int targetPenAngle = 0;

// Délai entre deux mouvements de moteur en millisecondes
int movementDelay = 100;

// Fonction pour lever le stylo
void liftPen() {
  targetPenAngle = 90;
  penServo.write(targetPenAngle);
  penMoveStartTime = millis();
  isPenMoving = true;
  penDown = false;
  // Envoie confirmation au PC
  Serial.println("ok - pen up");
  delay(movementDelay); // Attendre que le stylo se lève complètement
}

// Fonction pour abaisser le stylo
void lowerPen() {
  targetPenAngle = 0;
  penServo.write(targetPenAngle);
  penMoveStartTime = millis();
  isPenMoving = true;
  penDown = true;
  // Envoie confirmation au PC
  Serial.println("ok - pen down");
  delay(movementDelay); // Attendre que le stylo s'abaisse complètement
}

// Fonction pour surveiller et terminer les mouvements du stylo
void updatePenMovement() {
  if (isPenMoving && millis() - penMoveStartTime >= 100) {
    isPenMoving = false;
  }
}

// Vérification des limites
bool isWithinBounds(float xTarget, float zTarget) {
  return (xTarget >= 0 && xTarget <= maxX && zTarget >= 0 && zTarget <= maxZ);
}

// Fonction pour signaler une erreur de manière non bloquante
void signalError() {
  static unsigned long previousMillis = 0;
  static bool errorState = false;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;
    errorState = !errorState;
    digitalWrite(errorPin, errorState ? HIGH : LOW); // Alterne la LED/buzzer
  }
}

// Fonction pour interpréter les commandes G-code
void processGcode(String line) {
  line.trim();
  Serial.println("Processing: " + line);
  
  // Ignorer les commentaires et lignes vides
  if (line.length() == 0 || line.startsWith(";")) {
    Serial.println("ok");
// Dans la fonction processGcode(), modifier la réponse pour M114:
else if (command == "M11" || command == "M114") {  // Accepter les deux commandes
    Serial.print("X:");
    Serial.print(currentX);
    Serial.print(" Y:");  // Envoyer Y au lieu de Z pour correspondre au Python
    Serial.println(currentZ);
    Serial.println("ok");
    return;
}
    return;
  }
  
  // Analyser le type de commande
  String command = line.substring(0, 2);
  
  if (command == "G0" || command == "G1") {
    // Déplacement G0 (rapide) ou G1 (avec tracé)
    bool isG0 = (command == "G0");

    // Extraire les coordonnées X et Z (utilisé comme Y dans le code Python)
    float targetX = currentX;
    float targetZ = currentZ;
    
    // Chercher X
    int xIndex = line.indexOf('X');
    if (xIndex != -1) {
      int xEndIndex = line.indexOf(' ', xIndex);
      if (xEndIndex == -1) xEndIndex = line.length();
      targetX = line.substring(xIndex + 1, xEndIndex).toFloat();
    }
    
    // Chercher Y (qui correspond à Z dans notre code Arduino)
    int yIndex = line.indexOf('Y');
    if (yIndex != -1) {
      int yEndIndex = line.indexOf(' ', yIndex);
      if (yEndIndex == -1) yEndIndex = line.length();
      targetZ = line.substring(yIndex + 1, yEndIndex).toFloat();
    }
    
    // Chercher Z pour le stylo (0 = bas, 5 = haut)
    int zIndex = line.indexOf('Z');
    if (zIndex != -1) {
      int zEndIndex = line.indexOf(' ', zIndex);
      if (zEndIndex == -1) zEndIndex = line.length();
      float zValue = line.substring(zIndex + 1, zEndIndex).toFloat();
      
      if (zValue <= 1) {
        lowerPen();
      } else {
        liftPen();
      }
      
      Serial.println("ok");
      return;
    }
    
    // Vérifier les limites
    if (!isWithinBounds(targetX, targetZ)) {
      Serial.println("erreur: Coordonnées hors limites");
      signalError();
      return;
    }
    
    // Définir la vitesse en fonction du type de commande
    if (isG0) {
      motorX.setMaxSpeed(motorXFastSpeed);
      motorZ.setMaxSpeed(motorZFastSpeed);
    } else {
      motorX.setMaxSpeed(motorXNormalSpeed);
      motorZ.setMaxSpeed(motorZNormalSpeed);
    }
    
    // Convertir les mm en pas de moteur
    long stepsX = (targetX - currentX) * stepsPerMMX;
    long stepsZ = (targetZ - currentZ) * stepsPerMMZ;
    
    // Déplacer les moteurs
    motorX.move(stepsX);
    motorZ.move(stepsZ);
    
    // Mise à jour de la position actuelle
    currentX = targetX;
    currentZ = targetZ;
    
    // Afficher la position sur l'écran LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("X: ");
    lcd.print(currentX);
    lcd.setCursor(0, 1);
    lcd.print("Z: ");
    lcd.print(currentZ);
    
    // Attendre que les moteurs finissent leurs mouvements
    while (motorX.isRunning() || motorZ.isRunning()) {
      motorX.run();
      motorZ.run();
    }
    
    Serial.println("ok");
    return;
  }
  else if (command == "G90") {
    // Mode coordonnées absolues (déjà notre mode par défaut)
    Serial.println("ok");
    return;
  }
  else if (command == "G91") {
    // Mode coordonnées relatives (non implémenté)
    Serial.println("erreur: Mode relatif non implémenté");
    return;
  }
  else if (command == "M11") {
    // Demande de position actuelle
    Serial.print("X:");
    Serial.print(currentX);
    Serial.print(" Z:");
    Serial.println(currentZ);
    Serial.println("ok");
    return;
  }
  else {
    // Commande inconnue
    Serial.println("erreur: Commande inconnue");
    return;
  }
}

void setup() {
  // Initialiser la communication série
  Serial.begin(115200);
  
  // Configurer la broche d'erreur
  pinMode(errorPin, OUTPUT);
  digitalWrite(errorPin, LOW);
  
  // Configuration des moteurs
  motorX.setMaxSpeed(motorXNormalSpeed);
  motorX.setAcceleration(motorXAcceleration);
  motorZ.setMaxSpeed(motorZNormalSpeed);  
  motorZ.setAcceleration(motorZAcceleration);
  
  // Configurer la broche ENABLE
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Activer les moteurs
  
  // Initialiser le servo pour le stylo
  penServo.attach(servoPin);
  liftPen(); // Commencer avec le stylo levé
  
  // Initialiser l'écran LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Robot Traceur");
  lcd.setCursor(0, 1);
  lcd.print("Pret");
  
  Serial.println("Robot Traceur initialisation terminée");
}

void loop() {
  // Vérifier s'il y a des données en entrée
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    processGcode(line);
  }
  
  // Mise à jour des mouvements du stylo si nécessaire
  updatePenMovement();
  
  // Exécuter les mouvements des moteurs
  motorX.run();
  motorZ.run();
}