/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <ESP32Servo.h>
#include <Ultrasonic.h>

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

Servo myservo;
int pos = 0;    // variable to store the servo position
// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 
int servoPin = 13;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// LEFT DC MOTOR
#define PIN_IN1  25 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PIN_IN2  17 // ESP32 pin GIOP26 connected to the IN2 pin L298N
#define PIN_ENA  26 // ESP32 pin GIOP14 connected to the EN1 pin L298N
// RIGHT DC MOTOR
#define PIN_IN3  16 // ESP32 pin GIOP27 connected to the IN1 pin L298N
#define PIN_IN4  27 // ESP32 pin GIOP26 connected to the IN2 pin L298N
#define PIN_ENB  14 // ESP32 pin GIOP14 connected to the EN1 pin L298N

//Ultrasonic
#define PIN_TRIGGER   18 //Verde
#define PIN_ECHO      19 //Azul
#define INTERVALO_LEITURA 250 //(ms)
Ultrasonic ultrasonic(PIN_TRIGGER, PIN_ECHO);

//INFRA_RED
#define pinoSensor 23

char command;
bool x = 0;

bool obstacleLeft = false;
bool obstacleRight = false;

class ServerCB: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
      Serial.println(" *** Client Connected");
      deviceConnected = true;
      BLEDevice::startAdvertising();
    }

  void onDisconnect(BLEServer* pServer) {
    Serial.println(" *** Client Disconnected");
    deviceConnected = false;
  }
};

class CharacteristicCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string str = pCharacteristic->getValue();
    if(str.length() > 0) {
      command = str[0];
      Serial.print("Callback onWrite: ");
      Serial.println(str.c_str());
    }
  }
};

void setup() {
  // initialize digital pins as outputs.
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENA, OUTPUT);

  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  pinMode(pinoSensor, INPUT_PULLUP); // Infravermelho

  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("BLE Car");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                        BLECharacteristic::PROPERTY_INDICATE
                                      );
                          
  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setCallbacks(new CharacteristicCB());
  //pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  	// Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 500, 2400); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
  myservo.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:p-
  // delay(2000);
  myservo.write(90);
  if (ultrasonic.read(CM) < 30) {
    parar(1000); //Leitura do ultra frontal (90 graus)

    for (pos = 90; pos <= 180; pos += 1) // Infrared looks left
    {
		  myservo.write(pos);    
		  delay(15);
      if (180 <= pos >= 175 && digitalRead(pinoSensor) == LOW) {
        obstacleLeft = true;      
      }
    }

    myservo.write(90);
    for (pos = 90; pos >= 0; pos -= 1) // Infrared looks right
    {
      myservo.write(pos);
      delay(15);
      if(0 >= pos <= 10 && digitalRead(pinoSensor) == LOW) {
        obstacleRight = true;
      }
    }

    if(obstacleLeft && !obstacleRight){ //turn right if no obstacle on left side
      re(500, 150);
      direita(1500, 200);
      obstacleLeft = false;
    } else if (obstacleRight && !obstacleLeft) {
      re(500, 150);
      esquerda(1500, 200); // turn left
      obstacleRight = false;
    }

  } else frente(1, 200);
  
  if (deviceConnected) {
    Serial.println("Car BLE connected!");
    myservo.write(90);
    while( x == 0 ) {
      switch(command)
      {
        case 'f': { frente(1, 200); break; }
        case 'd': { direita(1, 200); break; }
        case 'i': { frenteDireita(1, 200); break; }
        case 'e': { esquerda(1, 200); break; }
        case 'g': { frenteEsquerda(1, 200); break; }
        case 'p': { parar(1); break; }
        case 'r': { re(1, 200); break; }
        case 'h': { reEsquerda(1, 200); break; }
        case 'j': { reDireita(1, 200); break; }
        case 'x': { parar(1000); deviceConnected = false; x = 1; break; }
        default: { parar(1); break; }
      }
    }
  }
  if (command == 'x') x = 0; // Key to reenter on BLE mode
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}

void frente(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  //Motor Direito
  analogWrite(PIN_ENB, vel);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  delay(tempo);
}

void frenteDireita(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  //Motor Direito
  analogWrite(PIN_ENB, vel/2);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  delay(tempo);
}

void frenteEsquerda(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel/2);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  //Motor Direito
  analogWrite(PIN_ENB, vel);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  delay(tempo);
}

void parar(int tempo) {
  /* Trava Motor - Para parar numa ladeira tem que colocar HIGH HIGH como se fosse freio de mão */
  //Motor Esquerdo
  analogWrite(PIN_ENA, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  //Motor Direito
  analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  delay(tempo);
}

void re(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);

  //Motor Direito
  analogWrite(PIN_ENB, vel);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  delay(tempo);
}

void reEsquerda(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  //Motor Direito
  analogWrite(PIN_ENB, vel);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  delay(tempo);
}

void reDireita(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);

  //Motor Direito
  analogWrite(PIN_ENB, vel);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  delay(tempo);
}

void direita(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel);
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);

  //Motor Direito
  analogWrite(PIN_ENB, vel);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  delay(tempo);
}

void esquerda(int tempo, byte vel) {
  //Motor Esquerdo
  analogWrite(PIN_ENA, vel);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);

  //Motor Direito
  analogWrite(PIN_ENB, vel);
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  delay(tempo);
}

