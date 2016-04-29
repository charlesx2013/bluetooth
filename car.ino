#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

const int motor1Red = A0;
const int motor1Bck = A1;
const int motor2Red = A2;
const int motor2Bck = A3;

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

// booleans for wheels
boolean moveForwards;
boolean moveBackwards;
boolean turnLeft;
boolean turnRight;

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  moveForwards = false;
  moveBackwards = false;
  turnLeft = false;
  turnRight = false;

  // put your setup code here, to run once:
  pinMode(motor1Red, OUTPUT);
  pinMode(motor1Bck, OUTPUT);
  pinMode(motor2Red, OUTPUT);
  pinMode(motor2Bck, OUTPUT);
  
  Serial.println("Setup motors");
}

void forward() {
  digitalWrite(motor1Red, HIGH);
  digitalWrite(motor1Bck, LOW);
  digitalWrite(motor2Red, HIGH);
  digitalWrite(motor2Bck, LOW);
}

void left() {
  digitalWrite(motor1Red, LOW);
  digitalWrite(motor1Bck, HIGH);
  digitalWrite(motor2Red, HIGH);
  digitalWrite(motor2Bck, LOW);
}

void right() {
  digitalWrite(motor1Red, HIGH);
  digitalWrite(motor1Bck, LOW);
  digitalWrite(motor2Red, LOW);
  digitalWrite(motor2Bck, HIGH);
}

void backward() {
  digitalWrite(motor1Red, LOW);
  digitalWrite(motor1Bck, HIGH);
  digitalWrite(motor2Red, LOW);
  digitalWrite(motor2Bck, HIGH);
}

void loop(void)
{
  if (moveForwards) {
    Serial.println("moving forwards");
    forward();
  } else if (moveBackwards) {
    Serial.println("moving backwards");
    backward();
  } else if (turnLeft) {
    Serial.println("turn left");
    left();
  } else if (turnRight) {
    Serial.println("turn right");
    right();
  }else {
    digitalWrite(motor1Red, LOW);
    digitalWrite(motor1Bck, LOW);
    digitalWrite(motor2Red, LOW);
    digitalWrite(motor2Bck, LOW);
  }
  
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0 || packetbuffer[1] != 'B') return;
    
  uint8_t buttnum = packetbuffer[2] - '0';
  boolean pressed = packetbuffer[3] - '0';
    
  Serial.print ("Button "); Serial.print(buttnum);
  switch(buttnum) {
    case 5:
      moveForwards = pressed;
      break;
    case 6:
      moveBackwards = pressed;
      break;
    case 7:
      turnLeft = pressed;
      break;
    case 8:
      turnRight = pressed;
      break;
  }
  
  if (pressed) {
    Serial.println(" pressed");
  } else {
    Serial.println(" released");
  }
}
