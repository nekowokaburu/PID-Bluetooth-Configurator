
/*
 Name:		PIDBluetoothConfig.ino
 Created:	12/20/2021
 Author:	Nicolas Vierl


 Always send back after received to update parameters in app!

 TODO:
 - protect P,I,D,SETPOINT against overflow and keep in range 0..254
 - Would be nice to have this as double read from EEPROM
 - parsing in app on getting u, also vars are not sent for some reason
*/

#include <EEPROM.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>

#define DEBUG 1

// PINS
constexpr const int BOOILER_TEMP_PIN = 10; // CS/SS
constexpr const int BOILER_SSR_PIN = 2;

// Global variables
double KP       = 0;
double KI       = 0;
double KD       = 0;
double SETPOINT = 0;

double CURRENT_TEMP = 0.0;


// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(BOOILER_TEMP_PIN);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0


constexpr const int WINDOW_SIZE = 5000;
unsigned long WINDOW_START_TIME;
double PID_OUTPUT = 0;

PID myPID(&CURRENT_TEMP, &PID_OUTPUT, &SETPOINT, KP, KI, KD, DIRECT);


void setup()
{
    while (!Serial);  // wait until serial ready
    Serial.begin(115200);
	if (DEBUG)
	{
		Serial.println("Started Serial!");
	}

    thermo.begin(MAX31865_2WIRE);  // set to 3WIRE or 4WIRE as necessary

    // Set pin modes for SSRs and pressure sensors
    pinMode(BOILER_SSR_PIN, OUTPUT);
    digitalWrite(BOILER_SSR_PIN, HIGH);  // Enable boiler heater when machine is turned on (Arduino running)

    // Load presure values from EEPROM
    KP       = static_cast<double>(EEPROM.read(0));
    KI       = static_cast<double>(EEPROM.read(1));
    KD       = static_cast<double>(EEPROM.read(2));
    SETPOINT = static_cast<double>(EEPROM.read(3));

    WINDOW_START_TIME = millis();

    //tell the PID to range between 0 and the full window size
    myPID.SetOutputLimits(0, WINDOW_SIZE);

    //turn the PID on
    myPID.SetMode(AUTOMATIC);
}

void loop()
{
	// For better message check response. Maybe lower to 30 for better PID?
    delay(50);

	// Create an empty string to store messages from Android. Do this in loop so nothing happens until new message received because it is reset here!
	String received_message;

    while (Serial.available())
    {                                             // keep reading bytes while they are still more in the buffer
        received_message += (char)Serial.read();  // read byte, convert to char, and append it to string
    }

    if (received_message.length())
    {  // if string is not empty do the following
		if(DEBUG)
		{
			Serial.print("Received Message: ");
			Serial.println(received_message);
		}
        /// Save values to eeprom
        if (received_message.startsWith("m"))
        {
            EEPROM.write(0, static_cast<uint8_t>(KP));
            EEPROM.write(1, static_cast<uint8_t>(KI));
            EEPROM.write(2, static_cast<uint8_t>(KD));
            EEPROM.write(3, static_cast<uint8_t>(SETPOINT));
			UpdateApp();
        }

        if (received_message.startsWith("p"))
        {
            KP = received_message.substring(1).toDouble();
            Serial.println(String("p") + KP);
        }

        if (received_message.startsWith("i"))
        {
            KI = received_message.substring(1).toDouble();
            Serial.println(String("i") + KI);
        }

        if (received_message.startsWith("d"))
        {
            KD = received_message.substring(1).toDouble();
            Serial.println(String("d") + KD);
        }

        if (received_message.startsWith("s"))
        {
            SETPOINT = received_message.substring(1).toDouble();
            Serial.println(String("s") + SETPOINT);
        }

        /// On connection, update App with last used pressure values from eeprom
        if (received_message.startsWith("u"))
        {
			UpdateApp();
        }
    }

    UpdateTemperature();
    Boiler();
}

//************************** Global functions *********************************************************************

void UpdateTemperature()
{
    // Read PT1000 here
    CURRENT_TEMP = thermo.temperature(RNOMINAL, RREF);
    Serial.println(String("c") + CURRENT_TEMP);
    // Serial.println(String("c") + CURRENT_TEMP++);
}

// Run boiler heater
void Boiler(void)
{
    myPID.Compute();
    if (millis() - WINDOW_START_TIME > WINDOW_SIZE)
    {
        WINDOW_START_TIME += WINDOW_SIZE;
    }

    if (PID_OUTPUT < millis() - WINDOW_START_TIME)
    {
        digitalWrite(BOILER_SSR_PIN, HIGH);
        Serial.println("o1");
    }
    else
    {
        digitalWrite(BOILER_SSR_PIN, LOW);
        Serial.println("o0");
    }
}

// Update App parameteres on startup or after parameter change
void UpdateApp(void)
{
	// Serial.println(String("p") + KP + "i" + KI + "d" + KD + "s" + SETPOINT);
	Serial.println(String("p") + KP);
	Serial.println(String("i") + KI);
	Serial.println(String("d") + KD);
	Serial.println(String("s") + SETPOINT);
}
