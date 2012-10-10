#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// PINS
// Sensor Pod/Arduino - Description
//  1/5V   - Red - 5V
//  2/3.3V - 3.3V
//  3/GND  - GND
//  4/D8   - OpenLog TXO
//  5/D9   - OpenLog RXI
//  6/D3   - DS18B20 Temperature Sensors
//  7/D5   - Remove Before Flight
//  8/D2   - AM2302 Humidity
//  9/D1   - D2523T-6 GPS RX
// 10/D0   - D2523T-6 GPS TX
// 11/A0   - TTF103 Thermistor - 9820 Ohms
// 12/A1   - TTF103 Thermistor - 9850 Ohms
// 13/A2   - TTF103 Thermistor - 9800 Ohms
// 14/A3   - TTF103 Thermistor - 9810 Ohms
// 15/NA   - Empty


//  digital pin that DHT is on
#define DHT_PIN          2  // humidity
#define ONE_WIRE_BUS_PIN 3  // temperature
#define OPENLOG_RX_PIN   8  // OpenLog RX
#define OPENLOG_TX_PIN   9  // OpenLog TX
#define RBF_PIN          5  // Remove before flight

// DHT 22  (AM2302)
#define DHT_TYPE DHT22

// Temperature data wire is plugged into port 3
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance
OneWire oneWire(ONE_WIRE_BUS_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress insideThermometer, outsideThermometer;

DHT dht(DHT_PIN, DHT_TYPE);

// RX on pin 8, TX on pin 9
AltSoftSerial mySerial;

// if not on software serial, should probably be on hardware serial. :)
#define GPS_ON_SWSERIAL 0

#define DUMPDEBUG 0

TinyGPS gps;

#if GPS_ON_SWSERIAL
SoftwareSerial nss(5, 6);
#endif

// THIS CODE EXPECTS THERMISTORS TO BE ON ANALOG PINS 0 - (NUMTHERMISTORS - 1)
// WHERE N IS THE NUMBER OF THERMISTORS
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3435
// the value of the 'other' resistor
#define RESISTOR0 9840
#define RESISTOR1 9860
#define RESISTOR2 9810
#define RESISTOR3 9820

// supports up to 4
#define NUMTHERMISTORS 4
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5

int whichSample;
int thermistorSamples[NUMTHERMISTORS][NUMSAMPLES];
int thermistorResistors[NUMTHERMISTORS];
float thermistorTemperatures[NUMTHERMISTORS];

struct GPSData
{
    // number of visible satellites
    int           satCount;
    // http://en.wikipedia.org/wiki/Dilution_of_precision_(GPS)
    // div by 100 to compare against above url
    unsigned long hdop;
    // 1/100000 degree
    long          lat;
    long          lon;
    // age of fix in ms
    unsigned long fixAge;
    // hhmmsscc (where cc is 1/100 sec)
    unsigned long time;
    // ddmmyy
    unsigned long date;
    // age of date in ms
    unsigned long dateAge;
    // 1/100 knot
    unsigned long speed;
    // 1/100 degree
    unsigned long course;
    // signed altitude in centimeters
    long          alt;
} gpsData;

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        // zero pad the address if necessary
        if (deviceAddress[i] < 16) Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
}

void setup()
{
    dht.begin();
    sensors.begin();

    // set the data rate for the SoftwareSerial port
    mySerial.begin(19200);

#if GPS_ON_SWSERIAL
    Serial.begin(19200);
#else
    Serial.begin(9600);
#endif

#if GPS_ON_SWSERIAL
    // locate devices on the bus
    Serial.print("Locating devices...");
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    // report parasite power requirements
    Serial.print("Parasite power is: ");
#endif
    if (sensors.isParasitePowerMode())
    {
#if GPS_ON_SWSERIAL
        Serial.println(F("ON"));
#endif
    }
    else
    {
#if GPS_ON_SWSERIAL
        Serial.println(F("OFF"));
#endif
    }

    if (!sensors.getAddress(insideThermometer, 0))
    {
#if GPS_ON_SWSERIAL
        Serial.println(F("Unable to find address for Device 0"));
#endif
    }
    if (!sensors.getAddress(outsideThermometer, 1))
    {
#if GPS_ON_SWSERIAL
        Serial.println(F("Unable to find address for Device 1"));
#endif
    }

#if GPS_ON_SWSERIAL
    // show the addresses we found on the bus
    Serial.print(F("Device 0 (Inside) Address: "));
    printAddress(insideThermometer);
    Serial.println();

    Serial.print(F("Device 1 (Outside) Address: "));
    printAddress(outsideThermometer);
    Serial.println();

    // set the resolution to 9 bit
    sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

    Serial.print(F("Device 0 (Inside) Resolution: "));
    Serial.print(sensors.getResolution(insideThermometer), DEC);
    Serial.println();

    Serial.print(F("Device 1 (Outside) Resolution: "));
    Serial.print(sensors.getResolution(outsideThermometer), DEC);
    Serial.println();
#endif

    Serial.print(F("Front loading "));
    Serial.print(NUMTHERMISTORS);
    Serial.println(F(" thermistors"));
    for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
    {
        for (int sample = 0; sample < NUMSAMPLES; ++sample)
        {
            thermistorSamples[thermistor][sample]  = analogRead(thermistor);
        }
    }
    whichSample = 0;

#if NUMTHERMISTORS > 0
    thermistorResistors[0] = RESISTOR0;
#endif
#if NUMTHERMISTORS > 1
    thermistorResistors[1] = RESISTOR1;
#endif
#if NUMTHERMISTORS > 2
    thermistorResistors[2] = RESISTOR2;
#endif
#if NUMTHERMISTORS > 3
    thermistorResistors[3] = RESISTOR3;
#endif

    analogReference(EXTERNAL);

#if GPS_ON_SWSERIAL
    nss.begin(9600);
#endif
}

void getThermistors()
{
    for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
    {
        thermistorSamples[thermistor][whichSample] = analogRead(thermistor);
    }

#if GPS_ON_SWSERIAL && DUMPDEBUG
    Serial.print(F("Raw analog readings "));
    for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
    {
        if (thermistor < NUMTHERMISTORS - 1)
        {
            Serial.print(thermistorSamples[thermistor][whichSample]);
            Serial.print(F(", "));
        }
        else
        {
           Serial.println(thermistorSamples[thermistor][whichSample]);
        }
    }
#endif

    whichSample = (whichSample + 1) % NUMSAMPLES;
}

void fillThermistorTemps()
{
    for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
    {
        // average all the samples out
        float average  = 0;
        for (int i = 0; i < NUMSAMPLES; ++i)
        {
            average = average + thermistorSamples[thermistor][i];
        }
        average = average / NUMSAMPLES;

        // convert the value to resistance
        average = 1023 / average - 1;
        average = thermistorResistors[thermistor] / average;

        float steinhart = average / THERMISTORNOMINAL;               // (R/Ro)
        steinhart = log(steinhart);                                  // ln(R/Ro)
        steinhart = steinhart / BCOEFFICIENT;                        // 1/B * ln(R/Ro)
        steinhart = steinhart + 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
        steinhart = 1.0 / steinhart;                                 // Invert
        steinhart = steinhart - 273.15;                              // convert to C

        thermistorTemperatures[thermistor] = steinhart;
    }
}

bool feedGPS(unsigned long start)
{
    const unsigned long timeout = 250;
#if GPS_ON_SWSERIAL
    while (nss.available())
    {
        if (gps.encode(nss.read()))
#else
    while (Serial.available())
    {
        if (gps.encode(Serial.read()))
#endif
        {
            return true;
        }
        if (millis() - start > timeout)
        {
            //break;
        }
    }
    return false;
}

void fillGPS(TinyGPS &gps)
{
    gpsData.satCount = gps.satellites();
    gpsData.hdop     = gps.hdop();

    gps.get_position(&gpsData.lat, &gpsData.lon, &gpsData.fixAge);
    gps.get_datetime(&gpsData.date, &gpsData.time, &gpsData.dateAge);

    gpsData.speed    = gps.speed();
    gpsData.course   = gps.course();
    gpsData.alt      = gps.altitude();
}

unsigned long lastUpdate(0);
unsigned long lastSensorUpdate(0);
byte updateTurn(0);
float insideTemp(0.0f);
float outsideTemp(0.0f);
float humidity(0.0f);

void loop()
{
    if (digitalRead(RBF_PIN) == LOW)
    {
        // slow?
        feedGPS(millis());

        if (millis() - lastUpdate >= 500)
        {
            // fast
            if (millis() - lastSensorUpdate > 5000)
            {
                switch (updateTurn)
                {
                    case 0:
                        sensors.requestTemperatures();
                        insideTemp  = sensors.getTempC(insideThermometer);
                        outsideTemp = sensors.getTempC(outsideThermometer);
                        break;
                    case 1:
                        humidity = dht.readHumidity();
                        break;
                }
                updateTurn = (updateTurn + 1) % 2;
            }

            fillGPS(gps);
            getThermistors();
            fillThermistorTemps();

            mySerial.print(F("!h:"));
            mySerial.print(humidity);
            mySerial.print(F(",i:"));
            mySerial.print(insideTemp);
            mySerial.print(F(",o:"));
            mySerial.print(outsideTemp);

            for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
            {
                mySerial.print(",t");
                mySerial.print(thermistor);
                mySerial.print(":");
                mySerial.print(thermistorTemperatures[thermistor]);
            }

            mySerial.print(F(",gsa:"));
            mySerial.print(gpsData.satCount);
            mySerial.print(F(",ghd:"));
            mySerial.print(gpsData.hdop);
            mySerial.print(F(",glt:"));
            mySerial.print(gpsData.lat);
            mySerial.print(F(",gln:"));
            mySerial.print(gpsData.lon);
            mySerial.print(F(",gfa:"));
            mySerial.print(gpsData.fixAge);
            Serial.print(F(",gal:"));
            Serial.print(gpsData.alt);
            mySerial.print(F(",gdt:"));
            mySerial.print(gpsData.date);
            mySerial.print(F(",gtm:"));
            mySerial.print(gpsData.time);
            mySerial.print(F(",gda:"));
            mySerial.print(gpsData.dateAge);
            mySerial.print(F(",gsp:"));
            mySerial.print(gpsData.speed);
            mySerial.print(F(",gco:"));
            mySerial.print(gpsData.course);
            mySerial.print(F("#\n"));

#if GPS_ON_SWSERIAL
            Serial.print(F("!h:"));
            Serial.print(humidity);
            Serial.print(F(",i:"));
            Serial.print(insideTemp);
            Serial.print(F(",o:"));
            Serial.print(outsideTemp);

            for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
            {
                Serial.print(",t");
                Serial.print(thermistor);
                Serial.print(":");
                Serial.print(thermistorTemperatures[thermistor]);
            }

            Serial.print(F(",gsa:"));
            Serial.print(gpsData.satCount);
            Serial.print(F(",ghd:"));
            Serial.print(gpsData.hdop);
            Serial.print(F(",glt:"));
            Serial.print(gpsData.lat);
            Serial.print(F(",gln:"));
            Serial.print(gpsData.lon);
            Serial.print(F(",gfa:"));
            Serial.print(gpsData.fixAge);
            Serial.print(F(",gal:"));
            Serial.print(gpsData.alt);
            Serial.print(F(",gdt:"));
            Serial.print(gpsData.date);
            Serial.print(F(",gtm:"));
            Serial.print(gpsData.time);
            Serial.print(F(",gda:"));
            Serial.print(gpsData.dateAge);
            Serial.print(F(",gsp:"));
            Serial.print(gpsData.speed);
            Serial.print(F(",gco:"));
            Serial.print(gpsData.course);
            Serial.print(F("#\n"));
#endif
            lastUpdate = millis();
        }
    }
}