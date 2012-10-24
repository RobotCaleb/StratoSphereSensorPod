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
//  7/D4   - Remove Before Flight
//  8/D2   - AM2302 Humidity
//  9/D1   - D2523T-6 GPS RX
// 10/D0   - D2523T-6 GPS TX
// 11/A0   - TTF103 Thermistor - 9820 Ohms
// 12/A1   - TTF103 Thermistor - 9850 Ohms
// 13/A2   - TTF103 Thermistor - 9800 Ohms
// 14/A3   - TTF103 Thermistor - 9810 Ohms
// 15/AREF - 3.3V AREF


// digital pin that DHT is on
#define DHT_PIN          2  // humidity
#define ONE_WIRE_BUS_PIN 3  // temperature
#define OPENLOG_RX_PIN   8  // OpenLog RX
#define OPENLOG_TX_PIN   9  // OpenLog TX
#define RBF_PIN          4  // Remove before flight

// DHT 22  (AM2302)
#define DHT_TYPE DHT22

// Temperature data wire is plugged into port 3
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance
#if WITH_ONE_WIRE
OneWire oneWire(ONE_WIRE_BUS_PIN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
#endif // WITH_ONE_WIRE

// arrays to hold device addresses
DeviceAddress insideThermometer, outsideThermometer;

DHT dht(DHT_PIN, DHT_TYPE);

// RX on pin 8, TX on pin 9
AltSoftSerial mySerial;

// if not on software serial, should probably be on hardware serial. :)
// if on software serial, 5 is rx, 6 is tx
#define GPS_ON_SWSERIAL 0
#define WITH_ONE_WIRE   0

#define DUMPDEBUG 0

TinyGPS gps;

#if GPS_ON_SWSERIAL
SoftwareSerial nss(5, 6);
#endif

void fillThermistorTemps();
void logData();

// THIS CODE EXPECTS THERMISTORS TO BE ON ANALOG PINS 0 - (N - 1)
// WHERE N IS THE NUMBER OF THERMISTORS
// supports up to 4
#define NUMTHERMISTORS 4
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5

int whichSample;
// the value of the 'other' resistor
int thermResistors[NUMTHERMISTORS]      = {9820, 9850, 9800, 9800};
// The beta coefficient of the thermistor (usually 3000-4000)
int thermCoefficients[NUMTHERMISTORS]   = {3950, 3435, 3435, 3435};
// temp. for nominal resistance (almost always 25 C)
int thermTempNominals[NUMTHERMISTORS]   = {  25,   25,   25,   25};
// resistance at 25 degrees C
int thermResistNominals[NUMTHERMISTORS] = {10000, 10000, 10000, 10000};
int thermSamples[NUMTHERMISTORS][NUMSAMPLES];
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
#if WITH_ONE_WIRE
    sensors.begin();
#endif //WITH_ONE_WIRE

    // set the data rate for the SoftwareSerial port
    mySerial.begin(19200);

#if GPS_ON_SWSERIAL
    Serial.begin(19200);
#else
    Serial.begin(9600);
#endif

#if WITH_ONE_WIRE
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
#endif // GPS_ON_SWSERIAL
#endif //WITH_ONE_WIRE

    Serial.print(F("Front loading "));
    Serial.print(NUMTHERMISTORS);
    Serial.println(F(" thermistors"));
    for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
    {
        for (int sample = 0; sample < NUMSAMPLES; ++sample)
        {
            fillThermistorTemps();
        }
    }
    whichSample = 0;

    analogReference(EXTERNAL);

#if GPS_ON_SWSERIAL
    nss.begin(9600);
#endif
}

void getThermistors()
{
    for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
    {
        thermSamples[thermistor][whichSample] = analogRead(thermistor);
    }

#if GPS_ON_SWSERIAL && DUMPDEBUG
    Serial.print(F("Raw analog readings "));
    for (int thermistor = 0; thermistor < NUMTHERMISTORS; ++thermistor)
    {
        if (thermistor < NUMTHERMISTORS - 1)
        {
            Serial.print(thermSamples[thermistor][whichSample]);
            Serial.print(F(", "));
        }
        else
        {
           Serial.println(thermSamples[thermistor][whichSample]);
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
            average = average + thermSamples[thermistor][i];
        }
        average = average / NUMSAMPLES;

        // convert the value to resistance
        average = 1023 / average - 1;
        average = thermResistors[thermistor] / average;

        float stein = average / thermResistNominals[thermistor];              // (R/Ro)
        stein       = log(stein);                                             // ln(R/Ro)
        stein       = stein / thermCoefficients[thermistor];                  // 1/B * ln(R/Ro)
        stein       = stein + 1.0 / (thermTempNominals[thermistor] + 273.15); // + (1/To)
        stein       = 1.0 / stein;                                            // Invert
        stein       = stein - 273.15;                                         // convert to C

        thermistorTemperatures[thermistor] = stein;
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
float hSenTemp(0.0f);
unsigned long timeOfStartLogging(0);

void loop()
{
    unsigned long curTime = millis();
    feedGPS(curTime);

    if (curTime - lastSensorUpdate > 1000)
    {
        switch (updateTurn)
        {
            case 0:
                humidity = dht.readHumidity();
                break;
            case 1:
                hSenTemp = dht.readTemperature();
                break;
            case 2:
#if WITH_ONE_WIRE
                sensors.requestTemperatures();
                insideTemp  = sensors.getTempC(insideThermometer);
                outsideTemp = sensors.getTempC(outsideThermometer);
#endif // WITH_ONE_WIRE
                break;
        }
        updateTurn = (updateTurn + 1) % 2;
        lastSensorUpdate = millis();
    }

    fillGPS(gps);
    getThermistors();
    fillThermistorTemps();

    if (digitalRead(RBF_PIN) == LOW)
    {
        if (timeOfStartLogging == 0)
        {
            timeOfStartLogging = curTime;
        }

        if (millis() - lastUpdate >= 1000)
        {
            logData();
            lastUpdate = millis();
        }
    }
}

// writes to serial and to OpenLog for logging
void logData()
{
    mySerial.print(F("!tm:"));
    mySerial.print(millis() - timeOfStartLogging);
    mySerial.print(F(",h:"));
    mySerial.print(humidity);
    mySerial.print(F(",ht:"));
    mySerial.print(hSenTemp);
#if WITH_ONE_WIRE
    mySerial.print(F(",i:"));
    mySerial.print(insideTemp);
    mySerial.print(F(",o:"));
    mySerial.print(outsideTemp);
#endif //WITH_ONE_WIRE

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
    mySerial.print(F(",gal:"));
    mySerial.print(gpsData.alt);
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
    Serial.print(F("!tm:"));
    Serial.print(millis() - timeOfStartLogging);
    Serial.print(F(",h:"));
    Serial.print(humidity);
    Serial.print(F(",ht:"));
    Serial.print(hSenTemp);
#if WITH_ONE_WIRE
    Serial.print(F(",i:"));
    Serial.print(insideTemp);
    Serial.print(F(",o:"));
    Serial.print(outsideTemp);
#endif //WITH_ONE_WIRE

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
}