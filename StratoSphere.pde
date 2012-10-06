#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <AltSoftSerial.h>

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

// which analog pin to connect
#define INTHERMISTORPIN  A0
#define OUTTHERMISTORPIN A1
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3435
// the value of the 'other' resistor
#define INRESISTOR  9810
#define OUTRESISTOR 9850

int inSamples[NUMSAMPLES];
int outSamples[NUMSAMPLES];
int whichSample;

float insideThermistor;
float outsideThermistor;

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

    Serial.begin(9600);

    // set the data rate for the SoftwareSerial port
    mySerial.begin(19200);

    delay(1000);

    // locate devices on the bus
    Serial.print("Locating devices...");
    Serial.print("Found ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" devices.");

    // report parasite power requirements
    Serial.print("Parasite power is: ");
    if (sensors.isParasitePowerMode())
    {
        Serial.println(F("ON"));
    }
    else
    {
        Serial.println(F("OFF"));
    }

    if (!sensors.getAddress(insideThermometer, 0))
    {
        Serial.println(F("Unable to find address for Device 0"));
    }
    if (!sensors.getAddress(outsideThermometer, 1))
    {
        Serial.println(F("Unable to find address for Device 1"));
    }

    // show the addresses we found on the bus
    Serial.print("Device 0 (Inside) Address: ");
    printAddress(insideThermometer);
    Serial.println();

    Serial.print("Device 1 (Outside) Address: ");
    printAddress(outsideThermometer);
    Serial.println();

    // set the resolution to 9 bit
    sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

    Serial.print("Device 0 (Inside) Resolution: ");
    Serial.print(sensors.getResolution(insideThermometer), DEC);
    Serial.println();

    Serial.print("Device 1 (Outside) Resolution: ");
    Serial.print(sensors.getResolution(outsideThermometer), DEC);
    Serial.println();

    for (int i = 0; i < NUMSAMPLES; ++i)
    {
        inSamples[i]  = 0;
        outSamples[i] = 0;
    }
    whichSample = 0;
    insideThermistor  = 0.0f;
    outsideThermistor = 0.0f;
    analogReference(EXTERNAL);
}

void GetThermistors()
{
    uint8_t i;
    float inAverage;
    float outAverage;

    inSamples[whichSample]  = analogRead(INTHERMISTORPIN);
    outSamples[whichSample] = analogRead(OUTTHERMISTORPIN);

    Serial.print("Raw analog readings ");
    Serial.print(inSamples[whichSample]);
    Serial.print(", ");
    Serial.println(outSamples[whichSample]);

    // average all the samples out
    inAverage  = 0;
    outAverage = 0;
    for (i=0; i< NUMSAMPLES; i++)
    {
       inAverage += inSamples[i];
       outAverage += outSamples[i];
    }
    inAverage /= NUMSAMPLES;
    outAverage /= NUMSAMPLES;

    // Serial.print("Average analog readings ");
    // Serial.print(inAverage);
    // Serial.print(", ");
    // Serial.println(outAverage);

    // convert the value to resistance
    inAverage = 1023 / inAverage - 1;
    inAverage = INRESISTOR / inAverage;
    outAverage = 1023 / outAverage - 1;
    outAverage = OUTRESISTOR / outAverage;
    
    // Serial.print("Thermistor resistances ");
    // Serial.print(inAverage);
    // Serial.print(", ");
    // Serial.println(outAverage);

    float inSteinhart;
    float outSteinhart;
    inSteinhart = inAverage / THERMISTORNOMINAL;        // (R/Ro)
    inSteinhart = log(inSteinhart);                       // ln(R/Ro)
    inSteinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
    inSteinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    inSteinhart = 1.0 / inSteinhart;                      // Invert
    inSteinhart -= 273.15;                              // convert to C

    outSteinhart = outAverage / THERMISTORNOMINAL;          // (R/Ro)
    outSteinhart = log(outSteinhart);                       // ln(R/Ro)
    outSteinhart /= BCOEFFICIENT;                        // 1/B * ln(R/Ro)
    outSteinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    outSteinhart = 1.0 / outSteinhart;                      // Invert
    outSteinhart -= 273.15;                              // convert to C

    // Serial.print("Temperatures ");
    // Serial.print(inSteinhart);
    // Serial.print(", ");
    // Serial.print(outSteinhart);
    // Serial.println(" *C");

    insideThermistor  = inSteinhart;
    outsideThermistor = outSteinhart;

    whichSample = (whichSample + 1) % NUMSAMPLES;
}

void loop()
{
    if (digitalRead(RBF_PIN) == LOW)
    {
        float humidity = dht.readHumidity();

        sensors.requestTemperatures();

        float insideTemp  = sensors.getTempC(insideThermometer);
        float outsideTemp = sensors.getTempC(outsideThermometer);

        GetThermistors();

        mySerial.print("!h:");
        mySerial.print(humidity);
        mySerial.print(",i:");
        mySerial.print(insideTemp);
        mySerial.print(",o:");
        mySerial.print(outsideTemp);
        mySerial.print(",it:");
        mySerial.print(insideThermistor);
        mySerial.print(",ot:");
        mySerial.print(outsideThermistor);
        mySerial.print("#\n");

        Serial.print("!h:");
        Serial.print(humidity);
        Serial.print(",i:");
        Serial.print(insideTemp);
        Serial.print(",o:");
        Serial.print(outsideTemp);
        Serial.print(",it:");
        Serial.print(insideThermistor);
        Serial.print(",ot:");
        Serial.print(outsideThermistor);
        Serial.print("#\n");
    }
}