/*  BGC MAP
           FRONT

          |#####|
       IO-|#####|-OI       R
L  [0] OI |#####| IO  [1]  I
E         |#####|          G
F         |#####|          H
T         |#####|          T
       IO-|#####|-OI
   [3] OI |#####| IO  [2]

            BACK
*/

#include <Wire.h>
#include <EEPROM.h>

#define debug false

#define i2cEEPROMadr 0
#define enc0EEPROMadr 1
#define enc1EEPROMadr 2

#define i2cMaster 0x10


#define speed 30

#define timerOutPin 16     // timing out pin for debugging and optimisation
#define printThrottel 1000 // output throttel of serial prints in loop (works in conjunction with 'counter')
#define execSpeed 2        // numer of adc measurements till execution (works in conjunction with 'measurement')

#define Encoder1 A0 // PC0
#define Encoder2 A1 // PC1
#define Coil1 0x653 // pins for motor 0
#define Coil2 0xBA9 // pins for motor 1

#define maxTorque 90 // defines the absolut torque in perzent time 100

#define resolution 6 // resolution of electrical rotaion
#define resolutionX2 12
#define resolutionX3 18
#define forward90 5
#define backward90 13
#define poles 7                // number of magnetig poles
#define electricalRotation 126 // resolution time 3, for all coils, time 7, for all poles

#define encoderRes 1023
#define halfEncoderRes 512
#define tSteps 21 // number of distinct torque steps

uint8_t sinLookup[resolutionX3][tSteps];

// values used inside adc interrupt
volatile uint8_t measurement;
volatile uint16_t analogValue;
volatile uint16_t counter = 0;

// values defining the the state of the executen and subject of control
uint8_t side = 0;
uint8_t state = 0;

// stuff for calibration
uint16_t counter2 = 0;
uint16_t countSides[2];
int8_t targetDirect[2];
uint16_t avgOffset[2][2];
short deadband = 12;
short gain = 2;

uint16_t encoder[2];
int16_t lastEnco[2];
bool calibrated = false;
bool configurator = false;
bool enable = false;

uint16_t virtAngle[2];
uint8_t encOffset[2];
uint8_t torque[2];
int16_t error[2];
char cMessage[10];
uint8_t selection;
uint8_t value;
uint8_t i2cAdresse = 0;

int absAngle[2];
int absSet[2];

uint8_t torqueLimit[2];
uint8_t orders[6];
uint8_t cState[6];
int16_t cycleCoil[2][3];

int16_t delta;
int16_t ldelta[2];
uint16_t activeCoil[2];

uint8_t i;
uint8_t i2cI;

void getEEPROM();
void setComs();
void setPinModes();
void setSINLookup();
void setADCRegister();
void setPWMRegister();
void setINTRegister();
void setConstants();
void setVariables();

int fWrap(int val, int lim);

void calibrationLoop();
void executeCalibration();
int circleAvg(int val1, int val2, int circumference);

void readADC();
void calculateAngles();
void calculateErrors();
void calculateCycles();
void executeCycles();
void switchSide();
void communicateOffset();
void setCycles();

typedef void (*taskPointer)();

taskPointer motorDriver[] = {
    readADC,
    calculateAngles,
    calculateErrors,
    calculateCycles,
    executeCycles,
    switchSide};

taskPointer calibDriver[] = {
    readADC,
    calculateAngles,
    communicateOffset,
    setCycles,
    executeCycles,
    switchSide};

void setup()
{
    getEEPROM();
    setConstants();
    setVariables();
    setComs();
    setPinModes();
    setSINLookup();
    cli();
    setADCRegister();
    setPWMRegister();
    sei();
}

void calibrationLoop()
{
    int executionCycles = electricalRotation * speed;
    torque[0] = tSteps>>1;
    torque[1] = tSteps>>1;
    executeCalibration(executionCycles, -1, -1);
    executeCalibration(executionCycles, 1, 1);
    avgOffset[0][1] = avgOffset[0][0] / (countSides[0]/speed);
    avgOffset[1][1] = avgOffset[1][0] / (countSides[1]/speed);
    executeCalibration(executionCycles, -1, -1);
    avgOffset[0][0] = avgOffset[0][0] / (countSides[0]/speed);
    avgOffset[1][0] = avgOffset[1][0] / (countSides[1]/speed);
    encOffset[0] = circleAvg(avgOffset[0][0], avgOffset[0][1], resolutionX3);
    encOffset[1] = circleAvg(avgOffset[1][0], avgOffset[1][1], resolutionX3);
    executeCalibration(executionCycles >> 1, 1, 1);
    torque[0] = 0;
    torque[1] = 0;
}

int circleAvg(int val1, int val2, int circumference)
{
    if (abs(val1 - val2) < abs(abs(val1 - val2) - circumference))
    {
        return (val1 + val2) / 2;
    }
    return fWrap((val1 + val2 + circumference) / 2, circumference);
}

void executeCalibration(int cycles, int d1, int d2)
{
    counter2 = 0;
    countSides[0] = 0;
    countSides[1] = 0;
    targetDirect[0] = d1;
    targetDirect[1] = d2;
    avgOffset[0][0] = 0;
    avgOffset[1][0] = 0;
    while (counter2 < cycles)
    {
        calibDriver[state]();
    }
}

void loop()
{
    motorDriver[state]();
    if (counter > printThrottel)
    {
        if (configurator)
        {

            if (Serial.available())
            {
                i = 0;
                while (Serial.available())
                {
                    if (i < 10)
                    {
                        cMessage[i] = Serial.read();
                    }
                    i++;
                }
                Serial.println();
                selection = cMessage[0] - '0';
                i = 0;
                while((cMessage[i] != '\n') && (i<10))
                {
                    i++;
                }
                cMessage[i] = '\0';
                value = 0;
                if (i > 1)
                {
                    i = 0;
                    while ((cMessage[i] != ' ') && (i<10))
                    {
                        i++;
                    }
                    value = atoi(&cMessage[i]);
                }
                i = 0;
                while (i < 10)
                {
                    cMessage[i] = 0;
                    i++;
                }
                Serial.print("Selected option ");
                Serial.print(selection);
                Serial.print(" with value ");
                Serial.println(value);
            }
            if (selection != 99)
            {
                switch (selection)
                {
                case 9:
                    configurator = false;
                    Serial.println("\n----- Goodbye! ------\n");
                    break;
                case 0:
                    EEPROM.write(i2cEEPROMadr, value&0xFF);
                    getEEPROM();
                    Serial.print("new address is: ");
                    Serial.println(i2cAdresse);
                    selection = 99;
                    break;
                case 1:
                    EEPROM.write(enc0EEPROMadr, value&0xFF);
                    getEEPROM();
                    selection = 99;
                    break;
                case 2:
                    EEPROM.write(enc1EEPROMadr, value&0xFF);
                    getEEPROM();
                    selection = 99;
                    break;
                case 3:
                    Serial.println(encoder[0]);
                    if (value < 1)
                    {
                        selection = 99;
                    }
                    value--;
                    break;
                case 4:
                    Serial.println(encoder[1]);
                    if (value < 1)
                    {
                        selection = 99;
                    }
                    value--;
                    break;
                case 5:
                    enable = true;
                    calibrationLoop();
                    enable = false;
                    state = 0;
                    Serial.print("Side 0 ");
                    Serial.print(encOffset[0]);
                    Serial.print(" Side 1 ");
                    Serial.println(encOffset[1]);
                    EEPROM.write(enc0EEPROMadr, encOffset[0]&0xFF);
                    EEPROM.write(enc1EEPROMadr, encOffset[1]&0xFF);
                    selection = 99;
                    break;
                default:
                    Serial.println("invalid Selection!!");
                    selection = 99;
                }
            }
        }
        else if (Serial.available() > 0)
        {
            while (Serial.available())
            {
                Serial.read();
            }
            configurator = true;
            selection = 99;
            Serial.println("----- Welcome! ------\n");
            Serial.print("The i2c Address is: ");
            Serial.println(i2cAdresse);
            Serial.print("encOffset0: ");
            Serial.print(encOffset[0]);
            Serial.print(" encOffset1: ");
            Serial.println(encOffset[1]);
            Serial.println("\nOptions:");
            Serial.println("----- Set Values ----");
            Serial.println("0 -> set i2c address of the board");
            Serial.println("1 -> set offset of encoder 0");
            Serial.println("2 -> set offset of encoder 1");
            Serial.println("----- Get Values ----");
            Serial.println("3 -> get value of encoder 0");
            Serial.println("4 -> get value of encoder 1");
            Serial.println("---- Calibration ----");
            Serial.println("5 -> get calibration value");
            Serial.println("9 -> end the configurator");
            Serial.println("Write the number of the option");
            Serial.println("and when neccesary, the variable");
            Serial.println("afterwards. maked sure to end the");
            Serial.println("message with a new line charakter");
        }
        if (debug)
        {

            // printing stuff here for debugging
        }
        counter = 0;
    }
}

void executeCycles()
{
  if(enable){
    for (i = 0; i < 3; i++)
    {
        analogWrite((activeCoil[side] >> i * 4) & 0xF, (sinLookup[cycleCoil[side][i]][torque[side]]));
    }
  } else{
    for (i = 0; i < 3; i++)
    {
        analogWrite((activeCoil[side] >> i * 4) & 0xF, 0);
    }
  }
    state = 5;
}

void setCycles()
{
    cycleCoil[side][0] = fWrap(cycleCoil[side][0] + resolutionX3, resolutionX3);
    cycleCoil[side][1] = fWrap(cycleCoil[side][0] + resolution, resolutionX3);
    cycleCoil[side][2] = fWrap(cycleCoil[side][0] + resolutionX2, resolutionX3);
    state = 4;
}

void communicateOffset()
{
    delta = (encoder[side] >> 3) - cycleCoil[side][0];
    delta = fWrap(delta + resolutionX3, resolutionX3);
    if(abs(delta-ldelta[side]) > 7){
      Serial.print(side);
      Serial.print("-side Error: ");
      Serial.print(delta);
      Serial.print(", ");
      Serial.println(ldelta[side]);
    } 
    ldelta[side] = delta;
    if (countSides[side] % speed == 0)
    {
        Serial.print(side);
        Serial.print("-side delta: ");
        Serial.println(delta);
        avgOffset[side][0] += delta;
        
        cycleCoil[side][0] += targetDirect[side];
    }
    countSides[side]++;
    counter2++;
    state = 3;
}

void calculateCycles()
{
    cycleCoil[side][0] = fWrap(virtAngle[side] + error[side] + resolutionX3, resolutionX3);
    cycleCoil[side][1] = fWrap(cycleCoil[side][0] + resolution, resolutionX3);
    cycleCoil[side][2] = fWrap(cycleCoil[side][0] + resolutionX2, resolutionX3);
    state = 4;
}

void calculateErrors()
{
    error[side] = absSet[side] - absAngle[side];
    torque[side] = constrain(((abs(error[side]) >> gain) - deadband), 0, torqueLimit[side]);
    if (error[side] > 0)
    {
        error[side] = forward90;
    }
    else if (error[side] < 0)
    {
        error[side] = backward90;
    }
    else
    {
        error[side] = 0;
    }
    state = 3;
}

void calculateAngles()
{
    virtAngle[side] = fWrap((encoder[side] >> 3) - encOffset[side] + 127, 127);
    delta = (encoder[side]) - lastEnco[side];
    lastEnco[side] = encoder[side];
    if (delta >= halfEncoderRes)
        delta = delta - encoderRes;
    else if (delta <= -halfEncoderRes)
        delta = delta + encoderRes;
    absAngle[side] += delta;
    state = 2;
}

void readADC()
{
    if (measurement > execSpeed)
    {
        digitalWrite(timerOutPin, HIGH);
        cli();
        encoder[side] = analogValue;
        sei();
        if (side)
            ADMUX &= B11111110;
        else
            ADMUX |= B00000001;
        measurement = 0;
        state = 1;
    }
}

void switchSide()
{
    side = (side + 1) & 0x1;
    state = 0;
    digitalWrite(timerOutPin, LOW);
}

int fWrap(int val, int lim)
{
    if (val >= lim)
    {
        val -= lim;
    }
    if (val < lim)
    {
        return val;
    }
    else
    {
        return fWrap(val, lim);
    }
}

/*----------------------------------------
      ADC interrupt
----------------------------------------*/

ISR(ADC_vect)
{
    analogValue = ADCL | (ADCH << 8);
    measurement++;
    counter++;
}

/*----------------------------------------
      Setup functions implementation
----------------------------------------*/

void setSINLookup()
{
    float incr = (2 * 3.14) / (float)resolutionX3;
    for (int t = 0; t < tSteps; t++)
    {
        for (int s = 0; s < resolutionX3; s++)
        {
            sinLookup[s][t] = (int)(((((sin(s * incr) * -127.) + 127.) * maxTorque / 100.) * ((float)t)) / ((float)tSteps - 1.));
        }
    }
}

void setPWMRegister()
{
    TCCR0B &= B11111000;
    TCCR0B |= B00000001; // 31250 Hz for Pins  5 and  6, but changes internal delay and mills (490hz og)
    TCCR1B &= B11111000;
    TCCR1B |= B00000001; // 31250 Hz for Pins  9 and 10
    TCCR2B &= B11111000;
    TCCR2B |= B00000001; // 31250 Hz for Pins 11 and  3
    ICR1 = 0xFF;         // 8 bit resolution for PWM
}

void setADCRegister()
{
    ADMUX |= B01000001;
    ADMUX &= B01110001;
    ADCSRA |= B11101111;
    ADCSRA &= B11101111;
    ADCSRB |= B00000000;
    ADCSRB &= B11111000;
    ADMUX &= B11111110;
}

void getEEPROM()
{
    i2cAdresse = EEPROM.read(i2cEEPROMadr);
    encOffset[0] = EEPROM.read(enc0EEPROMadr);
    encOffset[1] = EEPROM.read(enc1EEPROMadr);
}

void setComs()
{
    Serial.begin(115200);
    Wire.begin(i2cAdresse);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
}

void setPinModes()
{
    pinMode(Encoder1, INPUT);
    pinMode(Encoder2, INPUT);
    pinMode(timerOutPin, OUTPUT);
    for (i = 0; i < 3; i++)
    {
        pinMode((Coil1 >> i * 4) & 0xF, OUTPUT);
        pinMode((Coil2 >> i * 4) & 0xF, OUTPUT);
    }
}

void setConstants()
{
    activeCoil[0] = Coil1;
    activeCoil[1] = Coil2;
}

void setVariables()
{
    absSet[0] = 0;
    absSet[1] = 0;
    torqueLimit[0] = 0;
    torqueLimit[1] = 0;
    state = 0;
}

void receiveEvent(int howMany)
{
    enable = true;
    i2cI = 0;
    while (Wire.available())
    {
        orders[i2cI] = Wire.read();
        i2cI++;
    }
    switch (orders[0])
    {
    case 0xFF:
        absAngle[0] = 0;
        absAngle[1] = 0;
        break;
    case 0xFE:
        deadband = orders[1];
        break;
    case 0xFD:
        gain = orders[1];
        break;
    default:
        torqueLimit[0] = orders[0];
        torqueLimit[1] = orders[1];
        absSet[0] = orders[2] | orders[3] << 8;
        absSet[1] = orders[4] | orders[5] << 8;
    }
}

void requestEvent()
{
    if (calibrated)
    {
        cState[0] |= torque[0] & 0xFF;
        cState[1] |= torque[1] & 0xFF;
        cState[2] |= absAngle[0] & 0xFF;
        cState[3] |= (absAngle[0] >> 8) & 0xFF;
        cState[4] |= absAngle[1] & 0xFF;
        cState[5] |= (absAngle[1] >> 8) & 0xFF;
    }
    else
    {
        cState[0] = 0xFF;
        cState[1] = 0xFF;
        cState[2] = 0xFF;
        cState[3] = 0xFF;
        cState[4] = 0xFF;
        cState[5] = 0xFF;
    }
    Wire.write(cState, 6);
    for (i2cI = 0; i2cI < 6; i2cI++)
    {
        cState[i2cI] = 0;
    }
}
