#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>

#define MESSI_USART_START_CHR 'M'
#define MESSI_USART_END_CHR 'E'
#define MESSI_USART_ESC_CHR 'x'

#define TIMERBASE 0x6D84//0xB6C2
//#define TIMERBASE 0xF9E6

static void USARTinit(void);

static uint8_t USARTReceive(void);

static void USARTTransmitString(char *dataPtr);

static void USARTTransmit(unsigned char data);

static void messiUartTransmitStart(void);

static void messiUartTransmitEnd(void);

//Main function to send a byte over the UART to the PC
static void messiUartTransmit(unsigned char data);

static void messiUartTransmitString(uint8_t *msgPtr, uint8_t msgSize);

static void I2CInit(void);

static void I2CStart(void);

static void I2CWait(void);

static void I2CTransmit(uint8_t *msgPtr, uint8_t msgSize);

static void I2CReceive(uint8_t *msgPtr, uint8_t msgSize);

static void I2CACK(void);

static void I2CNACK(void);

static void I2CStop(void);

//main function to send a message with I2C
//*it accepts uinteger array of data packets, its size and type of command - READ/WRITE to perform
static void I2CDo(uint8_t *msg, uint8_t msgSize, uint8_t *readOrWriteCommand, uint8_t cmdSize);

static void formFloat(float number, uint16_t *arr);

static void PWMInit(void);

static void PWMSwitch(uint8_t isOn);

static void MotorInit(void);

static void MotorControl(uint8_t in1A, uint8_t in2A, uint16_t frequencyA, uint8_t in1B, uint8_t in2B,
                         uint16_t frequencyB);

static void bmp085Calibration(void);

static int16_t bmp085GetTemperature(int16_t ut);

static int32_t bmp085ReadUP(void);

static int16_t bmp085ReadUT(void);

static int32_t bmp085GetPressure(int32_t up);

static float bmp085calcAltitude(int32_t pressure);

static int16_t bmp085ReadInt(uint8_t registerAddr);

static void adxl345Init(void);

static void adxl345ReadAcc(void);

//static void adxl345BiasCalc(void);

static void adxl345CalcAcc(void);

static void adxl345SetRange(uint8_t range, uint8_t fullResolution);

static void activateHW(void);

static void timerInit(void);

//control
volatile uint8_t gCommand = 0;

volatile uint8_t gSpeed = 0;

static uint8_t msgUART[38] = {0};


//BMP085

static uint8_t bmp085WriteAddr = 0xEE;

static uint8_t bmp085ReadAddr = 0xEF;

static int16_t bmp085CalibrValuesInt[8] = {0};

static uint16_t bmp085CalibrValuesUInt[3] = {0};

static int32_t bmp085Temperature = 0;

static uint8_t OSS = 0;




static uint8_t adxl345WriteAddr = 0xA6;

static uint8_t adxl345ReadAddr = 0xA7;

static int16_t adxl345AccData[3] = {0};

static int16_t adxl345BiasData[3] = {-268, 170, -2156};

static int16_t adxl345BiasCycleCnt = 100;

volatile float adxl345CalcAccData[3] = {0};

static float adxl345Scale = 0;

static uint8_t gSensorUByteData[400] = {0};
static uint8_t gSensorTempData[200] = {0};

volatile int16_t temperature = 0;
volatile float altitude = 0;

static uint8_t gTempOffset = 2;

volatile uint16_t gTimerCounter = 0;
volatile uint16_t gTimerPremCounter = 0;

volatile uint16_t gSendEEPROMDataIdx = 0;

volatile uint16_t gTimeCountSum = 200;

#define read_eeprom_array(address, value_p, length) eeprom_read_block ((void *)value_p, (const void *)address, length)
#define write_eeprom_array(address, value_p, length) eeprom_write_block ((const void *)value_p, (void *)address, length)


//declare an eeprom array
uint8_t EEMEM gEEPROMUByteArray[400];


int main(void) {


    int32_t pressure;
    float fTemperature = 0;

    uint16_t arrFloat[3];
    uint8_t leftWheelFwd, leftWheelBck, rightWheelFwd, rightWheelBck;

    uint16_t frequencyA, frequencyB;

    /**********************************************************/

    USARTinit();

    I2CInit();

    sei();

    bmp085Calibration();

    adxl345SetRange(16, 1);
    adxl345Init();
    //adxl345BiasCalc();

    _delay_ms(10);

    while (1) {
        //measure data
        messiUartTransmitStart();
        adxl345ReadAcc();
        adxl345CalcAcc();

        temperature = (bmp085GetTemperature(bmp085ReadUT())) - gTempOffset;
        fTemperature = (float) temperature / (float) 10;
        pressure = bmp085GetPressure(bmp085ReadUP());
        altitude = bmp085calcAltitude(pressure);
        //send data with 2 decimal points
        formFloat(fTemperature, arrFloat);

        msgUART[0] = (uint8_t) (arrFloat[0]);
        msgUART[1] = (uint8_t) (arrFloat[1] >> 8);
        msgUART[2] = (uint8_t) (arrFloat[1] & 0x00FF);

        msgUART[3] = (uint8_t) (arrFloat[2] >> 8);
        msgUART[4] = (uint8_t) (arrFloat[2] & 0x00FF);

        formFloat(altitude, arrFloat);

        msgUART[5] = (uint8_t) (arrFloat[0]);
        msgUART[6] = (uint8_t) (arrFloat[1] >> 8);
        msgUART[7] = (uint8_t) (arrFloat[1] & 0x00FF);

        msgUART[8] = (uint8_t) (arrFloat[2] >> 8);
        msgUART[9] = (uint8_t) (arrFloat[2] & 0x00FF);

        formFloat(adxl345CalcAccData[1], arrFloat);

        msgUART[10] = (uint8_t) (arrFloat[0]);
        msgUART[11] = (uint8_t) (arrFloat[1] >> 8);
        msgUART[12] = (uint8_t) (arrFloat[1] & 0x00FF);

        msgUART[13] = (uint8_t) (arrFloat[2] >> 8);
        msgUART[14] = (uint8_t) (arrFloat[2] & 0x00FF);
        //wheels command translation into motor driver commands
        leftWheelFwd = ((gCommand == 5) || (gCommand == 7)) ? 1 : 0;
        leftWheelBck = ((gCommand == 9) || (gCommand == 11)) ? 1 : 0;
        rightWheelFwd = ((gCommand == 6) || (gCommand == 7)) ? 1 : 0;
        rightWheelBck = ((gCommand == 10) || (gCommand == 11)) ? 1 : 0;

        msgUART[15] = (gSpeed & 0x0C) >> 2;

        msgUART[16] = gSpeed & 0x03;

        switch (gSpeed & 0x03) {
            case 0:
                frequencyA = 16000;
                break;
            case 1:
                frequencyA = 32000;
                break;
            case 2:
                frequencyA = 48000;
                break;
            case 3:
                frequencyA = 65000;
                break;
            default:
                frequencyA = 0;
                break;
        }


        switch (gSpeed & 0x0C) {
            case 0:
                frequencyB = 16000;
                break;
            case 4:
                frequencyB = 32000;
                break;
            case 8:
                frequencyB = 48000;
                break;
            case 12:
                frequencyB = 65000;
                break;
            default:
                frequencyB = 0;
                break;
        }


        MotorControl(leftWheelFwd, leftWheelBck, frequencyA, rightWheelFwd, rightWheelBck, frequencyB);
        if (gCommand == 32) {
            if (gSendEEPROMDataIdx < gTimeCountSum) {
                msgUART[19] = gSensorUByteData[gSendEEPROMDataIdx * 2];
                msgUART[20] = gSensorUByteData[(gSendEEPROMDataIdx * 2) + 1];
                msgUART[21] = gSensorTempData[gSendEEPROMDataIdx];
                gSendEEPROMDataIdx++;
            }
            else {
                msgUART[19] = 0;
                msgUART[20] = 0;
                msgUART[21] = 0;
            }
        } else {
            msgUART[19] = 0;
            msgUART[20] = 0;
            msgUART[21] = 0;
        }



        msgUART[22] = (uint8_t) (gTimerPremCounter >> 8);
        msgUART[23] = (uint8_t) (gTimerPremCounter & 0x00FF);
        msgUART[24] = (uint8_t) (gTimerCounter >> 8);
        msgUART[25] = (uint8_t) (gTimerCounter & 0x00FF);
        msgUART[26] = gCommand;

        msgUART[27] = (uint8_t) (gSendEEPROMDataIdx >> 8);
        msgUART[28] = (uint8_t) (gSendEEPROMDataIdx & 0x00FF);


        messiUartTransmitString(msgUART, 38);

        messiUartTransmitEnd();

    }

    return 0;

}

void timerInit(void) {

    //disable Motor PWM timer
    TCCR1B = TCCR1B & ~(1 << WGM13);
    TCCR1B = TCCR1B & ~(1 << CS10);
    PWMSwitch(0);


    //Start Timer1 with prescaling
    TCCR1B = TCCR1B | (1 << CS11) | (1 << CS10);

    //Initialize counter
    TCNT1 = TIMERBASE;

    //Enable interrupt by overflow
    TIMSK1 = TIMSK1 | (1 << TOIE1);

}

void MotorInit(void) {

    PWMInit();


    DDRC = DDRC | (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3);
    DDRC = DDRB | (1 << DDB0);


    PORTB = 0x00;
    PORTC = 0x00;

    PWMSwitch(1);
    // switch on standby
    PORTB = PORTB | (1 << PORTB0);
}


void PWMInit(void) {

    //disable cs12 value from Motor timer
    TCCR1B = TCCR1B & ~(1 << CS11) & ~(1 << CS10);
    //Disable interrupts
    TIMSK1 = TIMSK1 & ~(1 << TOIE1);



    //PWM phase and freq correct mode
    TCCR1A = TCCR1A & ~(1 << WGM10);
    TCCR1A = TCCR1A & ~(1 << WGM11);

    TCCR1B = TCCR1B & ~(1 << WGM12);
    TCCR1B = TCCR1B | (1 << WGM13);

    ICR1 = 0xFFFF;
    // set TOP to 16bit

    TCCR1B |= (1 << CS10);
    // START the timer with no prescaler
    // set to output OCR1A,B registers
    DDRB = DDRB | (1 << DDB1) | (1 << DDB2);


}

void PWMSwitch(uint8_t isOn) {
    if (isOn == 0) {
        //switch off wave generation
        TCCR1A = TCCR1A & ~(1 << COM1A1) & ~(1 << COM1B1);
    } else {
        TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
    }
}

void activateHW(void) {
    DDRD = DDRD | (1 << DDD5);

    PORTD = PORTD | (1 << PORTD5);

    _delay_ms(5000);

    PORTD = PORTD & ~(1 << PORTD5);

}


void adxl345Init(void) {
    uint8_t cmdWrite[2] = {adxl345WriteAddr, 0x2D};
    uint8_t msgWrite[1] = {0x08};


    I2CDo(msgWrite, 1, cmdWrite, 2);


}

void adxl345SetRange(uint8_t range, uint8_t fullResolution) {
    uint8_t cmdSetReadingPoint[2] = {adxl345WriteAddr, 0x31};
    uint8_t msgSetReadingPoint[1] = {0};

    uint8_t cmdRead[1] = {adxl345ReadAddr};
    uint8_t msgRead[1] = {0};

    uint8_t cmdWrite[2] = {adxl345WriteAddr, 0x31};
    uint8_t msgWrite[1] = {0};

    I2CDo(msgSetReadingPoint, 0, cmdSetReadingPoint, 2);
    I2CDo(msgRead, 1, cmdRead, 1);

    // Get current data from this register.
    uint8_t data = msgRead[0];

    // We AND with 0xF4 to clear the bits are going to set.
    // Clearing ----X-XX
    data &= 0xF4;

    // By default (range 2) or FullResolution = true, scale is 2G.
    adxl345Scale = 0.00390625;

    // Set the range bits.
    switch (range) {
        case 2:
            break;
        case 4:
            data |= 0x01;
            if (fullResolution == 0) { adxl345Scale = 0.0078125; }
            break;
        case 8:
            data |= 0x02;
            if (fullResolution == 0) { adxl345Scale = 0.015625; }
            break;
        case 16:
            data |= 0x03;
            if (fullResolution == 0) { adxl345Scale = 0.03125; }
            break;
        default:
            break;
    }

    // Set the full resolution bit.
    if (fullResolution == 1)
        data |= 0x08;

    msgWrite[0] = data;

    I2CDo(msgWrite, 1, cmdWrite, 2);
}


void adxl345BiasCalc(void) {
    int32_t temp_bias_32_data[3] = {0, 0, 0};

    //read data 100 times @ 10msec interval and compute average value
    for (uint8_t count = 0; count < adxl345BiasCycleCnt; count++) {
        adxl345ReadAcc();
        temp_bias_32_data[0] += adxl345AccData[0];
        temp_bias_32_data[1] += adxl345AccData[1];
        temp_bias_32_data[2] += adxl345AccData[2];
        _delay_ms(10);

    }

    adxl345BiasData[0] = (int16_t) (temp_bias_32_data[0] / adxl345BiasCycleCnt);
    adxl345BiasData[1] = (int16_t) (temp_bias_32_data[1] / adxl345BiasCycleCnt);
    adxl345BiasData[2] = (int16_t) (temp_bias_32_data[2] / adxl345BiasCycleCnt);


}

void adxl345ReadAcc(void) {
    uint8_t cmdSetReadingPoint[2] = {adxl345WriteAddr, 0x32};
    uint8_t msgSetReadingPoint[1] = {0};

    uint8_t cmdRead[1] = {adxl345ReadAddr};
    uint8_t msgRead[6] = {0};

    I2CDo(msgSetReadingPoint, 0, cmdSetReadingPoint, 2);
    I2CDo(msgRead, 6, cmdRead, 1);

    adxl345AccData[0] = (msgRead[1] << 8) | msgRead[0];
    adxl345AccData[1] = (msgRead[3] << 8) | msgRead[2];
    adxl345AccData[2] = (msgRead[5] << 8) | msgRead[4];

}

void adxl345CalcAcc(void) {
    adxl345CalcAccData[0] = ((float) (adxl345AccData[0])) * adxl345Scale;
    adxl345CalcAccData[1] = ((float) (adxl345AccData[1])) * adxl345Scale;
    adxl345CalcAccData[2] = ((float) (adxl345AccData[2])) * adxl345Scale;
}

void bmp085Calibration(void) {
    //The data communication can be checked by checking that none of the words has the value 0 or
    //0xFFFF.

    bmp085CalibrValuesInt[0] = bmp085ReadInt(0xAA);
    bmp085CalibrValuesInt[1] = bmp085ReadInt(0xAC);
    bmp085CalibrValuesInt[2] = bmp085ReadInt(0xAE);
    bmp085CalibrValuesUInt[0] = (uint16_t) bmp085ReadInt(0xB0);
    bmp085CalibrValuesUInt[1] = (uint16_t) bmp085ReadInt(0xB2);
    bmp085CalibrValuesUInt[2] = (uint16_t) bmp085ReadInt(0xB4);
    bmp085CalibrValuesInt[3] = bmp085ReadInt(0xB6);
    bmp085CalibrValuesInt[4] = bmp085ReadInt(0xB8);
    bmp085CalibrValuesInt[5] = bmp085ReadInt(0xBA);
    bmp085CalibrValuesInt[6] = bmp085ReadInt(0xBC);
    bmp085CalibrValuesInt[7] = bmp085ReadInt(0xBE);

}

int32_t bmp085GetPressure(int32_t up) {
    int32_t x1, x2, x3, b3, b6, p;
    uint32_t b4, b7;

    b6 = bmp085Temperature - 4000;


    // Calculate B3
    x1 = (bmp085CalibrValuesInt[4] * ((b6 * b6) >> 12)) >> 11;
    x2 = (bmp085CalibrValuesInt[1] * b6) >> 11;
    x3 = x1 + x2;
    b3 = (((((int32_t) (bmp085CalibrValuesInt[0]) * 4) + x3) << OSS) + 2) >> 2;

    // Calculate B4
    x1 = (bmp085CalibrValuesInt[2] * b6) >> 13;
    x2 = (bmp085CalibrValuesInt[3] * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (bmp085CalibrValuesUInt[0] * (uint32_t) (x3 + 32768)) >> 15;

    b7 = ((uint32_t) ((uint32_t) (up) - (uint32_t) (b3)) * (uint32_t) (50000 >> OSS));
    if (b7 < 0x80000000)
        p = (int32_t) ((b7 << 1) / b4);
    else
        p = (int32_t) ((b7 / b4) << 1);

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return p;
}

float bmp085calcAltitude(int32_t pressure) {

    float p0 = 101325;
    float altitude = (float) 44330 * (1 - pow(((float) pressure / p0), 0.190295));

    return altitude;
}

// Read the uncompensated pressure value
int32_t bmp085ReadUP(void) {
    int32_t up = 0;

    uint8_t cmdWrite[2] = {bmp085WriteAddr, 0xF4};
    uint8_t msgWrite[1] = {0x34 + (OSS << 6)};


    uint8_t cmdSetReadingPoint[2] = {bmp085WriteAddr, 0xF6};
    uint8_t msgSetReadingPoint[1] = {0};

    uint8_t cmdRead[1] = {bmp085ReadAddr};
    uint8_t msgRead[3] = {0};

    // Write 0x34+(OSS<<6) into register 0xF4
    // Request a pressure reading w/ oversampling setting
    I2CDo(msgWrite, 1, cmdWrite, 2);

    // Wait for conversion, delay time dependent on OSS
    _delay_ms(2 + (3 << OSS));

    // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)

    I2CDo(msgSetReadingPoint, 0, cmdSetReadingPoint, 2);
    I2CDo(msgRead, 3, cmdRead, 1);

    up = (((int32_t) msgRead[0] << 16) | ((int32_t) msgRead[1] << 8) | (int32_t) msgRead[2]) >> (8 - OSS);

    return up;
}


int16_t bmp085GetTemperature(int16_t ut) {
    int32_t x1, x2;

    x1 = (((int32_t) ut - (int32_t) bmp085CalibrValuesUInt[2]) * (int32_t) bmp085CalibrValuesUInt[1]) >> 15;
    x2 = ((int32_t) bmp085CalibrValuesInt[6] << 11) / (x1 + bmp085CalibrValuesInt[7]);
    bmp085Temperature = x1 + x2;

    return ((bmp085Temperature + 8) >> 4);
}

// Read the uncompensated temperature value
int16_t bmp085ReadUT(void) {
    int16_t ut;
    uint8_t cmdWrite[2] = {bmp085WriteAddr, 0xF4};
    uint8_t msgWrite[1] = {0x2E};

    I2CDo(msgWrite, 1, cmdWrite, 2);

    _delay_ms(5);

    // Read two bytes from registers 0xF6 and 0xF7
    ut = bmp085ReadInt(0xF6);
    return ut;
}


int16_t bmp085ReadInt(uint8_t registerAddr) {
    int16_t value;
    uint8_t cmdSetReadingPoint[2] = {bmp085WriteAddr, registerAddr};
    uint8_t msgSetReadingPoint[1] = {0};

    uint8_t cmdRead[1] = {bmp085ReadAddr};
    uint8_t msgRead[2] = {0};

    I2CDo(msgSetReadingPoint, 0, cmdSetReadingPoint, 2);
    I2CDo(msgRead, 2, cmdRead, 1);

    value = (int16_t) (msgRead[0] << 8) | msgRead[1];

    return value;
}

void formFloat(float number, uint16_t *arr) {

    if (number < 0) {
        number = -1 * number;
        arr[0] = 1;
    } else {
        arr[0] = 0;
    }
    arr[1] = (uint16_t) number;

    arr[2] = (uint16_t) (trunc((number - (uint16_t) number) * 100));
}


// Function sends commands to TB6612FNG, see datasheet for details
// provide 16 bit value for duty cycle of 1A or 1B PWM output in frequencyA,B variables
void MotorControl(uint8_t in1A, uint8_t in2A, uint16_t frequencyA, uint8_t in1B, uint8_t in2B, uint16_t frequencyB) {
    if (in1A == 1) {
        PORTC = PORTC | (1 << PORTC0);
    } else {
        PORTC = PORTC & ~(1 << PORTC0);
    }
    if (in2A == 1) {
        PORTC = PORTC | (1 << PORTC1);
    } else {
        PORTC = PORTC & ~(1 << PORTC1);
    }
    if (in1B == 1) {
        PORTC = PORTC | (1 << PORTC2);
    } else {
        PORTC = PORTC & ~(1 << PORTC2);
    }
    if (in2B == 1) {
        PORTC = PORTC | (1 << PORTC3);
    } else {
        PORTC = PORTC & ~(1 << PORTC3);
    }

    OCR1A = frequencyA; // set PWM for (frequencyA/16bit)*100 duty cycle @ 16bit

    OCR1B = frequencyB;  // set PWM for (frequencyB/16bit)*100 duty cycle @ 16bit




}


void USARTinit(void) {
    //Baud rate setup
    //UBRR=f/(16*band)-1 f=12MHz band=9600,
    UBRR0H = 0;
    UBRR0L = 77;
    //UBRR0L = 6;
    //normal async duplex mode
    UCSR0A = UCSR0A & ~(1 << U2X0) & ~(1 << MPCM0);

    //Rx interrupt and enable receive transmit for UART
    //HB for frame size is 0 - 8 bits frame size
    UCSR0B = UCSR0B | (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0B = UCSR0B & ~(1 << TXCIE0) & ~(1 << UDRIE0) & ~(1 << UCSZ02);

    //use asynchronous UART, no parity bits, 1 stop bit, 8 bits frame size LB,0 clock polarity
    UCSR0C = UCSR0C | (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0C =
            UCSR0C & ~(1 << UMSEL01) & ~(1 << UMSEL00) & ~(1 << UPM01) & ~(1 << UPM00) & ~(1 << USBS0) & ~(1 << UCPOL0);
}


uint8_t USARTReceive(void) {

    uint8_t chr;

    while (!(UCSR0A & (1 << RXC0)));
    chr = UDR0;

    return chr;
}


void USARTReceiveArr(uint8_t *msgPtr) {

    uint8_t i = 0;

    while ((UCSR0A & (1 << RXC0))) {
        msgPtr[i] = UDR0;
        i++;
    }
}


void messiUartTransmitStart(void) {

    USARTTransmit((unsigned char) MESSI_USART_START_CHR);
}

void messiUartTransmitEnd(void) {

    USARTTransmit((unsigned char) MESSI_USART_END_CHR);
}

void messiUartTransmit(unsigned char data) {

    if ((data == MESSI_USART_START_CHR) || (data == MESSI_USART_END_CHR)) {
        USARTTransmit((unsigned char) MESSI_USART_ESC_CHR);
        USARTTransmit(data);
    } else {
        USARTTransmit(data);
    }
}

void messiUartTransmitString(uint8_t *msgPtr, uint8_t msgSize) {

    for (uint8_t i = 0; i < msgSize; i++) {
        messiUartTransmit((unsigned char) msgPtr[i]);
    }
}

void USARTTransmitFloat(float number) {
    char tempS[10];
    int d1 = 0;
    int d2 = 0;
    char format[9] = "%+d.%02d";

    d1 = (int) number;

    if (number < 0) {
        number = -1 * number;
    }
    d2 = (int) (trunc((number - (int) number) * 100));

    sprintf(tempS, format, d1, d2);

    USARTTransmitString(tempS);

}

void USARTTransmitString(char *dataPtr) {

    while (*dataPtr != 0x00) {
        USARTTransmit((unsigned char) *dataPtr);
        dataPtr++;
    }
}

void USARTTransmit(unsigned char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    /* Put data into buffer, sends the data */
    UDR0 = data;
}


void I2CInit(void) {

    TWBR = 0x07;    // Set bit rate register (Baudrate).
    //TWBR = 0x02;
    TWDR = 0xFF; // Default content = SDA released.
    TWCR = TWCR | (1 << TWEN);
    TWCR = TWCR & ~(1 << TWINT) & ~(1 << TWSTA) & ~(1 << TWSTO) & ~(1 << TWEA);

}

void I2CStart(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
}

void I2CWait(void) {
    // wait for i2c interface to complete operation
    while (!(TWCR & (1 << TWINT)));
}

void I2CTransmit(uint8_t *msgPtr, uint8_t msgSize) {

    uint8_t i;

    for (i = 0; i < msgSize; i++) {
        I2CWait();

        TWDR = msgPtr[i];
        TWCR = (1 << TWINT) | (1 << TWEN);
    }
}

void I2CReceive(uint8_t *msgPtr, uint8_t msgSize) {

    uint8_t i;

    for (i = 0; i < msgSize; i++) {
        I2CWait();

        msgPtr[i] = TWDR;
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);

    }
}

void I2CACK(void) {
    I2CWait();

    TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);

}

void I2CNACK(void) {

    TWCR = (1 << TWINT) | (1 << TWEN);
    TWCR = TWCR & ~(1 << TWEA);

}

void I2CStop(void) {

    I2CWait();

    TWCR = (1 << TWINT) | (1 << TWEN) |
           (1 << TWSTO);

}

//Function sends or receives data via I2C
void I2CDo(uint8_t *msgPtr, uint8_t msgSize, uint8_t *readOrWriteCommand, uint8_t cmdSize) {

    I2CStart();
    //send via I2C type of a command
    I2CTransmit(readOrWriteCommand, cmdSize);
    // cmd[0] shall always contain type of command
    // if low bit is equal to 0 - do the write
    if ((readOrWriteCommand[0] & 0x01) == 0) {
        I2CTransmit(msgPtr, msgSize);
    } else {
        I2CACK();
        I2CReceive(msgPtr, msgSize);
    }
    I2CNACK();
    I2CStop();

}

//Timer interrupt
//executes every 100ms. First it loops 2 minutes to safeguard data recording start at the right time.
//After 2 mins it starts saving sensor data into the arrays

//Data is saved a bit differently to make sure it consumes as less memory as possible.
//Acceleration can be multiplied by 10, since the maximum expected acceleration shall not exceed 16g.
//this way we can save integer part and one decimal sign in one byte.
//Altitude requirement is to have +-3 meters error, so we can save data simply into one byte.
//Temperature shall be saved with +-0.2 degrees, so we substracting -100 units from 0.1C measurements
//(flight was in July).
ISR(TIMER1_OVF_vect) {
    uint8_t lAcceleration, lAltitude, lTemperature;
    //Resets counter to 100ms base
    TCNT1 = TIMERBASE;


    if (gTimerPremCounter >= 50) {
        if (gTimerCounter <  gTimeCountSum) {

            lAcceleration = (uint8_t)((int16_t)(adxl345CalcAccData[1])*10);

            lAltitude = (uint8_t)altitude;
            lTemperature = (uint8_t)(temperature - 100);

            gSensorUByteData[gTimerCounter * 2] = (uint8_t) (lAcceleration);
            gSensorUByteData[(gTimerCounter * 2) + 1] = (uint8_t) (lAltitude);
            gSensorTempData[gTimerCounter] = (uint8_t) (lTemperature);
            gTimerCounter++;
        } else {
            //save it once in the end of the counting into EEPROM
            //since there is no place for temperature data we don't save temperature
            if (gTimerCounter == gTimeCountSum) {
                write_eeprom_array(gEEPROMUByteArray, gSensorUByteData, sizeof(gEEPROMUByteArray));
                gTimerCounter++;
            }
        }
    } else {
        gTimerPremCounter++;
    }


}

//TODO: make all commands check by bit mask
//Interrupt from our communication device (XBEE)
//We send data from PC via XBEE
//UART interrupt receives commands (1 byte) and depending
//on the value command is interpreted
ISR(USART_RX_vect) {


    uint8_t msgReceive = 0;

    msgReceive = USARTReceive();
    //assign command or speed depending of the value received
    if (msgReceive < 128) {
        gCommand = msgReceive;
    } else {
        gSpeed = msgReceive;
    }

    //command execution
    //activate hotwire function
    if ((gCommand & 0x40) == 64) {
        activateHW();
    }

    //initialize motor, PWM for motor control if back or forward commands given
    if (((gCommand & 0x08) == 8) || ((gCommand & 0x04) == 4)) {
        gTimerCounter = 0;
        gTimerPremCounter = 0;
        MotorInit();
    }

    if ((gCommand & 0x10) == 16) {
        gTimerCounter = 0;
        gTimerPremCounter = 0;
        timerInit();
    }


    if ((gCommand & 0x20) == 32) {
        gSendEEPROMDataIdx = 0;
        read_eeprom_array(gEEPROMUByteArray, gSensorUByteData, sizeof(gEEPROMUByteArray));

    }


}
