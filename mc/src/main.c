#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>


//Normal mode
#define MODE_SM 1
//EEPROM mode
#define MODE_MM 2

#define PI 3.14528f

#define MESSI_USART_START_CHR 'M'
#define MESSI_USART_END_CHR 'E'
#define MESSI_USART_ESC_CHR 'x'

static void USARTinit(void);

static uint8_t USARTReceive(void);

static void USARTReceiveArr(uint8_t *msgPtr);

static void USARTTransmitString(char *dataPtr);

static void USARTTransmit(unsigned char data);

static void USARTTransmitFloat(float number);

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

static float bmp085GetTemperature(int16_t ut);

static int32_t bmp085ReadUP(void);

static int16_t bmp085ReadUT(void);

static int32_t bmp085GetPressure(int32_t up);

static float bmp085calcAltitude(int32_t pressure);

static int16_t bmp085ReadInt(uint8_t registerAddr);

static void mpu6050Init(void);

static void mpu6050ReadAcc(void);

static void mpu6050BiasCalc(void);

static void mpu6050CalcAcc(void);

static void activateHW(void);

//control
volatile uint8_t gMode = 0;
//
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


//MPU6050

static uint8_t mpu6050SensivityRegister = 0;

static uint8_t mpu6050WriteAddr = 0xD0;

static uint8_t mpu6050ReadAddr = 0xD1;

static int16_t mpu6050Sensivity = 16384;

static int16_t mpu6050AccData[3] = {0};

static int16_t mpu6050BiasData[3] = {375, 700, 1738};

static int16_t mpu6050BiasCycleCnt = 100;

static float mpu6050CalcAccData[3] = {0};



// macro for easier usage
#define read_eeprom_word(address) eeprom_read_word ((const uint16_t*)address)
#define write_eeprom_word(address, value) eeprom_write_word ((uint16_t*)address,(uint16_t)value)
#define update_eeprom_word(address, value) eeprom_update_word ((uint16_t*)address,(uint16_t)value)

#define read_eeprom_array(address, value_p, length) eeprom_read_block ((void *)value_p, (const void *)address, length)
#define write_eeprom_array(address, value_p, length) eeprom_write_block ((const void *)value_p, (void *)address, length)


//declare an eeprom array
//unsigned int EEMEM  my_eeprom_array[10];

// declare a ram array
//unsigned int my_ram_array[10];

//declare an eeprom array
unsigned int EEMEM my_eeprom_array[10];

// declare a ram array and initialize
unsigned int my_ram_array[5] = {1, 2, 3, 4, 5};

unsigned int my_ram_array2[5] = {6, 7, 8, 9, 10};

// declare another ram array
unsigned int my_other_ram_array[5];

char tempS[50];

int main(void) {


    float temperature;
    int32_t pressure;
    float altitude;

    uint16_t arrFloat[3];
    uint8_t leftWheelFwd, leftWheelBck, rightWheelFwd, rightWheelBck;

    uint16_t frequencyA, frequencyB;

    /**********************************************************/

    USARTinit();

    I2CInit();
    MotorInit();

    sei();

    mpu6050Init();

    bmp085Calibration();

    //mpu6050BiasCalc();
    //activateHW();

    _delay_ms(10);

    while (1) {

        messiUartTransmitStart();

//        sprintf(tempS, "%d", mpu6050AccData[0]);
//        USARTTransmit('x');
//        USARTTransmitString(tempS);
//
//        sprintf(tempS, "%d", mpu6050AccData[1]);
//        USARTTransmit('y');
//        USARTTransmitString(tempS);
//
//        sprintf(tempS, "%d", mpu6050AccData[2]);
//        USARTTransmit('z');
//        USARTTransmitString(tempS);
        //

//        temperature = bmp085GetTemperature(bmp085ReadUT());
//        pressure = bmp085GetPressure(bmp085ReadUP());
//        altitude = bmp085calcAltitude(pressure);
//        mpu6050ReadAcc();

//        formFloat(temperature, arrFloat);
//
//        msgUART[0] = (uint8_t) (arrFloat[0]);
//        msgUART[1] = (uint8_t) (arrFloat[1] >> 8);
//        msgUART[2] = (uint8_t) (arrFloat[1] & 0x00FF);
//
//        msgUART[3] = (uint8_t) (arrFloat[2] >> 8);
//        msgUART[4] = (uint8_t) (arrFloat[2] & 0x00FF);
//
//        formFloat(altitude, arrFloat);
//
//        msgUART[5] = (uint8_t) (arrFloat[0]);
//        msgUART[6] = (uint8_t) (arrFloat[1] >> 8);
//        msgUART[7] = (uint8_t) (arrFloat[1] & 0x00FF);
//
//        msgUART[8] = (uint8_t) (arrFloat[2] >> 8);
//        msgUART[9] = (uint8_t) (arrFloat[2] & 0x00FF);
//
//        formFloat(mpu6050CalcAccData[2], arrFloat);
//
//        msgUART[10] = (uint8_t) (arrFloat[0]);
//        msgUART[11] = (uint8_t) (arrFloat[1] >> 8);
//        msgUART[12] = (uint8_t) (arrFloat[1] & 0x00FF);
//
//        msgUART[13] = (uint8_t) (arrFloat[2] >> 8);
//        msgUART[14] = (uint8_t) (arrFloat[2] & 0x00FF);


        leftWheelFwd = ((gCommand == 5) || (gCommand == 7)) ? 1 : 0;
        leftWheelBck = ((gCommand == 9) || (gCommand == 11)) ? 1 : 0;
        rightWheelFwd = ((gCommand == 6) || (gCommand == 7)) ? 1 : 0;
        rightWheelBck = ((gCommand == 10) || (gCommand == 11)) ? 1 : 0;

        msgUART[15] = (gSpeed & 0x0C) >> 2;

        msgUART[16] = gSpeed & 0x03;

        switch (gSpeed & 0x0C) {
            case 0:
                frequencyA = 16000;
                break;
            case 4:
                frequencyA = 32000;
                break;
            case 8:
                frequencyA = 48000;
                break;
            case 12:
                frequencyA = 65000;
                break;
            default:
                frequencyA = 0;
                break;
        }

        switch (gSpeed & 0x03) {
            case 0:
                frequencyB = 16000;
                break;
            case 1:
                frequencyB = 32000;
                break;
            case 2:
                frequencyB = 48000;
                break;
            case 3:
                frequencyB = 65000;
                break;
            default:
                frequencyB = 0;
                break;
        }

        MotorControl(leftWheelFwd, leftWheelBck, frequencyA, rightWheelFwd, rightWheelBck, frequencyB);


        messiUartTransmitString(msgUART, 38);

        messiUartTransmitEnd();

    }

    return 0;

}

void activateHW(void) {
    DDRD = DDRD | (1 << DDD5);

    PORTD = PORTD | (1 << PORTD5);

    _delay_ms(5000);

    PORTD = PORTD & ~(1 << PORTD5);

}


void mpu6050Init(void) {
    uint8_t cmdWrite[2] = {mpu6050WriteAddr, 0x6B};
    uint8_t msgWrite[1] = {0};

    uint8_t cmdWriteSens[2] = {mpu6050WriteAddr, 0x1C};
    //TODO: write 16g
    uint8_t msgWriteSens[1] = {mpu6050SensivityRegister};

    I2CDo(msgWrite, 1, cmdWrite, 2);
    //write sensivity value
    I2CDo(msgWriteSens, 1, cmdWriteSens, 2);

}

void mpu6050BiasCalc(void) {
    int32_t temp_bias_32_data[3] = {0, 0, 0};

    //read data 100 times @ 10msec interval and compute average value
    for (uint8_t count = 0; count < mpu6050BiasCycleCnt; count++) {
        mpu6050ReadAcc();
        temp_bias_32_data[0] += mpu6050AccData[0];
        temp_bias_32_data[1] += mpu6050AccData[1];
        temp_bias_32_data[2] += mpu6050AccData[2];
        _delay_ms(10);

    }

    mpu6050BiasData[0] = (int16_t) (temp_bias_32_data[0] / mpu6050BiasCycleCnt);
    mpu6050BiasData[1] = (int16_t) (temp_bias_32_data[1] / mpu6050BiasCycleCnt);
    mpu6050BiasData[2] = (int16_t) (temp_bias_32_data[2] / mpu6050BiasCycleCnt);


}

void mpu6050ReadAcc(void) {
    uint8_t cmdSetReadingPoint[2] = {mpu6050WriteAddr, 0x3B};
    uint8_t msgSetReadingPoint[1] = {0};

    uint8_t cmdRead[1] = {mpu6050ReadAddr};
    uint8_t msgRead[14] = {0};

    I2CDo(msgSetReadingPoint, 0, cmdSetReadingPoint, 2);
    I2CDo(msgRead, 14, cmdRead, 1);

    mpu6050AccData[0] = (msgRead[0] << 8) | msgRead[1];
    mpu6050AccData[1] = (msgRead[2] << 8) | msgRead[3];
    mpu6050AccData[2] = (msgRead[4] << 8) | msgRead[5];

}

void mpu6050CalcAcc(void) {
    mpu6050CalcAccData[0] = (mpu6050AccData[0] - mpu6050BiasData[0]) / mpu6050Sensivity;
    mpu6050CalcAccData[1] = (mpu6050AccData[1] - mpu6050BiasData[1]) / mpu6050Sensivity;
    mpu6050CalcAccData[2] = (mpu6050AccData[2] - mpu6050BiasData[2]) / mpu6050Sensivity;
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


float bmp085GetTemperature(int16_t ut) {
    int32_t x1, x2;

    x1 = (((int32_t) ut - (int32_t) bmp085CalibrValuesUInt[2]) * (int32_t) bmp085CalibrValuesUInt[1]) >> 15;
    x2 = ((int32_t) bmp085CalibrValuesInt[6] << 11) / (x1 + bmp085CalibrValuesInt[7]);
    bmp085Temperature = x1 + x2;

    return ((bmp085Temperature + 8) >> 4) / 10;
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


void PWMInit(void) {

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

void PWMSwitch(uint8_t isOn) {
    if (isOn == 0) {
        //switch off wave generation
        TCCR1A = TCCR1A & ~(1 << COM1A1) & ~(1 << COM1B1);
    } else {
        TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
    }
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
    //UBRR0L = 2;
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

ISR(USART_RX_vect) {


    uint8_t msgReceive = 0;

    msgReceive = USARTReceive();

    USARTTransmit((unsigned char) msgReceive);

    if (msgReceive < 128) {
        gCommand = msgReceive;
    } else {
        gSpeed = msgReceive;
    }

    // if ((chr & 0x01) == ) {

    //}

    /*switch (msgReceive) {
        case '0':
            chr = 0;
            break;
        case '1':
            chr = 1;
            break;
        case '2':
            chr = 2;
            break;
        case '3':
            chr = 3;
            break;
        case '4':
            chr = 4;
            break;
        case '5':
            chr = 5;
            break;
        case '6':
            chr = 6;
            break;
        case '7':
            chr = 7;
            break;
        case '8':
            chr = 8;
            break;
        case '9':
            chr = 9;
            break;
        default:
            chr = msgReceive;
            break;
    }


    if (((chr >= 0) && (chr <= 9)) || (chr == '-') || (chr == '+') || (chr == 'M') || (chr == 'E')) {
        if (chr == 'M') {
            gMsgReceiveIdx = 0;
            gMsgReceive[2] = 0;
            gMsgReceive[3] = 0;
            gMsgReceive[1] = 0;
        }
        else if (chr == 'E') {
            gMode = (uint8_t) gMsgReceive[1];//chr;
        } else {
            if (gMsgReceiveIdx > 0) {
                if (gMsgReceiveIdx == 1) {
                    if (chr == '-') {
                        gMsgReceive[3] = -1;
                    } else {
                        gMsgReceive[3] = 1;
                    }
                }
                if (gMsgReceiveIdx == 2) {
                    gMsgReceive[2] = 100 * chr;
                }
                if (gMsgReceiveIdx == 3) {
                    gMsgReceive[2] = gMsgReceive[2] + 10 * chr;
                }
                if (gMsgReceiveIdx == 4) {
                    gMsgReceive[2] = gMsgReceive[2] + chr;
                }
            }
            else {
                gMsgReceive[1] = chr;
            }

            gMsgReceiveIdx++;
        }
    }*/
    //start writing by command and continue only for
//    if (1 == 0) {
//        // Write data to EEPROM
//        write_eeprom_array(my_eeprom_array,my_ram_array,sizeof(my_eeprom_array));
//        // Read data and send all data
//    }

}
