#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

static void USARTinit(void);

static void USARTReceive(char *dataPtr);

static void USARTTransmitString(char *dataPtr);

static void USARTTransmit(unsigned char data);


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

#define TW_CMD_MAGN_WRITE 0x3C

#define TW_CMD_MAGN_READ 0x3D


int main(void) {

    USARTinit();
    I2CInit();

    uint8_t i = 0;
    char str[10] = "Hello!";
    char tempS[50];
    int16_t x,y;
    double pi = 3.14528;
    double heading;
    int16_t headingDegrees;

    //FDA command to test FDA
    uint8_t msgFDAWrite[] = {0x62, 0x03, 0x63, 0x00, 0x64, 0x02};


    uint8_t cmdMagnWriteCRA[2] = {TW_CMD_MAGN_WRITE, 0x00};
    uint8_t msgMagnWriteCRA[1] = {0x70};

    uint8_t cmdMagnWriteCRB[2] = {TW_CMD_MAGN_WRITE, 0x01};
    uint8_t msgMagnWriteCRB[1] = {0xA0};

    //Magnetometer command to set continuous mode
    uint8_t cmdMagnWrite[2] = {TW_CMD_MAGN_WRITE, 0x02};
    uint8_t msgMagnWrite[1] = {0x00};

    //Magnetometer command to set to the begin of measurements
    uint8_t cmdMagnSetReadingPoint[2] = {TW_CMD_MAGN_WRITE, 0x03};
    uint8_t msgMagnSetReadingPoint[1] = {0};

    //must read 6 values before going to the next measurement
    uint8_t cmdMagnRead[1] = {TW_CMD_MAGN_READ};
    uint8_t msgMagnRead[6] = {0};


    uint8_t tmpCmd[2] = {TW_CMD_MAGN_READ, 0x02};
    uint8_t msgMagnReadTemp[1] = {0};


//    I2CDo(msgMagnWriteCRA, 1, cmdMagnWriteCRA);

 //   I2CDo(msgMagnWriteCRB, 1, cmdMagnWriteCRB);

    //set Magnetometer to continuous mode
    I2CDo(msgMagnWrite, 1, cmdMagnWrite, 2);

    _delay_ms(10);
    while (1) {


        I2CDo(msgMagnSetReadingPoint, 0, cmdMagnSetReadingPoint, 2);
        I2CDo(msgMagnRead, 6, cmdMagnRead, 1);


        _delay_ms(100);


        //I2CDo(msgMagnReadTemp, 1, tmpCmd);

        USARTTransmit((unsigned char) 'M');

        USARTTransmit((unsigned char) 'X');
        //tmp16 =
        //sprintf(tempS, '%d', tmp16Ptr);
        USARTTransmit((unsigned char)msgMagnRead[0]);
        USARTTransmit((unsigned char)msgMagnRead[1]);
        x = (int16_t) (msgMagnRead[0] << 8) | msgMagnRead[1];

        //USARTTransmit((unsigned char) 'x');
        //USARTTransmit((unsigned char)msgMagnRead[1]);

        //USARTTransmit((unsigned char) 'Z');
        //sprintf(tempS, '%d', (int16_t) (msgMagnRead[2] << 8) | msgMagnRead[3]);
        //USARTTransmitString(tempS);

        //USARTTransmit((unsigned char) 'z');
        //USARTTransmit((unsigned char)magnetometerRead(msgMagnRead, 6, cmdMagnRead));
        //USARTTransmit((unsigned char)msgMagnRead[3]);

        USARTTransmit((unsigned char) 'Y');
        //sprintf(tempS, '%d', (int16_t) (msgMagnRead[4] << 8) | msgMagnRead[5]);
        USARTTransmit((unsigned char)msgMagnRead[4]);
        USARTTransmit((unsigned char)msgMagnRead[5]);
        y = (int16_t) (msgMagnRead[4] << 8) | msgMagnRead[5];

        heading = atan2(y, x);
        headingDegrees = (int16_t)(heading * (180/pi));
        headingDegrees = headingDegrees + 180;
        USARTTransmit((unsigned char) 'H');
        sprintf(tempS, '%d', (int16_t) (msgMagnRead[4] << 8) | msgMagnRead[5]);
        USARTTransmit((unsigned char)(headingDegrees >> 8));
        USARTTransmit((unsigned char)(headingDegrees & 0x00FF));

        //USARTTransmit((unsigned char) 'y');
        //USARTTransmit((unsigned char)msgMagnRead[5]);


        //USARTTransmit((unsigned char) msgMagnReadTemp[0]);

        i++;
    }

    return 0;

}

void USARTinit(void) {
    //Baud rate setup
    //UBRR=f/(16*band)-1 f=1000000?? band=9600,
    UBRR0H = 0;
    UBRR0L = 6;
    //normal async duplex mode
    UCSR0A = UCSR0A & ~(1 << U2X0) & ~(1 << MPCM0);

    //no interrupts and enable receive transmit for UART
    //HB for frame size is 0 - 8 bits frame size
    UCSR0B = UCSR0B | (1 << RXEN0) | (1 << TXEN0);
    UCSR0B = UCSR0B & ~(1 << RXCIE0) & ~(1 << TXCIE0) & ~(1 << UDRIE0) & ~(1 << UCSZ02);

    //use asynchronous UART, no parity bits, 1 stop bit, 8 bits frame size LB,0 clock polarity
    UCSR0C = UCSR0C | (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0C =
            UCSR0C & ~(1 << UMSEL01) & ~(1 << UMSEL00) & ~(1 << UPM01) & ~(1 << UPM00) & ~(1 << USBS0) & ~(1 << UCPOL0);
}

void USARTReceive(char *dataPtr) {

    while (*dataPtr != 0x00) {
        /* Wait for empty transmit buffer */
        while (!(UCSR0A & (1 << UDRE0)));
        /* Put data into buffer, sends the data */
        *dataPtr = (unsigned char) UDR0;
        dataPtr++;

    }
}


void USARTTransmitString(char *dataPtr) {

    while (*dataPtr != 0x00) {
        /* Wait for empty transmit buffer */
        USARTTransmit((unsigned char) *dataPtr);
        dataPtr++;
    }
}

static void USARTTransmit(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)));
    /* Put data into buffer, sends the data */
    UDR0 = data;
}



void I2CInit(void) {

    TWBR = 2;    // Set bit rate register (Baudrate). Defined in header file.
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

    TWCR = (1 << TWINT) |(1 << TWEA) | (1 << TWEN);

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
