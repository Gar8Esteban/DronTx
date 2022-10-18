#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

//********************************Comandos nRF24***************
#define NOP 0xFF
#define PWR_UP 0x0A
#define CHANNEL 0x5A
#define TX_PAYLOAD 0xA0
#define RX_PAYLOAD 0x61

//********************************Registros nRF24**************
#define CONFIG 0x00
#define EN_AA 0x01
#define STATUS 0x07
#define RF_CH 0x05 
#define RX_ADDR 0x0A
#define TX_ADDR 0x10
#define RX_PW 0x11
#define FIFO_STATUS 0x17
 
 //*******************************funciones********************
void csLow();
void csHigh();
void ceLow();
void ceHigh();
uint8_t leerReg(spi_inst_t *spi, uint8_t reg);
void escribirReg(spi_inst_t *spi, uint8_t reg, uint8_t data);
void config(spi_inst_t *spi);
void leerBytes(spi_inst_t *spi, uint8_t reg, char *msg, uint8_t size);
void ModoTx(spi_inst_t* spi);
void ModoRx(spi_inst_t* spi);
void enviarMsg(spi_inst_t* spi, char *msg);
void recirbirMsg(spi_inst_t* spi, char *msg);
uint8_t nuevoMsg(spi_inst_t* spi);

//********************************Variables Globales***********
const int sck_pin = 10;
const int mosi_pin = 11;
const int miso_pin = 12;
const int cs_pin = 13; // cns
const int ce_pin = 22; // RX__TX

//********************************I2C**************************
static int addr = 0x68;
#define I2C_PORT i2c0

//********************************Funciones I2c****************

static void mpu9250_reset();
void mpu9250_read_raw_accel(int16_t accel[3]);
void mpu9250_read_raw_gyro(int16_t gyro[3]);
void calibrate_gyro(int16_t gyroCal[3], int loop);
void calculate_angles_from_accel(int16_t eulerAngles[2], int16_t accel[3]);
void calculate_angles(int16_t eulerAngles[2], int16_t accel[3], int16_t gyro[3], uint64_t usSinceLastReading);
void convert_to_full(int16_t eulerAngles[2], int16_t accel[3], int16_t fullAngles[2]);

int main() {
    sleep_ms(100);
    // ***************************************Configuracion antena y spi************
    spi_inst_t *spi = spi1;

    // Initialize chosen serial port
    stdio_init_all();

    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    gpio_init(ce_pin);
    gpio_set_dir(ce_pin, GPIO_OUT);
    gpio_put(ce_pin, 0);

    // Inicializar el spi a 10MHz
    spi_init(spi, 10*1000*1000);

    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(mosi_pin, GPIO_FUNC_SPI);
    gpio_set_function(miso_pin, GPIO_FUNC_SPI);

    config(spi);
    ModoTx(spi);

    // Configuración del i2c y mpu******************************************************************

    int16_t acceleration[3],gyro[3], gyroCal [3] , eulerAngles [2] , fullAngles [2]; // Declaración de variables para las funciones
    absolute_time_t timeOfLastCheck;

    i2c_init(I2C_PORT, 400000); //400kHz
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);

    mpu9250_reset();

    // Llama a la función del acelerómetro
    mpu9250_read_raw_accel (acceleration) ; // Sets the absolute angle using the direction of gravity
    calculate_angles_from_accel (eulerAngles , acceleration);
    timeOfLastCheck = get_absolute_time();

    calibrate_gyro(gyroCal, 100); //Calibra el gyro

    char buffer[32];
    // Loop forever
    int cont = 0;
    while (true) {

        mpu9250_read_raw_accel(acceleration) ;// Reads the accel and gyro
        mpu9250_read_raw_gyro (gyro) ;

        gyro [ 0 ] -= gyroCal [ 0 ] ; // Aplica la calibración
        gyro [ 1 ] -= gyroCal [ 1 ] ;
        gyro [ 2 ] -= gyroCal [ 2 ] ;

        calculate_angles ( eulerAngles , acceleration , gyro , absolute_time_diff_us ( timeOfLastCheck , get_absolute_time())); // Calculates the angle
        timeOfLastCheck = get_absolute_time ();
        convert_to_full (eulerAngles , acceleration , fullAngles);

        sprintf(buffer,"%d %d %d %d %d %d" ,acceleration[0], acceleration[1], acceleration[2], gyro[0], gyro[1], gyro[2]);
        //printf("\nCadena:%s", valor1);
        enviarMsg(spi, buffer);
        
        if(cont >= 9){
            cont = 0;
        }else{
            cont++;
        }
        sleep_ms(100);
    }
    return 0;
}

//***************************** contexto de las funciones******


void csLow(){
    gpio_put(cs_pin, 0);
}

void csHigh(){
    gpio_put(cs_pin, 1);
}

void ceLow(){
    gpio_put(ce_pin, 0);
}

void ceHigh(){
    gpio_put(ce_pin, 1);
}

uint8_t leerReg(spi_inst_t *spi, uint8_t reg){
    uint8_t resultado = 0x00 | (0x1F & reg);
    csLow();
    spi_write_blocking(spi, &reg, 1);
    spi_read_blocking(spi, NOP, &resultado, 1);
    csHigh();

    return resultado;
}

void leerBytes(spi_inst_t *spi, uint8_t reg, char *msg, uint8_t size){
    reg = 0x00 | (0x1F & reg);
    csLow();
    spi_write_blocking(spi, &reg, 1);
    spi_read_blocking(spi, NOP, (uint8_t*)msg, size);
    csHigh();
}

void escribirReg(spi_inst_t *spi, uint8_t reg, uint8_t data){
    reg = 0x20 | (0x1F & reg);
    csLow();
    spi_write_blocking(spi, &reg, 1);
    spi_write_blocking(spi, &data, 1);
    csHigh();
}

void escribirRegBytes(spi_inst_t *spi, uint8_t reg, uint8_t *data, uint8_t size){
    reg = 0x20 | (0x1F & reg);
    csLow();
    spi_write_blocking(spi, &reg, 1);
    spi_write_blocking(spi, (uint8_t*)data, size);
    csHigh();
}

void config(spi_inst_t *spi){
    ceLow();
    csHigh();
    sleep_ms(11);

    escribirReg(spi, CONFIG, PWR_UP);
    sleep_us(1500);
    
    escribirReg(spi, EN_AA, 0x00); // shockburst desactivado
    
    escribirReg(spi, RF_CH, CHANNEL); // canal de radio frecuencia #90 

    escribirRegBytes(spi, RX_ADDR, (uint8_t*)"beast", 5);

    escribirRegBytes(spi, TX_ADDR, (uint8_t*)"beast", 5);

    escribirReg(spi, RX_PW, 32);
    
}

void ModoTx(spi_inst_t* spi){
    uint8_t reg = leerReg(spi, CONFIG);
    reg &= ~(1<<0);
    escribirReg(spi, CONFIG, reg);
    sleep_us(130);
}

void ModoRx(spi_inst_t* spi){
    uint8_t reg = leerReg(spi, CONFIG);
    reg &= ~(1<<0);
    reg |= (1<<0);
    escribirReg(spi, CONFIG, reg);
    ceLow();
    ceHigh();
    sleep_us(130);
}

void enviarMsg(spi_inst_t* spi, char *msg){
    uint8_t cmd = TX_PAYLOAD;
    csLow();
    spi_write_blocking(spi, &cmd, 1);
    spi_write_blocking(spi, (uint8_t*)msg, 32);
    csHigh();

    ceHigh();
    sleep_us(10);
    ceLow();
}

void recirbirMsg(spi_inst_t* spi, char *msg){
    uint8_t cmd = RX_PAYLOAD;
    csLow();
    spi_write_blocking(spi, &cmd, 1);
    spi_read_blocking(spi, NOP, (uint8_t*)msg, 32);
    csHigh();
}

uint8_t nuevoMsg(spi_inst_t* spi){
    uint8_t fifo_status = leerReg(spi, FIFO_STATUS) & 0x01;
    return !fifo_status;
}

// COntexto de las funciones del MPU ************************************************

static void mpu9250_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}



void mpu9250_read_raw_accel(int16_t accel[3]) { //Used to get the raw acceleration values from the mpu
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

void mpu9250_read_raw_gyro(int16_t gyro[3]) {  //Used to get the raw gyro values from the mpu
    uint8_t buffer[6];
    
    uint8_t val = 0x43;
    i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);  // False - finished with bus


    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
}

void calibrate_gyro(int16_t gyroCal[3], int loop)  //Used to calibrate the gyro. The gyro must be still while calibration happens
{
    int16_t temp[3];
    for (int i = 0; i < loop; i++)
    {
        mpu9250_read_raw_gyro(temp);
        gyroCal[0] += temp[0];
        gyroCal[1] += temp[1];
        gyroCal[2] += temp[2];
    }
    gyroCal[0] /= loop;
    gyroCal[1] /= loop;
    gyroCal[2] /= loop;
}

void calculate_angles_from_accel(int16_t eulerAngles[2], int16_t accel[3]) //Uses just the direction gravity is pulling to calculate angles.
{
    float accTotalVector = sqrt((accel[0] * accel[0]) + (accel[1] * accel[1]) + (accel[2] * accel[2]));

    float anglePitchAcc = asin(accel[1] / accTotalVector) * 57.296;
    float angleRollAcc = asin(accel[0] / accTotalVector) * -57.296;

    eulerAngles[0] = anglePitchAcc;
    eulerAngles[1] = angleRollAcc;
}

void calculate_angles(int16_t eulerAngles[2], int16_t accel[3], int16_t gyro[3], uint64_t usSinceLastReading) //Calculates angles based on the accelerometer and gyroscope. Requires usSinceLastReading to use the gyro.
{
    long hertz = 1000000/usSinceLastReading;
    
    if (hertz < 200)
    {
        calculate_angles_from_accel(eulerAngles, accel);
        return;
    }

    long temp = 1.l/(hertz * 65.5l);  

    eulerAngles[0] += gyro[0] * temp;
    eulerAngles[1] += gyro[1] * temp;

    eulerAngles[0] += eulerAngles[1] * sin(gyro[2] * temp * 0.1f);
    eulerAngles[1] -= eulerAngles[0] * sin(gyro[2] * temp * 0.1f);

    int16_t accelEuler[2];
    calculate_angles_from_accel(accelEuler, accel);

    eulerAngles[0] = eulerAngles[0] * 0.9996 + accelEuler[0] * 0.0004;
    eulerAngles[1] = eulerAngles[1] * 0.9996 + accelEuler[1] * 0.0004;
}


void convert_to_full(int16_t eulerAngles[2], int16_t accel[3], int16_t fullAngles[2]) //Converts from -90/90 to 360 using the direction gravity is pulling
{
    if (accel[1] > 0 && accel[2] > 0) fullAngles[0] = eulerAngles[0];
    if (accel[1] > 0 && accel[2] < 0) fullAngles[0] = 180 - eulerAngles[0];
    if (accel[1] < 0 && accel[2] < 0) fullAngles[0] = 180 - eulerAngles[0];
    if (accel[1] < 0 && accel[2] > 0) fullAngles[0] = 360 + eulerAngles[0];

    if (accel[0] < 0 && accel[2] > 0) fullAngles[1] = eulerAngles[1];
    if (accel[0] < 0 && accel[2] < 0) fullAngles[1] = 180 - eulerAngles[1];
    if (accel[0] > 0 && accel[2] < 0) fullAngles[1] = 180 - eulerAngles[1];
    if (accel[0] > 0 && accel[2] > 0) fullAngles[1] = 360 + eulerAngles[1];
}