/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

#define mpu_spi hspi2
#define mpu_spi_select() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define mpu_spi_deselect() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#include "MPU9250.h"


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
extern "C" void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

uint8_t readByte(uint8_t addr)
{
  uint8_t result;
  addr |= 0x80;
  HAL_SPI_Transmit(&hspi2, &addr, 1, 1000);
  HAL_SPI_Receive(&hspi2, &result, 1, 1000);
  return result;
}

void writeByte(uint8_t addr, uint8_t data)
{
  uint8_t buf[2] = {addr, data};
  HAL_SPI_Transmit(&hspi2, buf, 2, 1000);
}

int t;

/*void initMPU()
{
  uint8_t i = readByte(0x75); // 0x71
  uint8_t pw = readByte(0x6B);
  writeByte(0x6B PWR_MGMT_1, 0x00);
  for(int x = 0; x < 0x1000; ++x);
  
  writeByte(0x1A CONFIG, 0x03);
  
  writeByte(0x19 SMPLRT_DIV, 0x04);
  
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(GYRO_CONFIG);
  writeByte(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
 // Set accelerometer configuration
  c =  readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  writeByte(ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(ACCEL_CONFIG2);
  writeByte(ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  writeByte(ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(INT_PIN_CFG, 0x22);
   writeByte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt

   writeByte(I2C_SLV4_ADDR, AK8963_ADDRESS); // Address of mag on I2C bus
   writeByte(I2C_MST_CTRL, I2C_MST_CTRL|0x0D); // Speed of I2C 400kHz  
   
   //writeByte(FIFO_EN, 0x78); // ACCEL & GYRO
}

void readFIFO()
{
  uint8_t result[10];
  uint8_t addr = 0x80 | FIFO_R_W;
  HAL_SPI_Transmit(&hspi2, &addr, 1, 1000);
  HAL_SPI_Receive(&hspi2, result, 10, 1000);
  
}

void readAccelData(int16_t * dest)
{
  dest[0] = (readByte(ACCEL_XOUT_H) << 8) | readByte(ACCEL_XOUT_L);
  dest[1] = (readByte(ACCEL_YOUT_H) << 8) | readByte(ACCEL_YOUT_L);
  dest[2] = (readByte(ACCEL_ZOUT_H) << 8) | readByte(ACCEL_ZOUT_L);
}*/
 
/*void readGyroData(uint16_t sensor, int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(sensor, MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
}
 
void readMagData(uint16_t sensor, int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if(readByte(sensor, AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
  readBytes(sensor, AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  uint8_t c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
    destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
   }
  }
}*/

MPU9250 mpu9250;

void uprintf( const char* format, ... ) {
    va_list args;
    va_start( args, format );
    char buf[128];
    vsprintf( buf, format, args );
    va_end( args );
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 1000);
}

float sum = 0;
uint32_t sumCount = 0;

#define BIT_H_RESET 0x80
#define BITS_FS_250DPS 0x00
#define BITS_FS_2G 0x00

void initAK()
{
    uint8_t MPU_Init_Data[][2] = {
        {BIT_H_RESET, PWR_MGMT_1},     // Reset Device
        {0x01, PWR_MGMT_1},     // Clock Source
        {0x00, PWR_MGMT_2},     // Enable Acc & Gyro
        {0x01, CONFIG},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {BITS_FS_250DPS, GYRO_CONFIG},    // +-250dps
        {BITS_FS_2G, ACCEL_CONFIG},   // +-2G
        {0x01, ACCEL_CONFIG2}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, INT_PIN_CFG},    //
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, USER_CTRL},       // I2C Master mode
        {0x0D, I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz
        
        {AK8963_ADDRESS,I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2, I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, I2C_SLV0_DO}, // Reset AK8963
        {0x81, I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
#ifdef AK8963FASTMODE
        {0x16, I2C_SLV0_DO}, // Register value to 100Hz continuous measurement in 16bit
#else
        {0x12, I2C_SLV0_DO}, // Register value to 8Hz continuous measurement in 16bit
#endif
        {0x81, I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
        
    };

    for(int i = 0; i < sizeof(MPU_Init_Data) / sizeof(MPU_Init_Data[0]); i++) {
        mpu9250.writeByte(MPU_Init_Data[i][1], MPU_Init_Data[i][0]);
        wait(0.01);  //I2C must slow down the write speed, otherwise it won't work
    } 
}

uint8_t AK8963_whoami()
{
    uint8_t response;
    mpu9250.writeByte(I2C_SLV0_ADDR,AK8963_ADDRESS|READ_FLAG); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250.writeByte(I2C_SLV0_REG, WHO_AM_I_AK8963); //I2C slave 0 register address from where to begin data transfer
    mpu9250.writeByte(I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer

    //WriteReg(MPUREG_I2C_SLV0_CTRL, 0x81);    //Enable I2C and set bytes
    wait(0.01);
    response = mpu9250.readByte(EXT_SENS_DATA_00);    //Read I2C 
    //ReadRegs(MPUREG_EXT_SENS_DATA_00,response,1);
    //response=WriteReg(MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C 

    return response;
}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*char cmd[] = "AT+UART=115200,0,0\r\n";
  HAL_UART_Transmit_DMA(&huart2, (uint8_t *) cmd, sizeof(cmd) - 1);
  HAL_UART_Receive_DMA(&huart2, (uint8_t *) &y, sizeof(y));  */
  
  //#define MPU9250_ADDRESS (0x68 << 1)
  //uint8_t data[2] = {0x75 | 0x80, 0};
  //uint8_t res = readByte(0x75);//[2] = {0, 0};
  //initMPU();
  //HAL_SPI_TransmitReceive(&hspi2, data, res, 2, 1000);
  //HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDRESS, &data, sizeof(data), 1000);
  //HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDRESS, &res, sizeof(res), 1000);
    wait(0.1);
    uint8_t whoami = mpu9250.readByte(WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
    
    /*uint8_t p1 = mpu9250.readByte(PWR_MGMT_1);
    
    mpu9250.writeByte(PWR_MGMT_1, 0x80);
    
    mpu9250*/
    //p1 = mpu9250.readByte(PWR_MGMT_1);
    //uint8_t p2 = mpu9250.readByte(PWR_MGMT_2);
    
    
    //mpu9250.writeByte(USER_CTRL, 
    if(whoami == 0x71)
    {
        wait(0.01);
        mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
        mpu9250.MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    
        /*uprintf("x-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[0]);  
        uprintf("y-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[1]);  
        uprintf("z-axis self test: acceleration trim within : %f % of factory value\n\r", SelfTest[2]);  
        uprintf("x-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[3]);  
        uprintf("y-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[4]);  
        uprintf("z-axis self test: gyration trim within : %f % of factory value\n\r", SelfTest[5]);  */
        //mpu9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        /*uprintf("x gyro bias = %f\n\r", gyroBias[0]);
        uprintf("y gyro bias = %f\n\r", gyroBias[1]);
        uprintf("z gyro bias = %f\n\r", gyroBias[2]);
        uprintf("x accel bias = %f\n\r", accelBias[0]);
        uprintf("y accel bias = %f\n\r", accelBias[1]);
        uprintf("z accel bias = %f\n\r", accelBias[2]);*/
        wait(2);

        initAK();
        //wait(0.2);
        //uint8_t ak_wai2 = AK8963_whoami();
        wait(0.2);
        uint8_t ak_wai1 = mpu9250.readByteAK(WHO_AM_I_AK8963);//*/AK8963_whoami();
                
        //mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
        
        /*mpu9250.initMPU9250(); 
        uprintf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
        */
        mpu9250.initAK8963(magCalibration);
        
        /*uprintf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
        uprintf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
        uprintf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));*/
        /*if(Mscale == 0) uprintf("Magnetometer resolution = 14  bits\n\r");
        if(Mscale == 1) uprintf("Magnetometer resolution = 16  bits\n\r");
        if(Mmode == 2) uprintf("Magnetometer ODR = 8 Hz\n\r");
        if(Mmode == 6) uprintf("Magnetometer ODR = 100 Hz\n\r");*/
        wait(1);
    }
    else
    {
        uprintf("Could not connect to MPU9250: \n\r");
        uprintf("%#x \n",  whoami);    
        while(1);
    }
  
    mpu9250.getAres(); // Get accelerometer sensitivity
    mpu9250.getGres(); // Get gyro sensitivity
    mpu9250.getMres(); // Get magnetometer sensitivity
    /*uprintf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
    uprintf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
    uprintf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);*/
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

    while(1) 
    {
        // If intPin goes high, all data registers have new data
        if(mpu9250.readByte(INT_STATUS) & 0x01) 
        {  // On interrupt, check if data ready interrupt
          
            mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values   
            //uprintf("acc = (%d, %d, %d)", accelCount[0], accelCount[1], accelCount[2]);
            // Now we'll calculate the accleration value into actual g's
            ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            ay = (float)accelCount[1]*aRes - accelBias[1];   
            az = (float)accelCount[2]*aRes - accelBias[2];  
           
            mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
            //uprintf("gyr = (%d, %d, %d)", gyroCount[0], gyroCount[1], gyroCount[2]);
            // Calculate the gyro value into actual degrees per second
            gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
            gy = (float)gyroCount[1]*gRes - gyroBias[1];  
            gz = (float)gyroCount[2]*gRes - gyroBias[2];   
          
            mpu9250.readMagData(magCount);  // Read the x/y/z adc values   
            //uprintf("mag = (%d, %d, %d)", magCount[0], magCount[1], magCount[2]);
            // Calculate the magnetometer values in milliGauss
            // Include factory calibration per data sheet and user environmental corrections
            mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
            my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
            mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
        }
   
        uint32_t Now = HAL_GetTick();
        deltat = (float)((Now - lastUpdate)/1000.0f) ; // set integration time by time elapsed since last filter update
        lastUpdate = Now;
        
        sum += deltat;
        sumCount++;
    
        mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

        // Serial print and/or display at 0.5 s rate independent of data rates
        delt_t = HAL_GetTick() - count;
        if (delt_t > 500) 
        { // update LCD once per half-second independent of read rate
            //uprintf("acc = (%f, %f, %f)g", ax, ay, az);
            /*uprintf("ax = %f", 1000*ax); 
            uprintf(" ay = %f", 1000*ay); 
            uprintf(" az = %f  mg\n\r", 1000*az); */

            //uprintf("gyr = (%f, %f, %f)d/s\n", gx, gy, gz);
            /*uprintf("gx = %f", gx); 
            uprintf(" gy = %f", gy); 
            uprintf(" gz = %f  deg/s\n\r", gz); */
            
            //uprintf("mag = (%f, %f, %f)mG\n", mx, my, mz);
            /*uprintf("gx = %f", mx); 
            uprintf(" gy = %f", my); 
            uprintf(" gz = %f  mG\n\r", mz); */
            
            tempCount = mpu9250.readTempData();  // Read the adc values
            temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
            //uprintf(" temperature = %f  C\n\r", temperature); 
            
            uprintf("+q(%f,%f,%f,%f)\n", q[0], q[1], q[2], q[3]);
            /*uprintf("q0 = %f\n\r", q[0]);
            uprintf("q1 = %f\n\r", q[1]);
            uprintf("q2 = %f\n\r", q[2]);
            uprintf("q3 = %f\n\r", q[3]);      */
    
            // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
            // In this coordinate system, the positive z-axis is down toward Earth. 
            // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
            // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
            // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
            // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
            // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
            // applied in the correct order which for this configuration is yaw, pitch, and then roll.
            // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
            /*yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
            pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
            roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            pitch *= 180.0f / PI;
            yaw   *= 180.0f / PI; 
            yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
            roll  *= 180.0f / PI;*/

            //uprintf("Yaw, Pitch, Roll: %.1f %.1f %.1f\n\r", yaw, pitch, roll);
            //uprintf("average rate = %f\n\r", (float) sumCount/sum);
            
            //myled= !myled;
            count = HAL_GetTick(); 

            /*if(count > 1<<21) 
            {
                t.start(); // start the timer over again if ~30 minutes has passed
                count = 0;
                deltat= 0;
                lastUpdate = t.read_us();
            }*/
            sum = 0;
            sumCount = 0; 
        }
    }
    
    
    /*while (1)
    {
      //int ctr = readByte(FIFO_COUNTL) | (readByte(FIFO_COUNTH) << 8);
      float ax = 0;
      for(int x = 0; x < 256; x++)
      {
        for(int y = 0; y < 256; y++);
        int16_t acc[3];
        readAccelData(acc);
        ax += acc[0];
        
      }
      //ax /= 256;
      char b[16];
      int l = sprintf(b, "x = %f\n", ax);
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *) b, strlen(b));
      
      //readFIFO();
    }*/
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  GPIO_InitTypeDef GPIO_InitStruct;
  
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  mpu_spi_deselect();
}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 I2S3_SCK_Pin PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|I2S3_SCK_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
extern "C" void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
