/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define TELEMETRY
/*Number of pole-pairs*/
//#define PID  //define PID if integrator is used
#define P 7
#define kMax 720



#define ADDRESS  0xD0
#define ADDRESS2 0xD2
#define G        9.81f

#define TS       0.001

#define P1       40.3
#define I1       200.0
#define D1       3.8
#define SAT1     1400

#define P2       65.6154
#define I2       1.7477
#define D2       1.0642
#define SAT2     2100
#define N2       10.0
#define TI2      (P2 / I2)
#define TD2      (D2 / P2)
#define TDV2     (TD2 / N2)

#define P3       33.3
#define I3       1.1
#define D3       0.05
#define SAT3     1700
#define N3       10.0
#define TI3      (P3 / I3)
#define TD3      (D3 / P3)
#define TDV3     (TD3 / N3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double qsine[] = {0,20,39,59,78,98,118,137,157,176,196,216,235,255,274,294,313,332,352,371,
		391,410,429,448,468,487,506,525,544,563,582,601,620,639,658,676,695,714,732,751,769,788,
		806,824,842,861,879,897,915,933,950,968,986,1003,1021,1038,1056,1073,1090,1107,1124,1141,
		1158,1175,1192,1208,1225,1241,1258,1274,1290,1306,1322,1338,1353,1369,1385,1400,1415,1431,
		1446,1461,1475,1490,1505,1519,1534,1548,1562,1576,1590,1604,1618,1631,1645,1658,1671,1684,
		1697,1710,1723,1735,1748,1760,1772,1784,1796,1808,1819,1831,1842,1853,1865,1875,1886,1897,
		1907,1918,1928,1938,1948,1957,1967,1976,1986,1995,2004,2013,2021,2030,2038,2047,2055,2062,
		2070,2078,2085,2093,2100,2107,2113,2120,2126,2133,2139,2145,2151,2156,2162,2167,2172,2177,
		2182,2187,2191,2196,2200,2204,2208,2211,2215,2218,2221,2224,2227,2230,2232,2235,2237,2239,
		2240,2242,2244,2245,2246,2247,2248,2248,2249,2249,2249,2249,2249,2248,2248,2247,2246,2245,
		2244,2242,2240,2239,2237,2235,2232,2230,2227,2224,2221,2218,2215,2211,2208,2204,2200,2196,
		2191,2187,2182,2177,2172,2167,2162,2156,2151,2145,2139,2133,2126,2120,2113,2107,2100,2093,
		2085,2078,2070,2062,2055,2047,2038,2030,2021,2013,2004,1995,1986,1976,1967,1957,1948,1938,
		1928,1918,1907,1897,1886,1875,1865,1853,1842,1831,1819,1808,1796,1784,1772,1760,1748,1735,
		1723,1710,1697,1684,1671,1658,1645,1631,1618,1604,1590,1576,1562,1548,1534,1519,1505,1490,
		1475,1461,1446,1431,1415,1400,1385,1369,1353,1338,1322,1306,1290,1274,1258,1241,1225,1208,
		1192,1175,1158,1141,1124,1107,1090,1073,1056,1038,1021,1003,986,968,950,933,915,897,879,861,
		842,824,806,788,769,751,732,714,695,676,658,639,620,601,582,563,544,525,506,487,468,448,429,
		410,391,371,352,332,313,294,274,255,235,216,196,176,157,137,118,98,78,59,39,20,0,-20,-39,-59,
		-78,-98,-118,-137,-157,-176,-196,-216,-235,-255,-274,-294,-313,-332,-352,-371,-391,-410,-429,
		-448,-468,-487,-506,-525,-544,-563,-582,-601,-620,-639,-658,-676,-695,-714,-732,-751,-769,-788,
		-806,-824,-842,-861,-879,-897,-915,-933,-950,-968,-986,-1003,-1021,-1038,-1056,-1073,-1090,-1107,
		-1124,-1141,-1158,-1175,-1192,-1208,-1225,-1241,-1258,-1274,-1290,-1306,-1322,-1338,-1353,-1369,
		-1385,-1400,-1415,-1431,-1446,-1461,-1475,-1490,-1505,-1519,-1534,-1548,-1562,-1576,-1590,-1604,
		-1618,-1631,-1645,-1658,-1671,-1684,-1697,-1710,-1723,-1735,-1748,-1760,-1772,-1784,-1796,-1808,
		-1819,-1831,-1842,-1853,-1865,-1875,-1886,-1897,-1907,-1918,-1928,-1938,-1948,-1957,-1967,-1976,
		-1986,-1995,-2004,-2013,-2021,-2030,-2038,-2047,-2055,-2062,-2070,-2078,-2085,-2093,-2100,-2107,
		-2113,-2120,-2126,-2133,-2139,-2145,-2151,-2156,-2162,-2167,-2172,-2177,-2182,-2187,-2191,-2196,
		-2200,-2204,-2208,-2211,-2215,-2218,-2221,-2224,-2227,-2230,-2232,-2235,-2237,-2239,-2240,-2242,
		-2244,-2245,-2246,-2247,-2248,-2248,-2249,-2249,-2249,-2249,-2249,-2248,-2248,-2247,-2246,-2245,
		-2244,-2242,-2240,-2239,-2237,-2235,-2232,-2230,-2227,-2224,-2221,-2218,-2215,-2211,-2208,-2204,
		-2200,-2196,-2191,-2187,-2182,-2177,-2172,-2167,-2162,-2156,-2151,-2145,-2139,-2133,-2126,-2120,
		-2113,-2107,-2100,-2093,-2085,-2078,-2070,-2062,-2055,-2047,-2038,-2030,-2021,-2013,-2004,-1995,
		-1986,-1976,-1967,-1957,-1948,-1938,-1928,-1918,-1907,-1897,-1886,-1875,-1865,-1853,-1842,-1831,
		-1819,-1808,-1796,-1784,-1772,-1760,-1748,-1735,-1723,-1710,-1697,-1684,-1671,-1658,-1645,-1631,
		-1618,-1604,-1590,-1576,-1562,-1548,-1534,-1519,-1505,-1490,-1475,-1461,-1446,-1431,-1415,-1400,
		-1385,-1369,-1353,-1338,-1322,-1306,-1290,-1274,-1258,-1241,-1225,-1208,-1192,-1175,-1158,-1141,
		-1125,-1107,-1090,-1073,-1056,-1038,-1021,-1003,-986,-968,-950,-933,-915,-897,-879,-861,-842,-824,
		-806,-788,-769,-751,-732,-714,-695,-676,-658,-639,-620,-601,-582,-563,-544,-525,-506,-487,-468,-448,
		-429,-410,-391,-371,-352,-332,-313,-294,-274,-255,-235,-216,-196,-176,-157,-137,-118,-98,-78,-59,-39,-20};


double sinVal(double a)
{
	double an = fmod(-1 * a, 2 * M_PI) * 360 / M_PI;
	double ap = fmod(a, 2 * M_PI) * 360 / M_PI;
	double s1 = qsine[(uint32_t)(an)];
	double s2 = qsine[(uint32_t)(ap)];
	return ((a < 0 ? -1 *  s1 : s2) / 2249) * 2249/12;//1499 / 8;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  uint8_t check = 0u;
  uint8_t data=0;
  uint8_t buf[20];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  /*-------------Initialize camera gyro---------------*/
  HAL_Delay(100);
  if(HAL_OK == HAL_I2C_IsDeviceReady(&hi2c2,ADDRESS,1 ,100) )
  {
	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  }
  HAL_I2C_Mem_Read(&hi2c2, ADDRESS,0x75,1, &check, 1, 1000);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  HAL_Delay(200);
  if(0x68 == check)
  {
	  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  }
  /*Reset sensor*/
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c2, ADDRESS, 0x6B, 1,&data, 1, 1000);
  /*Wake up sensor*/
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c2, ADDRESS, 0x00, 1,&data, 1, 1000);
  /*Set DATA RATE of 1KHz by writing SMPLRT_DIV register*/
  data = 0x07;
  HAL_I2C_Mem_Write(&hi2c2, ADDRESS, 0x19, 1, &data, 1, 1000);

  /*Configure gyro (+-250 degrees/s) and accelerometer*/
  data = 0x10;
  HAL_I2C_Mem_Write(&hi2c2, ADDRESS, 0x1B, 1, &data, 1, 1000);
  data = 0x08;
  HAL_I2C_Mem_Write(&hi2c2, ADDRESS, 0x1C, 1, &data, 1, 1000);
 /*-----------------------------------------------------------------*/

  /*-------------Initialize mount gyro---------------*/
  HAL_Delay(100);
  if(HAL_OK == HAL_I2C_IsDeviceReady(&hi2c1,ADDRESS2,1 ,100) )
  {
	  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  }

  HAL_I2C_Mem_Read(&hi2c1, ADDRESS2,0x75,1, &check, 1, 1000);
  HAL_Delay(200);
  //HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  HAL_Delay(200);
  if(0x68 == check)
  {
	 // HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
  }
  /*Reset sensor*/
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS2, 0x6B, 1,&data, 1, 1000);
  /*Wake up sensor*/
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS2, 0x00, 1,&data, 1, 1000);
  /*Set DATA RATE of 1KHz by writing SMPLRT_DIV register*/
  data = 0x07;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS2, 0x19, 1, &data, 1, 1000);

  /*Configure gyro (+-250 degrees/s) and accelerometer*/
  data = 0x10;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS2, 0x1B, 1, &data, 1, 1000);
  data = 0x08;
  HAL_I2C_Mem_Write(&hi2c1, ADDRESS2, 0x1C, 1, &data, 1, 1000);
 /*-----------------------------------------------------------------*/

  /*-----------------Initialize motors-------------------------*/
  /*Start YAW*/
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /*Start YAW*/
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /*Start PITCH*/
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);



  /*Pull YAW rotor to  a init position*/
  TIM4->CCR3 = 1450;
  TIM2->CCR2 = 0u;
  TIM4->CCR4 = 0u;

  /*Pull ROLL rotor to  a init position*/
  TIM2->CCR3 = 1450;
  TIM2->CCR4 = 0u;
  TIM3->CCR1 = 0u;

  /*Pull PITCH rotor to  a init position*/
  TIM3->CCR2 = 1450;
  TIM3->CCR3 = 0u;
  TIM3->CCR4 = 0u;
  /*------------------------------------------------------------*/

  /*---------------Initialize variables---------------*/
  double dt = 1;
  double a, b, c;
  double fia, fib, fic;
  uint8_t values[6];
  int16_t Accel_X_RAW = 0;
  int16_t Accel_Y_RAW = 0;
  int16_t Accel_Z_RAW = 0;
  int16_t Gyro_X_RAW = 0;
  int16_t Gyro_Y_RAW = 0;
  int16_t Gyro_Z_RAW = 0;

  double Ax = 0.0f;
  double Ay = 0.0f;
  double ax = 0.0f;
  double ay = 0.0f;
  double az = 0.0f;

  double Gx = 0.0f;
  double Gy = 0.0f;
  double Gz = 0.0f;
  double gx = 0.0f;
  double gy = 0.0f;
  double gz = 0.0f;
  double fid = 0.0;
  double thetad = 0.0;
  double pszid = 0.0;
  double fi = 0.0;
  double theta = 0.0;
  double pszi = 0.0;

  double fiz = 0.0;

  double figx = 0.0;
  double figy = 0.0;

  double fi0, theta0, pszi0;

  /*----------PID PITCH vars---------*/
  double ik2, ik12, ek2, ek12, dk2, dk12, uk2;
  ik2 = ik12 = ek2 = ek12 = dk2 = dk12 = uk2 = 0.0;
  uint8_t sat2 = 0u;
  /*--------------*/

  /*----------PID ROLL vars---------*/
  double ik3, ik13, ek3, ek13, dk3, dk13, uk3;
  ik3 = ik13 = ek3 = ek13 = dk3 = dk13 = uk3 = 0.0;
  uint8_t sat3 = 0u;
  /*--------------*/

  /*------------Gyro mount vars--------------*/
  uint8_t values2[6];
  int16_t Accel_X_RAW2, Accel_Y_RAW2, Accel_Z_RAW2;
  int16_t Gyro_X_RAW2, Gyro_Y_RAW2, Gyro_Z_RAW2;
  double Ax2, Ay2, ax2, ay2, az2;
  double Gx2, Gy2, Gz2, gx2, gy2, gz2;
  double fid2, thetad2, pszid2, fi2, theta2, pszi2;
  double figx2, figy2;
  double fi20, theta20, pszi20;

  double firef0, thetaref0, psziref0;

  memset(buf, '\0',20);
  sprintf (buf, "X");
  data = '\n';
  uint8_t kezdo = 'X';

  double firef, thetaref, psziref;
  firef = psziref = thetaref = 0;
  uint8_t init = 1;
  double efi, etheta, epszi;
  efi = etheta = epszi = 0;
  double m1;

  uint16_t init2 = 200u;
  uint64_t iter = 0u;

  //motor angles
  double fim, thetam, pszim;

  HAL_Delay(2000);

  /*Registers for measuring time*/
  DWT->CTRL |= 1 ; // enable the counter

  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  while (1)
  {
	  /*Reading accelerometer registers*/
	  HAL_I2C_Mem_Read (&hi2c2,ADDRESS, 0x3B, 1, values, 6, 1000);
	  Accel_X_RAW = (int16_t)(values[0] << 8 | values [1]);
	  Accel_Y_RAW = (int16_t)(values[2] << 8 | values [3]);
	  Accel_Z_RAW = (int16_t)(values[4] << 8 | values [5]);

	  	  	  /*Read gyro registers*/
	  HAL_I2C_Mem_Read (&hi2c2, ADDRESS, 0x43, 1, values, 6, 1000);
	  Gyro_X_RAW = (int16_t)(values[0] << 8 | values [1]);
	  Gyro_Y_RAW = (int16_t)(values[2] << 8 | values [3]);
	  Gyro_Z_RAW = (int16_t)(values[4] << 8 | values [5]);

	  /*Reading mount accelerometer registers*/
	  HAL_I2C_Mem_Read (&hi2c1,ADDRESS2, 0x3B, 1, values2, 6, 1000);
	  Accel_X_RAW2 = (int16_t)(values2[0] << 8 | values2 [1]);
	  Accel_Y_RAW2 = (int16_t)(values2[2] << 8 | values2 [3]);
	  Accel_Z_RAW2 = (int16_t)(values2[4] << 8 | values2 [5]);

	  	  	  /*Read mount gyro registers*/
	  HAL_I2C_Mem_Read (&hi2c1, ADDRESS2, 0x43, 1, values2, 6, 1000);
	  Gyro_X_RAW2 = (int16_t)(values2[0] << 8 | values2 [1]);
	  Gyro_Y_RAW2 = (int16_t)(values2[2] << 8 | values2 [3]);
	  Gyro_Z_RAW2 = (int16_t)(values2[4] << 8 | values2 [5]);


	  /*Calculate elapsed time*/
	  dt = (((double)DWT->CYCCNT) / HAL_RCC_GetHCLKFreq());
	  DWT->CYCCNT = 0;
	  	  	  /*Get the acceleration values*/
	  ax = G*Accel_X_RAW / 8192.0;
	  ay = -1 * (G*Accel_Y_RAW / 8192.0);
	  az = -G*Accel_Z_RAW / 8192.0;
	  	  	  /*Get angular velocity values*/
	  gx = Gyro_X_RAW / 32.8;
	  gy = -1 * (Gyro_Y_RAW / 32.8);
	  gz = -Gyro_Z_RAW / 32.8;
	  	  	  /*Normalize values*/
	  ax = ax / G;
	  ay = ay / G;
	  az = az / G;

	  Ax = (atan(ay / az)-0.01);
	  Ay = (atan(-ax / (sqrt(pow(ay,2) + pow(az,2)))) - 0.027576);
	  Gx = (gx - 0.50491+0.06) * M_PI / 180.0;
	  Gy = (gy + 1.5-0.025) * M_PI / 180.0;
	  Gz = (gz + 0.024518) * M_PI / 180.0;

	  	  /*Get the mount acceleration values*/
      ay2 = -G*Accel_X_RAW2 / 8192.0;
      ax2 = 1 * (G*Accel_Y_RAW2 / 8192.0);
      az2 = G*Accel_Z_RAW2 / 8192.0;
	  	  /*Get mount angular velocity values*/
      gy2 = -Gyro_X_RAW2 / 32.8;
      gx2 = (Gyro_Y_RAW2 / 32.8);
      gz2 = Gyro_Z_RAW2 / 32.8;
	  	  /*Normalize values*/
      ax2 = ax2 / G;
      ay2 = ay2 / G;
      az2 = az2 / G;

      Ax2 = (atan(ay2 / az2)-0.01);
      Ay2 = (atan(-ax2 / (sqrt(pow(ay2,2) + pow(az2,2)))) - 0.027576);
      Gx2 = (gx2 - 1.1649) * M_PI / 180.0;
      Gy2 = (gy2 - 5.47) * M_PI / 180.0;
      Gz2 = (gz2 - 1.975482) * M_PI / 180.0;

	  if(1u == init)
	  {
		  init = 0;

		  figx = Ax;
		  figy = Ay;

		  fi = Ax;
		  theta = Ay;
		  fiz = 0;

		  dt = 0;

		  fi0 = Ax;
		  theta0 = Ay;
		  pszi0 = 0;

		  firef = fi0;
		  thetaref = theta0;
		  psziref = 0;

		  /*mount IMU*/
		  figx2 = Ax2;
		  figy2 = Ay2;

		  fi2 = Ax2;
		  theta2 = Ay2;
		  pszi2 = 0;

		  fi20 = Ax2;
		  theta20 = Ay2;
		  pszi20 = 0;
	  }
	  if(init2 != 0u)
	  {
		  firef -= fi0 / 200.0;
		  thetaref -= theta0 / 200.0;
		  init2--;
	  }
	  else
	  {
		  firef = 0;
		  thetaref = 0;
	  }

	  fid = Gx + Gy * sin(fi) * tan(theta) + Gz * cos(fi) * tan(theta);
	  thetad = Gy * cos(fi) - Gz * sin(fi);
	  pszid = (Gy * sin(fi) / cos(theta)) + Gz * cos(fi) / cos(theta);

	  figx = figx + fid * dt + dt * 0.16 * M_PI / 180;
	  figy = figy + thetad * dt - dt * 0.03 * M_PI / 180;
	  fiz = fiz + pszid * dt;
	  /*Calculate angle values*/
	  /*Pitch*/
	  fi = 0.94 * figx + 0.06 * Ax;
	  /*Roll*/
	  theta = 0.94 * figy + 0.06 * Ay;
	  /*Yaw*/
	  pszi = fiz;

	  /*mount IMU*/
	  fid2 = Gx2 + Gy2 * sin(fi2) * tan(theta2) + Gz2 * cos(fi2) * tan(theta2);
	  thetad2 = Gy2 * cos(fi2) - Gz2 * sin(fi2);
	  pszid2 = (Gy2 * sin(fi2) / cos(theta2)) + Gz2 * cos(fi2) / cos(theta2);

	  figx2 = figx2 + fid2 * dt + dt * 0.07 * M_PI / 180;
	  figy2 = figy2 + thetad2 * dt + dt * 0.03 * M_PI / 180;
	  /*Calculate angle values*/
	  /*Pitch*/
	  fi2 = 0.94 * figx2 + 0.06 * Ax2;
	  /*Roll*/
	  theta2 = 0.94 * figy2 + 0.06 * Ay2;
	  /*Yaw*/
	  pszi2 += pszid2 * dt;

	  /*Motor mechanical angles with respect to phase a*/
	  fim = fi - fi0 - fi2 + fi20;
	  thetam = theta - theta0 - theta2 + theta20;
	  pszim = pszi - pszi2;


	  /*Pull YAW rotor to  a init position*/
	  fia = pszim * P;
	  fib = pszim * P - (2 * M_PI / 3);
	  fic = pszim * P - (4 * M_PI / 3);

	  epszi = psziref - pszi;
	  a = P1 * epszi * (-1 * sinVal(fia) / 1.0);
	  b = P1 * epszi * (-1 * sinVal(fib) / 1.0);
	  c = P1 * epszi * (-1 * sinVal(fic) / 1.0);
	  m1 = fmax(fabs(a), fmax(fabs(b),fabs(c)));
	  if(m1 > SAT1)
	  {
		  m1 = SAT1 / m1;
		  a = m1 * a;
		  b = m1 * b;
		  c = m1 * c;
	  }
	 // TIM4->CCR3 = a > 0 ? (uint32_t)a : 0;
	  //TIM2->CCR2 = b > 0 ? (uint32_t)b : 0;
	 // TIM4->CCR4 = c > 0 ? (uint32_t)c : 0;
	  /*-----------------------------------*/

	  /*-----------PITCH-----------*/
	  fia = thetam * P;
	  fib = thetam * P - (2 * M_PI / 3);
	  fic = thetam * P - (4 * M_PI / 3);

	  etheta = thetaref - theta;
	  ek2 = etheta;
	  dk2 = (1 / (TS + TDV2)) * (TD2 * ek2 - TD2 * ek12 + TDV2 * dk12);
	  dk12 = dk2;
	  if(!sat2)
	  {
		  ik2 = (TS / TI2) * ek2 + ik12;
		  ik12 = ik2;
	  }
#ifdef PID
	  uk2 = ek2 * P2 + P2 * dk2 + P2 * ik2 ;

#else
	  uk2 = ek2 * P2 + P2 * dk2;
#endif
	  ek12 = ek2;

	  a = uk2 * (-1 * sinVal(fia) / 1.0);
	  b = uk2 * (-1 * sinVal(fib) / 1.0);
	  c = uk2 * (-1 * sinVal(fic) / 1.0);
	  m1 = fmax(fabs(a), fmax(fabs(b),fabs(c)));
	  if(m1 > SAT2)
	  {
		  m1 = SAT2 / m1;
		  a = m1 * a;
		  b = m1 * b;
		  c = m1 * c;
		  sat2 = 1u;
	  }
	  else
	  {
		  sat2 = 0u;
	  }

	  TIM2->CCR3 = a > 0 ? (uint32_t)a : 0;
	  TIM2->CCR4 = b > 0 ? (uint32_t)b : 0;
	  TIM3->CCR1 = c > 0 ? (uint32_t)c : 0;
	  /*-------------------------------------*/


	  /*----------ROLL-------*/
	  fia = fim * P;
	  fib = fim * P - (2 * M_PI / 3);
	  fic = fim * P - (4 * M_PI / 3);

	  efi = firef - fi;

	  ek3 = efi;
	  dk3 = (1 / (TS + TDV3)) * (TD3 * ek3 - TD3 * ek13 + TDV3 * dk13);
	  dk13 = dk3;
	  if(!sat3)
	  {
		  ik3 = (TS / TI3) * ek3 + ik13;
		  ik13 = ik3;
	  }
#ifdef PID
	  uk3 = ek3 * P3 + P3 * dk3 + P3 * ik3 ;
#else
	  uk3 = ek3 * P3 + P3 * dk3;
#endif
	  ek13 = ek3;

	  a = uk3 * (-1 * sinVal(fia) / 1.0);
	  b = uk3 * (-1 * sinVal(fib) / 1.0);
	  c = uk3 * (-1 * sinVal(fic) / 1.0);
	  m1 = fmax(fabs(a), fmax(fabs(b),fabs(c)));
	  if(m1 > SAT3)
	  {
		  m1 = SAT3 / m1;
		  a = m1 * a;
		  b = m1 * b;
		  c = m1 * c;
		  sat3 = 1u;
	  }
	  else{
		  sat3 = 0u;
	  }
	  TIM3->CCR2 = a > 0 ? (uint32_t)a : 0;
	  TIM3->CCR3 = b > 0 ? (uint32_t)b : 0;
	  TIM3->CCR4 = c > 0 ? (uint32_t)c : 0;
	  /*---------------------*/



#ifdef TELEMETRY
	  if(iter++ % 6 == 0)
	  {


		  CDC_Transmit_FS(&kezdo, sizeof(kezdo));
		  //Convert double to an array of uint8 and then send

		  HAL_Delay(2);

		  CDC_Transmit_FS(((uint8_t*)(&dt)), sizeof(double));
		  HAL_Delay(2);
		  CDC_Transmit_FS(((uint8_t*)(&theta2)), sizeof(double));
		  HAL_Delay(2);
		  CDC_Transmit_FS(((uint8_t*)(&fi)), sizeof(double));
		  HAL_Delay(2);
		  CDC_Transmit_FS(((uint8_t*)(&theta)), sizeof(double));
		  HAL_Delay(2);
		  CDC_Transmit_FS(&data, sizeof(data));
	  }
#endif
  }

  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2249;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
