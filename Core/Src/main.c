/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dshot.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//Partie moteurs (D-Shot)
#define CANONS_SPEED_1 100
#define CANONS_SPEED_2 300
#define CANONS_SPEED_3 400
#define TURBINE_SPEED_1 100
#define TURBINE_SPEED_2 300
#define TURBINE_SPEED_3 400
#define COMPRESSOR_SPEED 1200
//effet : change le bitrate du protocole D-Shot (jamais testé autre que 150)
//autres valeurs existantes : 300, 600, 1200
#define DSHOT_SPEED DSHOT150

//Partie Pression
//size of pressure values history buffer : effet : change proportionnellement la rapidité de réponse du moteur du compresseur
#define PRESSURES_SIZE 10
// effet : change la pression de Commande initiale (avant 1ère commande compresseur)
uint8_t Press_order = 0; //en 10e de bar

//Partie electrovannes
#define PULSE_TIME 150 //valeur du pulse des EV (ms)
uint8_t last_EV_command = 0;

//Partie Capteur de Temperature
#define COMPR_CRIT_TEMP 60 //à modifier en fonction du système (position de la sonde PT100)

//Partie timer global pour tempo AU => purge
int BAU_tick;
uint8_t BAU_tick_enable = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;
DMA_HandleTypeDef hdma_tim3_ch4_up;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Partie D-Shot
uint16_t my_motor_value[5] = {0, 0, 0, 0, 0};


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_StatusTypeDef res1;
	HAL_StatusTypeDef res2;

	//Partie commandes et lecture des capteurs
	uint8_t command_buffer = 0; // Signal commande Rpi -> Nucléo, pas de log pour l'instant donc taille de 1
	uint8_t return_buffer[2]; //infos retournée Nucléo -> Rpi
	uint8_t I2C_buf[4]; //Infos I2C : P_sensor (4 bytes) ou T_sensor_Comp (?? bytes)
	// uint8_t p_buf[4]; //Pression P_sensor (2 bytes) et température P_sensor (2 bytes)
	//garder les logs des commandes

	//partie Capteur de Pression
	//modèle : 2513130810401
	float pressure_val;
	//SENP:  Pressure sensor sensitivity : 4.196 ×10-2
	float SENP = 0.04196;
	//PMIN: Min Pressure output : -100 kPa
	int Pmin = -100;
	//OUTP_MIN: digital output at Pmin = 3277
	float OUT_Pmin = 3277;
	//P15bit = PH & PL
	float P15bit;
	//tableaux des 100 dernières valeurs de pression, toutes les valeurs initialisées à 0
	uint8_t pressures[PRESSURES_SIZE] = {0};
	//valeur pour remplir le tableau des valeurs de pressions avant de calculer la moyennes des pressions
	uint32_t pressures_mean = 0;

	//Partie capteur de température du compresseur
	uint16_t raw;
	float compr_temp;

	// explication de la conversion des valeurs de l'ADC en température
	// dans le fichier "Equation_Sonde_PT100, il y a 2 courbes, ces coeffs sont respectivement les pentes et les ordonnées à l'origine
	// raw = valeur numérique renvoyée par l'ADC
	// méthode de calibration sonde PT100 : mesure resistance et raw à 22°   &   mesure resistance et raw à 100° (pistolet à air chaud devant la PT100)
	float raw_to_res_mult = 0.0393;
	float raw_to_res_offset = -2.47;
	float res_to_temp_mult = -0.74;
	float res_to_temp_offset = 104;

	//Partie Arrêt d'urgence
	uint8_t AU_Current_Status = GPIO_PIN_RESET;
	uint8_t AU_Old_Status = GPIO_PIN_RESET;


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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_CAN_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	dshot_init(DSHOT_SPEED);
	//Initialization Des ESC des moteurs ET TOUT LE RESTE
	my_motor_value[0] = 0;
	my_motor_value[1] = 0;
	my_motor_value[2] = 0;
	my_motor_value[3] = 0;
	my_motor_value[4] = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
	//start to count (for tim6 interruption)
	HAL_TIM_Base_Start_IT(&htim6);
	//a peu près temps minimal de delay pour laisser le temps aux moteurs de s'initialiser
	HAL_Delay(2600);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//récupération du mot de commande

		//fréquence recommandée de boucle totale : 10Hz
		AU_Current_Status = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1);

		//Reset de tout lorsque l'AU repasse à l'état haut (non coupé) alors qu'il était à l'état bas (coupé) juste avant
		if ((AU_Current_Status == GPIO_PIN_RESET) && AU_Old_Status == GPIO_PIN_SET){
			// arrêt moteurs (compr, canons, turbine)
			my_motor_value[0] = 0;
			my_motor_value[1] = 0;
			my_motor_value[2] = 0;
			my_motor_value[3] = 0;
			my_motor_value[4] = 0;
			HAL_Delay(2600);
			// arrêt EV 1, 2, 3 et Purge
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
			// arrêt LCD et LED Enable
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			// reset la régulation de pression
			Press_order = 0;
			//reset le timer de purge
			BAU_tick = 0;
			BAU_tick_enable = 0;
		}

		//Si l'AU passe à l'état bas (coupé) alors qu'il était à l'état haut (non coupé), reset tout et lancer un timer de 2 minutes, au bout duquel on purge !
		if (AU_Current_Status == GPIO_PIN_SET && AU_Old_Status == GPIO_PIN_RESET){
			// arrêt moteurs (compr, canons, turbine)
			my_motor_value[0] = 0;
			my_motor_value[1] = 0;
			my_motor_value[2] = 0;
			my_motor_value[3] = 0;
			my_motor_value[4] = 0;
			// arrêt EV 1, 2, 3 et Purge
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
			// arrêt LCD et LED Enable
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
			// reset la régulation de pression
			Press_order = 0;
			// tempo de 2 min, puis purge
			BAU_tick_enable = 1;
		}
		//changer les valeurs des AU status
		AU_Old_Status = AU_Current_Status;

		//Réception du mot de 8 Bit de Goldo (par UART à priori)
		res1 = HAL_UART_Receive(&huart2, &command_buffer, 1, 100);
		if(res1 == HAL_OK){
			//Notifier à la Rpi la bonne réception du mot de commande
			return_buffer[0]|= 1;
			//Retourner les infos à la Rpi !
			HAL_UART_Transmit(&huart2, return_buffer, 1, 100);
		}



		//Lecture de la pression en I2C et activation OU NON du compresseur en fonction
		res2 = HAL_I2C_Master_Receive(&hi2c1, 0xf1, I2C_buf, 4, 200);
		if (res2 == HAL_OK){
			//Régulation Pression
			//Lecture de la pression (ET de la température un jour ?) => déterminer un Offset pour tout le match sur les vals de pression ? (voire toutes les 10 secondes)
			//sprintf((char*)p_buf, "I2C : %d %d %d %d\r\n", (int)I2C_buf[0], (int)I2C_buf[1], (int)I2C_buf[2], (int)I2C_buf[3]);
			//HAL_UART_Receive(&huart2, p_buf, 4, 100);

			//Calcul de la pression en 10èmes de bar RELATIFS
			//voir infos_pressure_sensor plus haut pour infos sur variables
			P15bit = (int)((I2C_buf[0] << 8)|I2C_buf[1]);
			//formule d'après la datasheet du capteur : pressure_val = [(P15bit - OUTP_MIN)*SENP)] + PMIN;
			//Conversion en 10èmes de bar à la fin
			pressure_val = P15bit - OUT_Pmin; //2383
			pressure_val = pressure_val*SENP; //99.99068
			pressure_val = pressure_val + Pmin; // -0,00932
			pressure_val = pressure_val * 0.1; // -0,000932

			//calcul de la moyenne de toutes les pressions précédentes
			//tant que le tableau n'est pas plein, on remplit
			//if (pressures[PRESSURES_SIZE-1] == 0){
			//	pressures[pressures_full_counter] = (int)pressure_val;
			//}
			//dès qu'il est plein, on calcule la moyenne du tableau renouvelé
			//on décale tout le tableau vers la droite, en faisant donc disparaître la valeur la plus ancienne

			for (int i = PRESSURES_SIZE-1; i > 0; i--){
				pressures[i] = pressures[i-1];
			}
			//On ajoute la nouvelle valeure
			pressures[0] = (int)pressure_val;

			//puis on fait la moyenne des pressions
			for (int i = 0; i < PRESSURES_SIZE; i++){
				pressures_mean+=pressures[i];
			}
			pressures_mean/=PRESSURES_SIZE;

			if (pressures_mean > (Press_order)){ // arrêter compresseur si dépassement de la pression de consigne
				my_motor_value[4] = 0;
			}
			else if (pressures_mean < (Press_order - 2)) { // démarrage compresseur avec hysteresis de 0.4 bar
				if(Press_order <= 0){}
				else {
					my_motor_value[4] = COMPRESSOR_SPEED;
				}
			}
			else {}

			//on remet à 0 la moyenne des pressions
			pressures_mean = 0;


			//retourner à la Rpi la pression courante du réservoir en dixième de bar [0ZZZZZZZ], MSB utilisé pour erreur de lecture en I2C de la pression
			//return_buffer[0] = (uint8_t)pressure_val;
		}
		//retourner [10000000] si on arrive pas à communiquer avec le capteur de Pression
		/*
		else {
			return_buffer[0]|=128;
		}
		*/

		//Lecture Température du compresseur réservoir et activation ou désactivation du compresseur en fonction SSI il est pas déjà désactivé
		//#####A IMPLEMENTER : Lecture de la température en analogique sonde PT100#####
		//start an ADC conversion
		HAL_ADC_Start(&hadc1);
		//processor waits for an ADC conversion to complete
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		//get raw value from the ADC channel register
		raw = HAL_ADC_GetValue(&hadc1);

		//get temperature value from raw value (100-Ohm = 0°C, 375-Ohm = 800°C)
		//first : conversion from raw value to resistance value : 2790=>112 kOhm, 90=>6 kOhm
		//then : conversion from resistance value to temperature value :  112 kOhm=>22 °c, 6 kOhm=>100°c
		compr_temp = res_to_temp_mult * (raw_to_res_mult * raw + raw_to_res_offset) + res_to_temp_offset;

		//si Température critique, arrêter le compresseur et notifier la température critique dans le retour à la Rpi
		/*
		if (compr_temp > COMPR_CRIT_TEMP) {
			my_motor_value[4] = 0;
			return_buffer[1]|=128;
		}
		*/


		if (res1 == HAL_OK){
			switch(command_buffer >> 6){
			case 0:
				//Mode 1 [00000001] : Reset nucleo: arrêt de TOUT
				if (command_buffer == 1){
					// arrêt moteurs (compr, canons, turbine)
					my_motor_value[0] = 0;
					my_motor_value[1] = 0;
					my_motor_value[2] = 0;
					my_motor_value[3] = 0;
					my_motor_value[4] = 0;
					// arrêt EV 1, 2, 3 et Purge
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
					// arrêt LCD et LED Enable
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				}
				//Mode 2 [00000010] : Reset Moteurs
				else if (command_buffer == 2){
					my_motor_value[0] = 0;
					my_motor_value[1] = 0;
					my_motor_value[2] = 0;
					my_motor_value[3] = 0;
					my_motor_value[4] = 0;
					HAL_Delay(2600);
				}

				break;
			case 1:
				//Mode 2 [01]: Canons
				//ordre des canons : left=1 - right=2 - top=3
				//écriture dans le moteur 1 (left)
				if ((command_buffer & 48) >> 4 == 0){
					my_motor_value[0] = 0;
				}
				else if ((command_buffer & 48) >> 4 == 1){
					my_motor_value[0] = CANONS_SPEED_1;
				}
				else if ((command_buffer & 48) >> 4 == 2){
					my_motor_value[0] = CANONS_SPEED_2;
				}
				else{
					my_motor_value[0] = CANONS_SPEED_3;
				}

				//écriture dans le moteur 2 (right)
				if ((command_buffer & 12) >> 2 == 0){
					my_motor_value[1] = 0;
				}
				else if ((command_buffer & 12) >> 2 == 1){
					my_motor_value[1] = CANONS_SPEED_1;
				}
				else if ((command_buffer & 12) >> 2 == 2){
					my_motor_value[1] = CANONS_SPEED_2;
				}
				else{
					my_motor_value[1] = CANONS_SPEED_3;
				}

				//écriture dans le moteur 3 (top)
				if ((command_buffer & 3) == 0){
					my_motor_value[2] = 0;
				}
				else if ((command_buffer & 3) == 1){
					my_motor_value[2] = CANONS_SPEED_1;
				}
				else if ((command_buffer & 3) == 2){
					my_motor_value[2] = CANONS_SPEED_2;
				}
				else{
					my_motor_value[2] = CANONS_SPEED_3;
				}

				break;
			case 2:
				//Mode 3 [10]: Compresseur / INIT
				//[10000000] : arrêt Compresseur + Consigne Pression => 0 + Purge EV4
				if ((command_buffer ^ 128) == 0){
					my_motor_value[4] = 0;
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
					Press_order = 0;
				}
				//[10PPPPPP] : Val de consigne Pression, à récup SSI différente pour régulation au prochain tour de boucle
				Press_order = command_buffer ^ (2 << 6);

				break;
			case 3:
				//Mode 4 [11]: electrovannes OU LED OU Turbine OU LCD
				//cas 1 : Electrovannes
				if ((command_buffer & 240) >> 4 == 12){
					//4*1 bits(ABCE) pour les EV (ordre du code : A: EV1<->PA7, B: EV2<->PA4, C: EV3<->PA3, E: EV_Purge<->PF0)
					HAL_GPIO_WritePin (GPIOA, GPIO_PIN_7, (command_buffer & 0x08) >> 3);
					HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, (command_buffer & 0x04) >> 2);
					HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, (command_buffer & 0x02) >> 1);
					HAL_GPIO_WritePin (GPIOF, GPIO_PIN_0, command_buffer & 0x01);
				}

				//cas 2 : EV-Pulse [1101000Z] => [0] : rien, [1] pulse ON-OFF-ON 1*, avec intervalle t-pulse_OFF
				if ((command_buffer & 240) >> 4 == 13){

					//Pulse EV 1
					if ((command_buffer & 8) >> 3 == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7)){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
						HAL_Delay(PULSE_TIME);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					}
					//Pulse EV 2
					if ((command_buffer & 4) >> 2 == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
						HAL_Delay(PULSE_TIME);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					}
					//Pulse EV 3
					if ((command_buffer & 2) >> 1 == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)){
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
						HAL_Delay(PULSE_TIME);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
					}
				}

				//cas 3 : Turbine [111000ZZ] (arrêt: [00], vitesse_1: [01], vitesse_2: [10], vitesse_3: [11])
				else if ((command_buffer & 240) >> 4 == 14){
					//ecriture valeurs dans moteur Turbine
					if ((command_buffer & 3) == 0){
						my_motor_value[3] = 0;
					}
					else if ((command_buffer & 3) == 1){
						my_motor_value[3] = TURBINE_SPEED_1;
					}
					else if ((command_buffer & 3) == 2){
						my_motor_value[3] = TURBINE_SPEED_2;
					}
					else{
						my_motor_value[3] = TURBINE_SPEED_3;
					}
				}

				//cas 4 : LCD/LED [111100YZ] => Y==0: LCD_reset/LED_reset, Z==1: LCD_set/LED_set
				else if ((command_buffer & 240) >> 4 == 15){
					//LCD
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, command_buffer & 2);

					//LED
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, command_buffer & 1);
				}
				break;
			}
		}

		HAL_Delay(1);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Period = 0;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 24;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_Output_EV_P_GPIO_Port, GPIO_Output_EV_P_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_Output_EV_3_Pin|GPIO_Output_EV_2_Pin|GPIO_Output_EV_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Built_in_LED_Pin|GPIO_Output_LED_EN_Pin|GPIO_Output_LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_Output_EV_P_Pin */
  GPIO_InitStruct.Pin = GPIO_Output_EV_P_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_Output_EV_P_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_Input_AU_STATUS_Pin */
  GPIO_InitStruct.Pin = GPIO_Input_AU_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_Input_AU_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Output_EV_3_Pin GPIO_Output_EV_2_Pin GPIO_Output_EV_1_Pin */
  GPIO_InitStruct.Pin = GPIO_Output_EV_3_Pin|GPIO_Output_EV_2_Pin|GPIO_Output_EV_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Built_in_LED_Pin GPIO_Output_LED_EN_Pin GPIO_Output_LCD_EN_Pin */
  GPIO_InitStruct.Pin = Built_in_LED_Pin|GPIO_Output_LED_EN_Pin|GPIO_Output_LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		dshot_write(my_motor_value);
	}
	//on start la tempo si le BAU est enclenché
	if (BAU_tick_enable == 1){
		BAU_tick++;
		// on purge quand on veut purger, 1000 => 1s
		if (BAU_tick > 30000){
			BAU_tick = 0;
			BAU_tick_enable = 0;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
