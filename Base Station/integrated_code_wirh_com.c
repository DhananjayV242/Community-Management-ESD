#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "stdio.h"
#include "string.h"
#include "rt_misc.h"
#include "stm32f4xx_hal_rtc.h"

void FLOWM_start (void);
uint32_t count,timer_count;
float pwm_freq;
static __IO uint32_t usTick;
void DELAY_Init(void);
void DELAY_Us(uint32_t us);
void DELAY_Ms(uint32_t ms);
int data1;
int data2;
uint32_t amountflowedval = 0;
int currenttime, previoustime;
float prev_flow;
uint8_t dummy_data[41]= {0,0,0,0,0,57,0,0,0,0,0,57,0};

#define FLOWM_PORT GPIOB
#define FLOWM_PIN GPIO_PIN_0

GPIO_InitTypeDef GPIO_InitStruct;

//#########################Clock Configuration#################
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}
//##############################################################
void DELAY_Init()
{
	// Configure the SysTick timer to raise interript every 1 us
	SysTick_Config(SystemCoreClock / 1000000);
	//SysTick_Config(1);
}

// SysTick_Handler function will be called every 1 us

void SysTick_Handler()
{
	if (usTick != 0)
	{
		usTick--;
	}
	count++;
	timer_count++;
}

void DELAY_Us(uint32_t us)
{
	// Reload us value
	usTick = us;
	// Wait until usTick reach zero
	while (usTick);
}

void DELAY_Ms(uint32_t ms)
{	// Wait until ms reach zero
	while (ms--)
	{
		// Delay 1ms
		DELAY_Us(1000);
	}
}

void set_gpio_input (void)
{
	/*Configure GPIO pin input: PA2 */
	__HAL_RCC_GPIOB_CLK_ENABLE() ;
  GPIO_InitStruct.Pin = FLOWM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLOWM_PORT, &GPIO_InitStruct);
	
	RCC->AHB1ENR|=(1UL<<3);//Enable GPIOD clock

	GPIOD->MODER &= 0xF7FFFFFF;
	GPIOD->MODER |= 0x04000000;
	GPIOD->PUPDR &= 0xF3FFFFFF;
	GPIOD->OTYPER &=0X0000CFFF;
	GPIOD->OSPEEDR = 0X00000000;
	GPIOD->ODR = 0X00000000;
}

void FLOWM_start (void)
{
	//set_gpio_output();  // set the pin as output
	//HAL_GPIO_WritePin (FLOWM_PORT, FLOWM_PIN, 0);   // pull the pin low
	//DELAY_Ms(18000);   // wait for 18ms
	set_gpio_input();   // set as input

}
float check_response (void)
{
	
	uint32_t i=0;
	float time=0;


	while(i!=10)
	{
			count = 0;
		while ((FLOWM_PORT->IDR & FLOWM_PIN));
		while (!(FLOWM_PORT->IDR & FLOWM_PIN));
			count=0;//CHECK FOR PIN LOW
		while ((FLOWM_PORT->IDR & FLOWM_PIN));  //CHECK FOR PIN HIGH
		while (!(FLOWM_PORT->IDR & FLOWM_PIN));
					time += count;
	i++;

	//while ((HAL_GPIO_ReadPin (FLOWM_PORT, FLOWM_PIN)));   // wait for the pin to go low
}
	time /= 10;
	time *= (((float)100)/32);
	pwm_freq = ((float)1000000)/time;

return pwm_freq;

}


float freq;
float readflowmeter()
{
	freq = check_response();
	float rate_second = freq/450;// millilitres per second (freq is linearly proportional to the rate of flow)(in the range of 33.3 to 200)
	return rate_second;
}

int amountflowed(){

	currenttime = timer_count ;
	timer_count = 0;
	float curr_flow= readflowmeter();
	float i=0;
	i = (curr_flow+prev_flow)*(currenttime)/2000;// interpolates the previous measured value and current measured value to integrate flow
	amountflowedval += i;// currenttime is measured in uS while the rate of flow is in litre/second. Therefore the amount flowed is divided by 1000 to get mL
	prev_flow=curr_flow;
	return amountflowedval;
}

void flowcontrol(uint32_t amount_required)
{
	if (amountflowed()>= amount_required) // amount flowed returns in mL, amount_required is in mL as well.
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);
	else
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13 ,1);
}

uint32_t amount_of_water_to_be_flowed( uint8_t *pointertovalue)//////////////can be modified
{
	uint8_t sum=0;
	uint8_t n0 = 0;
	uint32_t k = 10;///  500 mL for 50% humidity, can be scaled for practical application depending on real world calibrations.
	uint32_t i = 0;
		sum+=pointertovalue[5];
	 n0++;
		sum+=pointertovalue[11];
		n0++;
	uint32_t amountrequired  = (((uint32_t)n0 * (uint32_t)k )/ (uint32_t)sum); // average humidity of the sensors is inversely proportional to the the amount of water that shoulf flow.
	return amountrequired;

}

    ////////////////////////////////////////////////////////////////////////////////////////////


int amount_required_to_flow=0;

/*#####################################################################################################################*/

int get_data_cluster(void);
int uart_counter = 0;
void HAL_ResumeTick(void);
uint8_t dht_packet[10];
int get_data_dht(void);
int current_active_cluster=0;
int cluster1_addr=0x82;
//int cluster2_addr=0x82;
//int cluster3_addr=0x83;
uint8_t a[41];
int flash_write=0;
int bs_addr=0xA1;

void SER_Init(void);
int SER_GetChar (void);
int SER_PutChar (int c);
int send_data_dht(void);
int data1;
int length = 41; //data of one CH is containing 3 nodes + 1 byte to store the address of the clusterhead. 
int flag = 0;
int total_offset = 0;
int wakeup_counter = 0;
int alarm_counter = 0;
RTC_HandleTypeDef        hRTC_Handle;
static RTC_TimeTypeDef 	tRTC_handle;
static RTC_DateTypeDef 	dRTC_handle;
static RTC_AlarmTypeDef aRTC_handle;

/*##################################################################################################################*/
/*								FLASH CONFIGURATION DEFINITIONS														*/
/*##################################################################################################################*/

typedef enum
{
	DATA_TYPE_8=0,
	DATA_TYPE_16,
	DATA_TYPE_32,
}DataTypeDef;

//1. Erase Sector
static void flash_EraseSector(void);
//2. Set Sector Adress
void flash_SetSectorAddrs(uint8_t sector, uint32_t addrs);
//3. Write Flash
void flash_WriteN(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType);
//4. Read Flash
void flash_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType);

static uint32_t MY_SectorAddrs;
static uint8_t MY_SectorNum;

//1. Erase Sector
void FLASH_Unlock(void)
{
	if((FLASH->CR&0x80000000) != 0)
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
	if((FLASH->CR&0x80000000) != 0)
	{
		FLASH_Unlock();
	}
}

void FLASH_Lock(void)
{
	FLASH->CR |= 0x80000000;
}

static void flash_EraseSector(void)
{
	FLASH_Unlock();
	FLASH_Erase_Sector(MY_SectorNum, FLASH_VOLTAGE_RANGE_3);	//Erase the required Flash sector
	FLASH_Lock();
}

//2. Set Sector Adress
void flash_SetSectorAddrs(uint8_t sector, uint32_t addrs)
{
	MY_SectorNum = sector;
	MY_SectorAddrs = addrs;
	//Erase sector before write
	flash_EraseSector();
}

//3. Write Flash
void flash_WriteN(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = MY_SectorAddrs + idx;
	
	//Unlock Flash
	FLASH_Unlock();
	//Write to Flash
	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress , ((uint8_t *)wrBuf)[i]);
					flashAddress++;
				}
			break;
		
		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flashAddress , ((uint16_t *)wrBuf)[i]);
					flashAddress+=2;
				}
			break;
		
		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress , ((uint32_t *)wrBuf)[i]);
					flashAddress+=4;
				}
			break;
	}
	FLASH_Lock();		//Lock the Flash space
}

//4. Read Flash
void flash_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = MY_SectorAddrs + idx;
	
	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint8_t *)rdBuf + i) = *(uint8_t *)flashAddress;
					flashAddress++;
				}
			break;
		
		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint16_t *)rdBuf + i) = *(uint16_t *)flashAddress;
					flashAddress+=2;
				}
			break;
		
		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint32_t *)rdBuf + i) = *(uint32_t *)flashAddress;
					flashAddress+=4;
				}
			break;
	}
}
/*##################################################################################################################*/
/*								UART CONFIGURATION DEFINITIONS														*/
/*##################################################################################################################*/

#ifdef __DBG_ITM
volatile int32_t ITM_RxBuffer;
#endif

void SER_Init (void){
	
	RCC->APB1ENR|=(1UL<<19);//Enable USART 4 clock
	RCC->AHB1ENR|=(1UL<<2);//Enable GPIOC clock
	GPIOC->MODER &=0XFF0FFFFF;
	GPIOC->MODER |=0X00A00000;
	GPIOC->AFR[1]|=0X00008800;//PC10 UART4_Tx, PC11 UART4_Rx (AF8)
	UART4->BRR=0x1117;
	
	UART4->CR1=0X200C;
	UART4->CR1 |= 0x0020; // Enable RX interrupt
	NVIC_EnableIRQ(UART4_IRQn); // Enable IRQ for UART4 in NVIC 
	
}

int id_verify=0;
int offset=0;

int32_t UART4_IRQHandler (void){  
	// RX IRQ part
	data1 = SER_GetChar();
	uart_counter++;
	if((data1 == current_active_cluster)&&(id_verify == 0)){
		a[offset] = current_active_cluster;
		offset++;
		id_verify=1;
	}
	else if(id_verify==1){
		a[offset] = data1;
		offset++;
		if(offset == length){
			offset=0;
			id_verify=0;
			current_active_cluster=current_active_cluster+0x01;
		}
	}
	return 1;
}


int32_t SER_PutChar (int32_t ch){

	while (!(UART4->SR& 0X0080));

	UART4->SR&= 0XFFBF;
	UART4->DR=(ch &0xFF);

	return(ch);
}

int32_t SER_GetChar (void){

	if(UART4->SR & 0X0020)
		return (UART4->DR);

	return(-1);
}


int get_data_cluster(){
current_active_cluster = cluster1_addr;
SER_PutChar(current_active_cluster^bs_addr);
while(current_active_cluster == cluster1_addr);
flash_WriteN(0 ,a,length,DATA_TYPE_8);
flash_write=1;
/*
SER_PutChar(current_active_node);
while(current_active_node == node2_addr);
flash_WriteN(13,a,length,DATA_TYPE_8);
SER_PutChar(current_active_node);
while(current_active_node==node3_addr);
flash_WriteN(26,a,length,DATA_TYPE_8);
*/
current_active_cluster=0x00;
return 1;
}

/*##################################################################################################################*/
/*								RTC CONFIGURATION DEFINITIONS														*/
/*##################################################################################################################*/

void  config_rtc (void){
	
	tRTC_handle.Hours = 00U;
	tRTC_handle.Minutes = 00U;
	tRTC_handle.Seconds = 00U;
	tRTC_handle.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	tRTC_handle.SecondFraction = 00U;
	tRTC_handle.StoreOperation = RTC_STOREOPERATION_SET;
	tRTC_handle.SubSeconds = 00U;
	tRTC_handle.TimeFormat = RTC_HOURFORMAT12_AM;
	
	dRTC_handle.Date = 26U;
	dRTC_handle.Month = 11U;
	dRTC_handle.WeekDay = RTC_WEEKDAY_TUESDAY;
	dRTC_handle.Year = 19U;
	
	aRTC_handle.AlarmMask = RTC_ALARMMASK_NONE;
	aRTC_handle.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	aRTC_handle.AlarmDateWeekDay = 26U;
	aRTC_handle.Alarm = RTC_ALARM_A;
	aRTC_handle.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	aRTC_handle.AlarmTime.Hours = 00U;
	aRTC_handle.AlarmTime.Minutes = 04U;
	aRTC_handle.AlarmTime.Seconds = 00U;
	aRTC_handle.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	aRTC_handle.AlarmTime.SecondFraction = 00U;
	aRTC_handle.AlarmTime.StoreOperation = RTC_STOREOPERATION_SET;
	aRTC_handle.AlarmTime.SubSeconds = 00U;
	aRTC_handle.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
	
}

#define RTC_CLOCK_SOURCE_LSI

#define RTC_ASYNCH_PREDIV       127U
#define RTC_SYNCH_PREDIV        249U

void RTC_WKUP_IRQHandler(void);

/* This function configures the RTC_WKUP as a time base source */
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority){
	
  __IO uint32_t counter = 0U;
	
	config_rtc();//Setting values of time and Date. Also Alarm.
	
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  /* Configue LSI as RTC clock soucre */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK){ 
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) == HAL_OK)
    {
      /* Enable RTC Clock */
      __HAL_RCC_RTC_ENABLE();
     
      hRTC_Handle.Instance = RTC;
      hRTC_Handle.Init.HourFormat = RTC_HOURFORMAT_24;
      hRTC_Handle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
      hRTC_Handle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
      hRTC_Handle.Init.OutPut = RTC_OUTPUT_DISABLE;
      hRTC_Handle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
      hRTC_Handle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
      HAL_RTC_Init(&hRTC_Handle);

      /* Disable the write protection for RTC registers */
      __HAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);

      /* Disable the Wake-up Timer */
      __HAL_RTC_WAKEUPTIMER_DISABLE(&hRTC_Handle);

      /* In case of interrupt mode is used, the interrupt source must disabled */ 
      __HAL_RTC_WAKEUPTIMER_DISABLE_IT(&hRTC_Handle,RTC_IT_WUT);

      /* Wait till RTC WUTWF flag is set  */
      while(__HAL_RTC_WAKEUPTIMER_GET_FLAG(&hRTC_Handle, RTC_FLAG_WUTWF) == RESET){
        if(counter++ == (SystemCoreClock /48U)){
          return HAL_ERROR;
        }
      }

      /* Clear PWR wake up Flag */
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

      /* Clear RTC Wake Up timer Flag */
      __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hRTC_Handle, RTC_FLAG_WUTF);

      /* Configure the Wake-up Timer counter */
      hRTC_Handle.Instance->WUTR = (uint32_t) 30U;

      /* Clear the Wake-up Timer clock source bits in CR register */
      hRTC_Handle.Instance->CR &= (uint32_t)~RTC_CR_WUCKSEL;

      /* Configure the clock source */
      hRTC_Handle.Instance->CR |= (uint32_t)RTC_WAKEUPCLOCK_CK_SPRE_16BITS;

      /* RTC WakeUpTimer Interrupt Configuration: EXTI configuration */
      __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_IT();

      __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_RISING_EDGE();

      /* Configure the Interrupt in the RTC_CR register */
      __HAL_RTC_WAKEUPTIMER_ENABLE_IT(&hRTC_Handle,RTC_IT_WUT);

      /* Enable the Wake-up Timer */
      __HAL_RTC_WAKEUPTIMER_ENABLE(&hRTC_Handle);
			
			RTC_EnterInitMode(&hRTC_Handle);
			
			HAL_RTC_SetTime(&hRTC_Handle, &tRTC_handle ,RTC_FORMAT_BIN);
			
			HAL_RTC_SetDate(&hRTC_Handle, &dRTC_handle ,RTC_FORMAT_BIN);
			
			if(HAL_RTC_WaitForSynchro(&hRTC_Handle)== HAL_OK)
				hRTC_Handle.Instance->ISR &= (uint32_t)~RTC_ISR_INIT;
			
			HAL_RTC_SetAlarm_IT(&hRTC_Handle, &aRTC_handle, RTC_FORMAT_BIN);
			
			/* Enable the write protection for RTC registers */
      __HAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);

      HAL_NVIC_SetPriority(RTC_WKUP_IRQn, TickPriority, 0U);
      HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn); 
			HAL_NVIC_SetPriority(RTC_Alarm_IRQn, TickPriority, 1U);
      HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn); 
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

/* Suspend Tick increment */
void HAL_SuspendTick(void)
{
  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);
  /* Disable WAKE UP TIMER Interrupt */
  __HAL_RTC_WAKEUPTIMER_DISABLE_IT(&hRTC_Handle, RTC_IT_WUT);
  /* Enable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);
}

/* Resume Tick increment */

void HAL_ResumeTick(void){
  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hRTC_Handle);
  /* Enable  WAKE UP TIMER  interrupt */
  __HAL_RTC_WAKEUPTIMER_ENABLE_IT(&hRTC_Handle, RTC_IT_WUT);
  /* Enable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hRTC_Handle);
}

/* Wake Up Timer Event Callback in non blocking mode */

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc){
  HAL_IncTick();
}

/* This function handles  WAKE UP TIMER  interrupt request */

void RTC_WKUP_IRQHandler(void){	
	wakeup_counter++;
	HAL_RTCEx_WakeUpTimerIRQHandler(&hRTC_Handle);
	//HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
}

void RTC_Alarm_IRQHandler (void){
	alarm_counter++;
	get_data_cluster();
	flash_ReadN(0,dummy_data,41,DATA_TYPE_8);
	HAL_RTC_AlarmIRQHandler(&hRTC_Handle);
	amount_required_to_flow = amount_of_water_to_be_flowed(dummy_data);
}

/*##################################################################################################################*/
/*								MAIN FUNCTION																		*/
/*##################################################################################################################*/

int main(void){
	SystemClock_Config(); // Set clock to 168MHz// We will study more abvalue this later
	SER_Init();// Initiates pins for UART communication with the XBee board
	HAL_SuspendTick();
	flash_SetSectorAddrs(11,0x080E0000); // initializes Flash memory
	DELAY_Init(); // initializes sysTick and delay functions
	FLOWM_start(); // initializes flow meter and pump(motor) controller GPIO pins
	while (1){	
		flowcontrol(amount_required_to_flow);
	}
}
/*##################################################################################################################*/
/*								END																					*/
/*##################################################################################################################*/
