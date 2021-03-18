/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/07/13 1:27p $
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>


#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/

void AdcContScanModeTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/* ADC                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
double sliderDataAccumulation[2] = {0};
int sliderDataAverage[2] = {0};
uint8_t adcScanCount = 0;
#define MaxAdcScanTimes 10.0

/*---------------------------------------------------------------------------------------------------------*/
/* indicator lights                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/

uint8_t indicatorLights[16] = {0};

#define LatchPin P14
#define DataPin P15
#define ClockPin P17

/*---------------------------------------------------------------------------------------------------------*/
/* I2C Start                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/

uint8_t i2cWriteData[16][16];		// mcu write pi
uint8_t i2cReadData[16][16];		// mcu read pi

volatile uint8_t i2cWriteDataStartPos = 0;
volatile uint8_t i2cWriteDataEndPos = 0;

volatile uint8_t i2cReadDataStartPos = 0;
volatile uint8_t i2cReadDataEndPos = 0;

uint8_t i2cTempWriteBuffer[16] = {0};
uint8_t i2cTempReadBuffer[16] = {0};

volatile uint8_t writeDataLen = 0;
volatile uint8_t readDataLen = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/* I2C End                                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);
		
// -------PWM-------
		CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
// -------PWM-------end
	
    /* Set PLL to Power down mode and HW will also clear PLL_STB bit in CLKSTATUS register */
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;    
    
    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;
		
// -------PWM-------
		CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC22M_EN_Msk;
// -------PWM-------end
		
    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));
		
		

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
		
// -------PWM-------
		while(!(CLK->CLKSTATUS & (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC22M_STB_Msk)));
// -------PWM-------end
		
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;

// -------PWM-------
		/* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_PWM01_EN_Msk | CLK_APBCLK_PWM23_EN_Msk | CLK_APBCLK_PWM45_EN_Msk | CLK_APBCLK_PWM67_EN_Msk;
    /* IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL;
    /* IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_PWM01_S_HXT | CLK_CLKSEL1_PWM23_S_HXT;
		CLK->CLKSEL2 = CLK_CLKSEL2_PWM45_S_HXT | CLK_CLKSEL2_PWM67_S_HXT;

    /* Reset PWMA channel0~channel3 */
    SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk;
    SYS->IPRSTC2 = 0;
		
		SYS->IPRSTC2 = SYS_IPRSTC2_PWM47_RST_Msk;
    SYS->IPRSTC2 = 0;
// -------PWM-------end

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

// -------ADC-------
		/* Enable ADC module clock */
    CLK->APBCLK |= CLK_APBCLK_ADC_EN_Msk ;
// -------ADC-------end


    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_PLL;

// -------ADC-------
    /* Select ADC module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_ADC_S_Msk ;
    CLK->CLKSEL1 |= CLK_CLKSEL1_ADC_S_HIRC ;
		
		/* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK->CLKDIV  = (CLK->CLKDIV & ~CLK_CLKDIV_ADC_N_Msk) | (((7) - 1) << CLK_CLKDIV_ADC_N_Pos);
// -------ADC-------end

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);
		
// -------PWM-------
		/* Set P2 multi-function pins for PWMB Channel0~3  */
    SYS->P2_MFP &= ~(SYS_MFP_P20_Msk | SYS_MFP_P21_Msk | SYS_MFP_P22_Msk | SYS_MFP_P23_Msk | SYS_MFP_P24_Msk | SYS_MFP_P25_Msk | SYS_MFP_P26_Msk | SYS_MFP_P27_Msk);
    SYS->P2_MFP |= SYS_MFP_P20_PWM0 | SYS_MFP_P21_PWM1 | SYS_MFP_P22_PWM2 | SYS_MFP_P23_PWM3 | SYS_MFP_P24_PWM4 | SYS_MFP_P25_PWM5 | SYS_MFP_P26_PWM6 | SYS_MFP_P27_PWM7;
// -------PWM-------end
		
// -------ADC-------
		/* Configure the P1.0 - P1.3 ADC analog input pins */
    SYS->P1_MFP &= ~(SYS_MFP_P10_Msk | SYS_MFP_P11_Msk | SYS_MFP_P12_Msk); //| SYS_MFP_P13_Msk);
    SYS->P1_MFP |= SYS_MFP_P10_AIN0 | SYS_MFP_P11_AIN1 | SYS_MFP_P12_AIN2; //| SYS_MFP_P13_AIN3 ;
// -------ADC-------end
		
}

void I2C0_SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_I2C0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Configure the SDA0 & SCL0 of I2C0 pins */
    SYS->P3_MFP &= ~(SYS_MFP_P34_Msk | SYS_MFP_P35_Msk);
    SYS->P3_MFP |= (SYS_MFP_P34_SDA0 | SYS_MFP_P35_SCL0);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PLL_CLOCK, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void setPwm(){
	
		printf("setPWM\n");
	
		P2->PMD = (P2->PMD & (~GPIO_PMD_PMD0_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD0_Pos);
    P2->PMD = (P2->PMD & (~GPIO_PMD_PMD1_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD1_Pos);
    P2->PMD = (P2->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD2_Pos);
    P2->PMD = (P2->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD3_Pos);
    P2->PMD = (P2->PMD & (~GPIO_PMD_PMD4_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD4_Pos);
    P2->PMD = (P2->PMD & (~GPIO_PMD_PMD5_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD5_Pos);
    P2->PMD = (P2->PMD & (~GPIO_PMD_PMD6_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD6_Pos);
    P2->PMD = (P2->PMD & (~GPIO_PMD_PMD7_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD7_Pos);
	
		/*Set Pwm mode*/
		PWMA->PCR |= PWM_PCR_CH0MOD_Msk;
		PWMA->PCR |= PWM_PCR_CH1MOD_Msk;
		PWMA->PCR |= PWM_PCR_CH2MOD_Msk;
		PWMA->PCR |= PWM_PCR_CH3MOD_Msk;
	
		PWMB->PCR |= PWM_PCR_CH0MOD_Msk;
		PWMB->PCR |= PWM_PCR_CH1MOD_Msk;
		PWMB->PCR |= PWM_PCR_CH2MOD_Msk;
		PWMB->PCR |= PWM_PCR_CH3MOD_Msk;

		/*Set PWM Timer clock prescaler*/
		PWM_SET_PRESCALER(PWMA, PWM_CH0, 0x01); // Divided by 2
		PWM_SET_PRESCALER(PWMA, PWM_CH1, 0x01); // Divided by 2
		PWM_SET_PRESCALER(PWMA, PWM_CH2, 0x01); // Divided by 2
		PWM_SET_PRESCALER(PWMA, PWM_CH3, 0x01); // Divided by 2
	
		PWM_SET_PRESCALER(PWMB, PWM_CH0, 0x02); // Divided by 2
		PWM_SET_PRESCALER(PWMB, PWM_CH1, 0x02); // Divided by 2
		PWM_SET_PRESCALER(PWMB, PWM_CH2, 0x02); // Divided by 2
		PWM_SET_PRESCALER(PWMB, PWM_CH3, 0x02); // Divided by 2
	
	
		/*Set PWM Timer clock divider select*/
		PWM_SET_DIVIDER(PWMA, PWM_CH0, PWM_CLK_DIV_16);
		PWM_SET_DIVIDER(PWMA, PWM_CH1, PWM_CLK_DIV_16);
		PWM_SET_DIVIDER(PWMA, PWM_CH2, PWM_CLK_DIV_16);
		PWM_SET_DIVIDER(PWMA, PWM_CH3, PWM_CLK_DIV_16);

		PWM_SET_DIVIDER(PWMB, PWM_CH0, PWM_CLK_DIV_16);
		PWM_SET_DIVIDER(PWMB, PWM_CH1, PWM_CLK_DIV_16);
		PWM_SET_DIVIDER(PWMB, PWM_CH2, PWM_CLK_DIV_16);
		PWM_SET_DIVIDER(PWMB, PWM_CH3, PWM_CLK_DIV_16);
		
		/*Set PWM Timer duty*/
		//PWMA->CMR0 = g_au16ScaleCmr[(u8Item - '1')];
		PWMA->CMR0 = 0x0;
		PWMA->CMR1 = 0x0;
		PWMA->CMR2 = 0x0;
		PWMA->CMR3 = 0x0;
		
		PWMB->CMR0 = 0x0;
		PWMB->CMR1 = 0x0;
		PWMB->CMR2 = 0x0;
		PWMB->CMR3 = 0x0;
	
		/*Set PWM Timer period*/
		//PWMA->CNR0 = g_au16ScaleCnr[(u8Item - '1')];
		PWMA->CNR0 = 0x100;
		PWMA->CNR1 = 0x100;
		PWMA->CNR2 = 0x100;
		PWMA->CNR3 = 0x100;
											
		PWMB->CNR0 = 0x100;
		PWMB->CNR1 = 0x100;
		PWMB->CNR2 = 0x100;
		PWMB->CNR3 = 0x100;

		/* Enable PWM Output pin */
		PWMA->POE |= PWM_POE_PWM0_Msk;
		PWMA->POE |= PWM_POE_PWM1_Msk;
		PWMA->POE |= PWM_POE_PWM2_Msk;
		PWMA->POE |= PWM_POE_PWM3_Msk;
		
		PWMB->POE |= PWM_POE_PWM0_Msk;
		PWMB->POE |= PWM_POE_PWM1_Msk;
		PWMB->POE |= PWM_POE_PWM2_Msk;
		PWMB->POE |= PWM_POE_PWM3_Msk;

		/* Enable Timer period Interrupt */
		//PWMA->PIER |= PWM_PIER_PWMIE0_Msk;
		//PWMA->PIER |= PWM_PIER_PWMIE1_Msk;
		//PWMA->PIER |= PWM_PIER_PWMIE2_Msk;
		//PWMA->PIER |= PWM_PIER_PWMIE3_Msk;

		/* Enable PWMB NVIC */
		//NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn));

		/* Enable PWM Timer */
		PWMA->PCR |= PWM_PCR_CH0EN_Msk;
		PWMA->PCR |= PWM_PCR_CH1EN_Msk;
		PWMA->PCR |= PWM_PCR_CH2EN_Msk;
		PWMA->PCR |= PWM_PCR_CH3EN_Msk;
		
		PWMB->PCR |= PWM_PCR_CH0EN_Msk;
		PWMB->PCR |= PWM_PCR_CH1EN_Msk;
		PWMB->PCR |= PWM_PCR_CH2EN_Msk;
		PWMB->PCR |= PWM_PCR_CH3EN_Msk;

		//while(g_u8PWMCount);

		/*--------------------------------------------------------------------------------------*/
		/* Stop PWM Timer (Recommended procedure method 2)                                      */
		/* Set PWM Timer counter as 0, When interrupt request happen, disable PWM Timer         */
		/* Set PWM Timer counter as 0 in Call back function                                     */
		/*--------------------------------------------------------------------------------------*/

		/* Disable PWMA NVIC */
		//NVIC_DisableIRQ((IRQn_Type)(PWMA_IRQn));

		/* Wait until PWMA channel 0 Timer Stop */
		//while(PWMA->PDR0 != 0);

		/* Disable the PWM Timer */
		//PWMA->PCR &= ~PWM_PCR_CH0EN_Msk;

		/* Disable PWM Output pin */
		//PWMA->POE &= ~PWM_POE_PWM0_Msk;
	
}

void setGpio(){
	
		/* Configure P1.2 as Output mode and P4.1 as Input mode then close it */
    //P1->PMD = (P1->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD2_Pos);
    //P4->PMD = (P4->PMD & (~GPIO_PMD_PMD1_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD1_Pos);
		
		// read panel and keyboard
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD0_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD0_Pos);
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD1_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD1_Pos);
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD2_Pos);
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD3_Pos);
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD4_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD4_Pos);
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD5_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD5_Pos);
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD6_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD6_Pos);
		P0->PMD = (P0->PMD & (~GPIO_PMD_PMD7_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD7_Pos);
		
		P3->PMD = (P3->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD2_Pos);
		P3->PMD = (P3->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD3_Pos);
		P3->PMD = (P3->PMD & (~GPIO_PMD_PMD6_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD6_Pos);
		P3->PMD = (P3->PMD & (~GPIO_PMD_PMD7_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD7_Pos);
		
		// indicator lights shift register
		P1->PMD = (P1->PMD & (~GPIO_PMD_PMD4_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD4_Pos);
		P1->PMD = (P1->PMD & (~GPIO_PMD_PMD5_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD5_Pos);
		P1->PMD = (P1->PMD & (~GPIO_PMD_PMD7_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD7_Pos);
		
		// power and pedal
		P4->PMD = (P4->PMD & (~GPIO_PMD_PMD0_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD0_Pos);
		P1->PMD = (P1->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD3_Pos);
	
}


/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C0->I2CSTATUS;

    if(I2C0->I2CTOC & I2C_I2CTOC_TIF_Msk)
    {
        /* Clear I2C0 Timeout Flag */
        I2C0->I2CTOC |= I2C_I2CTOC_TIF_Msk;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C TRx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{  	
		uint32_t i;
	
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        //g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
				i2cTempReadBuffer[readDataLen] = (unsigned char)(I2C0->I2CDAT);
			
				readDataLen++;
			
				if((unsigned char) I2C_GET_DATA(I2C0) == '\0')	/* end of receive */
        {
					i2cReadDataEndPos++;
					if(i2cReadDataEndPos == 16)
						i2cReadDataEndPos = 0;
					
					// overflow so start pos move 1 forward
					if(i2cReadDataEndPos == i2cReadDataStartPos){
						i2cReadDataStartPos++;
						if(i2cReadDataStartPos == 16)
							i2cReadDataStartPos = 0;
					}
					
					memcpy(i2cReadData[i2cReadDataEndPos], i2cTempReadBuffer, 16);
					memset(i2cTempReadBuffer, 0, 16);
					
					readDataLen = 0;
				}
				
				
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xA8)                  /* Own SLA+R has been receive; ACK has been return */
    {
				if(i2cWriteDataStartPos == i2cWriteDataEndPos){
					I2C0->I2CDAT = 0x0;
					//i2cWriteDataStartPos = 0;
					
					memset(i2cTempWriteBuffer, 0, 16);
					
				}
				else{
					//printf("Read!\n");
					memcpy(i2cTempWriteBuffer, i2cWriteData[i2cWriteDataStartPos], 16);
					
					I2C0->I2CDAT = 0x80;
					
					i2cWriteDataStartPos++;
					if(i2cWriteDataStartPos == 16)
						i2cWriteDataStartPos = 0;
					
				}
				
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
		else if(u32Status == 0xB8)                  /* from datasheet to know 0xb8 is how to use TRM_ML51_Series_EN_Rev1.02.pdf */
    {
				if(writeDataLen == 16){
					I2C0->I2CDAT = 0x0;
					//I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI);
				}
				else{
					I2C0->I2CDAT = i2cTempWriteBuffer[writeDataLen++];
				}
			
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
				if(writeDataLen == 16){
					I2C0->I2CDAT = 0x0;
				}
				else{
					I2C0->I2CDAT = i2cTempWriteBuffer[writeDataLen++];
				}
				writeDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        //g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        //g_u8SlvDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
		
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
				I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
    }
}

void I2C0_Init(void)
{
    /* Reset I2C0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->I2CON |= I2C_I2CON_ENS1_Msk;

    /* I2C0 clock divider, I2C Bus Clock = PCLK / (4*125) = 100kHz */
    I2C0->I2CLK = 125 - 1;

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->I2CLK) + 1) << 2)));

    /* Set I2C0 4 Slave Addresses */
    /* Slave Address : 0x15 */
    I2C0->I2CADDR0 = (I2C0->I2CADDR0 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x15 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x35 */
    I2C0->I2CADDR1 = (I2C0->I2CADDR1 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x35 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x55 */
    I2C0->I2CADDR2 = (I2C0->I2CADDR2 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x55 << I2C_I2CADDR_I2CADDR_Pos);
    /* Slave Address : 0x75 */
    I2C0->I2CADDR3 = (I2C0->I2CADDR3 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x75 << I2C_I2CADDR_I2CADDR_Pos);

    /* Set I2C0 4 Slave Addresses Mask Bits*/
    /* Slave Address Mask Bits: 0x01 */
    I2C0->I2CADM0 = (I2C0->I2CADM0 & ~I2C_I2CADM_I2CADM_Msk) | (0x01 << I2C_I2CADM_I2CADM_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->I2CADM1 = (I2C0->I2CADM1 & ~I2C_I2CADM_I2CADM_Msk) | (0x04 << I2C_I2CADM_I2CADM_Pos);
    /* Slave Address Mask Bits: 0x01 */
    I2C0->I2CADM2 = (I2C0->I2CADM2 & ~I2C_I2CADM_I2CADM_Msk) | (0x01 << I2C_I2CADM_I2CADM_Pos);
    /* Slave Address Mask Bits: 0x04 */
    I2C0->I2CADM3 = (I2C0->I2CADM3 & ~I2C_I2CADM_I2CADM_Msk) | (0x04 << I2C_I2CADM_I2CADM_Pos);

    /* Enable I2C0 interrupt and set corresponding NVIC bit */
    I2C0->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->I2CON &= ~I2C_I2CON_EI_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->I2CON &= ~I2C_I2CON_ENS1_Msk;
    CLK->APBCLK &= ~CLK_APBCLK_I2C0_EN_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: AdcContScanModeTest                                                                           */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC continuous scan mode test.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AdcContScanModeTest()
{
    uint32_t u32ChannelCount;
    int32_t  i32ConversionData;
		double conversedData;

    //printf("\n\nConversion rate: %d samples/second\n", ADC_GetConversionRate());
    //printf("\n");
    //printf("+----------------------------------------------------------------------+\n");
    //printf("|                 ADC continuous scan mode sample code                 |\n");
    //printf("+----------------------------------------------------------------------+\n");

    //printf("\nIn this test, software will get 2 cycles of conversion result from the specified channels.\n");

		/* Set the ADC operation mode as continuous scan, input mode as single-end and enable the ADC converter */
		ADC->ADCR = (ADC_ADCR_ADMD_CONTINUOUS | ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADEN_CONVERTER_ENABLE);
		/* Enable analog input channel 0, 1, 2 , without 3, which is 0111 */
		ADC->ADCHER |= ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (0x7));

		/* clear the A/D interrupt flag for safe */
		ADC->ADSR = ADC_ADSR_ADF_Msk;

		/* start A/D conversion */
		ADC->ADCR |= ADC_ADCR_ADST_Msk;
		
		///* Wait conversion done */
		//while(!((ADC->ADSR & ADC_ADSR_ADF_Msk) >> ADC_ADSR_ADF_Pos));
		//
		///* Clear the ADC interrupt flag */
		//ADC->ADSR = ADC_ADSR_ADF_Msk;
		//
		//for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
		//{
		//		i32ConversionData = (ADC->ADDR[(u32ChannelCount)] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;
		//		
		//		conversedData = log((double)(3710 - i32ConversionData)) / log(1.04375021467);//(max3720)
		//		printf("%.3f ", conversedData);
		//}

		/* Wait conversion done */
		while(!((ADC->ADSR & ADC_ADSR_ADF_Msk) >> ADC_ADSR_ADF_Pos));

		/* Stop A/D conversion */
		ADC->ADCR &= ~ADC_ADCR_ADST_Msk;
		
		for(u32ChannelCount = 0; u32ChannelCount < 2; u32ChannelCount++)
		{
				i32ConversionData = (ADC->ADDR[(u32ChannelCount)] & ADC_ADDR_RSLT_Msk) >> ADC_ADDR_RSLT_Pos;
				
				conversedData = log((double)(3710 - i32ConversionData)) / log(1.04375021467);//(max3720)
				sliderDataAccumulation[u32ChannelCount] += conversedData;
				//printf("%.3f ", conversedData);
		}
		//printf("\n ");
		
		/* Clear the ADC interrupt flag */
		ADC->ADSR = ADC_ADSR_ADF_Msk;

    adcScanCount++;
		if(adcScanCount >= MaxAdcScanTimes){
				if((int)(sliderDataAccumulation[0] / MaxAdcScanTimes) > sliderDataAverage[0] + 1 || 
					 (int)(sliderDataAccumulation[0] / MaxAdcScanTimes) < sliderDataAverage[0] - 1){
						sliderDataAverage[0] = (int)(sliderDataAccumulation[0] / MaxAdcScanTimes);
						printf("slider 1:%d\n", sliderDataAverage[0]);
				}
				if((int)(sliderDataAccumulation[1] / MaxAdcScanTimes) > sliderDataAverage[1] + 1 ||
					 (int)(sliderDataAccumulation[1] / MaxAdcScanTimes) < sliderDataAverage[1] - 1){
						sliderDataAverage[1] = (int)(sliderDataAccumulation[1] / MaxAdcScanTimes);
						printf("slider 1:%d\n", sliderDataAverage[1]);
				}
				adcScanCount = 0;
				sliderDataAccumulation[0] = 0;
				sliderDataAccumulation[1] = 0;
		}
    
}

void setI2c(){
	
		uint8_t i, j;
		/* Init I2C0 */
    I2C0_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_I2CON_SI_AA);
		
		printf("+-------------------------------------------------------+\n");
    printf("|  M05xx I2C Driver Sample Code(Slave) for access Slave |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)               |\n");
    printf("+-------------------------------------------------------+\n");

    printf("Configure I2C0 as a slave.\n");
    printf("The I/O connection for I2C0:\n");
    printf("I2C0_SDA(P3.4), I2C0_SCL(P3.5)\n");
		
		/* I2C function to Slave receive/transmit data */
    s_I2C0HandlerFn = I2C_SlaveTRx;
		
		for(i = 0; i < 16; i++){
			for(j = 0; j < 16; j++){
				i2cWriteData[i][j] = 0;
				i2cReadData[i][j] = 0;
			}
		}
}

void SetPin(uint8_t port, uint8_t pin, uint8_t value){
	GPIO_PIN_ADDR(port, pin) = value;
}

uint32_t GetPin(uint8_t port, uint8_t pin){
	return GPIO_PIN_ADDR(port, pin);
}

void ReadPanel(){
	
		
		/*
		*	P0.0~P0.7 is KB in
		* P3.2 KB out A
		* P3.3 KB out B
		* P3.6 KB out C
		* P3.7 KB out Enable
		*/
	
		uint8_t i, j, k;
	
		uint8_t pressed[24] = {0};
		uint8_t pressedNum = 0;
		
		char tempCommand[16];
		char value[4];
			
		for(i = 0; i < 16; i++){
			if(i & 0x8)
				SetPin(3, 7, 1);
			else
				SetPin(3, 7, 0);
			
			if(i & 0x4)
				SetPin(3, 6, 1);
			else
				SetPin(3, 6, 0);
			
			if(i & 0x2)
				SetPin(3, 3, 1);
			else
				SetPin(3, 3, 0);
			
			if(i & 0x1)
				SetPin(3, 2, 1);
			else
				SetPin(3, 2, 0);
			
			for(j = 0; j < 8; j++){
				
				
				
				if(i == 8 || i ==10 || i == 12){
					
					pressedNum = (i-8)/2*8+j;
					if(GetPin(0, j) == 0){
						if(pressed[pressedNum] == 0){
							pressed[pressedNum] = 1;
							
							memset(tempCommand, 0x0, 16);
							sprintf(value, "%03d", 127 + 23 - pressedNum);
							strncpy(tempCommand, value, 3);
							//printf("read input %s \n",tempCommand);
							strncat(tempCommand, ",", 1);
							//tempCommand[4] = ',';
							sprintf(value, "%03d", rand()%128);
							strncat(tempCommand, value, 3);
							//printf("read input %s \n",tempCommand);
							//strncat(tempCommand, "\0", 1);
							
							//printf("end char %d \n",tempCommand[7]);
							
							memcpy(i2cWriteData[i2cWriteDataEndPos], tempCommand, 16);
							i2cWriteDataEndPos++;
							if(i2cWriteDataEndPos == 16)
								i2cWriteDataEndPos = 0;
							
							//printf("read input %s %d\n",tempCommand, i2cWriteDataEndPos);
							printf("read input %d %d \n", i, j);
							
						}
					}
					else
						pressed[pressedNum] = 0;
					
				}
				else{
					
					if(GetPin(0, j) == 0){
						if(i ==14 && (j == 4 || j == 5)){}
						else
								printf("read input i:%d j:%d \n",i, j);
						// lower octave: i=15, j=2
						// raise octave: i=15, j=1
						// left button(velocity): i=14, j=2
						// right button(sustain): i=14, j=1
						// pause: i=15, j=0
						// speed button: i=14, j=3
						// speed forward(clockwise): i=15, j=4
						// speed backward(counterclockwise): i=15, j=5
						// section forward(clockwise): i=14, j=4&5 (->) i=14, j=4 (->) i=14, j=4&5
						// section backward(counterclockwise): i=14, j=4&5 (->) i=14, j=5 (->) i=14, j=4&5
						// pedal: i=15, j=3
					}
				}
			}
		}
		
		//printf("power:%d pedal:%d\n", P40, P13);
		
}

void SetIndicatorLights(int lightNumber, int on){
		
		
		// 0: power
		// 1: sentivity
		// 2: sustain
		// 3: raise octave
		//    *13
		// *8    *12
		// *9    *11
		//    *10
		// 14: lower octave
	
		uint8_t i, j;
		// https://ddddiy.blogspot.com/2014/02/74hct595n.html
		if(on == 1)
			indicatorLights[lightNumber] = 1;
		else if(on == 0)
			indicatorLights[lightNumber] = 0;
	
	
		for(i = 0; i < 16; i++){
				LatchPin = 0;
				for(j = 15; j < 16; j--){
						ClockPin = 0;
						
						if(indicatorLights[j] == 1)
							DataPin = 1;
						else
							DataPin = 0;
						
						ClockPin = 1;
				}
				LatchPin = 1;
			
		}
}

/*
 * position: 0~8
 */
void SetPwmLightRing(int position){
	
		//				*B3
		//		*A0			*B2
		//	*A1					*B1
		//		*A2			*B0
		//				*A3
		
		uint8_t brightnessArray[8] = {0};
		uint8_t i;
		
		for(i = 0;i < 8; i++){
				if(i > position)
					brightnessArray[i] = 0x0;
				else if(i == position)
					brightnessArray[i] = 0xF0;
				else
					brightnessArray[i] = 0x20;
		}
		
		PWMA->CMR0 = brightnessArray[7];
		PWMA->CMR1 = brightnessArray[6];
		PWMA->CMR2 = brightnessArray[5];
		PWMA->CMR3 = brightnessArray[4];
								
		PWMB->CMR0 = brightnessArray[3];
		PWMB->CMR1 = brightnessArray[2];
		PWMB->CMR2 = brightnessArray[1];
		PWMB->CMR3 = brightnessArray[0];
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i32Err;
		int count = 0;
		int lightPos = 0;
		

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    //I2C0_SYS_Init();
		SYS_Init();
		
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();
	/*
        To enable semihost, user must define "DEBUG_ENABLE_SEMIHOST" constant when build code with M051Series BSP.
        This sample code is used to show how to print message/getchar on IDE debug environment.
        It will echo all input character back on UART #1 of KEIL IDE.

        In KEIL MDK, user need to open "View->Serial Window->UART #1" windows in debug mode.
        In IAR Workbench, user need to open "View->Terminal I/O" in debug mode.

        NOTE1: HardFault_Handler handler is implemented in retarget.c.
        NOTE2: Semihost only works with Nuvoton NuLink ICE Dongle in debug mode.
        NOTE3: It does not print any message if Nuvoton NuLink ICE Dongle is not connected.
    */

	//while(1)printf("1\n");
	
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|        All Panel Function Sample Code           |\n");
    printf("+-------------------------------------------------+\n\n");
		
		setPwm();
		
		setGpio();
		
		setI2c();

    
		
		
		/* --------------------main-----------------------*/
		
		while(1){
			
			ReadPanel();
			
			if(count % 100 == 0)
				AdcContScanModeTest();
			
			//if(count % 1000 == 0)
				//printf("%d\n", count);
			
			if(count % 1000 == 50){
					SetPwmLightRing(lightPos / 2);
					SetIndicatorLights(lightPos, 1);
					SetIndicatorLights(lightPos - 1 < 0 ? 15 : lightPos - 1, 0);
					if(++lightPos == 16)
						lightPos = 0;
			}
			
			if(++count > 10000)
				count = 0;
		}
			
		
		
		

    
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
