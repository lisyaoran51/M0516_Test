/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/05/22 2:03p $
 * @brief    Generate different frequency(Tenor C Do ~ Si) waveform by PWM.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

// Scale frequency and unit is Hz
#define TENOR_C 523
#define TENOR_D 587
#define TENOR_E 659
#define TENOR_F 698
#define TENOR_G 784
#define TENOR_A 880
#define TENOR_B 988

void PWM_PwmIRQHandler(void);
extern char GetChar(void);
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8PWMCount = 1;
volatile uint16_t g_u16Frequency;
volatile uint32_t g_u32Pulse = 0;

/* Assume PWM output frequency is 523Hz and duty ratio is 60%, user can calculate PWM settings by follows.
   PWM clock source frequency = __HXT = 12000000 in the sample code.
   (CNR+1) = PWM clock source frequency/prescaler/clock source divider/PWM output frequency
           = 12000000/2/1/523 = 11472 < 65536  (Note: If calculated value is larger than 65536, user should increase prescale value.)
   CNR = 11471 =>g_au16ScaleCnr[0] = 11471
   duty ratio = 60% = (CMR+1)/(CNR+1) ==> CMR = (CNR+1)*60/100-1 = 11472*60/100-1
   CMR = 6882 =>g_au16ScaleCmr[0] = 6882
*/
static const uint16_t g_au16ScaleFreq[7] = {TENOR_C, TENOR_D, TENOR_E, TENOR_F, TENOR_G, TENOR_A, TENOR_B};
static const uint16_t g_au16ScaleCnr[7] =  {11471, 10220, 9103, 8594, 7652, 6817, 6071};
static const uint16_t g_au16ScaleCmr[7] =  {6882 , 6131 , 5461, 5156, 4590, 4089, 3642};

/**
 * @brief       PWMA IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWMA interrupt event
 */
void PWMA_IRQHandler(void)
{
    uint32_t u32PwmIntFlag;

    /* Handle PWMA Timer function */
    u32PwmIntFlag = PWMA->PIIR;

    /* PWMB channel 0 PWM timer interrupt */
    if(u32PwmIntFlag & PWM_PIIR_PWMIF0_Msk)
    {
        PWMA->PIIR = PWM_PIIR_PWMIF0_Msk;
        PWM_PwmIRQHandler();
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* PWM Timer function                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void PWM_PwmIRQHandler(void)
{
    if(g_u32Pulse == 1 * g_u16Frequency / 10)
    {
        /*--------------------------------------------------------------------------------------*/
        /* Stop PWMA channel 0 Timer (Recommended procedure method 2)                           */
        /* Set PWM Timer counter as 0, When interrupt request happen, disable PWM Timer         */
        /*--------------------------------------------------------------------------------------*/
        PWMA->CNR0 = 0;
    }

    if(g_u32Pulse == 1 * g_u16Frequency / 10 + 1)
        g_u8PWMCount = 0;
    g_u32Pulse++;
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC22M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & (CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC22M_STB_Msk)));

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

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD  */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0;
    /* Set P4 multi-function pins for PWMA Channel0 */
    //SYS->P4_MFP &= ~(SYS_MFP_P40_Msk);
    //SYS->P4_MFP |= SYS_MFP_P40_PWM0;
		
		/* Set P2 multi-function pins for PWMB Channel0~3  */
		
    SYS->P2_MFP &= ~(SYS_MFP_P20_Msk | SYS_MFP_P21_Msk | SYS_MFP_P22_Msk | SYS_MFP_P23_Msk | SYS_MFP_P24_Msk | SYS_MFP_P25_Msk | SYS_MFP_P26_Msk | SYS_MFP_P27_Msk);
    SYS->P2_MFP |= SYS_MFP_P20_PWM0 | SYS_MFP_P21_PWM1 | SYS_MFP_P22_PWM2 | SYS_MFP_P23_PWM3 | SYS_MFP_P24_PWM4 | SYS_MFP_P25_PWM5 | SYS_MFP_P26_PWM6 | SYS_MFP_P27_PWM7;
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
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
		PWMA->CMR0 = 0x100;
		PWMA->CMR1 = 0x200;
		PWMA->CMR2 = 0x400;
		PWMA->CMR3 = 0x800;
		
		PWMB->CMR0 = 0x100;
		PWMB->CMR1 = 0x200;
		PWMB->CMR2 = 0x400;
		PWMB->CMR3 = 0x800;
	
		/*Set PWM Timer period*/
		//PWMA->CNR0 = g_au16ScaleCnr[(u8Item - '1')];
		PWMA->CNR0 = 0x1000;
		PWMA->CNR1 = 0x1000;
		PWMA->CNR2 = 0x1000;
		PWMA->CNR3 = 0x1000;
		
		PWMB->CNR0 = 0x1000;
		PWMB->CNR1 = 0x1000;
		PWMB->CNR2 = 0x1000;
		PWMB->CNR3 = 0x1000;

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


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Item, u8ItemOK;
		uint32_t u32Src;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWMA channel 0 to drive Buzzer\n");
    printf("  I/O configuration:\n");
    printf("    PWM0(P4.0 PWMA channel 0) <--> Buzzer\n");
    printf("\nPWM Timer Waveform Test. Waveform output(P4.0 PWMA channel 0) to Buzzer\n");
    /* P4.0 PWMA channel 0 generates PWM frequency Do - Si */

    printf("Select Test Item\n");
    printf(" 1: Do (523Hz)Tenor C\n");
    printf(" 2: Re (587Hz)\n");
    printf(" 3: Mi (659Hz)\n");
    printf(" 4: Fa (698Hz)\n");
    printf(" 5: Sol(784Hz)\n");
    printf(" 6: La (880Hz)\n");
    printf(" 7: Si (988Hz)\n");

		u32Src = (CLK->CLKSEL2 & (CLK_CLKSEL2_PWM45_S_Msk << (PWM_CH0 & 2))) >> (CLK_CLKSEL2_PWM45_S_Pos + (PWM_CH0 & 2));
		
		
    printf("clock source %d\n", u32Src);
		
		
		setPwm();
		
		printf("CNR: %d %d %d %d \n", PWMB->CNR0, PWMB->CNR1, PWMB->CNR2, PWMB->CNR3);

		
		printf("swePwm done\n");
		while(1){
			
			PWMA->CMR0++;
			PWMA->CMR1++;
			PWMA->CMR2++;
			PWMA->CMR3++;
			
			PWMB->CMR0++;
			PWMB->CMR1++;
			PWMB->CMR2++;
			PWMB->CMR3++;
			if(PWMA->CMR0 == 0x1000) PWMA->CMR0 = 0x0;
			if(PWMA->CMR1 == 0x1000) PWMA->CMR1 = 0x0;
			if(PWMA->CMR2 == 0x1000) PWMA->CMR2 = 0x0;
			if(PWMA->CMR3 == 0x1000) PWMA->CMR3 = 0x0;
			if(PWMB->CMR0 == 0x1000) PWMB->CMR0 = 0x0;
			if(PWMB->CMR1 == 0x1000) PWMB->CMR1 = 0x0;
			if(PWMB->CMR2 == 0x1000) PWMB->CMR2 = 0x0;
			if(PWMB->CMR3 == 0x1000) PWMB->CMR3 = 0x0;
			
			printf(" ");
			
		}

		printf("xxx\n");

    while(1)
    {
        u8ItemOK = 1;
        //u8Item = GetChar();
        //printf("\t");
        switch(u8Item)
        {
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
            //g_u16Frequency = g_au16ScaleFreq[(u8Item - '1')];
						g_u16Frequency = g_au16ScaleFreq[5];
            break;
        default:
            u8ItemOK = 1;
            break;
        }

        if(u8ItemOK)
        {
            g_u32Pulse = 0;
            g_u8PWMCount = 1;

            /* Assume PWM output frequency is 523Hz and duty ratio is 60%, user can calculate PWM settings by follows.
               duty ratio = (CMR+1)/(CNR+1)
               cycle time = CNR+1
               High level = CMR+1
               PWM clock source frequency = __XTAL = 12000000
               (CNR+1) = PWM clock source frequency/prescaler/clock source divider/PWM output frequency
                       = 12000000/2/1/523 = 11472
               (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
               CNR = 11471
               duty ratio = 60% ==> (CMR+1)/(CNR+1) = 60% ==> CMR = (CNR+1)*0.6-1 = 11472*60/100-1
               CMR = 6882
               Prescale value is 1 : prescaler= 2
               Clock divider is PWM_CSR_DIV1 : clock divider =1
            */
            /*Set Pwm mode*/
            PWMA->PCR |= PWM_PCR_CH0MOD_Msk;

            /*Set PWM Timer clock prescaler*/
            PWM_SET_PRESCALER(PWMA, PWM_CH0, 1); // Divided by 2

            /*Set PWM Timer clock divider select*/
            PWM_SET_DIVIDER(PWMA, PWM_CH0, PWM_CLK_DIV_1);

            /*Set PWM Timer duty*/
            PWMA->CMR0 = g_au16ScaleCmr[(u8Item - '1')];

            /*Set PWM Timer period*/
            //PWMA->CNR0 = g_au16ScaleCnr[(u8Item - '1')];
						PWMA->CNR0 = 0;

            /* Enable PWM Output pin */
            PWMA->POE |= PWM_POE_PWM0_Msk;

            /* Enable Timer period Interrupt */
            PWMA->PIER |= PWM_PIER_PWMIE0_Msk;

            /* Enable PWMB NVIC */
            NVIC_EnableIRQ((IRQn_Type)(PWMA_IRQn));

            /* Enable PWM Timer */
            PWMA->PCR |= PWM_PCR_CH0EN_Msk;

            while(g_u8PWMCount);

            /*--------------------------------------------------------------------------------------*/
            /* Stop PWM Timer (Recommended procedure method 2)                                      */
            /* Set PWM Timer counter as 0, When interrupt request happen, disable PWM Timer         */
            /* Set PWM Timer counter as 0 in Call back function                                     */
            /*--------------------------------------------------------------------------------------*/

            /* Disable PWMA NVIC */
            NVIC_DisableIRQ((IRQn_Type)(PWMA_IRQn));

            /* Wait until PWMA channel 0 Timer Stop */
            while(PWMA->PDR0 != 0);

            /* Disable the PWM Timer */
            PWMA->PCR &= ~PWM_PCR_CH0EN_Msk;

            /* Disable PWM Output pin */
            PWMA->POE &= ~PWM_POE_PWM0_Msk;
        }
    }
}




