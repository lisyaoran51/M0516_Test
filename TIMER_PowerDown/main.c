/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/05/22 2:35p $
 * @brief    Use timer-0 toggle-output interrupt event to wake-up system. 
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M051Series.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

int count = 0;


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u8IsTMR0WakeupFlag = 0;
volatile uint32_t g_au32TMRINTCount[4] = {0};
volatile uint8_t toggle = 0;


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power-down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    printf("\nSystem enter to power-down mode ...\n");

    /* To check if all the debug messages are finished */
    while(IsDebugFifoEmpty() == 0);

    SCB->SCR = 4;

    /* To program PWRCON register, it needs to disable register protection first. */
    CLK->PWRCON = (CLK->PWRCON & ~(CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk)) |
                  CLK_PWRCON_PD_WAIT_CPU_Msk | CLK_PWRCON_PD_WU_INT_EN_Msk;
    CLK->PWRCON |= CLK_PWRCON_PWR_DOWN_EN_Msk;

    __WFI();
}

/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M051Series.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
				toggle = 1;
    }

    if(TIMER_GetWakeupFlag(TIMER0) == 1)
    {
        /* Clear Timer0 wake-up flag */
        TIMER_ClearWakeupFlag(TIMER0);

        g_u8IsTMR0WakeupFlag = 1;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12 MHz XTAL, IRC10K */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC10K_STB_Msk));

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable peripheral clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_TMR0_EN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL | CLK_CLKSEL1_TMR0_S_LIRC;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD, TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |=  (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

    /* Set P3 multi-function pins for T0 */
    SYS->P3_MFP &= ~SYS_MFP_P34_Msk;
    SYS->P3_MFP |= SYS_MFP_P34_T0;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void SetPin(uint8_t port, uint8_t pin, uint8_t value){
	GPIO_PIN_ADDR(port, pin) = value;
}

uint32_t GetPin(uint8_t port, uint8_t pin){
	return GPIO_PIN_ADDR(port, pin);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
		int i, j;
    volatile uint32_t u32InitCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

#if !( __GNUC__ )
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
#endif
    printf("+-------------------------------------------------+\n");
    printf("|    Timer0 Power-down and Wake-up Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer Settings:\n");
    printf("  Timer0: Clock source is LIRC(10 kHz); Toggle-output mode and frequency is 2 Hz; Enable interrupt and wake-up.\n");
    printf("# System will enter to Power-down mode while Timer0 interrupt count is reaching 3;\n");
    printf("  And be waken-up while Timer0 interrupt count is reaching 4.\n\n");

    /* Open Timer0 frequency to 2 Hz in toggle-output mode */
    //TIMER0->TCMPR = __LIRC;
		//TIMER0->TCMPR = (22118400/1080); // 1080hz
		TIMER0->TCMPR = __LIRC / 1000UL;
		TIMER0->TCSR = TIMER_TOGGLE_MODE | TIMER_TCSR_IE_Msk | TIMER_TCSR_WAKE_EN_Msk;
		TIMER0->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);
		
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
		
		while(1){
			if(toggle == 1){
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
						
						if(GetPin(0, j) == 0)
							printf("read input %d %d \n", i, j);
							//count++;
					}
				}
				count++;
				toggle = 0;
			}			
			if(g_au32TMRINTCount[0] != u32InitCount)
			{		
					//printf("Timer0 interrupt count - %d\n", g_au32TMRINTCount[0]);
					if(g_au32TMRINTCount[0] / 1000 > g_au32TMRINTCount[1]){
						printf("scan input %d times\n", count);
						count = 0;
					
						printf("Timer0 interrupt count - %d\n", g_au32TMRINTCount[0]);
						g_au32TMRINTCount[1] = g_au32TMRINTCount[0] / 1000;
					}
				
					u32InitCount = g_au32TMRINTCount[0];
			}
		}
		
		

    u32InitCount = g_u8IsTMR0WakeupFlag = g_au32TMRINTCount[0] = 0;
    while(g_au32TMRINTCount[0] < 10)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
						printf("scan input %d times\n", count);
						count = 0;
					
            printf("Timer0 interrupt count - %d\n", g_au32TMRINTCount[0]);
            if(g_au32TMRINTCount[0] == 1000)
            {
                /* To program PWRCON register, it needs to disable register protection first. */
                SYS_UnlockReg();

                PowerDownFunction();

                /* Check if Timer0 time-out interrupt and wake-up flag occurred */
                while(1)
                {
                    if(((CLK->PWRCON & CLK_PWRCON_PD_WU_STS_Msk) == CLK_PWRCON_PD_WU_STS_Msk) && (g_u8IsTMR0WakeupFlag == 1))
                        break;
                }
                printf("System has been waken-up done. (Timer0 interrupt count is %d)\n\n", g_au32TMRINTCount[0]);
            }
            u32InitCount = g_au32TMRINTCount[0];
        }
    }

    /* Stop Timer0 counting */
    TIMER_Stop(TIMER0);

    printf("*** PASS ***\n");

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
