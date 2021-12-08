
/*Final Project "Arcade Basketball!"
 * by Christopher A. Mendoza & Evelyn Fernandez*/
#include "msp.h"
#include "lcdLib_432.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

double voltage;
double V;
int period = 60000;

int score;
char buff[16];
int hs = 0;
int s = 0;


//The purpose of this is to send a string to the tx buffer
void transmit(char *str) //
{
    while(*str != 0)
    {
        while((EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG))
            EUSCI_A0->TXBUF = *str++;
    }
}

void uart_init(void)
{
    P1->SEL0 |= BIT2 | BIT3; //configured for UART 1.2 is RxD and 1.3 and TxD on computer
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Reset eUSCI
     EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_B_CTLW0_SSEL__SMCLK; // Use SMCLK as the eUSCI clock source
     EUSCI_A0->BRW = 19; // 12000000/16/9600
     EUSCI_A0->MCTLW = 0xB601;
     EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
     //EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG; // Clear eUSCI RX interrupt flag
     //EUSCI_A0->IE |= EUSCI_A_IE_RXIE;
     NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);
}

int sec = 0;
char Time[16];//global variable that will act as a buffer for the LCD

void LCDInit()
{
     lcdInit();
     lcdClear();
}

void GPIOInit()
{



    P6->DIR &= ~0x07;
    P6->OUT &= ~0x07;
    P6->REN |= 0x07;
    P6->IES |= 0x07;
    P6->IE |= 0x07;
    P6->IFG = 0;
    NVIC->ISER[1] |= BIT8;

}



void PORT6_IRQHandler(){
    //Start timer if switch 5.2 is pressed
       if(P6->IFG & BIT0)
       {
           transmit("Try your hand at this Arcade Basketball!!\r\n");
           delay_ms(1000);
           transmit("Score as many points!\r\n");
           delay_ms(1000);
           lcdClear();
           lcdSetText("Ready?", 5, 1);
           delay_ms(6000);
           lcdClear();
           lcdSetText("3", 7, 1);
           delay_ms(4000);

           lcdSetText("2", 7, 1);
           delay_ms(4000);

           lcdSetText("1", 7, 1);
           delay_ms(4000);

           lcdSetText("START!", 5, 1);
           delay_ms(3000);

           lcdClear();
           SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
           sprintf(Time, "%02d:%02d", s, hs);
               lcdSetText(Time, 5, 1);
       }
       //Clear flags
       P6->IFG = 0;

}


void SysTickInit()
{
    //Setup SysTick but don't enable until user starts it
    SysTick->CTRL |= (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk);
    //Set load value to 3 million to time each second
    SysTick->LOAD = 30000;
    //Set initial value to start counting down from
    SysTick->VAL = 30000;
}

//Interrupt service routine invoked when SysTick down counter changes 0.
void SysTick_Handler()
{

    hs++;
      if(hs == 100)
      {
          s++;
          hs = 0;
      }
      if(s == 20)
      {

          SysTick->CTRL &=  ~(SysTick_CTRL_ENABLE_Msk);
          s = 0;
          hs = 0;
          P2->OUT ^= BIT3;
          delay_ms(500);
          P2->OUT &= ~BIT3;
      }

    sprintf(Time, "%02d:%02d", s, hs);
    lcdSetText(Time, 5, 1);

}

void ButtonInit()
{
//    //debugging thing
//    P1->DIR |= BIT0;
//    P1->OUT |= BIT0;

    //Pressure Sensor
    P5->DIR &= ~BIT0;
    P5->OUT &= ~BIT0;
    P5->REN |= BIT0;
    P5->IES &= ~BIT0;
    P5->IE |= BIT0;
    P5->IFG = 0;
    NVIC->ISER[1] |= BIT7;
    //Buzzer
    P2->DIR |= BIT3;
    P2->OUT |= BIT3;
    P2->OUT &= ~BIT3;
}
void PORT5_IRQHandler()
{

    if(P5->IFG & BIT0)
    {

        score++;
//        lcdClear();
//        lcdSetText("Score!:",0,0);
//        lcdSetInt(score, 0, 0);

        sprintf(buff, "Score: %d ", score);
        lcdSetText(buff,4,0);
       delay_ms(275);


    }
    //Clear flags
    P5->IFG = 0;
}
void updateVoltage()
{
    V = (3.3*voltage)/(4095) - 1.647;

    
//    lcdSetText("Voltage: ", 0, 1);
//    lcdSetDouble(V, 9, 1);
}
void PWMInit()
{
    //Setup pin 2.4 as PWM output
    P2->DIR |= BIT4;
    P2->SEL0 |= BIT4;
    P2->SEL1 &= ~BIT4;

    //Set period to 20 ms since (20/1000)*3e6 = 60000
    TIMER_A0->CCR[0] = 60000;

    //Put into reset/set mode
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;

    //Start motor at 0 degrees with a 5% DC (info from datasheet)
    TIMER_A0->CCR[1] = 0;

    //Setup TimerA
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK
                    TIMER_A_CTL_MC__UP |      // Up mode
                    TIMER_A_CTL_CLR;          // Clear TAR
}

void main(void)
{
    volatile unsigned int i;
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer
	//int i;
	     WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
	     LCDInit();
	     PWMInit();
	     SysTickInit();
	     ButtonInit();
	     GPIOInit();
	     uart_init();


//	     P1->OUT &= ~BIT0; // Clear LED to start
//	     P1->DIR |= BIT0; // Set P1.0/LED to output
	     P5->SEL1 |= BIT4; // Configure P5.4 for ADC
	     P5->SEL0 |= BIT4;

	       SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk; // Enable sleep on exit from ISR
	     __enable_irq();// Enable global interrupt
	     // Enable ADC interrupt in NVIC module
	     NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);

	     // Sampling time, S&H=16, ADC14 on
	     ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
	     ADC14->CTL1 = ADC14_CTL1_RES_2; // Use sampling timer, 12-bit conv. results
	     ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1; // A1 ADC input select; Vref=AVCC
	     ADC14->IER0 |= ADC14_IER0_IE0; // Enable ADC conv complete interrupt
	     SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk; // Wake up on exit from ISR

	     // Ensures SLEEPONEXIT takes effect immediately
	     __DSB();
	         while (1)
	         {
	             for (i = 20000; i > 0; i--); // Delay
	             // Start sampling/conversion
	             ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
	             __sleep();
	             __no_operation(); // For debugger
	          }
}
// ADC14 interrupt service routine
void ADC14_IRQHandler(void)
{
        // ADC12MEM0 = A1 > 0.5AVcc?
        //P1->OUT |= BIT0; // P1.0 = 1
        voltage = ADC14->MEM[0];
        if(V < -.1){
            TIMER_A0->CCR[1] = 3000;

        }
        if(V > 1){
            TIMER_A0->CCR[1] = 4500;

        }

        updateVoltage();
}