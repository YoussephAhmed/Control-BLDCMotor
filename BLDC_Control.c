// DSPIC30F4011 Configuration Bit Settings

// 'C' source line configuration statements

// FOSC
#pragma config FPR = XT_PLL4          // Primary Oscillator Mode (XT w/PLL 4x)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_PWMPIN      // PWM Output Pin Reset (Control with HPOL/LPOL bits)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

#include <xc.h>
#include <libpic30.h>
#include <libq.h>

#define FCY 8000000UL //clock 8 MHz//PLL 4X
#define FPWM 20000
#define MILLISEC FCY/8000// 1 mSec delay constant


void DelayNmSec(unsigned int N);

//FUNCTIONS PROTOTYPES

//ISRs
void __attribute__((__interrupt__,__no_auto_psv__)) _T1Interrupt(void);
void __attribute__((__interrupt__,__no_auto_psv__)) _ADCInterrupt(void);
void __attribute__((__interrupt__,__no_auto_psv__)) _CNInterrupt(void);


//PERIPHERALS INITIALIZATIONS
void PORTS_INIT(void); //PORTx initializations
void TMR1_INIT(void);//TMR1 intialization
void InitADC10(void); //ADC
void InitMCPWM(void); //PWM


//ALGORITHMS
void TOGGLE(void);//TOGGLE PORTB BIT0

//GLOBAL VARIABLE DECLARATIONS
int COUNT;
unsigned int StateLoTable[] = {0x0000, 0x0310, 0x0C01, 0x0C10 ,
0x3004, 0x0304, 0x3001, 0x0000};

unsigned int ReverseStateLoTable[] = {0x0000, 0x3001, 0x0304, 0xC004 ,
0x0C10, 0x0C01, 0x0310, 0x0000};

unsigned int HallValue;
float Duty =0;
int DutyCycle;
int flag =1 ; // to disable CN interrupt when free wheeling
int flag2 = 1; // to have one time delay only in adc interrupt when free wheeling
               // de bouncing delay

int main(){

    //STARTUP DELAY
    DelayNmSec(10);


    //GLOBAL VARIABLES INITIALIZATIONS
    COUNT = 1;
    //PERIPHERALS INITIALIZATIONS
    PORTS_INIT();
    TMR1_INIT();

    //PERIPHERALS START
    T1CONbits.TON = 1;
    InitMCPWM();
    InitADC10();


    // BLDC Control
    LATE = 0x0000;
    TRISE = 0xFFC0; // PWMs are outputs
    CNEN1 = 0x001C; // CN2,3 and 4 enabled
    CNEN2 = 0x0000;
    CNPU1 = 0x001C; // enable internal pullups

    IFS0bits.CNIF = 0; // clear CNIF
    IEC0bits.CNIE = 1; // enable CN interrupt


    // read hall position sensors on PORTB
    HallValue = PORTB & 0x0007; // mask RB0,1 & 3
    OVDCON = StateLoTable[HallValue]; // Load the overide control register

    while(1){

}

}
void PORTS_INIT(){


    TRISFbits.TRISF0 = 0; // Output for arduino interrupt (pin 30)
    TRISFbits.TRISF1 = 0; // Output for micro's heart (pin 29)
    TRISFbits.TRISF4 = 1; // Input for free wheeling (pin 28)
    IEC1 = IEC1 & 0xFEFF; // disable u2rx from pin 28 *****

    LATFbits.LATF0 = 1; // High
    LATFbits.LATF1 = 1; // High
}

void TMR1_INIT(void)
{
    T1CON = 0x0030;//prescale 256:1
    PR1 = 313;
    IFS0bits.T1IF = 0;//clear TMR1 flag
    IPC0 = 0x1000;//priorty level 2
    IEC0bits.T1IE = 1;// enable TMR1 interrupt
}

void TOGGLE(void)
{
      LATFbits.LATF1 =~  LATFbits.LATF1;
}

void __attribute__((__interrupt__,__no_auto_psv__)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0;//clear TMR1 flag

    if(COUNT == 100){COUNT = 1;
        TOGGLE();
    }
    COUNT++;
}

void __attribute__((__interrupt__,__no_auto_psv__)) _CNInterrupt(void)
{
    IFS0bits.CNIF = 0; // clear flag



    HallValue = PORTB & 0x0007; // mask RB0,1 & 2

    if(ADCBUF0 > 170 && flag)
    OVDCON = StateLoTable[HallValue];

    //arduino interrupt
  LATFbits.LATF0 = ~ LATFbits.LATF0;
}



void __attribute__((__interrupt__,__no_auto_psv__)) _ADCInterrupt(void)
{

        IFS0bits.ADIF = 0; // clear flag

    if (ADCBUF0 <= 170 ) // throttle = 0
    {
        DutyCycle = 0;
        OVDCON = 0x0000;
    }

  else
   {
        Duty = ((1.0*(ADCBUF0 - 170))/853)*(1023.0);
        DutyCycle = (int)Duty;


        // read hall position sensors on PORTB
        HallValue = PORTB & 0x0007; // mask RB0,1 & 2
        OVDCON = StateLoTable[HallValue]; // Load the override control register



        //  FREE WHEELING
    if (PORTFbits.RF4 == 1) // yes free wheel
    {
         flag = 0;
         DutyCycle = 0;
         OVDCON = 0x0000;
         if(flag2)
         {
         DelayNmSec(10); // for de bouncing
         flag2 = 0;
         }
    }

    else // not free wheeling
    {
        flag = 1;
        flag2  = 1;
    }



    }



    PDC1 = DutyCycle;
    PDC2 = PDC1; // and load all three PWMs ...
    PDC3 = PDC1; // duty cycles

}


void InitADC10(void)
{
    ADPCFG = 0xFFBF; // all PORTB = Digital;RB6 = analog
    ADCON1 = 0x0064; // PWM starts conversion
    ADCON2 = 0x0200; // simulataneous sample 4 channels
    ADCHS = 0x0006; // Connect RB6/AN6 as CH0 = pot ..
    ADCON3 = 0x0080; // Tad = internal RC (4uS)
    IFS0bits.ADIF = 0;
    IEC0bits.ADIE = 1;
    ADCON1bits.ADON = 1; // turn ADC ON
    IPC2 = 0x7000; //ADC Highest priority
}

void InitMCPWM(void)
{
    PTPER = FCY/FPWM - 1;
    PWMCON1 = 0x0077; // complementary pwm
    OVDCON = 0x0000; // allow control using OVD
    PDC1 = 0; // init PWM 1, 2 and 3 to 100
    PDC2 = 0;
    PDC3 = 0;
    SEVTCMP = PTPER;
    PWMCON2 = 0x0F00; // 16 postscale values
    PTCON = 0x8000; // start PWM
    DTCON1 = 0x0014; // Dead time = 2 us
}

//Generic ms Delay
void DelayNmSec(unsigned int N)
{
unsigned int j;
while(N--)
 for(j=0 ; j < MILLISEC ; j++);
}


