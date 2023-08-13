

#include "DSP28x_Project.h" // Device Headerfile and Examples Include File
#define __FALSE 0
#define __TRUE 1
typedef struct
{
volatile struct EPWM_REGS *EPwmRegHandle;
Uint16 EPwm_CMPA_Direction;
Uint16 EPwm_CMPB_Direction;
Uint16 EPwmTimerIntCount;
Uint16 EPwmMaxCMPA;
Uint16 EPwmMinCMPA;
Uint16 EPwmMaxCMPB;
Uint16 EPwmMinCMPB;
}EPWM_INFO;


// Global Variables for Inductor 1


int button_status1; //
Uint16 i=0; //Initialize i with a non-zero value to avoid the false value for x[-1], y[-1], etc.
Uint16 current_REF;
//Uint16 Voltage_REF2= 1700;
//Uint16 phase_value = 0;
float phase_value;
float current[]={0};
float x[]={0,0},y[]={0,0};
float d = 0;
float phase_shift;


// Global Variables for Inductor 2


int button_status2; //
//Uint16 i=0; //Initialize i with a non-zero value to avoid the false value for x[-1], y[-1], etc.
Uint16 current_REF0;
//Uint16 Voltage_REF2= 1700;
//Uint16 phase_value = 0;
float phase_value0;
float current0[]={0};
float x0[]={0,0},y0[]={0,0};
float d0 =0;
float phase_shift0;


//Controller coefficients for Inductor 1

//float B10=-0.00009375, B00=-0.00009375 , A10=1; // WORKING CONTROLLER FOR 40MSEC HARDWARE, SOFTWARE 20MSEC(01)

//Controller coefficients for Inductor 2

//float B1=-0.00009375, B0=-0.00009375, A1=1; //
float B10=-0.0000625, B00=-0.0000625 , A10=1; // WORKING CONTROLLER FOR 50MSEC HARDWARE, SOFTWARE 30MSEC(02)
float B1=-0.0000625, B0=-0.0000625, A1=1; //
//float B10=-0.00004375, B00=-0.00004375 , A10=1; // WORKING CONTROLLER FOR 80MSEC HARDWARE, SOFTWARE 20MSEC
//float B1=-0.00004375, B0=-0.00004375, A1=1; //
//float B1=-0.00004375, B0=-0.00004375, A1=1; // 50 rad/s
//y[i]=B1*x[i-1]+B0*x[i]+A1*y[i-1];

//Global Variables for voltage control


int button_status3;
float Voltage_REF; // 60 V is converted to 0.716V with voltage sensor and its digital value is 978
//float Voltage_REF=950;
float Voltage[]={0};
float dv=0;
float xo[]={0,0},yo[]={0,0};
//float B1o=0.0000016,B0o=0.0000016,A1o=1; //when H(s)=0.16/s; and fs=50 kHz, Ts=20 us for z-transform. wc=2 rad/s TBPRD for SOC=1500 (EV LAB Latest)
//float B1o=0.0000375, B0o=0.0000375, A1o=1;//settlingtimeof6msec//
float B1o=0.00008, B0o=0.00008, A1o=1;//settlingtimeof4msec//
float voltage_phase_value;
float phase_shifto;
float THETA;

// Prototype statements for functions found within this file.
void InitEPwm1Example(void);// Module for SOC
void InitEPwm2Example(void);//Module for s1 and s2
void InitEPwm3Example(void);//Module for s3 and s4
void InitEPwm4Example(void);//Module for s7 and its complimrnt is for s8
//void InitEPwm5Example(void);//Left for reference change
void InitEPwm6Example(void);//Module for s5 and s6
;
//__interrupt void epwm1_isr(void);
//__interrupt void epwm2_isr(void);
//__interrupt void epwm3_isr(void);
//__interrupt void epwm4_isr(void);
__interrupt void adc_isr(void);
//void update_compare(EPWM_INFO*);

// Global variables used in this example
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;
EPWM_INFO epwm4_info;
//EPWM_INFO epwm5_info;
EPWM_INFO epwm6_info;

#define ADC_CKPS 0x1 // ADC module clock = HSPCLK/2*ADC_CKPS = 25.0MHz/(1*2) = 12.5MHz, The ADCCLKPS[3:0] bits of the ADCTRL3 register. See Figure 7-2. Page 448/868
// Configure the period for each timer
#define EPWM1_TIMER_TBPRD 937 // for 80Khz frequency in UpDown Counter
//#define EPWM1_MAX_CMPA 750
//#define EPWM1_MIN_CMPA 50
#define EPWM1_CMPA 937 // for 80Khz frequency in UpDown Counter
//#define EPWM1_MAX_CMPB 750
//#define EPWM1_MIN_CMPB 50
#define EPWM1_CMPB 1000 // for 0.5 duty cycle
#define EPWM1_DB 10
#define EPWM2_TIMER_TBPRD 937 // for 80Khz frequency in UpDown Counter
//#define EPWM2_MAX_CMPA 750
//#define EPWM2_MIN_CMPA 50
#define EPWM2_CMPA 1000 // for 0.5 duty cycle
//#define EPWM2_MAX_CMPB 750
//#define EPWM2_MIN_CMPB 50
#define EPWM2_CMPB 700 // for 0.5 duty cycle
#define EPWM2_DB 10
#define EPWM3_TIMER_TBPRD 937 // for 80Khz frequency in UpDown Counter
#define EPWM3_CMPA 1000
#define EPWM3_CMPB 500
#define EPWM3_DB 10
//#define EPWM3_MIN_CMPB 50

#define EPWM4_TIMER_TBPRD 937 // for 80Khz frequency in UpDown Counter
#define EPWM4_CMPA 1000
#define EPWM4_CMPB 500
#define EPWM4_DB 10
//#define EPWM3_MIN_CMPB 50
// To keep track of which way the compare value is moving
#define EPWM_CMP_UP 1
#define EPWM_CMP_DOWN 0



void main(void)
{

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
InitSysCtrl();
EALLOW;
#if (CPU_FRQ_150MHZ) // Default - 150 MHz SYSCLKOUT
#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3) = 25.0 MHz
#endif
#if (CPU_FRQ_100MHZ)
#define ADC_MODCLK 0x2 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 100/(2*2) = 25.0 MHz
#endif
EDIS;
// gpio configuration
//GPIO8 is used for reference change of Inductor 1 and Inductor 2
//GPIO9 is used for reference change of Voltage
EALLOW;
GpioCtrlRegs.GPAMUX1.bit.GPIO8 =0;// selecting the gpio pin
GpioCtrlRegs.GPADIR.bit.GPIO8 =0;//1 mean output pin,0 mean input pin
GpioCtrlRegs.GPAPUD.bit.GPIO8 =0;// 0 PULL UP ENABLED 1 PULL UP DISABLE

GpioCtrlRegs.GPAMUX1.bit.GPIO9 =0;// selecting the gpio pin
GpioCtrlRegs.GPADIR.bit.GPIO9 =0;//1 mean output pin,0 mean input pin
GpioCtrlRegs.GPAPUD.bit.GPIO9 =0;// 0 PULL UP ENABLED 1 PULL UP DISABLE

EDIS;

EALLOW;
SysCtrlRegs.HISPCP.all = ADC_MODCLK;
EDIS;

// Step 2. Initialize GPIO:
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio(); // Skipped for this example

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2833x_EPwm.c file


InitEPwm1Gpio();// FOR SOC
InitEPwm2Gpio();//FOR S1 AND S2
InitEPwm3Gpio();//FOR S3 AND S4
InitEPwm4Gpio();//FOR S7(A) AND S8(B)
//InitEPwm5Gpio();//LEFT FOR REFERENCE CHANGE
InitEPwm6Gpio();//FOR S5 AND S6



// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the DSP2833x_PieCtrl.c file.
InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
IER = 0x0000;
IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example. This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
EALLOW; // This is needed to write to EALLOW protected registers
//PieVectTable.EPWM1_INT = &epwm1_isr;
//PieVectTable.EPWM2_INT = &epwm2_isr;
// PieVectTable.EPWM3_INT = &epwm3_isr;
//PieVectTable.EPWM4_INT = &epwm4_isr;
PieVectTable.ADCINT = &adc_isr;
EDIS; // This is needed to disable write to EALLOW protected registers


// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example

// For this example, only initialize the ePWM

EALLOW;
SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
EDIS;

InitEPwm1Example();
InitEPwm2Example();
InitEPwm3Example();
InitEPwm4Example();
//InitEPwm5Example();
InitEPwm6Example();
InitAdc();// For close loop, initialize the ADC

EALLOW;
SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
EDIS;



// Step 5. User specific code, enable interrupts:

// Enable CPU INT3 which is connected to EPWM1-3 INT:
//IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
// PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
// PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
// PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
// PieCtrlRegs.PIEIER3.bit.INTx4 = 1;
// Enable ADCINT in PIE
PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
IER |= M_INT1; // Enable CPU Interrupt 1
// Enable global Interrupts and higher priority real-time debug events:
EINT; // Enable Global interrupt INTM
ERTM; // Enable Global realtime interrupt DBGM

// Configure ADC
AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS; // The ADCCLKPS[3:0] bits of the ADCTRL3 register. Since above #define ADC_CKPS 0x1. ADC module clock = HSPCLK/2*ADC_CKPS = 25.0MHz/(1*2) = 12.5MHz
AdcRegs.ADCMAXCONV.all = 0x0002; // Setup 1 conv's on SEQ1
// AdcRegs.ADCMAXCONV.all = 0x0000; // Setup 1 conv's on SEQ1
AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; // Setup ADCINA0 as 1st SEQ1 conv.
AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1; // Setup ADCINA1 as 1st SEQ1 conv.
AdcRegs.ADCCHSELSEQ1.bit.CONV02 = 0x2; // Setup ADCINA2 as 1st SEQ1 conv.
AdcRegs.ADCTRL1.bit.CONT_RUN = 1; // Setup continuous run
AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;// Enable SOCA from ePWM to start SEQ1
AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1; // Enable SEQ1 interrupt (every EOS)
// Step 6. IDLE loop. Just sit and loop forever (optional):
for(;;)
{
__asm(" NOP");
}
}

__interrupt void adc_isr(void)

{
current[0] = AdcRegs.ADCRESULT0>>4; //for Inductor 1
current0[0] = AdcRegs.ADCRESULT1>>4; //for inductor 2
Voltage[0] = AdcRegs.ADCRESULT2 >>4; //for output voltage
// here read input data when you press the button read the button status
// when you press the button check the status
// if button status == 0 then turn of the led
// else button status == 1 then turn on the led
button_status1 = GpioDataRegs.GPADAT.bit.GPIO8;
if(button_status1 == __FALSE)
{
current_REF = 455;
}
else
{
current_REF = 700;
}







x[1]=current_REF-current[0];
y[1]=B1*x[0]+B0*x[1]+A1*y[0]; //y[i]=B1*x[i-1]+B0*x[i]+A1*y[i-1];






d = y[1]/EPWM2_TIMER_TBPRD; //define d duty-ratio as division y[1]/EPWM2_TIMER_TBPRD, here EPWM2_TIMER_TBPRD is fixed with value 3750
phase_shift = d;

//Limiter
//Three conditions: 1) if phase_shift>=160, then phase_shift= 170, 2) if phase_shift <= 20, then phase_shift = 5, 3) if 20<phase_shift<160, then phase_shift = phase_shift
if(phase_shift>=0.5)
{
phase_shift=0.45;
}

else if(phase_shift<=0.1)
{
phase_shift = 0.1;
}


button_status2 = GpioDataRegs.GPADAT.bit.GPIO8;
if(button_status2 == __FALSE)
{
current_REF0 = 450; //for 250 volts
}
else
{
current_REF0 = 700; // for 280 volts
}

//Voltage_REF = 939; //for 20 volts





x0[1]=current_REF0-current0[0];
y0[1]=B10*x0[0]+B00*x0[1]+A10*y0[0]; //y[i]=B1*x[i-1]+B0*x[i]+A1*y[i-1];




//Limiter
//Three conditions: 1) if y[1]>=TBPRD, then y[1]=TBPRD, 2) if y[1]<=0, then y[1]=0, 3) if 0<y[1]<TBPRD, then y[1]=y[1]
/*if(y[1]>=3000)
{
y[1]=3000;
}

else if(y[1]<=200)
{
y[1]=200;
}*/

d0 = y0[1]/EPWM2_TIMER_TBPRD; //define d duty-ratio as division y[1]/EPWM2_TIMER_TBPRD, here EPWM2_TIMER_TBPRD is fixed with value 3750
phase_shift0 = d0;

//Limiter
//Three conditions: 1) if phase_shift>=160, then phase_shift= 170, 2) if phase_shift <= 20, then phase_shift = 5, 3) if 20<phase_shift<160, then phase_shift = phase_shift
if(phase_shift0>=0.5)
{
phase_shift0=0.45;
}

else if(phase_shift0<=0.1)
{
phase_shift0 = 0.1;
}

button_status3 = GpioDataRegs.GPADAT.bit.GPIO9;
if(button_status3 == __FALSE)
{
    //Voltage_REF = 1677; //for 280 volts
    Voltage_REF = 1730;
    //Voltage_REF = 1794; //for 280 volts
}
else
{
    //Voltage_REF = 2032; // for 340 volts

Voltage_REF = 2164; // for 340 volts
}



xo[1]=Voltage_REF-Voltage[0];

yo[1]=B1o*xo[0]+B0o*xo[1]+A1o*yo[0];


dv=yo[1]/EPWM2_TIMER_TBPRD;



if(dv>=0.4)

{

dv=0.4;

}


else if(dv<=0.05)

{

dv=0.05;

}
THETA= dv*180;

phase_shifto = dv;

//phase_value = 2*(0.3*EPWM2_TIMER_TBPRD);//for Inductor 1
////
//phase_value0 = 2*(0.3*EPWM2_TIMER_TBPRD);//for inductor 2
phase_value = 2*(phase_shift*EPWM2_TIMER_TBPRD);//for Inductor 1

phase_value0 = 2*(phase_shift0*EPWM2_TIMER_TBPRD);//for inductor 2

voltage_phase_value = (phase_shifto*EPWM2_TIMER_TBPRD);//for output voltage


//set actions

//S1 S2
EPwm2Regs.CMPA.half.CMPA = phase_value; // Set compare A value
EPwm2Regs.AQCTLA.bit.ZRO=AQ_SET; // Set PWM2A on event A, up count
EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM2A on event A, down count



//S3 S4
EPwm3Regs.CMPA.half.CMPA = 937-phase_value0;
EPwm3Regs.AQCTLA.bit.PRD= AQ_SET; // Set PWM2A on event A, up count
EPwm3Regs.AQCTLA.bit.CAD= AQ_CLEAR; // Clear PWM2A on event A, down count

EPwm2Regs.TBPHS.half.TBPHS = 0x0000;
EPwm3Regs.TBPHS.half.TBPHS = 0x0000;


//for S7
//EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;
EPwm4Regs.TBPHS.half.TBPHS = voltage_phase_value;// SINCE UPDOWN SO 937 FOR UP AND 937 FOR DOWN MAKES TOTAL 360 DEGREE

EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;
EPwm4Regs.CMPA.half.CMPA = phase_value; // Set compare A value
EPwm4Regs.AQCTLA.bit.ZRO=AQ_SET; // Set PWM2A on event A, up count
EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM2A on event A, down count

//for S8

EPwm4Regs.CMPB = 937-phase_value0;
EPwm4Regs.AQCTLB.bit.PRD= AQ_SET; // Set PWM2A on event A, up count
EPwm4Regs.AQCTLB.bit.CBD= AQ_CLEAR; // Clear PWM2A on event A, down count

//for S5 and S6
EPwm6Regs.TBPHS.half.TBPHS = voltage_phase_value;
EPwm6Regs.CMPA.half.CMPA =phase_value ;//voltage_phase_value;//by dk voltage control value
EPwm6Regs.CMPB= phase_value;//phase_value;//voltage_phase_value+phase_value;

EPwm6Regs.AQCTLB.bit.CAU=AQ_SET;
EPwm6Regs.AQCTLB.bit.PRD=AQ_CLEAR;

EPwm6Regs.AQCTLB.bit.CBD=AQ_SET;
EPwm6Regs.AQCTLB.bit.ZRO=AQ_CLEAR;


//EPwm6Regs.AQCTLA.bit.CAU=AQ_SET;
//EPwm6Regs.AQCTLA.bit.PRD=AQ_CLEAR;
//
//EPwm6Regs.AQCTLA.bit.CBD=AQ_SET;
//EPwm6Regs.AQCTLA.bit.ZRO=AQ_CLEAR;

//phase-shift for EPwm2 register
x[0]=x[1];
y[0]=y[1];

x0[0]=x0[1];
y0[0]=y0[1];

xo[0]=xo[1];
yo[0]=yo[1];

// Reinitialize for next ADC sequence
AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1; // Reset SEQ1
AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // Clear INT SEQ1 bit
PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Acknowledge interrupt to PIE

//EPwm2Regs.TBPHS.half.TBPHS = -6000; //phase-shift for EPwm2 register
return;

}
void InitEPwm1Example()
{
// Setup TBCLK
EPwm1Regs.TBPRD = EPWM3_TIMER_TBPRD; // Set timer period
EPwm1Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
EPwm1Regs.TBCTR = 0x0000; // Clear counter

// Set Compare values
EPwm1Regs.CMPA.half.CMPA = 10; // Set compare A value
//EPwm2Regs.CMPB = EPWM2_CMPB; // Set Compare B value

// Setup counter mode
EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
//EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading. It means "EPwm1Regs.TBCTL.bit.PHSEN = 0x01;"
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  // EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;  //  It means "EPwm1Regs.TBCTL.bit.SYNCOSEL = 0x01;"
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

// Setup shadowing
EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


// Set actions
EPwm1Regs.AQCTLA.bit.CAD = AQ_SET; // Set PWM2A on event A, down count
EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM2A on event A, up count

EPwm1Regs.ETSEL.bit.SOCAEN = 1; // Enable SOC on A group
//EPwm3Regs.ETSEL.bit.SOCASEL = 4; // Select SOC from from CPMA on upcount
EPwm1Regs.ETSEL.bit.SOCASEL = 1; // Select SOC from TBCTR equal to zero
EPwm1Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event

// Information this example uses to keep track of the direction the CMPA/CMPB values are moving, the min and max allowed values and a pointer to the correct ePWM register
epwm1_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
epwm1_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // increasing CMPB
epwm1_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
epwm1_info.EPwmRegHandle = &EPwm1Regs; // Set the pointer to the ePWM module
}

void InitEPwm2Example()
{
// Setup TBCLK
EPwm2Regs.TBPRD = EPWM2_TIMER_TBPRD; // Set timer period 801 TBCLKs
//EPwm2Regs.TBPHS.half.TBPHS = 0; // Phase is 0
//EPwm2Regs.TBPHS.half.TBPHS = 220; // Phase is 45 deg
// EPwm2Regs.TBPHS.half.TBPHS = -6000; //zero phase value for EPWM1
// EPwm2Regs.TBPHS.half.TBPHS = phase_value; //phase-shift for EPwm2 register
//EPwm2Regs.TBPHS.half.TBPHS = 882; // Phase is 180 deg
EPwm2Regs.TBCTR = 0x0000; // Clear counter

// Set Compare values
//EPwm2Regs.CMPA.half.CMPA = EPWM2_CMPA; // Set compare A value
EPwm2Regs.CMPB = EPWM2_CMPB; // Set Compare B value

// Setup counter mode
EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Enable phase loading. It means "EPwm2Regs.TBCTL.bit.PHSEN = 0x01;"
EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // It means " EPwm2Regs.TBCTL.bit.SYNCOSEL = 0x00;"
EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

// Setup shadowing
EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

// Set actions
EPwm2Regs.AQCTLA.bit.ZRO=AQ_SET; // Set PWM2A on event A, up count
EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM2A on event A, down count

EPwm2Regs.AQCTLB.bit.CAD = AQ_SET; // Set PWM2B on event A, down count
EPwm2Regs.AQCTLB.bit.CAU = AQ_CLEAR; // Clear PWM2B on event A, up count

// Active high complementary PWMs - Setup the deadband
EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
EPwm2Regs.DBRED = EPWM2_DB;
EPwm2Regs.DBFED = EPWM2_DB;
//EPwm2_DB_Direction = DB_UP;

// Interrupt where we will change the Compare Values
//EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
EPwm2Regs.ETSEL.bit.INTEN = 1; // Enable INT
//EPwm2Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event

// Information this example uses to keep track
// of the direction the CMPA/CMPB values are
// moving, the min and max allowed values and
// a pointer to the correct ePWM registers
epwm2_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
epwm2_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // increasing CMPB
epwm2_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
epwm2_info.EPwmRegHandle = &EPwm2Regs; // Set the pointer to the ePWM module
// epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA; // Setup min/max CMPA/CMPB values
// epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
// epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
// epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;
}

void InitEPwm3Example(void)
{
// Setup TBCLK
EPwm3Regs.TBPRD = EPWM3_TIMER_TBPRD; // Set timer period 801 TBCLKs
//EPwm3Regs.TBPHS.half.TBPHS = 0; // Phase is 0
//EPwm2Regs.TBPHS.half.TBPHS = 220; // Phase is 45 deg
// EPwm3Regs.TBPHS.half.TBPHS = phase_value; // Phase is 30 deg
//EPwm2Regs.TBPHS.half.TBPHS = 882; // Phase is 180 deg
EPwm3Regs.TBCTR = 0x0000; // Clear counter

// Set Compare values
//EPwm3Regs.CMPA.half.CMPA = EPWM3_CMPA; // Set compare A value
//EPwm3Regs.CMPB = EPWM3_CMPB; // Set Compare B value

// Setup counter mode
EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Enable phase loading. It means "EPwm2Regs.TBCTL.bit.PHSEN = 0x01;"
EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // It means " EPwm2Regs.TBCTL.bit.SYNCOSEL = 0x00;"
EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

// Setup shadowing
EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

// Set actions
EPwm3Regs.AQCTLA.bit.PRD= AQ_SET; // Set PWM2A on event A, up count
EPwm3Regs.AQCTLA.bit.CAD= AQ_CLEAR; // Clear PWM2A on event A, down count

EPwm3Regs.AQCTLB.bit.CAD = AQ_SET; // Set PWM2B on event A, down count
EPwm3Regs.AQCTLB.bit.CAU = AQ_CLEAR; // Clear PWM2B on event A, up count

// Active high complementary PWMs - Setup the deadband
EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
EPwm3Regs.DBRED = EPWM2_DB;
EPwm3Regs.DBFED = EPWM2_DB;
//EPwm2_DB_Direction = DB_UP;

// Interrupt where we will change the Compare Values
// EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
EPwm3Regs.ETSEL.bit.INTEN = 0; // Enable INT
// EPwm3Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event

// Information this example uses to keep track
// of the direction the CMPA/CMPB values are
// moving, the min and max allowed values and
// a pointer to the correct ePWM registers
epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // increasing CMPB
epwm3_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
epwm3_info.EPwmRegHandle = &EPwm3Regs; // Set the pointer to the ePWM module
// epwm2_info.EPwmMaxCMPA = EPWM2_MAX_CMPA; // Setup min/max CMPA/CMPB values
// epwm2_info.EPwmMinCMPA = EPWM2_MIN_CMPA;
// epwm2_info.EPwmMaxCMPB = EPWM2_MAX_CMPB;
// epwm2_info.EPwmMinCMPB = EPWM2_MIN_CMPB;
}
void InitEPwm4Example()
{
// Setup TBCLK
EPwm4Regs.TBPRD = EPWM4_TIMER_TBPRD; // Set timer period 801 TBCLKs
//EPwm4Regs.TBCTL.bit.PHSDIR=TB_DOWN;
EPwm4Regs.TBCTL.bit.PHSDIR=TB_UP;
EPwm4Regs.TBPHS.half.TBPHS = voltage_phase_value; // Phase is 0
//EPwm4Regs.TBCTR = voltage_phase_value;;//voltage_phase_value; // Clear counter

// Set Compare values
//EPwm4Regs.CMPA.half.CMPA = EPWM4_CMPA; // Set compare A value
//EPwm4Regs.CMPB = EPWM4_CMPB; // Set Compare B value

// Setup counter mode
EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Enable phase loading. It means "EPwm1Regs.TBCTL.bit.PHSEN = 0x01;"
EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // It means "EPwm1Regs.TBCTL.bit.SYNCOSEL = 0x01;"
EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

// Setup shadowing
EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


// Set actions
//
// EPwm4Regs.AQCTLB.bit.CAD = AQ_SET; // Set PWM1B on event A, down count
// EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR; // Clear PWM1B on event A, up count

// Active high complementary PWMs - Setup the deadband
//EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//EPwm4Regs.DBRED = EPWM4_DB;
//EPwm4Regs.DBFED = EPWM4_DB;
//EPwm1_DB_Direction = DB_UP;

// Interrupt where we will change the Compare Values
// EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
EPwm4Regs.ETSEL.bit.INTEN = 0; // Enable INT
// EPwm4Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event

// Information this example uses to keep track
// of the direction the CMPA/CMPB values are
// moving, the min and max allowed values and
// a pointer to the correct ePWM registers
epwm4_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
epwm4_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
epwm4_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
epwm4_info.EPwmRegHandle = &EPwm4Regs; // Set the pointer to the ePWM module
// epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
// epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
// epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
// epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}
/*void InitEPwm5Example()
{
//EPWM3 is responsible to generate the Start of Conversion (SOC) for the ADC. Here also, TBPRD is fixed for 40 kHz. And SOC occurs for each TBCTR equals to zero.

// Setup TBCLK
EPwm5Regs.TBPRD = EPWM3_TIMER_TBPRD; // Set timer period
EPwm5Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
EPwm5Regs.TBCTR = 0x0000; // Clear counter

// Set Compare values
EPwm5Regs.CMPA.half.CMPA = 60; // Set compare A value
//EPwm2Regs.CMPB = EPWM2_CMPB; // Set Compare B value

// Setup counter mode
EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Count up
EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

// Setup shadowing
EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


// Set actions
// EPwm5Regs.AQCTLA.bit.CAD = AQ_SET; // Set PWM2A on event A, down count
// EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM2A on event A, up count

// Active high complementary PWMs - Setup the deadband
//EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//EPwm5Regs.DBRED = 50;
//EPwm5Regs.DBFED = 50;
//EPwm1_DB_Direction = DB_UP;

// Interrupt where we will change the Compare Values
// EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
EPwm5Regs.ETSEL.bit.INTEN = 0; // Enable INT
// EPwm4Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event

// Information this example uses to keep track
// of the direction the CMPA/CMPB values are
// moving, the min and max allowed values and
// a pointer to the correct ePWM registers
epwm5_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
//epwm5_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
epwm5_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
epwm5_info.EPwmRegHandle = &EPwm5Regs; // Set the pointer to the ePWM module
// epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
// epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
// epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
// epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;

}*/

void InitEPwm6Example()
{


// Setup TBCLK
EPwm6Regs.TBPRD = EPWM3_TIMER_TBPRD; // Set timer period
//EPwm6Regs.TBCTL.bit.PHSDIR=TB_UP;
//EPwm6Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
EPwm6Regs.TBCTR = voltage_phase_value; // Clear counter

// Set Compare values
//EPwm6Regs.CMPA.half.CMPA = 60; // Set compare A value
//EPwm2Regs.CMPB = EPWM2_CMPB; // Set Compare B value

// Setup counter mode
EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;// Disable phase loading
EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

// Setup shadowing
EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // Load on Zero
EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


// Set actions
// EPwm6Regs.AQCTLA.bit.CAD = AQ_SET; // Set PWM2A on event A, down count
// EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR; // Clear PWM2A on event A, up count

// Active high complementary PWMs - Setup the deadband
//EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
//EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
//EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
//EPwm6Regs.DBRED = EPWM4_DB;
//EPwm6Regs.DBFED = EPWM4_DB;
//EPwm1_DB_Direction = DB_UP;

// Interrupt where we will change the Compare Values
// EPwm4Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
EPwm6Regs.ETSEL.bit.INTEN = 0; // Enable INT
// EPwm4Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event

// Information this example uses to keep track
// of the direction the CMPA/CMPB values are
// moving, the min and max allowed values and
// a pointer to the correct ePWM registers
epwm6_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
epwm6_info.EPwm_CMPB_Direction = EPWM_CMP_UP; // decreasing CMPB
epwm6_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
epwm6_info.EPwmRegHandle = &EPwm6Regs; // Set the pointer to the ePWM module
// epwm1_info.EPwmMaxCMPA = EPWM1_MAX_CMPA; // Setup min/max CMPA/CMPB values
// epwm1_info.EPwmMinCMPA = EPWM1_MIN_CMPA;
// epwm1_info.EPwmMaxCMPB = EPWM1_MAX_CMPB;
// epwm1_info.EPwmMinCMPB = EPWM1_MIN_CMPB;
}




/* // Setup TBCLK
EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;// Count up/down
EPwm3Regs.TBPRD = EPWM3_TIMER_TBPRD; // Set timer period
EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
EPwm3Regs.TBPHS.half.TBPHS = 0x0000; // Phase is 0
EPwm3Regs.TBCTR = 0x0000; // Clear counter
EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

// Setup shadow register load on ZERO
EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

// Set Compare values
EPwm3Regs.CMPA.half.CMPA = EPWM3_MIN_CMPA; // Set compare A value
EPwm3Regs.CMPB = EPWM3_MAX_CMPB; // Set Compare B value

// Set Actions
EPwm3Regs.AQCTLA.bit.PRD = AQ_SET; // Set PWM3A on period
EPwm3Regs.AQCTLA.bit.CBD = AQ_CLEAR; // Clear PWM3A on event B, down count

EPwm3Regs.AQCTLB.bit.PRD = AQ_CLEAR; // Clear PWM3A on period
EPwm3Regs.AQCTLB.bit.CAU = AQ_SET; // Set PWM3A on event A, up count

// Interrupt where we will change the Compare Values
EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO; // Select INT on Zero event
EPwm3Regs.ETSEL.bit.INTEN = 1; // Enable INT
EPwm3Regs.ETPS.bit.INTPRD = ET_3RD; // Generate INT on 3rd event

// Information this example uses to keep track
// of the direction the CMPA/CMPB values are
// moving, the min and max allowed values and
// a pointer to the correct ePWM registers
epwm3_info.EPwm_CMPA_Direction = EPWM_CMP_UP; // Start by increasing CMPA &
epwm3_info.EPwm_CMPB_Direction = EPWM_CMP_DOWN; // decreasing CMPB
epwm3_info.EPwmTimerIntCount = 0; // Zero the interrupt counter
epwm3_info.EPwmRegHandle = &EPwm3Regs; // Set the pointer to the ePWM module
epwm3_info.EPwmMaxCMPA = EPWM3_MAX_CMPA; // Setup min/max CMPA/CMPB values
epwm3_info.EPwmMinCMPA = EPWM3_MIN_CMPA;
epwm3_info.EPwmMaxCMPB = EPWM3_MAX_CMPB;
epwm3_info.EPwmMinCMPB = EPWM3_MIN_CMPB;
}
/*
void update_compare(EPWM_INFO *epwm_info)
{
// Every 10'th interrupt, change the CMPA/CMPB values
if(epwm_info->EPwmTimerIntCount == 10)
{
epwm_info->EPwmTimerIntCount = 0;

// If we were increasing CMPA, check to see if
// we reached the max value. If not, increase CMPA
// else, change directions and decrease CMPA
if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
{
if(epwm_info->EPwmRegHandle->CMPA.half.CMPA < epwm_info->EPwmMaxCMPA)
{
epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
}
else
{
epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
}
}

// If we were decreasing CMPA, check to see if
// we reached the min value. If not, decrease CMPA
// else, change directions and increase CMPA
else
{
if(epwm_info->EPwmRegHandle->CMPA.half.CMPA == epwm_info->EPwmMinCMPA)
{
epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
}
else
{
epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
}
}

// If we were increasing CMPB, check to see if
// we reached the max value. If not, increase CMPB
// else, change directions and decrease CMPB
if(epwm_info->EPwm_CMPB_Direction == EPWM_CMP_UP)
{
if(epwm_info->EPwmRegHandle->CMPB < epwm_info->EPwmMaxCMPB)
{
epwm_info->EPwmRegHandle->CMPB++;
}
else
{
epwm_info->EPwm_CMPB_Direction = EPWM_CMP_DOWN;
epwm_info->EPwmRegHandle->CMPB--;
}
}

// If we were decreasing CMPB, check to see if
// we reached the min value. If not, decrease CMPB
// else, change directions and increase CMPB

else
{
if(epwm_info->EPwmRegHandle->CMPB == epwm_info->EPwmMinCMPB)
{
epwm_info->EPwm_CMPB_Direction = EPWM_CMP_UP;
epwm_info->EPwmRegHandle->CMPB++;
}
else
{
epwm_info->EPwmRegHandle->CMPB--;
}
}
}
else
{
epwm_info->EPwmTimerIntCount++;
}

return;
}*/
