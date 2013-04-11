/*
 * main_wheelie.c
 *
 *  Created on: Jan 24, 2013
 *      Author: oriol
 */

/*************************************************
*   _ _ _ _           _ _
*  | | | | |_ ___ ___| |_|___
*  | | | |   | -_| -_| | | -_|
*  |_____|_|_|___|___|_|_|___|
*
**************************************************/

#ifndef F_CPU
#define F_CPU 16000000
#endif

#ifndef NULL
#define NULL 0
#endif

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "ars.h"
#include "driverlib/rom.h"

//ADC Inputs
#define Ad_kanal_batt   1
#define Ad_kanal_zgyro  2
#define Ad_kanal_adxl   3
#define Ad_kanal_gyro   0
#define Ad_kanal_fuss   7
#define Ad_kanal_rocker 6
#define Ad_kanal_roll_adxl  4
#define Ad_kanal_roll_gyro  5

//maximum Power, limit to 20 for first Tests, then you don't destroy your furniture
#define max_PWM         550
//limit the I-Part of the PID-Control
#define Drivesumlimit   39000
//Calculate Wheel_const = circumference of the wheel in cm * 93,75 (f.e. 126 cm * 93,75 = 11812,5) for correct speed
#define Wheel_const     11812

//Status of the Wheelie
#define m_standby               1
#define m_run                   2
#define m_down                  3 //power down the wheels

//footswitch is pressed under this level
#define Sw_down                 50

int drive_r = 0;
int drive_l = 0;

int left = 0;
int right = 0;


signed long     Anglecorrection=0;
float                   Angle_Rate=0;
float                   Roll_Angle_Rate=0;
float                   Balance_moment=0;
float               Drive_sum=0;
int                 Drivespeed=0;

int Rockersq=0;
int Rockerold=0;
int Rocker_zero=0;
int Rocker_diff=0;

int Speed_sum=0;
int Rocker_sensivity=0;
int Rocker_position=0;
int Speed_error=0;
int Speed_diff=0;
int Speed_correction=0;
int Steeringsignal=0;

//the I²C-Values are stored in this Array (Speed, Current L and R)
unsigned short Speed_array[8]={ 0,0,0,0,0,0,0,0} ;
unsigned int* Speed_array16 = (unsigned int*)Speed_array;
#define Speed_left Speed_array16[0]
#define Speed_right Speed_array16[1]
#define Curr_left Speed_array16[2]
#define Curr_right Speed_array16[3]


// macro for sending one char over UART
#define uart_putch(ch) ROM_UARTCharPut(UART0_BASE, ch)



int Speed_left_out;
int Speed_right_out;
unsigned short cw_r;
unsigned short cw_l;

//This 6 values can controlled from the displayunit to improove PID and steering
int  Parameter1=15;             //Steering depends on speed                             higher Value, lower Steeringsignal
unsigned int P1 = 0x10;
int  Parameter2=27;             //Speederror from the Geartooths                higher Value, lower Correction
unsigned int P2 = 0x20;
int  Parameter3=210;
unsigned int P3 = 0x30;
int  Parameter4=50;    //Gain P-Part
unsigned int P4 = 0x40;
int  Parameter5=215;    //Gain I-Part
unsigned int P5 = 0x50;
int  Parameter6=155;   //Gain D-Part                                                    higher Value, lower Gain
unsigned int P6 = 0x60;

unsigned short Mmode = m_standby;
unsigned short Errno = 0;
unsigned short Zaehler = 0;
unsigned int spl = 0;

long Ad_gyro = 0;
long Ad_adxl = 0;
long Ad_batt = 0;
long Ad_adxl_roll = 0;
long Ad_gyro_roll = 0;
long Ad_rocker = 0;
long Ad_z = 0;
long Z_zero = 0;
int Z_diff = 0;
int Z_error = 0;

long Adxl_zero = 0;
long Roll_adxl_zero = 0;
long Gyro_zero = 0;
long Roll_gyro_zero = 0;
long Ad_swi = 0;

char Text[40];
char Inputstring[40];
unsigned short StringCounter = 0;
float roll_adxl_sensor = 0;
float adxl_sensor = 0;
float tilt_angle = 0.0;
float roll_angle = 0.0;

char s[20];
char t[20];
float dt;
struct Gyro1DKalman filter;
struct Gyro1DKalman filterX;

int x_error;

unsigned long ulPeriod;
unsigned long ulPeriodPWM;
unsigned long ulADC0Value[8];
unsigned long dutyCycle;

void init(void);
void startup(void);
void set_pwm(void);
//void uart_puts (const char *s);
int  main(void);
void algo(void);
void err_value(void);
void steering(void);
void process(void);
void gear(void);
void parser(void);
void saveData(void);
void initADC(void);
void fireAdc(void);
unsigned long getAdc(int);
void init_PWM(void);
extern void Timer0IntHandler(void);
void sendData(void);
void uart_puts(char *);
void getCommand(void);


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//initializes the adc
void initADC(void){
   	//enable the adc0 peripherial.
   	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
   	SysCtlDelay(100);
   	//set the speed to 1msps.
   	//SysCtlADCSpeedSet(SYSCTL_ADCSPEED_1MSPS);
   	//set the auto avergage to 64.
   	//ADCHardwareOversampleConfigure(ADC0_BASE, 64);
   	//before setting up I must disable the sequence 0.
   	ADCSequenceDisable(ADC0_BASE, 0);
   	SysCtlDelay(100);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	//
	// Select the analog ADC function for these pins.
	// Consult the data sheet to see which functions are allocated per pin.
	// TODO: change this to select the port/pin you are using.
	//
	//GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);


   	//set the sequence to use (adc0 sequence 0).
   	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
   	//set up the sequence step.
   	//set up the last step and start an interrupt when the conversion it's over.
   	//TODO s'han de configurar tots els steps
   	//ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
   	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH2 );
   	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1 );
   	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH0 );
   	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH6 );
   	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH5 );
   	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH4 | ADC_CTL_IE |
   	   									ADC_CTL_END);
   	//ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH4 );
   	//ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH4 );
   	//ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH4 | ADC_CTL_IE |
   								//	ADC_CTL_END);

   	//enable the sequence again!
   	ADCSequenceEnable(ADC0_BASE, 0);
   	SysCtlDelay(100);
}


//llegirem els canals ADC i el posarem al array
void fireAdc(void){

	//Primer agafarem tots els valors analògics a través de la Sequence0 (8 canals)
		//clear the interrupt flag
	ADCIntClear(ADC0_BASE, 0);
	//trigger the adc conversion process.
	ADCProcessorTrigger(ADC0_BASE, 0);
	//wait for the interrupt flag to get set!
	while(!ADCIntStatus(ADC0_BASE, 0, false))
	{
	}
	//get the actual data samples from adc0 sequencer 3!
	ADCSequenceDataGet(ADC0_BASE, 0, ulADC0Value);
	//convert the value!
	//ulTemp_ValueC = ((1475 * 1023) - (2250 * ulADC0Value[0])) / 10230;;
	//UARTprintf("Temperature = %3d*C \n\r", ulTemp_ValueC);
	//SysCtlDelay(SysCtlClockGet() / 12);
}



//Retorna una dada recuperada per l'ADC
unsigned long getAdc(int channel)
{

	//ADC stuff!
	return ulADC0Value[channel];

}

// Inicialització dels TIMERS per a PWM
void init_PWM(void)
{
	// Left Motor
	// PWM1 -> PF4 DIRECTION BIT
	// PWM2 -> PF3 T1CCP1
	//
	// Right Motor
	// PWM3 -> PB3 DIRECTION BIT
	// PWM4 -> PB4 T1CCP0


	ulPeriodPWM = 1000;
	dutyCycle = 250;

	 // Turn off LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);

	//SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//GPIOPinConfigure(GPIO_PF4_T2CCP0);
	GPIOPinConfigure(GPIO_PF3_T1CCP1);
	GPIOPinTypeTimer(GPIO_PORTF_BASE, GPIO_PIN_3);

	// Configure PB4 T1CCP0 and PB3 as Direction bit
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_3);
	//GPIOPinConfigure(GPIO_PB3_T3CCP1);
	GPIOPinConfigure(GPIO_PB4_T1CCP0);
	GPIOPinTypeTimer(GPIO_PORTB_BASE,GPIO_PIN_4);

	// Configure timer 1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PWM|TIMER_CFG_B_PWM);
	TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriodPWM -1);
	//TimerMatchSet(TIMER1_BASE, TIMER_A, dutyCycle); // PWM
	TimerEnable(TIMER1_BASE, TIMER_A);

	TimerLoadSet(TIMER1_BASE, TIMER_B, ulPeriodPWM -1);
	//TimerMatchSet(TIMER1_BASE, TIMER_A, dutyCycle); // PWM
	TimerEnable(TIMER1_BASE, TIMER_B);

}




/******************************************************************
*    Init:
*
*       set the Registers
******************************************************************/
void init(void)
{


		ROM_FPULazyStackingEnable();

        //inicialitzarem el Clolck del system a uns 40 mhz i despres inicialitzarem el Timer0 que sera el que controli tot el procÃ©s

		// 40 MHz system clock
		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL| SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);


		//
		// Initialize the UART.
		//
		ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
		ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
		ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		//ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

		UARTStdioInit(0);

		//unsigned long ulFrequency = 60000; //Used to set Clock Period
		//unsigned long ulFrequency = 400000; // Amb aquest valor tindrem una frequència de 100Hz de interrupció
											// No fem més frequència pq el gyro té un Bandwidht de 140 Hz

		unsigned long ulFrequency = 100;// Amb aquest valor tindrem una frequència de 100Hz de interrupció
		// No fem més frequència pq el gyro té un Bandwidht de 140 Hz
		//timer 0 set-up
		SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
		TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
		// Delay Calculations -- set collection period via ulFrequency -- see notes at beginning.
		ulPeriod = (SysCtlClockGet() / ulFrequency)/2; //100Hz freq.
		TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod-1);

	 // Freeze the timer counting if we are debugging (the counting is enabled automatically with the cpu).
		//TimerControlStall(TIMER0_BASE, TIMER_A, true);

		IntEnable(INT_TIMER0A);
		TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		IntMasterEnable();


		//TimerEnable(TIMER0_BASE, TIMER_A);

        init_PWM();

        initADC();
        int i = 0;
        for(i = 0;i<10;i++)
        {
        		fireAdc();
                Ad_swi += getAdc(Ad_kanal_fuss);
                SysCtlDelay(1000);
        }
        Ad_swi /= 10;
        if (Ad_swi < 100)
        {
                Errno = 4;
                //UARTprintf ("2:11=14\n");

        }
        if (Parameter1 == -1)        //no Data in EEPROM
        {
        	Parameter1 = 14;        // write the Standardvalues to EEPROM
        	saveData();
        }else{
       //    eeprom_read_block(&Parameter2,(unsigned short *)P2,sizeof(int));
       //    eeprom_read_block(&Parameter3,(unsigned short *)P3,sizeof(int));
       //    eeprom_read_block(&Parameter4,(unsigned short *)P4,sizeof(int));
       //    eeprom_read_block(&Parameter5,(unsigned short *)P5,sizeof(int));
       //    eeprom_read_block(&Parameter6,(unsigned short *)P6,sizeof(int));
        }
}
/*********************************************************************
*    startup:
*
*       calculate the zeropoints from the ADCs
*********************************************************************/
void startup(void)
{
   int i = 0;
   /*
   Ad_rocker = 0;
   Rocker_zero = 0;
   for (i = 0; i<10; i++)
   {
      Rocker_zero += ReadChannel(Ad_kanal_rocker);
      SysCtlDelay(3);
   }
   Rocker_zero /= 10;
   */
   Rocker_zero = 434; //the rocker is exact vertical to ground

   Ad_gyro = 0;
   Gyro_zero = 0;
   for (i = 0;i < 50;i++)
   {
	  fireAdc();
      Gyro_zero += getAdc(Ad_kanal_gyro);
      SysCtlDelay(6000);
   }
   Gyro_zero = Gyro_zero / i;

   Ad_adxl = 0;
   Adxl_zero = 0;
   for (i = 0;i <= 100;i++)
   {
	  fireAdc();
      Adxl_zero += getAdc(Ad_kanal_adxl);
      SysCtlDelay(300);
   }
   Adxl_zero /= i;

   Ad_z = 0;
   Z_zero = 0;
   for (i = 0;i <= 100;i++)
   {
	  fireAdc();
      //Z_zero += 1024 - getAdc(Ad_kanal_zgyro);
	  Z_zero +=getAdc(Ad_kanal_zgyro);
      SysCtlDelay(300);
   }
   Z_zero /= i;


   Ad_gyro_roll = 0;
   Roll_gyro_zero = 0;
   for (i = 0;i < 50;i++)
   {
	   fireAdc();
      //Roll_gyro_zero += (1024 - ReadChannel(Ad_kanal_roll_gyro));
	  Roll_gyro_zero += getAdc(Ad_kanal_roll_gyro);
      SysCtlDelay(6000);
   }
   Roll_gyro_zero /=  i;

   //Roll_gyro_zero = 505; //the platform is exact horizontal to ground

   Ad_adxl_roll = 0;
   Roll_adxl_zero = 0;
   for (i = 0;i < 100;i++)
   {
	   fireAdc();
      Roll_adxl_zero += getAdc(Ad_kanal_roll_adxl);
      SysCtlDelay(300);
   }
   Roll_adxl_zero /= i;

   //Roll_adxl_zero = 513; //the platform is exact horizontal to ground
}
/************************************************************************
*    main:
*
*       Mainloop
************************************************************************/
int main(void)

{

	int buffer = 0;

        init();
        startup();

        //TODO Mirar com puc fer los dels 15V



        //UARTprintf("This is Wheelie!!\n");

		init_Gyro1DKalman(&filter, 0.0001, 0.0003, 0.3);
        init_Gyro1DKalman(&filterX, 0.0001, 0.0003, 0.3);
        //dt = 0.0157;   // time passed in s since last call
        dt = 0.01;   // time passed in s since last call

        while(1)
        {

          process();
          //UARTprintf("Test");
          //UARTprintf("Test %d\n",buffer);
          //buffer++;
          SysCtlDelay(SysCtlClockGet() / 1000 / 2);
          //sendData();
        }
}
/***************************************************************
*    ISR:
*
*       Interrupt all 10ms
***************************************************************/
void Timer0IntHandler(void)
{
		// Clear the timer interrupt
		TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

		fireAdc();
		//get all values from the ADCs
		Ad_gyro = getAdc(Ad_kanal_gyro);
		Ad_adxl = getAdc(Ad_kanal_adxl);
		//Ad_z = 1024 - getAdc(Ad_kanal_zgyro);
		Ad_z = getAdc(Ad_kanal_zgyro);
		Ad_adxl_roll = getAdc(Ad_kanal_roll_adxl);
		//Ad_gyro_roll = 1024 - getAdc(Ad_kanal_roll_gyro);
		Ad_gyro_roll = getAdc(Ad_kanal_roll_gyro);
		Ad_rocker = getAdc(Ad_kanal_rocker);
		Ad_swi = 4094 - getAdc(Ad_kanal_fuss);
		Ad_batt = getAdc(Ad_kanal_batt);

        //kill the offset and calculate the Gains, so that all values are ° or °/s
        // 3.0V/1025/0.002V for IDG300
		// 3.0V/4095/3.3mV for LPR530AL = 0.22200022
		// 3.0V/4095/0.83mV for LPR530AL sense amplificar = 0.8826514
        //Angle_Rate = (float)(Ad_gyro-Gyro_zero)* 1.4634135;
		//Angle_Rate = (float)(Ad_gyro-Gyro_zero)* 0.22200022;
		Angle_Rate = (float)(Ad_gyro-Gyro_zero)* 0.8826514;
        //Roll_Angle_Rate = (float)(Ad_gyro_roll-Roll_gyro_zero)* 1.4634135;
		Roll_Angle_Rate = (float)(Ad_gyro_roll-Roll_gyro_zero)* 0.8826514;

        // 3.0V/1025/0.0019333V         0,348V for 180° = 1,9333mV per ° for ADXL320
        // 3.0V/1025/0.0046666V         0,840V for 180° = 4,6666mV per ° for ADXL322
	    //adxl_sensor = ((float)(Ad_adxl-Adxl_zero))* 1.5139;
        //roll_adxl_sensor = ((float)(Ad_adxl_roll-Roll_adxl_zero))* 1.5139;

		// 3.0V/4095/0.003V   for ADXL335
		adxl_sensor = ((float)(Ad_adxl-Adxl_zero))* 0.244200244;
        roll_adxl_sensor = ((float)(Ad_adxl_roll-Roll_adxl_zero))* 0.244200244;


        //the two Kalman steps: predict and update
        ars_predict(&filter, Angle_Rate, dt);    // Kalman predict    *0.62719;
        ars_predict(&filterX, Roll_Angle_Rate, dt);    // Kalman predict    *0.62719;
        tilt_angle = ars_update(&filter, adxl_sensor);
        roll_angle = ars_update(&filterX, roll_adxl_sensor) *  2.6; //Anpassung ans Lenkerpoti

        //Z-Axis
        //Z_diff = Ad_z - Z_zero ;
        //Z_error = Ad_z - Z_zero ;

        algo();
        //gear();
        steering();
        set_pwm();
        parser();
        sendData();
}

/**************************************************************************
*    algo:
*
*       PID-Controller
***************************************************************************/
void algo(void)
{
    Balance_moment =  Angle_Rate * Parameter5 / 100  + tilt_angle;                        //Angle_Rate is D-Part, tilt_angle is P-Part of the PID-Controller
    Balance_moment = Balance_moment * 29;
        //The I-Part of the PID-Controller
        Drive_sum += Balance_moment;
    if(Drive_sum >  Drivesumlimit)Drive_sum = Drivesumlimit;    //you have to limit the I-Controller
    if(Drive_sum <- Drivesumlimit)Drive_sum =-Drivesumlimit;    //Drive_sum max = 255*150

    Drivespeed = Drive_sum / Parameter6 + Balance_moment / 165;
}
/**************************************************************************
*    Gear:
*
*       get Speed from the Geartoothsensors
**************************************************************************/
void gear(void)
{
        unsigned short i = 0;
   // i2c_start_wait(0x41);
        for (i=0;i<7;i++)                                               // get the 8 Bytes from the I2C-Slave
        {
           //     Speed_array[i] = i2c_readAck();
    }
        //Speed_array[i] = i2c_readNak();
       // i2c_stop();
        //calculate Speed*10
        Speed_left = Wheel_const / Speed_left;
    Speed_right = Wheel_const / Speed_right;

        //so eine Kacke! Ich muß unbedingt die Drehrichtung der Räder messen!!
    if (Speed_left < 250) //value over 25km/h must be wrong, so ignore it
        {
      if ( cw_l == 1 ) Speed_left_out = Speed_left * -1;
      else Speed_left_out = Speed_left;
    }
    if (Speed_right < 250)
        {
      if ( cw_r == 1 ) Speed_right_out = Speed_right * -1;
      else Speed_right_out = Speed_right;
    }
    Speed_sum -= Speed_sum / 5;
    Speed_sum += Speed_right_out + Speed_left_out;

    Speed_diff -= Speed_diff / 5;
    Speed_diff += Speed_left_out - Speed_right_out;

    Speed_correction = Speed_diff / Parameter2;
 }
/****************************************************************************
*    Steering:
*
*       calculate the steering
****************************************************************************/
void steering(void)
{
   int Buf1;
   //Rockersq = Rocker_zero - Ad_rocker + (int)roll_angle;
   x_error = Rockersq;
   Buf1 = abs(Speed_sum / 70);
   if (Buf1 > 20) Buf1 = 20;
   Rocker_sensivity = 22 - Buf1;
   Rocker_sensivity = Rocker_sensivity / 2;
   if (Rockersq > 45) Rockersq = 45;
   if (Rockersq < -45) Rockersq = -45;
   Rocker_position = Rockersq;
   Rockersq = Rockersq * Rocker_sensivity;
   Rockersq = Rockersq / Parameter1;
   Speed_error = Rockersq - Speed_correction;
   if (Speed_error > 40) Speed_error = 40;
   if (Speed_error < -40) Speed_error = -40;
   Steeringsignal = Rockersq + Speed_error + Z_error;
   drive_r = Drivespeed - Steeringsignal;
   drive_l = Drivespeed + Steeringsignal;

}
/********************************************************************************
*    Process:
*
*       get and set the Status
********************************************************************************/
void process(void)
{
   int i;
   switch(Mmode)
   {
           case m_standby:
                  drive_l = 0;
              drive_r = 0;
                  Drivespeed = 0;
              Drive_sum = 0;
                  set_pwm();
                  fireAdc();
                  Ad_swi = 1024 - getAdc(Ad_kanal_fuss);
                  Ad_batt = getAdc(Ad_kanal_batt);


              if (Ad_swi > Sw_down){
                    //startup();
         			TimerEnable(TIMER0_BASE, TIMER_A); //comença el timer d'execució

					Mmode = m_run;
	          }
              break;
   case m_run:
      if (Ad_swi < Sw_down) {Mmode = m_down;}
          break;
   case m_down:

	     TimerDisable(TIMER0_BASE, TIMER_A);  //Aturem el timer
         for(i = 0; i < 255; i++)
                 {
            if (drive_l == 0 && drive_r == 0) {goto Shutdown;}
            if (drive_l > 0) { drive_l--;}
            if (drive_l < 0) { drive_l++;}
            if (drive_r > 0) { drive_r--;}
            if (drive_r < 0) { drive_r++;}
            SysCtlDelay(25);
            set_pwm();
         }
         Shutdown:
                 Mmode = m_standby;
                 break;
   default:
      Mmode = m_down;
   }
}
/**********************************************************************************
*    set_pwm:
*
*       set the PWMs and the Directionbits
***********************************************************************************/
void set_pwm(void)
{

	//drive_l = 0;
	//drive_r = 0;

	if(drive_l >  max_PWM)drive_l =  max_PWM;
	if(drive_l < -max_PWM)drive_l = -max_PWM;
	if(drive_r >  max_PWM)drive_r =  max_PWM;
	if(drive_r < -max_PWM)drive_r = -max_PWM;

	right = abs(drive_r);

	left = abs(drive_l);

    if(drive_r < 0)                                 // Check direction
    {
    	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,1);   //HEM DE POSAR A 1 UN BIT
        cw_r = 1;
    }
    else
    {
    	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0);//HEM DE POSAR A 0 UN BIT
        cw_r = 0;
    }

    if(drive_l < 0)                                 // Check direction
    {
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4,1);//HEM DE POSAR A 1 UN BIT
        cw_l = 1;
    }
    else
    {
    	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4,0);   //HEM DE POSAR A 1 UN BIT
        cw_l = 0;
    }

//        OCR1AL = 255 - r;
//        OCR1BL = l;
    //PWM d'un motor ENCARA FALTA ALTRE
    //TimerMatchSet(TIMER1_BASE, TIMER_B, ulPeriodPWM-right);
    TimerMatchSet(TIMER1_BASE, TIMER_B, right);
    TimerMatchSet(TIMER1_BASE, TIMER_A, left);

}
/********************************************************************************
*    Err_value:
*
*       throw the program in an black whole
********************************************************************************/
void err_value(void)
{
   while(1) { SysCtlDelay(2000);}
}
/********************************************************************************
*    Parser:
*
*       get Commands from USART
********************************************************************************/
void parser(void)
{

char *ptr;
char delimiter[] = ":=";
unsigned short Channel;
int Wert;
int NextChar;

  NextChar = uart_getc_nowait();
  while(NextChar != 10 && NextChar !=-1 && StringCounter < 40)
  {
    if (NextChar != '\0' && NextChar != 13) {Inputstring[StringCounter++] = (unsigned char)NextChar;}
    NextChar = uart_getc_nowait();
  }
  if (NextChar == 10)
  {
        if (Inputstring[1] != 58 || StringCounter > 39) {goto Ausgang;}
        Inputstring[StringCounter++] = '\0';
    //ptr = strtok(Inputstring, delimiter);

    if(ptr != NULL)
        {
            if (atoi(ptr)==0)
                {
      //                  ptr = strtok(NULL, delimiter);
                    Channel = atoi(ptr);
      //              ptr = strtok(NULL, delimiter);
                    Wert = atoi(ptr);
                        if (Channel == 17) Parameter1 = Wert;
                        if (Channel == 18) Parameter2 = Wert;
                        if (Channel == 19) Parameter3 = Wert;
                        if (Channel == 20) Parameter4 = Wert;
                        if (Channel == 21) Parameter5 = Wert;
                        if (Channel == 22) Parameter6 = Wert;
                        if (Channel == 16)
                        {
                                saveData();
        //                        sprintf( Text, "2:11=STORED!\r\n");
                        UARTprintf (Text);
                                SysCtlDelay(500);
                                Zaehler =0;
                        }
                }
        }
        Ausgang:
        Inputstring[0] = '\0';
        StringCounter = 0;
  }
}

/***************************************************************************************
 * Sending data to serial port
 */
void sendData(void){

		  //UARTprintf(":");
	      UARTprintf("2:4=%d\n",(int)tilt_angle);
	      UARTprintf("2:5=%d\n",(int)roll_angle);
	      //UARTprintf("%d\n",Rockersq);
	      //UARTprintf("%d\n",Rocker_sensivity);
	      //UARTprintf("%d\n",Z_diff);
	      //UARTprintf("%d\n",Steeringsignal);
	      UARTprintf("2:7=%d\n",drive_l);
	      UARTprintf("2:8=%d\n",drive_r);
	      SysCtlDelay(1000);
	      //UARTprintf("!\n");
	      UARTprintf("2:9=%d\n",Speed_left_out);
	      UARTprintf("2:10=%d\n",Speed_right_out);
	      //UARTprintf("%d\n",Mmode);
	      //UARTprintf("%d\n",Ad_batt);
	      //UARTprintf("2:7%d\n",Curr_left);
	      //UARTprintf("2:8%d\n",Curr_right);
	      SysCtlDelay(1000);
	      //UARTprintf("?");
	      //UARTprintf("%d\n",Parameter1);
	      //UARTprintf("%d\n",Parameter2);
	      //UARTprintf("%d\n",Parameter3);
	      //UARTprintf("%d\n",Parameter4);
	      //UARTprintf("%d\n",Parameter5);
	      //UARTprintf("%d\n",Parameter6);



}


// send nullterminated string over UART
void uart_puts(char * s) {
    while (*s != 0) {
        uart_putch(*s);
        s++;
    }
}





/***************************************************************************************
*   saveData:
*
*       store all Parameters in EEPROM
***************************************************************************************/
void saveData(void)
{
        //        eeprom_write_block(&Parameter1,(unsigned short *)P1,sizeof(int));
        //        eeprom_write_block(&Parameter2,(unsigned short *)P2,sizeof(int));
        //        eeprom_write_block(&Parameter3,(unsigned short *)P3,sizeof(int));
        //        eeprom_write_block(&Parameter4,(unsigned short *)P4,sizeof(int));
        //        eeprom_write_block(&Parameter5,(unsigned short *)P5,sizeof(int));
        //        eeprom_write_block(&Parameter6,(unsigned short *)P6,sizeof(int));
}
