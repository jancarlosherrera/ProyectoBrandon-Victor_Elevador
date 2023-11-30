#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c1294ncpdt.h"
#include <string.h>

int t,piso,pres,movi,glob; //vol;
double data[2],ent, volt,temp,valor,va1;

void inter();


int main(void)
{
    SYSCTL_RCGCGPIO_R |= 0xF1F; //Relojes de puertos
    while((SYSCTL_PRGPIO_R & 0xF1F) == 0); // Se espera a que el reloj se estabilice (p.499)
    SYSCTL_RCGCADC_R = 0x01; // Habilita reloj para lógica de ADC0
    while((SYSCTL_PRADC_R&0x01)==0);
    GPIO_PORTC_AHB_DIR_R = 0x00;
    GPIO_PORTC_AHB_DEN_R = 0x70;
    GPIO_PORTC_AHB_PDR_R = 0x70;

    GPIO_PORTD_AHB_DIR_R = 0x07;
    GPIO_PORTD_AHB_DEN_R = 0x07;

    GPIO_PORTE_AHB_DIR_R = 0x00;    // 2) PE5-4 entradas (analógica)
    GPIO_PORTE_AHB_AFSEL_R |= 0x30; // 3) Habilita Función Alterna en
    GPIO_PORTE_AHB_DEN_R &= ~0x30;  // 4) Deshabilita Función Digital en PE4-5
    GPIO_PORTE_AHB_AMSEL_R |= 0x30; // 5) Habilita Función Analógica de PE4-5

    GPIO_PORTL_DIR_R = 0x00;
    GPIO_PORTL_DEN_R = 0x3F;
    GPIO_PORTL_PDR_R = 0x3F;



   GPIO_PORTB_AHB_DIR_R |= 0x00;
   GPIO_PORTB_AHB_DEN_R |= 0X30;
   NVIC_EN0_R = 0x02;
   GPIO_PORTB_AHB_IM_R |= 0b110000;
   GPIO_PORTB_AHB_IS_R = 0x0;
   GPIO_PORTB_AHB_IEV_R |= 0b110000;




    GPIO_PORTM_DIR_R = 0xC3; // PM1 entrada (para timer)
    GPIO_PORTM_AFSEL_R |= 0x03; // Habilita Función Alterna de PM1
    GPIO_PORTM_DEN_R = 0xC3; // Deshabilita Función Digital de PM1
    GPIO_PORTM_PCTL_R |= 0x00033;

    Servo_init();
    HC05_init();
    parar();
    ServoDos0();
    ServoDos90();



    Bluetooth_Write_String("JCHG TIVA C DEVELOPER\n");
    Bluetooth_Write_String("Iniciando sistema\n");
    while(1){

    int act;

        LM35();
        Bluetooth_Write_String(temp);
        if (temp<36){
        Bluetooth_Write_String("Temperatura normal\n");
        }else{
                        Bluetooth_Write_String("Temperatura ALTA\n");
            parar();
            ServoDos0();
            Delay_ms(5000);
        }


        pres = GPIO_PORTL_DATA_R;
        switch (pres){
        case 0x04:
            //PISO 2
            act=pos();
            if (act==2){
               Bluetooth_Write_String("Ya me encuentro en piso 2");
            }else if (act!=2){
                ServoDos90();
                subir(2);
                Bluetooth_Write_String("Piso 2");
                ServoDos0();
            }
             break;
        case 0x08:
            //piso1
            act=pos();
            if (act==2){
                ServoDos90();
                bajar(1);
                Bluetooth_Write_String("Piso 1");
                ServoDos0();
            }else if (act==0){
                ServoDos90();
                subir(1);
                Bluetooth_Write_String("Piso 1");
                ServoDos0();
            }else{
                Bluetooth_Write_String("Ya me encuentro en piso 1");
            }
            break;

        case 0x10:
            //planta baja
            act=pos();
            if (act==0){
               Bluetooth_Write_String("Ya me encuentro en planta baja");
            }else if (act!=0){
                ServoDos90();
                bajar(0);
                Bluetooth_Write_String("Planta Baja");
                ServoDos0();
            }

          break;

        case 0x00:
          Delay_ms(2000);
          Bluetooth_Write_String("Detenido");
          ServoDos0();
            break;
        }



    }
}








void inter(){

        if(((GPIO_PORTB_AHB_DATA_R &= 0x10)!=0))
        {
            UART4_DR_R=0;
        Bluetooth_Write_String("1era interrupcion");
        Delay_ms(1000);
        while(GPIO_PORTL_DATA_R != 0x20){
            GPIO_PORTD_AHB_DATA_R = 0X00;//CONDICIONAR D2 AL RELÉ
            ServoUn0();
            }  GPIO_PORTB_AHB_ICR_R |= 0X30;
        }
        if(((GPIO_PORTB_AHB_DATA_R &= 0x20)!=0))
        {
            Bluetooth_Write_String("\n");
            Bluetooth_Write_String("1era interrupcion");
            Delay_ms(1000);
            while(GPIO_PORTL_DATA_R != 0x20){
                GPIO_PORTD_AHB_DATA_R = 0X00;//CONDICIONAR D2 AL RELÉ
                Bluetooth_Write_String("2da interrupcion");
                //ServoUn0();
                                    }  GPIO_PORTB_AHB_ICR_R |= 0X30;
         }
        GPIO_PORTB_AHB_ICR_R |= 0X30;


}

int LM35(){
    ADC0_PC_R = 0x01; // Configura para 125Ks/s
    ADC0_SSPRI_R = 0x3021; // SS2 con la más alta prioridad
    ADC0_ACTSS_R = 0x0000; // Deshabilita SS2 antes de cambiar configuración de registros
    ADC0_EMUX_R = 0x0000; // Iniciar muestreo por software
    ADC0_SSEMUX2_R = 0x00; // Rango de entradas para bit EMUX0: AIN[15:0]
    ADC0_SSMUX2_R = 0X1890; // Para bit MUX0: Canal AIN9 => PE4, canal AIN0 => PE3 (tabla 15-1). Ultima muestra es PE2
    ADC0_SSCTL2_R = 0x6000; // Entrada externa, NO habilita INT., SI última muestra, NO diferencial
    ADC0_IM_R = 0x0000; // Deshabilita interrupciones de SS2
    ADC0_ACTSS_R |= 0x0004; // Habilita SS2
    Delay_ms(300);
      ADC0_PSSI_R = 0x0004;            //Inicia conversión del SS2
      while((ADC0_RIS_R&0x04)==0);     // Espera a que SS2 termine conversión (polling)
      data[0] = (ADC0_SSFIFO2_R&0xFFF);// se lee el primer resultado
      valor=data[0]; //PE4
      for(t=0 ; t<10000; t++);
      volt = ((valor*5)/2048); //Alimentando con 5
      temp=volt/0.1;
      ADC0_ISC_R = 0x0004;
      return temp;
}
int Servo_init(){

    //******* SINCRONIZACIÓN DE PLL PARA UTILIZAR PIOSC ******************
    SYSCTL_PLLFREQ0_R |= SYSCTL_PLLFREQ0_PLLPWR; // Enciende PLL
    while((SYSCTL_PLLSTAT_R&0x01)==0); // Espera a que el PLL fije su frecuencia
    SYSCTL_PLLFREQ0_R &= ~SYSCTL_PLLFREQ0_PLLPWR; // Apaga PLL
    SYSCTL_RCGCTIMER_R=0X04;
    //SysCtlDelay(10);
    TIMER2_CTL_R=0X0000;  //HABILITA TIMER B
    TIMER2_CFG_R=0X04;
    TIMER2_TBMR_R=0XA; //timer B
    TIMER2_TAMR_R=0XA; //timer A
    //Seleccionar preescala
    TIMER2_TBPR_R=0x4;
    TIMER2_TBILR_R=0XE200;
    TIMER2_TBPMR_R=0X4;
    TIMER2_TBMATCHR_R=0X6500;

    TIMER2_TAPR_R=0x4; //Periodo de la señal
    TIMER2_TAILR_R=0XE200; //Periodo de la señal
    TIMER2_TAPMR_R=0X4; //Ciclo de trabajo
    TIMER2_TAMATCHR_R=0X6500;
    TIMER2_CTL_R=0x0101;//Habilita timer B
}

int ServoUn90(){
    TIMER2_TBPMR_R=0X4;
    TIMER2_TBMATCHR_R=0X6500;
    //PM1
}

int ServoUn0(){

    TIMER2_TBPMR_R=0X4;
    TIMER2_TBMATCHR_R=0XB000;
}

int ServoDos90(){
    TIMER2_TAPMR_R=0X4;
    TIMER2_TAMATCHR_R=0X6500;
    //PM0
}

int ServoDos0(){
    TIMER2_TAPMR_R=0X4;
    TIMER2_TAMATCHR_R=0XB000;

}

void Delay_ms(int time_ms)
  {
      int i, j;
      for(i = 0 ; i < time_ms; i++)
          for(j = 0; j < 3180; j++)
              {}  /* excute NOP for 1ms */
  }


void subir(int hasta){
    movi=pos();
    while(hasta!=movi){
    GPIO_PORTD_AHB_DATA_R = 0x01; //editar para relé
    movi=pos();
    }
    parar();
}
void bajar(int hasta){
    movi=pos();
    while(hasta!=movi){
        movi=pos();
    GPIO_PORTD_AHB_DATA_R = 0x02; //editar para relé
}
parar();
}
void parar(){
    GPIO_PORTD_AHB_DATA_R = 0x00; //editar para relé
}

int pos(){
    int limit;
    limit = GPIO_PORTC_AHB_DATA_R;
    switch (limit){
    case 0x1D:
        piso=2;
        return piso;
        break;
    case 0x2D:
        piso=1;
        return piso;
        break;
    case 0x4D:
        piso=0;
        return piso;
        break;
    }

}

void HC05_init(void)
{


    SYSCTL_RCGCUART_R |= 0x10;  /* enable clock to UART4 */
   SYSCTL_RCGCGPIO_R |= 0x200;  /* enable clock to PORTK for PK0/Rx and RK1/Tx */
    Delay(1);
    /* UART0 initialization */
            UART4_CTL_R = 0;         /* UART4 module disable */
           UART4_IBRD_R = 104;      /* for 9600 baud rate, integer = 104 */
           UART4_FBRD_R = 11;       /* for 9600 baud rate, fractional = 11*/
           UART4_CC_R = 0;          /*select system clock*/
           UART4_LCRH_R = 0x60;     /* data lenght 8-bit, not parity bit, no FIFO */
           UART4_CTL_R = 0x301;     /* Enable UART4 module, Rx and Tx */
          // UART4_DR_R = 0;
           /* UART4 TX4 and RX4 use PK0 and PK1. Configure them digital and enable alternate function */
                   GPIO_PORTK_DEN_R    = 0x03;      /* set PK0 and PK1 as digital */
                   GPIO_PORTK_AFSEL_R = 0x03;    /* Use PK0,PK1 alternate function */
                   GPIO_PORTK_AMSEL_R  = 0;    /* Turn off analg function*/
                   GPIO_PORTK_PCTL_R = 0x00000011;     /* configure PK0 and PK1 for UART */
}

char Bluetooth_Read(void)
{
    char data;
    while((UART4_FR_R & (1<<4)) != 0); /* wait until Rx buffer is not full */

    data = UART4_DR_R ;      /* before giving it another byte */
    return (unsigned char) data;
}

void Bluetooth_Write(unsigned char data)
{
    while((UART4_FR_R & (1<<5))!= 0); /* wait until Tx buffer not full */
    UART4_DR_R = data;                  /* before giving it another byte */
}

void Bluetooth_Write_String(char *str)
{
  while(*str)
    {
        Bluetooth_Write(*(str++));
    }
}

void Delay(unsigned long counter)
{
    unsigned long i = 0;

    for(i=0; i< counter; i++);
}
