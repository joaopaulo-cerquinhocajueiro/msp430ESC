/**********************************************************
 * MAPEAMENTO MSP430G2553
 *
 * P1.0 - N�vel Bateria_1
 * P1.1 - canal avan�o/recuo
 * P1.2 - canal direita/esquerda
 * P1.4 - led vermelho
 * P1.5 - led verde
 * P1.7 - N�vel Bateria_2
 * P2.0 - led azul
 * P2.1 - motor esquerdo recuo
 * P2.2 - motor esquerdo avanco
 * P2.4 - motor direito recuo
 * P2.5 - motor direito avanco

 *                           MSP430G2553
 *                         ---------------
 *                    VDD--| 1        20 |--GND
 *            P1.0  NBAT1--| 2        19 |--        P2.6
 *            P1.1  IN_AR--| 3        18 |--        P2.7
 *            P1.2  IN_DE--| 4        17 |--TEST
 *            P1.3       --| 5        16 |--RST
 *            P1.4  LED_R--| 6        15 |--NBAT2   P1.7
 *            P1.5  LED_G--| 7        14 |--        P1.6
 *            P2.0  LED_B--| 8        13 |--MDA     P2.5
 *            P2.1    MER--| 9        12 |--MDR     P2.4
 *            P2.2    MEA--| 10       11 |--        P2.3
 *                         ---------------
 **********************************************************/




#include <msp430.h> 
#include <msp430g2553.h>

// Ports e bits dos leds
// Port 1
#define LED_R BIT4
#define LED_G BIT5
// Port 2
#define LED_B BIT0

// Bits dos motores
#define MDR BIT4
#define MDA BIT5
#define MER BIT1
#define MEA BIT2

// Bits dos sinais de entrada
#define IN_AR BIT1
#define IN_DE BIT2

//Bits dos niveis de bateria
#define NBAT_1 BIT0
#define NBAT_2 BIT7

#define MAX_COMM_DELAY 120000 // em microsegundos/4 - 30 ms
// definindo o SMCLK em 4MHz, o LOOP_LENGTH causar� um loop a 40 kHz.
#define LOOP_LENGTH 100


// O n�mero de bits do PWM vai influenciar na resolu��o do sinal PWM.
#define BITS_PWM 8
// Com 8 bits, temos uma resolu��o de 256 passos. A frequ�ncia do PWM fica 4 MHz/(2*256) = 7,8 kHz  OBS.: o numero de passos � multipliado por 2 devido ao modo Up/Down
#define PWM_MAX ((1<<BITS_PWM)-1)

unsigned int inputChannel0Status, inputChannel1Status;
unsigned int inputChannel0Value,inputChannel1Value;
unsigned int inputChannel0Start,inputChannel1Start;

// Valores m�nimos e m�ximos do tempo do canal
unsigned int channel0Min = 1080; // 1000 micro segundos.
unsigned int channel0Max = 1920; // 2000 micro segundos.
unsigned int channel0DeltaT = 526;
unsigned int channel1Min = 1000; // 1000 micro segundos.
unsigned int channel1Max = 1920; // 2000 micro segundos.
unsigned int channel1DeltaT = 526;

long int intermediate;


int chARval, chDEval; // valor dos canais
int MDval, MEval; // Valor dos motores
unsigned char pwmAd, pwmRd, pwmAe, pwmRe; // sinais dos PWMs
unsigned char pwmCounter = 0;
int wtdgHappened = 0; // flag para indicar quando resetou por watchdog

void setupClock(){
    // Ajusta o DCO para gerar 16MHz
    DCOCTL = 0;              // Select lowest DCOx and MODx settings
    // Ajusta para aproximadamente 16MHz (RSELx = 15, DCOx = 4, MODx = 0)
    BCSCTL1 = XT2OFF | 0x0F; // Set range para 15, mantendo XT2 desligado
    DCOCTL = DCO2;            // DCOx=4, MODx=0
    // Definimos o MCLK como sendo o DCO (SELMx=00) sem dividir (DIVMx=00) e o SMCLK como sendo o DCO (SELS=0) dividido por 4 (DIVSx = 10) --> SMCLK = 4MHz;
    BCSCTL2 = SELM0 | DIVM_0 | DIVS_2;
}

void setupTimer_A1(){
    TA1CCR0 = 255; //Define o valor 255 (8 Bits) para CCR0 ---> Frequencia do PWM = 4 MHz/(255*2) ~= 7,8 kHz
    TA1CCR1 = 0; // PWM 0 em 0
    TA1CCR2 = 0; // PWM 1 em 0
    TA1CTL = TASSEL_2 | MC_2; // Define a frequ�ncia do Timer A1 igual a SMCLK e configura para o modo Up/Down;
    TA1CCTL1 = OUTMOD_7; // Define o modo Reset/Set para CCR1
    TA1CCTL2 = OUTMOD_7; // Define o modo Reset/Set para CCR2

}


void setupCapture(){
    // Timer0_A
    TACTL = TASSEL_2 // Escolhe o SMCLK (4MHz) como entrada
          | ID_2     // dividido por 4 (1MHz) - d� um per�odo total de 65,536 ms
          | MC_2;    // Continuous mode
          //| TAIE;   // com interrup��es habilitadas
    // Registrador de captura 0
    TACCTL0 = CAP   // Modo de captura
            | CM1   // Captura na subida (vai ser mudado a cada interrup��o)
            | CCIS_0 // Usa a entrada CCI0A (P1.1) (s� tem esta)
            | CCIE;  // Habilita interrup��o de captura
    // Registrador de captura 1
    TACCTL1 = CAP   // Modo de captura
            | CM1   // Captura na subida (vai ser mudado a cada interrup��o)
            | CCIS_0 // Usa a entrada CCI1A (P1.2) (s� tem esta)
            | CCIE;  // Habilita interrup��o de captura
    // Ajustando a fun��o dos canais de entrada
    P1DIR &= ~(IN_AR | IN_DE); // P1.1 e P1.2 s�o entradas do temporizador.
    P1REN |= (IN_AR | IN_DE);  // Habilita resistor;
    P1OUT &= ~(IN_AR | IN_DE); // Liga o resistor como pull-down;
    P1SEL |= (IN_AR | IN_DE);  // Define a fun��o auxiliar 1 - Timer A
    inputChannel0Status = 0; // Coloca que est� em zero o canal 0 para esperar a pr�xima subida
    inputChannel1Status = 0; // Coloca que est� em zero o canal 1 para esperar a pr�xima subida
    inputChannel0Start = TA0R; //Pega o valor do timer para inicializar o contador do canal 0
    inputChannel1Start = inputChannel0Start; //e tamb�m do canal 1
    __bis_SR_register(GIE);
}

int captureCheck(void){
    if(inputChannel0Status!=2){ // se j� n�o estiver sem comunica��o
        if((TA0R-inputChannel0Start)> MAX_COMM_DELAY){ // se passar mais que 30 ms sem sinal
            inputChannel0Status = 2;
        }
    }
    if(inputChannel1Status!=2){ // se j� n�o estiver sem comunica��o
        if((TA0R-inputChannel1Start)> MAX_COMM_DELAY){ // se passar mais que 30 ms sem sinal
            inputChannel1Status = 2;
        }
    }
    if ((inputChannel0Status==2)||(inputChannel1Status==2)){ // se erro na comunica��o
        P2OUT |= LED_B; // acende o azul
        P1OUT &= ~(LED_G | LED_R) ;  // desliga o verde e o vermelho
        return 1;
    } else {  // se comunica��o chegando ok
        P1OUT &= ~LED_R; // desliga o vermelho
        P2OUT &= ~LED_B; // desliga o azul
        P1OUT |= LED_G;  // e acende o verde
        return 0;
    }
}

//Fun��o de inicializa��o de PWM
void PWM_setup(){
    P2OUT &= ~(MER | MEA | MDR | MDA); // os 4 com valor 0
    P2SEL &= ~(MER | MEA | MDR | MDA);
    P2DIR |= MER | MEA | MDR | MDA; // Os 4 como sa�da
    P2SEL2 &= ~(MER | MEA | MDR | MDA); // o sel2 fica sempre em 0;

    chARval = 0;  // d� valores iniciais aos canais
    chDEval = 0;
    //deneg = deltaT
}

void shortDelay(void){ // delay por software
    volatile long int i;
    for(i=300000; i>0;i--){}
}

// Fun��o de inicializa��o do led RGB.
void ledSetup(void){
    P1OUT &= ~(LED_R | LED_G); // led RGB catodo comum, coloca as sa�das em baixo para apagar.
    P2OUT &= ~LED_B;
    P1DIR |= LED_R | LED_G; // define os leds como sa�da
    P2DIR |= LED_B;
    P2SEL &= ~LED_B;

    // pisca os leds, s� para mostrar que iniciou
    if(!wtdgHappened){     // S� pisca se n�o resetou por conta do watchdog
        WDTCTL = WDTPW | WDTHOLD;  //desabilita o watchdog para piscar os leds
//        shortDelay();
        P1OUT |=  LED_R; // pisca o vermelho
        shortDelay();
        P1OUT &= ~LED_R;
        P1OUT |=  LED_G; // pisca o verde
        shortDelay();
        P1OUT &= ~LED_G;
        P2OUT |=  LED_B; // pisca o azul
        shortDelay();
        P2OUT &= ~LED_B;
        shortDelay();
    }
    WDTCTL = WDTPW | WDTCNTCL; // habilita o watchdog
}


int calcVal(int chVal,int min,int max,int deltaT){
    int chValDiff = chVal-min;
    int den = (chValDiff<deltaT)?deltaT:max-(min+deltaT);
    intermediate = (long)(chValDiff-deltaT)<<BITS_PWM; // <<BITS_PWM = *(2^BITS_PWM)  --> Faz a multiplica��o do valor para ficar na escala do PWM (limitado pelo long)
    return intermediate/den;
}

void changePWM(){
    // Defini��o dos valores dos motores esquerdo e direito
    MDval = chARval - chDEval;
    MEval = chARval + chDEval;

    // Defini��o dos valores dos PWMs
    //MOTOR DIREITO
    if(MDval>0){ // se avan�o direito
        pwmAd = MDval>PWM_MAX?PWM_MAX:MDval;
        pwmRd = 0;
        P2SEL &= ~MDR; // Define o recuo como I/O
        TA1CCR2 = pwmAd;
        P2SEL |= MDA; // Define o avanco como controlado pelo timer
    } else {  // se recuo direito
        pwmAd = 0;
        pwmRd = MDval<-PWM_MAX?PWM_MAX:-MDval;
        P2SEL &= ~MDA; // Define o avanco como I/O
        TA1CCR2 = pwmRd;
        P2SEL |= MDR; // Define o recuo como controlado pelo timer
    }

    //MOTOR ESQUERDO
    if(MEval>0){ // se avan�o esquerdo
        pwmAe = MEval>PWM_MAX?PWM_MAX:MEval;
        pwmRe = 0;
        P2SEL &= ~MER; // Define o recuo como I/O
        TA1CCR1 = pwmAe;
        P2SEL |= MEA; // Define o avanco como controlado pelo timer
    } else {  // se recuo esquerdo
        pwmAe = 0;
        pwmRe = MEval<-PWM_MAX?PWM_MAX:-MEval;
        P2SEL &= ~MEA; // Define o avanco como I/O
        TA1CCR1 = pwmRe;
        P2SEL |= MER; // Define o recuo como controlado pelo timer
    }
}

// Fun��o de gera��o do PWM
void pwmUpdate(void){
    // Atualiza as informa��es do canal 0 (Avan�o/recuo)
    ////// Tem que testar estas equa��es
    if(inputChannel0Status==3){ // se recebeu nova informa��o
        if((inputChannel0Value>=channel0Min)&&(inputChannel0Value<=channel0Max)){ // s� calcula se dentro da faixa
            chARval = calcVal(inputChannel0Value,channel0Min,channel0Max,channel0DeltaT);
        }

        inputChannel0Status = 0;
        changePWM(); // atualiza os valores dos PWMs
     }

    // Atualiza as informa��es do canal 1 (Direita/esquerda)
    ////// Tem que testar estas equa��es
    if(inputChannel1Status==3){ // se recebeu nova informa��o
        if((inputChannel1Value>=channel1Min)&&(inputChannel1Value<=channel1Max)){ // s� calcula se dentro da faixa
            chDEval = calcVal(inputChannel1Value,channel1Min,channel1Max,channel1DeltaT);
        }
        inputChannel1Status = 0;
        changePWM(); // atualiza os valores dos PWMs
    }

//    pwmCounter=pwmCounter==0?PWM_MAX:pwmCounter-1;

    if((inputChannel1Status == 2)){ // se estourou o timeout, zera as sa�das
    //if((inputChannel0Status == 2)||(inputChannel1Status == 2)){ // se estourou o timeout, zera as sa�das
        P2SEL &= ~(MDA | MDR); // zera as duas sa�das do motor da direita
        P2SEL &= ~(MEA | MER); // zera as duas sa�das do motor da esquerda
    } else { // se funcionando normalmente, compara os valores com o contador
/*        if (pwmAd>pwmCounter){
            P1OUT |= MDA;
        } else {
            P1OUT &= ~MDA;
        }
        if (pwmRd>pwmCounter){
            P1OUT |= MDR;
        } else {
            P1OUT &= ~MDR;
        }
        if (pwmAe>pwmCounter){
            P1OUT |= MEA;
        } else {
            P1OUT &= ~MEA;
        }
        if (pwmRe>pwmCounter){
            P1OUT |= MER;
        } else {
            P1OUT &= ~MER;
        }
*/
    }
}


// TODO:
int getFromFlash(){

    return 0;
}

// TODO:
int saveInFlash(){

    return 0;
}

/******************MAIN***************/

int main(void)


{
    wtdgHappened = (IFG1 & WDTIFG);
    setupClock();   // ajusta os clocks
    setupTimer_A1(); //ajusta o TimerA1
    ledSetup();     // ajusta os leds
    //WDTCTL = WDTPW | WDTCNTCL; // habilita o watchdog
    setupCapture(); // ajusta o timer para captura
    unsigned int loopCounter = TA0R; // valor inicial do loopCounter
//   if(calibrateSignal() && !wtdgHappened){  // se detecta o sinal de calibra��o e n�o resetou por causa do watchdog
//        calibrate();    // executa o procedimento de calibra��o
//    } else if (!wtdgHappened)
//    {
//        getFromFlash();
//    }

    PWM_setup();     // incializa os PWMs
    while(1){
        if((TA0R-loopCounter)>=LOOP_LENGTH){
//          P2SEL &= ~(MER | MEA | MDR | MDA); //teste
            WDTCTL = WDTPW | WDTCNTCL;  // chuta o cachorro


            captureCheck();             // checa se chegou algum sinal
            pwmUpdate();                // roda o pwm
            loopCounter += LOOP_LENGTH;
        }
    }
    return 0;
}


/******************INTERRUPCOES***************/

// Timer A CCR0 interrupt vector
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A_CCR0(void){
    switch(inputChannel0Status){
    case 0:     // Se estava embaixo
        inputChannel0Start = TA0R; //pega o valor do contador
        TACCTL0 &= ~CM0; // coloca para capturar na descida (CM=10)
        TACCTL0 |= CM1;
        inputChannel0Status=1; //coloca que est� em alto
        break;
    case 1:    // se estava em alto
        inputChannel0Value = TA0R-inputChannel0Start; //pega o valor do contador
        TACCTL0 &= ~CM1; // coloca para capturar na subida (CM=01)
        TACCTL0 |= CM0;
        inputChannel0Status=3; //coloca que recebeu
        break;
    case 2:    // se tinha perdido o sinal e chegou novo
        TACCTL0 &= ~CM1; // coloca para capturar na subida (CM=01)
        TACCTL0 |= CM0;
        inputChannel0Status=0; //coloca que est� em baixo e espera o pr�ximo pulso
        break;
    }
//    inputChannel0Start=TA0R;
}


// Timer A Interrupt Vector (TAIV) handler
#pragma vector=TIMERA1_VECTOR
__interrupt void Timer_A(void)
{
switch( TAIV ){
  case  2:    // CCR1 -  usada na captura do canal 1
    switch(inputChannel1Status){
    case 0:     // Se estava embaixo
            inputChannel1Start = TA0R; //pega o valor do contador
            TACCTL1 &= ~CM0; // coloca para capturar na descida (CM=10)
            TACCTL1 |= CM1;
            inputChannel1Status=1; //coloca que est� em alto
            break;
    case 1:    // se estava em alto
        inputChannel1Value = TA0R-inputChannel1Start; //pega o valor do contador
        TACCTL1 &= ~CM1; // coloca para capturar na subida (CM=01)
        TACCTL1 |= CM0;
        inputChannel1Status=3; //coloca que recebeu
        break;
    case 2:    // se tinha perdido o sinal
        TACCTL1 &= ~CM1; // coloca para capturar na subida (CM=01)
        TACCTL1 |= CM0;
        inputChannel1Status=0; //coloca que est� em baixo e espera o pr�ximo pulso
        break;
    }
    break;
  case  4: break;  // CCR2 not used
  case 10:         // overflow - usada na calibra��o
   // timeCountCal++;
    break;
}

}
