/**********************************************************
 *
 * P1.1 - canal avanço/recuo
 * P1.2 - canal direita/esquerda
 * P1.0 - motor direito avanço
 * P1.6 - motor direito recuo
 * P1.3 - motor esquerdo avanço
 * P1.4 - led vermelho
 * P1.5 - led verde
 * P2.7 - led azul
 * P1.7  - motor esquerdo recuo
 *                         ---------------
 *                    VDD--| 1        14 |--GND
 *            P1.0    MDA--| 2        13 |--        P2.6
 *            P1.1  IN_AR--| 3        12 |-- LED_B  P2.7
 *            P1.2  IN_DE--| 4        11 |-- TEST
 *            P1.3    MEA--| 5        10 |-- RST
 *            P1.4  LED_R--| 6         9 |-- MER    P1.7
 *            P1.5  LED_G--| 7         8 |-- MDR    P1.6
 *                         --------------
 *********************************************************/

#include <msp430.h>

// Ports e bits dos leds
// Port 1
#define LED_R BIT4
#define LED_G BIT5
// Port 2
#define LED_B BIT7

// Bits dos motores
#define MDA BIT0
#define MDR BIT6
#define MEA BIT3
#define MER BIT7

// Bits dos sinais de entrada
#define IN_AR BIT1
#define IN_DE BIT2

#define MAX_COMM_DELAY 30000
// definindo o SMCLK em 1MHz, o LOOP_LENGTH causará um loop a 100 kHz.
#define LOOP_LENGTH 10
unsigned int inputChannel0Status, inputChannel1Status;
unsigned int inputChannel0Value,inputChannel1Value;
unsigned int inputChannel0Start,inputChannel1Start;

// Valores mínimos e máximos do tempo do canal
unsigned int channel0Min = 1053; // 1000 micro segundos.
unsigned int channel0Max = 2107; // 2000 micro segundos.
unsigned int channel0DeltaT = 526;
unsigned int channel1Min = 1053; // 1000 micro segundos.
unsigned int channel1Max = 2107; // 2000 micro segundos.
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
    // Definimos o MCLK como sendo o DCO (SELMx=00) sem dividir (DIVMx=00) e o SMCLK como sendo o DCO (SELS=0) dividido por 8 (DIVSx = 11);
    BCSCTL2 = SELM0 | DIVM_0 | DIVS_3;
}


void setupCapture(){
    // Timer0_A
    TACTL = TASSEL_2 // Escolhe o SMCLK (2MHz) como entrada
          | ID_1     // dividido por 2 (1MHz) - dá um período total de 65,536 ms
          | MC_2;    // Continuous mode
          //| TAIE;   // com interrupções habilitadas
    // Registrador de captura 0
    TACCTL0 = CAP   // Modo de captura
            | CM1   // Captura na subida (vai ser mudado a cada interrupção)
            | CCIS_0 // Usa a entrada CCI0A (P1.1) (só tem esta)
            | CCIE;  // Habilita interrupção de captura
    // Registrador de captura 1
    TACCTL1 = CAP   // Modo de captura
            | CM1   // Captura na subida (vai ser mudado a cada interrupção)
            | CCIS_0 // Usa a entrada CCI1A (P1.2) (só tem esta)
            | CCIE;  // Habilita interrupção de captura
    // Ajustando a função dos canais de entrada
    P1DIR &= ~(IN_AR | IN_DE); // P1.1 e P1.2 são entradas do temporizador.
    P1REN |= (IN_AR | IN_DE);  // Habilita resistor;
    P1OUT &= ~(IN_AR | IN_DE); // Liga o resistor como pull-down;
    P1SEL |= (IN_AR | IN_DE);  // Define a função auxiliar 1 - Timer A
    inputChannel0Status = 0; // Coloca que está em zero o canal 0 para esperar a próxima subida
    inputChannel1Status = 0; // Coloca que está em zero o canal 1 para esperar a próxima subida
    inputChannel0Start = TA0R; //Pega o valor do timer para inicializar o contador do canal 0
    inputChannel1Start = inputChannel0Start; //e também do canal 1
    __bis_SR_register(GIE);
}

int captureCheck(void){
    if(inputChannel0Status!=2){ // se já não estiver sem comunicação
        if((TA0R-inputChannel0Start)> MAX_COMM_DELAY){ // se passar mais que 30 ms sem sinal
            inputChannel0Status = 2;
        }
    }
    if(inputChannel1Status!=2){ // se já não estiver sem comunicação
        if((TA0R-inputChannel1Start)> MAX_COMM_DELAY){ // se passar mais que 30 ms sem sinal
            inputChannel1Status = 2;
        }
    }
    if ((inputChannel0Status==2)||(inputChannel1Status==2)){ // se erro na comunicação
        P2OUT &= ~LED_B; // desliga o azul
        P1OUT |= LED_R | LED_G;  // acende o vermelho e o verde
        return 1;
    } else {  // se comunicação chegando ok
        P1OUT &= ~LED_R; // desliga o vermelho
        P2OUT &= ~LED_B; // desliga o azul
        P1OUT |= LED_G;  // e acende o verde
        return 0;
    }
}

// Função de inicialização do PWM
void pwmSetup(void){
    P1OUT &= ~(MDA | MDR | MEA | MER);  // por segurança, zera os bits de saída PWM
    P1DIR |=  (MDA | MDR | MEA | MER);  // Define as saídas
    chARval = 0;  // dá valores iniciais aos canais
    chDEval = 0;
    //deneg = deltaT
}

void shortDelay(void){ // delay por software
    volatile long int i;
    for(i=300000; i>0;i--){}
}

// Função de inicialização do led RGB.
void ledSetup(void){
    P1OUT &= ~(LED_R | LED_G); // led RGB catodo comum, coloca as saídas em baixo para apagar.
    P2OUT &= ~LED_B;
    P1DIR |= LED_R | LED_G; // define os leds como saída
    P2DIR |= LED_B;
    P2SEL &= ~LED_B;

    // pisca os leds, só para mostrar que iniciou
    if(!wtdgHappened){     // Só pisca se não resetou por conta do watchdog
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
    intermediate = (long)(chValDiff-deltaT)<<8; // *256
    return intermediate/den;
}

void changePWM(){
    // Definição dos valores dos motores esquerdo e direito
    MDval = chARval - chDEval;
    MEval = chARval + chDEval;

    // Definição dos valores dos PWMs
    if(MDval>0){ // se avanço direito
        pwmAd = MDval>255?255:MDval;
        pwmRd = 0;
    } else {  // se recuo direito
        pwmAd = 0;
        pwmRd = MDval<-255?255:-MDval;
    }

    if(MEval>0){ // se avanço esquerdo
        pwmAe = MEval>255?255:MEval;
        pwmRe = 0;
    } else {  // se recuo esquerdo
        pwmAe = 0;
        pwmRe = MEval<-255?255:-MEval;
    }
}

// Função de geração do PWM
void pwmUpdate(void){
    // Atualiza as informações do canal 0 (Avanço/recuo)
    ////// Tem que testar estas equações
    if(inputChannel0Status==3){ // se recebeu nova informação
        if((inputChannel0Value>=channel0Min)&&(inputChannel0Value<=channel0Max)){ // só calcula se dentro da faixa
            chARval = calcVal(inputChannel0Value,channel0Min,channel0Max,channel0DeltaT);
//            int chValDiff = inputChannel0Value-channel0Min;
//            int den = (chValDiff<channel0DeltaT)?channel0DeltaT:channel0Max-(channel0Min+channel0DeltaT);
//            intermediate = (long)(chValDiff-channel0DeltaT)<<8; // *256
//            chARval = intermediate/den;
        }
        inputChannel0Status = 0;
        changePWM(); // atualiza os valores dos PWMs
     }
    // Atualiza as informações do canal 1 (Direita/esquerda)
    ////// Tem que testar estas equações
    if(inputChannel1Status==3){ // se recebeu nova informação
        if((inputChannel1Value>=channel1Min)&&(inputChannel1Value<=channel1Max)){ // só calcula se dentro da faixa
            chDEval = calcVal(inputChannel1Value,channel1Min,channel1Max,channel1DeltaT);
        }
        inputChannel1Status = 0;
        changePWM(); // atualiza os valores dos PWMs
    }

    pwmCounter++; // como é um char, vai de 0 a 255

    if((inputChannel1Status == 2)){ // se estourou o timeout, zera as saídas
    //if((inputChannel0Status == 2)||(inputChannel1Status == 2)){ // se estourou o timeout, zera as saídas
        P1OUT &= ~(MDA | MDR); // zera as duas saídas do motor da direita
        P1OUT &= ~(MEA | MER); // zera as duas saídas do motor da esquerda
    } else { // se funcionando normalmente, compara os valores com o contador
        if (pwmAd>pwmCounter){
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

//int calibrateSignal(void){
//    // espera receber sinal nos dois canais ou dar um timeout
//    P1OUT &= ~(LED_G | LED_R);
//    P2OUT |= LED_B;
//    while(!(((inputChannel0Status == 3) && (inputChannel1Status == 3)) || (inputChannel0Status == 2) || (inputChannel1Status == 2))){
//        captureCheck();
//    }
//    if((inputChannel0Status == 2) || (inputChannel1Status == 2)){ // se foi por timeout
//        return 0; // sai com falso
//    } else {
//        if ((inputChannel0Value < (channel0Min + (channel0DeltaT>>2))) && (inputChannel1Value < (channel1Min + (channel1DeltaT>>2)))){
//            return 1; // Se a condição for reconhecida
//        } else {
//            return 0;
//        }
//    }
//}
//
volatile int timeCountCal = 0;
//int calibrate(void){
//    // Acende o azul de calibração
//    unsigned int minch0 = 20000;
//    unsigned int minch1 = 20000;
//    unsigned int maxch0 = 0;
//    unsigned int maxch1 = 0;
//    unsigned int midch0, midch1;
//    TACTL |= TAIE; //  habilita a interrupção de estouro do Timer0_A
//    timeCountCal = 0; // inicializa o contador de tempo da calibração
//    // este contador é incrementado na interrupção a cada ~65ms. Para um tempo próximo a 2s tem-se que contá-lo 31 vezes
//    while(timeCountCal<31){
//        if(inputChannel0Status==3){ // se chegou valor no canal 0
//            if (inputChannel0Value<minch0) // atualiza o mínimo
//                minch0 = inputChannel0Value;
//            if (inputChannel0Value>maxch0) // atualiza o máximo
//                maxch0 = inputChannel0Value;
//            int deltaI0 = inputChannel0Value > midch0? inputChannel0Value - midch0: midch0 - inputChannel0Value;
//            if(deltaI0>50){
//                midch0 = inputChannel0Value;
//                timeCountCal = 0;
//            }
//            inputChannel0Status = 0; // coloca para receber novo valor
//        }
//        if(inputChannel1Status==3){ // se chegou valor no canal 1
//            if (inputChannel1Value<minch1) // atualiza o mínimo
//                minch1 = inputChannel1Value;
//            if (inputChannel1Value>maxch1) // atualiza o máximo
//                maxch1 = inputChannel1Value;
//            int deltaI1 = inputChannel1Value > midch1? inputChannel1Value - midch1: midch1 - inputChannel1Value;
//            if(deltaI1>50){
//                midch1 = inputChannel1Value;
//                timeCountCal = 0;
//            }
//            inputChannel0Status = 0; // coloca para receber novo valor
//        }
//    }
//    TACTL &= ~TAIE; // desabilita a interrupção de estouro do Timer0_A
//    if(((midch0-minch0)>200) && ((maxch0-minch0)>200) && ((midch1-minch1)>200) && ((maxch1-minch1)>200)){ // se valores novos fazem sentido
//        channel0Min = minch0;// atualiza limites do canal 0
//        channel0Max = maxch0;
//        channel0DeltaT = midch0-minch0;
//        channel1Min = minch1;// atualiza limites do canal 1
//        channel1Max = maxch1;
//        channel1DeltaT = midch1-minch1;
//        // Salve na flash
//        saveInFlash();
//        // pisca verde
//        P2OUT &= ~LED_B;
//        shortDelay();
//        P1OUT |= LED_G;
//        shortDelay();
//        P1OUT &= ~LED_G;
//    } else { // se calibração está esquisita
//        // pisca vermelho 2x
//        getFromFlash();
//        P2OUT &= ~LED_B;
//        shortDelay();
//        P1OUT |= LED_R;
//        shortDelay();
//        P1OUT &= ~LED_R;
//        shortDelay();
//        P1OUT |= LED_R;
//        shortDelay();
//        P1OUT &= ~LED_R;
//    }
//    return 0;
//}

int main(void)
{
    wtdgHappened = (IFG1 & WDTIFG);
    setupClock();   // ajusta os clocks
    ledSetup();     // ajusta os leds
    //WDTCTL = WDTPW | WDTCNTCL; // habilita o watchdog
    setupCapture(); // ajusta o timer para captura
    unsigned int loopCounter = TA0R; // valor inicial do loopCounter
//   if(calibrateSignal() && !wtdgHappened){  // se detecta o sinal de calibração e não resetou por causa do watchdog
//        calibrate();    // executa o procedimento de calibração
//    } else if (!wtdgHappened)
//    {
//        getFromFlash();
//    }
    
    pwmSetup();     // incializa os PWMs
    while(1){
        if((TA0R-loopCounter)>=LOOP_LENGTH){
            WDTCTL = WDTPW | WDTCNTCL;  // chuta o cachorro
            captureCheck();             // checa se chegou algum sinal 
            pwmUpdate();                // roda o pwm
            loopCounter += LOOP_LENGTH;
        }
    }
	return 0;
}

// Timer A CCR0 interrupt vector
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A_CCR0(void){
    switch(inputChannel0Status){
    case 0:     // Se estava embaixo
        inputChannel0Start = TA0R; //pega o valor do contador
        TACCTL0 &= ~CM0; // coloca para capturar na descida (CM=10)
        TACCTL0 |= CM1;
        inputChannel0Status=1; //coloca que está em alto
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
        inputChannel0Status=0; //coloca que está em baixo e espera o próximo pulso
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
            inputChannel1Status=1; //coloca que está em alto
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
        inputChannel1Status=0; //coloca que está em baixo e espera o próximo pulso
        break;
    }
    break;
  case  4: break;  // CCR2 not used
  case 10:         // overflow - usada na calibração
    timeCountCal++;
    break;
}

}
