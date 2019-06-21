#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "types.h"

typedef enum
{
    GUI_IDLE,
    WAITINGFORPITCH,
    WAITINGFORDISTANCE,
    WAITINGFORPOSITION,

} guiState_t;

volatile motion_status_t status;
volatile operation_mode_t mode;
volatile guiState_t guiState;
volatile int8_t currPage;
volatile int8_t pageRequested;
volatile int8_t inputUpdateRequested;
volatile uint32_t millis;
const char *statuses[] = {"IDLE", "PENDING", "JOG", "ACCELERATE", "FOLLOW", "DECELERATE", "DONE", "RETURN"};
const char *modes[] = {"BASIC", "THREAD", "INDEX"};
volatile float position;
#define STEPS_PER_MM 755.91;
volatile float pitch;
volatile float distance;
volatile float rpm;
volatile int editMode;

void usart_setup(void);
int waitNextionInput(char *ptr, char start_code);
uint16_t getPage();


int main(void) {
    static uint32_t lastrequested;
    motion_status_t prevstatus = FOLLOW;
    operation_mode_t prevmode = BASIC;
    currPage = -1; //-1 means non-valid;
    pageRequested = 0;
    inputUpdateRequested = 0;
    guiState = GUI_IDLE;
    uint16_t prevpage;
    status = IDLE;
    mode = BASIC;
    editMode = 0;
    usart_setup();
    delay_setup();

    delay_ms(1000);
    while(1)
    {
        delay_ms(10);
        if(inputUpdateRequested)
            getNextionInputs();

        if((!pageRequested && currPage < 0) || millis-lastrequested > 100)
        {
            lastrequested = millis;
            getPage();
        }
        else if(currPage > -1 && currPage < 2)
        {
            updateStatus();
            if(currPage != mode)
            {
                updateMode(currPage);
            }
            if(prevstatus != status)
                updateButtons();
            prevstatus = status;

            if(mode == BASIC)
                updatePage0();
            else if (mode == THREADING)
                updatePage1();
            else
                updatePage2();

        }
        //update done, invalidate currPage so we can request again

        if (currPage > -1)
            currPage = -1;

    }

}
#define USART_BUF_LEN 100

volatile char nextionBuf[USART_BUF_LEN];
volatile char elsBuf[USART_BUF_LEN];
volatile char elsOutBuf[USART_BUF_LEN];
volatile char nextionOutBuf[USART_BUF_LEN];
uint16_t nextionCtr;
uint16_t elsCtr;


void sys_tick_handler(void)
{
	millis++;
}

void changeBaud(void)
{
    int i = sprintf(nextionOutBuf, "baud=115200");
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
    delay_ms(100);
    usart_disable(USART3);
    usart_set_baudrate(USART3, 115200);
    usart_enable(USART3);

}
void delay_setup(void)
{
	/* set up a microsecond free running timer for ... things... */
	rcc_periph_clock_enable(RCC_TIM6);
	/* microsecond counter */
	timer_set_prescaler(TIM6, rcc_apb1_frequency / 1e6 - 1);
	timer_set_period(TIM6, 0xffff);
	timer_one_shot_mode(TIM6);
}
void delay_us(uint16_t us)
{
	TIM_ARR(TIM6) = us;
	TIM_EGR(TIM6) = TIM_EGR_UG;
	TIM_CR1(TIM6) |= TIM_CR1_CEN;
	//timer_enable_counter(TIM6);
	while (TIM_CR1(TIM6) & TIM_CR1_CEN);
}
void delay_ms(uint16_t ms)
{
    uint16_t i;
    for(i=0; i<ms; i++)
    {
        delay_us(1000);
    }
}
void usart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

	rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART3);
    /* 72MHz / 8 => 9000000 counts per second */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

    /* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
    /* SysTick interrupt every N clock pulses: set reload to N-1 */
    systick_set_reload(8999);

    systick_interrupt_enable();

    /* Start counting. */
    systick_counter_enable();

    nextionCtr=0;
    elsCtr=0;

    /* Enable the USART1 interrupt. */
    nvic_set_priority(NVIC_USART1_IRQ, 0);
	nvic_enable_irq(NVIC_USART1_IRQ);


	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  	/* Setup GPIO pin GPIO_USART1_RE_RX on GPIO port B for receive. */
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
              GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Enable USART1 Receive interrupt. */
	USART_CR1(USART1) |= USART_CR1_RXNEIE;

	/* Finally enable the USART. */
	usart_enable(USART1);


////// USART 3 for communicating to Nextion display

    /* Enable the USART2 interrupt. */
    nvic_set_priority(NVIC_USART3_IRQ, 4);
    nvic_enable_irq(NVIC_USART3_IRQ);

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);

    /* Setup GPIO pin GPIO_USART2_RE_RX on GPIO port  for receive. */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
              GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);
    /* Setup UART parameters. */
    usart_set_baudrate(USART3, 9600);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX_RX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

    /* Enable USART2 Receive interrupt. */
    USART_CR1(USART3) |= USART_CR1_RXNEIE;

    /* Finally enable the USART. */
    usart_enable(USART3);
}
void handleEls()
{

    char *p;
    int32_t tmp, tmp2;
    p = strtok(elsBuf, ",");
    mode = atoi(p);
    p = strtok(NULL, ",");
    status = atoi(p);
    p = strtok(NULL, ",");

    tmp = atoi(p);
    p = strtok(NULL, ",");
    pitch = atof(p);
    p = strtok(NULL, ",");
    distance = atof(p);
    p = strtok(NULL, ",");
    rpm = atof(p);
    p = strtok(NULL, ",");
    tmp2 = atoi(p);

    position = (tmp-tmp2) / (float)STEPS_PER_MM;

}
void changePage()
{
    int i = sprintf(nextionOutBuf, "page %i", mode);
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
}
void updateMode(uint16_t page)
{
    //Threading button

    int i = sprintf(elsOutBuf, "SET_MODE,%s,\n", modes[page]);
    elsWrite(i);
}
uint16_t getPage()
{
    uint16_t i;

    i = sprintf(nextionOutBuf, "sendme");
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
    pageRequested = 1;
//    i = waitNextionInput(retArray,0x66);
//    i = retArray[i];

//    return i;



}
void cancelEdit()
{
    delay_ms(100);
    int i = sprintf(nextionOutBuf, "click b251,0");
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
}
void nextionWrite(int len)
{
    int i;


    for(i = 0; i<len; i++)
        usart_send_blocking(USART3, nextionOutBuf[i]);

}

void elsWrite(int len)
{
    int i;

    for(i = 0; i<len; i++)
        usart_send_blocking(USART1, elsOutBuf[i]);

}
void updateButtons()
{
    uint16_t i;
    //buttons updated when status changes

            if(status == IDLE)
            {
                i = sprintf(nextionOutBuf,"tsw b0,1");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw b1,1");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw b2,1");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw b6,1");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);

                i = sprintf(nextionOutBuf,"tsw t5,1");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw t7,1");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw t10,1");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);

                i = sprintf(nextionOutBuf,"b3.txt=\"START\"");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
            }
            else
            {
                //disable all buttons and edits

                i = sprintf(nextionOutBuf,"tsw b0,0");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw b1,0");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw b2,0");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw b6,0");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);

                i = sprintf(nextionOutBuf,"tsw t5,0");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw t7,0");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);
                i = sprintf(nextionOutBuf,"tsw t10,0");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);


                i = sprintf(nextionOutBuf,"b3.txt=\"STOP\"");
            nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
                nextionWrite(i+3);

            }

}
void updatePage0()
{
    uint16_t i;
    int j = (int)status;
    //feed pitch
    i = sprintf(nextionOutBuf, "t5.txt=\"%4.2f\"", pitch);
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);


}
void updateStatus()
{
    //status
    uint16_t i;
    int j = (int)status;
    i = sprintf(nextionOutBuf, "t0.txt=\"%s\"", statuses[j]);
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
    j = (int)mode;
    i = sprintf(nextionOutBuf, "t2.txt=\"%s\"", modes[j]);
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
    //rpm
    i = sprintf(nextionOutBuf, "n0.val=%i", (int)(rpm*60));
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);

}
void updatePage1()
{
    uint16_t i;
    int j = (int)status;
    //feed pitch
    i = sprintf(nextionOutBuf, "t5.txt=\"%4.2f\"", pitch);
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
    //dist
    i = sprintf(nextionOutBuf, "t7.txt=\"%4.2f\"", distance);
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);
    //position
    i = sprintf(nextionOutBuf, "t10.txt=\"%4.2f\"", position);
nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
    nextionWrite(i+3);

}
void updatePage2()
{
}

void getNextionInputs()
{
    int i,j;
    float f;
    switch (mode)
    {
        uint32_t requestTime;
        case BASIC:
            i = sprintf(nextionOutBuf, "get t5.txt");
        nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
            nextionWrite(i+3);
            guiState = WAITINGFORPITCH;
            requestTime = millis;
            while (guiState == WAITINGFORPITCH)
            {
                if(millis-requestTime > 1000)
                {
                    guiState = GUI_IDLE;
                    return;
                }
            }
            break;
        case THREADING:
            i = sprintf(nextionOutBuf, "get t5.txt");
        nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
            nextionWrite(i+3);
            guiState = WAITINGFORPITCH;
            requestTime = millis;
            while (guiState == WAITINGFORPITCH)
            {
                if(millis-requestTime > 1000)
                {
                    guiState = GUI_IDLE;
                    return;
                }
            }
            i = sprintf(nextionOutBuf, "get t7.txt");
        nextionOutBuf[i] = 0xFF; nextionOutBuf[i+1]=0xFF; nextionOutBuf[i+2] = 0xFF;
            nextionWrite(i+3);
            guiState = WAITINGFORDISTANCE;
            requestTime = millis;
            while (guiState == WAITINGFORDISTANCE)
            {
                if(millis-requestTime > 1000)
                {
                    guiState = GUI_IDLE;
                    return;
                }
            }
            break;
    }
    inputUpdateRequested = 0;
}
void handleNextion()
{
    int i;
    float f;
    switch(nextionBuf[0])
    {

        case 0x65:
            //button pressed
            //page0 buttons
/*            if(nextionBuf[1] == 0 && nextionBuf[2] == 2 && nextionBuf[3] == 1)
            {
                //Threading button
                i = sprintf(outBuf, "SET_MODE,THREAD,\n");
                elsWrite(i);
            }
            else if(nextionBuf[1] == 0 && nextionBuf[2] == 3 && nextionBuf[3] == 0)
            {
                //Indexing button
                i = sprintf(outBuf, "SET_MODE,INDEX,\n");
                elsWrite(i);
            }*/
            if(nextionBuf[1] == 0 && nextionBuf[2] == 9 && nextionBuf[3] == 1)
            {
                //START button
                if(status == IDLE)
                    i = sprintf(elsOutBuf, "START,\n");
                else
                    i = sprintf(elsOutBuf, "STOP,\n");

                elsWrite(i);
            }
            else if(nextionBuf[1] == 0 && nextionBuf[2] == 0x0B && nextionBuf[3] == 1)
            {
                //Feed edit
                if(status==IDLE)
                    editMode = 1;
                else
                    cancelEdit();
            }

    /*        //page1 buttons
            else if(nextionBuf[1] == 1 && nextionBuf[2] == 1 && nextionBuf[3] == 1)
            {
                //Basic button
                i = sprintf(outBuf, "SET_MODE,BASIC,\n");
                elsWrite(i);
            }
            else if(nextionBuf[1] == 1 && nextionBuf[2] == 3 && nextionBuf[3] == 0)
            {
                //Indexing button
                i = sprintf(outBuf, "SET_MODE,INDEX,\n");
                elsWrite(i);
            }*/
            else if(nextionBuf[1] == 1 && nextionBuf[2] == 0x10 && nextionBuf[3] == 1)
            {
                //Set button of position

                i = sprintf(elsOutBuf, "SET_REF_POS,0,\n", f);

                elsWrite(i);
            }
            else if(nextionBuf[1] == 1 && nextionBuf[2] == 0x11 && nextionBuf[3] == 1)
            {
                //Start button
                if(status == IDLE)
                    i = sprintf(elsOutBuf, "START,\n");
                else
                    i = sprintf(elsOutBuf, "STOP,\n");

                elsWrite(i);
            }
            else if(nextionBuf[1] == 1 && nextionBuf[2] == 0x12 && nextionBuf[3] == 1)
            {
                //Start button
                i = sprintf(elsOutBuf, "RETURN,\n");
                elsWrite(i);
            }

            else if(nextionBuf[1] == 3 && nextionBuf[2] == 0x04 && nextionBuf[3] == 0)
            {
                //OK button from numeric edit
                inputUpdateRequested = 1;

            }
            break;
        case 0x66:
            //page received

            currPage = nextionBuf[1];
            pageRequested = 0;
            break;
        case 0x70:
            switch (guiState)
            {
                case GUI_IDLE:
                    return;
                case WAITINGFORPITCH:
                    f = atof(nextionBuf + 1);
                    if (f!=pitch)
                    {
                        i = sprintf(elsOutBuf, "SET_PITCH,%4.2f,\n", f);
                        elsWrite(i);
                    }
                    guiState = GUI_IDLE;
                    break;
                case WAITINGFORDISTANCE:
                    f = atof(nextionBuf + 1);
                    if (f!=distance)
                    {
                        i = sprintf(elsOutBuf, "SET_LENGTH,%4.2f,\n", f);
                        elsWrite(i);
                    }
                    guiState = GUI_IDLE;
                    break;
            }
            break;


    }

}
void usart1_isr(void)
{
    while ((USART_SR(USART1) & USART_SR_RXNE) != 0)
    {
        elsBuf[elsCtr] = usart_recv(USART1);
        if (elsBuf[elsCtr] == '\n' || elsBuf[elsCtr] == 13)
        {
            handleEls();
            elsCtr=0;
        }
        else if(elsCtr < USART_BUF_LEN-2)
            elsCtr++;


    }
}
void usart3_isr(void)
{
    while ((USART_SR(USART3) & USART_SR_RXNE) != 0)
    {
        nextionBuf[nextionCtr] = usart_recv(USART3);
        if (nextionBuf[nextionCtr] == 0xFF && nextionBuf[nextionCtr-1] == 0xFF && nextionBuf[nextionCtr-2] == 0xFF)
        {
            nextionBuf[nextionCtr-2] = '\0';
            handleNextion();
            nextionCtr=0;
        }
        else if(nextionCtr < USART_BUF_LEN-2)
            nextionCtr++;


    }
}
