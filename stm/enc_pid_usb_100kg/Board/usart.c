#include "usart.h"
#include "Dynamixel_control.h"
#include "robot.h"

/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */

dmaPackStruct dmaData= {-1,-1,0,0,0,0,0,0,0,0};
char Buffer[BufSize];
uint8_t usart1Data[6];
 extern encInPackStruct encData = {0xAA,0x01,0.0,0.0,0.0,0.0,0.0,0.0}; //входящие данные с энкодера
void init_USART1(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_9b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_0_5  ;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE); // enable the USART1 receive interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}
void init_USART2_1(uint32_t baudrate){
/*
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOD, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_9b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_0_5  ;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART2, USART_IT_TC, ENABLE); // enable the USART1 receive interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART2, ENABLE);
}
void init_USART6_1(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOC, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6); //
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_9b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_0_5  ;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART6, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART6, USART_IT_TC, ENABLE); // enable the USART1 receive interrupt
	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART6, ENABLE);
	}


/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}





char sendPacket(char* data, char size) // отправить пакет
{
  char state = 0;
  char i=0;

   for ( i = 0; i < (size ); i++)
        putchar(*(data + i));

  return state;
}

uint32_t packetCheck(char* dataToCheck, char size) //проверить пакет
{
     CRC->CR=1;
     char i;
     for ( i = 0; i < (size ); i++)
     CRC->DR = *(dataToCheck + i);
return CRC->DR;
}

void usartSendByte(USART_TypeDef *USART,uint8_t byte)
{
	while(!(USART->SR & USART_SR_TC));
	USART->DR = byte;
}
////////////////////////////////////////////////////////////////////////////////
void DMA1_Stream5_IRQHandler(void)     //????????????
{
  DMA1->HIFCR = DMA_HIFCR_CTCIF5;
}
////////////////////////////////////////////////////////////////////////////////
//void DMA1_Stream1_IRQHandler(void)
//{
//  DMA1->LIFCR = DMA_LIFCR_CTCIF1;
//}
////////////////////////////////////////////////////////////////////////////////
void uartInit(USART_TypeDef* USARTx, uint32_t USART_BaudRate)   // инициализация USART
{
  uint32_t tmpreg = 0x00, apbclock = 0x00;
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  RCC_ClocksTypeDef RCC_ClocksStatus;
/*---------------------------- USART BRR Configuration -----------------------*/
  RCC_GetClocksFreq(&RCC_ClocksStatus);
  if ((USARTx == USART1) || (USARTx == USART6))
    apbclock = RCC_ClocksStatus.PCLK2_Frequency;
  else
    apbclock = RCC_ClocksStatus.PCLK1_Frequency;
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    integerdivider = ((25 * apbclock) / (2 * USART_BaudRate ));
  else
    integerdivider = ((25 * apbclock) / (4 * USART_BaudRate ));
  tmpreg = (integerdivider / 100) << 4;
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
    tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
  else
    tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);

  //USARTx->CR3 |= 0x40;   // DMA enable receiver
  USARTx->BRR = (uint16_t)tmpreg;
  USARTx->CR1 = USART_CR1_RXNEIE | USART_CR1_TE |  USART_CR1_RE |  USART_CR1_UE;
}
////////////////////////////////////////////////////////////////////////////////
void configUsart1TXDMA(char * usart1Data, int count)
{
  DMA2_Stream7->CR &=~ DMA_SxCR_EN;
  DMA2_Stream7->CR |= 4 << 25;        //Выбираем channel 4
  DMA2_Stream7->PAR = (uint32_t) &USART1->DR;//Задаем адрес периферии - регистр результата преобразования АЦП для регулярных каналов.
  DMA2_Stream7->M0AR = (uint32_t) usart1Data; //Задаем адрес памяти - базовый адрес массива в RAM.
  DMA2_Stream7->CR &= ~DMA_SxCR_DIR; //Направление передачи данных - чтение из периферии, запись в память.
  DMA2_Stream7->CR |= DMA_SxCR_DIR_0; //Направление передачи данных - чтение из периферии, запись в память.
  DMA2_Stream7->NDTR = count;
  DMA2_Stream7->CR &= ~DMA_SxCR_PINC; //Адрес периферии не инкрементируется после каждой пересылки.
  DMA2_Stream7->CR |= DMA_SxCR_MINC; //Адрес памяти инкрементируется после каждой пересылки.
  DMA2_Stream7->CR |= DMA_SxCR_PL; //Приоритет
  DMA2_Stream7->CR |= DMA_SxCR_TCIE; // прерывание в конце передачи
  USART1->SR&=~(USART_SR_TC);
  DMA2->LIFCR = DMA_LIFCR_CTCIF3;
  DMA2_Stream7->CR |= DMA_SxCR_EN;
  NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}
////////////////////////////////////////////////////////////////////////////////
void configUsart1RXDMA(char * usart1Data, int count)
{
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;
  DMA2_Stream2->CR |= 4 << 25;        //Выбираем channel 4
  DMA2_Stream2->PAR = (uint32_t) &USART1->DR;//Задаем адрес периферии - регистр результата преобразования АЦП для регулярных каналов.
  DMA2_Stream2->M0AR = (uint32_t) usart1Data; //Задаем адрес памяти - базовый адрес массива в RAM.
  DMA2_Stream2->CR &= ~DMA_SxCR_DIR; //Направление передачи данных - чтение из периферии, запись в память.
  DMA2_Stream2->NDTR = count; //Количество пересылаемых значений
  DMA2_Stream2->CR &= ~DMA_SxCR_PINC; //Адрес периферии не инкрементируется после каждой пересылки.
  DMA2_Stream2->CR |= DMA_SxCR_MINC; //Адрес памяти инкрементируется после каждой пересылки.
  DMA2_Stream2->CR |= DMA_SxCR_PL; //Приоритет
  DMA2_Stream2->CR |= DMA_SxCR_TCIE; // прерывание в конце передачи
  DMA2->LIFCR = DMA_LIFCR_CTCIF1;
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

////////////////////////////////////////////////////////////////////////////////
//__________Прерывание_DMA1_____________________________________________________
//
void DMA1_Stream3_IRQHandler(void) // DMA USART2-TX
{
  int temp1=0;
   dmaData.stDmaBusy =0;

  if (DMA1->LISR & DMA_LISR_TCIF3) // если обмен закончен
  {

    DMA1_Stream3->CR &= ~DMA_SxCR_EN;
    DMA1->LIFCR |= DMA_LIFCR_CTCIF3|DMA_LIFCR_CHTIF3;//очистить флаг окончания обмена.
    if (dmaData.curWorkAdr>dmaData.dmaEndByte) temp1 =dmaData.curWorkAdr-dmaData.dmaEndByte-1;
    else                       temp1 = dmaData.curWorkAdr-1 + BufSize-dmaData.dmaEndByte-1;

    if ((temp1>0))
    {
      if (dmaData.curWorkAdr>dmaData.dmaEndByte)//заполнение буфера опережает отправку (h впереди cur)
      {
        if (dmaData.dmaEndByte==BufSize-1) dmaData.dmaEndByte=-1;
        dmaData.dmaStartByte = dmaData.dmaEndByte+1;
        dmaData.dmaEndByte =dmaData.curWorkAdr-1;
        dmaData.dmaCount=dmaData.dmaEndByte-dmaData.dmaStartByte+1;       // размер передаваемого буфера
        dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера

        dmaData.stDmaBusy = 1;    //DMA занят

        configUsart1TXDMA( dmaData.dmaAdr,(dmaData.dmaCount));
        dmaData.stDmainit =3;
        dmaData.allDMA+=dmaData.dmaCount;
        return;

      }
      else //при переполнении буфера (cur впереди h)
      {
        if (dmaData.dmaEndByte==BufSize-1)//99
        {
          dmaData.dmaStartByte =  0;
          dmaData.dmaEndByte =dmaData.curWorkAdr-1;
          dmaData.dmaCount= dmaData.dmaEndByte-dmaData.dmaStartByte+1;       // размер передаваемого буфера
          dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера
        }
        else
        {
          dmaData.dmaStartByte =  dmaData.dmaEndByte+1;
          dmaData.dmaEndByte =BufSize-1;
          dmaData.dmaCount= dmaData.dmaEndByte - dmaData.dmaStartByte+1;       // размер передаваемого буфера
          dmaData.dmaAdr=Buffer+dmaData.dmaStartByte; // адрес передаваемого буфера
        }
        dmaData.stDmaBusy = 1;    //DMA занят
        dmaData.allDMA+=dmaData.dmaCount;
        configUsart1TXDMA( dmaData.dmaAdr,(dmaData.dmaCount));
        dmaData.stDmainit =4;
        return;
      }
    }
    else
    {
      dmaData.stDmaBusy =0;
     return ;
    }
  }
  dmaData.stDmaBusy =0;
  return;
}

////////////////////////////////////////////////////////////////////////////////
//_________________________________USART______________________________________//
////////////////////////////////////////////////////////////////////////////////



void DMA2_Stream2_IRQHandler(void)
{
  uint16_t checkSum;
  DMA2_Stream2->CR &=~ DMA_SxCR_EN;
  DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
  USART1->CR1 |= USART_CR1_RXNEIE;
  checkSum  = packetCheck((char * )&encData,sizeof(encData)-2);
  if (checkSum == encData.checkSum )  //Проверка CRC принятого пакета
  {
    robotCoord[0] = encData.robotCoord[0];
    robotCoord[1] = encData.robotCoord[1];
    robotCoord[2] = encData.robotCoord[2];
    robotSpeed[0] = encData.robotSpeed[1];
    robotSpeed[1] = encData.robotSpeed[0];
    robotSpeed[2] = encData.robotSpeed[2];

  }

}

void USART1_IRQHandler(void)
{
     char state;
	 state = USART1->SR ;
     USART1->SR =0;//&=~USART_SR_RXNE;
     if (state & USART_SR_RXNE || state & USART_SR_ORE)  //получен байт
     {
         usart1Data[1] = USART1->DR;
         pushByte(usart1Data[1]);
         state = USART1->SR;
         if (state & USART_SR_ORE)
         {
              usart1Data[1] = USART1->DR;
              pushByte(usart1Data[1]);
              state = USART1->SR;
         }
         if ((usart1Data[0] == 0xAA) && (usart1Data[1] == 0x01)) //проверка начала пакета
         {
             encData.adress = usart1Data[1];
             configUsart1RXDMA(((char *)&encData)+1, sizeof(encData)-1); // запуск DMA для приема основной части пакета
             USART1->CR1&=~USART_CR1_RXNEIE;
             USART1->CR3|=USART_CR3_DMAR;
        }
        else usart1Data[0] = usart1Data[1];
    }
}
