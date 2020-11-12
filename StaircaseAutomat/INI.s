
;********************************************************************************
;* JM�NO SOUBORU	: INI.S
;* AUTOR			: Michal TOM��
;* DATUM			: 03/2009
;* POPIS			: soubor s pojmenovanymi adresami registru
;********************************************************************************

;Vojtech Michal added definitions for SPI, DMA, FLASH drivers and modified USART definitions

	AREA    STM32F10x_INI, CODE, READONLY    


GPIOA_BASE          EQU     0x40010800
GPIOB_BASE          EQU     0x40010C00    
GPIOC_BASE          EQU     0x40011000
GPIOD_BASE          EQU     0x40011400
GPIOE_BASE          EQU     0x40011800
    
GPIO_CRL_o EQU 0x0
GPIO_CRH_o EQU 0x4
GPIO_IDR_o EQU 0x8
GPIO_ODR_o EQU 0xc
GPIO_BSRR_o EQU 0x10
GPIO_BRR_o EQU 0x14
    
SPI2_BASE           EQU     0x40003800
    
SPI_CR1_o EQU 0x0
SPI_CR2_o EQU 0x4
SPI_SR_o EQU 0x8
SPI_DR_o EQU 0xc
    
USART1_BASE EQU 0x40013800

USART_SR_o EQU 0x0
USART_DR_o EQU 0x4
USART_BRR_o EQU 0x8
USART_CR1_o EQU 0xc
USART_CR2_o EQU 0x10
USART_CR3_o EQU 0x14
USART_GTPR_o EQU 0x18

USART_SR_RXNE EQU 1 :SHL: 5
    
DMA1_BASE EQU 0x40020000
    
DMA_ISR_o EQU 0x0
DMA_IFCR_o EQU 0x4
    
DMA_CCR4_o EQU 0x44
DMA_CNDTR4_o EQU 0x48
DMA_CPAR4_o EQU 0x4c
DMA_CMAR4_o EQU 0x50
    
FLASH_BASE EQU 0x40022000
    
FLASH_ACR_o EQU 0x0
FLASH_KEYR_o EQU 0x4
FLASH_SR_o EQU 0xc
FLASH_CR_o EQU 0x10    
FLASH_AR_o EQU 0x14    

;********************************************************************************
;*   				GPIOA				 	  	*
;********************************************************************************
;GPIOA Registers 
;-------------------------------------------------------------------------------- 

GPIOA_CRL			EQU		GPIOA_BASE + GPIO_CRL_o
;--------------------------------------------------------------------------------
GPIOA_CRH	 		EQU		GPIOA_BASE + GPIO_CRH_o
;--------------------------------------------------------------------------------
GPIOA_IDR	 		EQU		GPIOA_BASE + GPIO_IDR_o ; input data regist
;GPIOA_IDR	bits
AIDR15				EQU		0x4221013C; 15
AIDR14				EQU		0x42210138; 14
AIDR13				EQU		0x42210134; 13
AIDR12				EQU		0x42210130; 12
AIDR11				EQU		0x4221012C; 11
AIDR10				EQU		0x42210128; 10
AIDR9				EQU		0x42210124; 9
AIDR8				EQU		0x42210120; 8
AIDR7				EQU		0x4221011C; 7
AIDR6				EQU		0x42210118; 6
AIDR5				EQU		0x42210114; 5
AIDR4				EQU		0x42210110; 4
AIDR3				EQU		0x4221010C; 3
AIDR2				EQU		0x42210108; 2
AIDR1				EQU		0x42210104; 1
AIDR0				EQU		0x42210100; 0

;--------------------------------------------------------------------------------
GPIOA_ODR	 		EQU		0x4001080C ; output data registr
;GPIOA_ODR	bits
AODR15				EQU		0x422101BC; 15
AODR14				EQU		0x422101B8; 14
AODR13				EQU		0x422101B4; 13
AODR12				EQU		0x422101B0; 12
AODR11				EQU		0x422101AC; 11
AODR10				EQU		0x422101A8; 10
AODR9				EQU		0x422101A4; 9
AODR8				EQU		0x422101A0; 8
AODR7				EQU		0x4221019C; 7
AODR6				EQU		0x42210198; 6
AODR5				EQU		0x42210194; 5
AODR4				EQU		0x42210190; 4
AODR3				EQU		0x4221018C; 3
AODR2				EQU		0x42210188; 2
AODR1				EQU		0x42210184; 1
AODR0				EQU		0x42210180; 0

;--------------------------------------------------------------------------------
GPIOA_BSRR	 		EQU		0x40010810
;--------------------------------------------------------------------------------
GPIOA_BRR	 		EQU		0x40010814
;--------------------------------------------------------------------------------
GPIOA_LCKR	 		EQU		0x40010818
;--------------------------------------------------------------------------------

;********************************************************************************
;*   				GPIOC				 	  	*
;********************************************************************************
;GPIOC Registers 
;--------------------------------------------------------------------------------
GPIOC_CRL			EQU		GPIOC_BASE + GPIO_CRL_o
;--------------------------------------------------------------------------------
GPIOC_CRH	 		EQU		GPIOC_BASE + GPIO_CRH_o
;--------------------------------------------------------------------------------
GPIOC_IDR	 		EQU		GPIOC_BASE + GPIO_IDR_o; input data regist
;GPIOC_IDR	bits
CIDR15				EQU		0x4222013C; 15
CIDR14				EQU		0x42220138; 14
CIDR13				EQU		0x42220134; 13
CIDR12				EQU		0x42220130; 12
CIDR11				EQU		0x4222012C; 11
CIDR10				EQU		0x42220128; 10
CIDR9				EQU		0x42220124; 9
CIDR8				EQU		0x42220120; 8
CIDR7				EQU		0x4222011C; 7
CIDR6				EQU		0x42220118; 6
CIDR5				EQU		0x42220114; 5
CIDR4				EQU		0x42220110; 4
CIDR3				EQU		0x4222010C; 3
CIDR2				EQU		0x42220108; 2
CIDR1				EQU		0x42220104; 1
CIDR0				EQU		0x42220100; 0

;--------------------------------------------------------------------------------
GPIOC_ODR	 		EQU		0x4001100C ; output data registr
;GPIOC_ODR	bits
CODR15				EQU		0x422201BC; 15
CODR14				EQU		0x422201B8; 14
CODR13				EQU		0x422201B4; 13
CODR12				EQU		0x422201B0; 12
CODR11				EQU		0x422201AC; 11
CODR10				EQU		0x422201A8; 10
CODR9				EQU		0x422201A4; 9
CODR8				EQU		0x422201A0; 8
CODR7				EQU		0x4222019C; 7
CODR6				EQU		0x42220198; 6
CODR5				EQU		0x42220194; 5
CODR4				EQU		0x42220190; 4
CODR3				EQU		0x4222018C; 3
CODR2				EQU		0x42220188; 2
CODR1				EQU		0x42220184; 1
CODR0				EQU		0x42220180; 0
;--------------------------------------------------------------------------------
GPIOC_BSRR	 		EQU		0x40011010
;--------------------------------------------------------------------------------
GPIOC_BRR	 		EQU		0x40011014
;--------------------------------------------------------------------------------
GPIOC_LCKR	 		EQU		0x40011018
;--------------------------------------------------------------------------------


;********************************************************************************
;*   				GPIOB				 	  	*
;********************************************************************************
;GPIOB Registers 
;--------------------------------------------------------------------------------
GPIOB_CRL			EQU		0x40010C00
;--------------------------------------------------------------------------------
GPIOB_CRH	 		EQU		0x40010C04
;--------------------------------------------------------------------------------
GPIOB_IDR	 		EQU		0x40010C08 ; input data regist
;GPIOB_IDR	bits
BIDR15				EQU		0x4221813C; 15
BIDR14				EQU		0x42218138; 14
BIDR13				EQU		0x42218134; 13
BIDR12				EQU		0x42218130; 12
BIDR11				EQU		0x4221812C; 11
BIDR10				EQU		0x42218128; 10
BIDR9				EQU		0x42218124; 9
BIDR8				EQU		0x42218120; 8
BIDR7				EQU		0x4221811C; 7
BIDR6				EQU		0x42218118; 6
BIDR5				EQU		0x42218114; 5
BIDR4				EQU		0x42218110; 4
BIDR3				EQU		0x4221810C; 3
BIDR2				EQU		0x42218108; 2
BIDR1				EQU		0x42218104; 1
BIDR0				EQU		0x42218100; 0

;--------------------------------------------------------------------------------
GPIOB_ODR	 		EQU		0x40010C0C ; output data registr
;GPIOB_ODR	bits
BODR15				EQU		0x422181BC; 15
BODR14				EQU		0x422181B8; 14
BODR13				EQU		0x422181B4; 13
BODR12				EQU		0x422181B0; 12
BODR11				EQU		0x422181AC; 11
BODR10				EQU		0x422181A8; 10
BODR9				EQU		0x422181A4; 9
BODR8				EQU		0x422181A0; 8
BODR7				EQU		0x4221819C; 7
BODR6				EQU		0x42218198; 6
BODR5				EQU		0x42218194; 5
BODR4				EQU		0x42218190; 4
BODR3				EQU		0x4221818C; 3
BODR2				EQU		0x42218188; 2
BODR1				EQU		0x42218184; 1
BODR0				EQU		0x42218180; 0
;--------------------------------------------------------------------------------
GPIOB_BSRR	 		EQU		0x40010C10
;--------------------------------------------------------------------------------
GPIOB_BRR	 		EQU		0x40010C14
;--------------------------------------------------------------------------------
GPIOB_LCKR	 		EQU		0x40010C18
;--------------------------------------------------------------------------------


;********************************************************************************
;*   				RCC				 	  	*
;********************************************************************************
;RCC Registers   
;--------------------------------------------------------------------------------
RCC_CR		 		EQU		0x40021000; clock control registr
;RCC_CR BIT
PLLRDY				EQU		0x42420064; 25
PLLON				EQU		0x42420060; 24
CSSON				EQU		0x4242004C; 19
HSEBYP				EQU		0x42420048; 18
HSERDY				EQU		0x42420044; 17
HSEON				EQU		0x42420040; 16
HSICAL				EQU		0x42420020; 8
HSITRIM				EQU		0x4242000C; 3
HSIRDY				EQU		0x42420004; 1
HSION				EQU		0x42420000; 0
;--------------------------------------------------------------------------------
RCC_CFGR			EQU		0x40021004; clock configuration registr
;RCC_CFGR	bits
MCO			 		EQU		0x424200E0; 24
USBPRE				EQU		0x424200D8; 22
PLLMUL			 	EQU		0x424200C8; 18
PLLXTPRE		 	EQU		0x424200C4; 17
PLLSRC			 	EQU		0x424200C0; 16
ADCPRE				EQU		0x424200B8; 14
PPRE2				EQU		0x424200AC; 11
PPRE1				EQU		0x424200A0; 8
HPRE				EQU		0x42420090; 4
SWS				 	EQU		0x42420088; 2
SW			 		EQU		0x42420080; 0
;--------------------------------------------------------------------------------
RCC_CIR				EQU		0x40021008
;RCC_CIR	 bits

LSIRDYIE			EQU		0x42420120; 8
CSSF				EQU		0x4242011C; 7
PLLRDYF			 	EQU		0x42420110; 4
HSERDYF			 	EQU		0x4242010C; 3
HSIRDYF			 	EQU		0x42420108; 2
LSERDYF			 	EQU		0x42420104; 1
LSIRDYF				EQU		0x42420100; 0
;--------------------------------------------------------------------------------
RCC_APB2RSTR			EQU		0x4002100C
;--------------------------------------------------------------------------------
RCC_APB1RSTR 			EQU		0x40021010
;--------------------------------------------------------------------------------
RCC_AHBENR 			EQU		0x40021014
;--------------------------------------------------------------------------------
RCC_APB2ENR			EQU		0x40021018
;RCC_APB2ENR	bits
ADC3EN				EQU		0x4242033C; 15
USRT1EN				EQU		0x42420338; 14
TIM8EN				EQU		0x42420334; 13
SPI1EN				EQU		0x42420330; 12
TIM1EN				EQU		0x4242032C; 11
ADC2EN				EQU		0x42420328; 10
ADC1EN				EQU		0x42420324; 9
IOPGEN				EQU		0x42420320; 8
IOPFEN				EQU		0x4242031C; 7
IOPEEN				EQU		0x42420318; 6
IOPDEN				EQU		0x42420314; 5
IOPCEN				EQU		0x42420310; 4
IOPBEN				EQU		0x4242030C; 3
IOPAEN				EQU		0x42420308; 2
AFIOEN				EQU		0x42420300; 0
;--------------------------------------------------------------------------------
RCC_APB1ENR  		EQU		0x4002101C
;RCC_APB1ENR	bits
DACEN				EQU		0x424203F4; 29
PWREN				EQU		0x424203EC; 28
BKPEN				EQU		0x424203E8; 27
CANEN				EQU		0x424203E4; 25
USBEN				EQU		0x424203DC; 23
I2C2EN				EQU		0x424203D8; 22
I2C1EN				EQU		0x424203D4; 21
USART5EN			EQU		0x424203D0; 20
USART4EN			EQU		0x424203CC; 19
USART3EN			EQU		0x424203C8; 18
USART2EN			EQU		0x424203C4; 17
SPI3EN				EQU		0x424203BC; 15
SPI2EN				EQU		0x424203B8; 14
WWDGEN				EQU		0x424203AC; 11
TIM7EN				EQU		0x42420394; 5
TIM6EN				EQU		0x42420390; 4
TIM5EN				EQU		0x4242038C; 3
TIM4EN				EQU		0x42420388; 2
TIM3EN				EQU		0x42420384; 1
TIM2EN				EQU		0x42420380; 0
;--------------------------------------------------------------------------------
RCC_BDCR	 		EQU		0x40021020
;--------------------------------------------------------------------------------
RCC_CSR 		 	EQU		0x40021024
;--------------------------------------------------------------------------------



;********************************************************************************
;*   				AFIO				 	  	*
;********************************************************************************
;AFIO Registers   
;--------------------------------------------------------------------------------
AFIO_MAPR			EQU 	0x40010004
;AFIO_MAPR	bits
SWJ_CFG				EQU		0x422000E0; 24
ADC2_ETRGREG_REMAP	EQU		0x422000D0; 20
ADC2_ETRGINJ_REMAP	EQU		0x422000CC; 19
ADC1_ETRGREG_REMAP	EQU		0x422000C8; 18
ADC1_ETRGINJ_REMAP	EQU		0x422000C4; 17
TIM5CH4_IREMAP		EQU		0x422000C0; 16
PD01_REMAP			EQU		0x422000BC; 15
CAN_REMAP			EQU		0x422000B4; 13
TIM4_REMAP			EQU		0x422000B0; 12
TIM3_REMAP			EQU		0x422000A8; 10
TIM2_REMAP			EQU		0x422000A0; 8
TIM1_REMAP			EQU		0x42200098; 6
USART3_REMAP		EQU		0x42200090; 4
USART2_REMAP		EQU		0x4220008C; 3
USART1_REMAP		EQU		0x42200088; 2
I2C1_REMAP			EQU		0x42200084; 1
SPI1_REMAP			EQU		0x42200080; 0
;--------------------------------------------------------------------------------
AFIO_EXTICR1		EQU		0x40010008; External Interrupt Configuration Registr 1
AFIO_EXTICR2		EQU		0x4001000C; External Interrupt Configuration Registr 2
AFIO_EXTICR3		EQU		0x40010010; External Interrupt Configuration Registr 3
AFIO_EXTICR4		EQU		0x40010014; External Interrupt Configuration Registr 3



;********************************************************************************
;*   				USART				 	  	*
;********************************************************************************
;USART Registers   
;--------------------------------------------------------------------------------

USART_SR			EQU		0x40013800;status registr
;--------------------------------------------------------------------------------
USART_DR			EQU		0x40013804;data registr
;--------------------------------------------------------------------------------
USART_BRR			EQU		0x40013808;baud rate registr
;--------------------------------------------------------------------------------
USART_CR1			EQU		0x4001380C;control registr 1
;--------------------------------------------------------------------------------
USART_CR2			EQU		0x40013810;control registr 2
;--------------------------------------------------------------------------------
USART_CR3			EQU		0x40013814;control registr 3
;--------------------------------------------------------------------------------
USART_GTPR			EQU		0x40013818;guard time and predscale registr (hlidac a predelicka)		
;--------------------------------------------------------------------------------




;********************************************************************************
;*   				FLASH				 	  	*
;********************************************************************************
;FLASH Registers   
;--------------------------------------------------------------------------------
FLASH_ACR			EQU		0x40022000;flash access control registr
;--------------------------------------------------------------------------------


;********************************************************************************
;*   				TIM1				 	  	*
;********************************************************************************
;TIM1 Registers   
;--------------------------------------------------------------------------------
TIM1_CR1			EQU		0x40012C00; CONTROL 1 
;TIM1_CR1 	bits
DIR_T1				EQU		0x42258010; 4 bit DIRECTION
OPM_T1				EQU		0x4225800C; 3 bit ONE PULSE MODE
URS_T1				EQU		0x42258008; 2 bit Update Request Source
UDIS_T1				EQU		0x42258004; 1 bit Update Disable
CEN_T1				EQU		0x42258000; 0 bit Counter Enable
;-------------------------------------------------------------------------------
TIM1_CR2			EQU		0x40012C04; CONTROL 2
;--------------------------------------------------------------------------------
TIM1_SMCR			EQU		0x40012C08; SLAVE MODE CONTROL  
;--------------------------------------------------------------------------------
TIM1_DIER			EQU		0x40012C0C;  
;TIM1_DIER	bits
TIE_T1				EQU		0x42258198; 6 bit Trigger Interrupt Enable
CC4IE_T1			EQU		0x42258184; 4 bit Capture/Compare Interrupt Enable
UIE_T1				EQU		0x42258180; 0 bit Update Interrupt Enable
;--------------------------------------------------------------------------------
TIM1_SR				EQU		0x40012C10; STATUS 
;--------------------------------------------------------------------------------
TIM1_EGR			EQU		0x40012C14; EVENT GENERATION 
;--------------------------------------------------------------------------------
TIM1_CCMR1			EQU		0x40012C18;  
;--------------------------------------------------------------------------------
TIM1_CCMR2			EQU		0x40012C1C;
;--------------------------------------------------------------------------------
TIM1_CCR			EQU		0x40012C20; CAPTURE/COMPARE ENABLE
;--------------------------------------------------------------------------------
TIM1_CNT			EQU		0x40012C24; COUNTER
;--------------------------------------------------------------------------------
TIM1_PSC			EQU		0x40012C28; PRESCALER
;--------------------------------------------------------------------------------
TIM1_ARR			EQU		0x40012C2C; AUTO RELOAD
;--------------------------------------------------------------------------------
TIM1_RCR			EQU		0x40012C30; REPETITION
;--------------------------------------------------------------------------------
TIM1_CCR1			EQU		0x40012C34; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM1_CCR2			EQU		0x40012C38; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM1_CCR3			EQU		0x40012C3C; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM1_CCR4			EQU		0x40012C40; COMPARE/CAPTURE		 
;--------------------------------------------------------------------------------
TIM1_BDTR			EQU		0x40012C44; BREAK AND DEAD-TIME
;--------------------------------------------------------------------------------
TIM1_DCR			EQU		0x40012C48; DMA CONTROL
;--------------------------------------------------------------------------------
TIM1_DMAR			EQU		0x40012C4C; DMA ADDRESS FOR FULL TRANSFER



;********************************************************************************
;*   				TIM2				 	  	*
;********************************************************************************
;TIM2 Registers   
;--------------------------------------------------------------------------------
TIM2_CR1			EQU		0x40000000; CONTROL 1 
;TIM2_CR1 	bits
DIR_T2				EQU		0x42000010; 4 bit DIRECTION
OPM_T2				EQU		0x4200000C; 3 bit ONE PULSE MODE
URS_T2				EQU		0x42000008; 2 bit Update Request Source
UDIS_T2				EQU		0x42000004; 1 bit Update Disable
CEN_T2				EQU		0x42000000; 0 bit Counter Enable
;--------------------------------------------------------------------------------
TIM2_CR2			EQU		0x40000004; CONTROL 2
;--------------------------------------------------------------------------------
TIM2_SMCR			EQU		0x40000008; SLAVE MODE CONTROL
;--------------------------------------------------------------------------------
TIM2_DIER			EQU		0x4000000C; 
;TIM2_DIER	bits
TIE_T2				EQU		0x42000198; 6 bit Trigger Interrupt Enable
CC4IE_T2			EQU		0x42000190; 4 bit Capture/Compare Interrupt Enable
CC3IE_T2			EQU		0x4200018C; 3 bit Capture/Compare Interrupt Enable
CC2IE_T2			EQU		0x42000188; 2 bit Capture/Compare Interrupt Enable
CC1IE_T2			EQU		0x42000184; 1 bit Capture/Compare Interrupt Enable
UIE_T2				EQU		0x42000180; 0 bit Update Interrupt Enable
;--------------------------------------------------------------------------------
TIM2_SR				EQU		0x40000010; STATUS
;--------------------------------------------------------------------------------
TIM2_EGR			EQU		0x40000014; EVENT GENERATION
;-------------------------------------------------------------------------------- 
TIM2_CCMR1			EQU		0x40000018;  
;--------------------------------------------------------------------------------
TIM2_CCMR2			EQU		0x4000001C;
;--------------------------------------------------------------------------------
TIM2_CCER			EQU		0x40000020; CAPTURE/COMPARE ENABLE
;--------------------------------------------------------------------------------
TIM2_CNT			EQU		0x40000024; COUNTER
;--------------------------------------------------------------------------------
TIM2_PSC			EQU		0x40000028; PRESCALER
;--------------------------------------------------------------------------------
TIM2_ARR			EQU		0x4000002C; AUTO RELOAD
;--------------------------------------------------------------------------------
TIM2_RCR			EQU		0x40000030; REPETITION
;--------------------------------------------------------------------------------
TIM2_CCR1			EQU		0x40000034; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM2_CCR2			EQU		0x40000038; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM2_CCR3			EQU		0x4000003C; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM2_CCR4			EQU		0x40000040; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM2_BDTR			EQU		0x40000044; BREAK AND DEAD-TIME
;--------------------------------------------------------------------------------
TIM2_DCR			EQU		0x40000048; DMA CONTROL
;--------------------------------------------------------------------------------
TIM2_DMAR			EQU		0x4000004C; DMA ADDRESS FOR FULL TRANSFER
;--------------------------------------------------------------------------------



;********************************************************************************
;*   				TIM3				 	  	*
;********************************************************************************
;TIM3 Registers   
;--------------------------------------------------------------------------------
TIM3_CR1			EQU		0x40000400; CONTROL 1 
;TIM3_CR1	bits
DIR_T3				EQU		0x42008010; 4 bit DIRECTION
OPM_T3				EQU		0x4200800C; 3 bit ONE PULSE MODE
URS_T3				EQU		0x42008008; 2 bit Update Request Source
UDIS_T3				EQU		0x42008004; 1 bit Update Disable
CEN_T3				EQU		0x42008000; 0 bit Counter Enable
;--------------------------------------------------------------------------------
TIM3_CR2			EQU		0x40000404; CONTROL 2
;--------------------------------------------------------------------------------
TIM3_SMCR			EQU		0x40000408; SLAVE MODE CONTROL
;--------------------------------------------------------------------------------
TIM3_DIER			EQU		0x4000040C; 
;TIM3_DIER	bits
TIE_T3				EQU		0x42008198; 6 bit Trigger Interrupt Enable
CC4IE_T3			EQU		0x42008190; 4 bit Capture/Compare Interrupt Enable
CC3IE_T3			EQU		0x4200018C; 3 bit Capture/Compare Interrupt Enable
CC2IE_T3			EQU		0x42000188; 2 bit Capture/Compare Interrupt Enable
CC1IE_T3			EQU		0x42000184; 1 bit Capture/Compare Interrupt Enable
UIE_T3				EQU		0x42008180; 0 bit Update Interrupt Enable
;--------------------------------------------------------------------------------
TIM3_SR				EQU		0x40000410; STATUS
;--------------------------------------------------------------------------------
TIM3_EGR			EQU		0x40000414; EVENT GENERATION
;--------------------------------------------------------------------------------
TIM3_CCMR1			EQU		0x40000418;  
;--------------------------------------------------------------------------------
TIM3_CCMR2			EQU		0x4000041C;
;--------------------------------------------------------------------------------
TIM3_CCER			EQU		0x40000420; CAPTURE/COMPARE ENABLE
;--------------------------------------------------------------------------------
TIM3_CNT			EQU		0x40000424; COUNTER
;--------------------------------------------------------------------------------
TIM3_PSC			EQU		0x40000428; PRESCALER
;--------------------------------------------------------------------------------
TIM3_ARR			EQU		0x4000042C; AUTO RELOAD
;--------------------------------------------------------------------------------
TIM3_RCR			EQU		0x40000430; REPETITION
;--------------------------------------------------------------------------------
TIM3_CCR1			EQU		0x40000434; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM3_CCR2			EQU		0x40000438; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM3_CCR3			EQU		0x4000043C; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM3_CCR4			EQU		0x40000440; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM3_BDTR			EQU		0x40000444; BREAK AND DEAD-TIME
;--------------------------------------------------------------------------------
TIM3_DCR			EQU		0x40000448; DMA CONTROL
;--------------------------------------------------------------------------------
TIM3_DMAR			EQU		0x4000044C; DMA ADDRESS FOR FULL TRANSFER
;--------------------------------------------------------------------------------

;********************************************************************************
;*   				TIM4				 	  	*
;********************************************************************************
;TIM4 Registers   
;--------------------------------------------------------------------------------
TIM4_CR1			EQU		0x40000800; CONTROL 1 
;TIM4_CR1	bits
DIR_T4				EQU		0x42010010; 4 bit DIRECTION
OPM_T4				EQU		0x4201000C; 3 bit ONE PULSE MODE
URS_T4				EQU		0x42010008; 2 bit Update Request Source
UDIS_T4				EQU		0x42010004; 1 bit Update Disable
CEN_T4				EQU		0x42010000; 0 bit Counter Enable
;--------------------------------------------------------------------------------
TIM4_CR2			EQU		0x40000804; CONTROL 2
;--------------------------------------------------------------------------------
TIM4_SMCR			EQU		0x40000808; SLAVE MODE CONTROL
;--------------------------------------------------------------------------------
TIM4_DIER			EQU		0x4000080C; 
;TIM4_DIER	bits
TIE_T4				EQU		0x42010198; 6 bit Trigger Interrupt Enable
CC4IE_T4			EQU		0x42010190; 4 bit Capture/Compare Interrupt Enable
CC3IE_T4			EQU		0x4201018C; 3 bit Capture/Compare Interrupt Enable
CC2IE_T4			EQU		0x42010188; 2 bit Capture/Compare Interrupt Enable
CC1IE_T4			EQU		0x42010184; 1 bit Capture/Compare Interrupt Enable
UIE_T4				EQU		0x42010180; 0 bit Update Interrupt Enable
;--------------------------------------------------------------------------------
TIM4_SR				EQU		0x40000810; STATUS
;--------------------------------------------------------------------------------
TIM4_EGR			EQU		0x40000814; EVENT GENERATION
;--------------------------------------------------------------------------------
TIM4_CCMR1			EQU		0x40000818;  
;--------------------------------------------------------------------------------
TIM4_CCMR2			EQU		0x4000081C;
;--------------------------------------------------------------------------------
TIM4_CCER			EQU		0x40000820; CAPTURE/COMPARE ENABLE
;--------------------------------------------------------------------------------
TIM4_CNT			EQU		0x40000824; COUNTER
;--------------------------------------------------------------------------------
TIM4_PSC			EQU		0x40000828; PRESCALER
;--------------------------------------------------------------------------------
TIM4_ARR			EQU		0x4000082C; AUTO RELOAD
;--------------------------------------------------------------------------------
TIM4_RCR			EQU		0x40000830; REPETITION
;--------------------------------------------------------------------------------
TIM4_CCR1			EQU		0x40000834; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM4_CCR2			EQU		0x40000838; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM4_CCR3			EQU		0x4000083C; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM4_CCR4			EQU		0x40000840; COMPARE/CAPTURE
;--------------------------------------------------------------------------------
TIM4_BDTR			EQU		0x40000844; BREAK AND DEAD-TIME
;--------------------------------------------------------------------------------
TIM4_DCR			EQU		0x40000848; DMA CONTROL
;--------------------------------------------------------------------------------
TIM4_DMAR			EQU		0x4000084C; DMA ADDRESS FOR FULL TRANSFER
;--------------------------------------------------------------------------------


;********************************************************************************
;*   				EXTI				 	  	*
;********************************************************************************
;EXTI Registers   
;--------------------------------------------------------------------------------
EXTI_IMR			EQU		0x40010400;
;--------------------------------------------------------------------------------
EXTI_EMR			EQU		0x40010404
;--------------------------------------------------------------------------------
EXTI_RTSR			EQU		0x40010408
;--------------------------------------------------------------------------------
EXTI_FTSR			EQU		0x4001040C
;--------------------------------------------------------------------------------
EXTI_SWIER			EQU		0x40010410
;--------------------------------------------------------------------------------
EXTI_PR				EQU		0x40010414
;--------------------------------------------------------------------------------



;********************************************************************************
;*   				NVIC				 	  	*
;********************************************************************************
;NVIC Registers   
;--------------------------------------------------------------------------------
NVIC_ICSR			EQU		0xE000ED04; Interrupt Control State Register
;--------------------------------------------------------------------------------
NVIC_VTOR			EQU		0xE000ED08; Vector Table Offset Register		  
;--------------------------------------------------------------------------------
NVIC_AIRCR			EQU		0xE000ED0C; Application Interrupt and Reset Control Register
;--------------------------------------------------------------------------------
NVIC_ISER			EQU		0xE000E100; Interrupt Set-Enable Register
;--------------------------------------------------------------------------------
NVIC_ICER			EQU		0xE000E180; Interrupt Clear-Enable Register
;--------------------------------------------------------------------------------
NVIC_ISPR			EQU		0xE000E200; Interrupt Set-Pending Register
;--------------------------------------------------------------------------------
NVIC_ICPR			EQU		0xE000E280; Interrupt Clear-Pending Register
;--------------------------------------------------------------------------------
NVIC_IPR			EQU		0xE000E400; Interrupt Priority Register
;--------------------------------------------------------------------------------


;********************************************************************************
;*   				ADC				 	  	*
;********************************************************************************
;ADC1 Registers   
;--------------------------------------------------------------------------------
ADC_SR				EQU		0x40012400;
;--------------------------------------------------------------------------------
ADC_CR1				EQU		0x40012404;
;--------------------------------------------------------------------------------
ADC_CR2				EQU		0x40012408;
;--------------------------------------------------------------------------------
ADC_SMPR1			EQU		0x4001240C;
;--------------------------------------------------------------------------------
ADC_SMPR2			EQU		0x40012410;
;--------------------------------------------------------------------------------
ADC_JOFR1			EQU		0x40012414;
;--------------------------------------------------------------------------------
ADC_JOFR2			EQU		0x40012418;
;--------------------------------------------------------------------------------
ADC_JOFR3			EQU		0x4001241C;
;--------------------------------------------------------------------------------
ADC_JOFR4			EQU		0x40012420;
;--------------------------------------------------------------------------------
ADC_HTR				EQU		0x40012424;
;--------------------------------------------------------------------------------
ADC_LTR				EQU		0x40012428;
;--------------------------------------------------------------------------------
ADC_SQR1			EQU		0x4001242C;
;--------------------------------------------------------------------------------
ADC_SQR2			EQU		0x40012430;
;--------------------------------------------------------------------------------
ADC_SQR3			EQU		0x40012434;
;--------------------------------------------------------------------------------
ADC_JSQR			EQU		0x40012438;
;--------------------------------------------------------------------------------
ADC_JDR1			EQU		0x4001243C;
;--------------------------------------------------------------------------------
ADC_JDR2			EQU		0x40012440;
;--------------------------------------------------------------------------------
ADC_JDR3			EQU		0x40012444;
;--------------------------------------------------------------------------------
ADC_JDR4			EQU		0x40012448;
;--------------------------------------------------------------------------------
ADC_DR				EQU		0x4001244C;
;--------------------------------------------------------------------------------

	END