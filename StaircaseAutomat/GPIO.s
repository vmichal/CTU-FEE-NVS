;***************************************************************************************************
;*
;* Misto			: CVUT FEL, Katedra Mereni
;* Prednasejici		: Doc. Ing. Jan Fischer,CSc.
;* Predmet			: A4M38AVS
;* Vyvojovy Kit		: STM32 VL DISCOVERY (STM32F100RB)
;*
;**************************************************************************************************
;*
;* JMÉNO SOUBORU	: LED_TLC.ASM
;* AUTOR			: Vojtech Michal
;* DATUM			: 12/2010
;* POPIS			: Program pro stridave blikani LED na vyvodech PC8 a PC9 se dvema mody ovladanymi tlacitkem.
;*					  - konfigurace hodin na frekvenci 24MHz (HSE + PLL) 
;*					  - konfigurace pouzitych vyvodu procesotu (PC8 a PC9 jako vystup, PA0 jako vstup)
;*					  - rozblikani LED na PC8 a PC9, cteni stavu tlacitka a prepinani modu blikani
;* Poznamka			: Tento soubor obsahuje podrobny popis kodu vcetne vyznamu pouzitych instrukci
;*
;***************************************************************************************************
				
        AREA MYDATA, DATA, NOINIT, READWRITE
            
ButtonData
startPressedStart SPACE 4
startPressedCache SPACE 4
startLastValidState SPACE 4
    
    
OkPressedStart SPACE 4
OkPressedCache SPACE 4
OkLastValidState SPACE 4
    
    
PlusPressedStart SPACE 4
PlusPressedCache SPACE 4
PlusLastValidState SPACE 4
    
MinusPressedStart SPACE 4
MinusPressedCache SPACE 4
MinusLastValidState SPACE 4
                             
		AREA    GPIO_Driver, CODE, READONLY  	; hlavicka souboru
	
		GET		INI.s					; vlozeni souboru s pojmenovanymi adresami
										; jsou zde definovany adresy pristupu do pameti (k registrum)

PressedStart_o EQU 0
PressedCache_o EQU 4 
LastValidState_o EQU 8

startPort EQU GPIOA_BASE ;User button is connected to PA0
startPin EQU 0

PlusPort EQU GPIOC_BASE
PlusPin EQU 6   
MinusPort EQU GPIOC_BASE
MinusPin EQU 7
OkPort EQU GPIOA_BASE
OkPin EQU 11
    
leftPort EQU GPIOB_BASE
rightPort EQU GPIOB_BASE
leftPin EQU 8
rightPin EQU 9

ledBluePort EQU GPIOC_BASE
ledGreenPort EQU GPIOC_BASE
ledBluePin EQU 8
ledGreenPin EQU 9
    
spiClockPin EQU 13
spiDataPin EQU 15
    
uartPort EQU GPIOA_BASE
uartTxPin EQU 9
uartRxPin EQU 10    
    
debounceDelay EQU 80 ; in ms

BTNstart EQU 0
BTNok EQU 1    
BTNplus EQU 2   
BTNminus EQU 3
    
ButtonPorts
    DCD startPort
    DCD OkPort
    DCD PlusPort
    DCD MinusPort
        
ButtonPins
    DCD startPin
    DCD OkPin
    DCD PlusPin
    DCD MinusPin

    EXPORT GPIO_CNF      
    EXPORT buttonPressedFiltered
    EXPORT buttonSample
    EXPORT ledBlueOn        
    EXPORT ledBlueOff
    EXPORT ledGreenOn
    EXPORT ledGreenOff
    import GetTick
    import TimeElapsed
    export leftOn
    export rightOn
        
;**************************************************************************************************
;* Jmeno funkce		: GPIO_CNF
;* Popis			: Konfigurace brany A a C
;* Vstup			: Zadny
;* Vystup			: Zadny
;* Komentar			: Nastaveni PC08 a PC09 jako vystup (10MHz), PA0 jako vstup push-pull	
;**************************************************************************************************
GPIO_CNF								; Navesti zacatku podprogramu
    push {lr}
                LDR		R2, =0xFF		; Konstanta pro nulovani nastaveni bitu 8, 9	
				LDR		R0, =GPIOC_CRH	; Kopie adresy GPIOC_CRH (Port Configuration Register High)
										; do R0, GPIOC_CRH je v souboru INI.S	
				LDR		R1, [R0]		; Nacteni hodnoty z adresy v R0 do R1 
				BIC		R1, R1, R2 		; Nulovani bitu v R2 
				MOV		R2, #0x11		; Vlozeni 1 do R2
				ORR		R1, R1, R2		; maskovani, bit 8, 9 nastven jako vystup push-pull v modu 1 (10MHz)
				STR		R1, [R0]		; Ulozeni konfigurace PCO9 a PC09

				LDR		R2, =0xF		; Konstanta pro nulovani nastaveni bitu 0	
				LDR		R0, =GPIOA_CRL	; Kopie adresy GPIOA_CRL (Port Configuration Register Low)
										; do R0, GPIOA_CRL je v souboru INI.S	
				LDR		R1, [R0]		; Nacteni hodnoty z adresy v R0 do R1 
				BIC		R1, R1, R2 		; Nulovani bitu v R2 
				MOV		R2, #0x8		; Vlozeni 1 do R2
				ORR		R1, R1, R2		; maskovani, bit 0 nastven jako push-pull vstup
				STR		R1, [R0]		; Ulozeni konfigurace PAO0
                
                ;both PB13 and PB15 must be ouput alternate funciton push-pull for SPI
                ldr r0, =GPIOB_BASE
                ldr r1, [r0, #GPIO_CRH_o]
                ;mask clearing configuration of pins for spi data and spi clock
                ldr r2, =(0xf << ((spiClockPin - 8)*4)) :OR: (0xf << ((spiDataPin - 8)*4))
                bic r1, r2
                ldr r2, =(0xd << ((spiClockPin - 8)*4)) :OR: (0xd << ((spiDataPin - 8)*4))
                orr r1, r2
                str r1, [r0, #GPIO_CRH_o]

                ;configure PA11 as input pullup/pulldown
                ldr r0, =GPIOA_BASE
                ldr r1, [r0, #GPIO_CRH_o]
                ldr r2, = 0xf << ((OkPin - 8)*4)
                bic r1, r2
                ldr r2, = 0x8 << ((OkPin - 8)*4)
                orr r1, r2
                str r1, [r0, #GPIO_CRH_o]
                ldr r1, [r0, #GPIO_ODR_o]
                orr r1, #1 :SHL: OkPin
                str r1, [r0, #GPIO_ODR_o]
                
                ;configure PC7 and PC6 as input pullup
                ldr r0, =GPIOC_BASE
                ldr r1, [r0, #GPIO_CRL_o]
                ldr r2, = 0xf << (MinusPin*4) :OR: 0xf << (PlusPin*4)
                bic r1, r2
                ldr r2, = 0x8 << (MinusPin*4) :OR: 0x8 << (PlusPin*4)
                orr r1, r2
                str r1, [r0, #GPIO_CRL_o]
                ldr r1, [r0, #GPIO_ODR_o]
                orr r1, #(1 :SHL: PlusPin) :OR: (1 :SHL: MinusPin)
                str r1, [r0, #GPIO_ODR_o]
                
                ;configure PB8, PB9 as output push-pull
                ldr r0, =GPIOB_BASE
                ldr r1, [r0, #GPIO_CRH_o]
                ldr r2, = 0xf << ((leftPin-8)*4) :OR: 0xf << ((rightPin-8)*4)
                bic r1, r2
                ldr r2, = 0x2 << ((leftPin-8)*4) :OR: 0x2 << ((rightPin-8)*4)
                orr r1, r2
                str r1, [r0, #GPIO_CRH_o]
                ldr r1, [r0, #GPIO_ODR_o] ;clear pins (turn display on)
                orr r1, #(1 :SHL: rightPin) :OR: (1 :SHL: leftPin)
                str r1, [r0, #GPIO_ODR_o]                
                
                ;configure PA9, PA10 for uaart. tx .. AF push-pull, rx .. input pullup
                ldr r0, = uartPort
                ldr r1, [r0, #GPIO_CRH_o]
                ldr r2, = 0xf << ((uartTxPin-8)*4) :OR: 0xf << ((uartRxPin-8)*4)
                bic r1, r2
                ldr r2, = 0x9 << ((uartTxPin-8)*4) :OR: 0x8 << ((uartRxPin-8)*4)
                orr r1, r2            
                str r1, [r0, #GPIO_CRH_o]
                ldr r1, [r0, #GPIO_ODR_o]
                orr r1, #(1 :SHL: uartRxPin) ;activate pull up on rx pin
                str r1, [r0, #GPIO_ODR_o]
                
                ;initialize variables
                ldr r0, = ButtonData
                add r1, r0, #4*4*3
                mov r2, #0
                
LOOP
                str r2, [r0], #4
                cmp r0, r1
                ble LOOP
                
                pop {pc}
                

leftOn
    push {r0-r2, lr}
    ldr r0, =GPIOB_BASE
    ldr r1, [r0, #GPIO_ODR_o]
    orr r1, #(1 :SHL: leftPin)
    bic r1, #(1:SHL: rightPin)
    str r1, [r0, #GPIO_ODR_o]

    pop {r0-r2, pc}
    
rightOn
    push {r0-r2, lr}
    ldr r0, =GPIOB_BASE
    ldr r1, [r0, #GPIO_ODR_o]
    orr r1, #(1 :SHL: rightPin)
    bic r1, #(1 :SHL: leftPin)
    str r1, [r0, #GPIO_ODR_o]

    pop {r0-r2, pc}

;**************************************************************************************************
;* Jmeno funkce		: startPressedRaw
;* Popis			: 
;* Vstup			: r0 .. constant denoting a single button
;* Vystup			: r0 .. 1 iff button is currently presseed
;**************************************************************************************************

buttonPressedRaw								; Navesti zacatku podprogramu             
    push {r1-r4, lr}
    mov r4, r0
    mov r1, #4
    mul r1, r0
    ldr r2, =ButtonPorts
    ldr r3, =ButtonPins
    
    ldr r2, [r2, r1] ;buttons port
    ldr r3, [r3, r1] ; buttons pin
    ldr r0, [r2, #GPIO_IDR_o]
    lsr r0, r3
    
    mov r1, #~1
    bic r0, r1
    
    cmp r4, #BTNstart
    eorne r0, #1
    
    pop {r1-r4, pc}
    
;**************************************************************************************************
;* Jmeno funkce		: startPressed
;* Popis			: 
;* Vstup			: r0 .. constant denoting a single button
;* Vystup			: r0 .. 1 iff button is presseed (filtered from many previous measurements)
;**************************************************************************************************
buttonPressedFiltered								; Navesti zacatku podprogramu    
    push {r1, lr}
    mov r1, #4*3
    mul r0, r1
    add r0, #2*4

    ldr r1, =ButtonData
    ldr r0, [r1, r0]

    pop {r1, pc}
    
;**************************************************************************************************
;* Jmeno funkce		: startSample
;* Popis			: 
;* Vstup			: r0 .. constant denoting a single button
;* Vystup			: r0 .. 1 iff button is pressed
;**************************************************************************************************
sampleOne
    push {r0-r7, lr}
    mov r1, #4*3
    mul r7, r0, r1 
    ldr r1, =ButtonData
    add r7, r1 ;r7 points to button data
    
    bl buttonPressedRaw
    mov r3, r0; r3 == 1 if the button is pressed right now
    
    bl GetTick

    ldr r1, [r7, #PressedCache_o] ; r1 == 1 for pressed or 0 for not pressed
    
    cmp r1, r3
    
    beq SAME
DIFFERENT ;if the current state differs from the previous, store the new state
    str r0, [r7, #PressedStart_o] ;store the current time as the start time for press
    str r3, [r7, #PressedCache_o] ;stores one if the button is currently pressed
    
    pop {r0-r7, pc}

SAME ; if the state has been stable for long enough, save it
    ldr r1, [r7, #PressedStart_o]
    ; r1 == time when the last transition occured
    ; r2 == 1 iff the button is currently pressed
    mov r0, #debounceDelay ;debounce delay in ms
    bl TimeElapsed
    
    tst r0, r0
    ; if r0 != 0 then the button has been stable for a while. Otherwise return the other option
    
    popeq {r0-r7, pc}; return iff the button has not been stable

    ldr r0, [r7, #PressedCache_o]
    str r0, [r7, #LastValidState_o]
    
    pop {r0-r7, pc}; return

buttonSample
    push {r0, lr}
    mov r0, #BTNstart
NEXT_BUTTON
    bl sampleOne
    
    add r0, #1
    cmp r0, #BTNminus+1
    bne NEXT_BUTTON
    
    pop {r0, pc}
    
;**************************************************************************************************
;* Jmeno funkce		: led(Blue|Green)(On|Off)
;* Popis			: Turns the blue/green LED on or off
;* Vstup			: Zadny
;* Vystup			: None
;**************************************************************************************************
ledBlueOn     
    ldr r0, =ledBluePort
    add r0, #GPIO_BSRR_o ; r0 == &port::BSRR
    mov r1, #1 :SHL: ledBluePin
    
    str r1 , [r0]
    bx lr
    
ledGreenOn     
    ldr r0, =ledGreenPort
    add r0, #GPIO_BSRR_o ; r0 == &port::BSRR
    mov r1, #1 :SHL: ledGreenPin
    
    str r1 , [r0]
    bx lr
    
ledBlueOff     
    ldr r0, =ledBluePort
    add r0, #GPIO_BRR_o ; r0 == &port::BRR
    mov r1, #1 :SHL: ledBluePin
    
    str r1 , [r0]
    bx lr
    
ledGreenOff     
    ldr r0, =ledGreenPort
    add r0, #GPIO_BRR_o ; r0 == &port::BRR
    mov r1, #1 :SHL: ledGreenPin
    
    str r1 , [r0]
    bx lr

				END	
