;***************************************************************************************************
;* Vyvojovy Kit		: STM32 VL DISCOVERY (STM32F100RB)
;*
;**************************************************************************************************
;*
;* JMÉNO SOUBORU	: LED_TLC.ASM
;* AUTOR			: Vojtech Michal based on Michal TOMÁŠ's work
;***************************************************************************************************
    AREA strings, CODE, READONLY
stringHeader
    DCB "\rVomi's automat: "
stringHeaderEnd

stringOn
    DCB " ONLINE , "
stringOnEnd

stringOff
    DCB "OFFLINE , "
stringOffEnd

stringConf
    DCB " CONFIG , "
stringConfEnd

stringSaved
    DCB "  SAVED , "
stringSavedEnd


stringHeaderLen EQU (stringHeaderEnd - stringHeader)
stringOffLen EQU (stringOffEnd - stringOff)
stringOnLen EQU (stringOnEnd - stringOn)
stringConfLen EQU (stringConfEnd - stringConf)    
stringSavedLen EQU (stringSavedEnd - stringSaved)
stringIntLen EQU 2

stringHeaderStart EQU terminalString
stringOffStart EQU stringHeaderStart + stringHeaderLen
stringOnStart EQU stringOffStart
stringConfStart EQU stringOffStart
stringSavedStart EQU stringOffStart
stringIntStart EQU stringOnStart + stringOnLen
    
		AREA mojedata, DATA, NOINIT, READWRITE
            
; previous known state of the user button. If it differs from filtered value, it implies that an edge occured
lastButtons
lastStart SPACE 4 
lastOK SPACE 4
lastPlus SPACE 4
lastMinus SPACE 4
    
systemStartTimestamp SPACE 4 ;timestamp when the blue LED was turned on.
systemState SPACE 4 ;holds constant representing the current system state
tZap SPACE 4; hold the currently configured length of active state
tZapUnsaved SPACE 4; holds the new length of active state during configuration. It is not saved yet
terminalString SPACE stringHeaderLen + stringOffLen + stringIntLen
terminalStringEnd

		AREA    STM32F1xx, CODE, READONLY  	; hlavicka souboru
	
		GET		INI.s					; vlozeni souboru s pojmenovanymi adresami
										; jsou zde definovany adresy pristupu do pameti (k registrum)
        GET		CorePeripherals.s	
        
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++										
tZAPdefault  EQU 4 ; 4 seconds turned on
tIdleBlink EQU 1000;
tConfigBlink EQU 1000; half period of blinking in config state
    
;state machine constants
stateActive EQU 0; lights are on
stateConfiguration EQU 1; service buttons are being used
stateIdle EQU 2; we are waiting for user input
USART_baudrate EQU 9600
tZapMax EQU 99 ;in seconds
tZapMin EQU 1; in seconds


segmentA EQU 1 << 7
segmentB EQU 1 << 6
segmentC EQU 1 << 5
segmentD EQU 1 << 4
segmentE EQU 1 << 3
segmentF EQU 1 << 2
segmentG EQU 1 << 1
segmentDP EQU 1 << 0


segmentsThreeLines EQU segmentA :OR: segmentG :OR: segmentD;
number0 EQU segmentA :OR: segmentB :OR: segmentC :OR: segmentD :OR: segmentE :OR: segmentF ;0
number1 EQU segmentB :OR: segmentC ;1
number2 EQU segmentsThreeLines :OR: segmentB :OR: segmentE ;2
number3 EQU segmentsThreeLines :OR: segmentB :OR: segmentC;3
number4 EQU number1 :OR: segmentF :OR: segmentG ;4
number5 EQU segmentsThreeLines :OR: segmentC:OR: segmentF ;5
number6 EQU number5 :OR: segmentE ;6
number7 EQU number1 :OR: segmentA;7
number8 EQU number0 :OR: segmentG;8
number9 EQU number4 :OR: segmentA :OR: segmentD;
number10 EQU number0;10
    ALIGN 4
displayNumbers

    DCD ~number0
    DCD ~number1
    DCD ~number2
    DCD ~number3
    DCD ~number4
    DCD ~number5
    DCD ~number6
    DCD ~number7
    DCD ~number8
    DCD ~number9
        
BTNstart EQU 0
BTNok EQU 1    
BTNplus EQU 2   
BTNminus EQU 3
            
start_o EQU 0            
ok_o EQU 4
plus_o EQU 8
minus_o EQU 12
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++										


											
		EXPORT	__main					; export navesti pouzivaneho v jinem souboru, zde konkretne
		EXPORT	__use_two_region_memory	; jde o navesti, ktere pouziva startup code STM32F10x.s
        export applicationTick

        IMPORT STK_CONFIG
        IMPORT BlockingDelay
        IMPORT GPIO_CNF
        IMPORT RCC_CNF
        import GetTick
        import buttonPressedFiltered
    import ledBlueOn        
    import ledBlueOff
    import ledGreenOn
    import ledGreenOff     
    import rightOn
    import leftOn
        
		;SystemTicks ; Stores the number of ms since boot (for now lives in the beggining of RAM)
__use_two_region_memory	
__main								  						
		
		ENTRY							; vstupni bod do kodu, odtud se zacina vykonavat program

;***************************************************************************************************
;* Jmeno funkce		: MAIN
;* Popis			: Hlavni program + volani podprogramu nastaveni hodinoveho systemu a konfigurace
;*					  pouzitych vyvodu procesoru	
;* Vstup			: Zadny
;* Vystup			: Zadny
;***************************************************************************************************

MAIN									; MAIN navesti hlavni smycky programu
    ;clears variables in data
    ldr r0, =stringHeaderStart
    ldr r1, =stringHeader
    mov r2, #stringHeaderLen
    bl memcpy

    ldr r0, =stringOffStart
    ldr r1, =stringOff
    mov r2, #stringOffLen
    bl memcpy
    
    ldr r0, =stringIntStart
    mov r1, #' '
    mov r2, #stringIntLen
    bl memset
    
    mov r1, #0
    ldr r0, =lastButtons
    mov r2, #4*4
    bl memset
    
    ldr r0, =systemStartTimestamp
    str r1, [r0]
    
    ldr r0, =systemState
    mov r1, #stateIdle
    str r1, [r0]
    
    ldr r0, =tZap
    mov r1, #tZAPdefault
    str r1, [r0]

	BL		RCC_CNF			; Volani podprogramu nastaveni hodinoveho systemu procesoru
										; tj. skok na adresu s navestim RCC_CNF a ulozeni navratove 
										; adresy do LR (Link Register)
    mov r0, #24000 ;24 000 cycles between SysTick interrupt (corresponds to 24MHz system clock)
    bl STK_CONFIG
	bl GPIO_CNF		; Volani podprogramu konfigurace vyvodu procesoru
										; tj. skok na adresu s navestim GPIO_CNF 
										;*!* Poznamka pri pouziti volani podprogramu instrukci BL nesmi
										; byt v obsluze podprogramu tato instrukce jiz pouzita, nebot
										; by doslo k prepsani LR a ztrate navratove adresy ->
										; lze ale pouzit i jine instrukce (PUSH, POP) *!*
    bl initSPI
    bl initUSART
    bl initDMA
    
LOOP ;main application loop
    bl checkStartButton
    bl checkPlusButton
    bl checkMinusButton
    bl checkOkButton
    bl checkUsart
    b LOOP ; continue in main loop
    ALIGN 4
    LTORG

memcpy;(r0 ..destination, r1 .. source, r2 ... bytes)
    push {r3, lr}
    cmp r2, #0
    pople {r3, pc}
    
MEMCPY_LOOP    
    ldrb r3, [r1], #1
    strb r3, [r0], #1
    sub r2, #1
    cmp r2, #0
    bgt MEMCPY_LOOP
        
    pop {r3, pc}

memset;(r0 ...destination, r1 .. byte, r2 .. count
    push {r3, lr}
    cmp r2, #0
    pople {r3, pc}
MEMSET_LOOP
    strb r1, [r0], #1
    sub r2, #1
    cmp r2, #0
    bgt MEMSET_LOOP
    
    pop {r3, pc}

int2str;(r0 .. integer value, r1 .. pointer to buffer, r2 ... buffer size)
    push {r3-r7, lr}
    push {r0-r2}
    mov r0, r1
    mov r1, #' '
    bl memset ;clear the buffer with spaces.
    pop {r0-r2}

    push {r0-r2}
    mov r3, r0
    mov r4, r1
CONVERSION_LOOP
    mov r0, r3
    mov r1, #10
    bl modulo
    add r0, #'0'
    strb r0, [r4], #1
    mov r1, #10
    udiv r3, r3, r1
    cmp r3, #0
    bgt CONVERSION_LOOP
    
    pop {r0-r2}
    push {r0-r2}
    mov r3, #2
    udiv r4, r2, r3
    mov r0, #0
REVERSE_LOOP
    sub r3, r2, r0
    sub r3, #1
    ldrb r7, [r1, r0]
    ldrb r6, [r1, r3]
    strb r6, [r1, r0]
    strb r7, [r1, r3]
    add r0, #1
    cmp r0, r4
    blt REVERSE_LOOP

    pop {r0-r2}
    pop {r3-r7, pc}
    

configIncrease
    push {r0-r2, lr}
    ldr r0, =systemState
    ldr r1, [r0]
    cmp r1, #stateConfiguration
    it ne
    blne enterConfig

    ldr r0, =tZapUnsaved
    ldr r1, [r0]
    add r1, #1
    mov r0, r1
    mov r1, #tZapMin
    mov r2, #tZapMax
    bl clamp
    ldr r1, = tZapUnsaved
    str r0, [r1]
    pop {r0-r2,pc}

configDecrease
    push {r0-r2, lr}
    ldr r0, =systemState
    ldr r1, [r0]
    cmp r1, #stateConfiguration
    it ne
    blne enterConfig

    ldr r0, =tZapUnsaved
    ldr r1, [r0]
    sub r1, #1
    mov r0, r1
    mov r1, #tZapMin
    mov r2, #tZapMax
    bl clamp
    ldr r1, = tZapUnsaved
    str r0, [r1]
    pop {r0-r2,pc}


configSave
    push {r0-r1, lr}
    ldr r0, =systemState
    ldr r1, [r0]
    cmp r1, #stateConfiguration
    popne {r0-r1, pc}; we are not in state configuration -> ignore this button press
    
    mov r1, #stateIdle
    str r1, [r0] ; go to state idle
    ldr r0, =tZap
    ldr r1, =tZapUnsaved
    ldr r1, [r1]
    str r1, [r0]

    ldr r0, =stringSavedStart
    ldr r1, =stringSaved
    mov r2, #stringSavedLen
    bl memcpy
    
    pop {r0-r1, pc}

checkUsart
    push {r0-r3, lr}
    
    ldr r0, =USART1_BASE
    ldr r1, [r0, #USART_SR_o]
    tst r1, #USART_SR_RXNE ;RXNE
    popeq {r0-r3, pc} ;return if nothing has happened
    
    ldr r1, [r0, #USART_DR_o]; r1 contains the received character
    
    cmp r1, #'+'
    beq PLUS_RECEIVED
    cmp r1, #'-'
    beq MINUS_RECEIVED
    cmp r1, #' '
    beq SPACE_RECEIVED
    cmp r1, #'s'
    beq S_RECEIVED
    b USART_DONE
    
MINUS_RECEIVED
    bl configDecrease
    b USART_DONE
PLUS_RECEIVED
    bl configIncrease
    b USART_DONE
SPACE_RECEIVED
    bl configSave
    b USART_DONE
S_RECEIVED
    bl activateSystem
    
USART_DONE
    pop {r0-r3, pc}

checkMinusButton
    push {r0-r3, lr}
    mov r0, #BTNminus
    bl buttonPressedFiltered ; get current state into r0
    ldr r1, = lastButtons
    ldr r2, [r1, #minus_o] ;load previous state into r2
                
    cmp r2, r0 ;if the current state is the same as the previous state 
         
    popeq {r0-r3, pc}; button state has not changed, skip button validation
    str r0, [r1, #minus_o] ;store new valid state
                
    tst r0, r0 ;true (not zero) iff button is pressed
    popne {r0-r3, pc}; we will wait for release

    bl configDecrease
    
    pop {r0-r3, pc};


checkPlusButton
    push {r0-r3, lr}
    mov r0, #BTNplus
    bl buttonPressedFiltered ; get current state into r0
    ldr r1, = lastButtons
    ldr r2, [r1, #plus_o] ;load previous state into r2
                
    cmp r2, r0 ;if the current state is the same as the previous state 
         
    popeq {r0-r3, pc}; button state has not changed, skip button validation
    str r0, [r1, #plus_o] ;store new valid state
                
    tst r0, r0 ;true (not zero) iff button is pressed
    popne {r0-r3, pc}; we will wait for release

    bl configIncrease
    
    pop {r0-r3, pc};

checkOkButton
    push {r0-r3, lr}
    mov r0, #BTNok
    bl buttonPressedFiltered ; get current state into r0
    ldr r1, = lastButtons
    ldr r2, [r1, #ok_o] ;load previous state into r2
                
    cmp r2, r0 ;if the current state is the same as the previous state 
         
    popeq {r0-r3, pc}; button state has not changed, skip button validation
    str r0, [r1, #ok_o] ;store new valid state
                
    tst r0, r0 ;true (not zero) iff button is pressed
    popne {r0-r3, pc}; we will wait for release
    
    bl configSave
    
    pop {r0-r3, pc};
        
checkStartButton
    push {r0-r3, lr}
    mov r0, #BTNstart
    bl buttonPressedFiltered ; get current state into r0
    ldr r1, = lastButtons
    ldr r2, [r1, #start_o] ;load previous state into r2
                
    cmp r2, r0 ;if the current state is the same as the previous state 
         
    popeq {r0-r3, pc}; button state has not changed, skip button validation
    str r0, [r1, #start_o] ;store new valid state
                
    tst r0, r0 ;true (not zero) iff button is pressed
               
    bne START_PRESSED
    bl activateSystem
    pop {r0-r3, pc};
START_PRESSED                
    bl processStartPress
    pop {r0-r3, pc};
    
deactivateSystem
    push {r0-r3,lr}
    ldr r0, =systemState
    mov r1, #stateIdle
    str r1, [r0] ;mark the system as not active
    mov r0, #BTNstart
    bl buttonPressedFiltered;
    tst r0,r0
    it eq
    bleq ledGreenOff ;turn off active indicator only if the user button is not pressed
    bl ledBlueOff ;
    mvn r0, #0
    bl update7segment

    ldr r0, =stringOffStart
    ldr r1, =stringOff
    mov r2, #stringOffLen
    bl memcpy
    
    pop {r0-r3, pc}
				
activateSystem
    push {r0-r1, lr}
    bl ledBlueOn
    ldr r0, = systemState
    mov r1, #stateActive
    str r1, [r0] ; store systemActive as true
    
    bl GetTick
    ldr r1, =systemStartTimestamp
    str r0, [r1] ; store the current timestamp (start the timer)

    ldr r0, =tZap
    ldr r0, [r0]
    bl getNumberSegments

    bl update7segment
    
    ldr r0, =stringOnStart
    ldr r1, =stringOn
    mov r2, #stringOnLen
    bl memcpy
    
    pop {r0-r1, pc}
    
enterConfig
    push {r1-r3, lr}
    bl deactivateSystem
    ldr r0, =systemState
    mov r1, #stateConfiguration
    str r1, [r0]
    
    ldr r0, =tZap
    ldr r0, [r0]
    ldr r1, =tZapUnsaved
    str r0, [r1]
    mvn r0, #0
    bl update7segment ;turn the display "off"
    
    ldr r0, =stringConfStart
    ldr r1, =stringConf
    mov r2, #stringConfLen
    bl memcpy
    
    pop {r1-r3, pc}
    
processStartPress
    push {lr}
    bl ledGreenOn
    pop {pc}
    
    
initSPI ;initialize SPI2 connected to the 7segment shift register
    push {r0-r1}
    
    ldr r0, =SPI2_BASE
    ;set SPI as master (bit 2) and enable it
    ; make the peripheral send LSB first
    ; use only one direction of data transfer (from MISO to the shift register)
    ldr r1, = (1 :SHL:2) :OR: (1 :SHL: 6) :OR: (1 :SHL: 7) :OR: (3 :SHL: 14)
    str r1, [r0, #SPI_CR1_o]
    mvn r1, #0
    str r1, [r0, #SPI_DR_o]
    pop {r0-r1}
    bx lr
    
clamp;(r0 value, r1 min, r2 max)
    cmp r0, r2
    
    itt hi
    movhi r0, r2
    bxhi lr
    
    cmp r0, r1
    it le
    movle r0, r1
    
    bx lr
    
getNumberSegments;(r0 .. positive number from interval <0, 10>)
    push {r1-r3, lr}
    mov r1, #10
    bl modulo
    
    mov r2, #4
    mul r0, r2
    ldr r2, =displayNumbers
    ldr r0, [r2, r0]
    pop {r1-r3, pc}

update7segment;(r0 ... 8bit wide bitmask identifying segments with a...lsb, decimal point ... msb)
    push {r1, lr}
    ldr r1, =SPI2_BASE
    strh r0, [r1, #SPI_DR_o] ; store the new byte to be sent
    pop {r1, pc}

shallLeftBeOn ; returns true in r0 iff the left display shall be on
    push {lr}
    bl GetTick
    bic r0, #~1 ;clear everything except for lsb
    pop {pc}
    
applicationTick
    push {r0-r3,lr}
    
    mvn r0, #0
    bl update7segment
    bl shallLeftBeOn
    tst r0, r0
    bne LEFT_ON
    bl rightOn
    b DISPLAY_DONE
LEFT_ON
    bl leftOn        
DISPLAY_DONE

    ldr r0, =systemState
    ldr r0, [r0] ;r0 = current state
    
    cmp r0, #stateIdle; we have nothing to do ... just blink decimal point
    beq HANDLE_IDLE
    
    cmp r0, #stateConfiguration
    beq HANDLE_CONFIG

    ;the system is active
    bl GetTick;
    ldr r1, =systemStartTimestamp
    ldr r1, [r1]
    sub r0, r1 ; r0= elapsed ms
    ldr r1, =tZap
    ldr r1, [r1]
    mov r2, #1000
    mul r1, r2
    sub r0, r1, r0; r0 = ms remaining
    
    cmp r0, #0
    bge HANDLE_ACTIVE
    bl deactivateSystem
    pop {r0-r3, pc}
    
HANDLE_CONFIG

    ldr r0, =tZapUnsaved
    ldr r7, [r0]
    mov r0, r7
    ldr r1, =stringIntStart
    mov r2, #stringIntLen
    bl int2str
    
    mvn r6, #0 ;register with decimal point
    bl shallLeftBeOn
    tst r0, r0
    itte ne
    movne r1, #10
    udivne r7, r7, r1
    biceq r6, #segmentDP
    mov r0, r7
    bl getNumberSegments
    mov r7, r0
    bl GetTick
    mov r1, #tConfigBlink
    bl modulo
    cmp r0, #tConfigBlink*8/10
    mov r0, r7
    mvnge r0, #0
    
    and r0, r6
    bl update7segment
    pop {r0-r3, pc}
    
    
HANDLE_IDLE
    ldr r0, =tZap
    ldr r7, [r0]
    mov r0, r7
    ldr r1, =stringIntStart
    mov r2, #stringIntLen
    bl int2str
    bl shallLeftBeOn
    mov r6, r0
    tst r0, r0
    itt ne
    movne r1, #10
    udivne r7, r7, r1    
    mov r0, r7
    bl getNumberSegments
    mov r7, r0

    bl GetTick
    mov r1, #tIdleBlink*2
    bl modulo
    cmp r0, #tIdleBlink
    ite ge
    movge r0, #1
    movlt r0, #0
    mvn r6, r6
    tst r0, r6
    
    mov r0, r7
    it ne
    bicne r0, #segmentDP  
    bl update7segment
    pop {r0-r3, pc}
    
    
HANDLE_ACTIVE
    
    add r0, #500 ;nice delay so that the initial 5 is visible
    mov r1, #1000
    udiv r7, r0, r1
    push {r0-r1}
    mov r0, r7
    ldr r1, =stringIntStart
    mov r2, #stringIntLen
    bl int2str
    pop {r0-r1}
    bl shallLeftBeOn
    tst r0, r0
    itt ne
    movne r1, #10
    udivne r7, r7, r1    
    mov r0, r7
    
    bl getNumberSegments
    bl update7segment
    pop {r0-r3, pc}

modulo; returns r0 % r1
    push {r1-r2, lr}
    udiv r2, r0, r1
    mul r2, r1
    sub r0, r2
    pop {r1-r2, pc}

isDivisible;(r0 what, r1 divider)
    push {r1-r2, lr}
    bl modulo
    tst r0, r0
    ite eq
    moveq r0, #1
    movne r0, #0
    pop {r1-r2, pc}    
    
initUSART
    push {r0-r3, lr}
    ldr r0, =USART1_BASE
    ;we don't need to adjust anything in status reg, nor data reg
    ;TODO set baudrate
    ldr r1, =((24000000/8/2/USART_baudrate) :SHL: 4) :OR: 4
    str r1, [r0, #USART_BRR_o]
    
    ldr r1, [r0, #USART_CR1_o]
    orr r1, #1:SHL:13; ;enable USART
    orr r1, #1 :SHL: 2 :OR: 1 :SHL:3 ;enable transmiter and receiver
    str r1, [r0, #USART_CR1_o]
    
    ldr r1, [r0, #USART_CR3_o]
    orr r1, #1:SHL:7 ;dma transmission
    str r1, [r0, #USART_CR3_o]
    
    pop {r0-r3, pc}
    
initDMA; we use DMA1, channel 4 for USART1_TX
    push {r0-r3, lr}
    ldr r0, =DMA1_BASE
    
    mov r1, #stringHeaderLen + stringOffLen + stringIntLen
    str r1, [r0, #DMA_CNDTR4_o]
    
    ldr r1, =USART1_BASE + USART_DR_o
    str r1, [r0, #DMA_CPAR4_o]
    
    ldr r1, =terminalString
    str r1, [r0, #DMA_CMAR4_o]
    
    ldr r1, [r0, #DMA_CCR4_o]
    ;max priority, memory increment mode, circular mode, memory->peripheral, enable channel
    ldr r2, =(3 :SHL: 12) :OR: (1:SHL: 7) :OR: (1 :SHL: 5) :OR: (1 :SHL: 4) :OR: (1 :SHL: 0)
    orr r1, r2
    str r1, [r0, #DMA_CCR4_o]
    
    pop {r0-r3, pc}

    END	