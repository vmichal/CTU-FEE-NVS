;***************************************************************************************************
;* Vyvojovy Kit		: STM32 VL DISCOVERY (STM32F100RB)
;*
;**************************************************************************************************
;*
;* JMÉNO SOUBORU	: LED_TLC.ASM
;* AUTOR			: Vojtech Michal based on Michal TOMÁŠ's work
;***************************************************************************************************
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

		AREA    STM32F1xx, CODE, READONLY  	; hlavicka souboru
	
		GET		INI.s					; vlozeni souboru s pojmenovanymi adresami
										; jsou zde definovany adresy pristupu do pameti (k registrum)
        GET		CorePeripherals.s	
        
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++										
tZAPdefault  EQU 4 ; 4 seconds turned on
tIdleBlink EQU 1000;
tConfigBlink EQU 400; half period of blinking in config state
    
;state machine constants
stateActive EQU 0; lights are on
stateConfiguration EQU 1; service buttons are being used
stateIdle EQU 2; we are waiting for user input


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
    DCD ~number10
        
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
    mov r1, #0
    ldr r0, =lastButtons
    str r1, [r0]; initialize lastStart to false
    str r1, [r0, #4]; initialize lastOK to false
    str r1, [r0, #8]; initialize lastPlus to false
    str r1, [r0, #12]; initialize lastMinus to false

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
    mov r0, #tZAPdefault
    bl getNumberSegments
    bl update7segment
    
    
LOOP ;main application loop
    bl checkStartButton
    bl checkPlusButton
    bl checkMinusButton
    bl checkOkButton
    b LOOP ; continue in main loop

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

    ldr r0, =systemState
    ldr r1, [r0]
    cmp r1, #stateConfiguration
    it ne
    blne enterConfig

    ldr r0, =tZapUnsaved
    ldr r1, [r0]
    sub r1, #1
    mov r0, r1
    mov r1, #1
    mov r2, #10
    bl clamp
    ldr r1, = tZapUnsaved
    str r0, [r1]
    
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

    ldr r0, =systemState
    ldr r1, [r0]
    cmp r1, #stateConfiguration
    it ne
    blne enterConfig

    ldr r0, =tZapUnsaved
    ldr r1, [r0]
    add r1, #1
    mov r0, r1
    mov r1, #1
    mov r2, #10
    bl clamp
    ldr r1, = tZapUnsaved
    str r0, [r1]
    
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
    
    ldr r0, =systemState
    ldr r1, [r0]
    cmp r1, #stateConfiguration
    popne {r0-r3, pc}; we are not in state configuration -> ignore this button press
    
    mov r1, #stateIdle
    str r1, [r0] ; go to state idle
    ldr r0, =tZap
    ldr r1, =tZapUnsaved
    ldr r1, [r1]
    str r1, [r0]
    
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
    push {r0-r1,lr}
    ldr r0, =systemState
    mov r1, #stateIdle
    str r1, [r0] ;mark the system as not active
    mov r0, #BTNstart
    bl buttonPressedFiltered;
    tst r0,r0
    it eq
    bleq ledGreenOff ;turn off active indicator only if the user button is not pressed
    bl ledBlueOff ;
    mov r0, #~0
    bl update7segment
                
    pop {r0-r1, pc}
				
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

    pop {r0-r1, pc}
    
enterConfig
    push {r1-r3, lr}
    ldr r0, =systemState
    mov r1, #stateConfiguration
    str r1, [r0]
    
    ldr r0, =tZap
    ldr r0, [r0]
    ldr r1, =tZapUnsaved
    str r0, [r1]
    bl getNumberSegments
    bic r0, #segmentDP
    bl update7segment    
    bl ledBlueOff
    bl ledGreenOff
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
    mov r1, #0xff
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
    mov r1, #10+1
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
    
applicationTick
    push {r0-r3,lr}
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
    bge UPDATE_7_SEGMENT
    bl deactivateSystem
    pop {r0-r3, pc}
    
HANDLE_CONFIG
    bl GetTick
    mov r1, #tConfigBlink
    mov r3, r0
    bl isDivisible
    tst r0, r0
    
    it eq
    popeq {r0-r3, pc}
    mov r0, r3
    mov r1, #tConfigBlink*2
    bl isDivisible
    mov r3, r0
    
    ldr r0, =tZapUnsaved
    ldr r0, [r0]
    bl getNumberSegments
    
    tst r3, r3
    
    mov r1, #~segmentDP
    it eq
    andeq r1, r0
    mov r0, r1
    bl update7segment
    pop {r0-r3, pc}
    
    
SHALL_DRAW    
    ldr r0, =tZap
    ldr r0, [r0]
    bl getNumberSegments
    bl update7segment
    pop {r0-r3, pc}
    
    
HANDLE_IDLE
    bl GetTick
    mov r1, #tIdleBlink
    mov r3, r0
    bl isDivisible
    tst r0, r0

    it eq
    popeq {r0-r3, pc}
    
    ldr r0, =tZap
    ldr r0, [r0]
    bl getNumberSegments
    mov r7, r0

    mov r0, r3
    mov r1, #tIdleBlink*2
    bl isDivisible
    tst r0, r0
    
    mov r0, r7
    it eq
    biceq r0, #segmentDP  
    bl update7segment
    pop {r0-r3, pc}
    
    
UPDATE_7_SEGMENT
    
    add r0, #500 ;nice delay so that the initial 5 is visible
    mov r1, #1000
    udiv r0, r0, r1
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

    END	
                    
