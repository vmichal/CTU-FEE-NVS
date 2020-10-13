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
lastUserButtonState SPACE 4 
systemStartTimestamp SPACE 4 ;timestamp when the blue LED was turned on.
systemActive SPACE 4 ;true iff the automat is currently on (blue LED turned on)
                
		AREA    STM32F1xx, CODE, READONLY  	; hlavicka souboru
	
		GET		INI.s					; vlozeni souboru s pojmenovanymi adresami
										; jsou zde definovany adresy pristupu do pameti (k registrum)
        GET		CorePeripherals.s	
        
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++										
tZAP  EQU 5000 ; 5 seconds turned on
tBlinkOff EQU 100 ; indicates that the device is going to shut down soon
tBlinkOn EQU 500 ;
tBlinkPeriod EQU tBlinkOff + tBlinkOn
kBlinkCount EQU 4;how many times the LED blinks
tBlinkingLength EQU tBlinkPeriod * kBlinkCount

;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++										


											
		EXPORT	__main					; export navesti pouzivaneho v jinem souboru, zde konkretne
		EXPORT	__use_two_region_memory	; jde o navesti, ktere pouziva startup code STM32F10x.s

        IMPORT STK_CONFIG
        IMPORT BlockingDelay
        IMPORT GPIO_CNF
        IMPORT RCC_CNF
        import GetTick
        import userButtonPressedFiltered
                    import userButtonSample
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
    ldr r0, =lastUserButtonState 
    str r1, [r0]; initialize lastUserButtonState to false
    ldr r0, =systemStartTimestamp
    str r1, [r0]
    ldr r0, =systemActive
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

LOOP ;main application loop                
    bl userButtonPressedFiltered ; get current state into r0
    ldr r1, = lastUserButtonState
    ldr r2, [r1] ;load previous state into r2
                
    cmp r2, r0 ;if the current state is the same as the previous state 
                
    beq BUTTON_CHECK_DONE; button state has not changed, skip button validation
    str r0, [r1] ;store new valid state
                
    tst r0, r0 ;true (not zero) iff button is pressed
               
    bne BUTTON_PRESSED
    bl processButtonRelease
    b BUTTON_CHECK_DONE
BUTTON_PRESSED                
    bl processButtonPress
BUTTON_CHECK_DONE

    ldr r0, =systemActive
    ldr r0, [r0]
    tst r0, r0
    beq LOOP ;if the system is not active, break
    ;check current time
    ldr r0, =systemStartTimestamp
    ldr r1, [r0]
    bl GetTick; r0 == current tick, r1 == start tick
                
    sub r2, r0, r1; t2 == time elapsed since start
    mov r3, #tZAP
    cmp r2, r3
    bhs END_ACTIVE ;enough time has elapsed, turn the automat off
                
    bl getUninterruptedActiveTime
    cmp r2, r0
    blo LOOP ;uninterrupted time
                
BLINKING    ;the active duration is nearing the end and we have to blink the LED
    sub r2, r2, r0 ; r2 == only the extra time
    mov r0, #tBlinkPeriod
                
    udiv r1, r2, r0 ;elapsed time / length of blink period will give us the number of finished periods
    mul r1, r0
    sub r2, r1 ; r2 holds the number of ms in the current blink period.
               
    mov r1, #tBlinkOff
    cmp r2, r1 ; determine whether we are in the on or off part of blinkgin period    
               
    bhs BLINK_ON
    bl ledBlueOff
    b LOOP
                
BLINK_ON
    bl ledBlueOn                
    b LOOP
END_ACTIVE
    ldr r0, =systemActive
    mov r1, #0
    str r1, [r0] ;mark the system as not active
    bl ledGreenOff ;turn off active indicator
    bl ledBlueOff ;
                
    b LOOP
				
processButtonPress
    push {lr}
    bl ledGreenOn            
    pop {pc}
    
processButtonRelease
    push {r0, r1, lr}
    bl ledBlueOn
    bl ledGreenOn            
    
    ldr r0, = systemActive
    mov r1, #1
    str r1, [r0] ; store systemActive as true
    
    bl GetTick
    ldr r1, =systemStartTimestamp
    str r0, [r1] ; store the current timestamp (start the timer)
            
    pop {r0, r1, pc}                
    
;returns the number of ms, how long the system has to be active (led turned on) before blinking starts
; computed as period length * blink count, where period length = LEDonTime + LEDoffTime
; and blink count stores the number of 
getUninterruptedActiveTime
    mov r0, #tBlinkingLength
    bx lr
                END	
                    
