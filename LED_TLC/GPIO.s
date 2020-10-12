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

userButtonPressedStart SPACE 4
userButtonPressedCache SPACE 4
userButtonLastValidState SPACE 4
                
		AREA    GPIO_Driver, CODE, READONLY  	; hlavicka souboru
	
		GET		INI.s					; vlozeni souboru s pojmenovanymi adresami
										; jsou zde definovany adresy pristupu do pameti (k registrum)

userButtonPort EQU GPIOA_BASE ;User button is connected to PA0
userButtonPin EQU 0
    
ledBluePort EQU GPIOC_BASE
ledGreenPort EQU GPIOC_BASE
ledBluePin EQU 8
ledGreenPin EQU 9
    
debounceDelay EQU 80 ; in ms

    EXPORT GPIO_CNF      
    EXPORT userButtonPressedFiltered
    EXPORT userButtonSample
    EXPORT ledBlueOn        
    EXPORT ledBlueOff
    EXPORT ledGreenOn
    EXPORT ledGreenOff
        
    import GetTick
    import TimeElapsed

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
                
                ;initialize variables
                bl GetTick
                sub r0, #debounceDelay
                ldr r1, =userButtonPressedStart
                str r0, [r1]
                
                ldr r0,=userButtonPressedCache
                mov r1, #0
                str r1, [r0]
                ldr r0, =userButtonLastValidState
                str r1, [r0]
                
                pop {pc}
                

;**************************************************************************************************
;* Jmeno funkce		: userButtonPressedRaw
;* Popis			: 
;* Vstup			: Zadny
;* Vystup			: r0 .. 1 iff button is currently presseed
;**************************************************************************************************
userButtonPressedRaw								; Navesti zacatku podprogramu             
    ldr r0, =userButtonPort
    add r0, #GPIO_IDR_o ; r0 == &port::IDR
    ldr r0, [r0]
    tst r0, #1 :SHL: userButtonPin
    
    ite eq
    moveq r0, #0 
    movne r0, #1 ; bit is set
    
    bx lr
    
;**************************************************************************************************
;* Jmeno funkce		: userButtonPressed
;* Popis			: 
;* Vstup			: Zadny
;* Vystup			: r0 .. 1 iff button is presseed (filtered from many previous measurements)
;**************************************************************************************************
userButtonPressedFiltered								; Navesti zacatku podprogramu    
    ldr r0, =userButtonLastValidState
    ldr r0, [r0]
    bx lr
    
;**************************************************************************************************
;* Jmeno funkce		: userButtonSample
;* Popis			: 
;* Vstup			: Zadny
;* Vystup			: r0 .. 1 iff button is presseed
;**************************************************************************************************
userButtonSample								
    push {r0,r1,r2 ,r3, lr}
    bl userButtonPressedRaw
    mov r3, r0; r3 == 1 if the button is pressed right now
    
    bl GetTick

    ldr r1, =userButtonPressedCache
    ldr r1, [r1] ; r1 == 1 for pressed or 0 for not pressed
    
    cmp r1, r3
    
    beq SAME
DIFFERENT ;if the current state differs from the previous, store the new state
    ldr r2, =userButtonPressedStart
    str r0, [r2] ;store the current time as the start time for press
    ldr r1, =userButtonPressedCache
    str r3, [r1] ;stores one if the button is currently pressed
    
    pop {r0,r1,r2 ,r3, pc}

SAME ; if the state has been stable for long enough, save it
    ldr r1, =userButtonPressedStart
    ldr r1, [r1]
    ; r1 == time when the last transition occured
    ; r2 == 1 iff the button is currently pressed
    mov r0, #debounceDelay ;debounce delay in ms
    bl TimeElapsed
    
    tst r0, r0
    ; if r0 != 0 then the button has been stable for a while. Otherwise return the other option
    
    popeq {r0,r1,r2 ,r3, pc}; return iff the button has nott been stable

    ldr r0, = userButtonPressedCache
    ldr r0, [r0]
    ldr r1, = userButtonLastValidState
    
    str r0, [r1]
    
    pop {r0,r1,r2 ,r3, pc}; return


    
    
;**************************************************************************************************
;* Jmeno funkce		: ledBlueOn
;* Popis			: Turns the blue LED on
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
