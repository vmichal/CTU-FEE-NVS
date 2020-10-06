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
;* AUTOR			: Michal TOMÁŠ
;* DATUM			: 12/2010
;* POPIS			: Program pro stridave blikani LED na vyvodech PC8 a PC9 se dvema mody ovladanymi tlacitkem.
;*					  - konfigurace hodin na frekvenci 24MHz (HSE + PLL) 
;*					  - konfigurace pouzitych vyvodu procesotu (PC8 a PC9 jako vystup, PA0 jako vstup)
;*					  - rozblikani LED na PC8 a PC9, cteni stavu tlacitka a prepinani modu blikani
;* Poznamka			: Tento soubor obsahuje podrobny popis kodu vcetne vyznamu pouzitych instrukci
;*
;***************************************************************************************************
				
		AREA    STM32F1xx, CODE, READONLY  	; hlavicka souboru
	
		GET		INI.s					; vlozeni souboru s pojmenovanymi adresami
										; jsou zde definovany adresy pristupu do pameti (k registrum)
        GET		CorePeripherals.s	
        
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++										
konst1 EQU	0x80000 					; direktiva EQU priradi vyrazu 'konst1' hodnotu 0x80000 
konst2 EQU	0x10000						; direktiva EQU priradi vyrazu 'konst2' hodnotu 0x10000 
;++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++										

											
		EXPORT	__main					; export navesti pouzivaneho v jinem souboru, zde konkretne
		EXPORT	__use_two_region_memory	; jde o navesti, ktere pouziva startup code STM32F10x.s

        IMPORT STK_CONFIG
        IMPORT BlockingDelay
        IMPORT GPIO_CNF
        IMPORT RCC_CNF
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

				BL		RCC_CNF			; Volani podprogramu nastaveni hodinoveho systemu procesoru
										; tj. skok na adresu s navestim RCC_CNF a ulozeni navratove 
										; adresy do LR (Link Register)
                mov r0, #24000 ;24 000 cycles between SysTick interrupt (corresponds to 24MHz system clock)
                bl STK_CONFIG
				BL		GPIO_CNF		; Volani podprogramu konfigurace vyvodu procesoru
										; tj. skok na adresu s navestim GPIO_CNF 
										;*!* Poznamka pri pouziti volani podprogramu instrukci BL nesmi
										; byt v obsluze podprogramu tato instrukce jiz pouzita, nebot
										; by doslo k prepsani LR a ztrate navratove adresy ->
										; lze ale pouzit i jine instrukce (PUSH, POP) *!*


         
				MOV		R3, #konst1		; Kopie konstanty 'konst1' do R3
				MOV		R4,#0			; Vlozeni 0 do R4, nulovani citace (softwarový citac registr R4)


LOOP									; hlavni smycka programu, blikani LED a cteni stavu tlacitka	
				ADD		R4,R4,#1		; R4 = R4 + 1, inkrementace citace o 1
				TST		R4, R3			; Porovnani R4 a R3 => (R4 & R3) a nastaveni priznaku
                beq GREEN
BLUE
                bl ledBlueOn
                bl ledGreenOff
                b AFTER
GREEN
                bl ledBlueOff
                bl ledGreenOn
AFTER
; Blikani LED, frekvence je dana registrem R3


				; Testovani stisku tlacitka
                bl userButtonPressedFiltered
                
                tst r0, r0
				BEQ		LOOP			; Skok na navesti LOOP, je-li vysledek predchozi operace roven 0
										; tj. skok na LOOP pri nestisknutem tlacitku, jinak se pokracuje

				; Prodleva pro osetreni zakmitu tlacitka
				MOV		R0, #100			; Vlozeni hodnoty prodlevy do R0, tj. 50
				BL		BlockingDelay			; Volani rutiny prodlevy, R0 je vtupni parametr DELAY

				; Zmena modu blikani LED, vlozeni jine konstanty frekvence blikani do R3
 				TST		R3, #konst1		; Test puvodni hodnoty konstanty v R3, (R3 & 0x80000) nebo 
										; (R3 & 0x10000) a nastaveni priznaku
				BEQ		KONST			; Skok na navesti KONST pri R3 = konst2 (byla to puvodni
										; hodnota frekvence tak ji zmenime aby se blikalo jinou
										; frekvenci), jinak se pokracuje
				MOV		R3, #konst2		; Vlozeni konstanty 0x10000 do R3, zmena frekvence
				B		LOOP			; Skok na navesti LOOP pro opakovani smycky
KONST
				MOV		R3, #konst1		; Vlozeni konstanty 0x80000 do R3, zmena frekvence
				B		LOOP			; Skok na navesti LOOP pro opakovani smycky


				END	
