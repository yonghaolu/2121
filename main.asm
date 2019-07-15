  
;COMP212 Project.asm

;Author:
;Yonghao Lu z5125710
;Yanhao  Xu	z5155164
;Alex   Kim z5162106 

.include "m2560def.inc"

;macro section
.macro do_lcd_command
	ldi   r16, @0
	rcall lcd_command
	rcall lcd_wait
.endmacro

.macro do_lcd_data
	ldi   r16, @0
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro do_lcd_rdata
	mov   r16,@0
	subi  r16,-'0'
	rcall lcd_data
	rcall lcd_wait
.endmacro

.macro lcd_set
	sbi PORTA, @0
.endmacro

.macro lcd_clr
	cbi PORTA, @0
.endmacro

.macro clear 
	ldi YL,low(@0)  ;load the memory address to Y
	ldi YH,high(@0)
	clr temp
	st  Y+,temp  ;clear the two bytes at @0 in SRAM
	st  Y,temp
.endmacro

.macro show_LED
	mov   temp,@0	
	rcall s_LED
.endmacro

.macro change_flag
	mov  temp1,@0
	cpi  temp1,1
	breq be_zero
	rjmp change_to_one
	be_zero:
		ldi  temp1,0
		rjmp end_change_flag
	change_to_one:
		ldi  temp1,1
		rjmp end_change_flag

	end_change_flag:
		mov @0,temp1
.endmacro
 

.equ initial_Floor      = 1
.equ initial_next_Floor = 9
.equ i_direction        = 1    ;  1= up, 0=down
.equ PORTLDIR           = 0xF0 ; PL7-4: high  PL3-0,low
.equ INITCOLMASK        = 0xEF 
.equ INITROWMASK        = 0X01  
.equ ROWMASK            = 0x0F

.equ LCD_RS = 7
.equ LCD_E  = 6
.equ LCD_RW = 5
.equ LCD_BE = 4

;define register number 
.def count             = r1 ;
.def emergency_flag    = r2	;use for emergency motor control
.def open_flag         = r3	;record open push button pressed
.def close_flag        = r4	;record close push button pressed
.def Size_of_array_new = r5
.def Size_of_array     = r6
.def queueFloor        = r7
.def position          = r8
.def startFloor        = r9 
.def NextFloor         = r10

.def row   = r16
.def col   = r17
.def rmask = r18
.def cmask = r19

.def temp  = r20
.def temp1 = r21
.def direction = r22
.def c_level  = r23

.dseg
EmergencyCounter:
	.byte 2
array:
	.byte 30
TempCounter:
	.byte 2

.cseg

.org 0x0000	
	jmp  RESET

.org INT0addr
	rjmp EXI_INT0  ; do_close

.org INT1addr
	rjmp EXI_INT1  ; do_open

.org OVF0addr
	rjmp Timer0OVF ; jump to the interrupt handler for Timer0 overflow
	jmp  DEFAULT
 sort_array: .db	 8,10,"\0"	;'\0'		;i_size for this 6
			 
			 .set	 i_size = 2
DEFAULT: reti


//........................................RESET........................................\\
RESET:
	clr temp
	clr temp1
	clr emergency_flag
	clr close_flag
	clr open_flag
;......................................	 //array setup
	ldi	zh, high(sort_array << 1)
	ldi	zl, low(sort_array << 1)

	ldi temp,low(RAMEND)
	out SPL,temp
	ldi temp,high(RAMEND)
	out SPH,temp

	ldi		yh, high(array)
	ldi		yl, low(array)

	Initial_array:
	lpm		temp, z+
	mov		queueFloor,temp
	cpi		temp, 'a'
	breq	re_set

	mov		temp,queueFloor
	st		y+, temp

	mov		temp,Size_of_array
	inc		temp
	mov		Size_of_array,temp

	rjmp	Initial_array
re_set:
	sbiw	y, i_size										;!!! same with num of s_a
	mov		Size_of_array_new, Size_of_array	

;LED bar setup...........................
	ser  temp
	out  DDRC,temp	   ;open lower 8 LED
	out  DDRG,temp     ;open upper 2 LED 
	clr  temp
	out  PORTC,temp
	out  PORTG,temp
;LCD setup...............................
	ser  temp
	out  DDRF, temp
	out  DDRA, temp
;keypad setup............................
	ldi  temp1,PORTLDIR
	sts  DDRL,temp1
;Push botton setup.......................
	// initial DDRD , port D is a input pin 
	clr  temp 
	sts  DDRD, temp
;Active Motor
	ser  temp
	out  DDRE,temp

	ldi   temp,initial_next_Floor
	mov   NextFloor,temp
	ldi   c_level,initial_Floor
	rcall option1
	show_LED c_level

	jmp main
;\\...............................RESET.............................//


;-------------------------------------------------------------------
array_order:
	push	position			//r9
	push	temp				//
	push	temp1

start:
	mov		startFloor, c_level		

main_body_of_array_order:	
	;mov		temp,queueFloor	;keypad scan-temp 
	

	rcall	insert_request
	mov		Size_of_array, Size_of_array_new

	pop temp1
	pop	temp					//keypad scan
	pop	position				//r9
	ret

;----------------------------------------------------------------------------------------------------------
insert_request: ;store pressed keypad number
	;		prologue
	push	yh
	push	yl
;	push	queueFloor								; Insert_value			r10 mov to cur

;	push	Size_of_array								; could change here
;	push	startFloor
	push	position									; counter
;	push	NextFloor
	push	r5
	push	r4
	push	temp
	push	temp1


	
	
judgement:
	
	ld		temp1, y									;point to initial value of array
	mov		NextFloor,temp1

	mov		temp1,startFloor
	mov		temp,NextFloor
	cp		temp1, temp
	
	brlo	f_up_position
	breq	to_Floor_reached
	brge	f_down_position

to_Floor_reached:
	jmp Floor_reached


;------------------------------------	
f_up_position:

	ld		temp1, y+	
	mov		NextFloor,temp1								;up first ,so find position of max value

	ld		temp1, y
	mov		r5,temp1

	mov		temp1,position
	inc		temp1									;counter
	mov		position,temp1

	mov		temp1,NextFloor
	mov		temp,r5
	cp		temp1, temp
	brlo	f_up_position

f_up:
	mov		temp1,startFloor
	mov		temp,queueFloor
	cp		temp1, temp
	
	brlo	move_up1
	breq	to_end_order
	brsh	move_down1

to_end_order:
	jmp end_order

	move_up1:
			ld		temp1, -y
			mov		NextFloor,temp1

			mov		temp1,position
			cpi		temp1, 0	;
			breq	to_Store_pre1
			mov		temp1,position
			dec		temp1
			mov		position,temp1

			mov		temp1,queueFloor
			mov		temp,NextFloor
			cp		temp1, temp
			brlo	move_up1
			breq	to_end_order
			brsh	Store_pre1
			;dec		position
					
	move_down1:
			ld		temp1, y+
			mov		NextFloor,temp1
			
			mov		temp1,queueFloor
			mov		temp,NextFloor
			cp		temp1, temp

			breq	to_end_order
			brsh	Store_pre2
			
			mov		temp1,position
			mov		temp,Size_of_array
			cp		temp1, temp

			breq	Store_pre2 

			mov		temp1,position
			inc		temp1
			mov		position,temp1

			rjmp	move_down1

to_Store_pre1:
	jmp Store_pre1

;-------------------------------------	
f_down_position:
	ld		temp1, y+
	mov		NextFloor,temp1

	ld		temp1, y
	mov		r5,temp1

	mov		temp1,position
	inc		temp1
	mov		position,temp1

	mov		temp1,NextFloor
	mov		temp,r5
	cp		temp1, temp

	brsh	f_down_position

f_down:
	mov		temp1,startFloor
	mov		temp,queueFloor
	cp		temp1, temp

	brlo	move_up2
	breq	to_end_order
	brsh	move_down2

	move_down2:
			ld		temp1, -y
			mov		NextFloor,temp1

			mov		temp1,position
			cpi		temp1, 0
			breq	Store_pre1

			mov		temp1,position
			dec		temp1
			mov		position,temp1

			mov		temp1,queueFloor
			mov		temp,NextFloor
			cp		temp1, temp
			
			breq	end_order
			brlo	Store_pre1
			brsh	move_down2
	
	move_up2:
			mov		temp1,NextFloor
			ld		temp1, y+	
			cp		queueFloor, NextFloor

			breq	end_order
			brlo	Store_pre2
			
			mov		temp1,position
			mov		temp,Size_of_array
			cp		temp1, temp
			
			breq	Store_pre2

			mov		temp1,position
			inc		temp1
			mov		position,temp1

			rjmp	move_up2	
;--------------------------------------
Store_pre1:						; prepare for -y
	mov		temp1,Size_of_array
	inc		temp1
	mov		Size_of_array,temp1
		
	adiw	y, 1
	rjmp	Store
	
Store_pre2:						; prepare for y+
	mov		temp1,Size_of_array
	inc		temp1
	mov		Size_of_array,temp1

	mov		temp1,position
	dec		temp1
	mov		position,temp1

	sbiw	y, 1
	
	rjmp	Store
	
	Store:
			ld		temp1, y
			mov		r5,temp1
			mov		temp1,queueFloor
			st		y+, temp1
			mov		queueFloor, r5


			mov		temp1,position
			mov		temp,Size_of_array
			cp		temp1, temp

			breq	end_order

			mov		temp1,position
			inc		temp1
			mov		position,temp1
			
			rjmp	Store

	Floor_reached:
			mov		temp1,Size_of_array
			add		yl,temp1
			ld		temp1,-y
			mov		temp1,r5
			
			mov		temp1,r4
			st		y,temp1
						
		order:
			
			ld		temp1, -y				
			mov		r4,temp1
			mov		temp1,r5
			st		y, temp1
			mov		r5, r4	
			
			mov		temp1,position
			mov		temp,Size_of_array
			cp		temp1, temp

			breq	end_order1

			mov		temp1,position
			inc		temp1
			mov		position,temp1
			
			rjmp	order
end_order1:
	dec		Size_of_array_new
	jmp		end_order

end_order:

	mov		Size_of_array_new, Size_of_array
	
	pop		temp1
	pop		temp
	pop		r4
	pop		r5

	pop		position

	pop		yl
	pop		yh
	ret

//.........................Timer0.................................................\\ 
Timer0OVF:
	push  temp
	in    temp,SREG
	push  temp  
	push  YH	   
	push  YL
	push  r25
	push  r24

check_second_set:
	lds  r24,TempCounter
	lds  r25,TempCounter+1
	adiw r25:r24,1  ;increase the temporary counter by one 
 
	rcall keypad	   ; each cycle read keypad 
	;rcall array_order
	
	
;firstly, check emergency button
	mov	  temp, emergency_flag		;read emergency flag
	cpi	  temp, 1
	breq  go_Emergency		;mean emergency is pressed
	rjmp  keep_timing		;if not equal keep counting time

	go_Emergency:
		jmp Emergency

	keep_timing:
		cpi  r24, low(15624)	  ; 2 second 7812 x 2 = 15624
		ldi  temp,high(15624)     ;15624
		cpc  r25,temp
		brne go_NotSecond
		rjmp secondPassed
		
	go_NotSecond:
		jmp  NotSecond
	;if 1/4 second has passed, do secondPassed
		secondPassed:
			cp   c_level,NextFloor
			breq go_stop
			brlo go_up
			jmp  go_down  


//.................................................................//			
		go_up:
			cpi    c_level,10	 ;check whether at max level
			breq   dir_down       ;if at level 10, change direction
			inc    c_level
			rcall  option1		; show LCD
			rcall  keypad	    ;scan keypad

			show_LED c_level	; show LED 
			
			jmp  loop_finish
		
		go_down:
			cpi    c_level,1     ;check whether at lowest level
			breq   dir_up        ;change direciton
			dec    c_level
			rcall  option1
			rcall  keypad
			show_LED c_level

			jmp loop_finish
		
		dir_down: ;change direction to down
			ldi	  direction, 0		 ;0 is down
			jmp   secondPassed

		dir_up:	  ;change direction to up
			ldi	  direction, 1		 ;1 is up
			jmp   secondPassed
		
		loop_finish:												; once move finis,clear Timecounter
			clr     temp
			mov     count,temp
			clear   TempCounter
			rjmp    EndIF
 //................................................................//
	go_stop:
			;<blink LED Bar>
			;<open motor> <halt 3 second><close door>
			;<read next number>  
		clr   temp
		clr   temp1
		out   PORTC,temp
		out   PORTG,temp1

		mov   temp,open_flag	  ;read open and close flag
		cpi   temp,1
		breq  need_open
		mov   temp,close_flag
		cpi   temp,1
		breq  need_close
		jmp   step1_blink_and_roll	;if no flag deteted, follow normal process

	need_open:	    ;when close button press, jump to do open process
		ldi   temp,1
		mov   count,temp
		clr	  temp
		mov   open_flag,temp
		mov   close_flag,temp

		jmp   step1_record_count

	need_close:		 ;when close button press, jump to do close process
		ldi   temp,5
		mov   count,temp
		clr	  temp
		mov   open_flag,temp
		mov   close_flag,temp

		jmp   step1_record_count

		step1_blink_and_roll:
				mov   temp,count
				inc   temp
				mov   count,temp
				;rcall show_counter	    ;for testing the length of time of opening and closing
			step1_record_count:
				;rcall show_counter
				mov   temp,count
				cpi   temp,1
				breq  step1_open_motor  ;open door, start motor	with opening motor speed
				
				cpi  temp,2
				breq step1_stop_motor
				
				cpi  temp,3
				breq step1_blink_set

				cpi  temp,4
				breq  step1_blink_set
					
				cpi  temp,5
				breq step1_close_motor  ;close door, start motor with closing motor speed

				cpi  temp,6
				breq finish_stop_motor

		finish_stop_motor:		
				rcall   zero_speed       ;turn off motor
				show_LED c_level
				rcall   option1
				clr     temp
				mov     count,temp
				
				rcall keypad
				ldi   temp,2			 ;load nextfloor number from sorted array
				mov   NextFloor,temp	 ;

				clr	r26
				clr	r27
				pop r24
				pop r25
				pop YL
				pop YH
				rjmp loop_finish

		    step1_open_motor:
				rcall open_speed
				rjmp  step1_blink_set
			step1_close_motor:
				rcall close_speed	
				rjmp  step1_blink_set
			step1_stop_motor:
				rcall zero_speed
				rjmp  step1_blink_set
	   
step1_blink_set:   ;hald second past, display current floor on LED bar
		show_LED c_level
		adiw   r27:r26,1
	   	
		cpi	  r26, low(3906)
		ldi	  temp, high(3906)
		cpc	  r27, temp
		brne  step1_blink_set

step_blink_set2:   ;half second past, clear LED bar
	    clr   r26
		clr   r27
	 step1_blink:
		rcall  keypad

		adiw  r27:r26,1

		clr   temp
		out   PORTC,temp
		out   PORTG,temp
		
		cpi	  r26,  low (3906)	 ;half second 
		ldi	  temp, high(3906)
		cpc	  r27, temp
		brne  step1_blink
		
		clr   r26
		clr   r27
		rjmp  EndIF
 //....................................................................//


//....................................................................//
Emergency:
	rcall   show_emergency_LCD  ;display "Emergency Call 000"
	sbi		PORTA, 1            ;turn on LED bulb 
	ldi     temp,0
	mov     count,temp
	Emergency_set:
		push r24
		push r25

		clr	 r24
		clr	 r25
		clr	 emergency_flag
		clr	 temp
		clr	 temp1
		mov  count,temp
	E_move_set:	 ;prepare to go to 1st floot
		push yh
		push yl

		lds	 r24, EmergencyCounter
		lds	 r25, EmergencyCounter +1
		adiw r25:r24, 1
		
		cpi	 r24,  low (7812)	  ;one second
		ldi	 temp, high(7812)
		cpc	 r25, temp
		brne not_second_E
	
		;reach 1second
		cpi  c_level,0
		breq blink_and_roll  ;at level 1 ,start blink LED and motor
	
		dec   c_level		 ;going down
		rcall sleep_1s	     ;delay 2 second for each floor
		rcall sleep_1s	     
		show_LED c_level     ;display floor LED
	
		clear EmergencyCounter
		rjmp  E_end1
   not_second_E:
		sts		EmergencyCounter, r24
		sts		EmergencyCounter +1, r25
		rjmp	E_end1

	blink_and_roll:
		mov temp,count
		inc temp
		mov count,temp
		;rcall show_counter	 ;for testing the length of time of opening and closing

	record_count:
		mov  temp,count
		cpi  temp,1
		breq open_motor   ;open door ,start motor with opening motor speed
		cpi  temp,2
		breq stop_motor
		cpi  temp,5
		breq close_motor  ;close door, start motor with closing motor speed
		cpi  temp,6
		breq stop_motor
		rjmp blink_set    ;blink forever, waiting for the second emergency pressed.
	
	;setting motor speed	
		open_motor:
			rcall open_speed  ;motor run with high speed
			rjmp  blink_set
		close_motor:
			rcall close_speed ;motor run with low speed	
			rjmp  blink_set
		stop_motor:
			rcall zero_speed  ;turn off motor
			rjmp  blink_set

	blink_set:		  ;blink LED bar 
		clr    emergency_flag
		rcall  keypad  ;at floor 1,read keypad
		mov    temp,emergency_flag	   ;check whether star button press 2 times
		cpi    temp,1
		breq   true_end2
		adiw   r27:r26,1
		lcd_clr 1
		ldi	   temp, 0			;turn off led
		out	   PORTC, temp
		cpi	   r26,  low (3906)
		ldi	   temp, high(3906)
		cpc	   r27, temp
		brne   blink_set

	 blink_set2:  ;blink LED bulb
			clr		r26
			clr		r27
		blink:
			clr		emergency_flag
			rcall	keypad 
			mov		temp, emergency_flag  ;check whether star button press 2 times
			cpi		temp, 1
			breq	true_end2
			adiw	r27:r26, 1
			lcd_set 1
			ldi		temp, 1		 ;turn on led
			out		PORTC, temp
			cpi		r26,  low (3906)
			ldi		temp, high(3906)
			cpc		r27, temp
			brne	blink
			clr		r26
			clr		r27
			rjmp	blink_and_roll

	
	E_end1:	
		clr     temp
		mov     count,temp
		pop		yl
		pop		yh
		rjmp	E_move_set
		
 true_end2:
	
	 E_end2:
		clr		emergency_flag
		ldi		temp, 1
		out		PORTC, temp
		lcd_clr	1
		do_lcd_command 0b00000001  ;clear LCD
		ldi     c_level,1
		rcall   option1

	clr	r26
	clr	r27
	pop	yl
	pop	yh
	pop	r25
	pop	r24
	jmp	NotSecond	;go back normal operation

 
end_secondPassed:
			clear TempCounter
			rjmp  EndIF
;.........................................................//
NotSecond:
	sts TempCounter,r24
	sts TempCounter+1,r25
	jmp EndIF	
	
EndIF:
	pop r24
	pop r25
	pop YL
	pop YH
	pop temp
	out SREG,temp
	pop temp
	reti   ;return from the interrupt

//........................................................//
;External Interrupt 0 and 1

EXI_INT1:  ;do_open_process
	push  temp
	in    temp,SREG
	push  temp
	rcall debounce
	;lcd_set 1	
	ldi   temp,1
	mov   open_flag,temp
	pop   temp
	out   SREG,temp
	pop   temp
reti
//........................................................//
EXI_INT0:  ;do_close_process
	push  temp
	in    temp,SREG
	push  temp
	rcall debounce
	;lcd_clr 1
	ldi   temp,1
	mov   close_flag,temp
End_close:
	pop   temp
	out   SREG,temp
	pop   temp
reti
//.......................................................//

;main section
main:
	clr   temp
	clr   temp1	
	clear EmergencyCounter
	clear TempCounter
	ldi   direction,i_direction
	ldi   c_level, initial_Floor

;set INT0
	ldi temp,(1<<ISC01)|(0<<ISC00) ;set INT0 as low level /falling-edge trigged interrupt
	sts EICRA,temp
	in  temp,EIMSK
	ori temp,(1<<INT0) ; enable External Interrupt 0
	out EIMSK,temp

;set INT1
	ldi temp,(1<<ISC11)|(0<<ISC10) ;set INT1 as falling-edge trigged interrupt
	sts EICRA,temp
	in  temp,EIMSK
	ori temp,(1<<INT1) ; enable External Interrupt 1
	out EIMSK,temp

;set Timer0
	ldi temp,0b00000000
	out TCCR0A, temp
	ldi temp,0b00000010 ;prescaling value = 8 
	out TCCR0B, temp	; = 128 microseconds
	ldi temp, 1<<TOIE0  ;enable T/C0 Interrupt 
	sts TIMSK0, temp

;set PWM duty cycle	
	ldi temp,0x00
	sts OCR3BL, temp ;low register
	clr temp
	sts OCR3BH, temp ;high register

	ldi temp,(1<<CS30)
	sts TCCR3B,temp
	ldi temp,(1<<WGM30)|(1<<COM3B1)
	sts TCCR3A,temp

	sei
halt:
	rjmp halt
;\\...............................main............................................//	
keypad:
	push  cmask
	push  rmask
	push  col
	push  row
	push  r26
	push  r27

	clr	  cmask
	clr	  rmask
	clr	  col
	clr	  row

	ldi   cmask,INITCOLMASK ;initial column mask
	clr   col				  ;initial column

colloop:
	cpi  col,4
	breq convert_end; if all keys are scanned,repeat.
	sts  PORTL,cmask
;...........................slow down the scan operation	
	ldi  temp,0xFF
delay:
	dec  temp
	brne delay

	lds  temp,PINL	;read PORTL
	andi temp,ROWMASK ;Get the keypad output value
	cpi  temp,0xF
	breq nextcol
	; If yes, find which row is low
	ldi  rmask,INITROWMASK 
	clr  row

rowloop:
	cpi  row,4
	breq nextcol  ;the row scan is over
	mov  temp1,temp
	and  temp1,rmask 
	breq convert
	inc  row
	lsl  rmask
	jmp  rowloop

nextcol:
	lsl  cmask
	inc  col 
	jmp  colloop
	
convert: ;convert to decimal number
	cpi  row,3
	breq symbols

	mov  temp,row
	lsl  temp
	add  temp,row
	add  temp,col	;temp1 = row*3 +col
	subi temp,-1	
	jmp  save_request

symbols:
	cpi  col,0
	breq star ;emergency
	cpi  col,1
	breq Zero
	jmp  convert_end

star:
	clr  emergency_flag
	inc  emergency_flag	  ;when star pressed , emergency flag change to 1
	jmp  finial_counter_set

zero:
	ldi  temp,10
	mov  NextFloor,temp
	jmp  convert_end

save_request:
	mov  NextFloor,temp

	jmp  convert_end

convert_end:
	clr  emergency_flag
	jmp  final

finial_counter_set:
	rcall sleep_100ms
	rcall sleep_100ms
final:
	pop	 r27
	pop  r26
	pop	 row
	pop	 col
	pop	 rmask
	pop	 cmask
ret

;funciton	
show_emergency_LCD:
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	call sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?  Yes,
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00001110 ; turn on display and cursor
	do_lcd_command 0b00000110 ; increment, no display shift ;每输入一次就空一个
	do_lcd_command 0b00001100 ; Cursor on, bar, no blink
;first line
	do_lcd_data 'E'
	do_lcd_data 'm'
	do_lcd_data 'e'
	do_lcd_data 'r'
	do_lcd_data 'g'
	do_lcd_data 'e'
	do_lcd_data 'n'
	do_lcd_data 'c'
	do_lcd_data 'y'
;LCD second line 	
	do_lcd_command 0b11000000
	do_lcd_data 'C'
	do_lcd_data 'a'
	do_lcd_data 'l'
	do_lcd_data 'l'
	do_lcd_data ' '
	do_lcd_data '0'
	do_lcd_data '0'
	do_lcd_data '0'
ret

lcd_command:
	out   PORTF, r16
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	ret

lcd_data:
	out   PORTF, r16
	lcd_set LCD_RS
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	lcd_clr LCD_E
	rcall sleep_1ms
	lcd_clr LCD_RS
	ret

lcd_wait:
	push  r16
	clr   r16
	out   DDRF, r16
	out   PORTF, r16
	lcd_set LCD_RW
lcd_wait_loop:
	rcall sleep_1ms
	lcd_set LCD_E
	rcall sleep_1ms
	in r16, PINF
	lcd_clr LCD_E
	sbrc  r16, 7
	rjmp  lcd_wait_loop
	lcd_clr LCD_RW
	ser   r16
	out   DDRF, r16
	pop   r16
	ret

//...................................functions section.............................................\\
;sleep section
.equ F_CPU = 16000000
.equ DELAY_1MS = F_CPU / 4 / 1000 - 4
.equ DELAY_1S = F_CPU / 4
; 4 cycles per iteration - setup/call-return overhead

sleep_1ms:
	push r24
	push r25
	ldi  r25, high(DELAY_1MS)
	ldi  r24, low(DELAY_1MS)
delayloop_1ms:
	sbiw r25:r24, 1
	brne delayloop_1ms
	pop  r25
	pop  r24
	ret
sleep_5ms:
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	rcall sleep_1ms
	ret	

sleep_20ms:
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	rcall sleep_5ms
	ret
sleep_100ms:
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_20ms
	ret

sleep_250ms:
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_20ms
	rcall sleep_20ms
	rcall sleep_5ms
	rcall sleep_5ms
	ret


sleep_1s:
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	rcall sleep_100ms
	ret
	
debounce:
	rcall sleep_100ms
	rcall sleep_100ms
	ret		
//........................................................
close_speed:
	ldi temp,0x33    ;full speed 20%speed || 255*0.2 =51(DEC)=>33(Hex)
	sts OCR3BL, temp ;low register
	clr temp
	sts OCR3BH, temp ;high register
ret

open_speed:
	ldi temp,0xFF    ;full speed
	sts OCR3BL, temp ;low register
	clr temp
	sts OCR3BH, temp ;high register
ret

zero_speed:
	ldi temp,0x00    ;zero speed
	sts OCR3BL, temp ;low register
	clr temp
	sts OCR3BH, temp ;high register
ret

s_LED:
	cpi  temp,0
	breq to_zero
	cpi  temp,1
	breq to_one
	cpi  temp,2
	breq to_two
	cpi  temp,3
	breq to_three
	cpi  temp,4
	breq to_four
	cpi  temp,5
	breq to_five
	cpi  temp,6
	breq to_six
	cpi  temp,7
	breq to_seven
	cpi  temp,8
	breq to_eight
	cpi  temp,9
	breq to_nine
	cpi  temp,10
	breq to_ten
to_zero:
	jmp  toto_zero
to_one:
	jmp  one
to_two:
	jmp  two
to_three:
	jmp  three
to_four:
	jmp  four
to_five:
	jmp  five
to_six:
	jmp  six
to_seven:
	jmp  seven
to_eight:
	jmp  eight
to_nine:
	jmp  nine
to_ten:
	jmp  ten

toto_zero:
	ldi  temp ,0b00000000
	ldi  temp1,0b00000000
	jmp  light_led
one:
	ldi  temp ,0b00000001
	ldi  temp1,0b00000000
	jmp  light_led
two:
	ldi  temp ,0b00000011
	ldi  temp1,0b00000000
	jmp  light_led
three:
	ldi  temp ,0b00000111
	ldi  temp1,0b00000000
	jmp  light_led
four:
	ldi  temp ,0b00001111
	ldi  temp1,0b00000000
	jmp  light_led
five:
	ldi  temp ,0b00011111
	ldi  temp1,0b00000000
	jmp  light_led
six:
	ldi  temp ,0b00111111
	ldi  temp1,0b00000000
	jmp  light_led
seven:
	ldi  temp ,0b01111111
	ldi  temp1,0b00000000
	jmp  light_led
eight:
	ldi  temp ,0b11111111
	ldi  temp1,0b00000000
	jmp  light_led
nine:
	ldi  temp ,0b11111111
	ldi  temp1,0b00000001
	jmp  light_led
ten:
	ldi  temp ,0b11111111
	ldi  temp1,0b00000011
	jmp  light_led

light_led:
	out  PORTC,temp
	out  PORTG,temp1
ret

option1:  ;LCD show floor number
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?  Yes,
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00001110 ; turn on display and cursor
	do_lcd_command 0b00000110 ; increment, no display shift ;每输入一次就空一个
	do_lcd_command 0b00001100 ; Cursor on, bar, no blink	
;first line	
	do_lcd_data 'C'
	do_lcd_data 'u'
	do_lcd_data 'r'
	do_lcd_data 'r'
	do_lcd_data 'e'
	do_lcd_data 'n'
	do_lcd_data 't'
	do_lcd_data ' '
	do_lcd_data 'F'
	do_lcd_data 'l'
	do_lcd_data 'o'
	do_lcd_data 'o'
	do_lcd_data 'r'
	do_lcd_data ' '
	
	;ldi temp1,initial_Floor
	cpi  c_level,10
	breq show_10
	do_lcd_rdata c_level
	show_LED c_level

	jmp  second_line

show_10:
	do_lcd_data '1'
	do_lcd_data '0'		
	
second_line:
;LCD second line 	
	do_lcd_command 0b11000000
	do_lcd_data 'N'
	do_lcd_data 'e'
	do_lcd_data 'x'
	do_lcd_data 't'
	do_lcd_data ' '
	do_lcd_data 's'
	do_lcd_data 't'
	do_lcd_data 'o'
	do_lcd_data 'p'
	do_lcd_data ' '

	mov  temp1,NextFloor
	cpi  temp1,10
	breq show_second_10
	do_lcd_rdata temp1

	jmp  end_option1

show_second_10:
	do_lcd_data '1'
	do_lcd_data '0'
	
	end_option1:
ret

;test coding
show_counter: ;for test	  ,to show current open and close stage 
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_5ms
	do_lcd_command 0b00111000 ; 2x5x7
	rcall sleep_1ms
	do_lcd_command 0b00111000 ; 2x5x7
	do_lcd_command 0b00001000 ; display off?  Yes,
	do_lcd_command 0b00000001 ; clear display
	do_lcd_command 0b00001110 ; turn on display and cursor
	do_lcd_command 0b00000110 ; increment, no display shift
	do_lcd_command 0b00001100 ; Cursor on, bar, no blink	

	do_lcd_rdata   count
ret	