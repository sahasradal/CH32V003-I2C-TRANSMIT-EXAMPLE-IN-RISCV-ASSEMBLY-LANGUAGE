#  works fine as SLAVE .receives 8 bytes , stores in result1/result2. then transmits back 8 bytes.no memory address selection , block write/read
#BRR = BAUD  
#13.6=38400
#4.4 = 115200
#52.1 = 9600
#8.7 =  57600
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs
#USARTx_RX Full-duplex mode Floating input or pull-up input
#own address = 0x6F  , 0xDE write,0xDF read
#slave_address_write = 0xA0
#slave_address_read  = 0xA1
#buffer = 0x20000004
#system clock is internal RC oscillator 24mhz, default PCLK - 24/3 = 8 Mhz
#FRAM_addressW = 0xA0	,added to include file ch32v003_reg1.asm
#FRAM_addressR = 0xA1   ,added to include file ch32v003_reg1.asm
# TESTED CODE ON FUJITSU MB85RC256V memory FRAM and found to trasmit data
# no interrupts so we start from flash 0x8000000 directly 
#=======================================================================================    
#I2C_SCL = PC2			# I2C  clock on PC2 (reccomended external pullup 4.7k)
#I2C_SDA = PC1			# I2C  data on PC1 (reccomended external pullup 4.7k)
#UART_TX = PD5 
#UART_RX = PD6 
#========================================================================================



fclk = 24000000   # 24Mhz RCO internal , AHB =8Mhz by default
buffer0 = 0x20000004
statusreg = 0x20000004
buffer1 = 0x20000008
result_low = 0x20000004
result_hi = 0x20000008
state = 0x2000000C
result1 = 0x20000010
result2 = 0x20000014
dividend = 0x20000018 
divisor = 0x2000001C
scratch = 0x20000020
mem = 0x20000024

include CH32V003_reg1.asm

vtable:
	j reset_handler		#  longs 0x00000000 # RESERVED 0
align 4
  longs   0x00000000 # RESERVED 1
  longs   0x00000000 #pack <l longs NMI_IRQhandler
  longs   0x00000000 #pack <l HardFault_IRQhandler
  longs   0x00000000 # RESERVED 4
  longs   0x00000000 # RESERVED 5
  longs   0x00000000 # RESERVED 6
  longs   0x00000000 # RESERVED 7
  longs   0x00000000 # RESERVED 8
  longs   0x00000000 # RESERVED 9
  longs   0x00000000 # RESERVED 10
  longs   0x00000000 # RESERVED 11
  longs   0x00000000 # pack <l SysTick_IRQhandler	#; place the address of the mtime ISR subroutine in the vector table position 7,assembler will store isr address here, longs 0x00000000 # RESERVED 12	
  longs   0x00000000 # RESERVED 13
  longs   0x00000000 #pack <l SW_Software_IRQhandler
  longs   0x00000000 # RESERVED 15
  longs   0x00000000 #pack <l WWDG_IRQhandler
  longs   0x00000000 #pack <l PVD_IRQhandler
  longs   0x00000000 #pack <l FLASH_IRQhandler
  longs   0x00000000 #pack <l RCC_IRQhandler
  longs   0x00000000 #pack <l EXTI7_0_IRQhandler
  longs   0x00000000 #pack <l AWU_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH1_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH2_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH3_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH4_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH5_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH6_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH7_IRQhandler
  longs   0x00000000 #pack <l ADC1_IRQhandler
pack <l I2C1_EV_IRQhandler
pack <l I2C1_ER_IRQhandler
  longs   0x00000000 #pack <l USART1_IRQhandler
  longs   0x00000000 #pack <l SPI1_IRQhandler
  longs   0x00000000 #pack <l TIM1BRK_IRQhandler
  longs   0x00000000 #pack <l TIM1UP_IRQhandler
  longs   0x00000000 #pack <l TIM1TRG_COM_IRQhandler
  longs   0x00000000 #pack <l TIM1CC_IRQhandler
  longs   0x00000000 #pack <l TIM2_IRQhandler

reset_handler:


    	li sp, STACK			# load stack pointer with stack end address
	 
    	li t0, vtable			#BASEADDR[31:2],The interrupt vector table base address,which needs to be 1KB aligned
    	ori t0, t0, 3			#BASEADDR[31:2],1: Identify by absolute address,1: Address offset based on interrupt number *4
    	#csrrw zero,t0, mtvec		# write to mtvec
	longs 0x30529073  
    
   	li t0,main
	longs 0x34129073          	#csrw	mepc,t0 :mepc updated with address of main
	longs 0x30200073		# mret ( return from interrupt)	.
  
	align 4
main:
	
	li x10,buffer0			# clear
	sw x0,0(x10)
	li x10,statusreg		# clear
	sw x0,0(x10)
	li x10,result1			# clear
	sw x0,0(x10)
	li x10,result2			# clear
	sw x0,0(x10)

###########
#enable periphrel clocks
	li x10,R32_RCC_APB2PCENR	# load address of APB2PCENR register to x10 ,for enabling GPIO A,D,C peripherals
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB2PCENR pointed by x10
	li x7,((1<<0)|(1<<2)|(1<<4)|(1<<5)|(1<<14)|(1<<9))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<USART_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR

#Enable I2C clock in  APB1 register
    	li x10,R32_RCC_APB1PCENR	# load address of APB1PCENR register to x10 ,for enabling I2C peripheral
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB1PCENR pointed by x10
	li x7,(1<<21)			# 1<<I2C1_EN, = 1<<21 for I2C functions
	or x11,x11,x7			# or values 
	sw x11,0(x10)			

####################

#configure GPIO 
	li x10,R32_GPIOD_CFGLR		# load pointer x10 with address of R32_GPIOD_CFGLR , GPIO configuration register
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<20)|(0xf<<24))	#clear pd6,pd5,pd4,pd3. we need to setup PD5 & PD6 for usart tx and rx and pd4,pd3 for ADC
	and x11,x11,x7			# clear pd4,pd5,pd6 mode and cnf bits for selected pin D4,D5,D6
	li x7,((0x8<<24)|(0xB<<20))	# pd6 = input with PU/PD,pd5= multiplex pushpull output 50mhz,pd4,pd3= analoge input 0b0000 (AIN4,AIN7)
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR

#enable pull up for input	
	li x10,R32_GPIOD_OUTDR		# enable pullup resistor by setting OUTDR register
	lw x11,0(x10)			# this setting of GPIO_OUTDR for pullup resistor effects if corresonding pin is selected as input
	li x7,(1<<6)			#when PD6 is input with resistor selected 1= pullup and 0 = pulldown
	or x11,x11,x7
	sw x11,0(x0)

#configure GPIO PortC as multiplex open drain output
	li x10,R32_GPIOC_CFGLR		# load pointer x10 with address of R32_GPIOC_CFGLR , I2C SDA & SCL is on portC PC1,PC2
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<4)|(0xf<<8))	# clear pc1,pc2. we need to setup PC1 & PC2 for I2C
	and x11,x11,x7			# clear  mode and cnf bits for selected pin C1,C2
	li x7,((13<<4)|(13<<8))		# PC1 = multiplex open drain output 10mhz ,PC2= multiplex open drain output 10mhz, 0b1101
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR

#####################

#configure USART baud
	li x10,R32_USART_BRR		# USART BAUD setting
	lw x11,0(x10)			# copy R32_USART_BRR to x11
	li x7,((52<<4)|(1<<0))		# 52.1 in BRR =9600
	or x11,x11,x7			# or registers
	sw x11,0(x10)			# store in R32_USART_BRR

#setup UART control and enable	
	li x10,R32_USART_CTLR1		# load x10 with R32_USART_CTLR1 address
	lw x11,0(x10)			# load to x11 contents
	li x7,(1<<13)|(1<<3)|(1<<2)	# enable USART UE, TX,RX bits		# UE 
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

##########################

#I2C0 configuration 
	li x10,R32_RCC_APB1PRSTR	# set pointer to clock control  peripheral reset register 
	lw x11,0(x10)			# load contents to x11
	li x7,(1<<21)			# shift 1 to 21st bit position
	or x11,x11,x7			# OR with x11
	sw x11,0(x10)			# set bit 21 of R32_RCC_APB1PRSTR to reset I2C peripheral
	not x7,x7			# invert values in x7
	and x11,x11,x7			# and x11 to write a 0 in 21st bit
	sw x11,0(x10)			# store 0 in 21st bit to restart i2c engine
	
	li x10,R16_I2C_CTLR2		# set clock in control 2 register
    	lh x11,0(x10)			# copy contents of R16_I2C_CTLR2 to x11
	li x7,0xffffffc0		# clear frequency bits 0-5 with bit mask 0xffffffc0
	and x11,x11,x7			# AND will clear bit 0-5
    	li x7,(8<<0)			# 8Mhz I2C clock .default 24Mhz HSI/3 =8Mhz APB clock
    	or x11,x11,x7			# store APB clock frequency in bit 0-5
	sh x11,0(x10)			# store back in R16_I2C_CTLR2

	li x10,R16_I2C_CTLR2		# set clock in control 2 register
    	lh x11,0(x10)			# copy contents of R16_I2C_CTLR2 to x11
	li x7,(1<<10)|(1<<9)|(1<<8)	# load x7 with bits to enable buffer,event and error interrupts
	or x11,x11,x7			# OR the bits with x11
    	sh x11,0(x10)			# store back in R16_I2C_CTLR2

    	li x10,R16_I2C_CKCFGR		# set pointer to I2C clockregister
    	lh x11,0(x10)			# copy values to x11 from above register
	li x7,0xfffff000		# clear CCR bits 0-11 with bitmask 0xfffff000
	and x11,x11,x7			# ANDing clears bit 0-11 in x11 register
	li x7,(40<<0)			# CCR = t(rscl)+t(wsclh)/tpclk1 = 1000+4000/125 =40 , or (8000000/2*100000)=40 , PCLK/2*100Khz =CCR
    	or x11,x11,x7 			# store calculated CCR (data sheet)in x11 by OR
	sh x11,0(x10)			# store back in peripheral register

	li x10,R16_I2C_OADDR1
	lh x11,0(x10)
	li x7,0xDE			# own address 222,0xDE, b11011110
	or x11,x11,x7
	sh x11,0(x10)

	li x10,R16_I2C_CTLR1		# set pointer to I2C control register 1
	lh x11,0(x10)			# copy contents
	li x7,(1<<0)			# 1<<PE = 1<<0 enable bit is bit0,1<<10 is ack enable bit
	or x11,x11,x7			# OR enable bit to x11
	sh x11,0(x10)			# store half word in control register 1
	li x10,R16_I2C_CTLR1		# set pointer to I2C control register 1
	lh x11,0(x10)			# copy contents
	li x7,(1<<10)			# 1<<10 is ack enable bit
	or x11,x11,x7			# OR ACK enable bit to x11
	sh x11,0(x10)			# store half word in control register 1

##########################

# Enable I2C interrupt in PFIC interrupt controller(NVIC)
PFIC_CONFIG:
	li x10,R32_PFIC_CFGR		# reset core PFIC register for interrupts (NVIC_IRQ enable)
	lw x11,0(x10)
	li x7,((PFIC_KEY3<<16)|(1<<7))	# key3  and SYSRESET , reference manual tells to do it
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

	li x10,R32_PFIC_IENR1		# PFIC Interrupt Enable in core PFIC
	lw x11,0(x10)
	li x7,((1<<31)|(1<<30))		# enabled both I2C interrupts in PFIC (31 & 30th bit)
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

# enabling GLOBAL INTERRUPTS
	li t0, 0x88			# load MPIE and MIE bits , 1<<MIE in mstatus is enabling GLOBAL INTERRUPTS
	longs 0x30029073        	#csrw	mstatus,t0 ,manually assembled opcode









#main endless loop for uart transmit

example:

	li x10,name			# load address of label "name" to x10, string to be transmitted
string_loop:
	lb x8,0(x10)			# load 1 byte from 0 offset of "name"
	beqz x8,finish			# if byte in x8 null branch to label "finish"
	call USART_TX			# call subroutine USART_TX to transmit byte
	addi x10,x10,1			# increase pointer by 1 byte
	j string_loop			# jump back to label string_loop until null is encountered
finish:
	

	call delay			# delay for man in front of terminal
here:
	j here
#############################################################################################################
# SUBROUTINES
############################################################################################################	


###############################################################################################################
# I2C SUBROUTINES
############----I2C--FUNCTIONS---#############################################################################--------------------------------------------------------------------------

check_i2c_status:
	addi sp,sp,-16
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
	sw x12,12(sp)

	li x10, R16_I2C_STAR1			# set pointer x10 to R16_I2C_STAR1 , status register 1
	lh x11,0(x10)				# copy contents to x11
	li x10,	 R16_I2C_STAR2			# set pointer to R16_I2C_STAR2
	lh x12,0(x10)				# copy contents to x12
	slli x12,x12,16				# shift x12 16 position to left and OR it with x11 to hold both register values in 1 32 bit register
	or x11,x11,x12				# status register 1 = 0-15 bits and status register2 = 16-32 bit
	li x10,buffer1				# point x10 to SRAM buffer , address 0x20000004
	sw x11,0(x10)				# store status data in sram for future use

	lw x12,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,16
	ret

#########################################
# I2C EVENT ISR
########################################
I2C1_EV_IRQhandler:
	addi sp,sp,-60			# adjust stack pointer    		
	sw x15,56(sp)			# PUSH
	sw x14,52(sp)			# PUSH
	sw x13,48(sp)			# PUSH
	sw x12,44(sp)			# PUSH
	sw x11,40(sp)			# PUSH
	sw x10,36(sp)			# PUSH
	sw x9,32(sp)			# PUSH
	sw x8,28(sp)			# PUSH
	sw x7,24(sp)			# PUSH
	sw x6,20(sp)			# PUSH
	sw x5,16(sp)			# PUSH
	sw x4,12(sp)			# PUSH
	sw x3,8(sp)			# PUSH
	sw x2,4(sp)			# PUSH
	sw x1,0(sp)			# PUSH

	call check_i2c_status
	
	li x10,R16_I2C_CTLR1
	lh x11,0(x10)
	li x7,(1<<10)			# enable ACK
	or x11,x11,x7
	sw x11,0(x10)


#debug
	li x10,buffer1			# status reg2 & status reg1 	
	li t1,4				# print count 4 , 4 bytes to be transfered
readloop21:
	lb t2,3(x10)			# read from top most byte 
	call bin_to_ascii		# call routine that converts binary to ASCII format of hexadecimal , convets 1 byte
	addi x10,x10,-1			# decrease memorey address  counter
	addi t1,t1,-1			# decrease byte counter ( total 4 bytes to be converted and transmitted via uart)
	bnez t1,readloop21		# if counter greater than 0 loop
	li x8,0x0d			# carriage return
	call USART_TX			# transmit
	li x8,0x0a			# line feed
	call USART_TX			# transmit





	li x10,buffer1			# status reg2 in 16 to 31 and status reg1 in 0 to 15
	lw x11,0(x10)
	andi x11,x11,(1<<1)       	# address bit
	bnez x11,addset			# add flag set ,auto clears on reading status register , exit ISR
	
	
	lh x11,0(x10)
	li x7,(1<<6)			# write in
	and x11,x11,x7
	bnez x11, rxneset
	
	lh x11,0(x10)
	li x7,(1<<7)			# read out
	and x11,x11,x7
	bnez x11, txeset

	lh x11,0(x10)
	li x7,(1<<4)			# write
	and x11,x11,x7
	bnez x11, stopset

rxneset:
	li x10,buffer0		# storage array address pointer offset
	lw x11,0(x10)		# offset count in x11
	blt x11,8,noreset
	li x10,buffer0		# pointer x10 points to offset counter buffer0
	sw x0,0(x10)		# store 0 in counter , clear counter to start of array
noreset:
	li x10,R16_I2C_DATAR
	lb x7,0(x10)
	li x10,buffer0		# storage array address pointer offset
	lw x11,0(x10)		# offset count in x11
	li x10,result1		# storage array start address loaded in x10
	add x10,x10,x11		# add offset to array start address to get current address
	sb x7,0(x10)		# store received byte from data register to result1 + offset , need to count offset
	li x10,buffer0
	lw x7,0(x10)
	addi x7,x7,1
	sw x7,0(x10)
	J getout


txeset:
	li x10,buffer0		# storage array address pointer offset
	lw x11,0(x10)		# offset count in x11
	blt x11,8,noreset1
	li x10,buffer0		# pointer x10 points to offset counter buffer0
	sw x0,0(x10)		# store 0 in counter , clear counter to start of array
noreset1:
	li x10,buffer0		# storeage array address pointer offset
	lw x11,0(x10)		# offset count in x11
	li x10,result1		# storrage array start address loaded in x10
	add x10,x10,x11		# add offset to array start address to get current address
	lb x7,0(x10)		# store received byte from data register to result1 + offset , need to count offset
	li x10,R16_I2C_DATAR	# point x10 to I2C DATA register
	sb x7,0(x10)		# store data copied data from result1 to x7 in I2C DATA register
	li x10,buffer0		# pointer x10 points to offset counter buffer0
	lw x7,0(x10)		# copy contents of counter to x7
	addi x7,x7,1		# increase offset counter
	sw x7,0(x10)		# store back the new off set in buffer0
	J getout		# jump to exit


stopset:
	li x10,R16_I2C_STAR1	# status reg1 
	lh x11,0(x10)		# read staus register 1
	li x10,R16_I2C_CTLR1	# point to I2C_CTRL1 register
	lh x11,0(x10)		# copy contents
	li x7,(1<<9)		# stop bit , this is redundant as stop bit is cleared on reading status reg1 and writing to I2C_CTLR1, achived by status read and ACK enable
	or x11,x11,x7		# set bit in x11
	sh x11,0(x10)		# store half word in x11 to control register1
	j getout

addset:
	li x10,buffer0		# 
	sw x0,0(x10)
	

getout:
	lw x1,0(sp)			# POP
	lw x2,4(sp)			# POP
	lw x3,8(sp)			# POP
	lw x4,12(sp)			# POP
	lw x5,16(sp)			# POP
	lw x6,20(sp)			# POP
	lw x7,24(sp)			# POP
	lw x8,28(sp)			# POP
	lw x9,32(sp)			# POP
	lw x10,36(sp)			# POP
	lw x11,40(sp)			# POP
	lw x12,44(sp)			# POP
	lw x13,48(sp)			# POP
	lw x14,52(sp)			# POP
	lw x15,56(sp)			# POP
	addi sp,sp,60			# adjust stack pointer
	longs 0x30200073		# mret (manually assembled opcode for mret as per RISCV spec)
##############################################
# I2C ERROR ISR
##############################################

I2C1_ER_IRQhandler:
	addi sp,sp,-60			# adjust stack pointer    		
	sw x15,56(sp)			# PUSH
	sw x14,52(sp)			# PUSH
	sw x13,48(sp)			# PUSH
	sw x12,44(sp)			# PUSH
	sw x11,40(sp)			# PUSH
	sw x10,36(sp)			# PUSH
	sw x9,32(sp)			# PUSH
	sw x8,28(sp)			# PUSH
	sw x7,24(sp)			# PUSH
	sw x6,20(sp)			# PUSH
	sw x5,16(sp)			# PUSH
	sw x4,12(sp)			# PUSH
	sw x3,8(sp)			# PUSH
	sw x2,4(sp)			# PUSH
	sw x1,0(sp)			# PUSH

	li x8,'E'
	call USART_TX
	li x10,R16_I2C_STAR1		# status reg1 	
	li t1,4				# print count 4 , 4 bytes to be transfered
readloop22:
	lb t2,3(x10)			# read from top most byte 
	call bin_to_ascii		# call routine that converts binary to ASCII format of hexadecimal , convets 1 byte
	addi x10,x10,-1			# decrease memorey address  counter
	addi t1,t1,-1			# decrease byte counter ( total 4 bytes to be converted and transmitted via uart)
	bnez t1,readloop22		# if counter greater than 0 loop
	li x8,0x0d			# carriage return
	call USART_TX			# transmit
	li x8,0x0a			# line feed
	call USART_TX			# transmit

	li x10,R16_I2C_STAR1		# status reg1 
	lh x11,0(x10)
	li x7,~((1<<11)|(1<<10)|(1<<9)|(1<<8)) # clear overrun, AF, ARLO,BERR flags
	and x11,x11,x7
	sh x11,0(x10)
	
	


clearederrorflags:
	lw x1,0(sp)			# POP
	lw x2,4(sp)			# POP
	lw x3,8(sp)			# POP
	lw x4,12(sp)			# POP
	lw x5,16(sp)			# POP
	lw x6,20(sp)			# POP
	lw x7,24(sp)			# POP
	lw x8,28(sp)			# POP
	lw x9,32(sp)			# POP
	lw x10,36(sp)			# POP
	lw x11,40(sp)			# POP
	lw x12,44(sp)			# POP
	lw x13,48(sp)			# POP
	lw x14,52(sp)			# POP
	lw x15,56(sp)			# POP
	addi sp,sp,60			# adjust stack pointer
	longs 0x30200073		# mret (manually assembled opcode for mret as per RISCV spec)

###########################################################################################
# UART ROUTINES
#########################################################################################

USART_TX:
	addi sp,sp,-16			# add space in stack
	sw ra,0(sp)			# push ra
	sw x7,4(sp)			# push x7
	sw x10,8(sp)			# push x10
	sw x11,12(sp)			# push x11

	li x10,R32_USART_STATR		# load address of usart status register
	lw x11,0(x10)			# load contents of status register in x11
	andi x11,x11,(1<<7)		# mask out 7th bit, transmit buffer empty flag
	beqz x11,USART_TX		# if 0 transmit buffer full, wait until bit is set
	#li x8,0x30
	mv x7,x8			# move byte in x8 to x7
	li x10,R32_USART_DATAR		# x10 has the address of data register
	sb x7,0(x10)			#store byte in x7 to data register
TC_check:
	li x10,R32_USART_STATR		# get contents of status register again
	lw x11,0(x10)
	andi x11,x11,(1<<6)		# check transmit complete bit
	beqz x11,TC_check		# wait if bit is 0 , when transmit complete = 1
		
	lw x11,12(sp)			# pop x11
	lw x10,8(sp)			# pop x10
	lw x7,4(sp)			# pop x7
	lw ra,0(sp)			# pop ra
	addi sp,sp,16			# set SP back 16 bytes
	ret				# return to caller

###################################################
# DELAY
###################################################
delay:	
	addi sp,sp,-8			# move sp 2 words
	sw ra,0(sp)			# push ra
	sw x6,4(sp)			# push x6
	li x6,2000000			# load an arbitarary value 20000000 to t1 register		
dloop:
	addi x6,x6,-1			# subtract 1 from t1
	bne x6,zero,dloop		# if t1 not equal to 0 branch to label loop
	lw x6,4(sp)			# pop x6
	lw ra,0(sp)			# pop ra
	addi sp,sp,8			# sp back 2 words
	ret				# return to caller
###########################################################
name:
string SAJEEV SANKARAN CH32V003 UART
eol:
bytes 0x0d,0x0a,0x00

##########################################################################################################
# converts 1 byte into ASCII represented hexadecimal value
##########################################################################################################
bin_to_ascii:
	addi sp,sp,-4
	sw ra,0(sp)
	mv a3,t2
	andi a3,a3,0xf0
	srli a3,a3,4
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter1
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j low_nibble
letter1:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
low_nibble:
	mv a3,t2
	andi a3,a3,0x0f
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter2
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j exit_bin_to_ascii
letter2:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
exit_bin_to_ascii:
	lw ra,0(sp)
	addi sp,sp,4
	ret
#######################################################################################################


