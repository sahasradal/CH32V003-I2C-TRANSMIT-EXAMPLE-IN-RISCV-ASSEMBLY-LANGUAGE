include ch32v003_reg1.asm
#slave_address_write = 0xA0
#slave_address_read  = 0xA1
#buffer = 0x20000004
#system clock is internal RC oscillator 24mhz, default PCLK - 24/3 = 8 Mhz
#FRAM_addressW = 0xA0	,added to include file ch32v003_reg1.asm
#FRAM_addressR = 0xA1   ,added to include file ch32v003_reg1.asm
# TESTED CODE ON FUJITSU MB85RC256V memory FRAM and found to trasmit data
# no interrupts so we start from flash 0x8000000 directly 
#==============================================
sp_init:
    li sp, STACK		# initialize stack pointer
        
#=======================================================================================    
#I2C_SCL = PC2			# I2C  clock on PC2 (reccomended external pullup 4.7k)
#I2C_SDA = PC1			# I2C  data on PC1 (reccomended external pullup 4.7k)
#========================================================================================


I2C_INIT:

#Enable GPIO clocks & AFIO in APB2 clock register
        
    	li x10,R32_RCC_APB2PCENR	# load address of APB2PCENR register to x10 ,for enabling GPIO A,D,C peripherals
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB2PCENR pointed by x10
	li x7,((1<<2)|(1<<4)|(1<<5)|(1<<0))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<AFIOEN, enable port A,C,D and AFIO functions
	or x11,x11,x7			# or values 
	sw x11,0(x10)			# store modified enable values in R32_RCC_APB2PCENR

#Enable I2C clock in  APB1 register
    
    	li x10,R32_RCC_APB1PCENR	# load address of APB1PCENR register to x10 ,for enabling I2C peripheral
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB1PCENR pointed by x10
	li x7,(1<<21)			# 1<<I2C1_EN, = 1<<21 for I2C functions
	or x11,x11,x7			# or values 
	sw x11,0(x10)			
###########
 
#configure GPIO PortC as multiplex open drain output
	li x10,R32_GPIOC_CFGLR		# load pointer x10 with address of R32_GPIOC_CFGLR , I2C SDA & SCL is on portC PC1,PC2
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<4)|(0xf<<8))	# clear pc1,pc2. we need to setup PC1 & PC2 for I2C
	and x11,x11,x7			# clear  mode and cnf bits for selected pin C1,C2
	li x7,((13<<4)|(13<<8))		# PC1 = multiplex open drain output 10mhz ,PC2= multiplex open drain output 10mhz, 0b1101
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR


############
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
    	li x10,R16_I2C_CKCFGR		# set pointer to I2C clockregister
    	lh x11,0(x10)			# copy values to x11 from above register
	li x7,0xfffff000		# clear CCR bits 0-11 with bitmask 0xfffff000
	and x11,x11,x7			# ANDing clears bit 0-11 in x11 register
	li x7,(40<<0)			# CCR = t(rscl)+t(wsclh)/tpclk1 = 1000+4000/125 =40 , or (8000000/2*100000)=40 , PCLK/2*100Khz =CCR
    	or x11,x11,x7 			# store calculated CCR (data sheet)in x11 by OR
	sh x11,0(x10)			# store back in peripheral register
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

    	
main_loop:
	
	call I2C_BUSY			# check whether I2C is busy ,wait till free
	call I2C_START			# send a start sequence on the bus
        li x15,FRAM_addressW		# load register x15 with slave address (write), data to be sent is loaded in x15
	call SEND_ADDRESS		# routine sends address
WL:
	li x15,0x00			# load x15 with 0x00 high byte of FRAM register address to be transmitted on bus
	call I2C_WRITE			# call subroutine to transmit value loaded in x15
	li x15,0x22			# load x15 with 0x22 low byte of FRAM register address to be transmitted on bus
	call I2C_WRITE			# call subroutine to transmit value loaded in x15
	li x15,0xff			# load x15 with 0xff sample data to be written in selected address of FRAM 
	call I2C_WRITE			# call subroutine to transmit value loaded in x15
	call I2C_TX_COMPLETE		# call subroutine that checks the last data byte is transmitted and complete, called before stop
	call I2C_STOP			# call subroutine that stops I2C transmission
end:
	j end				# end of program

####----I2C--FUNCTIONS-----------------------------------------------------------------------------
I2C_BUSY:
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
IB:
	li x10, R16_I2C_STAR2		# set pointer x10 to I2C status register 2, busy bit resides there
	lh x11,0(x10)			# copy to x11 I2C_STAR2 register contents
	andi x11,x11,(1<<1) 		# and x11 with 1<<I2CBUSY
	bnez x11,IB	 		# if not 0 loop till I2CBUSY bit becomes 0

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret 
#################################################
I2C_START:				# send start condition on I2C bus
	addi sp,sp,-16
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
	sw x7,12(sp)

	li x10,R16_I2C_CTLR1		# start bit is in i2c cotrol register bit 8
	lh x11,0(x10)			# copy contents of control register
	ori x11,x11,((1<<10) | (1<<8)) 	# set start bit8 and ack enable bit10
	sh x11,0(x10)			# store in I2C_CTL0 register
check_master_mode_bit:			# below code checks start bit is set , master mode bit is set and bus busy bit is set, reading STAR1 & STAR2 clears start bit
	li x10, R16_I2C_STAR1		# set pointer to status register1
	lh x11,0(x10)			# read contents to x11
	li x10,	 R16_I2C_STAR2		# set pointer to status register2
	lh x12,0(x10)			# read contents to x12
	slli x12,x12,16			# shift x12 16 position to add x11 and x12 to accomodate all status bits in 1 register (x11)
	or x11,x11,x12			# OR both registers , both STAR1 & STAR2 in X11
	li x7,0x00030001		# BUSY, MSL and SB status bits
	and x11,x11,x7			# ANDing yeilds the above 3 status bits
	bne x11,x7,check_master_mode_bit 	# if all 3 bits not sets wait in a loop

	lw x7,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,16
	ret
#####################################################################
I2C_WRITE:
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
IW:
	li x10,R16_I2C_STAR1			# i2c_status1 register
	lh x11,0(x10)				# read and copy contents
	andi x11,x11,(1<<7)			# and contents of x11 with TxE bit7 , if set transmission buffer empty
	beqz x11, IW				# wait till TBE is set (loop if a3 is 0)
	li x10,R16_I2C_DATAR			# set pointer to I2C data register
	sb x15,0(x10)				# store data loaded in x15 to I2C data register

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret					# return to caller
############################################################################
CLEAR_ACK:					# subroutine to clear ACKEN bit in I2C_CTLR1 register
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)

	li x10,R16_I2C_CTLR1
	lh x11,0(x10)				# copy to x11 contents of I2C_CTL0 rgister
	andi x11,x11,~(1<<10)			# and with 0 shifted to ACK bit10
	sh x11,0(x10)				# write back to register

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret					# return to caller
###################################################################################
I2C_TX_COMPLETE:				# subroutine checks weather I2C transmission is complete
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
IC:
	li x10,R16_I2C_STAR1
	lh x11,0(x10)
	andi x11,x11,(1<<7)			# check TBE(7) is set 
	beqz x11, IC				# if not wait by looping to label I2C_TX_COMPLETE 

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret					# return to caller
###################################################################################
I2C_STOP:					# subroutine to stop I2C transmission
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)

	li x10,R16_I2C_CTLR1
	lh x11,0(x10)
	ori x11,x11,(1<<9)			# set STOP bit9 in I2C_CTRL1 register
	sh x11,0(x10)

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret					# return to caller
###################################################################################
SEND_ADDRESS:					# sends address , address to be loaded in x15
	addi sp,sp,-20
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
	sw x12,12(sp)
	sw x7,16(sp

	li x10,R16_I2C_DATAR			# set pointer to data register
	sb x15,0(x10)				# store byte in x15 to I2C data register
address_transmit:
	li x10, R16_I2C_STAR1			# reading STAR1 followed by STAR2 clears the address bit
	lh x11,0(x10)
	li x10,	 R16_I2C_STAR2			# reading STAR1 followed by STAR2 clears the address bit
	lh x12,0(x10)
	slli x12,x12,16				# shift STAR2 by 16 bits LHS
	or x11,x11,x12				# STAR1 & STAR2 now in x11, top16 bits STAR2 , lower 15 bits STAR1 
	li x7,0x00070082			# BUSY, MSL, ADDR, TXE and TRA status
	and x11,x11,x7				# and with above bit mask to see whether these bits are set in status register
	bne x11,x7,address_transmit		# sit in tight loop until above bits are set in both status register

	lw x7,16(sp)
	lw x12,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,20
	ret
###################################################################
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
	li x10,buffer				# point x10 to SRAM buffer , address 0x20000004
	sw x11,0(x10)				# store status data in sram for future use

	lw x12,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,16
	ret
##########################

#==========================================
delay:								# delay routine
	li t1,2000000						# load an arbitarary value 20000000 to t1 register		
loop:
	addi t1,t1,-1						# subtract 1 from t1
	bne t1,zero,loop					# if t1 not equal to 0 branch to label loop
	ret	
#################################################################################

