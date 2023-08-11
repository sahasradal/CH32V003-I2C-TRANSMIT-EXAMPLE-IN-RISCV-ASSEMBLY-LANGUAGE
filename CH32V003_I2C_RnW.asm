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
        
#==============================================    
#I2C_SCL = PC2			# I2C  clock on PC2 (reccomended external pullup 4.7k)
#I2C_SDA = PC1			# I2C  data on PC1 (reccomended external pullup 4.7k)
#=================================================


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
 
#configure GPIO PortC as multiplex open drain output for I2C
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
	li x7,(40<<0)			# CCR = t(rscl)+t(wsclh)/tpclk1 = 1000+4000/125 =40
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
	
	call I2C_BUSY			# check weather I2C is busy ,wait till free
	call I2C_START			# send a start sequence on the bus
        li x15,FRAM_addressW		# load register x15 with slave address (write), data to be sent is loaded in a0
	call SEND_ADDRESS		# subroutine that checks address is transmitted and clears ADDR bit in I2C_STAR1 register
WL:
	li x15,0x00			# load x15 with 0x00 high byte of FRAM register address to be transmitted on bus
	call I2C_WRITE			# call subroutine to transmit value loaded in x15
	li x15,0x00			# load x15 with 0x00 low byte of FRAM register address to be transmitted on bus
	call I2C_WRITE			# call subroutine to transmit value loaded in x15
	
	li x15,'s'
	call I2C_WRITE			# write 's'
	
	li x15,'a'
	call I2C_WRITE			# write 'a'
	
	li x15,'j'
	call I2C_WRITE			# write 'j'
	
	li x15,'e'
	call I2C_WRITE			# write 'e'
	
	li x15,'e'
	call I2C_WRITE			# write 'e'
	
	li x15,'v'
	call I2C_WRITE			# write 'v'

	call I2C_TX_COMPLETE		# wait till transmission buffer is empty
	call I2C_STOP			# call stop transmission subroutine

readfram:
	call I2C_BUSY			# check weather I2C is busy ,wait till free
	call I2C_START			# send a start sequence on the bus
        li x15,FRAM_addressW		# load register x15 with slave address (write), 
	call SEND_ADDRESS		# this subroutine checks completion of address transfer, master bit setting etc
	li x15,0x00			# load x15 with 0x00 high byte of FRAM internal register address to be read from
	call I2C_WRITE			# call subroutine to transmit value loaded in x15
	li x15,0x00			# load x15 with 0x00 low byte of FRAM internal register address to be read from
	call I2C_WRITE			# call subroutine to transmit value loaded in x15. The FRAM internal address pointer at 0x0000
	call I2C_TX_COMPLETE		# wait till transmission buffer is empty
	call I2C_START			# send a start sequence on the bus, this is now a repeated start
	li x15,FRAM_addressR		# load register x15 with slave address (Read),
	li t1,buffer			# load address of memory location with label buffer
	li t0,6				# t0 is loaded with number of bytes received 
	call MULTI_READ			# call subroutine that reads 2 or more bytes from slave
xxxx:
       j xxxx				# end of code , infinite loop to xxxx
	

####----I2C--FUNCTIONS-----------------------------------------------------------------------------
I2C_BUSY:
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
IB:
	li x10, R16_I2C_STAR2
	lh x11,0(x10)			# copy to x11 I2C_STAR2 register contents
	andi x11,x11,(1<<1) 		# and x11 with 1<<I2CBUSY
	bnez x11,IB	 		# if not 0 loop till I2CBUSY bit becomes 0

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret 
###########################
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
check_master_mode_bit:
	li x10, R16_I2C_STAR1
	lh x11,0(x10)
	li x10,	 R16_I2C_STAR2
	lh x12,0(x10)
	slli x12,x12,16
	or x11,x11,x12
	li x7,0x00030001		# BUSY, MSL and SB status bits
	and x11,x11,x7
	bne x11,x7,check_master_mode_bit

	lw x7,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,16
	ret
############################################
I2C_WRITE:
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
IW:
	li x10,R16_I2C_STAR1		# i2c_status1 register
	lh x11,0(x10)			# read and copy contents
	andi x11,x11,(1<<7)		# and contents of x11 with TxE bit7 , if set transmission buffer empty
	beqz x11, IW			# wait till TBE is set (loop if a3 is 0)
	li x10,R16_I2C_DATAR		# set pointer to I2C data register
	sb x15,0(x10)			# store data loaded in x15 to I2C data register

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret				# return to caller
############################################
CLEAR_ACK:				# subroutine to clear ACKEN bit in I2C_CTLR1 register
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)

	li x10,R16_I2C_CTLR1
	lh x11,0(x10)			# copy to x11 contents of I2C_CTL0 rgister
	andi x11,x11,~(1<<10)		# and with 0 shifted to ACK bit10
	sh x11,0(x10)			# write back to register

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret				# return to caller
############################################
I2C_TX_COMPLETE:			# subroutine checks weather I2C transmission is complete
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
TC:
	li x10,R16_I2C_STAR1		# pointer set to R16_I2C_CTLR1
	lh x11,0(x10)			# copy register contents to x11
	andi x11,x11,(1<<7)		# check TBE(7) is set 
	beqz x11,TC			# if not wait by looping to label I2C_TX_COMPLETE 

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret				# return to caller
###########################################
I2C_STOP:				# subroutine to stop I2C transmission
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)

	li x10,R16_I2C_CTLR1		# pointer set to R16_I2C_CTLR1
	lh x11,0(x10)			# copy register contents to x11
	ori x11,x11,(1<<9)		# set STOP bit9 in I2C_CTRL1 register
	sh x11,0(x10)			# store back new value in R16_I2C_CTLR1

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret				# return to caller
###########################################
SEND_ADDRESS:
	addi sp,sp,-20
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
	sw x12,12(sp)
	sw x7,16(sp)

	li x10,R16_I2C_DATAR		# set pointer to data register
	sb x15,0(x10)			# store byte in x15 ( write address)
address_transmit:
	li x10, R16_I2C_STAR1		# set pointer to status register1
	lh x11,0(x10)			# copy status
	li x10,	 R16_I2C_STAR2		# set pointer to status register2
	lh x12,0(x10)			# copy status
	slli x12,x12,16			# shift x12 16 bits left
	or x11,x11,x12			# OR X12 with x11 now both status registers in x11
	li x7,0x00070082		# BUSY, MSL, ADDR, TXE and TRA status
	and x11,x11,x7			# AND write address status bit mask with x11 to check address bit set
	bne x11,x7,address_transmit	# if address transmit bit not set wait in loop

	lw x7,16(sp)
	lw x12,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,20
	ret
##########################################
check_i2c_status:
	addi sp,sp,-16
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
	sw x12,12(sp)

	li x10, R16_I2C_STAR1		# set pointer x10 to R16_I2C_STAR1 , status register 1
	lh x11,0(x10)			# copy contents to x11
	li x10,	 R16_I2C_STAR2		# set pointer to R16_I2C_STAR2
	lh x12,0(x10)			# copy contents to x12
	slli x12,x12,16			# shift x12 16 position to left and OR it with x11 to hold both register values in 1 32 bit register
	or x11,x11,x12			# status register 1 = 0-15 bits and status register2 = 16-32 bit
	li x10,buffer			# point x10 to SRAM buffer , address 0x20000004
	sw x11,0(x10)			# store status data in sram for future use

	lw x12,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,16
	ret
#############################################
check_btf:
	addi sp,sp,-12
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
BTF:
	li x10,R16_I2C_STAR1		# set pointer to status register 1
	lw x11,0(x10)			# copy status
	andi x11,x11,(1<<2)		# and with BTF bitmask
	beqz x11,BTF			# loop if BTF not set

	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,12
	ret
#############################################################
# I2C read routines , call with address in x15
#############################################################
I2C_READ_ONE:  				# (subroutine for single byte reception)
	addi sp,sp,-16
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
	sw t1,12(sp)

	li t1,buffer			# load address of memory location with label buffer
radd_transmit1:				# this code block below checks whether address bit is set when address is transmitted successfully
	li x10,R16_I2C_STAR1		# set x10 pointer to I2C status register 1
	lh x11,0(x10)			# copy contents to x11
	li x10,R16_I2C_STAR2		# set x10 pointer to I2C status register 2 
	lh x12,0(x10)			# copy contents to x12
	slli x12,x12,16			# shift half woord in x12 16 bits left (top half)
	or x11,x11,x12			# OR left aligned X12 with right aligned x11, both status register bits now in x11
	li x7,0x00030002		# load bit mask to test BUSY, MSL, ADDR, status for Mster receiver mode
	and x11,x11,x7			# AND bit mask with x11 (status register contents)
	bne x11,x7,radd_transmit1	# if 0x00030002 is set address transmission success. else sit in tight loop till address tx is complete
	li x10,R16_I2C_STAR1		# dummy read R16_I2C_STAR1 followed by R16_I2C_STAR2 to clear ADDR bit in I2C_STAR1 register
	lh x11,0(x10)			# dummy read
	li x10,R16_I2C_STAR2		# dummy read R16_I2C_STAR1 followed by R16_I2C_STAR2 to clear ADDR bit in I2C_STAR1 register
	lh x11,0(x10)			# dummy read
	call CLEAR_ACK			# call subroutine to clear ACK bit (EVT6_1)
	call I2C_STOP			# call subroutine to set STOP bit (EVT6_1)
R1:
	li x10,R16_I2C_STAR1		# copy contents of I2C_STAT0 register to x11
	lw x11,0(x10)
	andi x11,x11,(1<<6)		# and x11 with RxNE bit6 mask (receive buffer not empty)
	beqz x11,R1			# if a3 is 0 wait by looping to R1 label until RBNE is set
	li x10,R16_I2C_DATAR		# set pointer to I2C data register
	lb x15,0(x10)			# copy contents of I2C data register to x15
	sb x15, 0(t1)			# store byte in x15 to memory location buffer pointed by t0 register offset 0
	
	lw t1,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,16
	ret				# return to caller

###############

###############################################################################
# subroutine to read 2 or more bytes , load read address in x15 and call
# load t1 with location address to store received data
# load t0 with number of bytes to be received
# load t2 with 
################################################################################
MULTI_READ:	
	addi sp,sp,-28
	sw ra,0(sp)
	sw x10,4(sp)
	sw x11,8(sp)
	sw t1,12(sp)
	sw t2,16(sp)
	sw t0,20(sp)
	sw x12,24(sp)

	li x10,R16_I2C_DATAR		# set pointer to I2C data register
	sb x15,0(x10)			# store byte in x15 to I2C data register to be transmitted to slave
radd_transmit:				# this code block below checks whether address bit is set when address is transmitted successfully
	li x10,R16_I2C_STAR1		# set x10 pointer to I2C status register 1
	lh x11,0(x10)			# copy contents to x11
	li x10,R16_I2C_STAR2		# set x10 pointer to I2C status register 2 
	lh x12,0(x10)			# copy contents to x12
	slli x12,x12,16			# shift half woord in x12 16 bits left (top half)
	or x11,x11,x12			# OR left aligned X12 with right aligned x11, both status register bits now in x11
	li x7,0x00030002		# load bit mask to test BUSY, MSL, ADDR, status for Mster receiver mode
	and x11,x11,x7			# AND bit mask with x11 (status register contents)
	bne x11,x7,radd_transmit	# if 0x00030002 is set address transmission success. else sit in tight loop till address tx is complete
	li x10,R16_I2C_STAR1		# dummy read R16_I2C_STAR1 followed by R16_I2C_STAR2 to clear ADDR bit in I2C_STAR1 register
	lh x11,0(x10)			# read
	li x10,R16_I2C_STAR2		# dummy read R16_I2C_STAR1 followed by R16_I2C_STAR2 to clear ADDR bit in I2C_STAR1 register
	lh x11,0(x10)			# read

#	li t1,buffer			# load address of memory location with label buffer
#	li t0,6				# t0 is loaded with number of bytes received
	li x7,2				# load t2 with compare value 2 (last 2 bytes count of message to be received)
	
R11:
	li x10,R16_I2C_STAR1		# copy contents of I2C_STAT1 register to x11
	lw x11,0(x10)			# copy to x11 contents I2C_STAT1
	andi x11,x11,(1<<6)		# and x11 with RxNE bit6 mask (receive buffer not empty)
	beqz x11,R11			# wait till RxNE is set ( data arrived in data register)
	
	li x10,R16_I2C_DATAR		# set pointer to I2C data register
	lb x15,0(x10)			# copy contents of I2C data register to x15
	sb x15,0(t1)			# store received byte in x15 to address pointed by t1 register (buffer here)
	addi t1,t1,1			# increase buffer address + 1
	addi t0,t0,-1			# decrease counter of received bytes
	bleu t0,x7,BBYTES2		# branch to label BYTES2 if t0 is equal or lower than t2 (t2 = 2) , if condition meets we have reached last 2 bytes 
	j R11				# if not reached last 2 bytes jump back to R2 to receive bytes
BBYTES2:				# reach here if the read has reached last 2 bytes of the message
	li x10,R16_I2C_STAR1		# status register1
	lh x11,0(x10)			# copy to x11 I2C_STAT0 register
	andi x11,x11,(1<<6)		# check RxNE bit6 is set by anding x11
	beqz x11,BBYTES2		# wait till RxNE is set by looping
	li x10,R16_I2C_DATAR		# set pointer to I2C data register
	lb x15,0(x10)			# copy contents of I2C data register to x15
	sb x15,0(t1)			# store 2nd last byte in x15 to memory location buffer
	addi t1,t1,1			# increase buffer address + 1
	call CLEAR_ACK			# clear ACK bit so that master will send NAK to stop slave from sending data after next byte
	call I2C_STOP			# set STOP bit to terminate I2C operation after next byte (last one)
BBYTES1:				# last byte
	li x10,R16_I2C_STAR1		# set pointer x10 to I2C status register1
	lh x11,0(x10)			# copy contents to x11
	andi x11,x11,(1<<6)		# check RxNE bit6 is set by anding x11
	beqz x11,BBYTES1		# sit in tight loop till RxNE sets
	li x10,R16_I2C_DATAR		# set pointer to I2C data register
	lb x15,0(x10)			# copy last byte transmitted from slave to I2C data register to x15
	sb x15,0(t1)			# store byte in x15 to memory location buffer pointed by t0

	lw x12,24(sp)
	lw t0,20(sp)
	lw t2,16(sp)
	lw t1,12(sp)
	lw x11,8(sp)
	lw x10,4(sp)
	lw ra,0(sp)
	addi sp,sp,28
	ret

################################################################################
#==========================================
delay:					# delay routine
	li t1,2000000			# load an arbitarary value 2000000 to t1 register,993ms @ 24Mhz		
loop:
	addi t1,t1,-1			# subtract 1 from t1
	bne t1,zero,loop		# if t1 not equal to 0 branch to label loop
	ret	
#################################################################################


