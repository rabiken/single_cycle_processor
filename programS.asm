.data                  # directive indicating start of the data segment


.text                 # beginning of the text segment (or code segment)
j main                # jump to main
#------------------------------------------------------------
# prime:
# Inputs: a2 - address to the number
# Outputs: a2 - isPrime: 1, not: 0
prime:
  addi t0, a2, 0	# assign a2 to t0
  addi t1, zero, 2	# initialize the loop counter t1 as 2
  
  beq t0, t1, isPrime   # if the input is 2, it is prime
  
  for_prime:
    rem t2, t0, t1		# t2 = t0 % t1
    beq t2, zero, isNotPrime	# if t2 == 0, t0 is not prime
  
    addi t1, t1, 1		# increment the loop counter t1 by 1
    blt t1, t0, for_prime	# if t1 < t0, continue the loop
    
  isPrime:
    addi a2, zero, 1		# a2 is prime 
    ret			# return from the routine
  isNotPrime:
    addi a2, zero, 0		# a2 is not prime
    ret                   # return from the routine
#------------------------------------------------------------
main:
  lw a0, 0x4(zero)	# first argument a0: array size 0x4
  lw a1, 0x8(zero)	# second argument a1: pointer to starting address of the array

  for_main:
    beq a0, zero, done_main	# if a0 == 0, terminate the iteration
    addi a0, a0, -1		# decrement a0
    lw a2, 0(a1)		# load the value at the address a1
    jal ra, prime			# determin the value of a2
    sw a2, 0(a1)		# write the data to the address a1
    addi a1, a1, 4		# increment the buffer a1 by 4
  
  j for_main
  
  done_main:
  nop
  