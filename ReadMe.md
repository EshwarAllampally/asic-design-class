# ASIC Design Lab Report

<details>
  <summary>Lab Session 1</summary>

## Lab Session 1: [16/07/2024]

### Objective
Compile and verify a basic C code using GCC and the RISC-V GNU compiler toolchain on Ubuntu, and compare the outputs.

### Materials and Tools
- **Software Tools:**
  - GCC (GNU Compiler Collection)
  - RISC-V GNU Compiler Toolchain
  - Ubuntu OS

### Pre-Lab Preparation
- Installed GCC and RISC-V GNU Compiler Toolchain on Ubuntu.
- Prepared a simple C program for compilation.

### Procedure

#### Task 1: Compile and Verify C Code using GCC
1. **Code Snippet:**
    ```c
    #include <stdio.h>

    int main() {
        int i, n=5, sum=0;
        for(i=1; i<=n; i++){
          sum = sum + i;
        }
        printf("The sum from 1 to %d is %d\n", n, sum);
        return 0;
    }
    ```
2. **Compile the code using GCC:**
    ```bash
    gcc sumton.c
    ```
3. **Run the compiled code:**
    ```bash
    ./a.out
    ```
4. **Output:**
    ```plaintext
    The sum from 1 to 5 is 15
    ```
    ![Output 1](https://github.com/EshwarAllampally/asic-design-class/blob/main/L1T1_Gcc_out.png)

#### Task 2: Compile and Verify C Code using RISC-V GNU Compiler Toolchain
1. **Compile the code using RISC-V GCC:**
    ```bash
    riscv64-unknown-elf-gcc -O1 -mabi=lp64 -march=rv64i -o sumton.o sumton.c
    ```
2. **Run the compiled code (using an emulator if necessary):**
    ```bash
    riscv64-unknown-elf-objdump -d sumton.o | less
    ```
3. **Output:**
    ![Output 2](https://github.com/EshwarAllampally/asic-design-class/blob/main/L1T2_riscv_gnu_out.png)

### References
- [GCC Documentation](https://gcc.gnu.org/)
- [RISC-V GNU Compiler Toolchain Documentation](https://github.com/riscv/riscv-gnu-toolchain)

</details>

<details>
  <summary>Lab Session 2</summary>

## Lab Session 2: [19/07/2024]

### Objective
1. To compile the Object dump file and verify the output with the GCC output from Lab 1.
2. To debug the main function and observe register values.

### Materials and Tools
- **Software Tools:**
  - GCC (GNU Compiler Collection)
  - RISC-V GNU Compiler Toolchain
  - Spike RISC-V Simulator
  - Ubuntu OS

### Pre-Lab Preparation
- Installed GCC, RISC-V GNU Compiler Toolchain, and Spike on Ubuntu.
- Prepared the `sumton.c` file for compilation.

### Procedure

#### Assembly code for reference:
![Assembly code](https://github.com/EshwarAllampally/asic-design-class/blob/main/L2_assembly_code.png)

#### Task 1: Compile and Verify Objdump File


1. **Compile and Run the Objdump File using Spike:**
    ```bash
    spike pk sumton.o
    ```

3. **Output:**
    ![Output Verified](https://github.com/EshwarAllampally/asic-design-class/blob/main/L2T1_output.png)


#### Task 2: Debugging the Main Function

1. **Start Debugging with Spike:**
    ```bash
    spike -d pk sumton.o
    ```

2. **Execute Until start of main:**
    ```plaintext
    until pc 0 100b0
    ```

3. **Run Next Commands and Observe Register Values:**
    - Press `Enter` to run the next command.
    - Use the following command to verify the data in the register `a0` and `sp` before and after execution:
    ```plaintext
    reg 0 a0
    reg 0 sp
    ```
3. **Output:**
    ![Debug Output](https://github.com/EshwarAllampally/asic-design-class/blob/main/L2T2_debug.png)

### References

- [Spike RISC-V Simulator Documentation](https://github.com/riscv/riscv-isa-sim)

</details>

<details>
  <summary>Lab Session 3</summary>
  
## Lab Session 3: [22/07/2024]

## Task 1: 

### Objective
1. To identify various RISC-V instruction types (R, I, S, B, U, J).
2. To determine the exact 32-bit instruction code for specific RISC-V instructions.

### Procedure
### RISC-V Instruction Formats

RISC-V instructions are divided into several formats, each serving a distinct purpose and structure. Here's an overview of the different instruction formats used in RISC-V:

## 1. R-Type (Register Type)

R-Type instructions use three registers. This format is commonly used for arithmetic and logical instructions.

```
| opcode (7) | rd (5) | funct3 (3) | rs1 (5) | rs2 (5) | funct7 (7) |
```

- **opcode**: Operation code
- **rd**: Destination register
- **funct3**: Function code (middle)
- **rs1**: Source register 1
- **rs2**: Source register 2
- **funct7**: Function code (top)

### Example

```assembly
add x1, x2, x3  // x1 = x2 + x3
```

## 2. I-Type (Immediate Type)

I-Type instructions use an immediate value along with two registers. This format is typically used for load instructions, arithmetic with immediate values, and logical operations.

```
| opcode (7) | rd (5) | funct3 (3) | rs1 (5) | imm (12) |
```

- **opcode**: Operation code
- **rd**: Destination register
- **funct3**: Function code
- **rs1**: Source register
- **imm**: Immediate value (signed)

### Example

```assembly
addi x1, x2, 10  // x1 = x2 + 10
```

## 3. S-Type (Store Type)

S-Type instructions are used for store operations. They involve two registers and an immediate value.

```
| opcode (7) | imm[4:0] (5) | funct3 (3) | rs1 (5) | rs2 (5) | imm[11:5] (7) |
```

- **opcode**: Operation code
- **imm[4:0]**: Immediate value (lower 5 bits)
- **funct3**: Function code
- **rs1**: Source register 1
- **rs2**: Source register 2
- **imm[11:5]**: Immediate value (upper 7 bits)

### Example

```assembly
sw x1, 0(x2)  // Store word in memory
```

## 4. B-Type (Branch Type)

B-Type instructions are used for conditional branches. They use two registers and an immediate value.

```
| opcode (7) | imm[11] (1) | imm[4:1] (4) | funct3 (3) | rs1 (5) | rs2 (5) | imm[10:5] (6) | imm[12] (1) |
```

- **opcode**: Operation code
- **imm[11], imm[4:1], imm[10:5], imm[12]**: Immediate value (split)
- **funct3**: Function code
- **rs1**: Source register 1
- **rs2**: Source register 2

### Example

```assembly
beq x1, x2, label  // Branch if x1 == x2
```

## 5. U-Type (Upper Immediate Type)

U-Type instructions are used for upper immediate operations, such as loading a 20-bit immediate value into the upper 20 bits of a register.

```
| opcode (7) | rd (5) | imm[31:12] (20) |
```

- **opcode**: Operation code
- **rd**: Destination register
- **imm[31:12]**: Immediate value (upper 20 bits)

### Example

```assembly
lui x1, 0x12345  // Load upper immediate
```

## 6. J-Type (Jump Type)

J-Type instructions are used for jump operations, such as jump and link, which involves jumping to a target address and storing the return address in a register.

```
| opcode (7) | rd (5) | imm[20] (1) | imm[10:1] (10) | imm[11] (1) | imm[19:12] (8) |
```

- **opcode**: Operation code
- **rd**: Destination register
- **imm[20], imm[10:1], imm[11], imm[19:12]**: Immediate value (split)

### Example

```assembly
jal x1, label  // Jump and link
```
----------------------------------------------
## Analyzing given Instructions
```
ADD r4, r5, r6
```
-  Type: R
-  Format: opcode | rd | funct3 | rs1 | rs2 | funct7
-  Opcode: 0110011
-  funct3: 000
-  funct7: 0000000
-  rd = 4, rs1 = 5, rs2 = 6
- **Instruction:** ```0000000 00110 00101 000 00100 0110011```
----------------------------------------------
```
SUB r6, r4, r5
```
- Type: R
- Format: opcode | rd | funct3 | rs1 | rs2 | funct7
- Opcode: 0110011
- funct3: 000
- funct7: 0100000
- rd = 6, rs1 = 4, rs2 = 5
- **Instruction:** ```0100000 00101 00100 000 00110 0110011```
----------------------------------------------
```
AND r5, r4, r6
```
- Type: R
- Format: opcode | rd | funct3 | rs1 | rs2 | funct7
- Opcode: 0110011
- funct3: 111
- funct7: 0000000
- rd = 5, rs1 = 4, rs2 = 6
- **Instruction:** ```0000000 00110 00100 111 00101 0110011```
----------------------------------------------
```
OR r8, r5, r5
```
- Type: R
- Format: opcode | rd | funct3 | rs1 | rs2 | funct7
- Opcode: 0110011
- funct3: 110
- funct7: 0000000
- rd = 8, rs1 = 5, rs2 = 5
- **Instruction:** ```0000000 00101 00101 110 01000 0110011```
----------------------------------------------
```
XOR r8, r4, r4
```
- Type: R
- Format: opcode | rd | funct3 | rs1 | rs2 | funct7
- Opcode: 0110011
- funct3: 100
- funct7: 0000000
- rd = 8, rs1 = 4, rs2 = 4
- **Instruction:** ```0000000 00100 00100 100 01000 0110011```
----------------------------------------------
```
SLT r10, r2, r4
```
- Type: R
- Format: opcode | rd | funct3 | rs1 | rs2 | funct7
- Opcode: 0110011
- funct3: 010
- funct7: 0000000
- rd = 10, rs1 = 2, rs2 = 4
- **Instruction:** ```0000000 00100 00010 010 01010 0110011```
----------------------------------------------
```
ADDI r12, r3, 5
```
- Type: I
- Format: opcode | rd | funct3 | rs1 | imm[11:0]
- Opcode: 0010011
- funct3: 000
- rd = 12, rs1 = 3, imm = 5
- **Instruction:** ```000000000101 00011 000 01100 0010011```
----------------------------------------------
```
SW r3, r1, 4
```
- Type: S
- Format: opcode | imm[11:5] | rs2 | rs1 | funct3 | imm[4:0]
- Opcode: 0100011
- funct3: 010
- rs2 = 3, rs1 = 1, imm = 4
- **Instruction:** ```0000000 00011 00001 010 00100 0100011```
----------------------------------------------
```
SRL r16, r11, r2
```
- Type: R
- Format: opcode | rd | funct3 | rs1 | rs2 | funct7
- Opcode: 0110011
- funct3: 101
- funct7: 0000000
- rd = 16, rs1 = 11, rs2 = 2
- **Instruction:** ```0000000 00010 01011 101 10000 0110011```
----------------------------------------------
```
BNE r0, r1, 20
```
- Type: B
- Format: opcode | imm[12] | imm[10:5] | rs2 | rs1 | funct3 | imm[4:1] | imm[11]
- Opcode: 1100011
- funct3: 001
- rs1 = 0, rs2 = 1, imm = 20
- **Instruction:** ```0000001 00001 00000 001 0100 1 1100011```
----------------------------------------------
```
BEQ r0, r0, 15
```
- Type: B
- Format: opcode | imm[12] | imm[10:5] | rs2 | rs1 | funct3 | imm[4:1] | imm[11]
- Opcode: 1100011
- funct3: 000
- rs1 = 0, rs2 = 0, imm = 15
- **Instruction:** ```0000000 00000 00000 000 1111 0 1100011```
----------------------------------------------
```
LW r13, r11, 2
```
- Type: I
- Format: opcode | rd | funct3 | rs1 | imm[11:0]
- Opcode: 0000011
- funct3: 010
- rd = 13, rs1 = 11, imm = 2
- **Instruction:** ```000000000010 01011 010 01101 0000011```
----------------------------------------------
```
SLL r15, r11, r2
```
- Type: R
- Format: opcode | rd | funct3 | rs1 | rs2 | funct7
- Opcode: 0110011
- funct3: 001
- funct7: 0000000
- rd = 15, rs1 = 11, rs2 = 2
- **Instruction:** ```0000000 00010 01011 001 01111 0110011```

## Task 2: 

### Objective
- Functional simulation Experiment

### Procedure

Hardcoded ISA and bit pattern of instructions present in referrence repo;
|Operation       |        Hardcoded ISA |   Bit Pattern (Hardcoded)
|----------------|----------------------|----------------------------------------------------
|ADD R6, R2, R1  |        32'h02208300  |   0000001 00010 00001 000 00110 0000000
|SUB R7, R1, R2  |        32'h02209380  |   0000001 00010 00001 001 00111 0000000
|AND R8, R1, R3  |        32'h0230a400  |   0000001 00011 00001 010 01000 0000000
|OR R9, R2, R5   |        32'h02513480  |   0000001 00101 00010 011 01001 0000000
|XOR R10, R1, R4 |        32'h0240c500  |   0000001 00100 00001 100 01010 0000000
|SLT R1, R2, R4  |        32'h02415580  |   0000001 00100 00010 101 01011 0000000  
|ADDI R12, R4, 5 |        32'h00520600  |   000000000101 00100 000 01100 0000000  
|BEQ R0, R0, 15  |    32'h00f00002  |   0 000000 01111 00000 000 0000 0 0000010
|SW R3, R1, 2    |    32'h00209181  |   0000000 00010 00001 001 00011 0000001
|LW R13, R1, 2   |        32'h00208681  |   000000000010 00001 000 01101 0000001  
|SRL R16, R14, R2|        32'h00271803  |   0000000 00010 01110 001 10000 0000011
|SLL R15, R1, R2 |        32'h00208783  |   0000000 00010 00001 000 01111 0000011

## Observing Waveforms for the above Instructions:

```
  ADD R6, R2, R1
```
  ![IMG_1](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_1.png)
  

```
  SUB R7, R1, R2
```
  ![IMG_2](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_2.png)

  
  ```
  AND R8, R1, R3
  ```
  ![IMG_3](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_3.png)


  ```
  OR R9, R2, R5
  ```
  ![IMG_4](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_4.png)


  ```
  XOR R10, R1, R4
  ```
  ![IMG_5](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_5.png)

```
  SLT R1, R2, R4
  ```
  ![IMG_6](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_6.png)


  ```
  ADDI R12, R4, 5
  ```
  ![IMG_7](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_7.png)


  ```
  BEQ R0, R0, 15
  ```
  ![IMG_8](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab3_Task2_8.png)
</details>

<details>
  <summary>Lab Session 4</summary>

## Lab Session 4: [13/08/2024]

# UART Communication Simulation in C


This tutorial demonstrates a simple simulation of UART (Universal Asynchronous Receiver-Transmitter) communication in C.

## Table of Contents

- [Objective](#objective)
- [Materials and Tools](#materials-and-tools)
- [Introduction](#introduction)
- [Code Overview](#code-overview)
- [Compiling the Code](#compiling-the-code)
- [Running the Simulation](#running-the-simulation)
- [Expected Output](#expected-output)
- [Conclusion](#conclusion)

---

## Objective

Compile and verify a UART Communication System in C code using GCC and the RISC-V GNU compiler toolchain on Ubuntu, and compare the outputs.

## Materials and Tools

-   **Software Tools:**
    -   GCC (GNU Compiler Collection)
    -   RISC-V GNU Compiler Toolchain
    -   Ubuntu OS

## Introduction

UART is a hardware communication protocol that allows data to be sent and received over serial communication. In this simulation, we’ll mimic the behavior of a UART interface using buffers and standard C functions.

## Code Overview

Below is the complete C code that simulates UART communication:

```c
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE 256

static char tx_buffer[BUFFER_SIZE]; // Transmit buffer
static char rx_buffer[BUFFER_SIZE]; // Receive buffer
static int tx_head = 0, tx_tail = 0; // Indices for transmit buffer
static int rx_head = 0, rx_tail = 0; // Indices for receive buffer

// Initialize UART (mock)
void UART_Init(unsigned long baud_rate) {
    printf("UART Initialized with baud rate %lu\n", baud_rate);
}

// Send a single byte of data via UART
void UART_SendByte(char data) {
    tx_buffer[tx_head] = data;
    tx_head = (tx_head + 1) % BUFFER_SIZE;
    printf("Sending byte: %c\n", data);
}

// Receive a single byte of data via UART
char UART_ReceiveByte(void) {
    if (rx_head == rx_tail) {
        return '\0'; // Return null character if no data
    }

    char data_to_return = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % BUFFER_SIZE;
    printf("Receiving byte: %c\n", data_to_return);
    return data_to_return;
}

// Simulate UART data transfer (for testing)
void SimulateUARTTransfer(void) {
    const char example_data[] = "Hello";

    for (size_t i = 0; i < strlen(example_data); i++) {
        rx_buffer[rx_head] = example_data[i];
        rx_head = (rx_head + 1) % BUFFER_SIZE;
    }
}

int main(void) {
    UART_Init(9600); // Initialize UART

    // Simulate UART data transfer
    SimulateUARTTransfer();

    // Send data (simulating what would be sent over UART)
    const char data_to_send[] = "Hello";
    for (size_t i = 0; i < strlen(data_to_send); i++) {
        UART_SendByte(data_to_send[i]);
    }

    // Process received data and echo it back
    for (int i = 0; i < strlen(data_to_send); i++) {
        char received_char = UART_ReceiveByte();
        if (received_char != '\0') {
            UART_SendByte(received_char);
        }
    }

    return 0;
}
```
## Compiling the Code

### Using GCC
```bash
gcc -o uart uart.c
```
### Using RISC-V GCC
```bash
riscv64-unknown-elf-gcc -O1 -mabi=lp64 -march=rv64i -o uart.o uart.c
```

## Running the Simulation

After compiling, run the simulation to observe the UART communication process. The program will initialize UART, simulate data transfer, send data, and echo back the received data.

### Using GCC
```bash
./uart
```
### Using Spike
```bash
spike pk uart.o
```

## Expected Output

The expected output of the program should look like this:
```bash
UART Initialized with baud rate 9600
Sending byte: H
Sending byte: e
Sending byte: l
Sending byte: l
Sending byte: o
Receiving byte: H
Sending byte: H
Receiving byte: e
Sending byte: e
Receiving byte: l
Sending byte: l
Receiving byte: l
Sending byte: l
Receiving byte: o
Sending byte: o
```
### UART Simulation Output
![Output Image](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_4/gcc_and_riscv.png)

## Conclusion

This simple simulation of UART communication in C demonstrates how data can be sent and received using buffers. It’s a great starting point for understanding how UART works in embedded systems.

</details>

<details>
  <summary>Lab Session 5</summary>

## Complete Pipeline RISC-V CPU Micro Architecture: [15/08/2024]

## Introduction

This project involves the implementation of a Complete Pipeline RISC-V CPU Micro Architecture using the Makerchip IDE. The goal is to design and simulate a basic RISC-V CPU that can execute a simple program to sum numbers from 1 to 9. The implementation follows the RISC-V RV32I base instruction set and is divided into multiple stages for instruction fetch, decode, execute, memory access, and write-back.

## Code Overview

### Program Summary

The program sums the integers from 1 to 9 and stores the result in a memory location. The following registers are used:

- **r10 (a0)**: Holds the initial value of 0 and the final sum.
- **r12 (a2)**: Used to store the count of 10.
- **r13 (a3)**: Holds the current integer value to be added.
- **r14 (a4)**: Stores the running sum.

### Instruction Memory (IMem)

- PC fetch, branch, jumps, and loads introduce 2 cycle bubbles in this pipeline.
- The program counter (PC) is managed to fetch instructions from instruction memory.

### Instruction Decode

- The program includes decoding of RISC-V instructions, including I, R, S, U, B, and J types.
- The immediate values are decoded, and the control signals for each instruction type are generated.

### Register Fetch and ALU Operations

- The register file is accessed to fetch source registers.
- ALU operations are performed based on the type of instruction, and results are computed.

### Branch Resolution and Pipeline Control

- The branch target PC is calculated for branch instructions.
- The pipeline manages data forwarding and RAW dependence checks to resolve hazards.
- Valid signals for branch, jump, and load instructions control the flow of the pipeline.

## Code Implementation

```verilog
\m4_TLV_version 1d: tl-x.org
\SV
   // Template code can be found in: https://github.com/stevehoover/RISC-V_MYTH_Workshop
   
   m4_include_lib(['https://raw.githubusercontent.com/BalaDhinesh/RISC-V_MYTH_Workshop/master/tlv_lib/risc-v_shell_lib.tlv'])

\SV
   m4_makerchip_module   // (Expanded in Nav-TLV pane.)
\TLV

   // /====================\
   // | Sum 1 to 9 Program |
   // \====================/
   //
   // Add 1,2,3,...,9 (in that order).
   //
   // Regs:
   //  r10 (a0): In: 0, Out: final sum
   //  r12 (a2): 10
   //  r13 (a3): 1..10
   //  r14 (a4): Sum
   // 
   // External to function:
   m4_asm(ADD, r10, r0, r0)             // Initialize r10 (a0) to 0.
   // Function:
   m4_asm(ADD, r14, r10, r0)            // Initialize sum register a4 with 0x0
   m4_asm(ADDI, r12, r10, 1010)         // Store count of 10 in register a2.
   m4_asm(ADD, r13, r10, r0)            // Initialize intermediate sum register a3 with 0
   // Loop:
   m4_asm(ADD, r14, r13, r14)           // Incremental addition
   m4_asm(ADDI, r13, r13, 1)            // Increment intermediate register by 1
   m4_asm(BLT, r13, r12, 1111111111000) // If a3 is less than a2, branch to label named <loop>
   m4_asm(ADD, r10, r14, r0)            // Store final result to register a0 so that it can be read by main program
   m4_asm(SW, r0, r10, 10000)           // Store r10 result in dmem
   m4_asm(LW, r17, r0, 10000)           // Load contents of dmem to r17
   m4_asm(JAL, r7, 00000000000000000000) // Done. Jump to itself (infinite loop). (Up to 20-bit signed immediate plus implicit 0 bit (unlike JALR) provides byte address; last immediate bit should also be 0)
   m4_define_hier(['M4_IMEM'], M4_NUM_INSTRS)

   |cpu
      @0
         $reset = *reset;
         $clk_esh = *clk;
         
         //PC fetch - branch, jumps and loads introduce 2 cycle bubbles in this pipeline
         $pc[31:0] = >>1$reset ? '0 : (>>3$valid_taken_br ? >>3$br_tgt_pc :
                                       >>3$valid_load     ? >>3$inc_pc[31:0] :
                                       >>3$jal_valid      ? >>3$br_tgt_pc :
                                       >>3$jalr_valid     ? >>3$jalr_tgt_pc :
                                                     (>>1$inc_pc[31:0]));
         // Access instruction memory using PC
         $imem_rd_en = ~ $reset;
         $imem_rd_addr[M4_IMEM_INDEX_CNT-1:0] = $pc[M4_IMEM_INDEX_CNT+1:2];
         
         
      @1
         $clock_esh = *clk;
         //Getting instruction from IMem
         $instr[31:0] = $imem_rd_data[31:0];
         
         //Increment PC
         $inc_pc[31:0] = $pc[31:0] + 32'h4;
         
         //Decoding I,R,S,U,B,J type of instructions based on opcode [6:0]
         //Only [6:2] is used here because this implementation is for RV64I which does not use [1:0]
         $is_i_instr = $instr[6:2] ==? 5'b0000x ||
                       $instr[6:2] ==? 5'b001x0 ||
                       $instr[6:2] == 5'b11001;
         
         $is_r_instr = $instr[6:2] == 5'b01011 ||
                       $instr[6:2] ==? 5'b011x0 ||
                       $instr[6:2] == 5'b10100;
         
         $is_s_instr = $instr[6:2] ==? 5'b0100x;
         
         $is_u_instr = $instr[6:2] ==? 5'b0x101;
         
         $is_b_instr = $instr[6:2] == 5'b11000;
         
         $is_j_instr = $instr[6:2] == 5'b11011;
         
         //Immediate value decode
         $imm[31:0] = $is_i_instr ? { {21{$instr[31]}} , $instr[30:20]} :
                      $is_s_instr ? { {21{$instr[31]}} , $instr[30:25] , $instr[11:8] , $instr[7]} :
                      $is_b_instr ? { {20{$instr[31]}} , $instr[7] , $instr[30:25] , $instr[11:8] , 1'b0} :
                      $is_u_instr ? { $instr[31] , $instr[30:12] , { 12{1'b0}} } :
                      $is_j_instr ? { {12{$instr[31]}} , $instr[19:12] , $instr[20] , $instr[30:21] , 1'b0} :
                      >>1$imm[31:0];
         
         //Generate valid signals for each instruction fields
         $rs1_or_funct3_valid    = $is_r_instr || $is_i_instr || $is_s_instr || $is_b_instr;
         $rs2_valid              = $is_r_instr || $is_s_instr || $is_b_instr;
         $rd_valid               = $is_r_instr || $is_i_instr || $is_u_instr || $is_j_instr;
         $funct7_valid           = $is_r_instr;
         
         //Decode other fields of instruction - source and destination registers, funct, opcode
         ?$rs1_or_funct3_valid
            $rs1[4:0]    = $instr[19:15];
            $funct3[2:0] = $instr[14:12];
         
         ?$rs2_valid
            $rs2[4:0]    = $instr[24:20];
         
         ?$rd_valid
            $rd[4:0]     = $instr[11:7];
         
         ?$funct7_valid
            $funct7[6:0] = $instr[31:25];
         
         $opcode[6:0] = $instr[6:0];
         
         //Decode instruction in subset of base instruction set based on RISC-V 32I
         $dec_bits[10:0] = {$funct7[5],$funct3,$opcode};
         
         //Branch instructions
         $is_beq   = $dec_bits ==? 11'bx_000_1100011;
         $is_bne   = $dec_bits ==? 11'bx_001_1100011;
         $is_blt   = $dec_bits ==? 11'bx_100_1100011;
         $is_bge   = $dec_bits ==? 11'bx_101_1100011;
         $is_bltu  = $dec_bits ==? 11'bx_110_1100011;
         $is_bgeu  = $dec_bits ==? 11'bx_111_1100011;
         
         //Jump instructions
         $is_auipc = $dec_bits ==? 11'bx_xxx_0010111;
         $is_jal   = $dec_bits ==? 11'bx_xxx_1101111;
         $is_jalr  = $dec_bits ==? 11'bx_000_1100111;
         
         //Arithmetic instructions
         $is_addi  = $dec_bits ==? 11'bx_000_0010011;
         $is_add   = $dec_bits ==  11'b0_000_0110011;
         $is_lui   = $dec_bits ==? 11'bx_xxx_0110111;
         $is_slti  = $dec_bits ==? 11'bx_010_0010011;
         $is_sltiu = $dec_bits ==? 11'bx_011_0010011;
         $is_xori  = $dec_bits ==? 11'bx_100_0010011;
         $is_ori   = $dec_bits ==? 11'bx_110_0010011;
         $is_andi  = $dec_bits ==? 11'bx_111_0010011;
         $is_slli  = $dec_bits ==? 11'b0_001_0010011;
         $is_srli  = $dec_bits ==? 11'b0_101_0010011;
         $is_srai  = $dec_bits ==? 11'b1_101_0010011;
         $is_sub   = $dec_bits ==? 11'b1_000_0110011;
         $is_sll   = $dec_bits ==? 11'b0_001_0110011;
         $is_slt   = $dec_bits ==? 11'b0_010_0110011;
         $is_sltu  = $dec_bits ==? 11'b0_011_0110011;
         $is_xor   = $dec_bits ==? 11'b0_100_0110011;
         $is_srl   = $dec_bits ==? 11'b0_101_0110011;
         $is_sra   = $dec_bits ==? 11'b1_101_0110011;
         $is_or    = $dec_bits ==? 11'b0_110_0110011;
         $is_and   = $dec_bits ==? 11'b0_111_0110011;
         
         //Store instructions
         $is_sb    = $dec_bits ==? 11'bx_000_0100011;
         $is_sh    = $dec_bits ==? 11'bx_001_0100011;
         $is_sw    = $dec_bits ==? 11'bx_010_0100011;
         
         //Load instructions - support only 4 byte load
         $is_load  = $dec_bits ==? 11'bx_xxx_0000011;
         
         $is_jump = $is_jal || $is_jalr;
         
      @2
         //Get Source register values from reg file
         $clock_esh = *clk;
         $rf_rd_en1 = $rs1_or_funct3_valid;
         $rf_rd_en2 = $rs2_valid;
         
         $rf_rd_index1[4:0] = $rs1[4:0];
         $rf_rd_index2[4:0] = $rs2[4:0];
         
         //Register file bypass logic - data forwarding from ALU to resolve RAW dependence
         $src1_value[31:0] = $rs1_bypass ? >>1$result[31:0] : $rf_rd_data1[31:0];
         $src2_value[31:0] = $rs2_bypass ? >>1$result[31:0] : $rf_rd_data2[31:0];
         
         //Branch target PC computation for branches and JAL
         $br_tgt_pc[31:0] = $imm[31:0] + $pc[31:0];
         
         //RAW dependence check for ALU data forwarding
         //If previous instruction was writing to reg file, and current instruction is reading from same register
         $rs1_bypass = >>1$rf_wr_en && (>>1$rd == $rs1);
         $rs2_bypass = >>1$rf_wr_en && (>>1$rd == $rs2);
         
      @3
         //ALU
         $clock_esh = *clk;
         $result[31:0] = $is_addi  ? $src1_value +  $imm :
                         $is_add   ? $src1_value +  $src2_value :
                         $is_andi  ? $src1_value &  $imm :
                         $is_ori   ? $src1_value |  $imm :
                         $is_xori  ? $src1_value ^  $imm :
                         $is_slli  ? $src1_value << $imm[5:0]:
                         $is_srli  ? $src1_value >> $imm[5:0]:
                         $is_and   ? $src1_value &  $src2_value:
                         $is_or    ? $src1_value |  $src2_value:
                         $is_xor   ? $src1_value ^  $src2_value:
                         $is_sub   ? $src1_value -  $src2_value:
                         $is_sll   ? $src1_value << $src2_value:
                         $is_srl   ? $src1_value >> $src2_value:
                         $is_sltu  ? $sltu_rslt[31:0]:
                         $is_sltiu ? $sltiu_rslt[31:0]:
                         $is_lui   ? {$imm[31:12], 12'b0}:
                         $is_auipc ? $pc + $imm:
                         $is_jal   ? $pc + 4:
                         $is_jalr  ? $pc + 4:
                         $is_srai  ? ({ {32{$src1_value[31]}} , $src1_value} >> $imm[4:0]) :
                         $is_slt   ? (($src1_value[31] == $src2_value[31]) ? $sltu_rslt : {31'b0, $src1_value[31]}):
                         $is_slti  ? (($src1_value[31] == $imm[31]) ? $sltiu_rslt : {31'b0, $src1_value[31]}) :
                         $is_sra   ? ({ {32{$src1_value[31]}}, $src1_value} >> $src2_value[4:0]) :
                         $is_load  ? $src1_value +  $imm :
                         $is_s_instr ? $src1_value + $imm :
                                    32'bx;
         
         $sltu_rslt[31:0]  = $src1_value <  $src2_value;
         $sltiu_rslt[31:0] = $src1_value <  $imm;
         
         //Jump instruction target PC computation
         $jalr_tgt_pc[31:0] = $imm[31:0] + $src1_value[31:0]; 
         
         //Branch resolution
         $taken_br = $is_beq ? ($src1_value == $src2_value) :
                     $is_bne ? ($src1_value != $src2_value) :
                     $is_blt ? (($src1_value < $src2_value) ^ ($src1_value[31] != $src2_value[31])) :
                     $is_bge ? (($src1_value >= $src2_value) ^ ($src1_value[31] != $src2_value[31])) :
                     $is_bltu ? ($src1_value < $src2_value) :
                     $is_bgeu ? ($src1_value >= $src2_value) :
                     1'b0;
         
         //Current instruction is valid if one of the previous 2 instructions were not (taken_branch or load or jump)
         $valid = ~(>>1$valid_taken_br || >>2$valid_taken_br || >>1$is_load || >>2$is_load || >>2$jump_valid || >>1$jump_valid);
         
         //Current instruction is valid & is a taken branch
         $valid_taken_br = $valid && $taken_br;
         
         //Current instruction is valid & is a load
         $valid_load = $valid && $is_load;
         
         //Current instruction is valid & is jump
         $jump_valid = $valid && $is_jump;
         $jal_valid  = $valid && $is_jal;
         $jalr_valid = $valid && $is_jalr;
         
         //Destination register update - ALU result or load result depending on instruction
         $rf_wr_en = (($rd != '0) && $rd_valid && $valid) || >>2$valid_load;
         $rf_wr_index[4:0] = $valid ? $rd[4:0] : >>2$rd[4:0];
         $rf_wr_data[31:0] = $valid ? $result[31:0] : >>2$ld_data[31:0];
         
      @4
         //Data memory access for load, store
         $clock_esh = *clk;
         $dmem_addr[3:0]     =  $result[5:2];
         $dmem_wr_en         =  $valid && $is_s_instr;
         $dmem_wr_data[31:0] =  $src2_value[31:0];
         $dmem_rd_en         =  $valid_load;
         
      
         //Write back data read from load instruction to register
         $ld_data[31:0]      =  $dmem_rd_data[31:0];
         
      
      

      // Note: Because of the magic we are using for visualisation, if visualisation is enabled below,
      //       be sure to avoid having unassigned signals (which you might be using for random inputs)
      //       other than those specifically expected in the labs. You'll get strange errors for these.

   
   // Assert these to end simulation (before Makerchip cycle limit).
   //Checks if sum of numbers from 1 to 9 is obtained in reg[17] and runs 10 cycles extra after this is met
   *passed = |cpu/xreg[17]>>10$value == (1+2+3+4+5+6+7+8+9);
   //Run for 200 cycles without any checks
   //*passed = *cyc_cnt > 200;
   *failed = 1'b0;
   
   // Macro instantiations for:
   //  o instruction memory
   //  o register file
   //  o data memory
   //  o CPU visualization
   |cpu
      m4+imem(@1)    // Args: (read stage)
      m4+rf(@2, @3)  // Args: (read stage, write stage) - if equal, no register bypass is required
      m4+dmem(@4)    // Args: (read/write stage)
   
   m4+cpu_viz(@4)    // For visualisation, argument should be at least equal to the last stage of CPU logic
                       // @4 would work for all labs
\SV
   endmodule
```

## Results

The project successfully simulates a RISC-V CPU that performs the sum of numbers from 1 to 9. The final result is stored in the register `r10` and can be accessed from memory.

### diagram output:
![Diagram Out](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_5/Diagram.png)

### viz:
![Viz Out](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_5/viz.png)

### Waveforms:
![Waveforms](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_5/waveforms.png)

### Contents of `xreg[14]`
![Contents of Xreg[14]](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_5/Xreg14.png)

## Conclusion

This implementation demonstrates the fundamentals of pipelining in a RISC-V CPU. The design can be extended further to include more complex instructions and optimizations.

</details>

<details>
  <summary>Lab Session 6</summary>

## Conversion of TLV to verilog with Sandpiper: [22/08/2024]

## Objective

Convert the High-level TLV Code to verilog using sandpiper and verify the output.

## Installation

### 1. Required Packages

```bash
sudo apt update
sudo apt install -y make python python3 python3-pip git iverilog gtkwave docker.io
sudo chmod 666 /var/run/docker.sock
sudo apt-get install python3-venv
```

### 2. Virtual Environment

```bash
cd ~
python3 -m venv .venv
source ~/.venv/bin/activate
pip install pyyaml click sandpiper-saas
```

### 3. Clone the Repo

```bash
git clone https://github.com/manili/VSDBabySoC.git
```

### 4. Replace TL-Verilog File

Replace the existing `.tlv` file in the `VSDBabySoC/src/module` dir with your our RISC-V `.tlv` file.

### 5. Convert TL-Verilog to Verilog

```bash
sandpiper-saas -i ./src/module/*.tlv -o rvmyth.v --bestsv --noline -p verilog --outdir ./src/module/
```

### 6. Create Pre-Synthesis Simulation File

```bash
make pre_synth_sim
```

### 7. Compile and Simulate RISC-V Design

```bash
iverilog -o output/pre_synth_sim.out -DPRE_SYNTH_SIM src/module/testbench.v -I src/include -I src/module
```

### 8. Run the Simulation Output

```bash
cd output
./pre_synth_sim.out
```

### 9. View Simulation Results with GTKWave

```bash
gtkwave pre_synth_sim.vcd
```

Review and compare waveform outputs from  Makerchip and GTKwave  to ensure design accuracy.

![TLV Output](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_5/Xreg14.png)

![verilog Output](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab6-7/Lab6out.png)

</details>

<details>
  <summary>Lab Session 7</summary>

## Integration of Peripherals for Digital-to-Analog Conversion Using DAC and PLL: [29/08/2024]

In this assignment, we incorporate two peripherals to facilitate the conversion of digital output to analog output: the PLL and DAC.

**Phase-Locked Loop (PLL)**: The onboard crystal oscillator provides a clock frequency ranging between 12-20 MHz. Since the processor operates at around 100 MHz, an IP/Peripheral is required to elevate this lower frequency clock to a higher frequency. This is where the PLL is utilized. The crystal oscillator clock serves as the input to the PLL, which then outputs a higher frequency clock to our RISC-V core. This clock is subsequently labeled as CPU_clk_GOUR_a0.

**Digital-to-Analog Converter (DAC)**: The processor functions with digital input, but the transmission and reception of signals occur in analog form. Therefore, to transform the digital signal from our RISC-V core into an analog signal, the Digital-to-Analog Converter IP is employed.

### Commands used:

```bash
iverilog -o ./pre_synth_sim.out -DPRE_SYNTH_SIM src/module/testbench.v -I src/include -I src/module/
```

```bash
cd output/
```

```bash
./pre_synth_sim.out
```

```bash
gtkwave pre_synth_sim.vcd
```
### Output Screen-snip: (with username and date Identifiers)

![Output](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab6-7/lab7out.png)

</details>

<details>
<summary>Lab Session 8</summary>
<br>
  
# Task : RTL design using Verilog with SKY130 Technology [15/10/24]
<details>
<summary>Day-1</summary>
<br>
  
## iVerilog based Simulation flow:
 
 ![image](https://github.com/user-attachments/assets/0e2f8052-f0f8-4cfa-bab0-fc83a490afb9)

## LAB-1:
**Aim: Cloning the required files from github repository:**

**Commands:**
```
sudo -i
sudo apt-get install git
ls
cd /home
mkdir VLSI
cd VLSI
git clone https://github.com/kunalg123/sky130RTLDesignAndSynthesisWorkshop.git
cd sky130RTLDesignAndSynthesisWorkshop/verilog_files
ls
```

**Screenshot of the terminal window:**

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L1_1.png)

## LAB-2:
**Aim: Introduction to iVerilog gtkwave:**

In this lab we will implement a 2:1 multiplexer.

**Command to view Verilog code & testbench file:**
```
gvim tb_good_mux.v -o good_mux.v
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L2_1.png)

**Steps for implementing the waveform on gtkwave:**
```
iverilog good_mux.v tb_good_mux.v
ls
./a.out
gtkwave tb_good_mux.vcd
```

**Screenshots of terminal window & gtkwave waveform:**

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L2_2.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L2_3.png)

## LAB-3:
**Aim: Synthesis of 2:1 Multiplexer using Yosys and Logic Synthesis:**

## YOSYS:
A synthesizer is essential in digital design, converting RTL (Register Transfer Level) code into a gate-level netlist. This netlist gives a detailed representation of the circuit, including the logic gates and their connections, forming the groundwork for subsequent steps like placement and routing. In this particular design process, Yosys, an open-source synthesis tool for Verilog HDL, is being used. Yosys employs various optimization strategies to produce an efficient gate-level design from the RTL code.

The primary inputs and outputs are the same in both the RTL design and the synthesized netlist, allowing the same test bench to be used for both.

**Block Diagram of Yosys setup :**
![image](https://github.com/user-attachments/assets/adf3de9f-78c3-4ea8-b07d-01788b398b76)



**Block Diagram of Systhesis Verification :**
![image](https://github.com/user-attachments/assets/72e54532-7abc-43ac-b32e-6a5bde0a9014)

## Logic Synthesis:

**RTL Design:** The design is modeled using a behavioral description in Hardware Description Language (HDL) according to the given specifications.

**Synthesis:** The RTL code is transformed into a gate-level representation, where the design is mapped into logic gates and connections, producing a file called the netlist. In Verilog, a netlist is a representation of a circuit that describes how various components (such as logic gates, flip-flops, or modules) are interconnected. 

## Command steps for Yosys:

**This will invoke/start the yosys:**
```
yosys
```


**Load the sky130 standard library:**
```
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib      
```


**Read the design files:**
```
read_verilog good_mux.v
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L3_1.png)


**Synthesize the top level module:**
```
synth -top good_mux
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L3_2.png)


**Map to the standard library:**
```
abc -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L3_3.png)
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L3_4.png)



**To view the graphical representation of the generated logic, simply enter:**
```
show
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L3_5.png)


**To save the netlist, use the write_verilog command. This will generate the netlist file in the current directory:**
```
write_verilog -noattr good_mux_netlist.v
!gvim good_mux_netlist.v
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_1/d1_L3_6.png)

</details>

<details>
<summary>Day-2</summary>
<br>

# Timing libs, hierarchical vs flat synthesis and efficient flop coding styles:

## LAB-4:
**Introduction and Walkthrough to ' dot lib ':**

The .lib file serves as a collection of standard cells, including slow cells, fast cells, and other essential components. To view the contents of a .lib file, use the following command:
```
sudo -i
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/lib
gvim sky130_fd_sc_hd__tt_025C_1v80.lib
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L4_1.png)


The .lib file provides critical information about the type of process used (such as 130nm technology in this case) and the process conditions like temperature, voltage, etc. It also defines various constraints, including units for variables and the type of technology used. For example:

* technology("cmos"): Specifies the technology as CMOS.
* delay_model : "table_lookup": Defines the delay model.
* bus_naming_style : "%s[%d]": Defines the naming convention for buses.
* time_unit : "1ns": Sets the unit of time.
* voltage_unit : "1V": Sets the unit of voltage.
* leakage_power_unit : "1nW": Defines the unit for leakage power.
* current_unit : "1mA": Sets the unit of current.
* pulling_resistance_unit : "1kohm": Specifies the unit for pulling resistance.
* capacitive_load_unit(1.0000000000, "pf"): Defines the unit for capacitive load.

Additionally, the .lib file provides details about the characteristics of various cells, such as leakage power, power consumption, area, input capacitance, and delay for different input combinations.


**Considering a two input **AND** gate:**

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L4_2.png)


## LAB-5:
**Hierarchical vs flat synthesis & Various Flop Coding Styles and optimization:**

## Hierarchical Synthesis:
**Instructions:**

```
cd ~
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
yosys
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
read_verilog multiple_modules.v
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_1.png)

**To Synthesize the Design:**
```
synth -top multiple_modules
```

When we run the command synth -top multiple_modules in Yosys, hierarchical synthesis is performed. This means that the relationships between the modules are preserved, maintaining the module hierarchy throughout the synthesis process.

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_2.png)

**Multiple Modules: - 2 SubModules**

Commands to generate the netlist & Create a Graphical Representation of Logic for Multiple Modules: 

```
abc -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
show multiple_modules
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_3.png)

**Commands to write the netlist and view it:**

```
write_verilog -noattr multiple_modules_hier.v
!vim multiple_modules_hier.v
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_4.png)

**Flattening:** To  merge all hierarchical modules in the design into a single module and generate a flat netlist, simply type the following command:
```
flatten
```
**Commands to write the netlist and view it:**

```
write_verilog -noattr multiple_modules_hier.v
!vim multiple_modules_hier.v
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_5.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_6.png)


**Graphical Representation of Logic for Multiple Modules:**
```
show 
```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_7.png)


## D Flip-Flop Design and Simulation Using Icarus Verilog, GTKWave, and Yosys:

This project demonstrates various coding styles for D Flip-Flops, followed by simulation using Icarus Verilog and GTKWave. It also covers the synthesis of these designs with Yosys. The simulations focus on three types of D Flip-Flops:

  *  D Flip-Flop with Asynchronous Reset
  *  D Flip-Flop with Asynchronous Set
  *  D Flip-Flop with Synchronous Reset

## 1. D Flip-Flop with Asynchronous Reset:

Verilog code for the D Flip-Flop with an asynchronous reset:
```
module dff_asyncres(input clk, input async_reset, input d, output reg q);
	always@(posedge clk, posedge async_reset)
	begin
		if(async_reset)
			q <= 1'b0;
		else
			q <= d;
	end
endmodule
```

Testbench for Asynchronous Reset D Flip-Flop:
```
module tb_dff_asyncres; 
	reg clk, async_reset, d;
	wire q;
	dff_asyncres uut (.clk(clk), .async_reset(async_reset), .d(d), .q(q));

	initial begin
		$dumpfile("tb_dff_asyncres.vcd");
		$dumpvars(0, tb_dff_asyncres);
		clk = 0;
		async_reset = 1;
		d = 0;
		#3000 $finish;
	end
	
	always #10 clk = ~clk;
	always #23 d = ~d;
	always #547 async_reset = ~async_reset; 
endmodule
```

**Steps to Run the Simulation:**

1. Navigate to the directory where the Verilog files are located:
```
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
```

2. Run the following commands to compile and simulate the design:
```
iverilog dff_asyncres.v tb_dff_asyncres.v
ls
```
The compiled output will be saved as a.out.

3. Execute the compiled output and open the waveform viewer:
```
./a.out
gtkwave tb_dff_asyncres.vcd
```

By following these steps,we can observe the behavior of the D Flip-Flop with an asynchronous reset in the waveform viewer:

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_1.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_2.png)


**Observation:** From the waveform, we can observe that when the asynchronous reset is activated (set high), the Q output immediately resets to zero, regardless of the clock's positive or negative edge. This demonstrates the asynchronous behavior of the reset signal.


## 2. D Flip-Flop with Asynchronous Set:

This section demonstrates the implementation of a D Flip-Flop with an asynchronous set, using Verilog. The design ensures that when the asynchronous set signal is high, the output Q is immediately set to 1, regardless of the clock signal.

Verilog Code for Asynchronous Set D Flip-Flop:
```
module dff_async_set(input clk, input async_set, input d, output reg q);
	always@(posedge clk, posedge async_set)
	begin
		if(async_set)
			q <= 1'b1;
		else
			q <= d;
	end
endmodule
```

Testbench Code:
```
module tb_dff_async_set; 
	reg clk, async_set, d;
	wire q;
	dff_async_set uut (.clk(clk), .async_set(async_set), .d(d), .q(q));

	initial begin
		$dumpfile("tb_dff_async_set.vcd");
		$dumpvars(0, tb_dff_async_set);
		// Initialize Inputs
		clk = 0;
		async_set = 1;
		d = 0;
		#3000 $finish;
	end

	always #10 clk = ~clk;
	always #23 d = ~d;
	always #547 async_set = ~async_set; 
endmodule
```


**Steps to Run the Simulation:**

1. Navigate to the directory containing the Verilog files:
```
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
```

2. Compile the Verilog code and the testbench using Icarus Verilog:
```
iverilog dff_async_set.v tb_dff_async_set.v
ls
```

The output will be saved as a.out.

3. Run the compiled file and open the waveform in GTKWave:
```
./a.out
gtkwave tb_dff_async_set.vcd
```

**Result:**

After running the simulation, we will observe the behavior of the D Flip-Flop with an asynchronous set in the waveform viewer. Below is a snapshot of the commands and the resulting waveforms.

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_async_Set_1.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_async_Set_2.png)

**Observation:** The waveform clearly shows that the Q output switches to one when the asynchronous set is asserted high, regardless of the clock edge (positive or negative).

## 3. D Flip-Flop with Synchronous Reset:

This section contains Verilog code to implement a D Flip-Flop with a **Synchronous Reset**.

The Verilog code defines a D flip-flop with a synchronous reset, where the reset signal is active high. When the reset is asserted during a clock edge, the output `q` is set to 0. Otherwise, the flip-flop captures the value of `d` on the rising edge of the clock.
```
module dff_syncres (input clk,
    input sync_reset,
    input d,
    output reg q
);
    
    always @(posedge clk) begin
        if (sync_reset)
            q <= 1'b0;
        else
            q <= d;
    end
endmodule
```

Testbench Code:
```
module tb_dff_syncres;
    reg clk, sync_reset, d;
    wire q;

    // Instantiate the Device Under Test (DUT)
    dff_syncres uut (.clk(clk), .sync_reset(sync_reset), .d(d), .q(q));

    initial begin
        // Initialize waveform dump
        $dumpfile("tb_dff_syncres.vcd");
        $dumpvars(0, tb_dff_syncres);

        // Initialize inputs
        clk = 0;
        sync_reset = 1;
        d = 0;

        // End simulation after a set time
        #3000 $finish;
    end

    // Clock generation
    always #10 clk = ~clk;

    // Toggle the input `d` every 23 time units
    always #23 d = ~d;

    // Toggle the reset signal every 547 time units
    always #547 sync_reset = ~sync_reset;
endmodule
```

**Steps to Run the Simulation:**

1. Navigate to the directory containing the Verilog files:
```
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
```

2. Compile the Verilog code and the testbench using Icarus Verilog:
```
iverilog dff_async_set.v tb_dff_async_set.v
ls
```

The output will be saved as a.out.

3. Run the compiled file and open the waveform in GTKWave:
```
./a.out
gtkwave tb_dff_async_set.vcd
```

**Result:**
After running the simulation, we will observe the behavior of the D Flip-Flop with an Synchronous Reset in the waveform viewer. Below is a snapshot of the commands and the resulting waveforms.

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_sync_reset_1.png)

**Observation:** From the waveform, it is evident that the Q output transitions to zero when the synchronous reset is asserted high, but only at the positive edge of the clock signal.


# Synthesis of Various D-Flip-Flops using Yosys

This repository demonstrates the synthesis and simulation of three types of D-Flip-Flops using Yosys:  
1. **Asynchronous Reset**  
2. **Asynchronous Set**  
3. **Synchronous Reset**

## 1. Asynchronous Reset D Flip-Flop

### Command Steps for Synthesis:

Follow the steps below to synthesize the asynchronous reset D Flip-Flop design:

1. Navigate to the required directory:

    ```
    cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
    ```

2. Launch Yosys:

    ```
    yosys
    ```

3. Read the standard cell library:

    ```
    read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
    ```

4. Read the Verilog design files:

    ```
    read_verilog dff_asyncres.v
    ```

5. Synthesize the design:

    ```
    synth -top dff_asyncres
    ```

6. Generate the netlist:

    ```
    dfflibmap -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
    ```

7. Create a graphical representation of the Asynchronous Reset D Flip-Flop:

    ```
    show
    ```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_synth_1.png)


## 2. Asynchronous Set D Flip-Flop

### Command Steps for Synthesis

Follow the steps below to synthesize the asynchronous set D Flip-Flop design:

1. Navigate to the required directory:

    ```
    cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
    ```

2. Launch Yosys:

    ```
    yosys
    ```

3. Read the standard cell library:

    ```
    read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
    ```

4. Read the Verilog design files:

    ```
    read_verilog dff_async_set.v
    ```

5. Synthesize the design:

    ```
    synth -top dff_async_set
    ```

6. Generate the netlist:

    ```
    dfflibmap -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
    ```

7. Create a graphical representation of the Asynchronous Set D Flip-Flop:

    ```
    show
    ```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_synth_2.png)


## 3. Synchronous Reset D Flip-Flop

### Command Steps for Synthesis

Follow the steps below to synthesize the synchronous reset D Flip-Flop design:

1. Navigate to the required directory:

    ```
    cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
    ```

2. Launch Yosys:

    ```
    yosys
    ```

3. Read the standard cell library:

    ```
    read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
    ```

4. Read the Verilog design files:

    ```
    read_verilog dff_syncres.v
    ```

5. Synthesize the design:

    ```
    synth -top dff_syncres
    ```

6. Generate the netlist:

    ```
    dfflibmap -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
    ```

7. Create a graphical representation of the Synchronous Reset D Flip-Flop:

    ```
    show
    ```
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_2/d2_L5_DFF_synth_3.png)

</details>


<details>
<summary>Day-3</summary>
<br>
  
# Combinational and sequential optmizations:

## LAB-6:
## Optimization of Various Combinational Designs using Yosys:

This section demonstrates the synthesis and optimization of various combinational designs using Yosys.

## Combinational Designs:
1. **2-input AND gate**
2. **2-input OR gate**
3. **3-input AND gate**
4. **2-input XNOR gate (3-input Boolean Logic)**
5. **Multiple Module Optimization-1**
6. **Multiple Module Optimization-2**

---

## 1. 2-input AND Gate

### Verilog Code:
```
module opt_check(input a, input b, output y);
	assign y = a?b:0;
endmodule
```

### Command Steps for Synthesis:

1. Navigate to the required directory:
```  
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
```
2. Launch Yosys:
```
yosys
```

3. Read the standard cell library:
```
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```

4. Read the Verilog design files:
```
read_verilog opt_check.v
```

5. Synthesize the design:
```
synth -top opt_check
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_1.png)


6. Generate the netlist:
```
abc -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```

7. Remove unused or redundant logic:
```
opt_clean -purge
```

8. Create a graphical representation:
```
show
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_2.png)

---

## 2. 2-input OR Gate

### Verilog Code:
```
module opt_check2(input a, input b, output y);
	assign y = a?1:b;
endmodule
```

### Command Steps for Synthesis:

Repeat the same steps as for the 2-input AND gate with the following changes:

1. Use `opt_check2.v` as the Verilog file:
```
read_verilog opt_check2.v
```

2. Synthesize the design with `opt_check2`:
```
synth -top opt_check2
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_3.png)


**NetList:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_4.png)


---

## 3. 3-input AND Gate

### Verilog Code:
```
module opt_check3(input a, input b, input c, output y);
	assign y = a?(b?c:0):0;
endmodule
```

### Command Steps for Synthesis:

Follow the same steps as for the 2-input AND gate with the following changes:

1. Use `opt_check3.v` as the Verilog file:
```
read_verilog opt_check3.v
```

2. Synthesize the design with `opt_check3`:
synth -top opt_check3

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_5.png)


**NetList:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_6.png)

---

## 4. 2-input XNOR Gate (3-input Boolean Logic)

### Verilog Code:
```
module opt_check4(input a, input b, input c, output y);
	assign y = a ? (b ? ~c : c) : ~c;
endmodule
```

### Command Steps for Synthesis:

Follow the same steps as for the 2-input AND gate with the following changes:

1. Use `opt_check4.v` as the Verilog file:
read_verilog opt_check4.v

2. Synthesize the design with `opt_check4`:
```
synth -top opt_check4
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_7.png)


**NetList:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_8.png)

---

## 5. Multiple Module Optimization-1

### Verilog Code:
```
module sub_module1(input a, input b, output y);
	assign y = a & b;
endmodule
module sub_module2(input a, input b, output y);
	assign y = a^b;
endmodule

module multiple_module_opt(input a, input b, input c, input d, output y);
	wire n1, n2, n3;
	
	sub_module1 U1 (.a(a), .b(1'b1), .y(n1));
	sub_module2 U2 (.a(n1), .b(1'b0), .y(n2));
	sub_module2 U3 (.a(b), .b(d), .y(n3));
	
	assign y = c | (b & n1);
endmodule
```

### Command Steps for Synthesis:

1. Navigate to the required directory:
```
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
```

2. Launch Yosys:
```
yosys
```

3. Read the standard cell library:
```
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```

4. Read the Verilog design files:
```
read_verilog multiple_module_opt.v
```

5. Synthesize the design:
```
synth -top multiple_module_opt
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_9.png)


6. Generate the netlist:
```
abc -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```

7. Remove unused or redundant logic:
```
opt_clean -purge
```

8. Flatten the design to merge hierarchical modules:
```
flatten
```

9. Create a graphical representation:
```
show
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_a.png)

---

## 6. Multiple Module Optimization-2

### Verilog Code:
```
module sub_module(input a, input b, output y);
	assign y = a & b;
endmodule

module multiple_module_opt2(input a, input b, input c, input d, output y);
	wire n1, n2, n3;
	
	sub_module U1 (.a(a), .b(1'b0), .y(n1));
	sub_module U2 (.a(b), .b(c), .y(n2));
	sub_module U3 (.a(n2), .b(d), .y(n3));
	sub_module U4 (.a(n3), .b(n1), .y(y));
endmodule
```

### Command Steps for Synthesis:

Follow the same steps as for Multiple Module Optimization-1 with the following changes:

1. Use `multiple_module_opt2.v` as the Verilog file:
```
read_verilog multiple_module_opt2.v
```

2. Synthesize the design with `multiple_module_opt2`:
```
synth -top multiple_module_opt2
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_b.png)

3. Flatten the design and create a graphical representation:
```
flatten
show
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab6/d3_l6_c.png)




## LAB-7 : 
## Optimization of various Sequential Designs

* D-Flipflop Constant 1 with Asynchronous Reset (active low)
* D-Flipflop Constant 2 with Asynchronous Reset (active high)
* D-Flipflop Constant 3 with Synchronous Reset (active low)
* D-Flipflop Constant 4 with Synchronous Reset (active high)
* D-Flipflop Constant 5 with Synchronous Reset
* Counter Optimization 1
* Counter Optimization 2

**1. D-Flipflop Constant 1 with Asynchronous Reset (active low):**

Verilog code for the asynchronous reset (active low):
```
module dff_const1(input clk, input reset, output reg q); 
always @(posedge clk, posedge reset)
begin
	if(reset)
		q <= 1'b0;
	else
		q <= 1'b1;
end
endmodule
```

Testbench code:
```
module tb_dff_const1; 
	reg clk, reset;
	wire q;

	dff_const1 uut (.clk(clk),.reset(reset),.q(q));

	initial begin
		$dumpfile("tb_dff_const1.vcd");
		$dumpvars(0,tb_dff_const1);
		// Initialize Inputs
		clk = 0;
		reset = 1;
		#3000 $finish;
	end

	always #10 clk = ~clk;
	always #1547 reset=~reset;
endmodule
```

Command steps:

Go to the required directory:
```
sudo -i  
cd ~  
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files  
```

Run the following commands to simulate and observe waveforms:
```
iverilog dff_const1.v tb_dff_const1.v  
ls  
```

After running the above command, iVerilog stores the output as 'a.out'. Now execute 'a.out' and observe waveforms:
```
./a.out  
gtkwave tb_dff_const1.vcd  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_1.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_2.png)


**Observation:** From the waveform, Q output is always high when reset is low, and the reset doesn’t depend on the clock edge.

**Synthesis:**

Go to the required directory:
```
cd ~  
sudo -i  
cd ~  
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files 
```

Invoke Yosys:
```
yosys  
```

Read the library:
```
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib  
```

Read the Verilog design files:
```
read_verilog dff_const1.v  
```

Synthesize the design:
```
synth -top dff_const1  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_3.png)


Generate the netlist:
```
dfflibmap -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib  
```

Create a graphical representation:
```
show  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_4.png)

**Observation:** Since the reset is asynchronous and does not depend on the clock edge, the D Flip-Flop remains intact and is not optimized out of the design.

**2. D-Flipflop Constant 2 with Asynchronous Reset (active high)**

Verilog code for the asynchronous reset (active high):
```
module dff_const2(input clk, input reset, output reg q); 
always @(posedge clk, posedge reset)
begin
	if(reset)
		q <= 1'b1;
	else
		q <= 1'b1;
end
endmodule
```

Testbench code:
```
module tb_dff_const2; 
	reg clk, reset;
	wire q;

	dff_const2 uut (.clk(clk),.reset(reset),.q(q));

	initial begin
		$dumpfile("tb_dff_const2.vcd");
		$dumpvars(0,tb_dff_const2);
		// Initialize Inputs
		clk = 0;
		reset = 1;
		#3000 $finish;
	end

	always #10 clk = ~clk;
	always #1547 reset=~reset;
endmodule
```

Command steps:
```
sudo -i  
cd ~  
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files  
```

Run the following commands:
```
iverilog dff_const2.v tb_dff_const2.v  
ls  
./a.out  
gtkwave tb_dff_const2.vcd  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_5.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_6.png)

**Observation:** The waveform shows that the Q output remains consistently high, regardless of the reset signal.

**Synthesis:**
```
cd ~  
sudo -i  
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files  
```

Invoke Yosys:
```
yosys  
```

Read the library:
```
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib  
```

Read the Verilog files:
```
read_verilog dff_const2.v  
```

Synthesize the design:
```
synth -top dff_const2  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_7.png)

Generate the netlist:
```
dfflibmap -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib  
```

Graphical representation:
```
show  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_8.png)

**Observation:** The output Q is always 1 and does not depend on the reset edge; therefore, the D Flip-Flop has been optimized away.


**3. D-Flipflop Constant 3 with Synchronous Reset (active low)**

Verilog code for Synchronous reset (active low):
```
module dff_const3(input clk, input reset, output reg q); 
	reg q1;
	always @(posedge clk, posedge reset)
	begin
		if(reset)
		begin
			q <= 1'b1;
			q1 <= 1'b0;
		end
		else
		begin	
			q1 <= 1'b1;
			q <= q1;
		end
	end
endmodule
```

Testbench is similar to the previous one.

Command steps:
```
sudo -i  
cd ~  
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files  
```

Run the following commands:
```
iverilog dff_const3.v tb_dff_const3.v  
ls  
./a.out  
gtkwave tb_dff_const3.vcd  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_9.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_a.png)

**Synthesis:**
```
cd ~  
sudo -i  
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files  
```

Invoke Yosys:
```
yosys  
```

Read the library:
```
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib  
```

Read Verilog files:
```
read_verilog dff_const3.v  
```

Synthesize the design:
```
synth -top dff_const3  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_b.png)

Generate netlist:
```
dfflibmap -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib  
```

Graphical representation:
```
show  
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_c.png)

**Observation:** This module implements a D Flip-Flop where the output Q is updated on every clock cycle following a reset.


**4. D-Flipflop Constant 4 with Synchronous Reset (active high)**

Verilog Code:
```
module dff_const4(input clk, input reset, output reg q); 
	reg q1;
	always @(posedge clk, posedge reset)
	begin
		if(reset)
		begin
			q <= 1'b1;
			q1 <= 1'b1;
		end
		else
		begin	
			q1 <= 1'b1;
			q <= q1;
		end
	end
endmodule
```

**Testbench** follows the same pattern as earlier.

**Synthesis** steps are identical to those described previously.

**gtkwave waveform:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_d.png)

**Synthesis:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_e.png)

**Netlist:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_f.png)

**Observations:** When synthesized, this design will yield a Flip-Flop where the output q is always 1, independent of the reset or clock states.


**5. D-Flipflop Constant 5 with Synchronous Reset**

Verilog Code:
```
module dff_const5(input clk, input reset, output reg q); 
	reg q1;
	always @(posedge clk, posedge reset)
	begin
		if(reset)
		begin
			q <= 1'b0;
			q1 <= 1'b0;
		end
		else
		begin	
			q1 <= 1'b1;
			q <= q1;
		end
	end
endmodule
```

**Simulation** and **Synthesis** follows the same steps as above.

**gtkwave waveform:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_g.png)

**Synthesis:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_i.png)

**Netlist:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_j.png)

**Observations:** When synthesized, the design will result in a flip-flop where q is always 1 after the first clock cycle post-reset.


**6. Counter Optimization 1**

Verilog Code:
```
module counter_opt (input clk, input reset, output q);
	reg [2:0] count;
	assign q = count[0];
	always @(posedge clk,posedge reset)
	begin
		if(reset)
			count <= 3'b000;
		else
			count <= count + 1;
	end
endmodule
```

Following similar steps as above for **synthesis** and **graphical representation:**

**gtkwave waveform:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_k.png)

**Synthesis:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_l.png)

**Netlist:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_m.png)

**7. Counter Optimization 2**

Verilog Code:
```
module counter_opt2 (input clk, input reset, output q);
	reg [2:0] count;
	assign q = (count[2:0] == 3'b100);
	always @(posedge clk,posedge reset)
	begin
		if(reset)
			count <= 3'b000;
		else
			count <= count + 1;
	end
endmodule
```

Following similar steps as above for **synthesis** and **Netlist:**

**Synthesis:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_n.png)

**Netlist:**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_3_Lab7/d3_l7_o.png)

</details>

<details>
<summary>Day-4</summary>
<br>

#  GLS, blocking vs non-blocking and Synthesis-Simulation mismatch:

## LAB-8:
## Gate Level Simulation (GLS), Synthesis-Simulation Mismatch, Non-Blocking and Blocking Statements

### Gate Level Simulation (GLS):

Gate Level Simulation is a vital step in verifying digital circuits. It involves simulating the synthesized netlist—a lower-level representation of the design—using a testbench to verify logical correctness and timing behavior. By comparing the simulated outputs with the expected results, GLS ensures that synthesis has not introduced any errors and that the design meets performance requirements.

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_1.png)


### Importance of Sensitivity Lists:

Accurate sensitivity lists are critical for ensuring correct circuit behavior. Incomplete sensitivity lists may result in unexpected latches. Similarly, blocking and non-blocking assignments within `always` blocks exhibit different execution behaviors. Incorrect use of blocking assignments may unintentionally create latches, leading to synthesis and simulation mismatches. To avoid these issues, circuit behavior should be carefully analyzed to ensure proper sensitivity lists and assignment usage.

### GLS Example 1: 2-to-1 MUX using Ternary Operator

Verilog code:
```
module ternary_operator_mux (input i0, input i1, input sel, output y);
assign y = sel ? i1 : i0;
endmodule
```

Simulation steps:
```
iverilog ternary_operator_mux.v tb_ternary_operator_mux.v
./a.out
gtkwave tb_ternary_operator_mux.vcd
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_2.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_3.png)



### Synthesis:

1. Invoke yosys:
```
yosys
```

2. Load the library:
```
read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```

3. Read the design:
```
read_verilog ternary_operator_mux.v
```

4. Synthesize the design:
```
synth -top ternary_operator_mux
```

5. Generate the netlist:
```
abc -liberty ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
```

6. Create a graphical representation:
```
show
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_4.png)


To view the netlist:
```
write_verilog -noattr ternary_operator_mux_net.v
gvim ternary_operator_mux_net.v
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_5.png)


### GLS Execution

Navigate to the appropriate directory and simulate:
```
sudo -i
cd ~/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
iverilog ../my_lib/verilog_model/primitives.v ../my_lib/verilog_model/sky130_fd_sc_hd.v ternary_operator_mux_net.v tb_ternary_operator_mux.v
./a.out
gtkwave tb_ternary_operator_mux.vcd
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_6.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_7.png)


### Example 2: 2-to-1 Bad MUX Design

Verilog code:
```
module bad_mux(input i0, input i1, input sel, output reg y);
    always@(sel) begin
        if(sel) y <= i1;
        else y <= i0;
    end
endmodule
```

Simulation steps:
```
iverilog bad_mux.v tb_bad_mux.v
./a.out
gtkwave tb_bad_mux.vcd
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_8.png)

### Example 3: Blocking Caveat

Verilog code:
```
module blocking_caveat(input a, input b, input c, output reg d);
    reg x;
    always@(*) begin
        d = x & c;
        x = a | b;
    end
endmodule
```

Simulation steps:
```
iverilog blocking_caveat.v tb_blocking_caveat.v
./a.out
gtkwave tb_blocking_caveat.vcd
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_a.png)

As shown in the waveform, when both A and B are zero, the expected output of the OR gate (X) should be zero, which would result in the AND gate output (D) also being zero. However, due to the blocking assignment in the design, the AND gate input X retains the previous value of A|B, which is one. This leads to a mismatch between the expected and actual output, highlighting the discrepancy caused by the blocking statement.


Following the same steps as above for **synthesis** and **graphical representation:**

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_b.png)

**Netlist**
```
write_verilog -noattr blocking_caveat_net.v
!gvim blocking_caveat_net.v
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_c.png)

### Gate Level Synthesis

Navigate to the appropriate directory and simulate:
```
sudo -i
cd /home/VLSI/sky130RTLDesignAndSynthesisWorkshop/verilog_files
iverilog ../my_lib/verilog_model/primitives.v ../my_lib/verilog_model/sky130_fd_sc_hd.v blocking_caveat_net.v tb_blocking_caveat.v
ls
./a.out
gtkwave tb_blocking_caveat.vcd
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_8/Day_4/d4_l8_d.png)

These waveforms represent the results of the Gate Level Synthesis for the Blocking Caveat, illustrating how the design behaves at the gate level with respect to the blocking assignment issue.

</details>
</details>


<details>
<summary>Lab Session 9 </summary>
<br>

# Aim: Synthesize the RISC-V core and compare its output with functional simulations.

## Steps:

1. **Copy source files:**

   First copy the `src` folder from `VSDBabySoC` directory to your `VLSI` folder. Then, move this folder into the `sky130RTLDesignAndSynthesisWorkshop` directory using the following commands:
   ```
   sudo -i
   cd /home/VLSI/
   cp -r src sky130RTLDesignAndSynthesisWorkshop/
   ```
   
2. **Navigate to the target directory:**

   Move to the correct directory to begin the synthesis process:
   ```
   cd ~/VLSI/sky130RTLDesignAndSynthesisWorkshop/src/module
   ```
   
3. **Start Yosys for synthesis:**

   Launch `yosys` to begin synthesizing the design:
   ```
   yosys
   ```
   
4. **Load the standard cell library:**

   Import the necessary library for synthesis:
   ```
   read_liberty -lib ../lib/sky130_fd_sc_hd__tt_025C_1v80.lib
   ```
   
5. **Load the Verilog design files:**

   Read in the Verilog files for design:
   ```
   read_verilog clk_gate.v
   read_verilog rvmyth.v
   ```

   ![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_9/1.png)

   
6. **Synthesize the RISC-V design:**

   Run the synthesis command for the top module `rvmyth`:
   ```
   synth -top rvmyth
   ```
   
7. **Generate the netlist:**

   Output the synthesized netlist:
   ```
   write_verilog -noattr rvmyth.v
   gvim rvmyth.v
   exit
   ```
   
   ![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_9/3.png)

   
8. **Simulate and observe the output waveform:**

   Use `iverilog` to simulate the synthesized RISC-V and generate the waveform:
   ```
   iverilog ../../my_lib/verilog_model/primitives.v ../../my_lib/verilog_model/sky130_fd_sc_hd.v rvmyth.v testbench.v vsdbabysoc.v avsddac.v avsdpll.v clk_gate.v
   ls
   ./a.out
   gtkwave dump.vcd
   ```

   ![Image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_9/4.png)

   **The waveform shows the generated sawtooth waveform and cells.**

   
## "Functional Simulations (Previously done in LAB-7)":

**Command Steps:**
```
cd ~
cd VSDBabySoC
iverilog -o ./pre_synth_sim.out -DPRE_SYNTH_SIM src/module/testbench.v -I src/include -I src/module/
./pre_synth_sim.out
gtkwave pre_synth_sim.vcd
```

## COMPARISON of Functionality vs Synthesized output waveform:

**LAB-7 Waveform (O1):**
![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_9/5.png)

**LAB-9 Waveform (O2):**
![Image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_9/4.png)


## CONCLUSION : The Functionality vs Synthesized output waveform matches, i.e, O1 = O2.

</details>

<details>
  <summary>Lab Session 10</summary>

# Static Timing Analysis for a Synthesized RISC-V Core with OpenSTA

## Tools Installation
**CUDD**
Download CUDD from **[here](https://github.com/davidkebo/cudd/blob/main/cudd_versions/cudd-3.0.0.tar.gz)** and move downloaded file to `home` directory
```
cd
tar xvfz cudd-3.0.0.tar.gz
cd cudd-3.0.0
./configure
make
```
**openSTA**
```
cd
sudo apt-get install cmake clang gcc tcl swig bison flex

git clone https://github.com/parallaxsw/OpenSTA.git
cd OpenSTA
cmake -DCUDD_DIR=/home/eshwar/cudd-3.0.0
make
cd app
./sta
```

![sta](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_10/l10_1.png)

```
cd /home/eshwar/OpenSTA
mkdir lab10
```
Download all **[these files](https://github.com/EshwarAllampally/asic-design-class/tree/main/lab_10)** to directory `lab10`

**Steps to do Timing Analysis**
- Clock period = 9.2ns
- Setup uncertainty and clock transition will be 5% of clock
- Hold uncertainty and data transition will be 8% of clock. 

```
cd /home/eshwar/OpenSTA/app
./sta

read_liberty /home/eshwar/OpenSTA/lab10/sky130_fd_sc_hd__tt_025C_1v80.lib
read_verilog /home/eshwar/OpenSTA/lab10/eshwar_riscv_netlist.v
link_design rvmyth

create_clock -name clk -period 9.2 [get_ports clk]
set_clock_uncertainty [expr 0.05 * 9.2] -setup [get_clocks clk]
set_clock_uncertainty [expr 0.08 * 9.2] -hold [get_clocks clk]
set_clock_transition [expr 0.05 * 9.2] [get_clocks clk]
set_input_transition [expr 0.08 * 9.2] [all_inputs]

report_checks -path_delay max
report_checks -path_delay min
```

To execute the OpenSTA and obtain the timing reports, run the below command,
```
sta scripts/sta.conf
```
Following are contents of the sta.conf file,
```
read_liberty -min ./lib/sta/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty -max ./lib/sta/sky130_fd_sc_hd__tt_025C_1v80.lib
read_liberty -min ./lib/avsdpll.lib
read_liberty -max ./lib/avsdpll.lib
read_liberty -min ./lib/avsddac.lib
read_liberty -max ./lib/avsddac.lib
read_verilog ./src/module/vsdbabysoc_synth.v
link_design vsdbabysoc
read_sdc ./src/sdc/sta_post_synth.sdc
```

![Img](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_10/l10_5.png)

![Img](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_10/l10_6.png)

</details>

<details>
<summary>Lab Session 11</summary>
<br>

# PVT Corner Analysis for Synthesized VSDBabySoC using OpenSTA [29/10/24]:

The PVT corner represents the combination of Process, Voltage, and Temperature variations that a semiconductor chip may encounter during its operation. These variations can impact key factors like performance, power consumption, and reliability of the chip. To ensure the chip operates correctly under different conditions, simulations across these PVT corners are performed.

The following Tcl script (sta_pvt.tcl) can be used to run Static Timing Analysis (STA) across available PVT corners using the Sky130 library files:

```
set list_of_lib_files(1) "sky130_fd_sc_hd__ff_100C_1v65.lib"
set list_of_lib_files(2) "sky130_fd_sc_hd__ff_100C_1v95.lib"
set list_of_lib_files(3) "sky130_fd_sc_hd__ff_n40C_1v56.lib"
set list_of_lib_files(4) "sky130_fd_sc_hd__ff_n40C_1v65.lib"
set list_of_lib_files(5) "sky130_fd_sc_hd__ff_n40C_1v76.lib"
set list_of_lib_files(6) "sky130_fd_sc_hd__ff_n40C_1v95.lib"
set list_of_lib_files(7) "sky130_fd_sc_hd__ss_100C_1v40.lib"
set list_of_lib_files(8) "sky130_fd_sc_hd__ss_100C_1v60.lib"
set list_of_lib_files(9) "sky130_fd_sc_hd__ss_n40C_1v28.lib"
set list_of_lib_files(10) "sky130_fd_sc_hd__ss_n40C_1v35.lib"
set list_of_lib_files(11) "sky130_fd_sc_hd__ss_n40C_1v40.lib"
set list_of_lib_files(12) "sky130_fd_sc_hd__ss_n40C_1v44.lib"
set list_of_lib_files(13) "sky130_fd_sc_hd__ss_n40C_1v60.lib"
set list_of_lib_files(14) "sky130_fd_sc_hd__ss_n40C_1v76.lib"
set list_of_lib_files(15) "sky130_fd_sc_hd__tt_025C_1v80.lib"
set list_of_lib_files(16) "sky130_fd_sc_hd__tt_100C_1v80.lib"
for {set i 1} {$i <= [array size list_of_lib_files]} {incr i} {
    read_liberty /home/eshwar/VSDBabySoC/src/timing_libs/$list_of_lib_files($i)
    read_verilog /home/eshwar/VSDBabySoC/src/module/vsdbabysoc.synth.v
    link_design rvmyth
    read_sdc /home/eshwar/VSDBabySoC/src/sdc/vsdbabysoc_synthesis.sdc
    check_setup -verbose
    report_checks -path_delay min_max -fields {nets cap slew input_pins fanout} -digits {4} > /home/eshwar/VSDBabySoC/src/sta_output/min_max_$list_of_lib_files($i).txt

    exec echo "$list_of_lib_files($i)" >> /home/eshwar/VSDBabySoC/src/sta_output/sta_worst_max_slack.txt
    report_worst_slack -max -digits {4} >> /home/eshwar/VSDBabySoC/src/sta_output/sta_worst_max_slack.txt

    exec echo "$list_of_lib_files($i)" >> /home/eshwar/VSDBabySoC/src/sta_output/sta_worst_min_slack.txt
    report_worst_slack -min -digits {4} >> /home/eshwar/VSDBabySoC/src/sta_output/sta_worst_min_slack.txt

    exec echo "$list_of_lib_files($i)" >> /home/eshwar/VSDBabySoC/src/sta_output/sta_tns.txt
    report_tns -digits {4} >> /home/eshwar/VSDBabySoC/src/sta_output/sta_tns.txt

    exec echo "$list_of_lib_files($i)" >> /home/eshwar/VSDBabySoC/src/sta_output/sta_wns.txt
    report_wns -digits {4} >> /home/eshwar/VSDBabySoC/src/sta_output/sta_wns.txt
}
```

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_11/1.png)


![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_11/2.png)



The SDC file, which is used to define clock and data constraints, is provided below:

## SDC constraints for VSDBabySoC:

```
create_clock -name CLK -period 9.2 [get_ports CLK]
set_clock_uncertainty [expr 0.05 * 9.2] -setup [get_clocks CLK]
set_clock_uncertainty [expr 0.08 * 9.2] -hold [get_clocks CLK]
set_clock_transition [expr 0.05 * 9.2] [get_clocks CLK]
set_input_transition [expr 0.08 * 9.2] [all_inputs]

set_input_transition [expr $PERIOD * 0.08] [get_ports ENB_CP]
set_input_transition [expr $PERIOD * 0.08] [get_ports ENB_VCO]
set_input_transition [expr $PERIOD * 0.08] [get_ports REF]
set_input_transition [expr $PERIOD * 0.08] [get_ports VCO_IN]
set_input_transition [expr $PERIOD * 0.08] [get_ports VREFH]

```

Run below commands on terminal to source the sta_pvt.tcl file:

```
source /home/eshwar/VSDBabySoC/src/tcl/sta_pvt.tcl
```

## Analysis Report:

## Table of slack report:

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_11/3.png)


## Total Negative Slack (ns):

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_11/4.png)

## Worst (Negative slack) Setup Slack (ns):

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_11/5.png)

## Worst Setup Slack (ns):

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_11/6.png)

## Worst Hold Slack (ns):

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/lab_11/7.png)


The analysis report shows the following key points:

1. **Worst Setup Slack:** sky130_fd_sc_hd__ss_n40C_1v28.lib library file has the worst setup slack.
2. **Worst Hold Slack:** sky130_fd_sc_hd__ff_100C_1v95.lib library file has the worst hold slack.

The total negative slack and worst negative slacks are provided in the detailed slacks report screenshots.

</details>


</details>

<details> 
<summary>Lab Session 12</summary>

# Complete the Advanced Physical Design using OpenLane workshop on VSDIAT platform. Create an inverter incorporating your name and document all laboratory exercises

<details> 
<summary> Day 1 - Introduction to Open-source EDA, OpenLANE, and Sky130 PDK </summary>


Perform synthesis for the 'picorv32a' design using OpenLANE. Follow these commands to start the OpenLANE flow and complete the synthesis process:

```bash
# Navigate to the OpenLANE flow directory
cd Desktop/work/tools/openlane_working_dir/openlane

# alias docker='docker run -it -v $(pwd):/openLANE_flow -v $PDK_ROOT:$PDK_ROOT -e PDK_ROOT=$PDK_ROOT -u $(id -u $USER):$(id -g $USER) efabless/openlane:v0.21'
# Since the docker command has been aliased, use 'docker' to launch the OpenLANE flow in a container
docker

# Once inside the docker container, open the OpenLANE flow in interactive mode with the following command
./flow.tcl -interactive

# Load the required OpenLANE package version 0.9 for proper operation
package require openlane 0.9

# Prepare the design environment for 'picorv32a' by running the prep command
prep -design picorv32a

# With the design prepared, start the synthesis process
run_synthesis

# Exit the OpenLANE environment
exit

# Exit the docker container
exit
```

## Relevant screenshots:

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_12/day1/day1_1.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_12/day1/day1_2.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_12/day1/day1_3.png)

![image](https://github.com/EshwarAllampally/asic-design-class/blob/main/Lab_12/day1/day1_4.png)


Calculation of Flop Ratio and DFF % based on the synthesis statistics report.



</details>

<details>

<summary>Day 2:  Good floorplan vs bad floorplan and introduction to library cells</summary> 

## 1. Run 'picorv32a' design floorplan using OpenLANE flow and generate necessary outputs.
Commands to invoke the OpenLANE flow and perform floorplan
```
# Change directory to openlane flow directory
cd Desktop/work/tools/openlane_working_dir/openlane

# alias docker='docker run -it -v $(pwd):/openLANE_flow -v $PDK_ROOT:$PDK_ROOT -e PDK_ROOT=$PDK_ROOT -u $(id -u $USER):$(id -g $USER) efabless/openlane:v0.21'
# Since we have aliased the long command to 'docker' we can invoke the OpenLANE flow docker sub-system by just running this command
docker
# Now that we have entered the OpenLANE flow contained docker sub-system we can invoke the OpenLANE flow in the Interactive mode using the following command
./flow.tcl -interactive

# Now that OpenLANE flow is open we have to input the required packages for proper functionality of the OpenLANE flow
package require openlane 0.9

# Now the OpenLANE flow is ready to run any design and initially we have to prep the design creating some necessary files and directories for running a specific design which in our case is 'picorv32a'
prep -design picorv32a

# Now that the design is prepped and ready, we can run synthesis using following command
run_synthesis

# Now we can run floorplan
run_floorplan

```

Screenshot of floorplan run
![image](https://github.com/user-attachments/assets/cad1ca0a-20fa-49ba-ae21-c91ce5d09116)

![image](https://github.com/user-attachments/assets/bcde11ad-9e5c-461a-8750-29c4bb51bfa8)

## 2. Calculate the die area in microns from the values in floorplan def.

Screenshot of contents of floorplan def
![image](https://github.com/user-attachments/assets/1dceeb35-ac5d-456e-887c-6771837c2365)

## 3. Load generated floorplan def in magic tool and explore the floorplan.
Commands to load floorplan def in magic in another terminal
```
# Change directory to path containing generated floorplan def
cd Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/17-03_12-06/results/floorplan/

# Command to load the floorplan def in magic tool
magic -T /home/vsduser/Desktop/work/tools/openlane_working_dir/pdks/sky130A/libs.tech/magic/sky130A.tech lef read ../../tmp/merged.lef def read picorv32a.floorplan.def &
```
Screenshots of floorplan def in magic
![image](https://github.com/user-attachments/assets/9963022b-d9a6-43e0-bc6d-20269437f599)

Equidistant placement of ports
![2 6](https://github.com/user-attachments/assets/18ba5ca0-606e-4df4-b663-7b04c16abc01)

Port layer as set through config.tcl
![image](https://github.com/user-attachments/assets/8663829e-1468-421f-bae2-e7a1632f8365)
![image](https://github.com/user-attachments/assets/489ae307-d941-4f7b-b537-accd86cebdba)

Decap Cells and Tap Cells

![image](https://github.com/user-attachments/assets/fc6e401b-4619-4804-87fb-5407cc42bf6d)

Unplaced standard cells at the origin
![2 8](https://github.com/user-attachments/assets/2cd7f768-a3ba-4989-a7cf-1384f10d8cf3)

### 4. Run 'picorv32a' design congestion aware placement using OpenLANE flow and generate necessary outputs.
Command to run placement
```
# Congestion aware placement by default
run_placement
```

Screenshots of placement run

![image](https://github.com/user-attachments/assets/11e3c9a4-0125-42e4-82f6-3ff975597d2a)

### 5. Load generated placement def in magic tool and explore the placement.
Commands to load placement def in magic in another terminal
```
# Change directory to path containing generated placement def
cd Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/17-03_12-06/results/placement/

# Command to load the placement def in magic tool
magic -T /home/vsduser/Desktop/work/tools/openlane_working_dir/pdks/sky130A/libs.tech/magic/sky130A.tech lef read ../../tmp/merged.lef def read picorv32a.placement.def &
```
Screenshots of floorplan def in magic

![image](https://github.com/user-attachments/assets/7c378875-931e-4406-8b27-945585d07a32)
![2 15](https://github.com/user-attachments/assets/b1288cac-9005-4ebe-a9fd-35e8a6282540)

Commands to exit from current run
```
# Exit from OpenLANE flow
exit

# Exit from OpenLANE flow docker sub-system
exit
```
</details>

<details>
	<summary>Day 3: Design library cell using Magic Layout and ngspice characterization</summary>

 1. Clone custom inverter standard cell design from github repository
  ```  
# Change directory to openlane
cd Desktop/work/tools/openlane_working_dir/openlane

# Clone the repository with custom inverter design
git clone https://github.com/nickson-jose/vsdstdcelldesign

# Change into repository directory
cd vsdstdcelldesign

# Copy magic tech file to the repo directory for easy access
cp /home/vsduser/Desktop/work/tools/openlane_working_dir/pdks/sky130A/libs.tech/magic/sky130A.tech .

# Check contents whether everything is present
ls

# Command to open custom inverter layout in magic
magic -T sky130A.tech sky130_inv.mag &
```
![3 1](https://github.com/user-attachments/assets/b84fd26e-0af8-420a-95cf-7f7b3d63eded)

### 2. Load the custom inverter layout in magic and explore.
Screenshot of custom inverter layout in magic

![image](https://github.com/user-attachments/assets/2be8f2c4-9f3d-45d4-93e7-502c769dd63e)

NMOS and PMOS identified

![3 3](https://github.com/user-attachments/assets/a75f41ac-54d8-4c7d-b2e5-fe4f5525822f)

![image](https://github.com/user-attachments/assets/ad5e2986-75af-46d4-80ef-823f22fd3d22)

Output Y connectivity to PMOS and NMOS drain verified

![image](https://github.com/user-attachments/assets/ef029b79-9c43-4647-b905-868ec5a63f6c)

NMOS source connectivity to VSS (here VGND) verified

![image](https://github.com/user-attachments/assets/a4c779c9-3e5d-4f6d-83b0-db970835158d)
Deleting necessary files

![image](https://github.com/user-attachments/assets/6f44fd16-abcb-40dd-af21-85a17df15c15)

## 3. Spice extraction of inverter in magic.
Commands for spice extraction of the custom inverter layout to be used in tkcon window of magic
```
# Check current directory
pwd

# Extraction command to extract to .ext format
extract all

# Before converting ext to spice this command enable the parasitic extraction also
ext2spice cthresh 0 rthresh 0

# Converting to ext to spice
ext2spice
```
Screenshot of tkcon window after running above commands
![image](https://github.com/user-attachments/assets/6effe560-fda8-40c0-9882-d1bf193d673c)

Screenshot of created spice file

![image](https://github.com/user-attachments/assets/4522a8e8-81eb-418f-a58a-4a8b63c9b649)

## 4. Editing the spice model file for analysis through simulation.
Measuring unit distance in layout grid

![image](https://github.com/user-attachments/assets/1e602dad-6106-4326-a18d-b061cd0f75c1)

Final edited spice file ready for ngspice simulation

![image](https://github.com/user-attachments/assets/01e95d50-15e7-488f-8605-2cc4bb73691c)

## 5. Post-layout ngspice simulations.
Commands for ngspice simulation
```
# Command to directly load spice file for simulation to ngspice
ngspice sky130_inv.spice

# Now that we have entered ngspice with the simulation spice file loaded we just have to load the plot
plot y vs time a
```
Screenshots of ngspice run
![3 11](https://github.com/user-attachments/assets/ef624724-309c-4261-8953-334bf87a95aa)

![3 12](https://github.com/user-attachments/assets/14331b9e-a6c4-4be5-9fa0-f567d390060d)

Plot:

![image](https://github.com/user-attachments/assets/5690d86e-c55d-48e6-902d-6e05064756ba)

Using this transient response, we will now characterize the cell's slew rate and propagation delay:

Rise Transition: Time taken for the output to rise from 20% to 80% of max value Fall Transition: Time taken for the output to fall from 80% to 20% of max value Cell Rise delay: difference in time(50% output rise) to time(50% input fall) Cell Fall delay: difference in time(50% output fall) to time(50% input rise)

![image](https://github.com/user-attachments/assets/76847a07-682b-4813-a533-2ad5785fa980)
```
Rise Transition : 2.2341 - 2.0833 =  0.1508 ns = 150.8  ps
Fall Transition : 4 - 4.05536 =  0.05536 ns = 55.36 ps
Cell Rise Delay : 2.25 - 2.0277 = 0.02223 ns = 22.23 ps
Cell Fall Delay : 4.07807 - 4.05 =0.02 ns = 20 ps
```
### 6. Find problem in the DRC section of the old magic tech file for the skywater process and fix them.

```
Commands to download and view the corrupted skywater process magic tech file and associated files to perform drc corrections

# Change to home directory
cd

# Command to download the lab files
wget http://opencircuitdesign.com/open_pdks/archive/drc_tests.tgz

# Since lab file is compressed command to extract it
tar xfz drc_tests.tgz

# Change directory into the lab folder
cd drc_tests

# List all files and directories present in the current directory
ls -al

# Command to view .magicrc file
gvim .magicrc

# Command to open magic tool in better graphics
magic -d XR &
```
snapshot of the command run

![image](https://github.com/user-attachments/assets/d78a60b4-6227-49c3-abbf-0a403568cf7c)


Screenshot of .magicrc file
![image](https://github.com/user-attachments/assets/7ad74a09-5cba-4af7-84b1-61bd94121dc0)

Incorrectly implemented poly.9 rule no drc violation even though spacing < 0.48u

![image](https://github.com/user-attachments/assets/96566cc1-c23c-40b7-b3ca-d065ff5310b4)
![image](https://github.com/user-attachments/assets/b763c9af-8f59-4907-9a9c-c2ba1cc959df)

New commands inserted in sky130A.tech file to update drc

![image](https://github.com/user-attachments/assets/0cbce4cc-7b8b-4543-87ce-c37494d9d8e2)

![image](https://github.com/user-attachments/assets/536a15a1-659c-491c-9a07-22a293ac87c6)

Commands to run in tkcon window
```
# Loading updated tech file
tech load sky130A.tech

# Must re-run drc check to see updated drc errors
drc check

# Selecting region displaying the new errors and getting the error messages 
drc why
```
Screenshot of magic window with rule implemented
![image](https://github.com/user-attachments/assets/89ff8296-ef2b-4583-939e-b08adb738a5c)

</details>
<details>
<summary>Day 4 Pre-layout timing analysis and importance of good clock tree </summary>	

Commands to extract tracks.info file:
```
cd Desktop/work/tools/openlane_working_dir/openlane/vsdstdcelldesign
cd ../../pdks/sky130A/libs.tech/openlane/sky130_fd_sc_hd/
less tracks.info
```
![image](https://github.com/user-attachments/assets/babb73d5-1e55-4e87-abf9-da5882ce02e5)

Commands for tkcon window to set grid as tracks of locali layer
```
# Get syntax for grid command
help grid

# Set grid values accordingly
grid 0.46um 0.34um 0.23um 0.17um
```
Screenshot of commands run
![image](https://github.com/user-attachments/assets/24341d5e-342c-4c36-a848-e521a19acc27)

 ![image](https://github.com/user-attachments/assets/25d7124b-d920-4d5f-814d-9f9c13386cfb)

Condition 1 Verified
![image](https://github.com/user-attachments/assets/10fe6b3e-0b8f-473f-b825-7e393d820358)

Condtion 2 verified
![image](https://github.com/user-attachments/assets/153d33f8-b9af-421d-83b1-01fe13db0a7d)

## 2. Save the finalized layout with custom name and open it.
Command for tkcon window to save the layout with custom name
```
# Command to save as
save sky130_eeshinv.mag
```
Command to open the newly saved layout
```
# Command to open custom inverter layout in magic
magic -T sky130A.tech sky130_eeshinv.mag &
```
Screenshot of newly saved layout
![image](https://github.com/user-attachments/assets/3ebaa5da-7739-41b9-9139-db4e8661ca40)

### 3. Generate lef from the layout.
Command for tkcon window to write lef
```
# lef command
lef write
```
Screenshot of command run
![image](https://github.com/user-attachments/assets/95563ed6-07ee-4a8c-9cfd-39bcb67f6241)

### 4. Copy the newly generated lef and associated required lib files to 'picorv32a' design 'src' directory.
Commands to copy necessary files to 'picorv32a' design 'src' directory
```
# Copy lef file
cp sky130_eeshinv.lef ~/Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/src/

# List and check whether it's copied
ls ~/Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/src/

# Copy lib files
cp libs/sky130_fd_sc_hd__* ~/Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/src/

# List and check whether it's copied
ls ~/Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/src/
```
Screenshot of commands run
![image](https://github.com/user-attachments/assets/c404c59d-a60d-400d-be42-c38124c4c0b2)

### 5. Edit 'config.tcl' to change lib file and add the new extra lef into the openlane flow.
Commands to be added to config.tcl to include our custom cell in the openlane flow
```
set ::env(LIB_SYNTH) "$::env(OPENLANE_ROOT)/designs/picorv32a/src/sky130_fd_sc_hd__typical.lib"
set ::env(LIB_FASTEST) "$::env(OPENLANE_ROOT)/designs/picorv32a/src/sky130_fd_sc_hd__fast.lib"
set ::env(LIB_SLOWEST) "$::env(OPENLANE_ROOT)/designs/picorv32a/src/sky130_fd_sc_hd__slow.lib"
set ::env(LIB_TYPICAL) "$::env(OPENLANE_ROOT)/designs/picorv32a/src/sky130_fd_sc_hd__typical.lib"

set ::env(EXTRA_LEFS) [glob $::env(OPENLANE_ROOT)/designs/$::env(DESIGN_NAME)/src/*.lef]
```
Edited `config.tcl` to include the added lef and change library to ones we added in src directory
![image](https://github.com/user-attachments/assets/26f288bc-d190-40c3-ad40-7bf595056c54)

### 6. Run openlane flow synthesis with newly inserted custom inverter cell.
Commands to invoke the OpenLANE flow include new lef and perform synthesis
```
# Change directory to openlane flow directory
cd Desktop/work/tools/openlane_working_dir/openlane

# alias docker='docker run -it -v $(pwd):/openLANE_flow -v $PDK_ROOT:$PDK_ROOT -e PDK_ROOT=$PDK_ROOT -u $(id -u $USER):$(id -g $USER) efabless/openlane:v0.21'
# Since we have aliased the long command to 'docker' we can invoke the OpenLANE flow docker sub-system by just running this command
docker
# Now that we have entered the OpenLANE flow contained docker sub-system we can invoke the OpenLANE flow in the Interactive mode using the following command
./flow.tcl -interactive

# Now that OpenLANE flow is open we have to input the required packages for proper functionality of the OpenLANE flow
package require openlane 0.9

# Now the OpenLANE flow is ready to run any design and initially we have to prep the design creating some necessary files and directories for running a specific design which in our case is 'picorv32a'
prep -design picorv32a

# Adiitional commands to include newly added lef to openlane flow
set lefs [glob $::env(DESIGN_DIR)/src/*.lef]
add_lefs -src $lefs

# Now that the design is prepped and ready, we can run synthesis using following command
run_synthesis
```

Screenshots of commands run

![image](https://github.com/user-attachments/assets/61a7bb3b-dfbe-4ed2-9765-099974e0349e)
![image](https://github.com/user-attachments/assets/aa842e9b-bcf5-41fd-bd71-a72b72bbafd7)

### 7. Remove/reduce the newly introduced violations with the introduction of custom inverter cell by modifying design parameters.
Noting down current design values generated before modifying parameters to improve timing

![image](https://github.com/user-attachments/assets/b9fae641-d024-4148-9f23-9a750f85b2f2)
![image](https://github.com/user-attachments/assets/9d000a7a-c7c8-4ce2-b5da-6899747aff5a)

Commands to view and change parameters to improve timing and run synthesis
```
# Now once again we have to prep design so as to update variables
prep -design picorv32a -tag 24-03_10-03 -overwrite

# Addiitional commands to include newly added lef to openlane flow merged.lef
set lefs [glob $::env(DESIGN_DIR)/src/*.lef]
add_lefs -src $lefs

# Command to display current value of variable SYNTH_STRATEGY
echo $::env(SYNTH_STRATEGY)

# Command to set new value for SYNTH_STRATEGY
set ::env(SYNTH_STRATEGY) "DELAY 3"

# Command to display current value of variable SYNTH_BUFFERING to check whether it's enabled
echo $::env(SYNTH_BUFFERING)

# Command to display current value of variable SYNTH_SIZING
echo $::env(SYNTH_SIZING)

# Command to set new value for SYNTH_SIZING
set ::env(SYNTH_SIZING) 1

# Command to display current value of variable SYNTH_DRIVING_CELL to check whether it's the proper cell or not
echo $::env(SYNTH_DRIVING_CELL)

# Now that the design is prepped and ready, we can run synthesis using following command
run_synthesis
```
Screenshot of merged.lef in tmp directory with our custom inverter as macro
![image](https://github.com/user-attachments/assets/45efab6a-3193-4cfa-9835-c2b3ec537552)

![image](https://github.com/user-attachments/assets/21622af8-ed9b-41b0-bb91-54021d81c72d)

### 8. Once synthesis has accepted our custom inverter we can now run floorplan and placement and verify the cell is accepted in PnR flow.
Now that our custom inverter is properly accepted in synthesis we can now run floorplan using following command
```
# Now we can run floorplan
run_floorplan
```
Screenshots of command run
![image](https://github.com/user-attachments/assets/ff9e2416-2fda-4d88-a7c0-fb475fcdf25a)
![image](https://github.com/user-attachments/assets/c46fc237-efbf-4a6b-bd38-ff37721a3a60)
```
# Follwing commands are alltogather sourced in "run_floorplan" command
init_floorplan
place_io
tap_decap_or
```
Screenshots of commands run
![image](https://github.com/user-attachments/assets/5d926aaf-4c03-46f5-8486-62b60aa77eb5)

Now that floorplan is done we can do placement using following command
```
# Now we are ready to run placement
run_placement
```
Screenshots of command run
![image](https://github.com/user-attachments/assets/4c43d18a-c256-4044-9652-709e1de7d97d)


Commands to load placement def in magic in another terminal
```
# Change directory to path containing generated placement def
cd Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/24-03_10-03/results/placement/

# Command to load the placement def in magic tool
magic -T /home/vsduser/Desktop/work/tools/openlane_working_dir/pdks/sky130A/libs.tech/magic/sky130A.tech lef read ../../tmp/merged.lef def read picorv32a.placement.def &
```
Screenshot of placement def in magic
![image](https://github.com/user-attachments/assets/32b49846-8f93-4624-8183-e6199d558783)

Screenshot of custom inverter inserted in placement def with proper abutment

![image](https://github.com/user-attachments/assets/65a24b14-1665-4fc3-ba16-d6b14aafd38d)

Command for tkcon window to view internal layers of cells

```
# Command to view internal connectivity layers
expand
```
Abutment of power pins with other cell from library clearly visible
![image](https://github.com/user-attachments/assets/e741432c-3bff-416b-b34e-bf6ec35b4803)

### 9. Do Post-Synthesis timing analysis with OpenSTA tool.
Since we are having 0 wns after improved timing run we are going to do timing analysis on initial run of synthesis which has lots of violations and no parameters were added to improve timing

Commands to invoke the OpenLANE flow include new lef and perform synthesis
```
# Change directory to openlane flow directory
cd Desktop/work/tools/openlane_working_dir/openlane

# alias docker='docker run -it -v $(pwd):/openLANE_flow -v $PDK_ROOT:$PDK_ROOT -e PDK_ROOT=$PDK_ROOT -u $(id -u $USER):$(id -g $USER) efabless/openlane:v0.21'
# Since we have aliased the long command to 'docker' we can invoke the OpenLANE flow docker sub-system by just running this command
docker
# Now that we have entered the OpenLANE flow contained docker sub-system we can invoke the OpenLANE flow in the Interactive mode using the following command
./flow.tcl -interactive

# Now that OpenLANE flow is open we have to input the required packages for proper functionality of the OpenLANE flow
package require openlane 0.9

# Now the OpenLANE flow is ready to run any design and initially we have to prep the design creating some necessary files and directories for running a specific design which in our case is 'picorv32a'
prep -design picorv32a

# Adiitional commands to include newly added lef to openlane flow
set lefs [glob $::env(DESIGN_DIR)/src/*.lef]
add_lefs -src $lefs

# Command to set new value for SYNTH_SIZING
set ::env(SYNTH_SIZING) 1

# Now that the design is prepped and ready, we can run synthesis using following command
run_synthesis
```

Commands run final screenshot
![image](https://github.com/user-attachments/assets/7304f8b3-e56e-4753-a5fa-fa422c221991)
![image](https://github.com/user-attachments/assets/e97b85a8-2c15-4ec0-a74a-9761b0b041c4)
![image](https://github.com/user-attachments/assets/3cd61e7a-b1f4-4513-af2a-3a6c6df55031)

Newly created pre_sta.conf for STA analysis in openlane directory
![image](https://github.com/user-attachments/assets/67721292-c579-4b34-856e-48fce14f5401)

Newly created` my_base.sdc` for STA analysis in openlane/designs/picorv32a/src directory based on the file `openlane/scripts/base.sdc`

![image](https://github.com/user-attachments/assets/8468dea0-8345-4016-b7e4-d2f20e076c16)

Commands to run STA in another terminal
```
# Change directory to openlane
cd Desktop/work/tools/openlane_working_dir/openlane

# Command to invoke OpenSTA tool with script
sta pre_sta.conf
```
Screenshots of commands run
![image](https://github.com/user-attachments/assets/184b8c1f-26e0-4e60-a105-0bb4cb239c81)
![image](https://github.com/user-attachments/assets/00518004-59cf-4c60-b1ba-992da0beecfe)

Since more fanout is causing more delay we can add parameter to reduce fanout and do synthesis again

Commands to include new lef and perform synthesis
```
# Now the OpenLANE flow is ready to run any design and initially we have to prep the design creating some necessary files and directories for running a specific design which in our case is 'picorv32a'
prep -design picorv32a -tag 25-03_18-52 -overwrite

# Adiitional commands to include newly added lef to openlane flow
set lefs [glob $::env(DESIGN_DIR)/src/*.lef]
add_lefs -src $lefs

# Command to set new value for SYNTH_SIZING
set ::env(SYNTH_SIZING) 1

# Command to set new value for SYNTH_MAX_FANOUT
set ::env(SYNTH_MAX_FANOUT) 4

# Command to display current value of variable SYNTH_DRIVING_CELL to check whether it's the proper cell or not
echo $::env(SYNTH_DRIVING_CELL)

# Now that the design is prepped and ready, we can run synthesis using following command
run_synthesis
```

Commands run final screenshot

![image](https://github.com/user-attachments/assets/b6c0d63d-8332-44f3-8bf8-ba1c310342e4)

![image](https://github.com/user-attachments/assets/6cb4961f-5724-4eed-a581-a51b9fce2ede)

Commands to run STA in another terminal
```
# Change directory to openlane
cd Desktop/work/tools/openlane_working_dir/openlane

# Command to invoke OpenSTA tool with script
sta pre_sta.conf
```
Screenshots of commands run
![image](https://github.com/user-attachments/assets/d808f3a6-c406-41c2-8fb6-854d66c8dd2e)

## 10. Make timing ECO fixes to remove all violations.
OR gate of drive strength 2 is driving 4 fanouts
![image](https://github.com/user-attachments/assets/65c60248-6fd1-46b2-8096-07138e8df883)

Commands to perform analysis and optimize timing by replacing with OR gate of drive strength 4
```
# Reports all the connections to a net
report_net -connections _11672_

# Checking command syntax
help replace_cell

# Replacing cell
replace_cell _14510_ sky130_fd_sc_hd__or3_4

# Generating custom timing report
report_checks -fields {net cap slew input_pins} -digits 4
```
Result - slack reduced

![image](https://github.com/user-attachments/assets/e90dee0a-27f7-408e-b973-f9c97811ab82)

Commands to perform analysis and optimize timing by replacing with OR gate of drive strength 4
```
# Reports all the connections to a net
report_net -connections _11675_

# Replacing cell
replace_cell _14514_ sky130_fd_sc_hd__or3_4

# Generating custom timing report
report_checks -fields {net cap slew input_pins} -digits 4
```
![image](https://github.com/user-attachments/assets/4a71c69f-9bc7-4ffa-a508-81f3aeedcb26)

Commands to perform analysis and optimize timing by replacing with OR gate of drive strength 4
```
# Reports all the connections to a net
report_net -connections _11643_

# Replacing cell
replace_cell _14481_ sky130_fd_sc_hd__or4_4

# Generating custom timing report
report_checks -fields {net cap slew input_pins} -digits 4
```
Result - slack reduced
![image](https://github.com/user-attachments/assets/6e4c1e4a-3f1c-454e-a0cf-c0eb93ba2db8)

Commands to perform analysis and optimize timing by replacing with OR gate of drive strength 4
```
# Reports all the connections to a net
report_net -connections _11668_

# Replacing cell
replace_cell _14506_ sky130_fd_sc_hd__or4_4

# Generating custom timing report
report_checks -fields {net cap slew input_pins} -digits 4
```

Result - slack reduced
![image](https://github.com/user-attachments/assets/4b0513e5-f7e0-4c0c-8ea4-aaebe7c846c9)

Commands to verify instance _14506_ is replaced with sky130_fd_sc_hd__or4_4
```
# Generating custom timing report
report_checks -from _29043_ -to _30440_ -through _14506_
```

Screenshot of replaced instance
![image](https://github.com/user-attachments/assets/32f7e3c9-2a69-4593-81b5-1f74f758519e)

## 11. Replace the old netlist with the new netlist generated after timing ECO fix and implement the floorplan, placement and cts.

Now to insert this updated netlist to PnR flow and we can use write_verilog and overwrite the synthesis netlist but before that we are going to make a copy of the old old netlist

Commands to make copy of netlist
```
# Change from home directory to synthesis results directory
cd Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/25-03_18-52/results/synthesis/

# List contents of the directory
ls

# Copy and rename the netlist
cp picorv32a.synthesis.v picorv32a.synthesis_old.v

# List contents of the directory
ls
```
Screenshot of commands run
![image](https://github.com/user-attachments/assets/b171008e-99f3-436f-bdf1-10c4ae0cdfe0)

Commands to write verilog
```
# Check syntax
help write_verilog

# Overwriting current synthesis netlist
write_verilog /home/vsduser/Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/24-03_10-03/results/synthesis/picorv32a.synthesis.v

# Exit from OpenSTA since timing analysis is done
exit
```
Screenshot of commands run
![image](https://github.com/user-attachments/assets/383111d6-efc7-45a7-ab2b-be4a3c1bb97b)

Verified that the netlist is overwritten by checking that instance _14506_ is replaced with sky130_fd_sc_hd__or4_4

![image](https://github.com/user-attachments/assets/32bacd7a-fd87-4094-b9a5-f2600ee41acc)

Since we confirmed that netlist is replaced and will be loaded in PnR but since we want to follow up on the earlier 0 violation design we are continuing with the clean design to further stages

Commands load the design and run necessary stages
```
# Now once again we have to prep design so as to update variables
prep -design picorv32a -tag 24-03_10-03 -overwrite

# Addiitional commands to include newly added lef to openlane flow merged.lef
set lefs [glob $::env(DESIGN_DIR)/src/*.lef]
add_lefs -src $lefs

# Command to set new value for SYNTH_STRATEGY
set ::env(SYNTH_STRATEGY) "DELAY 3"

# Command to set new value for SYNTH_SIZING
set ::env(SYNTH_SIZING) 1

# Now that the design is prepped and ready, we can run synthesis using following command
run_synthesis

# Follwing commands are alltogather sourced in "run_floorplan" command
init_floorplan
place_io
tap_decap_or

# Now we are ready to run placement
run_placement

# Incase getting error
unset ::env(LIB_CTS)

# With placement done we are now ready to run CTS
run_cts
```
Screenshots of commands run
![image](https://github.com/user-attachments/assets/47987520-3eb1-4590-ab15-b88868147522)

![image](https://github.com/user-attachments/assets/2773b1a5-d580-4efd-816d-cb8064887c29)



## 12. Post-CTS OpenROAD timing analysis.
Commands to be run in OpenLANE flow to do OpenROAD timing analysis with integrated OpenSTA in OpenROAD
```
# Command to run OpenROAD tool
openroad

# Reading lef file
read_lef /openLANE_flow/designs/picorv32a/runs/24-03_10-03/tmp/merged.lef

# Reading def file
read_def /openLANE_flow/designs/picorv32a/runs/24-03_10-03/results/cts/picorv32a.cts.def

# Creating an OpenROAD database to work with
write_db pico_cts.db

# Loading the created database in OpenROAD
read_db pico_cts.db

# Read netlist post CTS
read_verilog /openLANE_flow/designs/picorv32a/runs/24-03_10-03/results/synthesis/picorv32a.synthesis_cts.v

# Read library for design
read_liberty $::env(LIB_SYNTH_COMPLETE)

# Link design and library
link_design picorv32a

# Read in the custom sdc we created
read_sdc /openLANE_flow/designs/picorv32a/src/my_base.sdc

# Setting all cloks as propagated clocks
set_propagated_clock [all_clocks]

# Check syntax of 'report_checks' command
help report_checks

# Generating custom timing report
report_checks -path_delay min_max -fields {slew trans net cap input_pins} -format full_clock_expanded -digits 4

# Exit to OpenLANE flow
exit
```
Screenshots of commands run and timing report generated

![image](https://github.com/user-attachments/assets/bd1f8bc6-7579-4839-83e4-27e4b52a7f8a)
![image](https://github.com/user-attachments/assets/780f255b-5324-434e-9c46-660699f245b3)
![image](https://github.com/user-attachments/assets/2b848c5c-05cb-4e95-820e-14281643645d)

## 13. Explore post-CTS OpenROAD timing analysis by removing 'sky130_fd_sc_hd__clkbuf_1' cell from clock buffer list variable 'CTS_CLK_BUFFER_LIST'.

Commands to be run in OpenLANE flow to do OpenROAD timing analysis after changing `CTS_CLK_BUFFER_LIST`
```
# Checking current value of 'CTS_CLK_BUFFER_LIST'
echo $::env(CTS_CLK_BUFFER_LIST)

# Removing 'sky130_fd_sc_hd__clkbuf_1' from the list
set ::env(CTS_CLK_BUFFER_LIST) [lreplace $::env(CTS_CLK_BUFFER_LIST) 0 0]

# Checking current value of 'CTS_CLK_BUFFER_LIST'
echo $::env(CTS_CLK_BUFFER_LIST)

# Checking current value of 'CURRENT_DEF'
echo $::env(CURRENT_DEF)

# Setting def as placement def
set ::env(CURRENT_DEF) /openLANE_flow/designs/picorv32a/runs/24-03_10-03/results/placement/picorv32a.placement.def

# Run CTS again
run_cts

# Checking current value of 'CTS_CLK_BUFFER_LIST'

# Command to run OpenROAD tool
openroad

# Reading lef file
read_lef /openLANE_flow/designs/picorv32a/runs/24-03_10-03/tmp/merged.lef

# Reading def file
read_def /openLANE_flow/designs/picorv32a/runs/24-03_10-03/results/cts/picorv32a.cts.def

# Creating an OpenROAD database to work with
write_db pico_cts1.db

# Loading the created database in OpenROAD
read_db pico_cts.db

# Read netlist post CTS
read_verilog /openLANE_flow/designs/picorv32a/runs/24-03_10-03/results/synthesis/picorv32a.synthesis_cts.v

# Read library for design
read_liberty $::env(LIB_SYNTH_COMPLETE)

# Link design and library
link_design picorv32a

# Read in the custom sdc we created
read_sdc /openLANE_flow/designs/picorv32a/src/my_base.sdc

# Setting all cloks as propagated clocks
set_propagated_clock [all_clocks]

# Generating custom timing report
report_checks -path_delay min_max -fields {slew trans net cap input_pins} -format full_clock_expanded -digits 4

# Report hold skew
report_clock_skew -hold

# Report setup skew
report_clock_skew -setup

# Exit to OpenLANE flow
exit

# Checking current value of 'CTS_CLK_BUFFER_LIST'
echo $::env(CTS_CLK_BUFFER_LIST)

# Inserting 'sky130_fd_sc_hd__clkbuf_1' to first index of list
set ::env(CTS_CLK_BUFFER_LIST) [linsert $::env(CTS_CLK_BUFFER_LIST) 0 sky130_fd_sc_hd__clkbuf_1]

# Checking current value of 'CTS_CLK_BUFFER_LIST'
echo $::env(CTS_CLK_BUFFER_LIST)
```
Screenshots of commands run and timing report generated
![image](https://github.com/user-attachments/assets/c1ec1ff3-3930-49d2-8d41-4985adaf0cbc)
![image](https://github.com/user-attachments/assets/48b366cd-9dba-4e53-b6ca-f3c7f7c045fd)
![image](https://github.com/user-attachments/assets/f58fa1bb-fa3e-40eb-ba1a-8714bb0d08d1)
![image](https://github.com/user-attachments/assets/fbaf0dca-e222-4ad1-a6b6-02d4916b26cf)
![image](https://github.com/user-attachments/assets/42a36bde-f886-431c-b46c-5e9b555f7e11)

</details>

<details>
<summary>Day 5: Final steps for RTL2GDS using tritonRoute and openSTA:</summary>


### Theory:

**Implementation:**

* Section 5 tasks:-
  
1) Perform generation of Power Distribution Network (PDN) and explore the PDN layout.
2) Perfrom detailed routing using TritonRoute.
3) Post-Route parasitic extraction using SPEF extractor.
4) Post-Route OpenSTA timing analysis with the extracted parasitics of the route.
   
* All section 5 logs, reports and results can be found in following run folder:

**1. Perform generation of Power Distribution Network (PDN) and explore the PDN layout:**

Commands to perform all necessary stages up until now

```
# Change directory to openlane flow directory
cd Desktop/work/tools/openlane_working_dir/openlane

# alias docker='docker run -it -v $(pwd):/openLANE_flow -v $PDK_ROOT:$PDK_ROOT -e PDK_ROOT=$PDK_ROOT -u $(id -u $USER):$(id -g $USER) efabless/openlane:v0.21'
# Since we have aliased the long command to 'docker' we can invoke the OpenLANE flow docker sub-system by just running this command
docker
```
```
# Now that we have entered the OpenLANE flow contained docker sub-system we can invoke the OpenLANE flow in the Interactive mode using the following command
./flow.tcl -interactive

# Now that OpenLANE flow is open we have to input the required packages for proper functionality of the OpenLANE flow
package require openlane 0.9

# Now the OpenLANE flow is ready to run any design and initially we have to prep the design creating some necessary files and directories for running a specific design which in our case is 'picorv32a'
prep -design picorv32a

# Addiitional commands to include newly added lef to openlane flow merged.lef
set lefs [glob $::env(DESIGN_DIR)/src/*.lef]
add_lefs -src $lefs

# Command to set new value for SYNTH_STRATEGY
set ::env(SYNTH_STRATEGY) "DELAY 3"

# Command to set new value for SYNTH_SIZING
set ::env(SYNTH_SIZING) 1

# Now that the design is prepped and ready, we can run synthesis using following command
run_synthesis

# Following commands are alltogather sourced in "run_floorplan" command
init_floorplan
place_io
tap_decap_or

# Now we are ready to run placement
run_placement

# Incase getting error
unset ::env(LIB_CTS)

# With placement done we are now ready to run CTS
run_cts

# Now that CTS is done we can do power distribution network
gen_pdn
```
**Screenshots of power distribution network run:**
![image](https://github.com/user-attachments/assets/c5897e5b-7741-4178-81b4-8e563dafa9a0)


![image](https://github.com/user-attachments/assets/4e373443-3c44-4a97-a786-bc5ca0a6ba0d)


**Commands to load PDN def in magic in another terminal:**

```
# Change directory to path containing generated PDN def
cd Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/14-11_19-03/tmp/floorplan/

# Command to load the PDN def in magic tool
magic -T /home/vsduser/Desktop/work/tools/openlane_working_dir/pdks/sky130A/libs.tech/magic/sky130A.tech lef read ../../tmp/merged.lef def read 14-pdn.def &
```

**Screenshots of PDN def:**

![image](https://github.com/user-attachments/assets/a9400961-f3da-4cd4-a42a-420cab9981c9)

![image](https://github.com/user-attachments/assets/9212b66c-5afa-4542-9f35-d53a536a42c1)

![image](https://github.com/user-attachments/assets/eeccc6e5-6b8a-499a-bc96-4f6c5e5fd23e)



**2. Perfrom detailed routing using TritonRoute and explore the routed layout.**

**Command to perform routing**
```
# Check value of 'CURRENT_DEF'
echo $::env(CURRENT_DEF)

# Check value of 'ROUTING_STRATEGY'
echo $::env(ROUTING_STRATEGY)

# Command for detailed route using TritonRoute
run_routing
```

**Screenshots of routing run:**

![image](https://github.com/user-attachments/assets/f0266676-3568-493f-82fa-1e2c48346117)


![image](https://github.com/user-attachments/assets/8a153f9b-7779-49ba-9462-168bc1fa96bb)


**Commands to load routed def in magic in another terminal:**
```
# Change directory to path containing routed def
cd Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/14-11_19-03/results/routing/

# Command to load the routed def in magic tool
magic -T /home/vsduser/Desktop/work/tools/openlane_working_dir/pdks/sky130A/libs.tech/magic/sky130A.tech lef read ../../tmp/merged.lef def read picorv32a.def &
```

**Screenshots of routed def:**

![WhatsApp Image 2024-11-15 at 01 38 02_6a2d2a92](https://github.com/user-attachments/assets/228033ea-11d3-4824-b07b-6d52c6a6641e)

![image](https://github.com/user-attachments/assets/779331d3-e21f-44fc-b305-9b0def0b90ab)

![image](https://github.com/user-attachments/assets/a2e248ff-bf6e-4f2d-bc92-b6b3ec175aae)



**Screenshot of fast route guide present in openlane/designs/picorv32a/runs/14-11_19-03/tmp/routing directory:**
![image](https://github.com/user-attachments/assets/a980da75-e4aa-407f-92ea-3d3c91585863)



**3. Post-Route parasitic extraction using SPEF extractor.:**

```
# Change directory
cd Desktop/work/tools/SPEF_EXTRACTOR

# Command extract spef
python3 main.py /home/vsduser/Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/14-11_19-03/tmp/merged.lef /home/vsduser/Desktop/work/tools/openlane_working_dir/openlane/designs/picorv32a/runs/14-11_19-03/results/routing/picorv32a.def
```


**4.Post-Route OpenSTA timing analysis with the extracted parasitics of the route:**

Commands to be run in OpenLANE flow to do OpenROAD timing analysis with integrated OpenSTA in OpenROAD
```
# Command to run OpenROAD tool
openroad

# Reading lef file
read_lef /openLANE_flow/designs/picorv32a/runs/14-11_19-03/tmp/merged.lef

# Reading def file
read_def /openLANE_flow/designs/picorv32a/runs/14-11_19-03/results/routing/picorv32a.def

# Creating an OpenROAD database to work with
write_db pico_route.db

# Loading the created database in OpenROAD
read_db pico_route.db

# Read netlist post CTS
read_verilog /openLANE_flow/designs/picorv32a/runs/14-11_19-03/results/synthesis/picorv32a.synthesis_preroute.v

# Read library for design
read_liberty $::env(LIB_SYNTH_COMPLETE)

# Link design and library
link_design picorv32a

# Read in the custom sdc we created
read_sdc /openLANE_flow/designs/picorv32a/src/my_base.sdc

# Setting all clocks as propagated clocks
set_propagated_clock [all_clocks]

# Read SPEF
read_spef /openLANE_flow/designs/picorv32a/runs/14-11_19-03/results/routing/picorv32a.spef

# Generating custom timing report
report_checks -path_delay min_max -fields {slew trans net cap input_pins} -format full_clock_expanded -digits 4

# Exit to OpenLANE flow
exit
```


Screenshots of commands run and timing report generated:

![image](https://github.com/user-attachments/assets/cdc10ca8-4424-4567-935a-1611779827e1)

![image](https://github.com/user-attachments/assets/02c6ab3f-30d5-40c4-b3fe-a55dd59ebbc8)

![image](https://github.com/user-attachments/assets/0b4a0eed-28b7-4ca7-a028-0a72524f52ed)


 </details>

 </details>

 <details>
  <summary>Lab Session 13</summary>

## OpenRoad Physical Design: [19/11/2024]

**Introduction:**

- **Bombe:** The Bombe was an electro-mechanical machine designed during World War II to decrypt German Enigma-encrypted messages. It was refined and built by Alan Turing and Gordon Welchman at Bletchley Park, UK. The Bombe systematically tested possible rotor settings of the Enigma machine by exploiting known plaintext patterns. Its logical operations helped narrow down the vast number of possible keys, significantly accelerating the decryption process. The Bombe played a critical role in the Allied war effort.

- **ENIAC (Electronic Numerical Integrator and Computer):** It was developed during World War II by John Presper Eckert and John Mauchly at the University of Pennsylvania, was the first general-purpose, fully electronic digital computer. Completed in 1945, it was designed to compute artillery firing tables for the U.S. Army. ENIAC used vacuum tubes instead of mechanical or electromechanical components. However, it lacked a stored-program capability, requiring manual reconfiguration for each new task. ENIAC demonstrated the immense potential of electronic computing for large-scale numerical problems.

- **EDVAC (Electronic Discrete Variable Automatic Computer):** EDVAC, also developed by Eckert and Mauchly with conceptual input from John von Neumann, was one of the first computers to implement the stored-program concept. Completed in 1949, EDVAC represented a significant improvement over ENIAC by using binary representation instead of decimal and storing both data and instructions in memory. This innovation simplified programming and laid the groundwork for the modern von Neumann architecture.

**50 Years of Microprocessor Trend Data:**

![1](https://github.com/user-attachments/assets/e1796aa4-6a97-4729-af0d-271bbc124164)

**The Key metrics are:**

- **Transistors (Orange Triangles):** The number of transistors on a microprocessor chip (in thousands) has increased exponentially, following Moore's Law, which predicts a doubling approximately every two years. This growth enabled more complex and capable processors, reaching the range of billions of transistors by the 2020s.
- **Single-Thread Performance (Blue Circles):** It is measured using SpecINT. It indicates the computational ability of a single processor core. Performance grew steadily due to improvements in architecture, instruction-level parallelism, and clock speeds, but the growth rate slowed post-2005 due to physical limitations like power and heat.
- **Frequency (Green Diamonds):** Processor clock speed (in MHz) rose steadily until the early 2000s but then stagnated as increasing clock speeds became inefficient due to heat dissipation issues.
- **Typical Power (Red Triangles):** Power consumption increased with transistor density and frequency, becoming a critical design challenge around the mid-2000s.
- **Number of Logical Cores (Black Dots):** The transition to multi-core processors gained momentum in the mid-2000s as a response to the stagnation in single-thread performance. By increasing the number of cores, processors enabled more efficient parallel processing, leading to significant improvements in overall performance

**Key Milestones**

- **iPhone Release (~2007):** Signals the emergence of mobile computing, where power efficiency became as crucial as performance. This catalyzed innovations in low-power processor designs.
- **Datacenter-Scale Computing (Post-2010):** Marks the rise of cloud computing and large-scale data centers, where energy efficiency, scalability, and parallelism became central concerns.

**Path to zetta-scale computing**

![2](https://github.com/user-attachments/assets/472f2076-c63e-4ff1-a4e0-2c82a31310ad)

The path to zetta-scale computing, tracing the evolution of computing performance (measured in FLOPS—floating-point operations per second) from the gigascale era in 1984 to the projected zettascale by 2035.

**Key Performance Levels**

- **Gigascale (10⁹ FLOPS):** The starting point in 1984, marking the capability of early supercomputers.
- **Terascale (10¹² FLOPS):** Achieved around 1997, a significant milestone where systems like Jaguar (Cray XT5) delivered teraflop performance with power consumption of 7 MW.
- **Petascale (10¹⁵ FLOPS):** Achieved in 2008 with systems like Titan (Cray XK6) at 27 petaflops, consuming 9 MW. This milestone represents the era of petascale high-performance computing (HPC).
- **Exascale (10¹⁸ FLOPS):** Reached by systems like Frontier (Cray Shasta) in 2021, delivering 1.5 exaflops using 4 AMD GPUs and 1 AMD CPU, consuming 29 MW of power. Exascale computing enables highly detailed simulations and large-scale AI workloads.
- **Zettascale (10²¹ FLOPS):** Projected to be achieved by around 2035. At this scale, systems will handle unprecedented computational workloads, such as advanced climate modeling, AI, and large-scale simulations. Power consumption is estimated to range between 50-100 MW for a single zettascale machine.

**CMOS Evolution and Next-Gen Candidates**

![3](https://github.com/user-attachments/assets/8d99108c-6553-4cfe-92e4-a3357ceea2d1)

This diagram illustrates the evolving landscape of CMOS (Complementary Metal-Oxide-Semiconductor) technology and highlights emerging materials, structures, and processes being explored for next-generation semiconductor devices. These innovations aim to address the challenges of scaling CMOS technology down to the 1nm node and beyond.


- **Channel Material**
  - **Current Trends**: 
    - Silicon (Si) is the primary material used for the channel in traditional CMOS transistors, with **strained SiGe** (Silicon-Germanium) being used in some high-performance applications to enhance carrier mobility.
  
  - **Future Materials**: 
    - **2D materials** such as MoS₂ (Molybdenum Disulfide) are being explored due to their potential for better electrical characteristics at smaller scales.
    - **Germanium (Ge)** is gaining interest as it offers higher electron mobility, which could significantly boost transistor performance at small nodes.

- **Patterning**
  - **Current Techniques**: 
    - **Deep Ultraviolet (DUV)** lithography is the most commonly used technique for defining transistor features, with **ArF (Argon Fluoride)** and **KrF (Krypton Fluoride)** lasers operating at different wavelengths.
  
  - **Next-Gen**: 
    - **Extreme Ultraviolet (EUV)** lithography is expected to be a key technology for sub-7nm nodes. **High-NA (Numerical Aperture) EUV** will further improve the resolution for even smaller transistor nodes, pushing the boundaries of Moore's Law.

- **Gate Stack Material**
  - **Current Materials**: 
    - **High-K metal gates (HKMG)** are used in the gate stack of modern FETs to reduce gate leakage current and improve switching performance.
  
  - **Next-Gen Candidates**:
    - **NC-FET (Negative Capacitance FET)**: This is a promising transistor design that leverages ferroelectric materials to reduce power consumption by enabling lower voltage operation.
    - **TFET (Tunnel FET)**: TFETs use quantum tunneling to switch on and off, offering a significant reduction in power consumption compared to conventional FETs, especially for low-power applications.

- **Interconnection Material**
  - **Current Materials**: 
    - **Copper (Cu)** is the primary material used for interconnects due to its low resistivity, which helps in minimizing power loss and delays in transistor connections.
  
  - **Next-Gen Materials**:
    - **Ruthenium (Ru)** and **Compound metals** are being investigated for their potential to reduce resistance and improve performance in ultra-small transistors.
    - **Topological semi-metals** may offer unique properties, such as lower resistivity and increased performance at the atomic scale.


- **Device Structure**
  - **Current Architectures**: 
    - **FinFET** and **planar** transistors are used to maintain performance at smaller nodes. FinFETs, in particular, help improve control over short-channel effects by using a 3D structure.
  
  - **Next-Gen Architectures**:
    - **3DS-FET (3D Stacked FET)**: These are three-dimensional transistors where multiple layers of devices are stacked vertically, reducing footprint and improving performance.
    - **MBC-FET (Multi-Bridge Channel FET)**: This structure aims to enhance drive current by creating multiple channels within the same device.
    - **VFET (Vertical FET)**: VFETs utilize vertical channels to improve density and reduce power consumption.

- **Design Co-Optimization**
  - **DTCO (Design-Technology Co-Optimization)**: 
    - DTCO focuses on integrating new design techniques with advanced process technologies to maximize chip performance, often involving **backside interconnects (BSI)**, where interconnections are made at the back of the wafer for improved signal integrity and reduced latency.
  
  - **STCO (System-Technology Co-Optimization)**: 
    - This approach involves optimizing both the system architecture and the underlying technology. One example is the use of **chiplets**, which allow for modular, customized designs by integrating multiple smaller chips into one package, offering flexibility and reducing the complexity of scaling single-chip designs.

#### FinFETs

![4](https://github.com/user-attachments/assets/0d5c9ade-aa43-4750-a8a7-3b212cdba0c7)

This diagram illustrates the evolution of transistor technology from planar to more advanced architectures like FinFET and Gate-All-Around (GAA):

1. **Planar Transistor (Traditional)**:
   - Early transistor design with a flat channel and gate structure.
   - The gate controls the channel from one side only, leading to limited performance as scaling continues.

2. **FinFET (2011)**:
   - The channel is shaped like a vertical fin, allowing the gate to wrap around three sides of the channel.
   - Provides better control over the channel, reducing leakage and improving performance at smaller sizes.

3. **Gate-All-Around (GAA) Transistor (2025?)**:
   - The gate completely surrounds the channel, typically implemented using stacked nanosheets or nanowires.
   - Offers even better control over the channel compared to FinFET, allowing higher performance and efficiency with continued scaling.

Each step improves drive current capability and enhances control over the transistor's on/off states, critical for power efficiency and miniaturization in modern electronics.

**Why FinFETs and Gate-All-Around Transistors?**

![5](https://github.com/user-attachments/assets/878fda92-1897-4a1e-85ee-8649854466ef)

This diagram explains the advantages of FinFETs and Gate-All-Around (GAA) transistors compared to traditional planar structures:

 1. **Planar Transistors:**
   - **Challenges:**
     - Sub-channel leakage occurs where current leaks underneath the gate.
     - Results in reduced efficiency.
     - Increases power consumption.

2. **FinFET Transistors:**
   - The gate wraps around the channel (fin) on three sides, providing better control over the channel.
   - **Benefits:**
     - Reduces sub-threshold leakage.
     - Enhances drive current (\(I_{ON}\)).
     - Allows a smaller transistor area while maintaining high performance.

3. **Gate-All-Around (GAA) Transistors:**
   - The gate completely surrounds the channel, offering superior electrostatic control.
   - **Advantages:**
     - Improves short-channel performance by reducing drain capacitance and enhancing gate capacitance.
     - Improves scaling efficiency as indicated by the formula \(S \propto (1 + C_d / C_{ox})\).
     - Provides reduced sub-threshold slope and better performance at smaller scales.

4. **Graph Comparison:**
   - Illustrates the performance advantages of FinFETs and GAA over planar transistors.
   - Shows better efficiency and reduced sub-threshold slope as dimensions shrink.

![6](https://github.com/user-attachments/assets/36c9d9eb-7ff0-4f2c-9eb1-0d52cbea8f53)

**Reduced Leakage:** Tri-Gate transistors exhibit significantly lower leakage current compared to planar transistors at the same gate voltage. Lower leakage results in both reduced off-current at the same on-current and lower power dissipation.

**Higher Drive Current:** Tri-Gate transistors provide higher drive current compared to planar transistors at the same off-current. This results in improved circuit performance and greater efficiency in modern electronic applications.

#### FEOL Innovations:

FEOL refers to the initial stages of semiconductor manufacturing where the active devices (e.g., transistors) are built on the silicon wafer. It involves creating components such as transistors, capacitors, and isolation structures before metal interconnects are added. FEOL Innovations help drive Moore's Law forward by enabling smaller, more efficient, and more powerful transistors.

**CMOS Technology Inflection Points**

![7](https://github.com/user-attachments/assets/300b2921-e3c6-4c53-a281-94ea438d837d)

1. **Dennard Scaling**:
   - States that power density remains constant as transistors shrink.
   - Initially allowed voltage scaling with smaller gate lengths, shown in the bottom-left graph.

2. **Technology Nodes and Innovations**:
   - **~1 µm ("End of Scaling")**: Start of CMOS miniaturization.
   - **180 nm (Voltage Scaling)**: Start of drive voltage reduction.
   - **130 nm (Cu BEOL)**: Introduction of copper interconnects for better conductivity.
   - **90 nm (Uniaxial Strained Si NMOS)**: Strained silicon enhances electron mobility.
   - **65 nm (eSiGe CVD ULK)**: Embedded SiGe improves PMOS performance.
   - **45 nm (HK-first MG-last)**: High-k dielectrics and metal gates reduce leakage and improve gate control.
   - **32 nm (HKMG with Raised S/D NMOS)**: Advanced HKMG implementation and raised source/drain regions.

3. **SEM Images**

- **Left Image:** Shows the cross-sectional view of a transistor structure with High-k materials and embedded SiGe (Silicon-Germanium).It has high-k dielectric and metal gates are used to improve performance. SiGe regions enhance PMOS performance by applying strain to the silicon channel.

- **Right Image:** Demonstrates the raised source/drain (S/D) regions and gate channel in PMOS transistors at smaller nodes.

4. **Drive Voltage Scaling Graph (Bottom-left):** The graph shows the relationship between gate length (x-axis, logarithmic scale) and drive voltage (y-axis, logarithmic scale). The Ideal scaling behavior indicates that the voltage decreases linearly with shrinking gate length. Red and green markers show practical trends for low-power and high-performance devices, which deviate from ideal scaling due to challenges like leakage currents and increased power density.

![8](https://github.com/user-attachments/assets/fd3702fa-7e20-45c1-bec6-23632b94bcbd)

**Key Technology Nodes and Innovations**

- **22 nm**:
  - Introduction of **FinFET (Tri-Gate)** transistors, which reduce leakage and improve gate control.
  - Use of **self-aligned contacts (SAC)** and **copper interconnects (Co+Cu BEOL)**.

- **14 nm**:
  - Transition to **unidirectional metal routing** for better density.
  - Implementation of **SADP (Self-Aligned Double Patterning)** and **SDB (Single Diffusion Break)** for precise layout.

- **10 nm**:
  - Adoption of **advanced patterning techniques** such as:
    - **SA-SDB** (Self-Aligned SDB)
    - **LELELE** (Litho-Etch-Litho-Etch-Litho-Etch)
    - **SAQP (Self-Aligned Quadruple Patterning)** for tighter geometries.

- **7 nm**:
  - Introduction of **Extreme Ultraviolet Lithography (EUV)** to simplify the patterning process and reduce overlay errors.

- **5 nm**:
  - Integration of **SiGe (Silicon-Germanium) channels** for PMOS to enhance hole mobility.
  - Use of **EUV SA-LELE** (Self-Aligned Litho-Etch-Litho-Etch).

- **3 nm / 2 nm / 1.4 nm**:
  - Transition to **Gate-All-Around (GAA)** nanosheet transistors for improved electrostatic control.
  - GAA stacks nanosheets or nanowires horizontally to maximize current drive.

- **Sub-1 nm**:
  - Development of **CFET (Complementary FET)**, which vertically stacks NMOS over PMOS to save area.
  - Use of **2D materials**, such as **MoS₂**, for atomic-scale channel thickness in **2D FETs**.

![9](https://github.com/user-attachments/assets/0b0bb655-4cd9-4d98-b84e-df03f551a455)

The image illustrates how Samsung has scaled down the size of transistors in their successive generations of nodes (10nm, 8nm, 7nm, and 5nm) using a technique called Fin Depopulation. In FinFET transistors, the "fin" is the vertical channel that carries the current. Fin Depopulation involves reducing the number of fins per transistor while keeping the fin width constant. This allows for smaller transistors without compromising performance.

- 10nm (HD): The transistor has a fin height of 420nm and uses 10 fins.
- 8nm (UHD): The fin height is reduced to 378nm, and the number of fins is decreased to 9.
- 7nm (HD): The fin height remains at 27nm, but the number of fins is further reduced to 8.
- 5nm (UHD): The fin height is maintained at 27nm, and the number of fins is decreased to 7.

![10](https://github.com/user-attachments/assets/be1270a4-e6ea-441b-a2ae-b4cd2ee0f1e1)

- **Double Diffusion Break (DDB)**: Double Diffusion Break (DDB) involves creating a gap between the source and drain regions of a transistor. This gap is filled with an insulating material, which reduces the effective width of the transistor. By doing so, DDB enables the design of smaller cell sizes, allowing for higher transistor density and improved scalability. A cross-sectional view of a transistor with DDB highlights the insulating gap between the source and drain regions.

- **Single Diffusion Break (SDB)**: Single Diffusion Break (SDB) is similar to DDB but less aggressive. It involves introducing a gap on only one side of the transistor. This approach provides a balanced trade-off between size reduction and maintaining transistor performance. A cross-section of a transistor with SDB highlights the gap on one side, showcasing its simplicity compared to DDB.

- **Contact Over Field Gate (COFG)**: Contact Over Field Gate (COFG) places the gate contact directly over the field oxide region of a transistor. This design reduces lateral spacing between adjacent transistors, enabling smaller cell sizes without significant performance loss. A cross-sectional representation of a transistor with COFG illustrates the positioning of the gate contact over the field oxide.

- **Contact Over Active Gate (COAG)**: Contact Over Active Gate (COAG) is a more aggressive technique than COFG. Here, the gate contact is placed directly over the active gate region of the transistor. This approach enables even smaller cell sizes and higher transistor density, which are critical for advanced semiconductor nodes. A cross-sectional image of a transistor with COAG highlights the gate contact placement over the active gate.

- **Back-Side Power Delivery Network (BS-PDN)**: The Back-Side Power Delivery Network (BS-PDN) is an innovative approach where power supply rails are routed on the backside of the chip. This method reduces the height of the standard cell, creating space for more transistors and improving overall transistor density. Additionally, it enhances power delivery efficiency and reduces resistance, which is crucial for high-performance applications. A schematic of a standard cell with BS-PDN illustrates the positioning of power rails on the backside of the chip.

![11](https://github.com/user-attachments/assets/b04bbb8f-9066-4a22-ba73-9a2ed89e058e)

- **Planar Technology**: In early planar technology nodes (100nm and above), the Vt variability is significantly high, around 130mV. This is due to various factors like process variations, temperature fluctuations, and line-edge roughness.

- **FinFET Technology**: With the advent of FinFET technology (around 22nm), the Vt variability reduces significantly to around 14mV. This improvement is attributed to the better control over the channel length and width in FinFETs compared to planar transistors.

- **NW Technology (Nanowire)**: In the latest nanowire technology (14nm and below), the Vt variability is even lower, around 7mV. This further reduction is due to the precise control over the nanowire dimensions and the reduced impact of process variations.

![12](https://github.com/user-attachments/assets/b160cd89-d147-4552-b633-da94d32a351e)

**Planar MOSFETs**  
Planar MOSFETs, the traditional architecture, have a simple structure where the gate sits above the channel. In this design, the contact width (\(W_C\)) and gate width (\(W_G\)) are nearly equal, resulting in a ratio of \(W_C / W_G \approx 1\). This leads to a low parasitic resistance, with \(R_{EXT}\) being much smaller than \(R_{ch}\) (\(R_{EXT} / R_{ch} < 1\)). As a result, planar MOSFETs suffer minimal performance degradation due to parasitic resistance.

**FinFETs**  
FinFETs, a 3D transistor design, introduce vertical fins with the gate wrapping around them for improved control. However, the effective contact width decreases relative to the gate width, leading to \(W_C / W_G \approx 1/3\). Consequently, the parasitic resistance becomes comparable to the channel resistance (\(R_{EXT} / R_{ch} \approx 1\)), which begins to impact the performance of the device as it scales.

**Gate-All-Around (GAA) FETs**  
Gate-All-Around (GAA) FETs, which use nanosheets or nanowires, offer even better electrostatic control by fully surrounding the channel with the gate. However, the contact width further decreases compared to the gate width, resulting in \(W_C / W_G \approx 1/6\). This causes a significant increase in parasitic resistance, with \(R_{EXT}\) being approximately three times the channel resistance (\(R_{EXT} / R_{ch} \approx 3\)). While GAA FETs improve transistor density, the higher parasitic resistance becomes a challenge for maintaining performance.

**Complementary FETs (CFETs)**  
Complementary FETs (CFETs) take transistor stacking to the next level by vertically integrating NMOS and PMOS transistors. This approach maximizes space efficiency in advanced nodes but inherits the high parasitic resistance of GAA FETs. With \(W_C / W_G\) remaining small, the \(R_{EXT} / R_{ch}\) ratio is around 3, posing similar challenges to those faced by GAA FETs.

**Explanation of Parasitic Resistance**

![13](https://github.com/user-attachments/assets/b96165d9-b46c-490c-aaf0-f471ef906fbe)

The image highlights the breakdown of parasitic resistance (\(R_{EXT}\)) and approaches for reducing it in transistors. Here is a detailed explanation:

1. **Components of Parasitic Resistance (\(R_{EXT}\))**
The leftmost diagram illustrates the various contributors to \(R_{EXT}\) in a transistor:
- **\(R_{CA-BEOL}\)**: Resistance from the contact in the Back-End-Of-Line (BEOL).
- **\(R_{CA}\)**: Resistance at the contact area.
- **\(R_{CA-TS}\)**: Resistance at the contact to the transition structure.
- **\(R_{TS}\)**: Resistance in the transition structure.
- **\(R_{MOL}\)**: Middle-Of-Line resistance (includes lateral and vertical contributions).
- **\(R_C\)**: Contact resistance at the metal-semiconductor interface.
- **\(R_{EPI}\)**: Epitaxial layer resistance (contributes to current spreading).
- **\(R_{FEOL}\)**: Front-End-Of-Line resistance from the source/drain extensions.

2. **Initial vs. Improved \(R_{EXT}\) Breakdown**
The two pie charts in the center show the contributions of different resistance components for NFETs and PFETs before and after improvements:
- **NFET**:
  - **Initial**: Majority of \(R_{EXT}\) comes from \(R_C\) (63%) and \(R_{CA-BEOL}\) (31%).
  - **Improved**: Significant reduction in \(R_C\) (48%) and \(R_{CA-BEOL}\) (12%).
- **PFET**:
  - **Initial**: \(R_{FEOL}\) (50%) and \(R_C\) (45%) dominate.
  - **Improved**: Major reduction in \(R_{FEOL}\) (78%) and \(R_C\) (16%).

3. **Improving Ohmic/Tunneling Contacts**
The right section discusses methods to improve the metal-semiconductor interface:
- **Key Objectives**:
  - **Low Schottky Barrier Height (SBH)** (\(\phi_b\)): Reduces the energy barrier for carrier injection, enabling better contact conductivity.
  - **High Doping Concentration (\(N_d\))**: Increases carrier density at the interface, reducing contact resistance.
- **Equation for Specific Contact Resistivity (\(\rho_c\))**:
  \[
  \rho_c \propto \exp\left(\frac{2\phi_b}{\hbar} \sqrt{\frac{\epsilon_s m_x}{N_d}}\right)
  \]
  This equation shows how lowering \(\phi_b\) and increasing \(N_d\) can reduce \(\rho_c\).

- **Metal-Semiconductor Energy Diagram**:
  - The energy diagram shows how a reduction in \(\phi_b\) (Schottky Barrier Height) facilitates easier carrier injection from the metal to the semiconductor.

![14](https://github.com/user-attachments/assets/e7f9837f-9a35-4d8b-87f6-f7eda9aaa769)

The bar chart on the left shows how the composition of \(C_{eff}\) evolves from 22nm to 7nm technology nodes:

- At **22nm**, the dominant contributor to \(C_{eff}\) is \(C_{fr}\) (56%), while parasitic capacitances \(C_{pc-ca}\) (25%) and \(C_{g}\) (19%) contribute less.
- At **14nm** and **10nm**, parasitic capacitances (\(C_{pc-ca}\) and \(C_{fr}\)) start dominating, with \(C_{fr}\) decreasing to 38% and 25%, respectively, while \(C_{pc-ca}\) increases.
- At **7nm**, \(C_{g}\) becomes the most significant contributor (45%), with \(C_{pc-ca}\) and \(C_{fr}\) still present but reduced. This highlights the increasing impact of parasitic capacitance as node sizes shrink.

Plot Descriptions:

- The first scatter plot shows a reduction in normalized delay for a ring oscillator when using SiBCN spacers instead of SiN spacers, indicating improved performance.
- The second scatter plot demonstrates an 8% reduction in \(C_{eff}\) with SiBCN spacers, which corresponds to the delay improvement observed in the first plot.
- The rightmost figure shows the evolution of spacer materials from SiOCN to SiCO. This material transition aims to significantly reduce the gate-contact capacitance across nodes. At 14nm and beyond, low-\(k\) spacers improve performance by decoupling parasitic effects and maintaining capacitance at manageable levels.
- The bottom middle image shows a cross-sectional TEM view of a transistor with air spacers around the gate: i) Air, with its extremely low dielectric constant (\(k \approx 1\)), significantly reduces parasitic capacitance. The adjacent plot quantifies this improvement, showing a 15% reduction in \(C_{eff}\) when using air spacers compared to solid spacers.

![15](https://github.com/user-attachments/assets/b4930d47-6e63-4a08-8bbf-b9b29b353ec8)

**Key Properties of 2D Layered Materials (Compared to Silicon):**

- **Uniform Atomic Scale Thickness:** A single layer of MoS₂ is approximately 0.65 nm thick, offering an ideal thickness for scaling compared to silicon. 
- **Higher Effective Mass (\( m^* \)):** MoS₂ has an effective mass of about 0.55 times the mass of a free electron (\( m_0 \)), whereas silicon’s effective mass is around 0.22 \( m_0 \).
- **Bandgap:** Additionally, 2D materials like MoS₂ possess favorable bandgaps for electronic devices, with a monolayer bandgap of ~1.85 eV, which reduces to ~1.5 eV for a bilayer.

![image](https://github.com/user-attachments/assets/9402c63b-d052-4163-86ec-695c617b5857)

1. **Transistor Scaling**:  
   - To achieve smaller gate lengths, devices must address various physical and material constraints to ensure reliable operation.

2. **Challenges for Scaling**:
   - **Direct Source-to-Drain Tunneling**: As the gate length decreases, electrons can tunnel directly from the source to the drain, bypassing the gate control. To mitigate this, materials with a high effective mass (\( m^* \)) are needed.
   - **Surface Roughness and Thickness Variations**: Variability at atomic scales can cause performance issues. Uniform, atomically thin materials are essential for minimizing such variations.
   - **Capacitance Ratios (\( C_D \) and \( C_{ox} \))**: The capacitance of the depletion region (\( C_D \)) must remain low relative to the gate oxide capacitance (\( C_{ox} \)) to improve gate control. Materials with a low in-plane dielectric constant (\( \epsilon \)) are necessary for this.

3. **Diagrams**:  
   - The left shows the transistor structure with key components like the source, drain, gate, oxide, and silicon substrate.
   - The right illustrates two scenarios:  
     a. *Thermionic Emission*: Electrons cross the energy barrier as intended.  
     b. *Direct Tunneling*: At extremely small gate lengths, electrons tunnel directly from source to drain, leading to leakage.

4. **Key Takeaway**:  
   - New channel materials, such as 2D materials, are required to overcome these challenges while maintaining high performance and scalability.

![16](https://github.com/user-attachments/assets/ad25a60f-ac5a-48d3-810f-a06fb7e5ab7c)

Concept of Direct Source-to-Drain Tunneling: As the gate length (\(L_G\)) in MOSFETs decreases, direct tunneling of electrons from the source to the drain becomes significant, leading to increased leakage currents. This leakage is influenced by material properties, such as the effective mass (\(m^*\)) and the bandgap (\(E_G\)).

A higher \(m^*\) in MoS₂ suppresses tunneling leakage compared to silicon. The graph shows the leakage current (\(I_{SD, \text{leak}}\)) as a function of gate length (\(L_G\)) for various channel thicknesses (\(T_{CH}\)). MoS₂ exhibits lower leakage at smaller gate lengths compared to silicon, achieving up to 100x reduction in leakage due to its higher \(m^*\), larger \(E_G\), and lower dielectric constant (\(\epsilon\)).

The superior performance of MoS₂ in minimizing leakage currents results in significant energy savings, making it a promising material for future transistor scaling.

![17](https://github.com/user-attachments/assets/d8678f0c-a321-45f5-83a5-02dd94820747)

The MoS₂ transistor with a 1 nm gate length represents a breakthrough in miniaturization, featuring a thin MoS₂ channel for its excellent electronic properties. A single-walled carbon nanotube (SWCNT) serves as the ultra-small gate electrode, while Zirconium Dioxide (ZrO₂) acts as a high-k dielectric, reducing leakage and ensuring precise control. Built on a SiO₂ substrate with an n⁺ silicon back gate, the transistor uses the CNT gate to deplete a small region of the MoS₂ channel, enabling efficient modulation. This innovative design showcases the potential of 2D materials and nanoscale gates in advancing transistor technology.

![18](https://github.com/user-attachments/assets/8b68aa04-da7c-4fed-94d3-0d89d0007e46)

The slide illustrates the structure and fabrication of an All-2D MOSFET (Metal-Oxide-Semiconductor Field-Effect Transistor), where all key components, including the channel, gate, and contacts, are made using two-dimensional materials. This device leverages the exceptional properties of 2D materials to improve performance and scalability. Below is a breakdown of the key components and the fabrication process:

**Graphene Contacts (S, D, G):** Graphene is used as the source, drain, and gate electrodes. Its high conductivity and 2D nature make it ideal for ensuring low-resistance electrical contacts.
MoS₂ Channel:

**Molybdenum Disulfide (MoS₂)** serves as the semiconductor channel. MoS₂ is widely used in 2D MOSFETs due to its excellent on/off current ratio and atomic-scale thickness.
h-BN Dielectric:

**Hexagonal Boron Nitride (h-BN)** acts as the insulating dielectric layer. It is a 2D material with excellent insulating properties and high thermal stability, making it suitable for separating the graphene gate from the MoS₂ channel.
Si/SiO₂ Substrate:

The device is fabricated on a silicon dioxide (SiO₂) layer on top of a silicon substrate, which provides mechanical support and a global back gate.
Fabrication Process:

- A layer of graphene is deposited on the SiO₂ substrate, which will later serve as the gate electrode.
- Graphene is patterned to define the source and drain regions, leaving gaps for the channel.
- A monolayer of MoS₂ is transferred onto the patterned graphene, forming the channel region.
- An h-BN layer is added on top of the MoS₂ as the gate dielectric.
- A top layer of graphene is deposited and aligned as the gate electrode.
- The completed device is contacted using metallic electrodes (Ni/Au) for testing.

![19](https://github.com/user-attachments/assets/dbd316a3-5cc3-44c1-a509-72f79aba57d7)

The All-2D MOSFET exhibits excellent electrical performance:

- **Transfer Characteristics (I\(_D\) vs V\(_G\))**: Achieves a high on/off current ratio (>10⁵), demonstrating strong gate control for effective switching.
- **Output Characteristics (I\(_D\) vs V\(_{DS}\))**: Smooth current modulation with increasing V\(_G\) and V\(_{DS}\), indicating good output performance.
- **Mobility**: Field-effect mobility remains constant with increasing gate electric field, showing minimal scattering and high-quality interfaces in the 2D materials stack.

These results highlight the potential of 2D materials like MoS₂, graphene, and h-BN for scalable, high-performance transistor applications.

![20](https://github.com/user-attachments/assets/a4bbe723-a1a3-4bf7-8b6a-25c01fd96a0a)

The diagram on the top left shows a non-planar transistor with key components:

- Gate: Controls the flow of current through the channel.
- Channel: Region where current flows between the source (S) and drain (D).
- Body: Underlying region connected to the substrate.
- STI (Shallow Trench Isolation): Insulates neighboring devices.

The biggest challenge is to form a single-crystalline semiconductors on a non-planar surface is difficult using conventional semiconductor fabrication techniques.

![21](https://github.com/user-attachments/assets/f3929418-2f24-444c-9ecf-fa1000045a56)

Single-Layer CMOS (a): This is the traditional CMOS design where NMOS and PMOS transistors are fabricated on a single silicon layer. Each transistor operates in the same planar layer, with devices connected laterally.

Monolithic 3D CMOS (b): In this design, NMOS and PMOS transistors are stacked in two separate layers, enabling higher density. The P-Channel (PMOS) is placed on top of the N-Channel (NMOS), separated by an oxide layer. This approach reduces the footprint and allows better performance due to shorter interconnects.

Single-Layer CMOS Logic (c): Shows logic gates (inverter, 2-input NAND, and 2-input NOR) built using traditional single-layer CMOS. Transistors are laid out horizontally, with interconnections taking more space.

Monolithic 3D CMOS Logic (d): Logic gates are constructed with two transistor layers (Layer 1 and Layer 2), reducing the area required for interconnections. Vertical integration improves performance and reduces delay by shortening signal paths.

![22](https://github.com/user-attachments/assets/a6583598-da2e-490f-8a6c-f4fce46b89a0)


**Dual Damascene Cu**, used for the 7nm technology node with a 36nm pitch, combines vias (vertical connections) and lines (horizontal connections) in a single patterning step. It relies on copper (Cu) for interconnections; however, as dimensions shrink, challenges such as gap filling and maintaining reliability become increasingly significant.

**Single Damascene Cu**, used for the 5nm technology node with a 28nm pitch, involves splitting the creation of vias (vertical connections) and lines (horizontal connections) into separate steps. This approach addresses the challenges of smaller dimensions, with a primary focus on reducing resistance (R) in both lines and vias to maintain optimal performance.

**Barrier and via metal optimization**, introduced at the 3nm technology node with a 20-24nm pitch, focuses on reducing the thickness of barrier layers (insulating layers) to minimize resistance while maintaining robust and reliable via connections. This optimization is essential to meet the performance and scaling demands of advanced nodes.

At sub-18nm pitch, **subtractive RIE** and alternative metals like ruthenium (Ru) are introduced to address the reliability and scaling challenges faced by traditional copper interconnects. Subtractive Reactive Ion Etching (RIE) enables more precise patterning of interconnects, while the use of Ru provides improved performance and durability at such advanced dimensions.

For future nodes with pitches below 15nm, **post-Damascene interconnects** featuring tall, barrier-less designs are envisioned. This approach enhances electromigration (EM) reliability, ensuring durable and robust connections despite the continued shrinking of dimensions, thereby addressing key challenges in advanced interconnect scaling.

![23](https://github.com/user-attachments/assets/a52f26b6-28ce-4691-85a0-dee1cc018112)

The image shows how a selective barrier, typically tantalum nitride (TaN), can improve copper interconnects in semiconductor manufacturing. This barrier reduces resistance, enhances reliability by preventing copper ion migration, and aids in controlling copper thickness. The process involves cleaning the copper surface, depositing TaN using atomic layer deposition (ALD), and removing sacrificial layers. This technique is crucial for advancing semiconductor technology and ensuring reliable, high-performance devices.

![24](https://github.com/user-attachments/assets/8a6939cb-8e2a-4c6b-be0c-3598a6abda7a)

**Back-Side Power Delivery Network (BS-PDN)**

In advanced semiconductor manufacturing, efficient power delivery is critical to the performance and reliability of integrated circuits. Traditional Front-Side Power Delivery Networks (FS-PDNs) often suffer from high IR-drop, which can limit device performance and reliability. To address this challenge, Back-Side Power Delivery Networks (BS-PDNs) have emerged as a promising solution.

BS-PDNs involve routing power supply rails on the backside of the chip, enabling shorter and wider power lines. This configuration significantly reduces IR-drop, leading to improved power delivery efficiency. As a result, BS-PDNs offer several advantages:

- Reduced IR-drop: Lower voltage drops across the power network, leading to improved performance and reliability.
- Decreased standard cell area: More efficient power delivery allows for smaller standard cell sizes.
- Improved performance: Lower IR-drop leads to faster switching speeds and reduced power dissipation.

By adopting BS-PDNs, semiconductor manufacturers can develop high-performance and energy-efficient integrated circuits that meet the demands of modern electronics.


**Installing and setting up ORFS**

```
git clone --recursive https://github.com/The-OpenROAD-Project/OpenROAD-flow-scripts .
cd OpenRoad
sudo ./setup.sh
./build_openroad.sh --local
```
**Verify Installation**

```
source ./env.sh
yosys -help
openroad -help
cd flow
make
```

![Screenshot from 2024-11-26 02-01-48](https://github.com/user-attachments/assets/f21682b8-3780-48c4-9785-dbcb55ea39f4)

![Screenshot from 2024-11-26 02-02-00](https://github.com/user-attachments/assets/5345ef36-6661-4c8f-851a-843e753bfa0b)

```
make gui_final
```

![Screenshot from 2024-11-26 02-02-22](https://github.com/user-attachments/assets/985294e8-8851-41ea-9ab4-e1602f9f315f)

![Screenshot from 2024-11-26 02-02-32](https://github.com/user-attachments/assets/3b17ee81-996b-4ef0-8a00-ec5815c3f0ed)

**ORFS Directory Structure and File formats**


``` 
├── OpenROAD-flow-scripts             
│   ├── docker           -> It has Docker based installation, run scripts and all saved here
│   ├── docs             -> Documentation for OpenROAD or its flow scripts.  
│   ├── flow             -> Files related to run RTL to GDS flow  
|   ├── jenkins          -> It contains the regression test designed for each build update
│   ├── tools            -> It contains all the required tools to run RTL to GDS flow
│   ├── etc              -> Has the dependency installer script and other things
│   ├── setup_env.sh     -> Its the source file to source all our OpenROAD rules to run the RTL to GDS flow
```

Now, go to flow directory

``` 
├── flow           
│   ├── design           -> It has built-in examples from RTL to GDS flow across different technology nodes
│   ├── makefile         -> The automated flow runs through makefile setup
│   ├── platform         -> It has different technology note libraries, lef files, GDS etc 
|   ├── tutorials        
│   ├── util            
│   ├── scripts             
```

Automated RTL2GDS Flow for VSDBabySoC:

Initial Steps:

- We need to create a directory `vsdbabysoc` inside `OpenROAD-flow-scripts/flow/designs/sky130hd`
- Now copy the folders `gds`, `include`, `lef` and `lib` from the VSDBabySoC folder in your system into this directory.
  - The `gds` folder would contain the files `avsddac.gds` and `avsdpll.gds`
  - The `include` folder would contain the files `sandpiper.vh`, `sandpiper_gen.vh`, `sp_default.vh` and `sp_verilog.vh`
  - The `gds` folder would contain the files `avsddac.lef` and `avsdpll.lef`
  - The `lib` folder would contain the files `avsddac.lib` and `avsdpll.lib`
- Now copy the constraints file(`vsdbabysoc_synthesis.sdc`) from the VSDBabySoC folder in your system into this directory.
- Now copy the files(`macro.cfg` and `pin_order.cfg`) from the VSDBabySoC folder in your system into this directory.
- Now, create a macro.cfg file whose contents are shown below:

```
```

Now go to terminal and run the following commands:

```
cd OpenRoad
source env.sh
cd flow
```

![Screenshot from 2024-11-26 01-40-17](https://github.com/user-attachments/assets/8fb461be-e61a-4f8d-846f-41d75e7db0f0)


Commands for **synthesis**:

```
make DESIGN_CONFIG=./designs/sky130hd/vsdbabysoc/config.mk synth
```

![Screenshot from 2024-11-26 02-18-25](https://github.com/user-attachments/assets/db6d2dc4-8a23-4279-ba34-a85600fa1f62)

![Screenshot from 2024-11-26 02-18-36](https://github.com/user-attachments/assets/b9d5a46f-e859-4c85-a431-d7bfa6a0bc8f)

Synthesis netlist:

![Screenshot from 2024-11-26 02-21-59](https://github.com/user-attachments/assets/aee11109-c3bc-4000-9d48-01596c621a5e)

Synthesis log:

![Screenshot from 2024-11-26 02-22-41](https://github.com/user-attachments/assets/badb581a-da33-463a-b099-b7cc6a4cbd30)

Synthesis Check:

![Screenshot from 2024-11-26 02-23-53](https://github.com/user-attachments/assets/ec00f507-6d84-40b8-bbd8-99eda69e8ded)

Synthesis Stats:

![Screenshot from 2024-11-26 02-25-33](https://github.com/user-attachments/assets/db673013-8f4f-49c5-bf4a-53bb4dbdaa07)

![Screenshot from 2024-11-26 02-26-54](https://github.com/user-attachments/assets/fba718cc-01c0-483e-9d2c-c562ebc00ddb)

![Screenshot from 2024-11-26 02-27-08](https://github.com/user-attachments/assets/bf643d1d-22aa-4041-9909-671fb8ed4f63)

Commands for **floorplan**:

```
make DESIGN_CONFIG=./designs/sky130hd/vsdbabysoc/config.mk floorplan
```

![Screenshot from 2024-11-26 02-27-30](https://github.com/user-attachments/assets/67f5ceff-2486-46da-8bd5-f2e77c34796a)

![Screenshot from 2024-11-26 02-27-37](https://github.com/user-attachments/assets/f1d89c78-9d3e-45cc-84b3-990958082716)

![Screenshot from 2024-11-26 07-16-50](https://github.com/user-attachments/assets/4f8d835e-c7da-434b-b2c0-faf5c9d56b7b)

![Screenshot from 2024-11-26 07-18-26](https://github.com/user-attachments/assets/02e361c1-ebb8-4451-a956-feebe1930e1a)

```
make gui_floorplan
```

![Screenshot from 2024-11-26 02-27-53](https://github.com/user-attachments/assets/1047f3b8-9863-4e5c-a6df-f62f30001f06)

![Screenshot from 2024-11-26 02-28-01](https://github.com/user-attachments/assets/df2521c7-8e2d-4702-92bc-e5cbaf99c01e)

##### HeatMap:

![Screenshot from 2024-11-26 07-08-55](https://github.com/user-attachments/assets/8dbdaf11-be14-443b-8549-8cc93f9c6950)

```
make DESIGN_CONFIG=./designs/sky130hd/vsdbabysoc/config.mk gui_floorplan
```

![Screenshot from 2024-11-26 02-28-49](https://github.com/user-attachments/assets/44570267-400d-4333-ba9b-567dd79ece33)

![Screenshot from 2024-11-26 01-49-43](https://github.com/user-attachments/assets/a782e595-2a4f-4bac-8579-96874acee2ba)

![Screenshot from 2024-11-26 05-37-45](https://github.com/user-attachments/assets/56cd8fbb-2d2f-4ccd-994b-a37cfeacef94)

![Screenshot from 2024-11-26 05-38-37](https://github.com/user-attachments/assets/03a75b26-7b95-430b-b0af-3a594b63ca15)


```
make DESIGN_CONFIG=./designs/sky130hd/vsdbabysoc/config.mk place
```

![Screenshot from 2024-11-26 02-30-06](https://github.com/user-attachments/assets/2ea129fd-ceef-4bce-85b7-1284bd59fb83)

![Screenshot from 2024-11-26 02-30-35](https://github.com/user-attachments/assets/c858c087-85c1-47d4-865f-34d09f1a8532)

![Screenshot from 2024-11-26 07-19-03](https://github.com/user-attachments/assets/590df9ef-9753-4649-8d84-6dbc46315e7b)

![Screenshot from 2024-11-26 07-19-13](https://github.com/user-attachments/assets/fa1d687c-54d1-43a3-a479-287c8e6bb0f8)

```
make gui_place
```

![Screenshot from 2024-11-26 02-32-56](https://github.com/user-attachments/assets/b4600924-3c71-48fb-9b46-fcbe7399fae8)

![Screenshot from 2024-11-26 02-33-21](https://github.com/user-attachments/assets/c5dd63ea-258b-445d-8348-2bafa320605c)

![Screenshot from 2024-11-26 02-35-37](https://github.com/user-attachments/assets/a965c983-a745-4851-82d7-c42b97943240)

![Screenshot from 2024-11-26 02-36-40](https://github.com/user-attachments/assets/7e272205-b4ef-41fc-a06f-666a1391ff08)

![Screenshot from 2024-11-26 05-40-12](https://github.com/user-attachments/assets/9ec68fcd-8130-4499-8661-47f45b816752)

![Screenshot from 2024-11-26 05-40-57](https://github.com/user-attachments/assets/33d456d9-55c8-4dca-a879-40fcf9654f63)

#### Heat Map:

![Screenshot from 2024-11-26 07-14-10](https://github.com/user-attachments/assets/0aa1d1a4-1104-4677-87a0-6136f2495499)


```
make DESIGN_CONFIG=./designs/sky130hd/vsdbabysoc/config.mk cts
```

![Screenshot from 2024-11-26 02-37-13](https://github.com/user-attachments/assets/48a81224-e72e-412d-9579-c5bd94cf8779)

![Screenshot from 2024-11-26 07-20-23](https://github.com/user-attachments/assets/6047209c-1f3f-4e47-af76-61271c2510be)

![Screenshot from 2024-11-26 07-20-27](https://github.com/user-attachments/assets/11872c3e-6cf6-4eb4-84fb-4ce253f9b84a)

```
make gui_cts
```

![Screenshot from 2024-11-26 02-37-28](https://github.com/user-attachments/assets/95f0918f-300c-4896-a137-9f87a753b0bd)

![Screenshot from 2024-11-26 02-40-02](https://github.com/user-attachments/assets/2dc03997-9e68-4cdd-aa7c-c775770fbf22)

![Screenshot from 2024-11-26 05-41-49](https://github.com/user-attachments/assets/e183a67e-08ec-446c-98b4-5d5e1676b2e5)

![Screenshot from 2024-11-26 05-42-06](https://github.com/user-attachments/assets/754d495a-fd7c-43db-979f-7483c168e0d4)

#### Heat Map:

![Screenshot from 2024-11-26 07-16-50](https://github.com/user-attachments/assets/18c18d7a-e153-4d58-bcf8-8ad6f5a377e3)


```
make DESIGN_CONFIG=./designs/sky130hd/vsdbabysoc/config.mk route
```

![Screenshot from 2024-11-26 02-44-40](https://github.com/user-attachments/assets/80704f1e-ecd3-4b7b-810c-baa856e5701e)

```
make gui_route
```

![Screenshot from 2024-11-26 02-46-06](https://github.com/user-attachments/assets/aded152e-57d7-426b-b559-b47ed9b4513c)

```
make gui_final
```

![Screenshot from 2024-11-26 02-48-04](https://github.com/user-attachments/assets/2ba9546f-c20b-44dc-92d0-a982def0c6bd)

![Screenshot from 2024-11-26 02-49-53](https://github.com/user-attachments/assets/d926e1e5-4021-494c-a843-00699f1663fc)

![Screenshot from 2024-11-26 02-46-31](https://github.com/user-attachments/assets/74138e56-66e8-4e8f-b5d2-144b3d066244)

![Screenshot from 2024-11-26 02-50-09](https://github.com/user-attachments/assets/43866203-4dad-42fd-b3b9-7af23ce23b8a)

![Screenshot from 2024-11-26 02-51-03](https://github.com/user-attachments/assets/24d91d81-e370-470b-865e-e80dc181871d)

![Screenshot from 2024-11-26 02-51-34](https://github.com/user-attachments/assets/b38f8751-bf80-4785-9e44-5fe99c7b90eb)

```
 klayout -e -nn ./platforms/nangate45/FreePDK45.lyt -l ./platforms/nangate45/FreePDK45.lyp ./results/nangate45/gcd/base/6_final.gds
```

![Screenshot from 2024-11-26 02-52-27](https://github.com/user-attachments/assets/cef992d0-f992-45a2-9ba0-17ec66843cb5)

![Screenshot from 2024-11-26 02-52-36](https://github.com/user-attachments/assets/d63412d9-a6ef-41cc-afae-748b96716d2a)

 </details>

---

*Prepared by:* Eshwar Allampally 
*Student ID:* MT2024504  
*Course:* ASIC Design  
*Instructor:* Prof. Kunal gosh
