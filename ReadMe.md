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

---

*Prepared by:* [Eshwar Allampally]  
*Student ID:* [MT2024504]  
*Course:* [ASIC Design]  
*Instructor:* [Prof. Kunal gosh]
