# ASIC Design Lab Report

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

---
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

  ---
## Lab Session 3: [22/07/2024]

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

---

*Prepared by:* [Eshwar Allampally]  
*Student ID:* [MT2024504]  
*Course:* ASIC Design  
*Instructor:* [Prof. Kunal gosh]
