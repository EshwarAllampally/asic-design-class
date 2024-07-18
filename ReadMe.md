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
    /add img1

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
    /add Img2

### References
- [GCC Documentation](https://gcc.gnu.org/)
- [RISC-V GNU Compiler Toolchain Documentation](https://github.com/riscv/riscv-gnu-toolchain)

---

*Prepared by:* [Eshwar Allampally]  
*Student ID:* [MT2024504]  
*Course:* ASIC Design  
*Instructor:* [Prof. Kunal gosh]
