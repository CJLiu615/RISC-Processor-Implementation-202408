**RISC Processor on FPGA (EE5193 Final Project)**

This project implements a Reduced Instruction Set Computer (RISC) architecture on an FPGA using Verilog HDL. Designed as the final project for the course FPGA and HDL (EE5193), the system features a custom instruction set, memory-mapped I/O, and complete execution flow — including fetch, decode, execute, and program counter update stages.

**📌 Project Goals**

* Design a functional RISC processor with a 16-bit instruction format

* Implement instruction execution over 4 stages

* Develop a Verilog-based controller and datapath

* Synthesize the design on an FPGA board

* Display output values using a 7-segment display

**🧱 Architecture Components**

🔧 Instruction Execution Pipeline

1. Fetch: Load instruction from memory into the Instruction Register (IR)

2. Decode: Interpret opcode and operands

3. Execute: Perform ALU or memory operations

4. Update PC: Move to next instruction

**🧮 Supported Instructions**

![image](https://github.com/user-attachments/assets/cf01d930-7aff-47a4-9d00-d530fbfd5b89)
![image](https://github.com/user-attachments/assets/27abe9ef-929e-48d1-9116-7d34caff1fcb)

**🧾 Instruction Requirements**

![image](https://github.com/user-attachments/assets/a46c6a96-0278-424e-8f0b-1d91e14a73f4)

**💾 Memory Mapping**

* 201: Initial value = 1111 (hex)

* 202: Initial value = 2222 (hex)

* 203, 204, 205: Final result outputs

**📟 Output Display**

Final values stored in memory are shown via the 7-segment display on the FPGA board.

**🛠️ Implementation Details**

* Language: Verilog HDL

* Modules: Register file, ALU, control unit, data memory, binary-to-decimal decoder

* Top module integrates all subsystems for synthesis

* ASM-D Chart used for controller FSM design
![state diagram 1](https://github.com/user-attachments/assets/7665e422-a78e-4511-8da9-c1a7fc9a2690)
![state diagram 2](https://github.com/user-attachments/assets/74caa2a2-446c-4fbd-8957-5682ae8d503e)
![state diagram 3](https://github.com/user-attachments/assets/a55f8ab9-3ac0-4afe-b28a-159c74135c20)

* Testbench verifies pipeline execution

* Synthesis completed and validated on hardware
![image](https://github.com/user-attachments/assets/f76abd77-47c9-4f4a-9e5c-3e920719998c)
![block diagram](https://github.com/user-attachments/assets/3bf85c31-3b46-4ecf-ab01-31ca53784d01)



**📈 Simulation and Verification**

* Waveform results included for each instruction

* Internal registers and memory updated per clock cycle

**Output:**

![value3](https://github.com/user-attachments/assets/4a3d8b97-63bf-4661-9d75-f1a20adfc5ed)
![value4](https://github.com/user-attachments/assets/78fac1b0-7241-41a6-aaec-0c443d2792ec)
![value5](https://github.com/user-attachments/assets/66b543c5-c5c6-469b-bf35-49230f6c8398)

**🧪 Tools Used**

* Xilinx Vivado (Simulation and Synthesis)

* FPGA Development Board (on-board validation)

* ModelSim (Optional testbench)

* Verilog HDL

**🔍 File Structure**

├── src/                    
:Verilog source code (modules, controller, ALU)

├── testbench/             
:Simulation environment

├── asm_chart/             
:State machine diagrams (ASM-D)

├── bitstream/             
:Compiled FPGA bitstream (optional)

├── docs/                  
:Project report and waveform screenshots

└── README.md

**📸 On-Board Verification**

* Confirmed value outputs via 7-segment display

* Instruction sequence successfully executed on real hardware

* Photo and video documentation included in submission

**👨‍💻 Author**

Chao-Jia Liu (Peter)

Graduate Student – Electrical & Computer Engineering

The University of Texas at San Antonio
