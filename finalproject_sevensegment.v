`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Chao-Jia Liu (Peter)
// 
// Create Date: 07/22/2024 12:35:48 AM
// Design Name: 
// Module Name: finalproject_sevensegment
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module MUX2to1_8b(PC_addr, D_addr, D_addr_sel, addr);
    parameter word_size = 8;
    input [word_size-1:0] PC_addr, D_addr;
    input D_addr_sel;
    output reg [word_size-1:0] addr;
    always@(*) begin
        if(D_addr_sel == 0) addr <= PC_addr;
        else if (D_addr_sel == 1) addr <= D_addr;
    end
endmodule

module MUX3to1_16b(data_0, data_1, data_2, sel, mux_out);
    parameter word_size = 16;
    parameter select_size = 2;
    input [word_size-1:0] data_0, data_1, data_2;
    input [select_size-1:0] sel;
    output reg [word_size-1:0] mux_out;
    
    always@(*) begin
        case(sel) 
            2'b00: mux_out <= data_0;
            2'b01: mux_out <= data_1;
            2'b10: mux_out <= data_2;
            2'b11: mux_out <= 16'bX;
        endcase
    end 
endmodule

module RegisterUnit(data_out, data_in, wr, rp_rd, rq_rd, wr_addr_sel, rp_addr_sel, rq_addr_sel, clk, rst);
    parameter word_size = 16;
    parameter address_size = 4;
    output reg [word_size-1:0] data_out;
    input [word_size-1:0] data_in;
    input wr_addr_sel, rp_addr_sel, rq_addr_sel;
    input wr, rp_rd, rq_rd, clk, rst;
    reg [word_size-1:0] data_inner;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            data_inner <= 0;
            data_out <= 0;
        end else if (wr && wr_addr_sel) begin
            data_inner <= data_in;
        end
    end
    
    always @(*) begin
        if (rp_rd && rp_addr_sel) begin
            data_out <= data_inner;
        end else if (rq_rd && rq_addr_sel) begin
            data_out <= data_inner;
        end else begin
            data_out <= data_out;
        end
    end
endmodule

module decoder1to16(address_out, address_in);
    input [3:0] address_in;
    output reg [15:0] address_out;
        always@(*) begin
            case(address_in)
                4'b0000: address_out <= 16'b0000000000000001; 
                4'b0001: address_out <= 16'b0000000000000010;  
                4'b0010: address_out <= 16'b0000000000000100; 
                4'b0011: address_out <= 16'b0000000000001000; 
                4'b0100: address_out <= 16'b0000000000010000; 
                4'b0101: address_out <= 16'b0000000000100000; 
                4'b0110: address_out <= 16'b0000000001000000; 
                4'b0111: address_out <= 16'b0000000010000000; 
                4'b1000: address_out <= 16'b0000000100000000; 
                4'b1001: address_out <= 16'b0000001000000000; 
                4'b1010: address_out <= 16'b0000010000000000; 
                4'b1011: address_out <= 16'b0000100000000000; 
                4'b1100: address_out <= 16'b0001000000000000; 
                4'b1101: address_out <= 16'b0010000000000000; 
                4'b1110: address_out <= 16'b0100000000000000; 
                4'b1111: address_out <= 16'b1000000000000000; 
            endcase
        end
endmodule

module selector16to2(r0out, r1out, r2out, r3out, r4out, r5out, r6out, r7out, r8out, r9out,
                     r10out, r11out, r12out, r13out, r14out, r15out, Rp_select, Rq_select, Rp_data, Rq_data);
    parameter word_size = 16; 
    parameter address = 4;
    input [word_size-1:0] r0out, r1out, r2out, r3out, r4out, r5out, r6out, r7out, r8out, r9out,
                          r10out, r11out, r12out, r13out, r14out, r15out;
    input [address-1:0] Rp_select, Rq_select;
    output reg [word_size-1:0] Rp_data, Rq_data;
    always@(*) begin
        case (Rp_select)
            4'b0000: Rp_data <= r0out;
            4'b0001: Rp_data <= r1out;
            4'b0010: Rp_data <= r2out;
            4'b0011: Rp_data <= r3out;
            4'b0100: Rp_data <= r4out; 
            4'b0101: Rp_data <= r5out;
            4'b0110: Rp_data <= r6out;
            4'b0111: Rp_data <= r7out;
            4'b1000: Rp_data <= r8out;
            4'b1001: Rp_data <= r9out; 
            4'b1010: Rp_data <= r10out;
            4'b1011: Rp_data <= r11out;
            4'b1100: Rp_data <= r12out;
            4'b1101: Rp_data <= r13out;
            4'b1110: Rp_data <= r14out; 
            4'b1111: Rp_data <= r15out;
        endcase
    end
    always@(*) begin
        case (Rq_select)
            4'b0000: Rq_data <= r0out;
            4'b0001: Rq_data <= r1out;
            4'b0010: Rq_data <= r2out;
            4'b0011: Rq_data <= r3out;
            4'b0100: Rq_data <= r4out; 
            4'b0101: Rq_data <= r5out;
            4'b0110: Rq_data <= r6out;
            4'b0111: Rq_data <= r7out;
            4'b1000: Rq_data <= r8out;
            4'b1001: Rq_data <= r9out; 
            4'b1010: Rq_data <= r10out;
            4'b1011: Rq_data <= r11out;
            4'b1100: Rq_data <= r12out;
            4'b1101: Rq_data <= r13out;
            4'b1110: Rq_data <= r14out; 
            4'b1111: Rq_data <= r15out;
        endcase
    end
endmodule

module RegisterFile16x16(Rp_data, Rq_data, W_data, W_addr, W_wr, Rp_addr, Rp_rd, Rq_addr, Rq_rd, clk, rst);
    parameter word_size = 16;
    parameter address = 4;
    output wire [word_size-1:0] Rp_data, Rq_data;
    input [word_size-1:0] W_data;
    input [address-1:0] W_addr, Rp_addr, Rq_addr;
    input W_wr, Rp_rd, Rq_rd, clk, rst;
    wire [word_size-1:0] data_out0, data_out1, data_out2, data_out3, data_out4, data_out5,
                         data_out6, data_out7, data_out8, data_out9, data_out10, data_out11,
                         data_out12, data_out13, data_out14, data_out15;
    wire [word_size-1:0] write_address_select, rp_address_select, rq_address_select;
    reg [15:0] Inner_Register; // keep the data in case, simulation verification purpose
    
    always@(W_data) begin
        Inner_Register <= W_data;
    end
    
    decoder1to16 Write_address_decoder(.address_out(write_address_select),.address_in(W_addr));
    decoder1to16 Rp_address_decoder(.address_out(rp_address_select),.address_in(Rp_addr));
    decoder1to16 Rq_address_decoder(.address_out(rq_address_select),.address_in(Rq_addr));
    RegisterUnit R0(.data_out(data_out0), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[0]), .rp_addr_sel(rp_address_select[0]), .rq_addr_sel(rq_address_select[0]), .clk(clk), .rst(rst));
    RegisterUnit R1(.data_out(data_out1), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[1]), .rp_addr_sel(rp_address_select[1]), .rq_addr_sel(rq_address_select[1]), .clk(clk), .rst(rst));
    RegisterUnit R2(.data_out(data_out2), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[2]), .rp_addr_sel(rp_address_select[2]), .rq_addr_sel(rq_address_select[2]), .clk(clk), .rst(rst));
    RegisterUnit R3(.data_out(data_out3), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[3]), .rp_addr_sel(rp_address_select[3]), .rq_addr_sel(rq_address_select[3]), .clk(clk), .rst(rst));
    RegisterUnit R4(.data_out(data_out4), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[4]), .rp_addr_sel(rp_address_select[4]), .rq_addr_sel(rq_address_select[4]), .clk(clk), .rst(rst));
    RegisterUnit R5(.data_out(data_out5), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[5]), .rp_addr_sel(rp_address_select[5]), .rq_addr_sel(rq_address_select[5]), .clk(clk), .rst(rst));
    RegisterUnit R6(.data_out(data_out6), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[6]), .rp_addr_sel(rp_address_select[6]), .rq_addr_sel(rq_address_select[6]), .clk(clk), .rst(rst));
    RegisterUnit R7(.data_out(data_out7), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[7]), .rp_addr_sel(rp_address_select[7]), .rq_addr_sel(rq_address_select[7]), .clk(clk), .rst(rst));
    RegisterUnit R8(.data_out(data_out8), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[8]), .rp_addr_sel(rp_address_select[8]), .rq_addr_sel(rq_address_select[8]), .clk(clk), .rst(rst));
    RegisterUnit R9(.data_out(data_out9), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[9]), .rp_addr_sel(rp_address_select[9]), .rq_addr_sel(rq_address_select[9]), .clk(clk), .rst(rst));
    RegisterUnit R10(.data_out(data_out10), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[10]), .rp_addr_sel(rp_address_select[10]), .rq_addr_sel(rq_address_select[10]), .clk(clk), .rst(rst));
    RegisterUnit R11(.data_out(data_out11), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[11]), .rp_addr_sel(rp_address_select[11]), .rq_addr_sel(rq_address_select[11]), .clk(clk), .rst(rst));
    RegisterUnit R12(.data_out(data_out12), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[12]), .rp_addr_sel(rp_address_select[12]), .rq_addr_sel(rq_address_select[12]), .clk(clk), .rst(rst));
    RegisterUnit R13(.data_out(data_out13), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[13]), .rp_addr_sel(rp_address_select[13]), .rq_addr_sel(rq_address_select[13]), .clk(clk), .rst(rst));
    RegisterUnit R14(.data_out(data_out14), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[14]), .rp_addr_sel(rp_address_select[14]), .rq_addr_sel(rq_address_select[14]), .clk(clk), .rst(rst));
    RegisterUnit R15(.data_out(data_out15), .data_in(W_data), .wr(W_wr), .rp_rd(Rp_rd), .rq_rd(Rq_rd), .wr_addr_sel(write_address_select[15]), .rp_addr_sel(rp_address_select[15]), .rq_addr_sel(rq_address_select[15]), .clk(clk), .rst(rst));
    selector16to2 memory_output_selector(.r0out(data_out0),.r1out(data_out1),.r2out(data_out2),.r3out(data_out3),.r4out(data_out4),
                  .r5out(data_out5),.r6out(data_out6),.r7out(data_out7),.r8out(data_out8),.r9out(data_out9), 
                  .r10out(data_out10),.r11out(data_out11),.r12out(data_out12),.r13out(data_out13),.r14out(data_out14),
                  .r15out(data_out15),.Rp_select(Rp_addr),.Rq_select(Rq_addr),.Rp_data(Rp_data),.Rq_data(Rq_data));                     
endmodule

module ALU(A, B, sel, alu_out);
    parameter word_size = 16;
    parameter op_size = 4;
    //operation code definition:
    parameter ADD = 4'b0000;
    parameter SUB = 4'b0001;
    parameter AND = 4'b0010;
    parameter OR  = 4'b0011;
    parameter XOR = 4'b0100;
    parameter NOT = 4'b0101;
    parameter SLA = 4'b0110;
    parameter SRA = 4'b0111;
    input [word_size-1:0] A, B;
    input [op_size-2:0] sel;
    output reg [word_size-1:0] alu_out;
    
     always@(*) begin
        case(sel)
            ADD: alu_out <= A + B;
            SUB: alu_out <= A - B;
            AND: alu_out <= A & B;
            OR : alu_out <= A | B;
            XOR: alu_out <= A ^ B;
            NOT: alu_out <= ~A;
            SLA: alu_out <= A << 1;
            SRA: alu_out <= A >> 1;
            default alu_out <= 16'b0;
        endcase
     end
endmodule

module D_memory(R_data, W_data, addr, clk, rd, wr);
    parameter address_size = 8;
    parameter word_size = 16;
    parameter memory_size = 256;
    output [word_size-1:0] R_data;
    input [word_size-1:0] W_data;
    input [address_size-1:0] addr;
    input clk, rd, wr;
    reg [word_size-1:0] memory [memory_size-1:0];
    
    // Initialize memory with specific values at specific addresses
    initial begin
        memory[0] = 16'b1001010111001001;
        memory[1] = 16'b1001011011001010;
        memory[2] = 16'b0000011101010110;
        memory[3] = 16'b1010011111001011;
        memory[4] = 16'b1000100011111010;
        memory[5] = 16'b0001010010000101;
        memory[6] = 16'b1010010011001100;
        memory[7] = 16'b0111001101110000;
        memory[8] = 16'b0100001000110100;
        memory[9] = 16'b1010001011001101;
        memory[201] = 16'h1111; // Initial value for location 201
        memory[202] = 16'h2222; // Initial value for location 202
    end
    
    always @(posedge clk) begin
        if (wr) begin
            memory[addr] <= W_data;
        end
    end
    assign R_data = memory[addr];
endmodule

module Datapath(M_data_in, RF_RP_zero, data_JR, M_data_out, RF_W_data, mux_sel, W_addr, W_wr, Rp_addr, Rp_rd, Rq_addr, Rq_rd, alu_sel, clk, rst);
    parameter word_size1 = 8;
    parameter word_size2 = 16;
    parameter mux_select_size = 2;
    parameter op_size = 4;
    parameter address = 4;
    output RF_RP_zero;
    output [word_size2-1:0] M_data_in;
    output [word_size1-1:0] data_JR;
    input [word_size2-1:0] RF_W_data;
    input [word_size2-1:0] M_data_out;
    wire  [word_size2-1:0] alutomux31;
    input [mux_select_size-1:0] mux_sel;
    wire  [word_size2-1:0] muxtoRF;
    input [address-1:0] W_addr, Rp_addr, Rq_addr;
    wire  [word_size2-1:0] Rp_RFtoalu, Rq_RFtoalu;
    input W_wr, Rp_rd, Rq_rd, clk, rst;
    input [op_size-2:0] alu_sel;
    
    MUX3to1_16b M1(.data_0(alutomux31), .data_1(M_data_out), .data_2(RF_W_data), .sel(mux_sel), .mux_out(muxtoRF));
    RegisterFile16x16 R1(.Rp_data(Rp_RFtoalu), .Rq_data(Rq_RFtoalu), .W_data(muxtoRF), .W_addr(W_addr), .W_wr(W_wr),
                         .Rp_addr(Rp_addr), .Rp_rd(Rp_rd), .Rq_addr(Rq_addr), .Rq_rd(Rq_rd), .clk(clk), .rst(rst));
    assign M_data_in = Rp_RFtoalu;
    
    assign data_JR = Rp_RFtoalu[7:0];
    
    //probably we need to change
    assign RF_RP_zero = ~|Rp_RFtoalu; // reduced OR (unary OR) operation for comparison
    
    ALU alu1(.A(Rp_RFtoalu), .B(Rq_RFtoalu), .sel(alu_sel), .alu_out(alutomux31));
endmodule



module Controller(IR_out, RF_Rp_zero, PC_ld, PC_clr, PC_inc, 
                  D_addr_sel, D_addr, IR_ld, mux_sel, D_rd, D_wr, 
                  RF_W_data_sel, RF_W_addr, W_wr, RF_Rp_addr, Rp_rd,
                  RF_Rq_addr, Rq_rd, RF_rst, alu_sel, clk, rst);
    parameter Instruction_size = 16;
    parameter S_initial = 0, S_fetch = 1, S_decode = 2; 
    parameter S_executeALU = 3, S_after_execution = 4;
    parameter S_executeBIZ = 5, S_executeBNZ = 6, S_executeJAL = 7, S_afterJR = 8, S_stay = 9;         
    parameter ADD = 0, SUB = 1, AND = 2, OR = 3;
    parameter XOR = 4, NOT = 5, SLA = 6, SRA = 7;
    parameter LI = 8, LW = 9, SW = 10, BIZ = 11;
    parameter BNZ = 12, JAL = 13, JMP = 14, JR = 15;
    input [Instruction_size-1:0] IR_out;
    input RF_Rp_zero;
    output reg PC_ld, PC_clr, PC_inc, D_addr_sel;
    output reg [7:0] D_addr;
    output reg IR_ld;
    output reg [1:0] mux_sel;
    output reg D_rd, D_wr, RF_W_data_sel;
    output reg [3:0] RF_W_addr, RF_Rp_addr, RF_Rq_addr;
    output reg W_wr, Rp_rd, Rq_rd, RF_rst;
    output reg [2:0] alu_sel;
    input clk, rst;
    reg [3:0] state, next_state;
    reg err_flag; // debug simulation
    wire [Instruction_size-1:0] IR_out;
    wire [4:0]opcode = IR_out[15:12];
    wire [4:0] Rd = IR_out[11:8];
    wire [4:0] Rp = IR_out[7:4];
    wire [4:0] Rq = IR_out[3:0];
    wire [7:0] Immediate = IR_out[7:0];
    wire [2:0] aluselect = IR_out[14:12];
    
    
    always@(posedge clk)begin: State_transitions
        if(rst == 1) state <= S_initial; else state <= next_state;
    end
    
    always@(state or opcode or RF_W_addr or RF_Rp_addr or RF_Rq_addr or RF_Rp_zero)begin 
        PC_ld = 0; PC_clr = 0; PC_inc = 0; D_addr_sel =0;
        D_addr = 8'b00000000; IR_ld = 0; mux_sel = 2'b00;
        D_rd = 0; D_wr = 0; RF_W_data_sel = 0;
        RF_W_addr = Rd; RF_Rp_addr = Rp; RF_Rq_addr = Rq;
        W_wr = 0; Rp_rd = 0; Rq_rd = 0; RF_rst = 0;
        alu_sel = 3'b000;
        err_flag = 0;
        next_state = state;
        
        case (state)
            S_initial:  begin
                        next_state = S_fetch;
                        PC_clr = 1; // PC_addr = 8'b00000000
                        PC_ld = 0;
                        PC_inc = 0;
                        IR_ld = 0;
                        $display("State: S_initial");
                        end
            S_fetch  :  begin
                        next_state = S_decode;
                        PC_clr = 0;
                        PC_ld = 0;
                        PC_inc = 1;
                        IR_ld = 1;
                        $display("State: S_fetch");
                        end
            S_decode :  begin
                        $display("opcode: %b", opcode);
                        case(opcode)
                           ADD, SUB, AND, OR, XOR, NOT, SLA, SRA: begin // ALU
                            next_state = S_executeALU;
                            D_rd = 1;
                            mux_sel = 2'b01;
                            RF_Rp_addr = Rp;
                            Rp_rd = 1;
                            RF_Rq_addr = Rq;
                            Rq_rd = 1;                           
                            $display("ALU operation");
                           end
                           LI : begin // LI
                           next_state = S_after_execution;
                            RF_W_data_sel = 0;
                            mux_sel = 2'b10;
                            RF_W_addr = Rd;
                            W_wr = 1;
                            $display("LI operation");
                           end
                           LW : begin // LW
                           next_state = S_after_execution;
                            D_rd = 1;
                            D_addr_sel = 1;
                            D_addr = Immediate;
                            IR_ld = 0;
                            mux_sel = 2'b01;
                            RF_W_addr = Rd;
                            W_wr = 1;
                            $display("LW operation");
                           end
                           SW : begin // SW
                           next_state = S_after_execution;
                            RF_Rp_addr = Rd;
                            Rp_rd = 1;
                            D_rd = 0;
                            D_wr = 1;
                            D_addr = Immediate;
                            D_addr_sel = 1;
                            $display("SW operation");
                           end
                           BIZ: begin // BIZ
                            next_state = S_executeBIZ;
                            RF_Rp_addr = Rd;
                            Rp_rd = 1;
                            $display("BIZ operation");
                           end
                           BNZ: begin // BNZ
                            next_state = S_executeBNZ;
                            RF_Rp_addr = Rd;
                            Rp_rd = 1;
                            $display("BNZ operation");
                           end
                           JAL: begin // JAL
                            next_state = S_after_execution;
                            RF_W_data_sel = 1;
                            mux_sel = 2'b10;
                            RF_W_addr = Rd;
                            W_wr = 1;
                            PC_ld = 1;
                            PC_inc = 0;
                            $display("JAL operation");
                           end
                           JMP: begin // JMP
                            next_state = S_after_execution;
                            PC_ld = 1;
                            PC_inc = 0;
                            $display("JMP operation");
                           end
                           JR : begin // JR
                            $display("JR operation");
                            D_rd = 1;
                            mux_sel = 2'b01;
                            RF_Rp_addr = Rp;
                            Rp_rd = 1;
                            PC_ld = 1;
                            PC_inc = 1;   
                           end
                           default: 
                           begin next_state = S_stay;
                           $display("Invalid opcode");
                           end
                        endcase // opcode
                        end
             S_executeALU:begin
                        next_state = S_after_execution;
                        alu_sel = aluselect;
                        mux_sel = 2'b00;
                        RF_W_addr = Rd;
                        W_wr = 1;
                        $display("State: S_afterALU");
                        end
            S_after_execution:begin
                        next_state = S_fetch;
                        IR_ld = 1;
                        $display("State: S_after Execution");
                        end           
            S_executeBIZ:begin
                          if(RF_Rp_zero == 0)
                            begin
                            next_state = S_fetch;
                            PC_ld = 1;
                            PC_inc = 0;
                            end
                         else begin
                            next_state = S_fetch;
                            end
                            $display("State: S_executeBIZ");
                         end 
            S_executeBNZ:begin
                          if(RF_Rp_zero != 0)
                            begin
                            next_state = S_fetch;
                            PC_ld = 1;
                            PC_inc = 0;
                            end
                         else begin
                            next_state = S_fetch;
                            end
                            $display("State: S_executeBNZ");
                         end
            S_stay       :begin
                           D_rd = 1;                 
                           $display("State: S_stay");
                          end
            default: begin next_state = S_initial;
            $display("State: Default, resetting to S_initial");
            end
        endcase
    end
endmodule

module ProgramCounter(PC_addr, data_in, data_JR, PC_ld, PC_inc, PC_clr, clk);
    parameter word_size = 8;
    output reg [word_size-1:0] PC_addr;
    input  [word_size-1:0] data_in, data_JR;
    input  PC_ld, PC_inc, PC_clr, clk;
    
    always@(posedge clk)
        if(PC_clr==1) PC_addr <= 8'b0; 
        else if (PC_ld == 1 && PC_inc == 0) PC_addr <= data_in; 
        else if (PC_inc == 1 && PC_ld == 0) PC_addr <= PC_addr + 1;
        else if (PC_ld == 1 && PC_inc == 1) PC_addr <= data_JR; 
endmodule

module InstructionRegister(data_out, data_in, IR_ld, rst, clk);
    parameter word_size = 16;
    output reg [word_size-1:0] data_out;
    input [word_size-1:0] data_in;
    input IR_ld, rst, clk;
    always@(posedge clk)
        if (IR_ld) data_out <= data_in;
endmodule

module Data_in_compute(a, b, out);
    parameter word_size = 8;
    input [word_size-1:0] a, b;
    output [word_size-1:0] out;
    assign out = a + b - 1;
endmodule

module RF_W_data_selector(data_0, data_1, sign_extendBit, RF_W_data_sel, mux_out);
    parameter in_size = 8;
    parameter out_size = 16;
    input [in_size-1:0] data_0, data_1;
    input RF_W_data_sel, sign_extendBit;
    output reg [out_size-1:0] mux_out;
    
    always@(*) begin
        case(RF_W_data_sel) 
            1'b0: 
                if(sign_extendBit == 0)
                    mux_out = {8'b00000000, data_0};
                else if(sign_extendBit == 1)
                    mux_out = {8'b11111111, data_0};
            1'b1: mux_out = {8'b00000000, data_1};
        endcase
    end 
endmodule

module ControlUnit(RF_Rp_zero, D_addr_sel, D_addr, mux_sel, D_rd, D_wr, RF_W_addr, RF_Rp_addr, RF_Rq_addr,
                   W_wr, Rp_rd, Rq_rd, RF_rst, alu_sel, clk, rst, Mem_to_IR, data_JR, PC_addr, mux_out);
    parameter word_size = 16;
    // Controller 
    input RF_Rp_zero;
    output D_addr_sel;
    output [7:0] D_addr;
    output [1:0] mux_sel;
    output D_rd, D_wr;
    output [3:0] RF_W_addr, RF_Rp_addr, RF_Rq_addr;
    output W_wr, Rp_rd, Rq_rd, RF_rst;
    output [2:0] alu_sel;
    input clk, rst;
    // Instruction Register 
    input [word_size-1:0] Mem_to_IR;
    // Program Counter 
    input [7:0] data_JR;
    output [7:0] PC_addr;
    // RF_W_data_selector
    output [word_size-1:0] mux_out;
    // Controller inner wire
    wire PC_ld, PC_clr, PC_inc, IR_ld, RF_W_data_sel;
    // Instruction Register inner wire
    wire [word_size-1:0] IR_data_out;
    // Program Counter wire
    wire [7:0] data_in;
//    wire [7:0] PC_addr;
    //other 

    
    Controller ctrl1(.IR_out(IR_data_out), .RF_Rp_zero(RF_Rp_zero), .PC_ld(PC_ld), .PC_clr(PC_clr), .PC_inc(PC_inc), 
                     .D_addr_sel(D_addr_sel), .D_addr(D_addr), .IR_ld(IR_ld), .mux_sel(mux_sel), .D_rd(D_rd), .D_wr(D_wr), 
                     .RF_W_data_sel(RF_W_data_sel), .RF_W_addr(RF_W_addr), .W_wr(W_wr), .RF_Rp_addr(RF_Rp_addr), .Rp_rd(Rp_rd),
                     .RF_Rq_addr(RF_Rq_addr), .Rq_rd(Rq_rd), .RF_rst(RF_rst), .alu_sel(alu_sel), .clk(clk), .rst(rst));
    ProgramCounter PC1(.PC_addr(PC_addr), .data_in(data_in), .data_JR(data_JR), .PC_ld(PC_ld), .PC_inc(PC_inc), .PC_clr(PC_clr), .clk(clk));
    InstructionRegister IR1(.data_out(IR_data_out), .data_in(Mem_to_IR), .IR_ld(IR_ld), .rst(PC_clr), .clk(clk));
    Data_in_compute Compute1(.a(PC_addr), .b(IR_data_out[7:0]), .out(data_in));
    RF_W_data_selector sel1(.data_0(IR_data_out[7:0]), .data_1(PC_addr), .sign_extendBit(IR_data_out[7]), .RF_W_data_sel(RF_W_data_sel), .mux_out(mux_out));
endmodule

module RISC(mem201, mem202, mem203, mem204, mem205, clk, rst);
    parameter instruction_size = 16;
    parameter address_size = 8;
    parameter register_address_size = 4;
    input clk, rst;
    output reg [15:0] mem201, mem202, mem203, mem204, mem205;
  
    //wire 
    //Datapath to Control Unit
    wire RF_Rp_zero;
    wire [address_size-1:0] data_JR;
    //Datapath to Memory
    wire [instruction_size-1:0] Datapath_to_Mem;
    //Memory to Control Unit
    wire [instruction_size-1:0] Memory_out;
    //Contorl Unit to MUX2X1
    wire D_addr_sel;
    wire [address_size-1:0] PC_addr, D_addr;
    //Contorl Unit to D_Memory
    wire D_rd, D_wr;
    //Control Unit to Datapath
    wire [1:0] mux_sel;
    wire [register_address_size-1:0] RF_W_addr, RF_Rp_addr, RF_Rq_addr;
    wire W_wr, Rp_rd, Rq_rd;
    wire [2:0] alu_sel;
    wire [instruction_size-1:0] mux_out;
    //MUX2X1 to D_Memory
    wire [address_size-1:0] addr;
    
    ControlUnit ctrl1(.RF_Rp_zero(RF_Rp_zero), .D_addr_sel(D_addr_sel), .D_addr(D_addr), .mux_sel(mux_sel), .D_rd(D_rd), .D_wr(D_wr), 
                      .RF_W_addr(RF_W_addr), .RF_Rp_addr(RF_Rp_addr), .RF_Rq_addr(RF_Rq_addr), .W_wr(W_wr), .Rp_rd(Rp_rd), .Rq_rd(Rq_rd), 
                      .RF_rst(RF_rst), .alu_sel(alu_sel), .clk(clk), .rst(rst), .Mem_to_IR(Memory_out), .data_JR(data_JR), .PC_addr(PC_addr), 
                      .mux_out(mux_out));
    Datapath datapath1(.M_data_in(Datapath_to_Mem), .RF_RP_zero(RF_Rp_zero), .data_JR(data_JR), .M_data_out(Memory_out), .RF_W_data(mux_out), 
                       .mux_sel(mux_sel), .W_addr(RF_W_addr), .W_wr(W_wr), .Rp_addr(RF_Rp_addr), .Rp_rd(Rp_rd), .Rq_addr(RF_Rq_addr), .Rq_rd(Rq_rd), 
                       .alu_sel(alu_sel), .clk(clk), .rst(RF_rst));
    D_memory dmem1(.R_data(Memory_out), .W_data(Datapath_to_Mem), .addr(addr), .clk(clk), .rd(D_rd), .wr(D_wr));  
    MUX2to1_8b mux1(.PC_addr(PC_addr), .D_addr(D_addr), .D_addr_sel(D_addr_sel), .addr(addr));        
    
    always@(*)begin
        mem201 <= dmem1.memory[201];
        mem202 <= dmem1.memory[202];
        mem203 <= dmem1.memory[203];
        mem204 <= dmem1.memory[204];
        mem205 <= dmem1.memory[205];
    end
        
endmodule

module bin2bcd( // binary to decimal using double dabble algorithm
    input [15:0] bin,
    output reg [19:0] bcd
    );

    integer i;

    always @(bin) begin
    bcd = 20'b0;

    for (i = 0; i < 16; i = i + 1) begin
        // Add 3 to each BCD digit if it is greater than or equal to 5 (Double Dabble algroithm)
        // compare and add the BCD value
        if (bcd[3:0] >= 5) bcd[3:0] = bcd[3:0] + 3; 
        if (bcd[7:4] >= 5) bcd[7:4] = bcd[7:4] + 3;
        if (bcd[11:8] >= 5) bcd[11:8] = bcd[11:8] + 3;
        if (bcd[15:12] >= 5) bcd[15:12] = bcd[15:12] + 3;
        if (bcd[19:16] >= 5) bcd[19:16] = bcd[19:16] + 3;
        // Shift the current BCD value left by one bit
        bcd = {bcd[18:0], bin[15 - i]};
        end
    end
endmodule

module sevensegment1to8( // seven segmant displayment for one LED segment
    input [3:0] in,
    output reg [7:0] seg
);
    always@(in) begin 
        case (in) 
            4'd0: seg = 8'b11000000; // 0
            4'd1: seg = 8'b11111001; // 1
            4'd2: seg = 8'b10100100; // 2
            4'd3: seg = 8'b10110000; // 3
            4'd4: seg = 8'b10011001; // 4
            4'd5: seg = 8'b10010010; // 5
            4'd6: seg = 8'b10000010; // 6
            4'd7: seg = 8'b11111000; // 7
            4'd8: seg = 8'b10000000; // 8
            4'd9: seg = 8'b10010000; // 9
            default seg = 8'b11111111; // turn off the LED
        endcase
    end
endmodule

module finalproject_sevensegment(
    input clk_i, 
    input [15:0] SW,
    output [15:0] LED,
    output reg [7:0] disp_an_o,
    output reg [7:0] disp_seg_o
    );
    wire [15:0] mem201, mem202, mem203, mem204, mem205;
     
     assign  LED[0] = SW[0];
    
    RISC risc1(.mem201(mem201), .mem202(mem202), .mem203(mem203), .mem204(mem204), .mem205(mem205), .clk(clk_i), .rst(SW[0]));
    
    reg [32:0] counter_for_mem;
    reg [2:0] select_for_mem;
    reg [15:0] select_for_mem_value;  
    
    always @(posedge clk_i) begin 
        if (counter_for_mem != 100000000) begin 
            counter_for_mem <= counter_for_mem + 1;
        end
        else begin 
            counter_for_mem <= 0;
            select_for_mem <= (select_for_mem < 4)? select_for_mem+1:0; //loop through 0 to 4
        end
    end

    always @(*) begin
        // use case statement 
        case (select_for_mem)
            3'd0: begin 
                select_for_mem_value <= mem201;
            end
            3'd1: begin
                select_for_mem_value <= mem202;
            end
            3'd2: begin
                select_for_mem_value <= mem203;
            end
            3'd3: begin
                select_for_mem_value <= mem204;
            end
            3'd4: begin
                select_for_mem_value <= mem205;
            end
         endcase
      end
    
    // BCD 
    wire [19:0] bcd;
    bin2bcd uut(
        .bin(select_for_mem_value),
        .bcd(bcd)
    );
    // store the value for each LED Segment
    // 5 segment LED will displayed so 5 wire and each are 8-bit
    wire [7:0] seg0;
    wire [7:0] seg1;
    wire [7:0] seg2;
    wire [7:0] seg3;
    wire [7:0] seg4;
    //get the values for each segment (calling 5 times)
    // For seg 0
    sevensegment1to8 LED0(
        .in(bcd[3:0]),
        .seg(seg0)
    );
    //For seg 1
    sevensegment1to8 LED1(
        .in(bcd[7:4]),
        .seg(seg1)
    );
    //For seg 2
    sevensegment1to8 LED2(
        .in(bcd[11:8]),
        .seg(seg2)
    );
    //For seg 3
    sevensegment1to8 LED3(
        .in(bcd[15:12]),
        .seg(seg3)
    );
    //For seg 4
    sevensegment1to8 LED4(
        .in(bcd[19:16]),
        .seg(seg4)
    );
    
    //make a 1KHz clock input so the human eyes can not see it switching
    reg [16:0] counter; // 17-bit counter to divide 100MHDZ Clock into 1KHz
    reg [2:0] select; // Selector for the mux (3 bits because only need 5) 
    always @(posedge clk_i) begin 
        if (counter != 50000) begin 
            counter <= counter + 1;
        end
        else begin 
            counter <= 0;
            select <= (select < 4)? select+1:0; //loop through 0 to 4
        end
    end

    always @(*) begin
        // use case statement 
        case (select)
            3'd0: begin 
                disp_an_o <= 8'b11111110; //display the AN0
                disp_seg_o <= seg0;
            end
            3'd1: begin
                disp_an_o <= 8'b11111101; //display the AN1
                disp_seg_o <= seg1;
            end
            3'd2: begin
                disp_an_o <= 8'b11111011; //display the AN2 
                disp_seg_o <= seg2;
            end
            3'd3: begin
                disp_an_o <= 8'b11110111;  //display the AN3 
                disp_seg_o <= seg3;
            end
            3'd4: begin
                disp_an_o <= 8'b11101111; //display the AN4
                disp_seg_o <= seg4;
            end
            default: begin 
                disp_an_o <= 8'b11111111; // all display not used
            end
        endcase
    end
endmodule
