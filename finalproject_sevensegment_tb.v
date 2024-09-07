`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Chao-Jia Liu (Peter)
// 
// Create Date: 07/22/2024 05:04:15 PM
// Design Name: 
// Module Name: finalproject_sevensegment_tb
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


module finalproject_sevensegment_tb;
    reg clk_i, rst;
    wire [15:0] SW;
    wire [15:0] mem201, mem202, mem203, mem204, mem205;

    finalproject_sevensegment uut (
        .clk_i(clk_i),
        .SW(SW[0]),
        .disp_an_o(),
        .disp_seg_o()
    );

    initial begin
        clk_i = 0;
        forever #5 clk_i = ~clk_i;
    end

    initial begin
        rst = 1;
        #10;
        rst = 0;

        // Simulate reset
        #10;
        rst = 1;
        #10;
        rst = 0;

        #1000000;
        $stop;
    end

    initial begin
        $monitor("Time: %d, mem201: %h, mem202: %h, mem203: %h, mem204: %h, mem205: %h", 
                 $time, uut.risc1.mem201, uut.risc1.mem202, uut.risc1.mem203, uut.risc1.mem204, uut.risc1.mem205);
    end

endmodule
