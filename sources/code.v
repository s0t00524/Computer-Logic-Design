/******************************************************************************/
/* Sample Verilog HDL Code for Computer Logic Design     Arch Lab. TOKYO TECH */
/******************************************************************************/
`default_nettype none
/******************************************************************************/
`define NOP  {21'h0, 11'h20}
`define ADD  6'h0
`define ADDI 6'h8
`define LW   6'h23
`define SW   6'h2b
`define BEQ  6'h4
`define BNE  6'h5
`define HALT 6'h11 /* this is not for MIPS */

/******************************************************************************/

 module m_top ();
  reg r_clk=0; initial forever r_clk = ~r_clk;
  reg r_rst=0;

  wire w_halt;
  wire [31:0] w_rout;
  m_proc08 p (r_clk, r_rst, w_rout, w_halt);
  always@(posedge r_clk) if (w_halt) $finish;

  reg [31:0] r_cnt = 0;
  always@(posedge r_clk) begin
    r_cnt <= r_cnt + 1;
    if(p.state==5)
      $write("%8d : %d %x %x: %x : %x %x -> %x\n",
             r_cnt, p.state, p.r_pc, p.w_ir, p.w_op,
             p.w_rrs, p.w_rrt2, p.w_rslt2);
  end
endmodule


/******************************************************************************/
// module m_main (w_clk, w_btnu, w_btnd, w_led, r_sg, r_an);
//   input  wire w_clk, w_btnu, w_btnd;
//   output wire [15:0] w_led;
//   output reg [6:0] r_sg;  // cathode segments
//   output reg [7:0] r_an;  // common anode
//
//   wire w_clk2, w_locked;
//   clk_wiz_0 clk_wiz (w_clk2, 0, w_locked, w_clk);
//
//   wire w_rst = ~w_locked;
//   wire [31:0] w_rout;
//   wire w_halt;
//   m_proc08 p (w_clk2, w_rst, w_rout, w_halt);
//
//   reg [31:0] r_cnt = 0;
//   always @(posedge w_clk2) r_cnt <= (w_rst) ? 0 : (~w_halt) ? r_cnt + 1 : r_cnt;
//
//   wire [31:0] w_data = (w_btnu) ? r_cnt : w_rout;
//   assign w_led = (w_btnd) ? w_data[31:16] : w_data[15:0];
//
//   wire [6:0] w_sg;
//   wire [7:0] w_an;
//   m_7segcon m_7segcon(w_clk2, w_data, w_sg, w_an);
//   always @(posedge w_clk2) r_sg <= w_sg;
//   always @(posedge w_clk2) r_an <= w_an;
// endmodule

/******************************************************************************/
/* define the state of each section */
`define S_IF  1
`define S_ID  2
`define S_EX  3
`define S_MEM 4
`define S_WB  5

module m_proc08 (w_clk, w_rst, r_rout, r_halt);
  input  wire w_clk, w_rst;
  output reg [31:0] r_rout;
  output reg        r_halt;

  wire [31:0] w_ir, w_rrs, w_rrt, w_imm32, w_rrt2, w_rslt, w_ldd, w_rslt2;
  reg  [31:0] r_pc   = 0;
  reg  [31:0] r_rrs  = 0;
  reg  [31:0] r_rrt  = 0;
  reg  [31:0] r_rrt2 = 0;
  reg  [31:0] r_npc  = 0;
  reg   [2:0] state  = 0;
  always @(posedge w_clk) begin
    state <= #3 (w_rst | r_halt) ?
      0 : (state==`S_WB || (state==`S_EX && (w_op==`BEQ | w_op==`BNE)) || (state==`S_MEM && w_op==`SW)) ?
        `S_IF : (state==`S_EX && (w_op==`ADD | w_op==`ADDI)) ?
            `S_WB : state + 1;
    // state <= #3 (w_rst | r_halt) ? 0 : (state==`S_WB) ? `S_IF : state + 1;
  end
  /**************************** IF  **********************************/
  wire  [5:0] w_op = w_ir[31:26];
  wire  [4:0] w_rs = w_ir[25:21];
  wire  [4:0] w_rt = w_ir[20:16];
  wire  [4:0] w_rd = w_ir[15:11];
  wire        w_taken = ((w_op==`BEQ && r_rrs==r_rrt2) ||
                         (w_op==`BNE && r_rrs!=r_rrt2));
  wire [31:0] w_npc = r_pc + 4;
  wire [31:0] w_tpc = r_npc + {w_imm32[29:0], 2'h0};
  always @(posedge w_clk) if(state==`S_WB) begin
    r_pc <= #3 (w_rst | r_halt) ? 0 : (w_taken) ? w_tpc : w_npc;
  end
  m_memory m_imem (w_clk, r_pc[13:2], 1'b0, 0, w_ir);
  always @(posedge w_clk) if(state==`S_IF) begin
    r_npc <= #3 w_npc;
  end
  /**************************** ID  **********************************/
  wire  [4:0] w_rd2 = (w_op!=0) ? w_rt : w_rd;
  wire [15:0] w_imm = w_ir[15:0];
  wire        w_w = (w_op==0 || (w_op>6'h5 && w_op<6'h28));
  m_regfile m_regs (w_clk, w_rs, w_rt, w_rd2, r_w, w_rslt2, w_rrs, w_rrt);
  assign w_imm32 = {{16{w_imm[15]}}, w_imm};
  assign w_rrt2  = (w_op>6'h5) ? w_imm32 : w_rrt;
  always @(posedge w_clk) if(state==`S_ID) begin
    r_rrs  <= #3 w_rrs;
    r_rrt  <= #3 w_rrt;
    r_rrt2 <= #3 w_rrt2;
  end
  /**************************** EX  **********************************/
  wire       w_we = (w_op>6'h27);
  assign #10 w_rslt = r_rrs + r_rrt2;
  reg [31:0] r_rslt = 0;
  reg [31:0] r_dmin = 0;
  reg        r_we   = 0;
  always @(posedge w_clk) if(state==`S_EX) begin
    r_rslt <= #3 w_rslt;
    r_dmin <= #3 r_rrt;
  end
  always @(posedge w_clk) r_we <= #3 (state!=`S_EX) ? 0 : w_we;
  /**************************** MEM **********************************/
  m_memory m_dmem (w_clk, r_rslt[13:2], r_we, r_dmin, w_ldd);
  reg [31:0] r_mrslt = 0;
  always @(posedge w_clk) if(state==`S_MEM) begin
    r_mrslt <= #3 r_rslt;
  end
  /**************************** WB  **********************************/
  assign w_rslt2 = (w_op>6'h19 && w_op<6'h28) ? w_ldd : r_mrslt;
  reg r_w = 0;
  always @(posedge w_clk) begin
    r_w <= (state!=`S_MEM) ? 0 : (w_op==0 || (w_op>6'h5 && w_op<6'h28));
  end
  /******************************************************************/
  initial r_halt = 0;
  always @(posedge w_clk) if (w_op==`HALT) r_halt <= 1;
  initial r_rout = 0;
  reg [31:0] r_tmp=0;
  always @(posedge w_clk) r_tmp <= (w_rst) ? 0 : (w_rs==30) ? w_rrs : r_rout;
  always @(posedge w_clk) r_rout <= r_tmp;
endmodule

/******************************************************************************/
module m_memory (w_clk, w_addr, w_we, w_din, r_dout);
  input  wire w_clk, w_we;
  input  wire [11:0] w_addr;
  input  wire [31:0] w_din;
  output reg  [31:0] r_dout;
  reg [31:0] cm_ram [0:4095]; // 4K word (4096 x 32bit) memory
  always @(posedge w_clk) begin
    r_dout <= cm_ram[w_addr];
    if (w_we) cm_ram[w_addr] <= w_din;
  end
  initial r_dout = 0;

  initial begin
    cm_ram[0] ={`NOP};                            //     nop
    cm_ram[1] ={`ADDI, 5'd0, 5'd8, 16'd4096};     //     addi $8, $0, 4096
    cm_ram[2] ={`ADDI, 5'd0, 5'd9, 16'h0};        //     addi $9, $0, 0
    cm_ram[3] ={`ADDI, 5'd0, 5'd10,16'h0};        //     addi $10,$0, 0
    cm_ram[4] ={`SW,   5'd10,5'd9, 16'd0};        // L01:sw   $9, 0($10)
    cm_ram[5] ={`ADDI, 5'd9, 5'd9, 16'h1};        //     addi $9, $9, 1
    cm_ram[6] ={`ADDI, 5'd10,5'd10,16'h4};        //     addi $10,$10,4
    cm_ram[7] ={`BNE,  5'd8, 5'd9, 16'hfffc};     //     bne  $8, $9, L01
    cm_ram[8] ={`NOP};                            //     nop

    cm_ram[9] ={`ADD,  5'd0, 5'd0, 5'd12,11'h20}; //     addi $12,$0, $0
    cm_ram[10]={`ADDI, 5'd0, 5'd8, 16'd4096};     //     addi $8, $0, 4096
    cm_ram[11]={`ADDI, 5'd0, 5'd9, 16'h0};        //     addi $9, $0, 0
    cm_ram[12]={`ADDI, 5'd0, 5'd10,16'h0};        //     addi $10,$0, 0
    cm_ram[13]={`LW,   5'd10,5'd11,16'd0};        // L02:lw   $11,0($10)
    cm_ram[14]={`ADDI, 5'd9, 5'd9, 16'h1};        //     addi $9, $9, 1
    cm_ram[15]={`ADDI, 5'd10,5'd10,16'h4};        //     addi $10,$10,4
    cm_ram[16]={`ADD,  5'd12,5'd11,5'd12,11'h20}; //     add  $12,$12,$11
    cm_ram[17]={`BNE,  5'd8, 5'd9, 16'hfffb};     //     bne  $8, $9, L02
    cm_ram[18]={`NOP};                            //     nop

    cm_ram[19]={`ADD,  5'd12,5'd0, 5'd30,11'h20}; //     add  $30,$12,$0
    cm_ram[20]={`ADD,  5'd30,5'd0, 5'd0, 11'h20}; //     add  $0, $30,$0
    cm_ram[21]={`HALT, 26'h0};                    //     halt
    cm_ram[22]={`NOP};                            //     nop
    cm_ram[23]={`NOP};                            //     nop
    cm_ram[24]={`NOP};                            //     nop
    cm_ram[25]={`NOP};                            //     nop
    cm_ram[26]={`NOP};                            //     nop
  end
endmodule

/******************************************************************************/
module m_regfile (w_clk, w_rr1, w_rr2, w_wr, w_we, w_wdata, w_rdata1, w_rdata2);
  input  wire        w_clk;
  input  wire  [4:0] w_rr1, w_rr2, w_wr;
  input  wire [31:0] w_wdata;
  input  wire        w_we;
  output wire [31:0] w_rdata1, w_rdata2;

  reg [31:0] r[0:31];
  assign #15 w_rdata1 = (w_rr1==0) ? 0 : r[w_rr1];
  assign #15 w_rdata2 = (w_rr2==0) ? 0 : r[w_rr2];
  always @(posedge w_clk) if(w_we) r[w_wr] <= w_wdata;

  initial begin
    r[1] = 1;
    r[2] = 2;
  end
endmodule

/******************************************************************************/
module m_7segled (w_in, r_led);
  input  wire [3:0] w_in;
  output reg  [6:0] r_led;
  always @(*) begin
    case (w_in)
      4'h0  : r_led <= 7'b1111110;
      4'h1  : r_led <= 7'b0110000;
      4'h2  : r_led <= 7'b1101101;
      4'h3  : r_led <= 7'b1111001;
      4'h4  : r_led <= 7'b0110011;
      4'h5  : r_led <= 7'b1011011;
      4'h6  : r_led <= 7'b1011111;
      4'h7  : r_led <= 7'b1110000;
      4'h8  : r_led <= 7'b1111111;
      4'h9  : r_led <= 7'b1111011;
      4'ha  : r_led <= 7'b1110111;
      4'hb  : r_led <= 7'b0011111;
      4'hc  : r_led <= 7'b1001110;
      4'hd  : r_led <= 7'b0111101;
      4'he  : r_led <= 7'b1001111;
      4'hf  : r_led <= 7'b1000111;
      default:r_led <= 7'b0000000;
    endcase
  end
endmodule

`define DELAY7SEG  200000 // 200000 for 100MHz, 100000 for 50MHz
/******************************************************************************/
module m_7segcon (w_clk, w_din, r_sg, r_an);
  input  wire w_clk;
  input  wire [31:0] w_din;
  output reg [6:0] r_sg;  // cathode segments
  output reg [7:0] r_an;  // common anode

  reg [31:0] r_val   = 0;
  reg [31:0] r_cnt   = 0;
  reg  [3:0] r_in    = 0;
  reg  [2:0] r_digit = 0;
  always@(posedge w_clk) r_val <= w_din;

  always@(posedge w_clk) begin
    r_cnt <= (r_cnt>=(`DELAY7SEG-1)) ? 0 : r_cnt + 1;
    if(r_cnt==0) begin
      r_digit <= r_digit+ 1;
      if      (r_digit==0) begin r_an <= 8'b11111110; r_in <= r_val[3:0];   end
      else if (r_digit==1) begin r_an <= 8'b11111101; r_in <= r_val[7:4];   end
      else if (r_digit==2) begin r_an <= 8'b11111011; r_in <= r_val[11:8];  end
      else if (r_digit==3) begin r_an <= 8'b11110111; r_in <= r_val[15:12]; end
      else if (r_digit==4) begin r_an <= 8'b11101111; r_in <= r_val[19:16]; end
      else if (r_digit==5) begin r_an <= 8'b11011111; r_in <= r_val[23:20]; end
      else if (r_digit==6) begin r_an <= 8'b10111111; r_in <= r_val[27:24]; end
      else                 begin r_an <= 8'b01111111; r_in <= r_val[31:28]; end
    end
  end
  wire [6:0] w_segments;
  m_7segled m_7segled (r_in, w_segments);
  always@(posedge w_clk) r_sg <= ~w_segments;
endmodule
/******************************************************************************/
