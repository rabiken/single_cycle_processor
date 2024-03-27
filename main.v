`default_nettype none
module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
    //... write your code here ...
    wire BranchBeq;
    wire BranchBlt;
    wire BranchJal;
    wire RegWrite;
    wire BranchJalr;
    wire MemToReg;
    wire MemWrite;
    wire [3:0] ALUControl;
    wire ALUSrc;
    wire [2:0] immControl;
    wire luiSignal;
    wire auipcSignal;

    control_unit control_unit_inst (   .instruction ( instruction ), 
                                       .BranchBeq ( BranchBeq ), 
                                       .BranchBlt ( BranchBlt ), 
                                       .BranchJal ( BranchJal ), 
                                       .RegWrite ( RegWrite ), 
                                       .BranchJalr ( BranchJalr ), 
                                       .MemToReg ( MemToReg ), 
                                       .MemWrite ( MemWrite ), 
                                       .ALUControl ( ALUControl ), 
                                       .ALUSrc ( ALUSrc ), 
                                       .immControl ( immControl ),
                                       .luiSignal ( luiSignal ), 
                                       .auipcSignal ( auipcSignal )
                        ); 
    data_path  data_path_inst   (      .clk ( clk ), 
                                       .reset ( reset ), 
                                       .instruction ( instruction ), 
                                       .data_from_mem ( data_from_mem ), 
                                       .PC ( PC ), 
                                       .address_to_mem ( address_to_mem ),
                                       .data_to_mem ( data_to_mem ),
                                       .BranchBeq ( BranchBeq ), 
                                       .BranchBlt ( BranchBlt ), 
                                       .BranchJal ( BranchJal ), 
                                       .RegWrite ( RegWrite ), 
                                       .BranchJalr ( BranchJalr ), 
                                       .MemToReg ( MemToReg ), 
                                       .MemWrite ( MemWrite ), 
                                       .ALUControl ( ALUControl ), 
                                       .ALUSrc ( ALUSrc ), 
                                       .immControl ( immControl ),
                                       .luiSignal ( luiSignal ), 
                                       .auipcSignal ( auipcSignal )
                                ); 
    assign WE = MemWrite;
endmodule

//... add new Verilog modules here ...

module control_unit (   input   [31:0]   instruction, 
                        output           BranchBeq, 
                        output           BranchBlt, 
                        output           BranchJal, 
                        output           RegWrite, 
                        output           BranchJalr, 
                        output           MemToReg, 
                        output           MemWrite, 
                        output   [3:0]   ALUControl, 
                        output           ALUSrc, 
                        output   [2:0]   immControl,
                        output           luiSignal, 
                        output           auipcSignal
                    ); 
    reg [6:0] opcode;
    reg [2:0] funct3;
    reg [6:0] funct7;

    reg BranchBeq_res ;
    reg BranchBlt_res ;
    reg BranchJal_res ;
    reg RegWrite_res ;
    reg BranchJalr_res ;
    reg MemToReg_res ;
    reg MemWrite_res ;
    reg [3:0] ALUControl_res ;
    reg ALUSrc_res ;
    reg [2:0] immControl_res;
    reg luiSignal_res ;
    reg auipcSignal_res;

    always @(*) begin
        // initialization
        opcode = instruction[6:0];
        funct3 = instruction[14:12];
        funct7 = instruction[31:25];

        BranchBeq_res = 0;
        BranchBlt_res = 0;
        BranchJal_res = 0;
        RegWrite_res = 0;
        BranchJalr_res = 0;
        MemToReg_res = 0;
        MemWrite_res = 0;
        ALUControl_res = 0;
        ALUSrc_res = 0;
        immControl_res= 0;
        luiSignal_res = 0;
        auipcSignal_res= 0;


        case ( opcode )
            // R-type
            7'b0110011: begin
                RegWrite_res = 1;
                case (funct3)
                    // addsub
                    3'b000: begin
                        case (funct7)
                            // add 
                            7'b0000000: begin
                                ALUControl_res = 0;
                            end

                            //sub
                            7'b0100000: begin
                                ALUControl_res = 1;
                            end
                        endcase
                    end
                    // and
                    3'b111: begin
                        ALUControl_res = 2;
                    end

                    // slt
                    3'b010: begin
                        ALUControl_res = 3;
                    end

                    // div
                    3'b100: begin
                        ALUControl_res = 4;
                    end

                    //rem
                    3'b110: begin
                        ALUControl_res = 5;
                    end

                    // sll
                    3'b001: begin
                        ALUControl_res = 6;
                    end

                    // srl or sra
                    3'b101: begin
                        case ( funct7 ) 
                            // srl
                            7'b0000000: begin
                                ALUControl_res = 7;
                            end
                            // sra
                            7'b0100000: begin
                                ALUControl_res = 8;
                            end
                        endcase
                    end

                endcase
            end
            // I-type: ALU-imm (addi)
            7'b0010011: begin
                immControl_res = 0;
                ALUSrc_res = 1;
                ALUControl_res = 0; // addition
                RegWrite_res = 1;
            end

            // B-type: Brunch
            7'b1100011: begin
                immControl_res = 2;
                ALUSrc_res = 0; // should be zero!!!!!
                case ( funct3 ) 
                    // beq
                    3'b000: begin
                        BranchBeq_res = 1;
                        // BranchBlt_res = 0;
                        ALUControl_res = 1; // subtraction
                    end

                    // blt
                    3'b100: begin
                        // BranchBeq_res = 0;
                        BranchBlt_res = 1;
                        ALUControl_res = 3; // slt
                    end
                endcase
            end
            
            // I-type: Memory load  (lw)
            7'b0000011: begin
                immControl_res = 0;
                ALUSrc_res = 1;
                MemToReg_res = 1;
                RegWrite_res = 1;
            end
            
            // S-type: Memory store (sw)
            7'b0100011: begin
                immControl_res = 1;
                ALUSrc_res = 1;
                ALUControl_res = 0;  // addition
                MemWrite_res = 1;
            end

            // undef lui    U-type
            7'b0110111: begin
                immControl_res = 3;
                RegWrite_res = 1;
                // luiSignal!!!!!
                luiSignal_res = 1;
            end
            
            // J-type: jal
            7'b1101111: begin
                immControl_res = 4;
                RegWrite_res = 1;
                BranchJal_res = 1;
            end

            // I-type: jalr // jalr is NOT J-type!!!! jalr is I-type!!!
            7'b1100111: begin
                immControl_res = 0;
                ALUSrc_res = 1;
                ALUControl_res = 0; // addition
                RegWrite_res = 1;
                BranchJalr_res = 1;
            end

            // undef auipc      U-type
            7'b0010111: begin
                immControl_res = 3;
                RegWrite_res = 1;
                // auipcSignal!!!!
                auipcSignal_res = 1;
            end

        endcase
    end

    assign BranchBeq = BranchBeq_res ;
    assign BranchBlt = BranchBlt_res ;
    assign BranchJal = BranchJal_res ;
    assign RegWrite = RegWrite_res ;
    assign BranchJalr = BranchJalr_res ;
    assign MemToReg = MemToReg_res ;
    assign MemWrite = MemWrite_res ;
    assign ALUControl = ALUControl_res ;
    assign ALUSrc = ALUSrc_res ;
    assign immControl= immControl_res;
    assign luiSignal = luiSignal_res ;
    assign auipcSignal= auipcSignal_res;

endmodule

module data_path    (   input           clk, reset, 
                        input  [31:0]   instruction, 
                        input  [31:0]   data_from_mem, 
                        output [31:0]   PC, 
                        output [31:0]   address_to_mem,
                        output [31:0]   data_to_mem,
                        input           BranchBeq, 
                        input           BranchBlt, 
                        input           BranchJal, 
                        input           RegWrite, 
                        input           BranchJalr, 
                        input           MemToReg, 
                        input           MemWrite, 
                        input   [3:0]   ALUControl, 
                        input           ALUSrc, 
                        input   [2:0]   immControl,
                        input           luiSignal, 
                        input           auipcSignal 
                    ); 
    wire [31:0] data_to_reg,
                rs1, 
                rs2;
    gpr_set gpr_set_inst    (   .clk ( clk ), 
                                .we3 ( RegWrite ), 
                                .a1 ( instruction[19:15] ),
                                .a2 ( instruction[24:20] ),
                                .a3 ( instruction[11:7]  ),
                                .wd3 ( data_to_reg ), 
                                .rd1 ( rs1 ), 
                                .rd2 ( rs2 ) 
                            );

    wire [31:0] ImmOp;
    imm_decode imm_decode_inst  (   .imm_ctrl ( immControl ),   // immediate control
                                    .inst ( instruction ),  // imm from instruction
                                    .imm_out ( ImmOp ) 
                                );

    wire [31:0] SrcA, SrcB;
    assign SrcA = rs1;
    assign SrcB = ALUSrc ? ImmOp : rs2  ;
    wire [31:0] ALUOut;
    wire zeroFlag, lessThanFlag;

    alu alu_inst    (   .alu_ctrl ( ALUControl ),
                        .src_a ( SrcA ), 
                        .src_b ( SrcB ), 
                        .alu_out ( ALUOut ), 
                        .zero ( zeroFlag ), 
                        .lessThan ( lessThanFlag )
                    );

    wire [31:0] ImmPcSum;
    assign ImmPcSum = $signed(ImmOp) + PC;      // should ImmOp be signed?

    wire [31:0] BranchTarget;
    assign BranchTarget = BranchJalr ? ALUOut : ImmPcSum ;

    wire BranchJalx;
    assign BranchJalx = BranchJal | BranchJalr  ;

    wire [31:0] PCPlus4;
    assign PCPlus4 = PC + 4;

    wire [31:0] ALUOut_PCPlus4_res;
    assign ALUOut_PCPlus4_res = BranchJalx ? PCPlus4 : ALUOut;

    wire [31:0] res;
    assign res = MemToReg ? data_from_mem : ALUOut_PCPlus4_res  ;
    reg [31:0] data_to_reg_res;
    always @ ( * ) begin
        data_to_reg_res = luiSignal ? ImmOp : auipcSignal ? ImmPcSum : res ;
    end
    assign data_to_reg = data_to_reg_res;


    wire BranchBeq_res = BranchBeq & zeroFlag ;
    wire BranchBlt_res = BranchBlt & lessThanFlag ;

    
    wire BranchOutcome;
    assign BranchOutcome = BranchJalx | BranchBeq_res | BranchBlt_res ;

    wire [31:0] PCn;
    assign PCn = BranchOutcome ? BranchTarget : PCPlus4 ;

    register register_inst  (   .data_in ( PCn ), 
                                .clk ( clk ),
                                .reset ( reset ),
                                .data_out ( PC ) 
                            );

    assign address_to_mem = ALUOut;    // critical mistake!!!
    assign data_to_mem = rs2;
endmodule

module gpr_set (    input clk, we3, 
                    input [4:0] a1, a2, a3, 
                    input [31:0] wd3, 
                    output [31:0] rd1, rd2 
                );
    reg [31:0] registers [31:0];

    always @(posedge clk) begin
        registers[0] = 0;
        if ( we3 ) begin
            registers[a3] = wd3;
        end
        registers[0] = 0;
    end

    assign rd1 = registers[a1];
    assign rd2 = registers[a2];

endmodule

module imm_decode   (   input [2:0] imm_ctrl,   // immediate control
                        input [31:0] inst,  // imm from instruction
                        output [31:0] imm_out 
                    );
    reg [31:0] res;
    reg zero;
    always @(*) begin
        res = 0;
        zero = 0;
        case ( imm_ctrl ) 
            0: begin
                // I-immediate
                res = {{21{inst[31]}}, inst[30:20] };
            end
            1: begin
                // S-immediate
                res = {{21{inst[31]}}, inst[30:25], inst[11:8], inst[7] };
            end 
            2: begin
                // B-immediate
                res = {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], zero };
            end
            3: begin
                // U-immediate
                res = {inst[31], inst[30:20], inst[19:12], {12{zero}} };
            end
            4: begin
                // J-immediate
                res = {{12{inst[31]}}, inst[19:12], inst[20], inst[30:21], zero };
            end
            // 5: begin
            //     // for lui and auipc
            //     res = {inst[31:12], {20{zero}}};
            // end
            default: begin
                res = 0;
            end
        endcase
    end
    
    assign imm_out = res;
    
endmodule

module alu  (   input [3:0] alu_ctrl,
                input [31:0] src_a, src_b, 
                output [31:0] alu_out, 
                output zero, 
                output lessThan
            );
    reg [31:0] res;
    reg zero_res;
    reg lessThan_res;
    always @(*) begin
        zero_res = 0;
        lessThan_res = 0;
        case ( alu_ctrl ) 
            0:  begin
                // addition
                res = $signed(src_a) + $signed(src_b);
            end
            1: begin
                // subtraction
                res = $signed(src_a) - $signed(src_b);
            end
            2: begin
                // and 
                res = src_a & src_b;
            end
            3: begin
                // slt
                res = ($signed(src_a)) < ($signed(src_b)) ? 1 : 0;
                lessThan_res = res;
            end
            4: begin
                // div
                res = $signed(src_a) / $signed(src_b);
            end
            5: begin
                // rem
                res = $signed(src_a) % $signed(src_b);
            end
            6: begin
                // sll
                res = src_a << src_b;
            end
            7: begin
                // srl
                res = $unsigned(src_a) >> src_b;
            end
            8: begin
                // sra
                res = $signed(src_a) >>> src_b; // maybe wrong? just >> ?
            end             
            default:  begin
                res = 0;
            end
        endcase
        zero_res = res == 0 ? 1 : 0;
    end
    assign alu_out = res;
    assign zero = zero_res;
    assign lessThan = lessThan_res;
endmodule

module register (   input [31:0] data_in, 
                    input clk,
                    input reset,
                    output [31:0] data_out 
                );
    reg [31:0] rgst;
    assign data_out = rgst;

    always @(posedge clk, posedge reset ) begin
        if ( reset ) begin
            rgst = 0;
        end
        else begin
            rgst = data_in;
        end
    end
endmodule

`default_nettype wire