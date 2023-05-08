--
-- SPDX-License-Identifier: CERN-OHL-P-2.0+
--
-- Copyright (C) 2021 Embedded and Reconfigurable Computing Lab, American University of Beirut
-- Contributed by:
-- Mazen A. R. Saghir <mazen@aub.edu.lb>
--
-- This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
-- INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR
-- A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2 for applicable
-- conditions.
-- Source location: https://github.com/ERCL-AUB/archer/rv32i_single_cycle
--
-- Archer RV32I single-cycle datapath wrapper (top-level entity)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity archer_rv32i_single_cycle is
    port (
        clk : in std_logic;
        rst_n : in std_logic;
        -- local instruction memory bus interface
        imem_addr : out std_logic_vector (ADDRLEN-1 downto 0);
        imem_datain : out std_logic_vector (XLEN-1 downto 0);
        imem_dataout : in std_logic_vector (XLEN-1 downto 0);
        imem_wen : out std_logic; -- write enable signal
        imem_ben : out std_logic_vector (3 downto 0); -- byte enable signals
        -- local data memory bus interface
        dmem_addr : out std_logic_vector (ADDRLEN-1 downto 0);
        dmem_datain : out std_logic_vector (XLEN-1 downto 0);
        dmem_dataout : in std_logic_vector (XLEN-1 downto 0);
        dmem_wen : out std_logic; -- write enable signal
        dmem_ben : out std_logic_vector (3 downto 0) -- byte enable signals
    );
end archer_rv32i_single_cycle;

architecture rtl of archer_rv32i_single_cycle is
    component add4
        port (
            datain : in std_logic_vector (XLEN-1 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component alu
        port (
            inputA : in std_logic_vector (XLEN-1 downto 0);
            inputB : in std_logic_vector (XLEN-1 downto 0);
            ALUop : in std_logic_vector (3 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component m_alu
        port (
            inputA : in std_logic_vector (XLEN-1 downto 0);
            inputB : in std_logic_vector (XLEN-1 downto 0);
            ALUop : in std_logic_vector (3 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component branch_cmp
        port (
            inputA : in std_logic_vector(XLEN-1 downto 0);
            inputB : in std_logic_vector(XLEN-1 downto 0);
            cond : in std_logic_vector(2 downto 0);
            result : out std_logic
        );
    end component;

    component control
        port (
            instruction : in std_logic_vector (XLEN-1 downto 0);
            BranchCond : in std_logic;
            Jump : out std_logic;
            Lui : out std_logic;
            CSR: out std_logic;
            PCSrc : out std_logic;
            RegWrite : out std_logic;
            ALUSrc1 : out std_logic;
            ALUSrc2 : out std_logic;
            ALUSrc3 : out std_logic;
            ALUOp : out std_logic_vector (3 downto 0);
            MemWrite : out std_logic;
            MemRead : out std_logic;
            MemToReg : out std_logic;
            CSR_addr: out std_logic_vector (11 downto 0)
        ) ;
      end component; 

    component lmb
        port (
            proc_addr : in std_logic_vector (XLEN-1 downto 0);
            proc_data_send : in std_logic_vector (XLEN-1 downto 0);
            proc_data_receive : out std_logic_vector (XLEN-1 downto 0);
            proc_byte_mask : in std_logic_vector (1 downto 0); -- "00" = byte; "01" = half-word; "10" = word
            proc_sign_ext_n : in std_logic;
            proc_mem_write : in std_logic;
            proc_mem_read : in std_logic;
            mem_addr : out std_logic_vector (ADDRLEN-1 downto 0);
            mem_datain : out std_logic_vector (XLEN-1 downto 0);
            mem_dataout : in std_logic_vector (XLEN-1 downto 0);
            mem_wen : out std_logic; -- write enable signal
            mem_ben : out std_logic_vector (3 downto 0) -- byte enable signals
        );
    end component;

    component immgen
        port (
            instruction : in std_logic_vector (XLEN-1 downto 0);
            immediate : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component imem
        port (
            address : in std_logic_vector (ADDRLEN-1 downto 0);
            dataout : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component mux2to1
        port (
            sel : in std_logic;
            input0 : in std_logic_vector (XLEN-1 downto 0);
            input1 : in std_logic_vector (XLEN-1 downto 0);
            output : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;
    
    component pc
        port (
            clk : in std_logic;
            rst_n : in std_logic;
            datain : in std_logic_vector(XLEN-1 downto 0);
            dataout : out std_logic_vector(XLEN-1 downto 0)
        );
    end component;

    component regfile
        port (
            clk : in std_logic;
            rst_n : in std_logic;
            RegWrite : in std_logic;
            rs1 : in std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
            rs2 : in std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
            rd : in std_logic_vector (LOG2_XRF_SIZE-1 downto 0);
            datain : in std_logic_vector (XLEN-1 downto 0);
            regA : out std_logic_vector (XLEN-1 downto 0);
            regB : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component CSR_reg is
        port (
            clk: in std_logic;
            rst_n : in std_logic;
            instruction : in std_logic_vector (XLEN-1 downto 0);
            CSR: in std_logic;

            CSR_out: out std_logic_vector (31 downto 0);
            CSR_in: in std_logic_vector (31 downto 0);
            CSR_addr: in std_logic_vector (11 downto 0)
        ) ;
    end component ;

    component IFID is
        port(
            clk: in std_logic;
            rst: in std_logic;
            --IFFlush: in std_logic;
            PCin: in std_logic_vector(XLEN-1 downto 0);
            IMEMin: in std_logic_vector(XLEN-1 downto 0);

            PCout: out std_logic_vector(XLEN-1 downto 0);
            IMEMout: out std_logic_vector(XLEN-1 downto 0)
        );
    end component;

    component IDEX is
        port(
            clk: in std_logic;
            rst: in std_logic;
            JumpIn : in std_logic;
            LuiIn : in std_logic;
            CSRIn: in std_logic;
            PCSrcIn : in std_logic;
            RegWriteIn : in std_logic;
            ALUSrc1In : in std_logic;
            ALUSrc2In : in std_logic;
            ALUSrc3In : in std_logic;
            ALUOpIn : in std_logic_vector (3 downto 0);
            MemWriteIn : in std_logic;
            MemReadIn : in std_logic;
            MemToRegIn : in std_logic;

            JumpOut : out std_logic;
            LuiOut : out std_logic;
            CSROut: out std_logic;
            PCSrcOut: out std_logic;
            RegWriteOut : out std_logic;
            ALUSrc1Out : out std_logic;
            ALUSrc2Out : out std_logic;
            ALUSrc3Out : out std_logic;
            ALUOpOut : out std_logic_vector (3 downto 0);
            MemWriteOut : out std_logic;
            MemReadOut : out std_logic;
            MemToRegOut : out std_logic;

            PCin: in std_logic_vector(XLEN-1 downto 0);
            regAin: in std_logic_vector(XLEN-1 downto 0);
            regBin: in std_logic_vector(XLEN-1 downto 0);
            rdin: in std_logic_vector(XLEN-1 downto 0);
            immgenin: in std_logic_vector(XLEN-1 downto 0);

            immgenout: out std_logic_vector(XLEN-1 downto 0);
            PCOut: out std_logic_vector(XLEN-1 downto 0);
            regAout: out std_logic_vector(XLEN-1 downto 0);
            regBout: out std_logic_vector(XLEN-1 downto 0);
            rdout: out std_logic_vector(XLEN-1 downto 0)        
        );
    end component;

    component EXMEM is
        port(
            clk: in std_logic;
            rst: in std_logic;
            JumpIn : in std_logic;
            LuiIn : in std_logic;
            PCSrcIn : in std_logic;
            RegWriteIn : in std_logic;
            MemWriteIn : in std_logic;
            MemReadIn : in std_logic;
            MemToRegIn : in std_logic;

            JumpOut : out std_logic;
            LuiOut : out std_logic;
            PCSrcOut: out std_logic;
            RegWriteOut : out std_logic;
            MemWriteOut : out std_logic;
            MemReadOut : out std_logic;
            MemToRegOut : out std_logic;

            ALUin: in std_logic_vector(XLEN-1 downto 0);
            regBin: in std_logic_vector(XLEN-1 downto 0);
            rdin: in std_logic_vector(XLEN-1 downto 0);

            ALUout: out std_logic_vector(XLEN-1 downto 0);
            regBout: out std_logic_vector(XLEN-1 downto 0);
            rdout: out std_logic_vector(XLEN-1 downto 0)        
        );
    end component;

    component MEMWB is
        port(
            clk: in std_logic;
            rst: in std_logic;
            JumpIn : in std_logic; --dk
            LuiIn : in std_logic;  --dk
            RegWriteIn : in std_logic; --WB
            MemToRegIn : in std_logic; --WB

            JumpOut : out std_logic;
            LuiOut : out std_logic;
            RegWriteOut : out std_logic;
            MemToRegOut : out std_logic;

            readin: in std_logic_vector(XLEN-1 downto 0);
            rdin: in std_logic_vector(XLEN-1 downto 0);
            ALUin: in std_logic_vector(XLEN-1 downto 0);
            
            ALUout: out std_logic_vector(XLEN-1 downto 0);
            readout: out std_logic_vector(XLEN-1 downto 0);
            rdout: out std_logic_vector(XLEN-1 downto 0)        
        );
    end component;

    component DFU is 
        port(
            EXMEMRegWrite: in std_logic;
            MEMWBRegWrite: in std_logic;
            EXMEMReg: in std_logic_vector(4 downto 0);
            MEMWBReg: in std_logic_vector(4 downto 0);
            RS1: in std_logic_vector(4 downto 0);
            RS2: in std_logic_vector(4 downto 0);

            ForwardA: out std_logic_vector(1 downto 0);
            ForwardB: out std_logic_vector(1 downto 0)
        );
    end component;

    component HDU is 
        port(
            Inst: in std_logic_vector(31 downto 0);
            IDEXMemRead: in std_logic;
            ExReg: in std_logic_vector(4 downto 0);

            ctrl: out std_logic; --Goes into control mux 
            IFIDWrite out std_logic;
            PCWrite: out std_logic 
        );
    end component;

    component mux3to1 is
        port (
            sel : in std_logic_vector(1 downto 0);
            input0 : in std_logic_vector (XLEN-1 downto 0);
            input1 : in std_logic_vector (XLEN-1 downto 0);
            input2 : in std_logic_vector (XLEN-1 downto 0);
            output : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component add is
        port (
            datain1 : in std_logic_vector (XLEN-1 downto 0);
            datain2 : in std_logic_vector (XLEN-1 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component control_mux is
        port (
            Selector : in std_logic;

            JumpIn : in std_logic;
            LuiIn : in std_logic;
            CSRIn: in std_logic;
            PCSrcIn : in std_logic;
            RegWriteIn : in std_logic;
            ALUSrc1In : in std_logic;
            ALUSrc2In : in std_logic;
            ALUSrc3In : in std_logic;
            ALUOpIn : in std_logic_vector (3 downto 0);
            MemWriteIn : in std_logic;
            MemReadIn : in std_logic;
            MemToRegIn : in std_logic; 
            CSR_addrIn: in std_logic_vector (11 downto 0);

            JumpOut : out std_logic;
            LuiOut : out std_logic;
            CSROut: out std_logic;
            PCSrcOut : out std_logic;
            RegWriteOut : out std_logic;
            ALUSrc1Out : out std_logic;
            ALUSrc2Out : out std_logic;
            ALUSrc3Out : out std_logic;
            ALUOpOut : out std_logic_vector (3 downto 0);
            MemWriteOut : out std_logic;
            MemReadOut : out std_logic;
            MemToRegOut : out std_logic; 
            CSR_addrOut : out std_logic_vector (11 downto 0)
        ) ;
    end component;

    -- ID signals
    signal PCin : std_logic_vector (XLEN-1 downto 0);
    signal f_pc_out : std_logic_vector (XLEN-1 downto 0);
    signal c_ifFlush : std_logic;
    signal c_pcout : std_logic_vector (XLEN-1 downto 0);
    signal c_immemout : std_logic_vector (XLEN-1 downto 0);
    signal c_write : std_logic;

    -- EX signals 
    signal e_regAout : std_logic_vector (XLEN-1 downto 0);
    signal e_regBout : std_logic_vector (XLEN-1 downto 0);
    signal e_rdout : std_logic_vector (XLEN-1 downto 0); 

    -- MEM signals 
    signal m_JumpOut : std_logic;
    signal m_LuiOut : std_logic;
    signal m_PCSrcOut : std_logic;
    signal m_RegWriteOut : std_logic;
    signal m_MemWriteOut : std_logic;
    signal m_MemReadOut : std_logic;
    signal m_MemToRegOut : std_logic;
    signal m_ALUout : std_logic_vector (XLEN-1 downto 0);
    signal m_rdout : std_logic_vector (4 downto 0);
    signal m_ALUSrc2out : std_logic_vector (XLEN-1 downto 0);

    -- WB signals 
    signal w_ALUout : std_logic_vector (XLEN-1 downto 0);
    signal w_readout : std_logic_vector (XLEN-1 downto 0);
    signal w_rdout : std_logic_vector (4 downto 0);

    -- pc signals
    signal d_pc_in : std_logic_vector (XLEN-1 downto 0);
    signal d_pc_out : std_logic_vector (XLEN-1 downto 0);

    -- imem signals
    signal d_imem_addr : std_logic_vector (ADDRLEN-1 downto 0);
    signal d_instr_word : std_logic_vector (XLEN-1 downto 0);

    -- add4 signals
    signal d_pcplus4 : std_logic_vector (XLEN-1 downto 0);

    -- control signals
    signal c_branch_out : std_logic;
    signal c_jump : std_logic;
    signal c_lui : std_logic;
    signal c_CSR : std_logic;
    signal c_PCSrc : std_logic;
    signal c_reg_write : std_logic;
    signal c_alu_src1 : std_logic_vector (1 downto 0);
    signal c_alu_src2 : std_logic_vector (1 downto 0);
    signal c_alu_src3 : std_logic;
    signal c_alu_op : std_logic_vector (3 downto 0);
    signal c_mem_write : std_logic;
    signal c_mem_read : std_logic;
    signal c_mem_to_reg : std_logic;
    signal c_CSR_out: std_logic_vector (31 downto 0);
    signal c_CSR_in: std_logic_vector (31 downto 0);
    signal c_CSR_addr: std_logic_vector (11 downto 0);

    -- register file signals

    signal d_reg_file_datain : std_logic_vector (XLEN-1 downto 0);
    signal d_regA : std_logic_vector (XLEN-1 downto 0);
    signal d_regB : std_logic_vector (XLEN-1 downto 0);

    -- immgen signals

    signal d_immediate : std_logic_vector (XLEN-1 downto 0);

    -- lui_mux signals

    signal d_zero : std_logic_vector (XLEN-1 downto 0);
    signal d_lui_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- alu_src1_mux signals 

    signal d_alu_src1 : std_logic_vector (XLEN-1 downto 0);

    -- alu_src2_mux signals 

    signal d_alu_src2 : std_logic_vector (XLEN-1 downto 0);

    -- alu_src3_mux signals 

    signal d_alu_src3 : std_logic_vector (XLEN-1 downto 0);

    -- CSR_mux signals

    signal d_csr_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- CSR_WB_mux signals

    signal d_csr_wb_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- alu signals

    signal d_alu_out : std_logic_vector (XLEN-1 downto 0);
    signal d_mul_alu_out : std_logic_vector (XLEN-1 downto 0);

    -- mem_mux signals

    signal d_mem_mux_out : std_logic_vector (XLEN-1 downto 0);

    -- lmb signals

    signal d_byte_mask : std_logic_vector (1 downto 0);
    signal d_sign_ext_n : std_logic;
    signal d_data_mem_out : std_logic_vector (XLEN-1 downto 0);

    -- instruction word fields

    signal d_rs1 : std_logic_vector (4 downto 0);
    signal d_rs2 : std_logic_vector (4 downto 0);
    signal d_rd : std_logic_vector (4 downto 0);
    signal d_funct3 : std_logic_vector (2 downto 0);
    signal d_funct7 : std_logic_vector (6 downto 0);

    -- add signals
    signal a_1 : std_logic_vector (XLEN-1 downto 0);
    signal a_2 : std_logic_vector (XLEN-1 downto 0);

    -- control_mux signals
    signal    cm_selector : std_logic;
    signal    cm_JumpOut :  std_logic;
    signal    cm_LuiOut :  std_logic;
    signal    cm_CSROut:  std_logic;
    signal    cm_PCSrcOut :  std_logic;
    signal    cm_RegWriteOut :  std_logic;
    signal    cm_ALUSrc1Out :  std_logic;
    signal    cm_ALUSrc2Out :  std_logic;
    signal    cm_ALUSrc3Out :  std_logic;
    signal    cm_ALUOpOut :  std_logic_vector (3 downto 0);
    signal    cm_MemWriteOut :  std_logic;
    signal    cm_MemReadOut :  std_logic;
    signal    cm_MemToRegOut :  std_logic; 
    signal    cm_CSR_addrOut :  std_logic_vector (11 downto 0)
    
    signal    ex_JumpOut :  std_logic;
    signal    ex_LuiOut :  std_logic;
    signal    ex_CSROut:  std_logic;
    signal    ex_PCSrcOut :  std_logic;
    signal    ex_RegWriteOut :  std_logic;
    signal    ex_ALUSrc1Out :  std_logic;
    signal    ex_ALUSrc2Out :  std_logic;
    signal    ex_ALUSrc3Out :  std_logic;
    signal    ex_ALUOpOut :  std_logic_vector (3 downto 0);
    signal    ex_MemWriteOut :  std_logic;
    signal    ex_MemReadOut :  std_logic;
    signal    ex_MemToRegOut :  std_logic; 
    signal    ex_CSR_addrOut :  std_logic_vector (11 downto 0);

    --HDU signals
    signal hd_IFIDWrite : std_logic;
    signal hd_PCWrite : std_logic;


begin

    pc_inst : pc port map (clk => clk, rst_n => rst_n, datain => d_pc_in, dataout => d_pc_out);

    limb_inst : lmb port map (proc_addr => d_pc_out, proc_data_send => (others=>'0'), proc_data_receive => d_instr_word,
                              proc_byte_mask => "10", proc_sign_ext_n => '1', proc_mem_write => '0',
                              proc_mem_read => '1', mem_addr => imem_addr, mem_datain => imem_datain, 
                              mem_dataout => imem_dataout, mem_wen => imem_wen, mem_ben => imem_ben);

    add4_inst : add4 port map (datain => d_pc_out, result => d_pcplus4);
    pc_mux : mux2to1 port map (sel => c_PCSrc, input0 => d_pcplus4, input1 => d_alu_out, output => d_pc_in);
    control_inst : control port map (instruction => d_instr_word, BranchCond => c_branch_out, 
                                    Jump => c_jump, Lui => c_lui, CSR => c_CSR, PCSrc => c_PCSrc, RegWrite => c_reg_write,
                                    ALUSrc1 => c_alu_src1, ALUSrc2 => c_alu_src2, ALUSrc3 => c_alu_src3, ALUOp => c_alu_op, MemWrite => c_mem_write,
                                    MemRead => c_mem_read, MemToReg => c_mem_to_reg, CSR_addr => c_CSR_addr);

    write_back_mux : mux2to1 port map (sel => c_jump, input0 => d_mem_mux_out, input1 => d_pcplus4, output => d_reg_file_datain);

    RF_inst : regfile port map (clk => clk, rst_n => rst_n, RegWrite => c_reg_write, rs1 => d_rs1, rs2 => d_rs2, 
                                rd => d_rd, datain => d_reg_file_datain, regA => d_regA, regB => d_regB);

    brcmp_inst : branch_cmp port map (inputA => d_regA, inputB => d_regB, cond => d_funct3, result => c_branch_out);

    immgen_inst : immgen port map (instruction => d_instr_word, immediate => d_immediate);

    -- operand 1 mux system
    lui_mux : mux2to1 port map (sel => c_lui, input0 => d_pc_out, input1 => d_zero, output => d_lui_mux_out);
    alu_src1_mux : mux3to1 port map (sel => c_alu_src1, input0 => e_regAout, input1 => d_reg_file_datain, input2 => dmem_datain, output => d_alu_src1);

    -- operand 2 mux system
    CSR_out_mux : mux2to1 port map (sel => c_CSR, input0 => d_immediate, input1 => c_CSR_out, output => d_csr_mux_out);
    alu_src2_mux : mux3to1 port map (sel => c_alu_src2, input0 => d_regBout, input1 => d_csr_mux_out, input2 => dmem_datain, output => d_alu_src2);

    alu_inst : alu port map (inputA => d_alu_src1, inputB => d_alu_src2, ALUop => ex_ALUOpOut, result => d_alu_out);
    m_alu_inst : m_alu port map (inputA => d_alu_src1, inputB => d_alu_src2, ALUop => ex_ALUOpOut, result => d_mul_alu_out);

    alu_src3_mux : mux2to1 port map (sel => c_alu_src3, input0 => d_alu_out, input1 => d_mul_alu_out, output => d_alu_src3);

    ldmb_inst : lmb port map (proc_addr => d_alu_out, proc_data_send => d_regB,
                               proc_data_receive => d_data_mem_out, proc_byte_mask => d_byte_mask,
                               proc_sign_ext_n => d_sign_ext_n, proc_mem_write => c_mem_write, proc_mem_read => c_mem_read,
                               mem_addr => dmem_addr, mem_datain => dmem_datain, mem_dataout => dmem_dataout,
                               mem_wen => dmem_wen, mem_ben => dmem_ben);

    CSR_WB_mux : mux2to1 port map (sel => c_CSR, input0 => d_alu_src3, input1 => c_CSR_out, output => d_csr_wb_mux_out);
    mem_mux : mux2to1 port map (sel => c_mem_to_reg, input0 => d_csr_wb_mux_out, input1 => d_data_mem_out, output => d_mem_mux_out);

    CSR_RF: CSR_reg port map (clk => clk, rst_n => rst_n, instruction => d_instr_word, CSR => c_CSR, CSR_out => c_CSR_out, CSR_in => d_alu_out, CSR_addr => c_CSR_addr);

    add : add port map (datain1 => a_1, datain2 => d_immediate, result => d_pc_in);

    control_mux : control_mux port map (Selector => cm_selector,
                                    JumpIn => c_jump, LuiIn => c_lui, CSRIn => c_CSR, PCSrcIn => c_PCSrc, RegWriteIn => c_reg_write,
                                    ALUSrc1In => c_alu_src1, ALUSrc2In => c_alu_src2, ALUSrc3In => c_alu_src3, ALUOpIn => c_alu_op, MemWriteIn => c_mem_write,
                                    MemReadIn => c_mem_read, MemToRegIn => c_mem_to_reg, CSR_addrIn => c_CSR_addr
                                    JumpOut => cm_JumpOut, LuiOut => cm_LuiOut, CSROut => cm_CSROut, PCSrcOut => cm_PCSrcOut, RegWriteOut => cm_RegWriteOut,
                                    ALUSrc1Out => cm_ALUSrc1Out, ALUSrc2Out => cm_ALUSrc2Out, ALUSrc3Out => cm_ALUSrc3Out, ALUOpOut => cm_ALUOpOut, MemWriteOut => cm_MemWriteOut,
                                    MemReadOut => cm_MemReadOut, MemToRegOut => cm_MemToRegOut, CSR_addrOut => cm_CSR_addrOut);

    IFID : IFID port map (clk => clk, rst => rst_n,

                          ifFlush => c_ifFlush, ifidwrite => c_write,

                          PCin => d_pc_out, IMEMin => imem_dataout,

                          PCOut => c_pcout, IMEMOut => c_immemout);

    IDEX : IDEX port map (clk => clk, rst => rst_n,

                          JumpIn => cm_JumpOut, LuiIn => cm_LuiOut, CSRIn => cm_CSROut, PCSrcIn => cm_PCSrcOut, RegWriteIn => cm_RegWriteOut,
                          ALUSrc1 => cm_ALUSrc1Out, ALUSrc2 => cm_ALUSrc2Out, ALUSrc3 => cm_ALUSrc3Out, ALUOpIn => cm_ALUOpOut, MemWriteIn => cm_MemWriteOut, MemReadIn => cm_MemReadOut, MemToRegIn => cm_MemToRegOut,

                          JumpOut => ex_JumpOut, LuiOut => ex_LuiOut, CSROut => ex_CSROut, PCSrcOut => ex_PCSrcOut, RegWriteOut => ex_RegWriteOut, ALUSrc1Out => ex_ALUSrc1Out, ALUSrc2Out => ex_ALUSrc2Out, ALUSrc3Out => ex_ALUSrc3Out, ALUOpOut => ex_ALUOpOut, MemWriteOut => ex_MemWriteOut, MemReadOut => ex_MemReadOut, MemToRegOut => ex_MemToRegOut,

                          regAin => d_regA, regBin => d_regB, rdin => d_rd, rs1in => d_rs1, rs2in =>d_rs2,
                          regAout => e_regAout, regBout => e_regBout, rdout => e_rdout);
    
    EXMEM : EXMEM port map (clk => clk, rst => rst_n,

                          JumpIn => ex_JumpOut,LuiIn => ex_LuiOut, PCSrcIn => ex_PCSrcOut, RegWriteIn => ex_RegWriteOut, MemWriteIn => ex_MemWriteOut, MemReadIn => ex_MemReadOut, MemToRegIn => ex_MemToRegOut,

                          JumpOut => m_JumpOut, LuiOut => m_LuiOut, PCSrcOut => m_PCSrcOut, RegWriteOut => m_RegWriteOut, MemWriteOut => m_MemWriteOut, MemReadOut => m_MemReadOut, MemToRegOut => m_MemToRegOut,

                          ALUin => d_alu_src3, rdin => m_rdin, ALUSrc2in => d_alu_src2,
                          ALUout => dmem_datain, rdout => m_rdout, ALUSrc2out => dmem_addr);
    
    MEMWB : MEMWB port map (clk => clk, rst => rst_n,

                          JumpIn => m_JumpOut, RegWriteIn => m_RegWriteOut, MemToRegIn => m_MemToRegOut,

                          JumpOut => c_jump,RegWriteOut => c_reg_write, MemToRegOut => c_mem_to_reg,

                          readin => dmem_dataout, rdin => m_rdout, ALUin => dmem_datain, 
                          ALUout => w_ALUout, readout => w_readout, rdout => w_rdout);
    
    DFU : DFU port map (EXMEMRegWrite => m_RegWriteOut, MEMWBRegWrite => c_reg_write, EXMEMReg => m_rdout, MEMWBReg => w_rdout,
                         RS1 => d_rs1, RS2 => d_rs2,
                        ForwardA => c_alu_src1, ForwardB => c_alu_src2);
    
    HDU : HDU port map (Inst => c_immemout, IDEXMemRead => ex_MemReadOut, ExReg => e_rdout,
                        ctrl => cm_selector, IFIDWrite => hd_IFIDWrite , PCWrite => hd_PCWrite
    );


    d_rs1 <= c_immemout (LOG2_XRF_SIZE+14 downto 15);
    d_rs2 <= c_immemout (LOG2_XRF_SIZE+19 downto 20);
    d_rd <= c_immemout (LOG2_XRF_SIZE+6 downto 7);
    d_funct3 <= c_immemout (14 downto 12);
    d_funct7 <= c_immemout (31 downto 25);

    d_zero <= (others=>'0');

    d_byte_mask <= d_funct3(1 downto 0);
    d_sign_ext_n <= d_funct3(2);

end architecture;
