library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity InstructionDecode is
    port(
        clk : in std_logic;
        rst : in std_logic;
        RegWrite: in std_logic;
        inst : in std_logic_vector(XLEN-1 downto 0);
        PCout : in std_logic_vector(XLEN-1 downto 0);
        regA : out std_logic_vector(XLEN-1 downto 0);
        regB: out std_logic_vector(XLEN-1 downto 0);
        branchad: out std_logic_vector(XLEN-1 downto 0);
        
    );
end InstructionDecode;

architecture Decode of INstructionDecode is
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

    component immgen
        port (
            instruction : in std_logic_vector (XLEN-1 downto 0);
            immediate : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component add is
        port (
            datain1 : in std_logic_vector (XLEN-1 downto 0);
            datain2 : in std_logic_vector (XLEN-1 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
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

    signal rs1,rs2

    
    
