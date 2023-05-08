library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity IDEX is
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
        regAin: in std_logic_vector(XLEN-1 downto 0);
        regBin: in std_logic_vector(XLEN-1 downto 0);
        rdin: in std_logic_vector(XLEN-1 downto 0);
        rs1in : in std_logic_vector (4 downto 0);
        rs2in : in std_logic_vector (4 downto 0);
        idexregIn: in std_logic_vector (4 downto 0);
        regAout: out std_logic_vector(XLEN-1 downto 0);
        regBout: out std_logic_vector(XLEN-1 downto 0);
        idexregOut: out std_logic_vector (4 downto 0);
        rdout: out std_logic_vector(XLEN-1 downto 0)        
    );
end IDEX;

architecture IDEX of IDEX is
begin
    process(clk,rst) is 
    begin
        if(rst = '0') then 
            
            JumpOut <= '0';
            LuiOut <= '0';
            CSROut<= '0';
            PCSrcOut<= '0';
            RegWriteOut <= '0';
            ALUSrc1Out <= ('0';
            ALUSrc2Out <= '0';
            ALUSrc3Out <= '0';
            ALUOpOut <= (others => '0');
            MemWriteOut <= '0';
            MemReadOut <= '0';
            MemToRegOut <= '0';
            PCOut<= (others => '0');
            regAout<= (others => '0');
            regBout<= (others => '0');
            rdout<= (others => '0');  
        elsif rising_edge(clk) then 
           
            JumpOut <= JumpIn;
            LuiOut <= LuiIn;
            CSROut<= CSRin;
            PCSrcOut<= PCSrcIn;
            RegWriteOut <= RegWriteIn;
            ALUSrc1Out <= ALUSRC1In;
            ALUSrc2Out <= ALUSRC2In;
            ALUSrc3Out <= ALUSRC3In;
            ALUOpOut <= ALUOpIn;
            MemWriteOut <= MemWriteIn;
            MemReadOut <= MemReadIn;
            MemToRegOut <= MemToRegIn;
            PCOut<= PCIn;
            regAout<= regAin;
            regBout<= regBin;
            rdout<= rdin;  
        end if;
    end process;
end architecture;
