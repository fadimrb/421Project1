library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity EXMEM is
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
end EXMEM;

architecture EXMEM of EXMEM is
begin
    process(clk,rst) is 
    begin
        if(rst = '0') then 
            JumpOut <= '0';
            LuiOut <= '0';
            PCSrcOut<= '0';
            RegWriteOut <= '0';
            MemWriteOut <= '0';
            MemReadOut <= '0';
            MemToRegOut <= '0';
            regBout<= (others => '0');
            ALUout <= (others => '0');
            rdout<= (others => '0');  
        elsif rising_edge(clk) then 
            JumpOut <= JumpIn;
            LuiOut <= LuiIn;
            PCSrcOut<= PCSrcIn;
            RegWriteOut <= RegWriteIn;
            MemWriteOut <= MemWriteIn;
            MemReadOut <= MemReadIn;
            MemToRegOut <= MemToRegIn;
            ALUout,<= ALUin;
            regBout<= regBin;
            rdout<= rdin;  
        else then 
            JumpOut <= '0';
            LuiOut <= '0';
            PCSrcOut<= '0';
            RegWriteOut <= '0';
            MemWriteOut <= '0';
            MemReadOut <= '0';
            MemToRegOut <= '0';
            regBout<= (others => '0');
            ALUout <= (others => '0');
            rdout<= (others => '0'); 
        end if;
    end process;
end architecture;
