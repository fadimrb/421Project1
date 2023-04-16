library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity MEMWB is
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
end MEMWB;

architecture EXMEM of EXMEM is
begin
    process(clk,rst) is 
    begin
        if(rst = '0') then 
            JumpOut <= '0';
            LuiOut <= '0';
            MemToRegOut <= '0';
            RegWriteOut <= '0';
            readout<= (others => '0');
            ALUout <= (others => '0');
            rdout<= (others => '0');  
        elsif rising_edge(clk) then 
            JumpOut <= JumpIn;
            LuiOut <= LuiIn;
            RegWriteOut <= RegWriteIn;
            MemToRegOut <= MemToRegIn;
            ALUout,<= ALUin;
            readout<= readin;
            rdout<= rdin;  
        else then 
            JumpOut <= '0';
            LuiOut <= '0';
            MemToRegOut <= '0';
            RegWriteOut <= '0';
            readout<= (others => '0');
            ALUout <= (others => '0');
            rdout<= (others => '0');  
        end if;
    end process;
end architecture;
