library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity IFID is
    port(
        clk: in std_logic;
        rst: in std_logic;
        PCin: in std_logic_vector(XLEN-1 downto 0);
        IMEMin: in std_logic_vector(XLEN-1 downto 0);
        PCout: out std_logic_vector(XLEN-1 downto 0);
        IMEMout: out std_logic_vector(XLEN-1 downto 0)
    );
end IFID;

architecture IFID of IFID is
begin
    process(clk,rst) is 
    begin
        if(rst = '0') then 
            PCOut <= (others => '0');
            IMEMout <= (others => '0');
        elsif rising_edge(clk) then 
            PCout <= PCin;
            IMEMout <= IMEMin;
        else then 
            PCOut <= (others => '0');
            IMEMout <= (others => '0');
        end if;
    end process;
end architecture;
