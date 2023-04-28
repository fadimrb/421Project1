library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity mux3to1 is
    port (
        sel : in std_logic_vector(1 downto 0);
        input0 : in std_logic_vector (XLEN-1 downto 0);
        input1 : in std_logic_vector (XLEN-1 downto 0);
        input2 : in std_logic_vector (XLEN-1 downto 0);
        output : out std_logic_vector (XLEN-1 downto 0)
    );
end mux3to1;

architecture rtl of mux3to1 is
begin
    output <=   input0 when sel = "00" else 
                input1 when sel = "01" else
                input2 when sel = "10" else
                (others=>'0');
end architecture;
