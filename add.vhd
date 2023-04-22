library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity add is
    port (
        datain1 : in std_logic_vector (XLEN-1 downto 0);
        datain2 : in std_logic_vector (XLEN-1 downto 0);
        result : out std_logic_vector (XLEN-1 downto 0)
    );
end add;

architecture rtl of add is
begin
    result <= std_logic_vector(unsigned(datain1) + unsigned(datain2));
end architecture;
