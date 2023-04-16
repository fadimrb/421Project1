library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity HFU is
    port (
        inst: in std_logic_vector(XLEN-1 downto 0);
        IDEXout: in std_logic_vector(XLEN-1 downto 0);
        Min: in std_logic;
        PCWrite: out std_logic;
        IFIDWrite: out std_logic;
    );
end HFU;
