library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity DFU is 
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
end DFU;

architecture DFU of DFU is 
  begin 
    process(EXMEMRegWrite, MEMWBRegWrite, EXMEMReg, MEMWBReg)
      begin
        if(EXMEMRegWrite = '0' and EXMEMReg /= "00000" and EXMEMReg = Rs1) then 
          ForwardA <= "10";
        elsif (MEMWBRegWrite = '0' and MEMWBReg /= "00000" and EXMEMReg /= Rs1 and MEMWBReg = Rs1) then 
          ForwardA <= "01";
        else ForwardA <= "00";
        end if;
        if(EXMEMRegWrite = '0' and EXMEMReg /= "00000" and EXMEMReg = Rs2) then 
          ForwardB <= "10";
        elsif (MEMWBRegWrite = '0' and MEMWBReg /= "00000" and EXMEMReg /= Rs2 and MEMWBReg = Rs2) then
          ForwardB <= "01";
        else ForwardB <= "00";
        end if;
    end process;
end architecture;
