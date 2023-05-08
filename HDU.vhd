library IEEE;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity HDU is 
  port(
    Inst: in std_logic_vector(31 downto 0);
    IDEXMemRead: in std_logic;
    ExReg: in std_logic_vector(4 downto 0);
    ctrl: out std_logic; --Goes into control mux 
    IFIDWrite : out std_logic;
    PCWrite: out std_logic 
  );
end HDU;

architecture HDU of HDU is 
  signal rs1,rs2: std_logic_vector(4 downto 0);
  begin 
    rs1 <= unsigned(instruction(19 downto 15));
    rs2 <= unsigned(instruction(24 downto 20));
    process(rs1,rs2,IDEXMemRead, MemReg) 
      begin 
        if(IDEXMemRead = '1' and (ExReg = rs1 or ExReg = rs2)) then 
          ctrl <= '1';
          IFIDWrite <= '0';
          PCWrite <= '0';
        else then 
          ctrl <= '0';
          IFIDWrite <= '1';
          PCWrite <= '1';
        end if;
    end process;
end architecture; 
 
