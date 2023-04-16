library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity InstructionFetch is
    port(
        clk = in std_logic;
        rst = in std_logic;
        branch_in = in std_logic_vector(XLEN-1 downto 0);
        branch_ctrl = in std_logic
        PC_out = out std_logic_vector(XLEN-1 downto 0);
        imem_out = out std_logic_vector(XLEN-1 downto 0)
    );
architecture InstructionFetch of INstructionFetch is 
    component add4
        port (
            datain : in std_logic_vector (XLEN-1 downto 0);
            result : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component imem
        port (
            address : in std_logic_vector (ADDRLEN-1 downto 0);
            dataout : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component mux2to1
        port (
            sel : in std_logic;
            input0 : in std_logic_vector (XLEN-1 downto 0);
            input1 : in std_logic_vector (XLEN-1 downto 0);
            output : out std_logic_vector (XLEN-1 downto 0)
        );
    end component;

    component pc
        port (
            clk : in std_logic;
            rst_n : in std_logic;
            datain : in std_logic_vector(XLEN-1 downto 0);
            dataout : out std_logic_vector(XLEN-1 downto 0)
        );
    end component;
signal d_branchin, d_pcplus4, d_pcin, d_pcout: std_logic_vector(XLEN-1 downto 0);

begin
    PC: PC port map(clk<=clk, rst_n<=rst, datain<= d_pcin, dataout<=pcout);
    add4: add4 port map(datain <= pc_out, result <= d_pcplus4);
    pcmux: mux2to1 port map(sel<=branch_ctrl, input0<=pcplus4, input1<=branch_in, output<=d_pcin)
    imem: imem port map(address<=pc_out, dataout<=imem_out)
end InstructionFetch;

