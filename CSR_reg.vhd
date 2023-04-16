library ieee ;
use ieee.std_logic_1164.all ;
use ieee.numeric_std.all ;
use work.archer_pkg.all;

entity CSR_reg is
  port (
    clk: in std_logic;
    rst_n : in std_logic;
    instruction : in std_logic_vector (XLEN-1 downto 0);
    CSR: in std_logic;

    CSR_out: out std_logic_vector (31 downto 0);
    CSR_in: in std_logic_vector (31 downto 0);
    CSR_addr: in std_logic_vector (11 downto 0)
  ) ;
end CSR_reg ;

architecture arch of CSR_reg is

    type CSR_regs_type is record
      RDCYCLE : std_logic_vector(63 downto 0);
      RDTIME : std_logic_vector(63 downto 0);
      RDINSTRET : std_logic_vector(63 downto 0);
    end record;

    
    signal CSR_regs: CSR_regs_type := (others => (others => '1'));

begin

    process (rst_n, csr_addr) is
    begin
        if rst_n = '0' then
            CSR_out <= (others => '0');
        else
            case csr_addr is
                when CSR_RDCYCLE => CSR_out <= CSR_regs.RDCYCLE(31 downto 0);
                when CSR_RDCYCLEH => CSR_out <= CSR_regs.RDCYCLE(63 downto 32);
                when CSR_RDTIME => CSR_out <= CSR_regs.RDTIME(31 downto 0); 
                when CSR_RDTIMEH => CSR_out <= CSR_regs.RDTIME(63 downto 32);
                when CSR_RDINSTRET => CSR_out <= CSR_regs.RDINSTRET(31 downto 0);
                when CSR_RDINSTRETH => CSR_out <= CSR_regs.RDINSTRET(63 downto 32);
                when others => CSR_out <= (others => '0');
            end case;
        end if;
    end process;


    process(rst_n, clk, CSR_in, instruction)
    variable old_inst: std_logic_vector (XLEN-1 downto 0);
    begin
    if rst_n = '0' then
        CSR_regs <= (others => (others => '1'));
    elsif (old_inst /= instruction) then -- only increment when new instruction is sent
        old_inst := instruction;
        CSR_regs.RDINSTRET <= std_logic_vector(unsigned(CSR_regs.RDINSTRET) + 1);
    elsif(rising_edge(clk)) then
        CSR_regs.RDCYCLE <= std_logic_vector(unsigned(CSR_regs.RDCYCLE) + 1);
    elsif (CSR = '1') then
        case csr_addr is
            when CSR_RDCYCLE => null;
            when CSR_RDCYCLEH => null;
            when CSR_RDTIME => null;
            when CSR_RDTIMEH => null;
            when CSR_RDINSTRET => null;
            when CSR_RDINSTRETH => null;
            when others => null;
        end case;
    end if;

    
  end process;

end architecture;