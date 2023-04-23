--
-- SPDX-License-Identifier: CERN-OHL-P-2.0+
--
-- Copyright (C) 2021 Embedded and Reconfigurable Computing Lab, American University of Beirut
-- Contributed by:
-- Mazen A. R. Saghir <mazen@aub.edu.lb>
--
-- This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
-- INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR
-- A PARTICULAR PURPOSE. Please see the CERN-OHL-P v2 for applicable
-- conditions.
-- Source location: https://github.com/ERCL-AUB/archer/rv32i_single_cycle
--
-- M extension Arithmetic and Logic Unit (MALU)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.archer_pkg.all;

entity m_alu is
    port (
        inputA : in std_logic_vector (XLEN-1 downto 0);
        inputB : in std_logic_vector (XLEN-1 downto 0);
        ALUop : in std_logic_vector (3 downto 0);
        result : out std_logic_vector (XLEN-1 downto 0)
    );
end m_alu;

architecture rtl of m_alu is
    signal mul_result_temp : std_logic_vector (2*XLEN-1 downto 0);
    signal mul_result : std_logic_vector (XLEN-1 downto 0);
    signal mulh_result : std_logic_vector (XLEN-1 downto 0);
    signal mulhsu_result : std_logic_vector (XLEN-1 downto 0);
    signal mulhsu_result_temp : std_logic_vector (2*XLEN-1 downto 0);
    signal mulhu_result : std_logic_vector (XLEN-1 downto 0);
    signal mulhu_result_temp : std_logic_vector (2*XLEN-1 downto 0);
    signal div_result : std_logic_vector (XLEN-1 downto 0);
    signal divu_result : std_logic_vector (XLEN-1 downto 0);
    signal rem_result : std_logic_vector (XLEN-1 downto 0);
    signal remu_result : std_logic_vector (XLEN-1 downto 0);
begin

    mul_result_temp <= std_logic_vector(signed(inputA) * signed(inputB));
    mul_result <= mul_result_temp(31 downto 0);
    mulh_result <= mul_result_temp(63 downto 32);
    mulhsu_result_temp <= std_logic_vector(signed(inputA) * signed(inputB));
    mulhsu_result <= mulhsu_result_temp(31 downto 0);
    mulhu_result_temp <= std_logic_vector(unsigned(inputA) * unsigned(inputB));
    mulhu_result <= mulhu_result_temp(31 downto 0);
    process(inputA, inputB)
    begin
    if signed(inputB) /= 0 then
	    div_result <= std_logic_vector(signed(inputA) / signed(inputB));
	    divu_result <= std_logic_vector(unsigned(inputA) / unsigned(inputB));
	    rem_result <= std_logic_vector(signed(inputA) rem signed(inputB));
	    remu_result <= std_logic_vector(unsigned(inputA) / unsigned(inputB));
	end if;	    
	end process;

    with ALUop select
    result <=   mul_result when ALU_OP_MUL, -- mul
                mulh_result when ALU_OP_MULH, -- mulh
                mulhsu_result when ALU_OP_MULHSU, -- mulhsu
                mulhu_result when ALU_OP_MULHU, -- mulhu
                div_result when ALU_OP_DIV, -- div
                divu_result when ALU_OP_DIVU, -- divu
                rem_result when ALU_OP_REM, -- rem
                remu_result when ALU_OP_REMU, -- remu
                (others=>'0') when others;

end architecture;
