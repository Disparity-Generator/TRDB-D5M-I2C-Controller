library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use std.textio.all;
  use ieee.std_logic_textio.all;

entity tb_kamera_settings_controller is -- keine Schnittstellen
end entity tb_kamera_settings_controller;

architecture arch of tb_kamera_settings_controller is

  signal tb_we_s             : std_logic;
  signal tb_start_n_s        : std_logic;
  signal tb_error_s          : std_logic;
  signal tb_reset_n_s        : std_logic;
  signal tb_address_s        : std_logic_vector(7 downto 0);
  signal tb_data_in_s        : std_logic_vector(15 downto 0);
  signal tb_data_out_s       : std_logic_vector(15 downto 0);
  signal tb_data_out_valid_s : std_logic;
  signal tb_sdata_s          : std_logic;
  signal sclk_s              : std_logic;
  signal clk_s               : std_logic;

  signal tb_ack_we_s : std_logic := '0';
  signal tb_ack_high_s : std_logic;

  signal tb_ready_s : std_logic;

  component kamera_i2c_controller is
    port (
      i_clock   : in    std_logic; --! input clock used as base to divide
      i_reset_n : in    std_logic; --! async, low active reset
  
      io_sdata : inout std_logic; --! data wired directly to the i2c port
      o_sclk   : out   std_logic; --! clock wired directly to the i2c port
  
      i_we      : in    std_logic;                     --! decides if the next command is a read or write
      i_start_n : in    std_logic;                     --! signal to start state machine (the next command)
      i_address : in    std_logic_vector(7 downto 0);  --! target address for the next read or write data
      i_data    : in    std_logic_vector(15 downto 0); --! data to write if the next command is a write
  
      o_q     : out   std_logic_vector(15 downto 0); --! output data if the last command was a read
      o_error : out   std_logic;                     --! indicates if the command stopped unexpectedly (not used atm)
      o_valid : out   std_logic;                     --! indicates if the output from read command is valid
      o_ready : out   std_logic                      --! indicates if the i2c is ready for the next command
    );
  end component kamera_i2c_controller;

begin

  dut : component kamera_i2c_controller
    port map (
      i_clock => clk_s,
      i_reset_n  => tb_reset_n_s,
      
      io_sdata    => tb_sdata_s,
      o_sclk     => sclk_s,
      
      i_we       => tb_we_s,
      i_start_n  => tb_start_n_s,
      i_address  => tb_address_s,
      i_data  => tb_data_in_s,

      o_q => tb_data_out_s,
      o_error    => tb_error_s,
      o_ready => tb_ready_s
    );

  -- Schreibtest

  p_clk : process is

  begin

    clk_s <= '1';
    wait for 625 ns;
    clk_s <= '0';
    wait for 625 ns;

  end process p_clk;

  tests_p : process is
  begin

    -- tb_reset_n_s <= '1' after 0 ns, '0' after 10 ns;
    tb_reset_n_s <= '1';
    tb_data_in_s <= "1010101010101010";
    tb_address_s <= "11001100";
    tb_we_s      <= '1';
    tb_start_n_s <= '0';

    wait for 5000 ns;

    tb_start_n_s <= '1';

    wait for 190000 ns;
    tb_address_s <= "11100011";
    tb_we_s      <= '0';
    tb_start_n_s <= '0';

    wait for 11000 ns;
    tb_start_n_s <= '1';

    wait;

  end process tests_p;

  -- acknowledges generieren
  tb_ack_high_s <= '0', '1' after 289600 ns, '1' after 289800 ns, '0' after 295130 ns, '1' after 300990 ns, '0' after 305980 ns, '1' after 310970 ns, '0' after 316000 ns, '1' after 320980 ns, '0' after 326040 ns;

 -- tb_ack_we_s <= '0', '1' after 48000 ns, '0' after 52940 ns, '1' after 93430 ns, '0' after 98450 ns, '1' after 194350 ns, '0' after 199440 ns, '1' after 239600 ns, '0' after 244490 ns,
  -- daten zum Lesen generieren
  --'1' after 289820 ns;

  --tb_sdata_s <= '0' when tb_ack_we_s = '1' and tb_ack_high_s = '0' else
   --             '1' when tb_ack_we_s = '1' and tb_ack_high_s = '1' else
   --             'Z';

end architecture arch;

-- 340000 ns