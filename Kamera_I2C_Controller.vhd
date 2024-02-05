library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.std_logic_arith.all;

entity KAMERA_I2C_CONTROLLER is
  port (
    I_CLOCK   : in    std_logic;                     --! input clock used as base to divide
    I_RESET_N : in    std_logic;                     --! async, low active reset

    IO_SDATA  : inout std_logic;                     --! data wired directly to the i2c port
    O_SCLK    : out   std_logic;                     --! clock wired directly to the i2c port

    I_WE      : in    std_logic;                     --! decides if the next command is a read or write
    I_START_N : in    std_logic;                     --! signal to start state machine (the next command)
    I_ADDRESS : in    std_logic_vector(7 downto 0);  --! target address for the next read or write data
    I_DATA    : in    std_logic_vector(15 downto 0); --! data to write if the next command is a write

    O_Q       : out   std_logic_vector(15 downto 0); --! output data if the last command was a read
    O_ERROR   : out   std_logic;                     --! indicates if the command stopped unexpectedly (not used atm)
    O_VALID   : out   std_logic;                     --! indicates if the output from read command is valid
    O_ACK     : out   std_logic;
    O_READY   : out   std_logic                      --! indicates if the i2c is ready for the next command
  );
end entity KAMERA_I2C_CONTROLLER;

-- The I2C bus of the TRBD-5M works as follows:

-- Start and stop sequence are according to the I2C standard.

-- Despite having many registers, we have only two slave addresses: one for writes and one for reads.
-- The register to read or write is transmitted as the first TWO data bytes
-- Depending on if we want to write or read, the next TWO data bytes are eiter written to io_sdata
-- or read from the data bus.
-- EACH byte is acknowledged or needs to get acknowledged, thus although we don't check the acknowledge,
-- we have to wait one extra cycle after each outgoing transmission and send an ack after each incoming transmission.

architecture ARCH of KAMERA_I2C_CONTROLLER is

  constant c_slave_read_address  : std_logic_vector(7 downto 0) := x"BB";            -- slave address to start a read
  constant c_slave_write_address : std_logic_vector(7 downto 0) := x"BA";            -- slave address to start a write

  type t_states is (
    IDLE, START_SEQUENCE, ACTIVATE_SLAVE, WAIT_FOR_SLAVE_ACK, SELECT_REGISTER,
    WAIT_FOR_REGISTER_ACK, SEND_DATA1, WAIT_FOR_SEND_DATA1_ACK, SEND_DATA2,
    WAIT_FOR_SEND_DATA2_ACK, STOP_TRANSMISSION, START_ACTIVATE_READ_SLAVE, ACTIVATE_READ_SLAVE, WAIT_FOR_READ_SLAVE_ACK,
    RECV_DATA1, SEND_RECV_DATA1_ACK, RECV_DATA2, SEND_RECV_DATA2_ACK, ERROR
  );

  signal r_state                 : t_states := idle;
  signal w_next_state            : t_states;

  signal r_clk_divided           : std_logic := '1';                                 --! output of divided clock
  signal r_clk_valid             : std_logic := '0';                                 --! trigger for divided clock

  signal w_half_high_flag        : std_logic;                                        --! inidcates the middle of a divided clock high pulse
  signal w_half_low_flag         : std_logic;                                        --! inidcates the middle of a divided clock low pulse

  signal w_error                 : std_logic;                                        -- indicates if the slave returned an error at some point
  signal w_valid                 : std_logic;                                        -- inidcates if the current shift register values are valid
  signal w_ready                 : std_logic;                                        -- inidcates if the i2c master is ready for a transmission
  signal w_sdata                 : std_logic;                                        -- wire to the data port
  signal w_sclk                  : std_logic;                                        -- wire to the clock port

  signal r_clock_count           : integer range 0 to 3          := 0;               --! counter for clock divider
  signal r_shift_out             : std_logic_vector(15 downto 0) := (others => '0'); --! shift register to store read data
  signal r_count                 : integer range 0 to 7          := 7;               --! counter to keep track of the current transmitted bit
  signal r_we                    : std_logic                     := '0';             --! locks the write_enable input signal before the start of command
  signal r_address               : std_logic_vector(7 downto 0)  := (others => '0'); -- holds the target register address of the current transmission
  signal r_data                  : std_logic_vector(15 downto 0) := (others => '0'); -- holds the input data of the current transmission

  signal r_ack_slave             : std_logic := '0';                                 -- holds the acknowldge of the slave address transmission
  signal r_ack_address           : std_logic := '0';                                 -- holds the acknowledge of the address transmission
  signal r_ack_data_1            : std_logic := '0';                                 -- holds the acknowledge of the first sent byte
  signal r_ack_data_2            : std_logic := '0';                                 -- holds the acknowledge of the second sent byte

begin

  -------------------
  -- State machine --
  -------------------

  -- State machine output and next state generator.
  PROC_OUTPUT : process (I_START_N, r_address, r_we, r_data, r_state, r_clk_divided, r_count) is
  begin

    -- Preset clock if not in start or stop sequence.
    w_sclk <= not r_clk_divided;

    case r_state is

      ----------- general path ------------
      when IDLE =>

        if (I_START_N = '1') then
          w_next_state <= IDLE;
        else
          w_next_state <= START_SEQUENCE;
        end if;

        w_sdata <= '1';
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '1';

      when START_SEQUENCE =>

        w_next_state <= ACTIVATE_SLAVE;

        if (r_count = 7) then
          w_next_state <= START_SEQUENCE;
          w_sdata      <= '1';
          w_sclk       <= '1';
        elsif (r_count = 6) then
          w_next_state <= START_SEQUENCE;
          w_sdata      <= '0';
          w_sclk       <= '1';
        else
          w_next_state <= ACTIVATE_SLAVE;
          w_sdata      <= c_slave_write_address(r_count);
          w_sclk       <= '0';
        end if;

        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when ACTIVATE_SLAVE =>

        if (r_count > 0) then
          w_next_state <= ACTIVATE_SLAVE;
        else
          w_next_state <= WAIT_FOR_SLAVE_ACK;
        end if;

        w_sdata <= c_slave_write_address(r_count);
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when WAIT_FOR_SLAVE_ACK =>

        -- if (io_sdata = '0') then
        w_next_state <= SELECT_REGISTER;
        -- else
        --  w_next_state <= error;
        -- end if;

        w_sdata <= 'Z';
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when SELECT_REGISTER =>

        if (r_count > 0) then
          w_next_state <= SELECT_REGISTER;
        else
          w_next_state <= WAIT_FOR_REGISTER_ACK;
        end if;

        w_sdata <= r_address(r_count);
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when WAIT_FOR_REGISTER_ACK =>

        -- if io_sdata = '0'
        if (r_we = '1') then
          w_next_state <= SEND_DATA1;
        elsif (r_we = '0') then
          w_next_state <= START_ACTIVATE_READ_SLAVE;
        else
          w_next_state <= ERROR;
        end if;

        w_sdata <= 'Z';
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      --------------- write path --------------
      when SEND_DATA1 =>

        if (r_count > 0) then
          w_next_state <= SEND_DATA1;
        else
          w_next_state <= WAIT_FOR_SEND_DATA1_ACK;
        end if;

        w_sdata <= r_data(r_count + 8);
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when WAIT_FOR_SEND_DATA1_ACK =>

        -- if (io_sdata = '0') then
        w_next_state <= SEND_DATA2;
        -- else
        --  w_next_state <= error;
        -- end if;

        w_sdata <= 'Z';
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when SEND_DATA2 =>

        if (r_count > 0) then
          w_next_state <= SEND_DATA2;
        else
          w_next_state <= WAIT_FOR_SEND_DATA2_ACK;
        end if;

        w_sdata <= r_data(r_count);
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when WAIT_FOR_SEND_DATA2_ACK =>

        -- if (io_sdata = '0') then
        w_next_state <= STOP_TRANSMISSION;
        -- else
        --    w_next_state <= error;
        -- end if;

        w_sdata <= 'Z';
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      --------------- read path ------------------

      when START_ACTIVATE_READ_SLAVE =>

        w_next_state <= ACTIVATE_READ_SLAVE;

        if (r_count = 7) then
          w_next_state <= START_ACTIVATE_READ_SLAVE;
          w_sdata      <= '1';
          w_sclk       <= '1';
        elsif (r_count = 6) then
          w_next_state <= START_ACTIVATE_READ_SLAVE;
          w_sdata      <= '0';
          w_sclk       <= '1';
        else
          w_next_state <= ACTIVATE_READ_SLAVE;
          w_sdata      <= '0';
          w_sclk       <= '0';
        end if;

        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when ACTIVATE_READ_SLAVE =>

        if (r_count > 0) then
          w_next_state <= ACTIVATE_READ_SLAVE;
        else
          w_next_state <= WAIT_FOR_READ_SLAVE_ACK;
        end if;

        w_sdata <= c_slave_read_address(r_count);
        w_valid <= '0';
        w_ready <= '0';
        w_error <= '0';

      when WAIT_FOR_READ_SLAVE_ACK =>

        w_next_state <= RECV_DATA1;

        w_sdata <= 'Z';
        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when RECV_DATA1 =>

        if (r_count > 0) then
          w_next_state <= RECV_DATA1;
        else
          w_next_state <= SEND_RECV_DATA1_ACK;
        end if;

        w_valid <= '0';
        w_sdata <= 'Z';
        w_error <= '0';
        w_ready <= '0';

      when SEND_RECV_DATA1_ACK =>

        w_next_state <= RECV_DATA2;

        w_valid <= '0';
        w_sdata <= '0';
        w_error <= '0';
        w_ready <= '0';

      when RECV_DATA2 =>

        if (r_count > 0) then
          w_next_state <= RECV_DATA2;
        else
          w_next_state <= SEND_RECV_DATA2_ACK;
        end if;

        w_valid <= '0';
        w_sdata <= 'Z';
        w_error <= '0';
        w_ready <= '0';

      when SEND_RECV_DATA2_ACK =>

        w_next_state <= STOP_TRANSMISSION;

        w_valid <= '1';
        w_sdata <= '0';
        w_error <= '0';
        w_ready <= '0';

      ------------ general path -------------------

      when STOP_TRANSMISSION =>

        if (r_count = 7) then
          w_next_state <= STOP_TRANSMISSION;
          w_sdata      <= '0';
          w_sclk       <= '0';
        elsif (r_count = 6) then
          w_next_state <= STOP_TRANSMISSION;
          w_sdata      <= '0';
          w_sclk       <= '1';
        else
          w_next_state <= IDLE;
          w_sdata      <= '1';
          w_sclk       <= '1';
        end if;

        w_error <= '0';
        w_valid <= '0';
        w_ready <= '0';

      when ERROR =>

        w_next_state <= ERROR;

        w_error <= '1';
        w_sdata <= '1';
        w_valid <= '0';
        w_ready <= '0';

    end case;

  end process PROC_OUTPUT;

  -- This flip flock locks the next state.
  PROC_STATE_FF : process (I_RESET_N, w_half_low_flag) is
  begin

    if (I_RESET_N = '0') then
      r_state <= idle;
    elsif (rising_edge(w_half_low_flag)) then
      r_state <= w_next_state;
    end if;

  end process PROC_STATE_FF;

  -- This counter counts the amount of cycles, we are already in the current state.
  -- We use the counter to count the amount of transmitted bits.
  PROC_STATE_COUNTER : process (I_RESET_N, w_half_low_flag) is
  begin

    if (I_RESET_N = '0') then
      r_count <= 7;
    elsif (rising_edge(w_half_low_flag)) then
      if (w_next_state /= r_state) then
        r_count <= 7;
      elsif (r_count > 0 and r_state /= idle) then
        r_count <= r_count - 1;
      else
        r_count <= 7;
      end if;
    end if;

  end process PROC_STATE_COUNTER;

  ---------------------
  -- Clock generator --
  ---------------------

  PROC_CLK_DIVIDER : process (I_CLOCK, I_RESET_N) is
  begin

    if (I_RESET_N = '0') then
      r_clock_count <= 0;
      r_clk_valid   <= '0';
      r_clk_divided <= '0';
    elsif (rising_edge(I_CLOCK)) then
      if (r_clock_count < 3) then
        r_clock_count <= r_clock_count + 1;
      else
        r_clock_count <= 0;
      end if;

      if (r_clk_valid = '0') then
        r_clk_valid <= '1';
      else
        r_clk_valid <= '0';

        if (r_clk_divided = '1') then
          r_clk_divided <= '0';
        else
          r_clk_divided <= '1';
        end if;
      end if;
    end if;

  end process PROC_CLK_DIVIDER;

  -- Generates signals to indicate if we enter
  -- the middle of a divided high/low clock pulse
  -- We need this to follow the documentation that
  -- says changes to the data signal should always
  -- occur in the middle of a low pulse.
  -- Additionally the start and stop signals should
  -- always occur in the middle of a high pulse

  -- -------     -------     -------     -------     -
  -- |  0  |  1  |  2  |  3  |  0  |  1  |  2  |  3  |  base clock
  -- |     -------     -------     -------     -------

  --       |           |           |           |
  --      \|/         \|/         \|/         \|/
  -- -------------           -------------           -
  -- |           |           |           |           |  divided clock
  -- |           -------------           -------------

  PROC_HALF_CLK_DETECTOR : process (r_clock_count) is
  begin

    if (r_clock_count = 1) then
      w_half_high_flag <= '1';
      w_half_low_flag  <= '0';
    elsif (r_clock_count = 3) then
      w_half_high_flag <= '0';
      w_half_low_flag  <= '1';
    else
      w_half_high_flag <= '0';
      w_half_low_flag  <= '0';
    end if;

  end process PROC_HALF_CLK_DETECTOR;

  -----------------
  -- Register(s) --
  -----------------

  PROC_ACK_REGISTERS : process (I_RESET_N, w_half_high_flag) is
  begin

    if (I_RESET_N = '0') then
      r_ack_slave   <= '0';
      r_ack_address <= '1';
      r_ack_data_1  <= '1';
      r_ack_data_2  <= '1';
    elsif (rising_edge(w_half_high_flag)) then
      if (r_state = WAIT_FOR_SLAVE_ACK) then
        r_ack_slave <= IO_SDATA;
      elsif (r_state = WAIT_FOR_REGISTER_ACK) then
        r_ack_address <= IO_SDATA;
      elsif (r_state = WAIT_FOR_REGISTER_ACK) then
        r_ack_address <= IO_SDATA;
      elsif (r_state = WAIT_FOR_SEND_DATA1_ACK) then
        r_ack_data_1 <= IO_SDATA;
      elsif (r_state = WAIT_FOR_SEND_DATA2_ACK) then
        r_ack_data_2 <= IO_SDATA;
      end if;
    end if;

  end process PROC_ACK_REGISTERS;

  -- if we receive data, the output bits are shifted through this register
  -- When the transmission is done, the valid signal is asserted and the register can be read.
  PROC_SHIFT_REGISTER : process (I_RESET_N, r_clk_divided) is
  begin

    if (I_RESET_N = '0') then
      r_shift_out <= (others => '0');
    elsif (rising_edge(r_clk_divided)) then
      if (r_state = recv_data1 or r_state = recv_data2) then
        r_shift_out(15) <= r_shift_out(14);
        r_shift_out(14) <= r_shift_out(13);
        r_shift_out(13) <= r_shift_out(12);
        r_shift_out(12) <= r_shift_out(11);
        r_shift_out(11) <= r_shift_out(10);
        r_shift_out(10) <= r_shift_out(9);
        r_shift_out(9)  <= r_shift_out(8);
        r_shift_out(8)  <= r_shift_out(7);
        r_shift_out(7)  <= r_shift_out(6);
        r_shift_out(6)  <= r_shift_out(5);
        r_shift_out(5)  <= r_shift_out(4);
        r_shift_out(4)  <= r_shift_out(3);
        r_shift_out(3)  <= r_shift_out(2);
        r_shift_out(2)  <= r_shift_out(1);
        r_shift_out(1)  <= r_shift_out(0);
        r_shift_out(0)  <= IO_SDATA;
      end if;
    end if;

  end process PROC_SHIFT_REGISTER;

  PROC_LOCK_INPUT_SIGNALS : process (I_RESET_N, w_half_low_flag) is
  begin

    if (I_RESET_N = '0') then
      r_we      <= '0';
      r_address <= (others => '0');
      r_data    <= (others => '0');
    elsif (rising_edge(w_half_low_flag)) then
      if (r_state = IDLE) then
        r_we      <= I_WE;
        r_address <= I_ADDRESS;
        r_data    <= I_DATA;
      end if;
    end if;

  end process PROC_LOCK_INPUT_SIGNALS;

  -------------------------------------
  -- SDATA generators and processors --
  -------------------------------------

  -----------------------
  -- Wires and outputs --
  -----------------------

  O_ACK <= '1' when r_ack_slave = '1' or r_ack_address = '1' or r_ack_data_1 = '1' or r_ack_data_2 = '1' else
           '0';

  O_ERROR  <= w_error;
  O_VALID  <= w_valid;
  IO_SDATA <= w_sdata;
  O_READY  <= w_ready;
  O_SCLK   <= w_sclk;

end architecture ARCH;
