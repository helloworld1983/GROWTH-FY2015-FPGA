library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library unisim;
use unisim.vcomponents.all;

library work;
use work.SpaceWireCODECIPPackage.all;


entity SpaceWireTimecodeEmitter is
  generic(
    constant TimecodeEmitClockCycle : integer := 1562500   -- 15.625ms@100MHz
    );
  port (
    clock              : in  std_logic;
    transmitClock      : in  std_logic;
    receiveClock       : in  std_logic;
    reset              : in  std_logic;
    --
	 linkEstablished    : out std_logic;
	 tickIn_out         : out std_logic;
	 timeIn_out         : out std_logic_vector(5 downto 0);
	 tickOut_out        : out std_logic;
	 timeOut_out        : out std_logic_vector(5 downto 0);
	 --
    spaceWireDataOut   : out std_logic;
    spaceWireStrobeOut : out std_logic;
    spaceWireDataIn    : in  std_logic;
    spaceWireStrobeIn  : in  std_logic
    );
end SpaceWireTimecodeEmitter;

architecture behavioral of SpaceWireTimecodeEmitter is

  -- timecode-emit counter
  signal   tickCounter : integer range 0 to TimecodeEmitClockCycle := 0;

  -- Initial wait after reset
  constant InitialWait        : integer                        := 10000;
  signal   initialWaitCounter : integer range 0 to InitialWait := 0;

  -- SpaceWireCODECIP
  component SpaceWireCODECIP is
    port (
      clock                       : in  std_logic;
      transmitClock               : in  std_logic;
      receiveClock                : in  std_logic;
      reset                       : in  std_logic;
      --
      transmitFIFOWriteEnable     : in  std_logic;
      transmitFIFODataIn          : in  std_logic_vector(8 downto 0);
      transmitFIFOFull            : out std_logic;
      transmitFIFODataCount       : out std_logic_vector(5 downto 0);
      receiveFIFOReadEnable       : in  std_logic;
      receiveFIFODataOut          : out std_logic_vector(8 downto 0);
      receiveFIFOFull             : out std_logic;
      receiveFIFOEmpty            : out std_logic;
      receiveFIFODataCount        : out std_logic_vector(5 downto 0);
      --
      tickIn                      : in  std_logic;
      timeIn                      : in  std_logic_vector(5 downto 0);
      controlFlagsIn              : in  std_logic_vector(1 downto 0);
      tickOut                     : out std_logic;
      timeOut                     : out std_logic_vector(5 downto 0);
      controlFlagsOut             : out std_logic_vector(1 downto 0);
      --
      linkStart                   : in  std_logic;
      linkDisable                 : in  std_logic;
      autoStart                   : in  std_logic;
      linkStatus                  : out std_logic_vector(15 downto 0);
      errorStatus                 : out std_logic_vector(7 downto 0);
      transmitClockDivideValue    : in  std_logic_vector(5 downto 0) := conv_std_logic_vector(9,6);
      creditCount                 : out std_logic_vector(5 downto 0);
      outstandingCount            : out std_logic_vector(5 downto 0);
      --
      transmitActivity            : out std_logic;
      receiveActivity             : out std_logic;
      --
      spaceWireDataOut            : out std_logic;
      spaceWireStrobeOut          : out std_logic;
      spaceWireDataIn             : in  std_logic;
      spaceWireStrobeIn           : in  std_logic;
      --                
      statisticalInformationClear : in  std_logic;
      statisticalInformation      : out bit32X8Array

      );
  end component;

  -- tx
  signal transmitFIFOWriteEnable : std_logic;
  signal transmitFIFODataIn      : std_logic_vector(8 downto 0);
  signal transmitFIFOFull        : std_logic;
  signal transmitFIFODataCount   : std_logic_vector(5 downto 0);

  -- rx
  signal receiveFIFOReadEnable : std_logic;
  signal receiveFIFODataOut    : std_logic_vector(8 downto 0);
  signal receiveFIFOFull       : std_logic;
  signal receiveFIFOEmpty      : std_logic;
  signal receiveFIFODataCount  : std_logic_vector(5 downto 0);

  -- timecode-receive related
  signal tickIn         : std_logic;
  signal timeIn         : std_logic_vector(5 downto 0);
  signal controlFlagsIn : std_logic_vector(1 downto 0);

  -- timecode-emit related
  signal tickOut         : std_logic;
  signal timeOut         : std_logic_vector(5 downto 0);
  signal controlFlagsOut : std_logic_vector(1 downto 0);

  signal linkStart   : std_logic;
  signal linkDisable : std_logic;
  signal autoStart   : std_logic;

  signal linkStatus                  : std_logic_vector(15 downto 0);
  signal errorStatus                 : std_logic_vector(7 downto 0);
  signal transmitClockDivideValue    : std_logic_vector(5 downto 0);
  signal creditCount                 : std_logic_vector(5 downto 0);
  signal outstandingCount            : std_logic_vector(5 downto 0);
  signal transmitActivity            : std_logic;
  signal receiveActivity             : std_logic;
  signal statisticalInformationClear : std_logic;
  signal statisticalInformation      : bit32x8array;

  signal iLinkEstablished : std_logic := '0';
begin

  -- does not use Tx/Rx functions
  transmitFIFOWriteEnable <= '0';
  receiveFIFOReadEnable   <= '0';

  --control flag
  controlFlagsOut <= "00";
  
  iLinkEstablished <= '1' when linkStatus(8) = '1' else '0';
  linkEstablished <= iLinkEstablished;
  
  tickIn_out <= tickIn;
  timeIn_out <= timeIn;
  
  tickOut_out <= tickOut;
  timeOut_out <= timeOut;
  
  process(clock, reset)
  begin
    if(reset = '1')then
      --reset SpaceWire link
      linkDisable        <= '1';
      linkStart          <= '0';
      autoStart          <= '0';
      --
      initialWaitCounter <= 0;
      tickCounter     <= 0;
		tickIn <= '0';
    else
      if(clock = '1' and clock'event)then
        if(InitialWait /= initialWaitCounter)then
          initialWaitCounter <= initialWaitCounter + 1;
                                        --reset SpaceWire link
          linkDisable        <= '1';
          linkStart          <= '0';
          autoStart          <= '0';
        else
                                        --enable SpaceWire link
          linkDisable <= '0';
          linkStart   <= '1';
          autoStart   <= '1';

          if(tickCounter = TimecodeEmitClockCycle)then
            tickCounter   <= 0;
            if(iLinkEstablished='1')then
              tickIn <= '1';
              timeIn <= timeIn + 1;
            else
              tickIn <= '0';
            end if;
          else
            tickCounter <= tickCounter + 1;
            tickIn        <= '0';
          end if;
        end if;
      end if;

    end if;
  end process;

  instanceOfSpaceWireCODECIP : SpaceWireCODECIP
    port map(
      clock                       => clock,
      transmitClock               => transmitClock,
      receiveClock                => receiveClock,
      reset                       => reset,
      transmitFIFOWriteEnable     => transmitFIFOWriteEnable,
      transmitFIFODataIn          => transmitFIFODataIn,
      transmitFIFOFull            => transmitFIFOFull,
      transmitFIFODataCount       => transmitFIFODataCount,
      receiveFIFOReadEnable       => receiveFIFOReadEnable,
      receiveFIFODataOut          => receiveFIFODataOut,
      receiveFIFOFull             => receiveFIFOFull,
      receiveFIFOEmpty            => receiveFIFOEmpty,
      receiveFIFODataCount        => receiveFIFODataCount,
      tickIn                      => tickIn,
      timeIn                      => timeIn,
      controlFlagsIn              => controlFlagsIn,
      tickOut                     => tickOut,
      timeOut                     => timeOut,
      controlFlagsOut             => controlFlagsOut,
      linkStart                   => linkStart,
      linkDisable                 => linkDisable,
      autoStart                   => autoStart,
      linkStatus                  => linkStatus,
      errorStatus                 => errorStatus,
      transmitClockDivideValue    => transmitClockDivideValue,
      creditCount                 => creditCount,
      outstandingCount            => outstandingCount,
      transmitActivity            => transmitActivity,
      receiveActivity             => receiveActivity,
      spaceWireDataOut            => spaceWireDataOut,
      spaceWireStrobeOut          => spaceWireStrobeOut,
      spaceWireDataIn             => spaceWireDataIn,
      spaceWireStrobeIn           => spaceWireStrobeIn,
      statisticalInformationClear => statisticalInformationClear,
      statisticalInformation      => statisticalInformation
      );
end behavioral;
