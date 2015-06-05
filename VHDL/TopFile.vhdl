----------------------------------------------------------------------------------
-- Engineer: Takayuki Yuasa
-- 
-- Create Date:    19:00:00 06/01/2015 
-- Module Name:    Tokuden_GROWTH_FY2015_FPGA - Behavioral 
-- Project Name: GROWTH-FY2015-FPGA
-- Target Devices: 
-- Description: FPGA project for the GROWTH-FY2015 detector
--
-- Dependencies: 
--    - Tokuden Spartan-6 FPGA source
--    - SpaceWire CODEC IP Core
--    - RMAP Target IP Core
--    - Modules (UART/SSDTP/iBus)
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_arith.all;
use ieee.std_logic_unsigned.all;

library UNISIM;
use UNISIM.VComponents.all;

library work;
use work.iBus_Library.all;
use work.iBus_AddressMap.all;
use work.UserModule_Library.all;

library work;
use work.SpaceWireCODECIPPackage.all;
use work.RMAPTargetIPPackage.all;

entity Tokuden_GROWTH_FY2015_FPGA is
  port (
    xtalclk_ip : in std_logic;

    USB_FD_BP      : inout std_logic_vector(15 downto 0);
    USB_FLAGA_IP   : in    std_logic;
    USB_FLAGB_IP   : in    std_logic;
    USB_FLAGC_IP   : in    std_logic;
    USB_SLRD_OP    : out   std_logic;
    USB_SLWR_OP    : out   std_logic;
    USB_SLOE_OP    : out   std_logic;
    USB_FIFOADR_OP : out   std_logic_vector(1 downto 0);
    USB_PKTEND_OP  : out   std_logic;
    USB_CLKOUT_IP  : in    std_logic;
    USB_IFCLK_OP   : out   std_logic;
    USB_RESET_BP   : inout std_logic;

--                      DDR2_RAS     : out std_logic;
--                      DDR2_CAS     : out std_logic;
--                      DDR2_WEN     : out std_logic;
--                      DDR2_CS      : out std_logic;
--                      DDR2_CKE     : out std_logic;
--                      DDR2_A       : out std_logic_vector(13 downto 0);
--                      DDR2_BA      : out std_logic_vector(1 downto 0);
--                      DDR2_D       : inout std_logic_vector(7 downto 0);
--                      DDR2_DM      : out std_logic;
--                      DDR2_ODT     : out std_logic;
--                      --DDR2_CK_P    : out std_logic;
--                      --DDR2_CK_N    : out std_logic;
--                      DDR2_DQS_P   : inout std_logic;
--                      DDR2_DQS_N   : inout std_logic;

--                      CFG_CCLK     : out std_logic;
--                      CFG_MOSI     : out std_logic;
--                      CFG_MISO     : in  std_logic;
--                      CFG_CSO      : out std_logic;

    HDR_A_BP : inout std_logic_vector(27 downto 0);
    HDR_B_BP : inout std_logic_vector(35 downto 0);

    pushsw_ip : in  std_logic;
    led_op    : out std_logic_vector(7 downto 0)
--                      DUMMY_op     : out std_logic
    );
end Tokuden_GROWTH_FY2015_FPGA;

architecture Behavioral of Tokuden_GROWTH_FY2015_FPGA is

  signal dramclk  : std_logic;
  signal dramclkn : std_logic;

  signal count     : std_logic_vector(23 downto 0);
  signal pushsw    : std_logic;
  signal pushswd   : std_logic;
  signal led       : std_logic_vector(7 downto 0);
  signal pls_count : std_logic_vector(6 downto 0);

  component clkdcm is
    port (
      RST_IN          : in  std_logic;
      CLKIN_IN        : in  std_logic;
      LOCKED_OUT      : out std_logic;
      CLK2X_OUT       : out std_logic;
      CLKFX_OUT       : out std_logic;
      CLKFX180_OUT    : out std_logic;
      CLKDV_OUT       : out std_logic;
      CLKIN_IBUFG_OUT : out std_logic;
      CLK0_OUT        : out std_logic);
  end component;

  signal USBCLK : std_logic;

  signal BRAM_DOUT    : std_logic_vector(15 downto 0);
  signal SDRAM_DOUT   : std_logic_vector(15 downto 0);
  signal GPIO_DOUT    : std_logic_vector(19 downto 0);
  signal TEST_PATTERN : std_logic_vector(15 downto 0);
  signal LFSR         : std_logic_vector(31 downto 0);
  signal HCOUNT       : std_logic_vector(11 downto 0);
  signal VCOUNT       : std_logic_vector(11 downto 0);
  signal FRAMENUM     : std_logic_vector(11 downto 0);
  signal SEQ_PATTERN  : std_logic_vector(15 downto 0);
  signal hdr_pls      : std_logic_vector(63 downto 0);

  component ezusbfx2_ctrl is port (
    -- ezusb fx2 port
    usb_clkout_ip   : in    std_logic;
    usb_ifclk_op    : out   std_logic;
    usb_fd_bp       : inout std_logic_vector(15 downto 0);
    usb_flaga_ip    : in    std_logic;
    usb_flagb_ip    : in    std_logic;
    usb_flagc_ip    : in    std_logic;
    usb_sloe_op     : out   std_logic;
    usb_slrd_op     : out   std_logic;
    usb_slwr_op     : out   std_logic;
    usb_fifoaddr_op : out   std_logic_vector(1 downto 0);
    usb_pktend_op   : out   std_logic;
    usb_reset_bp    : inout std_logic;

    -- user interface port
    uif_sysclk_ip  : in  std_logic;
    uif_reset_ip   : in  std_logic;
    uif_rd_data_op : out std_logic_vector(15 downto 0);
    uif_wr_data_ip : in  std_logic_vector(15 downto 0);
    uif_rd_rdy_op  : out std_logic;
    uif_rd_wait_op : out std_logic;
    uif_wr_req_op  : out std_logic;
    uif_wr_wait_op : out std_logic;
    uif_rd_ip      : in  std_logic;
    uif_wr_ip      : in  std_logic;

    -- following signals art option
    uif_usbclk_op : out   std_logic;
    uif_length_op : out   std_logic_vector(24 downto 0);
    uif_addr_op   : out   std_logic_vector(26 downto 0);
    uif_flag_op   : out   std_logic_vector(15 downto 0);
    uif_debug     : inout std_logic_vector(15 downto 0)  -- for debug
    );
  end component;

  signal uif_rd      : std_logic;
  signal uif_rd_data : std_logic_vector(15 downto 0);
  signal uif_rd_rdy  : std_logic;
  signal uif_rd_wait : std_logic;
  signal uif_wr      : std_logic;
  signal uif_wr_req  : std_logic;
  signal uif_wr_data : std_logic_vector(15 downto 0);
  signal uif_wr_wait : std_logic;
  signal uif_wr_pre  : std_logic;
  signal uif_flag    : std_logic_vector(15 downto 0);
  signal uif_debug   : std_logic_vector(15 downto 0);
  signal uif_length  : std_logic_vector(24 downto 0);
  signal uif_lengthd : std_logic_vector(24 downto 0);
  signal uif_addr    : std_logic_vector(26 downto 0);

  signal gstate : std_logic_vector(2 downto 0);

  ---------------------------------------------
  -- Reset
  ---------------------------------------------
  signal reset : std_logic := '0';

  ---------------------------------------------
  --Clock
  ---------------------------------------------
  signal   Clock100MHz : std_logic;
  signal   Clock200MHz : std_logic;
  constant Count1sec   : integer                      := 1e8;
  signal   counter1sec : integer range 0 to Count1sec := 0;

  ---------------------------------------------
  -- Clock Generator (clock_generator.xco)
  ---------------------------------------------
  component clock_generator
    port
      (                                 -- Clock in ports
        CLK_IN1  : in  std_logic;       --50MHz in
        -- Clock out ports
        CLK_OUT1 : out std_logic;       --200MHz
        CLK_OUT2 : out std_logic;       --250MHz
        -- Status and control signals
        RESET    : in  std_logic;
        LOCKED   : out std_logic
        );
  end component;

  signal locked : std_logic;

  ---------------------------------------------
  --LED
  ---------------------------------------------
  signal iLED : std_logic_vector(7 downto 0) := (others => '0');

  ---------------------------------------------
  --ADC
  ---------------------------------------------  
  constant NADCChannels : integer := 4;

  signal ADCClock          : std_logic;
  signal ADCClock_previous : std_logic;
  signal ADCData           : Vector10Bits(NADCChannels-1 downto 0);

  ---------------------------------------------
  -- Channel Manager
  ---------------------------------------------
  component UserModule_ChannelManager is
    generic(
      InitialAddress : std_logic_vector(15 downto 0);
      FinalAddress   : std_logic_vector(15 downto 0)
      );
    port(
      --signals connected to BusController
      BusIF2BusController        : out iBus_Signals_BusIF2BusController;
      BusController2BusIF        : in  iBus_Signals_BusController2BusIF;
      --ch mgr(time, veto, ...)
      ChMgr2ChModule_vector      : out Signal_ChMgr2ChModule_Vector(NumberOfProducerNodes-1 downto 0);
      ChModule2ChMgr_vector      : in  Signal_ChModule2ChMgr_Vector(NumberOfProducerNodes-1 downto 0);
      --control
      CommonGateIn               : in  std_logic;
      --ADC Clock
      ADCClockFrequencySelection : out adcClockFrequencies;
      --clock and reset
      Clock                      : in  std_logic;
      GlobalReset                : in  std_logic;
      ResetOut                   : out std_logic  -- 0=reset, 1=no reset
      );
  end component;


  ---------------------------------------------
  --Channel Module
  ---------------------------------------------
  component UserModule_ChannelModule is
    generic(
      InitialAddress : std_logic_vector(15 downto 0);
      FinalAddress   : std_logic_vector(15 downto 0);
      ChNumber       : std_logic_vector(2 downto 0) := (others => '0')
      );
    port(
      --signals connected to BusController
      BusIF2BusController  : out iBus_Signals_BusIF2BusController;
      BusController2BusIF  : in  iBus_Signals_BusController2BusIF;
      --adc signals
      AdcDataIn            : in  std_logic_vector(ADCResolution-1 downto 0);
      AdcClockIn           : in  std_logic;
      --ch mgr(time, veto, ...)
      ChModule2ChMgr       : out Signal_ChModule2ChMgr;
      ChMgr2ChModule       : in  Signal_ChMgr2ChModule;
      --consumer mgr
      Consumer2ConsumerMgr : out Signal_Consumer2ConsumerMgr;
      ConsumerMgr2Consumer : in  Signal_ConsumerMgr2Consumer;
      --debug
      Debug                : out std_logic_vector(7 downto 0);
      --clock and reset
      ReadClock            : in  std_logic;
      GlobalReset          : in  std_logic
      );
  end component;


  constant NumberOfADCChannels           : integer                                       := 4;
  constant ChannelModuleInitialAddresses : iBusAddresses(NumberOfADCChannels-1 downto 0) :=
    (0 => InitialAddressOf_ChModule_0, 1 => InitialAddressOf_ChModule_1, 2 => InitialAddressOf_ChModule_2, 3 => InitialAddressOf_ChModule_3);
  constant ChannelModuleFinalAddresses : iBusAddresses(NumberOfADCChannels-1 downto 0) :=
    (0 => FinalAddressOf_ChModule_0, 1 => FinalAddressOf_ChModule_1, 2 => FinalAddressOf_ChModule_2, 3 => FinalAddressOf_ChModule_3);

  --ch mgr(time, veto, ...)
  signal ChModule2ChMgr       : Signal_ChModule2ChMgr_vector(NumberOfADCChannels-1 downto 0);
  signal ChMgr2ChModule       : Signal_ChMgr2ChModule_vector(NumberOfADCChannels-1 downto 0);
  --consumer mgr
  signal Consumer2ConsumerMgr : Signal_Consumer2ConsumerMgr_vector(NumberOfADCChannels-1 downto 0);
  signal ConsumerMgr2Consumer : Signal_ConsumerMgr2Consumer_vector(NumberOfADCChannels-1 downto 0);

  ---------------------------------------------
  -- Consumer Manager
  ---------------------------------------------
  component UserModule_ConsumerManager_SocketFIFO is
    generic(
      bufferDataCountWidth : integer := 11;
      InitialAddress       : std_logic_vector(15 downto 0);
      FinalAddress         : std_logic_vector(15 downto 0)
      );
    port(
      --signals connected to BusController
      BusIF2BusController         : out iBus_Signals_BusIF2BusController;
      BusController2BusIF         : in  iBus_Signals_BusController2BusIF;
      --signals connected to ConsumerModule
      Consumer2ConsumerMgr_vector : in  Signal_Consumer2ConsumerMgr_Vector(NumberOfConsumerNodes-1 downto 0);
      ConsumerMgr2Consumer_vector : out Signal_ConsumerMgr2Consumer_Vector(NumberOfConsumerNodes-1 downto 0);
      ---------------------------------------------
      -- SocketFIFO signals
      ---------------------------------------------
      tcpSendFIFOData             : out std_logic_vector(7 downto 0);
      tcpSendFIFOWriteEnable      : out std_logic;
      tcpSendFIFOFull             : in  std_logic;
      tcpReceiveFIFOEmpty         : in  std_logic;
      tcpReceiveFIFOData          : in  std_logic_vector(7 downto 0);
      tcpReceiveFIFODataCount     : in  std_logic_vector(bufferDataCountWidth-1 downto 0);
      tcpReceiveFIFOReadEnable    : out std_logic;
      tcpConnectionEstablished    : in  std_logic;
      --clock and reset
      Clock                       : in  std_logic;
      GlobalReset                 : in  std_logic
      );
  end component;

  ---------------------------------------------
  -- UART
  ---------------------------------------------
  component UARTInterface is
    generic(
      InputClockPeriodInNanoSec : integer := 20;     -- ns
      BaudRate                  : integer := 115200  -- bps
      );
    port(
      Clock     : in  std_logic;  -- Clock input (tx/rx clocks will be internally generated)
      Reset     : in  std_logic;        -- Set '1' to reset this modlue
      TxSerial  : out std_logic;        -- Serial Tx output
      RxSerial  : in  std_logic;        -- Serial Rx input
      txDataIn  : in  std_logic_vector(7 downto 0);  -- Send data
      rxDataOut : out std_logic_vector(7 downto 0);  -- Received data
      txEnable  : in  std_logic;        -- Set '1' to send data in DataIn
      received  : out std_logic;        -- '1' when new DataOut is valid
      txReady   : out std_logic         -- '1' when Tx is not busy
      );
  end component;

  ---------------------------------------------
  -- SpaceWire
  ---------------------------------------------
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
      transmitClockDivideValue    : in  std_logic_vector(5 downto 0) := conv_std_logic_vector(9, 6);
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

  signal   transmitClock   : std_logic;
  signal   receiveClock    : std_logic;
  constant nSpaceWirePorts : integer := 1;

  signal SPW_DOUT : std_logic := '0';
  signal SPW_SOUT : std_logic := '0';

  signal SPW_DIN : std_logic := '0';
  signal SPW_SIN : std_logic := '0';

  signal linkEstablished : std_logic_vector(nSpaceWirePorts-1 downto 0);

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

  ---------------------------------------------
  -- internal Bus (iBus)
  ---------------------------------------------
  component iBus_RMAPConnector is
    generic(
      InitialAddress : std_logic_vector(15 downto 0);
      FinalAddress   : std_logic_vector(15 downto 0)
      );
    port(
      --connected to BusController
      BusIF2BusController         : out iBus_Signals_BusIF2BusController;
      BusController2BusIF         : in  iBus_Signals_BusController2BusIF;
      --RMAP bus signals
      rmapBusMasterCycleOut       : in  std_logic;
      rmapBusMasterStrobeOut      : in  std_logic;
      rmapBusMasterAddressOut     : in  std_logic_vector (31 downto 0);
      rmapBusMasterByteEnableOut  : in  std_logic_vector (1 downto 0);
      rmapBusMasterDataIn         : out std_logic_vector (15 downto 0);
      rmapBusMasterDataOut        : in  std_logic_vector (15 downto 0);
      rmapBusMasterWriteEnableOut : in  std_logic;
      rmapBusMasterReadEnableOut  : in  std_logic;
      rmapBusMasterAcknowledgeIn  : out std_logic;
      rmapBusMasterTimeOutErrorIn : out std_logic;
      --debug
      rmapProcessStateInteger     : out integer range 0 to 7;
      --clock and reset
      Clock                       : in  std_logic;
      GlobalReset                 : in  std_logic
      );
  end component; constant iBusNumberofNodes : integer := 7;
  signal BusIF2BusController                : ibus_signals_busif2buscontroller_vector(iBusNumberofNodes-1 downto 0);
  signal BusController2BusIF                : ibus_signals_buscontroller2busif_vector(iBusNumberofNodes-1 downto 0);

  ---------------------------------------------
  -- SocketVHDL-RMAP
  ---------------------------------------------
  component SSDTP2ToRMAPTargetBridge is
    generic (
      gBusWidth            : integer range 8 to 32 := 32;  -- 8 = 8bit, 16 = 16bit, 32 = 32bit
      bufferDataCountWidth : integer               := 10
      );  
    port(
      -- clock and reset
      clock                        : in  std_logic;
      reset                        : in  std_logic;
      ---------------------------------------------
      -- SocketVHDL signals
      ---------------------------------------------
      tcpSendFIFOData              : out std_logic_vector(7 downto 0);
      tcpSendFIFOWriteEnable       : out std_logic;
      tcpSendFIFOFull              : in  std_logic;
      tcpReceiveFIFOEmpty          : in  std_logic;
      tcpReceiveFIFOData           : in  std_logic_vector(7 downto 0);
      tcpReceiveFIFODataCount      : in  std_logic_vector(bufferDataCountWidth-1 downto 0);
      tcpReceiveFIFOReadEnable     : out std_logic;
      ---------------------------------------------
      -- RMAPTarget signals 
      ---------------------------------------------
      --Internal BUS 
      busMasterCycleOut            : out std_logic;
      busMasterStrobeOut           : out std_logic;
      busMasterAddressOut          : out std_logic_vector (31 downto 0);
      busMasterByteEnableOut       : out std_logic_vector ((gBusWidth/8)-1 downto 0);
      busMasterDataIn              : in  std_logic_vector (gBusWidth-1 downto 0);
      busMasterDataOut             : out std_logic_vector (gBusWidth-1 downto 0);
      busMasterWriteEnableOut      : out std_logic;
      busMasterReadEnableOut       : out std_logic;
      busMasterAcknowledgeIn       : in  std_logic;
      busMasterTimeOutErrorIn      : in  std_logic;
      -- RMAP Statemachine state                                     
      commandStateOut              : out commandStateMachine;
      replyStateOut                : out replyStateMachine;
      -- RMAP_User_Decode
      rmapLogicalAddressOut        : out std_logic_vector(7 downto 0);
      rmapCommandOut               : out std_logic_vector(3 downto 0);
      rmapKeyOut                   : out std_logic_vector(7 downto 0);
      rmapAddressOut               : out std_logic_vector(31 downto 0);
      rmapDataLengthOut            : out std_logic_vector(23 downto 0);
      requestAuthorization         : out std_logic;
      authorizeIn                  : in  std_logic;
      rejectIn                     : in  std_logic;
      replyStatusIn                : in  std_logic_vector(7 downto 0);
      -- RMAP Error Code and Status
      rmapErrorCode                : out std_logic_vector(7 downto 0);
      -- SSDTP2 state out
      stateOutSSDTP2TCPToSpaceWire : out std_logic_vector(7 downto 0);
      stateOutSSDTP2SpaceWireToTCP : out std_logic_vector(7 downto 0);
      -- statistics                                    
      statisticalInformationClear  : in  std_logic;
      statisticalInformation       : out bit32X8Array
      );
  end component;
  constant RMAPTargetLogicalAddress : std_logic_vector(7 downto 0) := x"FE";
  constant RMAPTargetKey            : std_logic_vector(7 downto 0) := x"00";
  constant RMAPTargetCRCRevision    : std_logic                    := '1';  -- RMAP Draft F version


  constant tcpRMAPBusWidth                    : integer                                           := 16;
  signal   tcpRMAPBusMasterCycleOut           : std_logic                                         := '0';
  signal   tcpRMAPBusMasterStrobeOut          : std_logic                                         := '0';
  signal   tcpRMAPBusMasterAddressOut         : std_logic_vector (31 downto 0)                    := (others => '0');
  signal   tcpRMAPBusMasterByteEnableOut      : std_logic_vector ((tcpRMAPBusWidth/8)-1 downto 0) := (others => '0');
  signal   tcpRMAPBusMasterDataIn             : std_logic_vector (tcpRMAPBusWidth-1 downto 0)     := (others => '0');
  signal   tcpRMAPBusMasterDataOut            : std_logic_vector (tcpRMAPBusWidth-1 downto 0)     := (others => '0');
  signal   tcpRMAPBusMasterWriteEnableOut     : std_logic                                         := '0';
  signal   tcpRMAPBusMasterReadEnableOut      : std_logic                                         := '0';
  signal   tcpRMAPBusMasterAcknowledgeIn      : std_logic                                         := '0';
  signal   tcpRMAPBusMasterTimeOutErrorIn     : std_logic                                         := '0';
  signal   tcpRMAPProcessStateInteger         : integer range 0 to 7;
  signal   tcpRMAPProcessStateIntegerPrevious : integer range 0 to 7;
  signal   tcpRMAPCommandStateOut             : commandstatemachine;
  signal   tcpRMAPReplyStateOut               : replystatemachine;
  signal   tcpRMAPCommandStateOutAscii        : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   tcpRMAPReplyStateOutAscii          : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   tcpRMAPLogicalAddressOut           : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   tcpRMAPCommandOut                  : std_logic_vector(3 downto 0)                      := (others => '0');
  signal   tcpRMAPKeyOut                      : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   tcpRMAPAddressOut                  : std_logic_vector(31 downto 0)                     := (others => '0');
  signal   tcpRMAPDataLengthOut               : std_logic_vector(23 downto 0)                     := (others => '0');
  signal   tcpRMAPRequestAuthorization        : std_logic                                         := '0';
  signal   tcpRMAPAuthorizeIn                 : std_logic                                         := '0';
  signal   tcpRMAPRejectIn                    : std_logic                                         := '0';
  signal   tcpRMAPReplyStatusIn               : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   tcpRMAPErrorCode                   : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   stateOutSSDTP2TCPToSpaceWire       : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   stateOutSSDTP2SpaceWireToTCP       : std_logic_vector(7 downto 0)                      := (others => '0');
  signal   tcpRMAPStatisticalInformationClear : std_logic                                         := '0';
  signal   tcpRMAPStatisticalInformation      : bit32x8array;

  signal tcpRMAPBusMasterWriteEnableOutPrevious : std_logic := '0';
  signal tcpRMAPBusMasterReadEnableOutPrevious  : std_logic := '0';
  signal tcpRMAPBusMasterAcknowledgeInPrevious  : std_logic := '0';

---------------------------------------------
-- ADC
---------------------------------------------
  signal ADC0_D   : std_logic_vector(9 downto 0) := (others => '0');
  signal ADC1_D   : std_logic_vector(9 downto 0) := (others => '0');
  signal ADC2_D   : std_logic_vector(9 downto 0) := (others => '0');
  signal ADC3_D   : std_logic_vector(9 downto 0) := (others => '0');
  signal ADC0_CLK : std_logic                    := '0';
  signal ADC1_CLK : std_logic                    := '0';
  signal ADC2_CLK : std_logic                    := '0';
  signal ADC3_CLK : std_logic                    := '0';

  signal ADC_PDWN : std_logic                    := '0';
  signal ADC_MODE : std_logic                    := '0';  -- 0 = Offset Binary Data Format, Duty Cycle Stabilizer Disabled
  signal TRIG_OUT : std_logic_vector(3 downto 0) := (others => '0');

  -- ADC's SENSE pin is connected to GND.
  -- This results in "Vref=1.0" and span = 2 * Vref = 2.0V.

---------------------------------------------
-- UART (Raspberry Pi / FT232)
---------------------------------------------
  signal RPI_RX_FPGA_TX       : std_logic := '0';
  signal RPI_TX_FPGA_RX       : std_logic := '0';
  signal FT232_RX_FPGA_TX     : std_logic := '0';
  signal FT232_TX_FPGA_RX     : std_logic := '0';
  signal FT232_nCTS_FPGA_nRTS : std_logic := '0';  -- FPGA output, FT232 input
  signal FT232_nRTS_FPGA_nCTS : std_logic := '0';  -- FPGA input, FT232 output
  signal GPS_RX_FPGA_TX       : std_logic := '0';
  signal GPS_TX_FPGA_RX       : std_logic := '0';

  constant ClockPeriodInNanoSec_for_UART : integer := 10;  --10ns for Clock100MHz
  constant BaudRate_FT232                : integer := 9600;
  constant BaudRate_GPS                  : integer := 9600;
  constant BaudRate_RPI                  : integer := 115200;

  signal ft232TxData        : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232RxData        : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232RxDataLatched : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232TxEnable      : std_logic                    := '0';
  signal ft232Received      : std_logic                    := '0';
  signal ft232TxReady       : std_logic                    := '0';

  signal gpsTxData   : std_logic_vector(7 downto 0) := (others => '0');
  signal gpsRxData   : std_logic_vector(7 downto 0) := (others => '0');
  signal gpsTxEnable : std_logic                    := '0';
  signal gpsReceived : std_logic                    := '0';
  signal gpsTxReady  : std_logic                    := '0';

  signal rpiTxData   : std_logic_vector(7 downto 0) := (others => '0');
  signal rpiRxData   : std_logic_vector(7 downto 0) := (others => '0');
  signal rpiTxEnable : std_logic                    := '0';
  signal rpiReceived : std_logic                    := '0';
  signal rpiTxReady  : std_logic                    := '0';

  constant UART_CANNOT_RECEIVE : std_logic := '1';
  constant UART_CAN_RECEIVE    : std_logic := '0';

  ---------------------------------------------
  -- Time
  ---------------------------------------------
  signal GPS_1PPS                     : std_logic := '0';
  signal gpsData                      : std_logic_vector(7 downto 0);
  signal gpsDataEnable                : std_logic;
  signal gps1PPS                      : std_logic;
  signal gpsDDMMYY                    : std_logic_vector(47 downto 0);
  signal gpsHHMMSS_SSS                : std_logic_vector(71 downto 0);
  signal gpsDateTimeUpdatedSingleShot : std_logic;
  signal gps1PPSSingleShot            : std_logic;
  signal gpsLED                       : std_logic := '0';

  ---------------------------------------------
  -- ADC
  ---------------------------------------------
  constant CountADCClock   : integer                          := 5;
  signal   counterADCClock : integer range 0 to CountADCClock := 0;


---------------------------------------------

  -- UART debug signals
  signal   uartState                 : integer range 0 to 255 := 0;
  signal   uartStateNext             : integer range 0 to 255 := 0;
  signal   uartStateAfterSend        : integer range 0 to 255 := 0;
  signal   uartStateAfterSendNewLine : integer range 0 to 255 := 0;
  constant UART_STATE_SEND           : integer                := 250;
  constant UART_STATE_SEND_WAIT1     : integer                := 251;
  constant UART_STATE_SEND_WAIT2     : integer                := 252;
  constant UART_STATE_SEND_NEWLINE   : integer                := 253;
  constant UART_STATE_SEND_NEWLINE_1 : integer                := 254;
  signal   uartLED                   : std_logic              := '0';

  -- waveform debug
  signal WaveformCountMax : integer range 0 to 65535 := 512;
  signal waveformCount    : integer range 0 to 65535 := 0;

  signal waveformFIFO_WriteData   : std_logic_vector(9 downto 0);
  signal waveformFIFO_WriteEnable : std_logic;
  signal waveformFIFO_ReadEnable  : std_logic;
  signal waveformFIFO_ReadData    : std_logic_vector(9 downto 0);
  signal waveformFIFO_Full        : std_logic;
  signal waveformFIFO_Empty       : std_logic;
  signal waveformFIFO_DataCount   : std_logic_vector(11 downto 0);
  signal selectedADCIndex         : integer range 0 to NADCChannels-1 := 0;

  signal triggeredWaveformFIFO_Reset       : std_logic := '0';
  signal triggeredWaveformFIFO_WriteData   : std_logic_vector(9 downto 0);
  signal triggeredWaveformFIFO_WriteEnable : std_logic;
  signal triggeredWaveformFIFO_ReadEnable  : std_logic;
  signal triggeredWaveformFIFO_ReadData    : std_logic_vector(9 downto 0);
  signal triggeredWaveformFIFO_Full        : std_logic;
  signal triggeredWaveformFIFO_Empty       : std_logic;
  signal triggeredWaveformFIFO_DataCount   : std_logic_vector(11 downto 0);

  signal ft232Sent10BitData : std_logic_vector(9 downto 0);

  signal GlobalReset : std_logic := '1';  --active-low reset signal (used in iBus and old UserModule modules)

  signal ResetByCommand : std_logic := '0';

  signal Count1secAtADCClock : integer                                := 200000000;
  signal adcClockCounter     : integer range 0 to Count1secAtADCClock := 0;

  signal led1sec : std_logic := '0';

  signal   ft232OutputSelector    : integer range 0 to 7 := 0;  -- 0 = FPGA, 1 = ADC, 2 = GPS
  constant FT232_OUTPUT_MODE_FPGA : integer              := 0;
  constant FT232_OUTPUT_MODE_ADC  : integer              := 1;
  constant FT232_OUTPUT_MODE_GPS  : integer              := 2;

  signal FPGA_GPIO0 : std_logic;


  signal Baseline        : Vector10Bits(7 downto 0);
  signal BaselineSum     : std_logic_vector(12 downto 0) := (others => '0');
  signal BaselineAverage : std_logic_vector(9 downto 0)  := (others => '0');
  signal Delta           : integer range 0 to 65535      := 0;
  signal Threshold       : integer range 0 to 1023       := 530;
  signal Trigger         : std_logic                     := '0';
  signal TriggerCount    : std_logic_vector(11 downto 0) := (others => '0');
  
begin

  
  instanceOfBlinker : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => gps1PPSSingleShot,
      blinkOut  => gpsLED
      );

  instanceOfBlinkerTrigger : entity work.Blinker
    generic map(
      LedBlinkDuration => 100000        -- 1ms = 10ns * 100000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => Trigger,
      blinkOut  => FPGA_GPIO0
      );

  led_op           <= iLED;
  --iLED(7) <= led1sec;
  -- iLED(6) <= uartLED;
  -- iLED(5) <= gpsLED;
  iLED(7 downto 0) <= conv_std_logic_vector(uartState, 8);


  transmitClock <= Clock100MHz;
  receiveClock  <= Clock200MHz;

  pushsw <= pushsw_ip;
  reset  <= '0';

  -- ADC0
  HDR_B_BP(9 downto 0) <= (others => 'Z');
  ADC0_D               <= HDR_B_BP(9 downto 0);

  -- ADC1
  HDR_B_BP(19 downto 10) <= (others => 'Z');
  ADC1_D                 <= HDR_B_BP(19 downto 10);

  -- ADC2
  HDR_A_BP(17 downto 8) <= (others => 'Z');
  ADC2_D                <= HDR_A_BP(17 downto 8);

  -- ADC3
  HDR_A_BP(27 downto 18) <= (others => 'Z');
  ADC3_D                 <= HDR_A_BP(27 downto 18);

  -- Header B
  HDR_B_BP(20)   <= ADC_PDWN;
  HDR_B_BP(21)   <= ADC_MODE;
  HDR_B_BP(22)   <= RPI_RX_FPGA_TX;
  HDR_B_BP(23)   <= ADC0_CLK;
  HDR_B_BP(24)   <= 'Z';
  RPI_TX_FPGA_RX <= HDR_B_BP(24);
  HDR_B_BP(25)   <= ADC1_CLK;
  HDR_B_BP(26)   <= SPW_DOUT;
  HDR_B_BP(27)   <= ADC2_CLK;
  HDR_B_BP(28)   <= SPW_SOUT;
  HDR_B_BP(29)   <= ADC3_CLK;
  HDR_B_BP(30)   <= 'Z';
  SPW_DIN        <= HDR_B_BP(30);

  -- GPIO Input mode
  --HDR_B_BP(31)   <= 'Z';
  --FPGA_GPIO0     <= HDR_B_BP(31);

  -- GPIO Output mode
  HDR_B_BP(31) <= FPGA_GPIO0;

  HDR_B_BP(32)   <= 'Z';
  SPW_SIN        <= HDR_B_BP(32);
  HDR_B_BP(33)   <= 'Z';
  GPS_1PPS       <= HDR_B_BP(33);
  HDR_B_BP(34)   <= 'Z';
  GPS_TX_FPGA_RX <= HDR_B_BP(34);
  HDR_B_BP(35)   <= GPS_RX_FPGA_TX;

  -- Header A
  HDR_A_BP(0)          <= FT232_RX_FPGA_TX;
  HDR_A_BP(1)          <= 'Z';
  FT232_TX_FPGA_RX     <= HDR_A_BP(1);
  HDR_A_BP(2)          <= FT232_nCTS_FPGA_nRTS;
  HDR_A_BP(3)          <= 'Z';
  FT232_nRTS_FPGA_nCTS <= HDR_A_BP(3);
  HDR_A_BP(7 downto 4) <= TRIG_OUT;

  -- When UART receive buffer is full in FPGA, assert
  -- FT232_nCTS_FPGA_nRTS <= UART_CANNOT_RECEIVE; --'1'
  -- When UART receive buffer has room, assert
  -- FT232_nCTS_FPGA_nRTS <= UART_CAN_RECEIVE; --'0'

  FT232_nCTS_FPGA_nRTS <= UART_CAN_RECEIVE;

  ---------------------------------------------
  -- Process
  ---------------------------------------------

  instanceOfWaveformFIFO : entity work.WaveformFIFO
    port map(
      clk        => Clock100MHz,
      rst        => Reset,
      din        => waveformFIFO_WriteData,
      wr_en      => waveformFIFO_WriteEnable,
      rd_en      => waveformFIFO_ReadEnable,
      dout       => waveformFIFO_ReadData,
      full       => waveformFIFO_Full,
      empty      => waveformFIFO_Empty,
      data_count => waveformFIFO_DataCount
      );

  instanceOfTriggeredWaveformFIFO2 : entity work.WaveformFIFO
    port map(
      clk        => Clock100MHz,
      rst        => triggeredWaveformFIFO_Reset,
      din        => triggeredWaveformFIFO_WriteData,
      wr_en      => triggeredWaveformFIFO_WriteEnable,
      rd_en      => triggeredWaveformFIFO_ReadEnable,
      dout       => triggeredWaveformFIFO_ReadData,
      full       => triggeredWaveformFIFO_Full,
      empty      => triggeredWaveformFIFO_Empty,
      data_count => triggeredWaveformFIFO_DataCount
      );

  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
      counter1sec <= 0;
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      if(counter1sec = Count1sec)then
        counter1sec <= 0;
        led1sec     <= not led1sec;
      else
        counter1sec <= counter1sec + 1;
      end if;
    end if;
  end process;

  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
      uartLED            <= '0';
      uartState          <= 0;
      uartStateNext      <= 0;
      uartStateAfterSend <= 0;
      ft232TxEnable      <= '0';
      waveformCount      <= 0;
      TriggerCount       <= (others => '0');
    elsif(Clock100MHz = '1' and Clock100MHz'event)then

      ADCClock_previous <= AdcClock;
      if(adcClock = '0' and adcClock_previous = '1')then
        adcData(0) <= "1111111111"-ADC0_D;
        adcData(1) <= "1111111111"-ADC1_D;
        adcData(2) <= "1111111111"-ADC2_D;
        adcData(3) <= "1111111111"-ADC3_D;

        Baseline(7) <= Baseline(6);
        Baseline(6) <= Baseline(5);
        Baseline(5) <= Baseline(4);
        Baseline(4) <= Baseline(3);
        Baseline(3) <= Baseline(2);
        Baseline(2) <= Baseline(1);
        Baseline(1) <= Baseline(0);
        Baseline(0) <= adcData(selectedADCIndex);

        BaselineSum     <= ("000" & Baseline(7)) + ("000" & Baseline(6)) + ("000" & Baseline(5)) + ("000" & Baseline(4)) + ("000" & Baseline(3)) + ("000" & Baseline(2)) + ("000" & Baseline(1)) + ("000" & Baseline(0));
        BaselineAverage <= BaselineSum(12 downto 3);
        Delta           <= conv_integer(BaselineAverage)-conv_integer(ADCData(selectedADCIndex));
        --if( Threshold < Delta ) then
        if (conv_integer(ADCData(selectedADCIndex)) > Threshold)then
          Trigger      <= '1';
          TriggerCount <= TriggerCount + 1;
        else
          Trigger <= '0';
        end if;
      end if;


      case uartState is
        when 0 =>
          ft232TxEnable                     <= '0';
          waveformCount                     <= 0;
          triggeredWaveformFIFO_ReadEnable  <= '0';
          triggeredWaveformFIFO_WriteEnable <= '0';
          triggeredWaveformFIFO_Reset       <= '0';

          --receive
          if(ft232Received = '1')then
            ft232RxDataLatched <= ft232RxData;
            uartState          <= 1;
          elsif(counter1sec = 3e7)then
            uartState <= 120;
          elsif(counter1sec = 5e7)then
            uartState <= 200;           -- ADC
          elsif(counter1sec = 7e7)then
            uartState <= 195;           -- send trigger count and FIFO status
          elsif(counter1sec = Count1sec)then
            uartState <= 100;           -- GPS
          elsif(triggeredWaveformFIFO_Empty = '1' and Trigger = '1')then
            uartState <= 180;           -- start recording triggered waveform
          end if;
        when 1 =>
          --wait until uartReceived is back to 0
          if(ft232Received = '0')then
            uartState <= 2;
          end if;
        when 2 =>
          if(ft232RxDataLatched = x"61")then     -- 'a'
            uartState <= 200;           -- start sending ADC data
          elsif(ft232RxDataLatched = x"67")then  -- 'g'
            uartState <= 100;           -- start sending GPS data
          elsif(ft232RxDataLatched = x"6C")then  -- 'l'
            uartLED   <= not uartLED;
            uartState <= 0;
          elsif(ft232RxDataLatched = x"77")then  -- 'w'
            --record waveform
            uartState <= 150;
          elsif(ft232RxDataLatched = x"30")then  -- '0'
            uartState        <= 0;
            selectedADCIndex <= 0;
          elsif(ft232RxDataLatched = x"31")then  -- '1'
            uartState        <= 0;
            selectedADCIndex <= 1;
          elsif(ft232RxDataLatched = x"32")then  -- '2'
            uartState        <= 0;
            selectedADCIndex <= 2;
          elsif(ft232RxDataLatched = x"33")then  -- '3'
            uartState        <= 0;
            selectedADCIndex <= 3;
          elsif(ft232RxDataLatched = x"35")then  -- '5'
            uartState        <= 0;
            WaveformCountMax <= 256;
          elsif(ft232RxDataLatched = x"36")then  -- '6'
            uartState        <= 0;
            WaveformCountMax <= 512;
          elsif(ft232RxDataLatched = x"37")then  -- '7'
            uartState        <= 0;
            WaveformCountMax <= 1024;
          elsif(ft232RxDataLatched = x"38")then  -- '8'
            uartState        <= 0;
            WaveformCountMax <= 2045;
          elsif(ft232RxDataLatched = x"3C")then  -- '<'
            uartState          <= 0;
            Threshold          <= Threshold - 10;
            ft232Sent10BitData <= conv_std_logic_vector(Threshold - 10, 10);
            ft232TxData        <= x"74";         -- t
            uartState          <= UART_STATE_SEND;
            uartStateAfterSend <= 201;
          elsif(ft232RxDataLatched = x"3E")then  -- '>'
            uartState          <= 0;
            Threshold          <= Threshold + 10;
            ft232Sent10BitData <= conv_std_logic_vector(Threshold + 10, 10);
            ft232TxData        <= x"74";         -- t
            uartState          <= UART_STATE_SEND;
            uartStateAfterSend <= 201;
          elsif(ft232RxDataLatched = x"63")then  -- 'c'
            uartState                   <= 0;
            triggeredWaveformFIFO_Reset <= '1';
          elsif(ft232RxDataLatched = x"74")then  -- 't' read triggered waveform
            uartState <= 181;           -- send triggered waveform
          else
            ft232TxData               <= x"2A";  -- *
            uartState                 <= UART_STATE_SEND;
            uartStateAfterSend        <= UART_STATE_SEND_NEWLINE;
            uartStateAfterSendNewLine <= 0;
          end if;

          -------------------
          -- record/send triggered waveform
          -------------------
        when 180 =>                     -- record triggered waveform
          if(adcClock = '0' and adcClock_previous = '1')then
            if(waveformCount = waveformCountMax)then
              triggeredWaveformFIFO_WriteEnable <= '0';
              uartState                         <= 0;
            else
              triggeredWaveformFIFO_WriteData   <= ADCData(selectedADCIndex);
              triggeredWaveformFIFO_WriteEnable <= '1';
              waveformCount                     <= waveformCount + 1;
            end if;
          else
            triggeredWaveformFIFO_WriteEnable <= '0';
          end if;
        when 181 =>                     -- send triggered waveform
          if(triggeredWaveformFIFO_Empty = '1')then
            --waveform sending completed
            uartState                 <= UART_STATE_SEND_NEWLINE;
            uartStateAfterSendNewLine <= 0;  -- go back to idle after new line
          else
            triggeredWaveformFIFO_ReadEnable <= '1';
            uartState                        <= 182;
          end if;
        when 182 =>
          triggeredWaveformFIFO_ReadEnable <= '0';
          uartState                        <= 183;
        when 183 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(triggeredWaveformFIFO_ReadData(9)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 184;
        when 184 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(triggeredWaveformFIFO_ReadData(8 downto 6)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 185;
        when 185 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(triggeredWaveformFIFO_ReadData(5 downto 3)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 186;
        when 186 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(triggeredWaveformFIFO_ReadData(2 downto 0)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 187;
        when 187 =>
          ft232TxData        <= x"20";  -- space
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 181;
        when 190 =>                     -- clear fifo
          if(triggeredWaveformFIFO_Empty = '1')then
            triggeredWaveformFIFO_ReadEnable <= '0';
            uartState                        <= 0;
          else
            triggeredWaveformFIFO_ReadEnable <= '1';
          end if;
        when 195 =>                     -- send trigger count and FIFO status
          ft232TxData        <= x"54";  -- T
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 196;
        when 196 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer((not triggeredWaveformFIFO_Empty) & TriggerCount(10 downto 9)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 197;
        when 197 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(TriggerCount(8 downto 6)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 198;
        when 198 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(TriggerCount(5 downto 3)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 199;
        when 199 =>
          ft232TxData               <= x"30"+conv_std_logic_vector(conv_integer(TriggerCount(2 downto 0)), 8);
          uartState                 <= UART_STATE_SEND;
          uartStateAfterSend        <= UART_STATE_SEND_NEWLINE;
          uartStateAfterSendNewLine <= 0;    -- idle


          -------------------
          -- send BaselineAverage
          -------------------
        when 120 =>                        --Baseline
          ft232Sent10BitData <= BaselineAverage;
          ft232TxData        <= x"42";     -- B
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 201;
          -------------------
          -- send ADC data
          -------------------
        when 200 =>                        --ADC
          ft232Sent10BitData <= adcData(selectedADCIndex);
          ft232TxData        <= x"41";     -- A
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 201;
          -------------------
          -- send subroutine
          -------------------
        when 201 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(ft232Sent10BitData(9)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 202;
        when 202 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(ft232Sent10BitData(8 downto 6)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 203;
        when 203 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(ft232Sent10BitData(5 downto 3)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 204;
        when 204 =>
          ft232TxData               <= x"30"+conv_std_logic_vector(conv_integer(ft232Sent10BitData(2 downto 0)), 8);
          uartState                 <= UART_STATE_SEND;
          uartStateAfterSend        <= UART_STATE_SEND_NEWLINE;
          uartStateAfterSendNewLine <= 0;  -- idle

          -------------------
          -- send GPS data
          -------------------
        when 100 =>                        -- send GPS data
          ft232TxData        <= x"47";     -- G
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 101;
        when 101 =>                        --YY
          ft232TxData        <= gpsDDMMYY(15 downto 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 102;
        when 102 =>                        --YY
          ft232TxData        <= gpsDDMMYY(7 downto 0);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 103;
        when 103 =>                        --MM
          ft232TxData        <= gpsDDMMYY(31 downto 24);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 104;
        when 104 =>                        --MM
          ft232TxData        <= gpsDDMMYY(23 downto 16);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 105;
        when 105 =>                        --DD
          ft232TxData        <= gpsDDMMYY(47 downto 40);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 106;
        when 106 =>                        --DD
          ft232TxData        <= gpsDDMMYY(39 downto 32);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 107;
        when 107 =>                        --DD
          ft232TxData        <= x"5F";     -- _
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 108;
        when 108 =>                        --hh
          ft232TxData        <= gpsHHMMSS_SSS(71 downto 64);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 109;
        when 109 =>                        --hh
          ft232TxData        <= gpsHHMMSS_SSS(63 downto 56);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 110;
        when 110 =>                        --mm
          ft232TxData        <= gpsHHMMSS_SSS(55 downto 48);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 111;
        when 111 =>                        --mm
          ft232TxData        <= gpsHHMMSS_SSS(47 downto 40);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 112;
        when 112 =>                        --ss
          ft232TxData        <= gpsHHMMSS_SSS(39 downto 32);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 113;
        when 113 =>                        --ss
          ft232TxData        <= gpsHHMMSS_SSS(31 downto 24);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 114;
        when 114 =>                        --.
          ft232TxData        <= x"2E";     -- .
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 115;
        when 115 =>                        --ss
          ft232TxData        <= gpsHHMMSS_SSS(23 downto 16);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 116;
        when 116 =>                        --ss
          ft232TxData        <= gpsHHMMSS_SSS(15 downto 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 117;
        when 117 =>                        --ss
          ft232TxData               <= gpsHHMMSS_SSS(7 downto 0);
          uartState                 <= UART_STATE_SEND;
          uartStateAfterSend        <= UART_STATE_SEND_NEWLINE;
          uartStateAfterSendNewLine <= 0;  -- idle

          -------------------
          -- record waveform
          -------------------
        when 150 =>
          if(adcClock = '0' and adcClock_previous = '1')then
            if(waveformCount = waveformCountMax)then
              waveformFIFO_WriteEnable <= '0';
              uartState                <= 151;
            else
              waveformFIFO_WriteData   <= ADCData(selectedADCIndex);
              waveformFIFO_WriteEnable <= '1';
              waveformCount            <= waveformCount + 1;
            end if;
          else
            waveformFIFO_WriteEnable <= '0';
          end if;
        when 151 =>
          if(waveformFIFO_Empty = '1')then
            --waveform recording and sending completed
            uartState                 <= UART_STATE_SEND_NEWLINE;
            uartStateAfterSendNewLine <= 0;  -- go back to idle after new line
          else
            waveformFIFO_ReadEnable <= '1';
            uartState               <= 152;
          end if;
        when 152 =>
          waveformFIFO_ReadEnable <= '0';
          uartState               <= 160;
        when 160 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(waveformFIFO_ReadData(9)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 161;
        when 161 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(waveformFIFO_ReadData(8 downto 6)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 162;
        when 162 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(waveformFIFO_ReadData(5 downto 3)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 163;
        when 163 =>
          ft232TxData        <= x"30"+conv_std_logic_vector(conv_integer(waveformFIFO_ReadData(2 downto 0)), 8);
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 164;
        when 164 =>
          ft232TxData        <= x"20";       -- space
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= 151;

          -------------------
          -- tx subroutine
          -------------------
        when UART_STATE_SEND =>
          if(ft232TxReady = '1')then
            ft232TxEnable <= '1';
            uartState     <= UART_STATE_SEND_WAIT1;
          else
            ft232TxEnable <= '0';
          end if;
        when UART_STATE_SEND_WAIT1 =>
          ft232TxEnable <= '0';
          if(ft232TxReady = '0')then
            uartState <= UART_STATE_SEND_WAIT2;
          end if;
        when UART_STATE_SEND_WAIT2 =>
          if(ft232TxReady = '1')then
            uartState <= uartStateAfterSend;
          end if;
          -------------------
          -- send CR+LF
          -------------------
        when UART_STATE_SEND_NEWLINE =>
          ft232TxData        <= x"0D";  -- CR
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= UART_STATE_SEND_NEWLINE_1;
        when UART_STATE_SEND_NEWLINE_1 =>
          ft232TxData        <= x"0A";  -- LF
          uartState          <= UART_STATE_SEND;
          uartStateAfterSend <= uartStateAfterSendNewLine;
        when others =>
          uartState <= 0;
      end case;
    end if;
  end process;

  --============================================
  --============================================
  --============================================
  ---------------------------------------------
  -- Instantiation
  ---------------------------------------------
  inst_clkdcm : clkdcm port map (
    RST_IN          => '0',
    CLKIN_IN        => xtalclk_ip,
    LOCKED_OUT      => open,
    CLK2X_OUT       => Clock100MHz,
    CLKFX_OUT       => Clock200MHz,     -- 200MHz
    CLKFX180_OUT    => open,            -- 200MHz
    CLKDV_OUT       => open,
    CLKIN_IBUFG_OUT => open,
    CLK0_OUT        => open
    );

  INST_USBCTRL : ezusbfx2_ctrl port map (
    usb_clkout_ip   => USB_CLKOUT_IP,
    usb_ifclk_op    => USB_IFCLK_OP,
    usb_fd_bp       => USB_FD_BP,
    usb_flaga_ip    => USB_FLAGA_IP,
    usb_flagb_ip    => USB_FLAGB_IP,
    usb_flagc_ip    => USB_FLAGC_IP,
    usb_sloe_op     => USB_SLOE_OP,
    usb_slrd_op     => USB_SLRD_OP,
    usb_slwr_op     => USB_SLWR_OP,
    usb_fifoaddr_op => USB_FIFOADR_OP,
    usb_pktend_op   => USB_PKTEND_OP,
    usb_reset_bp    => USB_RESET_BP,

    uif_sysclk_ip  => Clock100MHz,
    uif_reset_ip   => '0',
    uif_rd_data_op => uif_rd_data,
    uif_wr_data_ip => uif_wr_data,
    uif_rd_rdy_op  => uif_rd_rdy,
    uif_rd_wait_op => uif_rd_wait,
    uif_wr_req_op  => uif_wr_req,
    uif_wr_wait_op => uif_wr_wait,
    uif_rd_ip      => uif_rd,
    uif_wr_ip      => uif_wr,

    uif_usbclk_op => USBCLK,
    uif_length_op => uif_length,
    uif_addr_op   => uif_addr,
    uif_flag_op   => uif_flag,
    uif_debug     => uif_debug
    );

  ---------------------------------------------
  -- UART (FT232)
  ---------------------------------------------
  uartFT232 : UARTInterface
    generic map(
      InputClockPeriodInNanoSec => ClockPeriodInNanoSec_for_UART,  --ns
      BaudRate                  => BaudRate_FT232
      )
    port map(
      Clock     => Clock100MHz,
      Reset     => '0',
      TxSerial  => FT232_RX_FPGA_TX,
      RxSerial  => FT232_TX_FPGA_RX,
      txDataIn  => ft232TxData,
      rxDataOut => ft232RxData,
      txEnable  => ft232TxEnable,
      received  => ft232Received,
      txReady   => ft232TxReady
      );

  ---------------------------------------------
  -- UART (GPS)
  ---------------------------------------------
  instanceOfGPSUARTInterface : entity work.GPSUARTInterface
    generic map(
      InputClockPeriodInNanoSec => ClockPeriodInNanoSec_for_UART,  --ns
      BaudRate                  => BaudRate_GPS
      )
    port map(
      clock                        => Clock100MHz,
      reset                        => reset,
      --from GPS
      gpsUARTIn                    => GPS_TX_FPGA_RX,
      gps1PPS                      => GPS_1PPS,
      --processed signals
      gpsDDMMYY                    => gpsDDMMYY,
      gpsHHMMSS_SSS                => gpsHHMMSS_SSS,
      gpsDateTimeUpdatedSingleShot => gpsDateTimeUpdatedSingleShot,
      gps1PPSSingleShot            => gps1PPSSingleShot
      );

  ---------------------------------------------
  -- UART (Raspberry Pi)
  ---------------------------------------------
  uartRPI : UARTInterface
    generic map(
      InputClockPeriodInNanoSec => ClockPeriodInNanoSec_for_UART,  --ns
      BaudRate                  => BaudRate_RPI
      )
    port map(
      Clock     => Clock100MHz,
      Reset     => '0',
      TxSerial  => RPI_RX_FPGA_TX,
      RxSerial  => RPI_TX_FPGA_RX,
      txDataIn  => rpiTxData,
      rxDataOut => rpiRxData,
      txEnable  => rpiTxEnable,
      received  => rpiReceived,
      txReady   => rpiTxReady
      );

  ---------------------------------------------
  -- ADC
  ---------------------------------------------

  -- ADC clock
  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      if(counterADCClock = CountADCClock)then
        counterADCClock <= 0;
        ADCClock        <= not ADCClock;
      else
        counterADCClock <= counterADCClock + 1;
      end if;
    end if;
  end process;
  ADC0_CLK <= AdcClock;
  ADC1_CLK <= AdcClock;
  ADC2_CLK <= AdcClock;
  ADC3_CLK <= AdcClock;

  -- ADC Interface
  -- todo

  -----------------------------------------------
  ---- Channel Manager
  -----------------------------------------------
  --instanceOfChannelManager : UserModule_ChannelManager
  --  generic map(
  --    InitialAddress => InitialAddressOf_ChMgr,
  --    FinalAddress   => FinalAddressOf_ChMgr
  --    )
  --  port map(
  --    --signals connected to BusController
  --    BusIF2BusController        => BusIF2BusController(0),
  --    BusController2BusIF        => BusController2BusIF(0),
  --    --ch mgr(time, veto, ...)
  --    ChMgr2ChModule_vector      => ChMgr2ChModule,
  --    ChModule2ChMgr_vector      => ChModule2ChMgr,
  --    --control
  --    CommonGateIn               => '0',  -- todo: implement this
  --    --ADCClockSelection
  --    ADCClockFrequencySelection => adcClockFrequencySelection,
  --    --clock and reset
  --    Clock                      => Clock100MHz,
  --    GlobalReset                => GlobalReset,
  --    ResetOut                   => open
  --    );

  -----------------------------------------------
  ---- Channel Module
  -----------------------------------------------  
  --ChannelModuleGenerate : for i in 0 to 3 generate
  --  instanceOfChannelModule0 : UserModule_ChannelModule
  --    generic map(
  --      InitialAddress => ChannelModuleInitialAddresses(i),
  --      FinalAddress   => ChannelModuleFinalAddresses(i),
  --      ChNumber       => conv_std_logic_vector(i, 3)
  --      )
  --    port map(
  --      BusIF2BusController  => BusIF2BusController(i+1),
  --      BusController2BusIF  => BusController2BusIF(i+1),
  --      AdcDataIn            => AdcData(i),
  --      AdcClockIn           => AdcClock(i/2),
  --      ChModule2ChMgr       => ChModule2ChMgr(i),
  --      ChMgr2ChModule       => ChMgr2ChModule(i),
  --      Consumer2ConsumerMgr => Consumer2ConsumerMgr(i),
  --      ConsumerMgr2Consumer => ConsumerMgr2Consumer(i),
  --      Debug                => open,
  --      ReadClock            => Clock100MHz,
  --      GlobalReset          => GlobalReset
  --      );
  --end generate ChannelModuleGenerate;


  ----iBus Mapping
  ---- 0   => Channel Manager
  ---- 1-4 => Channel Module
  ---- 5 => Consumer Manager
  ---- 6 => iBus-RMAP bridge

  -----------------------------------------------
  ---- Consumer Manager
  -----------------------------------------------
  --instanceOfConsumerManager : UserModule_ConsumerManager_SocketFIFO
  --  generic map(
  --    bufferDataCountWidth => 11,
  --    InitialAddress       => InitialAddressOf_ConsumerMgr,
  --    FinalAddress         => FinalAddressOf_ConsumerMgr
  --    )
  --  port map(
  --    --signals connected to BusController
  --    BusIF2BusController         => BusIF2BusController(5),
  --    BusController2BusIF         => BusController2BusIF(5),
  --    --signals connected to ConsumerModule
  --    Consumer2ConsumerMgr_vector => Consumer2ConsumerMgr,
  --    ConsumerMgr2Consumer_vector => ConsumerMgr2Consumer,
  --    -- SocketFIFO signals
  --    tcpSendFIFOData             => iSocketSendFIFOWriteData(1),
  --    tcpSendFIFOWriteEnable      => iSocketSendFIFOWriteEnable(1),
  --    tcpSendFIFOFull             => iSocketSendFIFOFull(1),
  --    tcpReceiveFIFOEmpty         => iSocketReceiveFIFOEmpty(1),
  --    tcpReceiveFIFOData          => iSocketReceiveFIFOReadData(1),
  --    tcpReceiveFIFODataCount     => iSocketReceiveFIFOCount(1),
  --    tcpReceiveFIFOReadEnable    => iSocketReceiveFIFOReadEnable(1),
  --    tcpConnectionEstablished    => iConnectionEstablished(1),
  --    --clock and reset
  --    Clock                       => Clock100MHz,
  --    GlobalReset                 => GlobalReset
  --    );

  -----------------------------------------------
  ---- SocketVHDL-RMAP
  -----------------------------------------------
  --instanceOfSSDTP2RMAP : SSDTP2ToRMAPTargetBridge
  --  generic map(
  --    gBusWidth            => tcpRMAPBusWidth,  --16 for iBus bridging, 32 for SDRAM-RMAP bridging
  --    bufferDataCountWidth => iSocketReceiveFIFOCount(1)'length
  --    )
  --  port map(
  --    clock                        => Clock100MHz,
  --    reset                        => not iConnectionEstablished(2),
  --    -- TCP socket signals (send)
  --    tcpSendFIFOData              => iSocketSendFIFOWriteData(2),
  --    tcpSendFIFOWriteEnable       => iSocketSendFIFOWriteEnable(2),
  --    tcpSendFIFOFull              => iSocketSendFIFOFull(2),
  --    -- TCP socket signals (receive)
  --    tcpReceiveFIFOEmpty          => iSocketReceiveFIFOEmpty(2),
  --    tcpReceiveFIFOData           => iSocketReceiveFIFOReadData(2),
  --    tcpReceiveFIFODataCount      => iSocketReceiveFIFOCount(2),
  --    tcpReceiveFIFOReadEnable     => iSocketReceiveFIFOReadEnable(2),
  --    -- RMAP Target signals (bus access)
  --    busMasterCycleOut            => tcpRMAPBusMasterCycleOut,
  --    busMasterStrobeOut           => tcpRMAPBusMasterStrobeOut,
  --    busMasterAddressOut          => tcpRMAPBusMasterAddressOut,
  --    busMasterByteEnableOut       => tcpRMAPBusMasterByteEnableOut,
  --    busMasterDataIn              => tcpRMAPBusMasterDataIn,
  --    busMasterDataOut             => tcpRMAPBusMasterDataOut,
  --    busMasterWriteEnableOut      => tcpRMAPBusMasterWriteEnableOut,
  --    busMasterReadEnableOut       => tcpRMAPBusMasterReadEnableOut,
  --    busMasterAcknowledgeIn       => tcpRMAPBusMasterAcknowledgeIn,
  --    busMasterTimeOutErrorIn      => tcpRMAPBusMasterTimeOutErrorIn,
  --    -- RMAP Target signals (transaction control)
  --    commandStateOut              => tcpRMAPCommandStateOut,
  --    replyStateOut                => tcpRMAPReplyStateOut,
  --    rmapLogicalAddressOut        => tcpRMAPLogicalAddressOut,
  --    rmapCommandOut               => tcpRMAPCommandOut,
  --    rmapKeyOut                   => tcpRMAPKeyOut,
  --    rmapAddressOut               => tcpRMAPAddressOut,
  --    rmapDataLengthOut            => tcpRMAPDataLengthOut,
  --    requestAuthorization         => tcpRMAPRequestAuthorization,
  --    authorizeIn                  => tcpRMAPAuthorizeIn,
  --    rejectIn                     => tcpRMAPRejectIn,
  --    replyStatusIn                => tcpRMAPReplyStatusIn,
  --    rmapErrorCode                => tcpRMAPErrorCode,
  --    stateOutSSDTP2TCPToSpaceWire => stateOutSSDTP2TCPToSpaceWire,
  --    stateOutSSDTP2SpaceWireToTCP => stateOutSSDTP2SpaceWireToTCP,
  --    statisticalInformationClear  => tcpRMAPStatisticalInformationClear,
  --    statisticalInformation       => tcpRMAPStatisticalInformation
  --    );
  --tcpRMAPAuthorizeIn   <= '1' when tcpRMAPRequestAuthorization = '1' else '0';
  --tcpRMAPRejectIn      <= '0';
  --tcpRMAPReplyStatusIn <= (others => '0');

  -----------------------------------------------
  ---- iBus Controller
  -----------------------------------------------
  --instanceOfiBus_BusController : entity work.iBus_BusController
  --  generic map(
  --    NumberOfNodes => iBusNumberofNodes
  --    )
  --  port map(
  --    BusIF2BusController => BusIF2BusController,
  --    BusController2BusIF => BusController2BusIF,
  --    Clock               => Clock100MHz,
  --    GlobalReset         => GlobalReset
  --    );

  -----------------------------------------------
  ---- iBus-RMAP bridge
  -----------------------------------------------  
  --instanceOfiBus_RMAPConnector : iBus_RMAPConnector
  --  generic map(
  --    InitialAddress => x"FFF0",  -- not used because no iBus module can access to RMAPConnector (i.e. this module acts as target)
  --    FinalAddress   => x"FFFF"  -- not used because no iBus module can access to RMAPConnector (i.e. this module acts as target)
  --    )
  --  port map(
  --    BusIF2BusController         => BusIF2BusController(6),
  --    BusController2BusIF         => BusController2BusIF(6),
  --    rmapBusMasterCycleOut       => tcpRMAPBusMasterCycleOut,
  --    rmapBusMasterStrobeOut      => tcpRMAPBusMasterStrobeOut,
  --    rmapBusMasterAddressOut     => tcpRMAPBusMasterAddressOut,
  --    rmapBusMasterByteEnableOut  => tcpRMAPBusMasterByteEnableOut,
  --    rmapBusMasterDataIn         => tcpRMAPBusMasterDataIn,
  --    rmapBusMasterDataOut        => tcpRMAPBusMasterDataOut,
  --    rmapBusMasterWriteEnableOut => tcpRMAPBusMasterWriteEnableOut,
  --    rmapBusMasterReadEnableOut  => tcpRMAPBusMasterReadEnableOut,
  --    rmapBusMasterAcknowledgeIn  => tcpRMAPBusMasterAcknowledgeIn,
  --    rmapBusMasterTimeOutErrorIn => tcpRMAPBusMasterTimeOutErrorIn,
  --    rmapProcessStateInteger     => tcpRMAPProcessStateInteger,

  --    Clock       => Clock100MHz,
  --    GlobalReset => iConnectionEstablished(2)  --reset when tcp client is disconnected
  --    );

  GlobalReset <= not Reset;
end Behavioral;
