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
  signal EventFIFOWriteData   : std_logic_vector(15 downto 0);
  signal EventFIFOWriteEnable : std_logic;
  signal EventFIFOFull        : std_logic;
  signal EventFIFOReadData    : std_logic_vector(15 downto 0);
  signal EventFIFOReadEnable  : std_logic;
  signal EventFIFOEmpty       : std_logic;
  signal EventFIFODataCount   : std_logic_vector(13 downto 0);
  signal EventFIFOReset       : std_logic := '0';
  signal EventFIFOReset_from_ConsumerMgr       : std_logic := '0';

  type     EventFIFOReadStates is (Initialization, Idle, Read1, Read2, Ack, Finalize);
  signal   EventFIFOReadState                  : EventFIFOReadStates                          := Initialization;
  signal   gpsYYMMDDHHMMSS_latched             : std_logic_vector(95 downto 0)                := (others => '0');  -- 96 bits = 12 bytes
  signal   fpgaRealtime_latched                : std_logic_vector(WidthOfRealTime-1 downto 0) := (others => '0');  --48 bits = 6 bytes
  signal   GPSTimeTableRegister                : std_logic_vector(143 downto 0)               := (others => '0');  -- (12 + 6 + 2)=20 bytes = 160 bits = 10 16-bit words
  constant AddressOfEventFIFODataCountRegister : std_logic_vector(31 downto 0)                := x"20000000";
  constant AddressOfGPSTimeRegister            : std_logic_vector(31 downto 0)                := x"20000002";
  constant GPSRegisterLengthInBytes : integer := 20;

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

  type   RMAPAccessModeType is (RMAPAccessMode_iBus, RMAPAccessMode_EventFIFO);
  signal RMAPAccessMode : RMAPAccessModeType := RMAPAccessMode_iBus;

  constant RMAPTargetLogicalAddress : std_logic_vector(7 downto 0) := x"FE";
  constant RMAPTargetKey            : std_logic_vector(7 downto 0) := x"00";
  constant RMAPTargetCRCRevision    : std_logic                    := '1';  -- RMAP Draft F version


  constant uartRMAPBusWidth                       : integer                                            := 16;
  signal   uartRMAPBusMasterCycleOut              : std_logic                                          := '0';
  signal   uartRMAPBusMasterStrobeOut             : std_logic                                          := '0';
  signal   uartRMAPBusMasterAddressOut            : std_logic_vector (31 downto 0)                     := (others => '0');
  signal   uartRMAPBusMasterByteEnableOut         : std_logic_vector ((uartRMAPBusWidth/8)-1 downto 0) := (others => '0');
  signal   uartRMAPBusMasterDataIn                : std_logic_vector (uartRMAPBusWidth-1 downto 0)     := (others => '0');
  signal   uartRMAPBusMasterDataIn_iBus           : std_logic_vector (uartRMAPBusWidth-1 downto 0)     := (others => '0');
  signal   uartRMAPBusMasterDataIn_EventFIFO      : std_logic_vector (uartRMAPBusWidth-1 downto 0)     := (others => '0');
  signal   uartRMAPBusMasterDataOut               : std_logic_vector (uartRMAPBusWidth-1 downto 0)     := (others => '0');
  signal   uartRMAPBusMasterWriteEnableOut        : std_logic                                          := '0';
  signal   uartRMAPBusMasterReadEnable            : std_logic                                          := '0';
  signal   uartRMAPBusMasterReadEnable_iBus       : std_logic                                          := '0';
  signal   uartRMAPBusMasterReadEnable_EventFIFO  : std_logic                                          := '0';
  signal   uartRMAPBusMasterAcknowledge           : std_logic                                          := '0';
  signal   uartRMAPBusMasterAcknowledge_iBus      : std_logic                                          := '0';
  signal   uartRMAPBusMasterAcknowledge_EventFIFO : std_logic                                          := '0';
  signal   uartRMAPBusMasterTimeOutErrorIn        : std_logic                                          := '0';
  signal   uartRMAPProcessStateInteger            : integer range 0 to 7;
  signal   uartRMAPProcessStateIntegerPrevious    : integer range 0 to 7;
  signal   uartRMAPCommandStateOut                : commandstatemachine;
  signal   uartRMAPReplyStateOut                  : replystatemachine;
  signal   uartRMAPCommandStateOutAscii           : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   uartRMAPReplyStateOutAscii             : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   uartRMAPLogicalAddressOut              : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   uartRMAPCommandOut                     : std_logic_vector(3 downto 0)                       := (others => '0');
  signal   uartRMAPKeyOut                         : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   uartRMAPAddressOut                     : std_logic_vector(31 downto 0)                      := (others => '0');
  signal   uartRMAPDataLengthOut                  : std_logic_vector(23 downto 0)                      := (others => '0');
  signal   uartRMAPRequestAuthorization           : std_logic                                          := '0';
  signal   uartRMAPAuthorizeIn                    : std_logic                                          := '0';
  signal   uartRMAPRejectIn                       : std_logic                                          := '0';
  signal   uartRMAPReplyStatusIn                  : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   uartRMAPErrorCode                      : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   stateOutSSDTP2TCPToSpaceWire           : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   stateOutSSDTP2SpaceWireToTCP           : std_logic_vector(7 downto 0)                       := (others => '0');
  signal   uartRMAPStatisticalInformationClear    : std_logic                                          := '0';
  signal   uartRMAPStatisticalInformation         : bit32x8array;

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
  constant BaudRate_FT232                : integer := 230400;
  constant BaudRate_GPS                  : integer := 9600;
  --constant BaudRate_RPI                  : integer := 115200;


  signal ft232TxData        : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232RxData        : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232RxDataLatched : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232TxEnable      : std_logic                    := '0';
  signal ft232Received      : std_logic                    := '0';
  signal ft232TxReady       : std_logic                    := '0';

  signal ft232ReceiveFIFOWriteData   : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232ReceiveFIFOWriteEnable : std_logic                    := '0';
  signal ft232ReceiveFIFOReadEnable  : std_logic                    := '0';
  signal ft232ReceiveFIFOReadData    : std_logic_vector(7 downto 0) := (others => '0');
  signal ft232ReceiveFIFOFull        : std_logic                    := '0';
  signal ft232ReceiveFIFOEmpty       : std_logic                    := '0';
  signal ft232ReceiveFIFODataCount   : std_logic_vector(9 downto 0) := (others => '0');

  signal ft232SendFIFOClock       : std_logic;
  signal ft232SendFIFOReset       : std_logic;
  signal ft232SendFIFODataIn      : std_logic_vector(7 downto 0);
  signal ft232SendFIFOWriteEnable : std_logic;
  signal ft232SendFIFOReadEnable  : std_logic;
  signal ft232SendFIFODataOut     : std_logic_vector(7 downto 0);
  signal ft232SendFIFOFull        : std_logic;
  signal ft232SendFIFOEmpty       : std_logic;
  signal ft232SendFIFODataCount   : std_logic_vector(9 downto 0);


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
  constant CountADCClock   : integer                          := 1;
  signal   counterADCClock : integer range 0 to CountADCClock := 0;

  ---------------------------------------------

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
  signal debug_gpsRMAPAccess: std_logic := '0';


  --ssdtp debug
  signal eventFIFODataSendState      : integer range 0 to 255       := 0;
  signal rpiDumpState                : integer range 0 to 255       := 0;
  signal rpiDumpStateNext            : integer range 0 to 255       := 0;
  signal receiveFIFOReadEnableCount  : std_logic_vector(7 downto 0) := (others => '0');
  signal receiveFIFOWriteEnableCount : std_logic_vector(7 downto 0) := (others => '0');
  signal sendFIFOReadEnableCount     : std_logic_vector(7 downto 0) := (others => '0');
  signal sendFIFOWriteEnableCount    : std_logic_vector(7 downto 0) := (others => '0');



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

  instanceOfBlinker_2 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => uartRMAPBusMasterReadEnable,
      blinkOut  => iLED(0)
      );

  instanceOfBlinker_3 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => ft232ReceiveFIFOReadEnable,
      blinkOut  => iLED(1)
      );

  instanceOfBlinker_5 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => ft232ReceiveFIFOReadEnable,
      blinkOut  => iLED(2)
      );


  instanceOfBlinker_6 : entity work.Blinker
    generic map(
      LedBlinkDuration => 30000000      -- 300ms = 10ns * 30000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => uartRMAPBusMasterTimeOutErrorIn,
      blinkOut  => iLED(6)
      );

  instanceOfBlinker_7 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => FT232_nRTS_FPGA_nCTS,
      blinkOut  => iLED(7)
      );

  --iLED(7 downto 3) <= stateOutSSDTP2TCPToSpaceWire(4 downto 0);

  led_op <= iLED;
  
  --iLED(7) <= led1sec;
  iLED(5) <= gpsLED;

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

  TRIG_OUT <= (0 => GPS_1PPS, 1 => debug_gpsRMAPAccess, 2 => ChModule2ChMgr(2).Trigger, 3 => ChModule2ChMgr(3).Trigger);

  ---------------------------------------------
  -- Process
  ---------------------------------------------


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
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      if(adcClock = '0' and adcClock_previous = '1')then
        adcData(0) <= "1111111111"-ADC0_D;
        adcData(1) <= "1111111111"-ADC1_D;
        adcData(2) <= "1111111111"-ADC2_D;
        adcData(3) <= "1111111111"-ADC3_D;
      end if;
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

  uartReceiveFIFO : entity work.fifo8x1k
    port map(
      clk        => Clock100MHz,
      rst        => Reset,
      din        => ft232ReceiveFIFOWriteData,
      wr_en      => ft232ReceiveFIFOWriteEnable,
      rd_en      => ft232ReceiveFIFOReadEnable,
      dout       => ft232ReceiveFIFOReadData,
      full       => ft232ReceiveFIFOFull,
      empty      => ft232ReceiveFIFOEmpty,
      data_count => ft232ReceiveFIFODataCount
      );

  uartSendFIFO : entity work.fifo8x1k
    port map (
      clk        => Clock100MHz,
      rst        => Reset,
      din        => ft232SendFIFODataIn,
      wr_en      => ft232SendFIFOWriteEnable,
      rd_en      => ft232SendFIFOReadEnable,
      dout       => ft232SendFIFODataOut,
      full       => ft232SendFIFOFull,
      empty      => ft232SendFIFOEmpty,
      data_count => ft232SendFIFODataCount
      );

  FT232_nCTS_FPGA_nRTS        <= UART_CAN_RECEIVE when ft232ReceiveFIFOFull = '0' else UART_CANNOT_RECEIVE;
  ft232ReceiveFIFOWriteData   <= ft232RxData;
  ft232ReceiveFIFOWriteEnable <= ft232Received;


  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
      rpiTxData <= (others => '0');
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      if(ft232ReceiveFIFOReadEnable = '1')then
        receiveFIFOReadEnableCount <= receiveFIFOReadEnableCount + 1;
      end if;
      if(ft232ReceiveFIFOWriteEnable = '1')then
        receiveFIFOWriteEnableCount <= receiveFIFOWriteEnableCount + 1;
      end if;
      if(ft232SendFIFOReadEnable = '1')then
        sendFIFOReadEnableCount <= sendFIFOReadEnableCount + 1;
      end if;
      if(ft232SendFIFOWriteEnable = '1')then
        sendFIFOWriteEnableCount <= sendFIFOWriteEnableCount + 1;
      end if;

      case rpiDumpState is
        when 0 =>
          if(counter1sec = Count1sec)then
            rpiTxData        <= x"57";  -- W (meaning WriteEnableCount of uartReceiveFIFO)
            rpiDumpState     <= 100;    -- send
            rpiDumpStateNext <= 1;
          end if;
        when 1 =>
          rpiTxData        <= receiveFIFOWriteEnableCount;
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 2;
        when 2 =>
          rpiTxData        <= x"52";  -- R (meaning ReadEnableCount of uartReceiveFIFO)
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 3;
        when 3 =>
          rpiTxData        <= receiveFIFOReadEnableCount;
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 4;
        when 4 =>
          rpiTxData        <= x"53";    -- S (meaning State of TCP2SpaceWire)
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 5;
        when 5 =>
          rpiTxData        <= stateOutSSDTP2TCPToSpaceWire;
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 6;
        when 6 =>
          rpiTxData        <= stateOutSSDTP2SpaceWireToTCP;
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 7;
        when 7 =>
          rpiTxData        <= x"77";  -- w (meaning WriteEnableCount of uartSendFIFO)
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 8;
        when 8 =>
          rpiTxData        <= sendFIFOWriteEnableCount;
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 9;
        when 9 =>
          rpiTxData        <= x"72";  -- r (meaning ReadEnableCount of uartSendFIFO)
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 10;
        when 10 =>
          rpiTxData        <= sendFIFOReadEnableCount;
          rpiDumpState     <= 100;      -- send
          rpiDumpStateNext <= 0;
        when 100 =>
          if(rpiTxReady = '0')then
            rpiTxEnable  <= '0';
            rpiDumpState <= 101;
          else
            rpiTxEnable <= '1';
          end if;
        when 101 =>
          if(rpiTxReady = '1')then
            rpiDumpState <= rpiDumpStateNext;
          end if;
        when others =>
          rpiDumpState <= 0;
      end case;

      case eventFIFODataSendState is
        when 0 =>
          if(ft232SendFIFOEmpty = '0' and ft232TxReady = '1' and FT232_nRTS_FPGA_nCTS = UART_CAN_RECEIVE)then  -- if not empty and Tx is ready
            ft232SendFIFOReadEnable <= '1';
            eventFIFODataSendState  <= 1;
          else
            ft232SendFIFOReadEnable <= '0';
          end if;
        when 1 =>
          ft232SendFIFOReadEnable <= '0';
          eventFIFODataSendState  <= 2;
        when 2 =>
          ft232TxData <= ft232SendFIFODataOut;
          if(ft232TxReady = '0')then
            ft232TxEnable          <= '0';
            eventFIFODataSendState <= 3;
          else
            ft232TxEnable <= '1';
          end if;
        when 3 =>
          ft232TxEnable <= '0';
          if(ft232TxReady = '1')then
            eventFIFODataSendState <= 0;
          end if;
        when others =>
          eventFIFODataSendState <= 0;
      end case;
    end if;
  end process;

  -----------------------------------------------
  ---- UART (GPS)
  -----------------------------------------------
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
      BaudRate                  => BaudRate_FT232
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

  --rpiTxData   <= ft232RxData;
  --rpiTxEnable <= ft232Received;

  -----------------------------------------------
  ---- ADC
  -----------------------------------------------

  -- ADC clock
  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      ADCClock_previous <= ADCClock;
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

  ---------------------------------------------
  -- Channel Manager
  ---------------------------------------------
  instanceOfChannelManager : UserModule_ChannelManager
    generic map(
      InitialAddress => InitialAddressOf_ChMgr,
      FinalAddress   => FinalAddressOf_ChMgr
      )
    port map(
      --signals connected to BusController
      BusIF2BusController        => BusIF2BusController(0),
      BusController2BusIF        => BusController2BusIF(0),
      --ch mgr(time, veto, ...)
      ChMgr2ChModule_vector      => ChMgr2ChModule,
      ChModule2ChMgr_vector      => ChModule2ChMgr,
      --control
      CommonGateIn               => '0',  -- todo: implement this
      --ADCClockSelection
      ADCClockFrequencySelection => open,
      --clock and reset
      Clock                      => Clock100MHz,
      GlobalReset                => GlobalReset,
      ResetOut                   => open
      );

  ---------------------------------------------
  -- Channel Module
  ---------------------------------------------  
  ChannelModuleGenerate : for i in 0 to 3 generate
    instanceOfChannelModule0 : UserModule_ChannelModule
      generic map(
        InitialAddress => ChannelModuleInitialAddresses(i),
        FinalAddress   => ChannelModuleFinalAddresses(i),
        ChNumber       => conv_std_logic_vector(i, 3)
        )
      port map(
        BusIF2BusController  => BusIF2BusController(i+1),
        BusController2BusIF  => BusController2BusIF(i+1),
        AdcDataIn            => AdcData(i),
        AdcClockIn           => AdcClock,
        ChModule2ChMgr       => ChModule2ChMgr(i),
        ChMgr2ChModule       => ChMgr2ChModule(i),
        Consumer2ConsumerMgr => Consumer2ConsumerMgr(i),
        ConsumerMgr2Consumer => ConsumerMgr2Consumer(i),
        Debug                => open,
        ReadClock            => Clock100MHz,
        GlobalReset          => GlobalReset
        );
  end generate ChannelModuleGenerate;


  --iBus Mapping
  -- 0   => Channel Manager
  -- 1-4 => Channel Module
  -- 5 => Consumer Manager
  -- 6 => iBus-RMAP bridge

  ---------------------------------------------
  -- Consumer Manager
  ---------------------------------------------
  instanceOfConsumerManager : entity work.UserModule_ConsumerManager_EventFIFO
    generic map(
      bufferDataCountWidth => EventFIFODataCount'length,
      InitialAddress       => InitialAddressOf_ConsumerMgr,
      FinalAddress         => FinalAddressOf_ConsumerMgr
      )
    port map(
      --signals connected to BusController
      BusIF2BusController         => BusIF2BusController(5),
      BusController2BusIF         => BusController2BusIF(5),
      --signals connected to ConsumerModule
      Consumer2ConsumerMgr_vector => Consumer2ConsumerMgr,
      ConsumerMgr2Consumer_vector => ConsumerMgr2Consumer,
      -- SocketFIFO signals
      EventFIFOWriteData          => EventFIFOWriteData,
      EventFIFOWriteEnable        => EventFIFOWriteEnable,
      EventFIFOFull               => EventFIFOFull,
      EventFIFOReset              => EventFIFOReset_from_ConsumerMgr,
      --clock and reset
      Clock                       => Clock100MHz,
      GlobalReset                 => GlobalReset
      );

  ---------------------------------------------
  -- SocketVHDL-RMAP
  ---------------------------------------------
  
  instanceOfSSDTP2RMAP : entity work.SSDTP2ToRMAPTargetBridge
    generic map(
      gBusWidth            => uartRMAPBusWidth,  --16 for iBus bridging, 32 for SDRAM-RMAP bridging
      bufferDataCountWidth => ft232ReceiveFIFODataCount'length
      )
    port map(
      clock                        => Clock100MHz,
      reset                        => Reset,
      -- TCP socket signals (tcp send)
      tcpSendFIFOData              => ft232SendFIFODataIn,
      tcpSendFIFOWriteEnable       => ft232SendFIFOWriteEnable,
      tcpSendFIFOFull              => ft232SendFIFOFull,
      -- TCP socket signals (tcp receive)
      tcpReceiveFIFOEmpty          => ft232ReceiveFIFOEmpty,
      tcpReceiveFIFOData           => ft232ReceiveFIFOReadData,
      tcpReceiveFIFODataCount      => ft232ReceiveFIFODataCount,
      tcpReceiveFIFOReadEnable     => ft232ReceiveFIFOReadEnable,
      -- RMAP Target signals (bus access)
      busMasterCycleOut            => uartRMAPBusMasterCycleOut,
      busMasterStrobeOut           => uartRMAPBusMasterStrobeOut,
      busMasterAddressOut          => uartRMAPBusMasterAddressOut,
      busMasterByteEnableOut       => uartRMAPBusMasterByteEnableOut,
      busMasterDataIn              => uartRMAPBusMasterDataIn,
      busMasterDataOut             => uartRMAPBusMasterDataOut,
      busMasterWriteEnableOut      => uartRMAPBusMasterWriteEnableOut,
      busMasterReadEnableOut       => uartRMAPBusMasterReadEnable,
      busMasterAcknowledgeIn       => uartRMAPBusMasterAcknowledge,
      busMasterTimeOutErrorIn      => uartRMAPBusMasterTimeOutErrorIn,
      -- RMAP Target signals (transaction control)
      commandStateOut              => uartRMAPCommandStateOut,
      replyStateOut                => uartRMAPReplyStateOut,
      rmapLogicalAddressOut        => uartRMAPLogicalAddressOut,
      rmapCommandOut               => uartRMAPCommandOut,
      rmapKeyOut                   => uartRMAPKeyOut,
      rmapAddressOut               => uartRMAPAddressOut,
      rmapDataLengthOut            => uartRMAPDataLengthOut,
      requestAuthorization         => uartRMAPRequestAuthorization,
      authorizeIn                  => uartRMAPAuthorizeIn,
      rejectIn                     => uartRMAPRejectIn,
      replyStatusIn                => uartRMAPReplyStatusIn,
      rmapErrorCode                => uartRMAPErrorCode,
      stateOutSSDTP2TCPToSpaceWire => stateOutSSDTP2TCPToSpaceWire,
      stateOutSSDTP2SpaceWireToTCP => stateOutSSDTP2SpaceWireToTCP,
      statisticalInformationClear  => uartRMAPStatisticalInformationClear,
      statisticalInformation       => uartRMAPStatisticalInformation
      );
  uartRMAPAuthorizeIn   <= '1' when uartRMAPRequestAuthorization = '1' else '0';
  uartRMAPRejectIn      <= '0';
  uartRMAPReplyStatusIn <= (others => '0');

  ---------------------------------------------
  -- iBus Controller
  ---------------------------------------------
  instanceOfiBus_BusController : entity work.iBus_BusController
    generic map(
      NumberOfNodes => iBusNumberofNodes
      )
    port map(
      BusIF2BusController => BusIF2BusController,
      BusController2BusIF => BusController2BusIF,
      Clock               => Clock100MHz,
      GlobalReset         => GlobalReset
      );

  ---------------------------------------------
  -- iBus-RMAP bridge
  ---------------------------------------------  
  instanceOfiBus_RMAPConnector : entity work.iBus_RMAPConnector
    generic map(
      InitialAddress => x"FFF0",  -- not used because no iBus module can access to RMAPConnector (i.e. this module acts as target)
      FinalAddress   => x"FFFF"  -- not used because no iBus module can access to RMAPConnector (i.e. this module acts as target)
      )
    port map(
      BusIF2BusController         => BusIF2BusController(6),
      BusController2BusIF         => BusController2BusIF(6),
      rmapBusMasterCycleOut       => uartRMAPBusMasterCycleOut,
      rmapBusMasterStrobeOut      => uartRMAPBusMasterStrobeOut,
      rmapBusMasterAddressOut     => uartRMAPBusMasterAddressOut,
      rmapBusMasterByteEnableOut  => uartRMAPBusMasterByteEnableOut,
      rmapBusMasterDataIn         => uartRMAPBusMasterDataIn_iBus,
      rmapBusMasterDataOut        => uartRMAPBusMasterDataOut,
      rmapBusMasterWriteEnableOut => uartRMAPBusMasterWriteEnableOut,
      rmapBusMasterReadEnableOut  => uartRMAPBusMasterReadEnable_iBus,
      rmapBusMasterAcknowledgeIn  => uartRMAPBusMasterAcknowledge_iBus,
      rmapBusMasterTimeOutErrorIn => uartRMAPBusMasterTimeOutErrorIn,
      rmapProcessStateInteger     => uartRMAPProcessStateInteger,

      Clock       => Clock100MHz,
      GlobalReset => GlobalReset        --reset when tcp client is disconnected
      );

  GlobalReset <= not Reset;

  -- multiplexing RMAP Read access
  -- Address 0x0000_xxxx = iBus address space
  -- Address 0x1000_xxxx = EventFIFO read data
  -- Address 0x2000_0000 = EventFIFO data size register
  RMAPAccessMode                        <= RMAPAccessMode_iBus               when uartRMAPBusMasterAddressOut(31 downto 16) = x"0000" or uartRMAPBusMasterAddressOut(31 downto 16) = x"0101" else RMAPAccessMode_EventFIFO;
  uartRMAPBusMasterDataIn               <= uartRMAPBusMasterDataIn_iBus      when RMAPAccessMode = RMAPAccessMode_iBus                                                                       else uartRMAPBusMasterDataIn_EventFIFO;
  uartRMAPBusMasterReadEnable_iBus      <= uartRMAPBusMasterReadEnable       when RMAPAccessMode = RMAPAccessMode_iBus                                                                       else '0';
  uartRMAPBusMasterReadEnable_EventFIFO <= uartRMAPBusMasterReadEnable       when RMAPAccessMode = RMAPAccessMode_EventFIFO                                                                  else '0';
  uartRMAPBusMasterAcknowledge          <= uartRMAPBusMasterAcknowledge_iBus when RMAPAccessMode = RMAPAccessMode_iBus else uartRMAPBusMasterAcknowledge_EventFIFO;

  EventFIFO : entity work.EventFIFO
    port map(
      rst        => EventFIFOReset,
      clk        => Clock100MHz,
      din        => EventFIFOWriteData,
      wr_en      => EventFIFOWriteEnable,
      rd_en      => EventFIFOReadEnable,
      dout       => EventFIFOReadData,
      full       => EventFIFOFull,
      empty      => EventFIFOEmpty,
      data_count => EventFIFODataCount
      );

  -- RMAP registers
  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
      EventFIFOReadState <= Initialization;
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      EventFIFOReset <= EventFIFOReset_from_ConsumerMgr or Reset;

      if(gps1PPSSingleShot = '1' or gpsDateTimeUpdatedSingleShot='1')then
        gpsYYMMDDHHMMSS_latched <=
          gpsDDMMYY(15 downto 0) & gpsDDMMYY(31 downto 16) & gpsDDMMYY(47 downto 32)
          & gpsHHMMSS_SSS(71 downto 24);
        fpgaRealtime_latched <= ChMgr2ChModule(0).Realtime;
      end if;

      -- EventFIFO readout process
      case EventFIFOReadState is
        when Initialization =>
          EventFIFOReadEnable <= '0';
          EventFIFOReadState  <= Idle;
        when Idle =>
          if(uartRMAPBusMasterReadEnable_EventFIFO = '1')then
            if(uartRMAPBusMasterAddressOut = AddressOfEventFIFODataCountRegister)then  -- EventFIFO DataCount register
              uartRMAPBusMasterDataIn_EventFIFO(15)          <= '0';
              uartRMAPBusMasterDataIn_EventFIFO(14)          <= EventFIFOFull;
              uartRMAPBusMasterDataIn_EventFIFO(13 downto 0) <= EventFIFODataCount;
              EventFIFOReadState                             <= Ack;
            elsif(
              AddressOfGPSTimeRegister        <= uartRMAPBusMasterAddressOut
              and uartRMAPBusMasterAddressOut <= AddressOfGPSTimeRegister+conv_std_logic_vector(GPSRegisterLengthInBytes,31)
              )then                     -- GPS register first word
              case conv_integer(uartRMAPBusMasterAddressOut(15 downto 0)-AddressOfGPSTimeRegister(15 downto 0)) is
                when 0 =>               -- latch register
                  uartRMAPBusMasterDataIn_EventFIFO <= x"4750"; -- 'GP' in ASCII
                  GPSTimeTableRegister              <= gpsYYMMDDHHMMSS_latched & fpgaRealtime_latched;
                when 18 =>               -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(15 downto 0);
                when 16 =>               -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(31 downto 16);
                when 14 =>               -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(47 downto 32);
                when 12 =>               -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(63 downto 48);
                when 10 =>              -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(79 downto 64);
                when 8 =>              -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(95 downto 80);
                when 6 =>              -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(111 downto 96);
                when 4 =>              -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(127 downto 112);
                when 2 =>              -- 
                  uartRMAPBusMasterDataIn_EventFIFO <= GPSTimeTableRegister(143 downto 128);
                when others =>
                  uartRMAPBusMasterDataIn_EventFIFO <= x"FFFF";
              end case;
              EventFIFOReadState <= Ack;
            else                        -- read EventFIFO
              EventFIFOReadEnable <= '1';
              EventFIFOReadState  <= Read1;
            end if;
          else
            EventFIFOReadEnable <= '0';
          end if;
        when Read1 =>
          EventFIFOReadEnable <= '0';
          EventFIFOReadState  <= Read2;
        when Read2 =>
          uartRMAPBusMasterDataIn_EventFIFO <= EventFIFOReadData;
          EventFIFOReadState                <= Ack;
        when Ack =>
          if(uartRMAPBusMasterReadEnable_EventFIFO = '0')then
            uartRMAPBusMasterAcknowledge_EventFIFO <= '0';
            EventFIFOReadState                     <= Idle;
          else                          -- still read enable '1'
            uartRMAPBusMasterAcknowledge_EventFIFO <= '1';
          end if;
        when others =>
          EventFIFOReadState <= Initialization;
      end case;
    end if;
  end process;

end Behavioral;
