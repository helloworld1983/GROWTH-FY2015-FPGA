----------------------------------------------------------------------------------
-- Engineer: Takayuki Yuasa
-- 
-- Create Date:    19:00:00 06/01/2015 
-- Module Name:    GROWTH_FY2015_FPGA_VersionB - Behavioral 
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

entity GROWTH_FY2015_FPGA_VersionB is
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
end GROWTH_FY2015_FPGA_VersionB;

architecture Behavioral of GROWTH_FY2015_FPGA_VersionB is

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

  type     EventFIFOReadStates is (Initialization, Idle, Read1, Read2, Read_gpsDataFIFO_1, Read_gpsDataFIFO_2, Read_gpsDataFIFO_3, Ack, Finalize);
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

  -- ADC's SENSE pin is connected to GND.
  -- This results in "Vref=1.0" and span = 2 * Vref = 2.0V.

---------------------------------------------
-- UART (Raspberry Pi / FT232)
---------------------------------------------
  signal FT232_RX_FPGA_TX     : std_logic := '0';
  signal FT232_TX_FPGA_RX     : std_logic := '0';
  signal GPS_RX_FPGA_TX       : std_logic := '0';
  signal GPS_TX_FPGA_RX       : std_logic := '0';

  constant ClockPeriodInNanoSec_for_UART : integer := 10;  --10ns for Clock100MHz
  constant BaudRate_FT232                : integer := 230400;
  constant BaudRate_GPS                  : integer := 9600;

  --ft232
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

  --ft245
  signal ft245WriteData : std_logic_vector(7 downto 0) := (others => '0');
  signal ft245ReadData : std_logic_vector(7 downto 0) := (others => '0');
  signal ft245WriteEnable: std_logic := '0';
  signal ft245ReadEnable: std_logic := '0';
  signal ft245LatchData: std_logic := '0';
  signal FT245_nTXE: std_logic := '0';
  signal FT245_nRXF: std_logic := '0';
  signal FT245_WE: std_logic := '0';
  signal FT245_nRD: std_logic := '0';
  type FT245ModeStates is (FT245ModeTx, FT245ModeRX);
  signal ft245Mode: FT245ModeStates := FT245ModeRX;

  constant UART_CANNOT_RECEIVE : std_logic := '1';
  constant UART_CAN_RECEIVE    : std_logic := '0';

  ---------------------------------------------
  -- Time
  ---------------------------------------------
  signal GPS_1PPS                     : std_logic := '0';
  signal gpsData                      : std_logic_vector(7 downto 0);
  signal gpsDataEnable                : std_logic;
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

  signal RPI_OUT_FPGA_IN0 : std_logic := '0'; -- Raspberry Pi GPIO
  signal FPGA_GPIO : std_logic_vector(1 downto 0) := (others => '0');
  signal debug_gpsRMAPAccess: std_logic := '0';

  signal eventFIFODataSendState      : integer range 0 to 255       := 0;

  --version register
  constant FPGAType : std_logic_vector(31 downto 0) := x"0000ADCB";
  constant FPGAVersion : std_logic_vector(31 downto 0) := x"20151115";
  signal AddressOfFPGATypeRegister_L : std_logic_vector(31 downto 0) := x"80000000";
  signal AddressOfFPGATypeRegister_H : std_logic_vector(31 downto 0) := x"80000002";
  signal AddressOfFPGAVersionRegister_L : std_logic_vector(31 downto 0) := x"80000004";
  signal AddressOfFPGAVersionRegister_H : std_logic_vector(31 downto 0) := x"80000006";

  --GPS data FIFO
  signal gpsDataFIFOReset       : std_logic := '0';
  signal gpsDataFIFODataIn      : std_logic_vector(7 downto 0);
  signal gpsDataFIFOWriteEnable : std_logic;
  signal gpsDataFIFOReadEnable  : std_logic;
  signal gpsDataFIFODataOut     : std_logic_vector(7 downto 0);
  signal gpsDataFIFOFull        : std_logic;
  signal gpsDataFIFOEmpty       : std_logic;
  signal gpsDataFIFODataCount   : std_logic_vector(9 downto 0);
  signal AddressOfGPSDataFIFOResetRegister : std_logic_vector(31 downto 0) := x"20001000";
  signal InitialAddressOfGPSDataFIFO : std_logic_vector(31 downto 0) := x"20001002";
  signal FinalAddressOfGPSDataFIFO : std_logic_vector(31 downto 0) := x"20001FFF";

begin

  
  instanceOfBlinker_0 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => gps1PPSSingleShot,
      blinkOut  => iLED(0)
      );

  instanceOfBlinker_1 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => uartRMAPBusMasterReadEnable,
      blinkOut  => iLED(1)
      );

  instanceOfBlinker_2 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => ft232ReceiveFIFOReadEnable,
      blinkOut  => iLED(2)
      );

  instanceOfBlinker_3 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => ft245WriteEnable,
      blinkOut  => iLED(3)
      );

  instanceOfBlinker_4 : entity work.Blinker
    generic map(
      LedBlinkDuration => 10000000      -- 100ms = 10ns * 10000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => ft245ReadEnable,
      blinkOut  => iLED(4)
      );

  instanceOfBlinker_5 : entity work.Blinker
    generic map(
      LedBlinkDuration => 30000000      -- 300ms = 10ns * 30000000
      )
    port map(
      clock     => Clock100MHz,
      reset     => reset,
      triggerIn => uartRMAPBusMasterTimeOutErrorIn,
      blinkOut  => iLED(5)
      );

  led_op <= iLED;
  
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
  HDR_B_BP(20)   <= 'Z';--FT245_nTXE
  HDR_B_BP(21)   <= 'Z';--FT245_nRXF
  HDR_B_BP(22)   <= ft245WriteData(0) when ft245Mode=FT245ModeTx else 'Z';--FT245_D0
  HDR_B_BP(23)   <= ADC0_CLK;
  HDR_B_BP(24)   <= ft245WriteData(1) when ft245Mode=FT245ModeTx else 'Z';--FT245_D1
  HDR_B_BP(25)   <= ADC1_CLK;
  HDR_B_BP(26)   <= ft245WriteData(2) when ft245Mode=FT245ModeTx else 'Z';--FT245_D2
  HDR_B_BP(27)   <= ADC2_CLK;
  HDR_B_BP(28)   <= ft245WriteData(3) when ft245Mode=FT245ModeTx else 'Z';--FT245_D3
  HDR_B_BP(29)   <= ADC3_CLK;
  HDR_B_BP(30)   <= ft245WriteData(4) when ft245Mode=FT245ModeTx else 'Z';--FT245_D4
  HDR_B_BP(31)   <= 'Z';--RPI_OUT_FPGA_IN0
  HDR_B_BP(32)   <= ft245WriteData(5) when ft245Mode=FT245ModeTx else 'Z';--FT245_D5
  HDR_B_BP(33)   <= 'Z';--GPS_1PPS
  HDR_B_BP(34)   <= 'Z';--GPS_TX_FPGA_RX
  HDR_B_BP(35)   <= GPS_RX_FPGA_TX;

  -- Header A
  HDR_A_BP(0)          <= FT232_RX_FPGA_TX;
  HDR_A_BP(1)          <= 'Z';--FT232_TX_FPGA_RX
  HDR_A_BP(2)          <= FT245_WE;
  HDR_A_BP(3)          <= FT245_nRD;
  HDR_A_BP(4) <= ft245WriteData(6) when ft245Mode=FT245ModeTx else 'Z';--FT245_D6
  HDR_A_BP(5) <= ft245WriteData(7) when ft245Mode=FT245ModeTx else 'Z';--FT245_D7
  HDR_A_BP(6)          <= 'Z';--FPGA_GPIO(0)
  HDR_A_BP(7)          <= 'Z';--FPGA_GPIO(1)

  ---------------------------------------------
  -- Process
  ---------------------------------------------

  --synchronization
  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      RPI_OUT_FPGA_IN0 <= HDR_B_BP(31);
      GPS_1PPS       <= HDR_B_BP(33);
      GPS_TX_FPGA_RX <= HDR_B_BP(34);
      
      FT245_nTXE <= HDR_B_BP(20);
      FT245_nRXF <= HDR_B_BP(21);

      if(ft245LatchData='1')then
        ft245ReadData(0) <= HDR_B_BP(22);
        ft245ReadData(1) <= HDR_B_BP(24);
        ft245ReadData(2) <= HDR_B_BP(26);
        ft245ReadData(3) <= HDR_B_BP(28);
        ft245ReadData(4) <= HDR_B_BP(30);
        ft245ReadData(5) <= HDR_B_BP(32);
        ft245ReadData(6) <= HDR_A_BP(4);
        ft245ReadData(7) <= HDR_A_BP(5);
      end if;

      FT232_TX_FPGA_RX     <= HDR_A_BP(1);
    end if;
  end process;


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
        --for VersionA
        --adcData(0) <= "1111111111"-ADC0_D;
        --adcData(1) <= "1111111111"-ADC1_D;
        --adcData(2) <= "1111111111"-ADC2_D;
        --adcData(3) <= "1111111111"-ADC3_D;
        --for VersionB
        adcData(0) <= ADC0_D;
        adcData(1) <= ADC1_D;
        adcData(2) <= ADC2_D;
        adcData(3) <= ADC3_D;
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

  ft232ReceiveFIFOWriteData   <= ft232RxData;
  ft232ReceiveFIFOWriteEnable <= ft232Received;

  --GPS Data FIFO
  gpsDataFIFO : entity work.fifo8x1k
    port map (
      clk        => Clock100MHz,
      rst        => gpsDataFIFOReset,
      din        => gpsDataFIFODataIn,
      wr_en      => gpsDataFIFOWriteEnable,
      rd_en      => gpsDataFIFOReadEnable,
      dout       => gpsDataFIFODataOut,
      full       => gpsDataFIFOFull,
      empty      => gpsDataFIFOEmpty,
      data_count => gpsDataFIFODataCount
      );
  gpsDataFIFODataIn <= gpsData;
  gpsDataFIFOWriteEnable <= gpsDataEnable;

  ---------------------------------------------
  -- Processes
  ---------------------------------------------
  process(Clock100MHz, reset)
  begin
    if(reset = '1')then
    elsif(Clock100MHz = '1' and Clock100MHz'event)then
      case eventFIFODataSendState is
        when 0 =>
          if(ft232SendFIFOEmpty = '0' and ft232TxReady = '1')then  -- if not empty and Tx is ready
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
      gpsReceivedByte              => gpsData,
      gpsReceivedByteEnable        => gpsDataEnable,
      gpsDDMMYY                    => gpsDDMMYY,
      gpsHHMMSS_SSS                => gpsHHMMSS_SSS,
      gpsDateTimeUpdatedSingleShot => gpsDateTimeUpdatedSingleShot,
      gps1PPSSingleShot            => gps1PPSSingleShot
      );

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
          gpsDataFIFOReset <= '0';
          gpsDataFIFOReadEnable <= '0';
          EventFIFOReadEnable <= '0';
          EventFIFOReadState  <= Idle;
        when Idle =>
          if(uartRMAPBusMasterReadEnable_EventFIFO = '1')then
            if(-- Access to EventFIFO DataCount Register
              uartRMAPBusMasterAddressOut = AddressOfEventFIFODataCountRegister)then
              uartRMAPBusMasterDataIn_EventFIFO(15)          <= '0';
              uartRMAPBusMasterDataIn_EventFIFO(14)          <= EventFIFOFull;
              uartRMAPBusMasterDataIn_EventFIFO(13 downto 0) <= EventFIFODataCount;
              EventFIFOReadState                             <= Ack;
            elsif( -- Access to GPS Time Register
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
            elsif(-- Access to GPS Data FIFO Reset Register
              uartRMAPBusMasterAddressOut = AddressOfGPSDataFIFOResetRegister
              )then
                --reset GPS Data FIFO
                gpsDataFIFOReset <= '1';
                EventFIFOReadState <= Ack;
            elsif(-- Access to GPS Data FIFO
              InitialAddressOfGPSDataFIFO<=uartRMAPBusMasterAddressOut 
              and uartRMAPBusMasterAddressOut <=FinalAddressOfGPSDataFIFO
              )then
              if(gpsDataFIFOEmpty='1')then -- if already empty, return dummy data
                uartRMAPBusMasterDataIn_EventFIFO <= (others => '0');
                EventFIFOReadState <= Ack;
              else -- if gpsDataFIFO has data
                gpsDataFIFOReadEnable <= '1';
                EventFIFOReadState <= Read_gpsDataFIFO_1;
              end if;
            else -- Access to undefined memory address
              -- read EventFIFO
              EventFIFOReadEnable <= '1';
              EventFIFOReadState  <= Read1;
            end if;
          else
            --if no Read access from RMAP Target IP Core, initialize control signals
            gpsDataFIFOReadEnable <= '0';
            gpsDataFIFOReset <= '0';
            EventFIFOReadEnable <= '0';
          end if;
        when Read1 =>
          EventFIFOReadEnable <= '0';
          EventFIFOReadState  <= Read2;
        when Read2 =>
          uartRMAPBusMasterDataIn_EventFIFO <= EventFIFOReadData;
          EventFIFOReadState                <= Ack;
        when Read_gpsDataFIFO_1 =>
          gpsDataFIFOReadEnable <= '1';
          EventFIFOReadState  <= Read_gpsDataFIFO_2;
        when Read_gpsDataFIFO_2 =>
          gpsDataFIFOReadEnable <= '0';
          uartRMAPBusMasterDataIn_EventFIFO(15 downto 8) <= gpsDataFIFODataOut;
          EventFIFOReadState  <= Read_gpsDataFIFO_3;
        when Read_gpsDataFIFO_3 =>
          uartRMAPBusMasterDataIn_EventFIFO(7 downto 0) <= gpsDataFIFODataOut;
          EventFIFOReadState  <= Ack;
        when Ack =>
          if(uartRMAPBusMasterReadEnable_EventFIFO = '0')then
            uartRMAPBusMasterAcknowledge_EventFIFO <= '0';
            EventFIFOReadState                     <= Idle;
          else                          -- still read enable '1'
            uartRMAPBusMasterAcknowledge_EventFIFO <= '1';
          end if;
          gpsDataFIFOReadEnable <= '0';
          gpsDataFIFOReset <= '0';
        when others =>
          EventFIFOReadState <= Initialization;
      end case;
    end if;
  end process;

end Behavioral;
