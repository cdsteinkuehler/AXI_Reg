-- Copyright (C) 2015, Charles Steinkuehler
-- <charles AT steinkuehler DOT net>
-- All rights reserved
--
-- This program is is licensed under a disjunctive dual license giving you
-- the choice of one of the two following sets of free software/open source
-- licensing terms:
--
--    * GNU General Public License (GPL), version 2.0 or later
--    * 3-clause BSD License
-- 
--
-- The GNU GPL License:
-- 
--     This program is free software; you can redistribute it and/or modify
--     it under the terms of the GNU General Public License as published by
--     the Free Software Foundation; either version 2 of the License, or
--     (at your option) any later version.
-- 
--     This program is distributed in the hope that it will be useful,
--     but WITHOUT ANY WARRANTY; without even the implied warranty of
--     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--     GNU General Public License for more details.
-- 
--     You should have received a copy of the GNU General Public License
--     along with this program; if not, write to the Free Software
--     Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
-- 
-- 
-- The 3-clause BSD License:
-- 
--     Redistribution and use in source and binary forms, with or without
--     modification, are permitted provided that the following conditions
--     are met:
-- 
--         * Redistributions of source code must retain the above copyright
--           notice, this list of conditions and the following disclaimer.
-- 
--         * Redistributions in binary form must reproduce the above
--           copyright notice, this list of conditions and the following
--           disclaimer in the documentation and/or other materials
--           provided with the distribution.
-- 
--         * Neither the name of the copyright holder nor the names of its
--           contributors may be used to endorse or promote products
--           derived from this software without specific prior written
--           permission.
-- 
-- 
-- Disclaimer:
-- 
--     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
--     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
--     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
--     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
--     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
--     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
--     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
--     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
--     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
--     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
--     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
--     POSSIBILITY OF SUCH DAMAGE.

library ieee;

use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use work.Reg_Pkg.all;

library soc_system;

entity DE0_Nano_Reg is
    port (
        ----------------
        -- ADC        --
        ----------------
        ADC_CONVST          : out   std_logic;
        ADC_SCK             : out   std_logic;
        ADC_SDI             : out   std_logic;
        ADC_SDO             : in    std_logic;

        ----------------
        -- ARDUINO    --
        ----------------
        ARDUINO_IO          : inout std_logic_vector(15 downto 0);
        ARDUINO_RESET_N     : inout std_logic;

        ----------------
        -- CLK        --
        ----------------
        CLK_I2C_SCL         : inout std_logic;
        CLK_I2C_SDA         : inout std_logic;

        ----------------
        -- FPGA       --
        ----------------
        FPGA_CLK1_50        : in    std_logic;
        FPGA_CLK2_50        : in    std_logic;
        FPGA_CLK3_50        : in    std_logic;

        ----------------
        -- GPIO       --
        ----------------
        GPIO_0              : inout std_logic_vector(35 downto 0);
        GPIO_1              : inout std_logic_vector(35 downto 0);

        ----------------
        -- HPS        --
        ----------------
        HPS_CONV_USB_N      : inout std_logic;
        HPS_DDR3_ADDR       : out   std_logic_vector(14 downto 0);
        HPS_DDR3_BA         : out   std_logic_vector( 2 downto 0);
        HPS_DDR3_CAS_N      : out   std_logic;
        HPS_DDR3_CKE        : out   std_logic;
        HPS_DDR3_CK_N       : out   std_logic;
        HPS_DDR3_CK_P       : out   std_logic;
        HPS_DDR3_CS_N       : out   std_logic;
        HPS_DDR3_DM         : out   std_logic_vector( 3 downto 0);
        HPS_DDR3_DQ         : inout std_logic_vector(31 downto 0);
        HPS_DDR3_DQS_N      : inout std_logic_vector( 3 downto 0);
        HPS_DDR3_DQS_P      : inout std_logic_vector( 3 downto 0);
        HPS_DDR3_ODT        : out   std_logic;
        HPS_DDR3_RAS_N      : out   std_logic;
        HPS_DDR3_RESET_N    : out   std_logic;
        HPS_DDR3_RZQ        : in    std_logic;
        HPS_DDR3_WE_N       : out   std_logic;
        HPS_ENET_GTX_CLK    : out   std_logic;
        HPS_ENET_INT_N      : inout std_logic;
        HPS_ENET_MDC        : out   std_logic;
        HPS_ENET_MDIO       : inout std_logic;
        HPS_ENET_RX_CLK     : in    std_logic;
        HPS_ENET_RX_DATA    : in    std_logic_vector( 3 downto 0);
        HPS_ENET_RX_DV      : in    std_logic;
        HPS_ENET_TX_DATA    : out   std_logic_vector( 3 downto 0);
        HPS_ENET_TX_EN      : out   std_logic;
        HPS_GSENSOR_INT     : inout std_logic;
        HPS_I2C0_SCLK       : inout std_logic;
        HPS_I2C0_SDAT       : inout std_logic;
        HPS_I2C1_SCLK       : inout std_logic;
        HPS_I2C1_SDAT       : inout std_logic;
        HPS_KEY             : inout std_logic;
        HPS_LED             : inout std_logic;
        HPS_LTC_GPIO        : inout std_logic;
        HPS_SD_CLK          : out   std_logic;
        HPS_SD_CMD          : inout std_logic;
        HPS_SD_DATA         : inout std_logic_vector( 3 downto 0);
        HPS_SPIM_CLK        : out   std_logic;
        HPS_SPIM_MISO       : in    std_logic;
        HPS_SPIM_MOSI       : out   std_logic;
        HPS_SPIM_SS         : inout std_logic;
        HPS_UART_RX         : in    std_logic;
        HPS_UART_TX         : out   std_logic;
        HPS_USB_CLKOUT      : in    std_logic;
        HPS_USB_DATA        : inout std_logic_vector( 7 downto 0);
        HPS_USB_DIR         : in    std_logic;
        HPS_USB_NXT         : in    std_logic;
        HPS_USB_STP         : out   std_logic;

        ----------------
        -- KEY        --
        ----------------
        KEY                 : in    std_logic_vector( 1 downto 0);

        ----------------
        -- LED        --
        ----------------
        LED                 : out   std_logic_vector( 7 downto 0);

        ----------------
        -- SW         --
        ----------------
        SW                  : in    std_logic_vector( 3 downto 0) );

end DE0_Nano_Reg;

architecture arch of DE0_Nano_Reg is
    signal axi_reg_awid         : std_logic_vector(13 downto 0);                    -- axs_s1_awid
    signal axi_reg_awaddr       : std_logic_vector(13 downto 0);                    -- axs_s1_awaddr
    signal axi_reg_awlen        : std_logic_vector(7 downto 0);                     -- axs_s1_awlen
    signal axi_reg_awsize       : std_logic_vector(2 downto 0);                     -- axs_s1_awsize
    signal axi_reg_awburst      : std_logic_vector(1 downto 0);                     -- axs_s1_awburst
    signal axi_reg_awvalid      : std_logic;                                        -- axs_s1_awvalid
    signal axi_reg_awready      : std_logic                     := '0';             -- axs_s1_awready
    signal axi_reg_wdata        : std_logic_vector(31 downto 0);                    -- axs_s1_wdata
    signal axi_reg_wstrb        : std_logic_vector(3 downto 0);                     -- axs_s1_wstrb
    signal axi_reg_wvalid       : std_logic;                                        -- axs_s1_wvalid
    signal axi_reg_wready       : std_logic                     := '0';             -- axs_s1_wready
    signal axi_reg_bid          : std_logic_vector(13 downto 0) := (others => '0'); -- axs_s1_bid
    signal axi_reg_bvalid       : std_logic                     := '0';             -- axs_s1_bvalid
    signal axi_reg_bready       : std_logic;                                        -- axs_s1_bready
    signal axi_reg_arid         : std_logic_vector(13 downto 0);                    -- axs_s1_arid
    signal axi_reg_araddr       : std_logic_vector(13 downto 0);                    -- axs_s1_araddr
    signal axi_reg_arlen        : std_logic_vector(7 downto 0);                     -- axs_s1_arlen
    signal axi_reg_arsize       : std_logic_vector(2 downto 0);                     -- axs_s1_arsize
    signal axi_reg_arburst      : std_logic_vector(1 downto 0);                     -- axs_s1_arburst
    signal axi_reg_arvalid      : std_logic;                                        -- axs_s1_arvalid
    signal axi_reg_arready      : std_logic                     := '0';             -- axs_s1_arready
    signal axi_reg_rid          : std_logic_vector(13 downto 0) := (others => '0'); -- axs_s1_rid
    signal axi_reg_rdata        : std_logic_vector(31 downto 0) := (others => '0'); -- axs_s1_rdata
    signal axi_reg_rlast        : std_logic                     := '0';             -- axs_s1_rlast
    signal axi_reg_rvalid       : std_logic                     := '0';             -- axs_s1_rvalid
    signal axi_reg_rready       : std_logic;                                        -- axs_s1_rready
    signal axi_reg_clk          : std_logic;                                        -- clk_o
    signal axi_reg_reset        : std_logic;                                        -- reset_o

    signal fpga_led_internal    : std_logic_vector(3 downto 0);
    signal hps_fpga_reset_n     : std_logic;

    signal Bank1RegWr           : RegWrA_T;
    signal Bank2RegWr           : RegWrA_T;
    signal FIFORegWr            : FIFORegWrA_T;

    signal Bank2RegRd           : RegRd_A(255 downto 0);
    signal Bank1RegRd           : RegRd_A(255 downto 0);

    signal ctrlreg              : std_logic_vector(63 downto 0);

begin
    SoC : entity soc_system.soc_system
    port map (
        -- Clock & Reset
        clk_clk                                 => FPGA_CLK1_50,            --  clk.clk
        reset_reset_n                           => '1',                     --  reset.reset_n

        -- HPS DDR3
        memory_mem_a                            => HPS_DDR3_ADDR,           --  memory.mem_a
        memory_mem_ba                           => HPS_DDR3_BA,             -- .mem_ba
        memory_mem_ck                           => HPS_DDR3_CK_P,           -- .mem_ck
        memory_mem_ck_n                         => HPS_DDR3_CK_N,           -- .mem_ck_n
        memory_mem_cke                          => HPS_DDR3_CKE,            -- .mem_cke
        memory_mem_cs_n                         => HPS_DDR3_CS_N,           -- .mem_cs_n
        memory_mem_ras_n                        => HPS_DDR3_RAS_N,          -- .mem_ras_n
        memory_mem_cas_n                        => HPS_DDR3_CAS_N,          -- .mem_cas_n
        memory_mem_we_n                         => HPS_DDR3_WE_N,           -- .mem_we_n
        memory_mem_reset_n                      => HPS_DDR3_RESET_N,        -- .mem_reset_n
        memory_mem_dq                           => HPS_DDR3_DQ,             -- .mem_dq
        memory_mem_dqs                          => HPS_DDR3_DQS_P,          -- .mem_dqs
        memory_mem_dqs_n                        => HPS_DDR3_DQS_N,          -- .mem_dqs_n
        memory_mem_odt                          => HPS_DDR3_ODT,            -- .mem_odt
        memory_mem_dm                           => HPS_DDR3_DM,             -- .mem_dm
        memory_oct_rzqin                        => HPS_DDR3_RZQ,            -- .oct_rzqin                                  

        -- HPS ethernet      
        hps_0_hps_io_hps_io_emac1_inst_TX_CLK   => HPS_ENET_GTX_CLK,        -- .hps_0_hps_io.hps_io_emac1_inst_TX_CLK
        hps_0_hps_io_hps_io_emac1_inst_TXD0     => HPS_ENET_TX_DATA(0),     -- .hps_io_emac1_inst_TXD0
        hps_0_hps_io_hps_io_emac1_inst_TXD1     => HPS_ENET_TX_DATA(1),     -- .hps_io_emac1_inst_TXD1
        hps_0_hps_io_hps_io_emac1_inst_TXD2     => HPS_ENET_TX_DATA(2),     -- .hps_io_emac1_inst_TXD2
        hps_0_hps_io_hps_io_emac1_inst_TXD3     => HPS_ENET_TX_DATA(3),     -- .hps_io_emac1_inst_TXD3
        hps_0_hps_io_hps_io_emac1_inst_RXD0     => HPS_ENET_RX_DATA(0),     -- .hps_io_emac1_inst_RXD0
        hps_0_hps_io_hps_io_emac1_inst_RXD1     => HPS_ENET_RX_DATA(1),     -- .hps_io_emac1_inst_RXD1
        hps_0_hps_io_hps_io_emac1_inst_RXD2     => HPS_ENET_RX_DATA(2),     -- .hps_io_emac1_inst_RXD2
        hps_0_hps_io_hps_io_emac1_inst_RXD3     => HPS_ENET_RX_DATA(3),     -- .hps_io_emac1_inst_RXD3        
        hps_0_hps_io_hps_io_emac1_inst_MDIO     => HPS_ENET_MDIO,           -- .hps_io_emac1_inst_MDIO
        hps_0_hps_io_hps_io_emac1_inst_MDC      => HPS_ENET_MDC,            -- .hps_io_emac1_inst_MDC
        hps_0_hps_io_hps_io_emac1_inst_RX_CTL   => HPS_ENET_RX_DV,          -- .hps_io_emac1_inst_RX_CTL
        hps_0_hps_io_hps_io_emac1_inst_TX_CTL   => HPS_ENET_TX_EN,          -- .hps_io_emac1_inst_TX_CTL
        hps_0_hps_io_hps_io_emac1_inst_RX_CLK   => HPS_ENET_RX_CLK,         -- .hps_io_emac1_inst_RX_CLK

        -- HPS SD card 
        hps_0_hps_io_hps_io_sdio_inst_CLK       => HPS_SD_CLK,              -- .hps_io_sdio_inst_CLK
        hps_0_hps_io_hps_io_sdio_inst_CMD       => HPS_SD_CMD,              -- .hps_io_sdio_inst_CMD
        hps_0_hps_io_hps_io_sdio_inst_D0        => HPS_SD_DATA(0),          -- .hps_io_sdio_inst_D0
        hps_0_hps_io_hps_io_sdio_inst_D1        => HPS_SD_DATA(1),          -- .hps_io_sdio_inst_D1
        hps_0_hps_io_hps_io_sdio_inst_D2        => HPS_SD_DATA(2),          -- .hps_io_sdio_inst_D2
        hps_0_hps_io_hps_io_sdio_inst_D3        => HPS_SD_DATA(3),          -- .hps_io_sdio_inst_D3

        -- HPS USB         
        hps_0_hps_io_hps_io_usb1_inst_D0        => HPS_USB_DATA(0),         -- .hps_io_usb1_inst_D0
        hps_0_hps_io_hps_io_usb1_inst_D1        => HPS_USB_DATA(1),         -- .hps_io_usb1_inst_D1
        hps_0_hps_io_hps_io_usb1_inst_D2        => HPS_USB_DATA(2),         -- .hps_io_usb1_inst_D2
        hps_0_hps_io_hps_io_usb1_inst_D3        => HPS_USB_DATA(3),         -- .hps_io_usb1_inst_D3
        hps_0_hps_io_hps_io_usb1_inst_D4        => HPS_USB_DATA(4),         -- .hps_io_usb1_inst_D4
        hps_0_hps_io_hps_io_usb1_inst_D5        => HPS_USB_DATA(5),         -- .hps_io_usb1_inst_D5
        hps_0_hps_io_hps_io_usb1_inst_D6        => HPS_USB_DATA(6),         -- .hps_io_usb1_inst_D6
        hps_0_hps_io_hps_io_usb1_inst_D7        => HPS_USB_DATA(7),         -- .hps_io_usb1_inst_D7
        hps_0_hps_io_hps_io_usb1_inst_CLK       => HPS_USB_CLKOUT,          -- .hps_io_usb1_inst_CLK
        hps_0_hps_io_hps_io_usb1_inst_STP       => HPS_USB_STP,             -- .hps_io_usb1_inst_STP
        hps_0_hps_io_hps_io_usb1_inst_DIR       => HPS_USB_DIR,             -- .hps_io_usb1_inst_DIR
        hps_0_hps_io_hps_io_usb1_inst_NXT       => HPS_USB_NXT,             -- .hps_io_usb1_inst_NXT

        -- HPS SPI         
        hps_0_hps_io_hps_io_spim1_inst_CLK      => HPS_SPIM_CLK,            -- .hps_io_spim1_inst_CLK
        hps_0_hps_io_hps_io_spim1_inst_MOSI     => HPS_SPIM_MOSI,           -- .hps_io_spim1_inst_MOSI
        hps_0_hps_io_hps_io_spim1_inst_MISO     => HPS_SPIM_MISO,           -- .hps_io_spim1_inst_MISO
        hps_0_hps_io_hps_io_spim1_inst_SS0      => HPS_SPIM_SS,             -- .hps_io_spim1_inst_SS0

        -- HPS UART      
        hps_0_hps_io_hps_io_uart0_inst_RX       => HPS_UART_RX,             -- .hps_io_uart0_inst_RX
        hps_0_hps_io_hps_io_uart0_inst_TX       => HPS_UART_TX,             -- .hps_io_uart0_inst_TX

        -- HPS I2C1
        hps_0_hps_io_hps_io_i2c0_inst_SDA       => HPS_I2C0_SDAT,           -- .hps_io_i2c0_inst_SDA
        hps_0_hps_io_hps_io_i2c0_inst_SCL       => HPS_I2C0_SCLK,           -- .hps_io_i2c0_inst_SCL

        -- HPS I2C2
        hps_0_hps_io_hps_io_i2c1_inst_SDA       => HPS_I2C1_SDAT,           -- .hps_io_i2c1_inst_SDA
        hps_0_hps_io_hps_io_i2c1_inst_SCL       => HPS_I2C1_SCLK,           -- .hps_io_i2c1_inst_SCL

        -- HPS GPIO 
        hps_0_hps_io_hps_io_gpio_inst_GPIO09    => HPS_CONV_USB_N,          -- .hps_io_gpio_inst_GPIO09
        hps_0_hps_io_hps_io_gpio_inst_GPIO35    => HPS_ENET_INT_N,          -- .hps_io_gpio_inst_GPIO35
        hps_0_hps_io_hps_io_gpio_inst_GPIO40    => HPS_LTC_GPIO,            -- .hps_io_gpio_inst_GPIO40
        hps_0_hps_io_hps_io_gpio_inst_GPIO53    => HPS_LED,                 -- .hps_io_gpio_inst_GPIO53
        hps_0_hps_io_hps_io_gpio_inst_GPIO54    => HPS_KEY,                 -- .hps_io_gpio_inst_GPIO54
        hps_0_hps_io_hps_io_gpio_inst_GPIO61    => HPS_GSENSOR_INT,         -- .hps_io_gpio_inst_GPIO61

        -- HPS Resets
        hps_0_f2h_stm_hw_events_stm_hwevents    => (others=>'0'),           --  hps_0_f2h_stm_hw_events.stm_hwevents
        hps_0_h2f_reset_reset_n                 => hps_fpga_reset_n,        --  hps_0_h2f_reset.reset_n
        hps_0_f2h_warm_reset_req_reset_n        => '1',                     --  hps_0_f2h_warm_reset_req.reset_n    
        hps_0_f2h_debug_reset_req_reset_n       => '1',                     --  hps_0_f2h_debug_reset_req.reset_n  
        hps_0_f2h_cold_reset_req_reset_n        => '1',                     --  hps_0_f2h_cold_reset_req.reset_n
         
        -- AXI Conduit for register I/O
        axi_conduit_0_axi_out_axs_s1_awid       => axi_reg_awid,
        axi_conduit_0_axi_out_axs_s1_awaddr     => axi_reg_awaddr,
        axi_conduit_0_axi_out_axs_s1_awlen      => axi_reg_awlen,
        axi_conduit_0_axi_out_axs_s1_awsize     => axi_reg_awsize,
        axi_conduit_0_axi_out_axs_s1_awburst    => axi_reg_awburst,
        axi_conduit_0_axi_out_axs_s1_awvalid    => axi_reg_awvalid,
        axi_conduit_0_axi_out_axs_s1_awready    => axi_reg_awready,
        axi_conduit_0_axi_out_axs_s1_wdata      => axi_reg_wdata,
        axi_conduit_0_axi_out_axs_s1_wstrb      => axi_reg_wstrb,
        axi_conduit_0_axi_out_axs_s1_wvalid     => axi_reg_wvalid,
        axi_conduit_0_axi_out_axs_s1_wready     => axi_reg_wready,
        axi_conduit_0_axi_out_axs_s1_bid        => axi_reg_bid,
        axi_conduit_0_axi_out_axs_s1_bvalid     => axi_reg_bvalid,
        axi_conduit_0_axi_out_axs_s1_bready     => axi_reg_bready,
        axi_conduit_0_axi_out_axs_s1_arid       => axi_reg_arid,
        axi_conduit_0_axi_out_axs_s1_araddr     => axi_reg_araddr,
        axi_conduit_0_axi_out_axs_s1_arlen      => axi_reg_arlen,
        axi_conduit_0_axi_out_axs_s1_arsize     => axi_reg_arsize,
        axi_conduit_0_axi_out_axs_s1_arburst    => axi_reg_arburst,
        axi_conduit_0_axi_out_axs_s1_arvalid    => axi_reg_arvalid,
        axi_conduit_0_axi_out_axs_s1_arready    => axi_reg_arready,
        axi_conduit_0_axi_out_axs_s1_rid        => axi_reg_rid,
        axi_conduit_0_axi_out_axs_s1_rdata      => axi_reg_rdata,
        axi_conduit_0_axi_out_axs_s1_rlast      => axi_reg_rlast,
        axi_conduit_0_axi_out_axs_s1_rvalid     => axi_reg_rvalid,
        axi_conduit_0_axi_out_axs_s1_rready     => axi_reg_rready,
        axi_conduit_0_axi_out_clk_o             => axi_reg_clk,
        axi_conduit_0_axi_out_reset_o           => axi_reg_reset
    );


    axi_reg_wr : entity work.AXI_Reg_Wr_E
    port map (
        clk             => axi_reg_clk,
        rst             => axi_reg_reset,

        -- Write Address Channel
        axi_awid        => axi_reg_awid,
        axi_awaddr      => axi_reg_awaddr,
        axi_awlen       => axi_reg_awlen,
        axi_awsize      => axi_reg_awsize,
        axi_awburst     => axi_reg_awburst,
      --axi_awlock      : in  std_logic_vector(1 downto 0);
      --axi_awcache     : in  std_logic_vector(3 downto 0);
      --axi_awprot      : in  std_logic_vector(2 downto 0);
        axi_awvalid     => axi_reg_awvalid,
        axi_awready     => axi_reg_awready,

        -- Write Data Channel
        axi_wdata       => axi_reg_wdata,
        axi_wstrb       => axi_reg_wstrb,
      --axi_wlast       : in  std_logic;
        axi_wvalid      => axi_reg_wvalid,
        axi_wready      => axi_reg_wready,

        -- Write Response Channel
        axi_bid         => axi_reg_bid,
      --axi_bresp       : out std_logic_vector(1 downto 0);
        axi_bvalid      => axi_reg_bvalid,
        axi_bready      => axi_reg_bready,

        -- Register write interface
        Bank1RegWr      => Bank1RegWr,
        Bank2RegWr      => Bank2RegWr,
        FIFORegWr       => FIFORegWr );

    AXI_rd : entity work.AXI_Reg_Rd_E
    port map (
        clk             => axi_reg_clk,
        rst             => axi_reg_reset,

        -- Read Address Channel
        axi_arid        => axi_reg_arid,
        axi_araddr      => axi_reg_araddr,
        axi_arlen       => axi_reg_arlen,
        axi_arsize      => axi_reg_arsize,
        axi_arburst     => axi_reg_arburst,
        axi_arvalid     => axi_reg_arvalid,
        axi_arready     => axi_reg_arready,

        -- Read Data Channel
        axi_rid         => axi_reg_rid,
        axi_rdata       => axi_reg_rdata,
        axi_rlast       => axi_reg_rlast,
        axi_rvalid      => axi_reg_rvalid,
        axi_rready      => axi_reg_rready,

        -- Register read data
        Bank1RegRd      => Bank1RegRd,
        Bank2RegRd      => Bank2RegRd );

    -----------------------------
    -- Control register writes --
    -----------------------------
    process(axi_reg_clk)
    begin
        if rising_edge(axi_reg_clk) then
            if axi_reg_reset='1' then
                ctrlreg <= (others=>'0');
            else
                if Bank1RegWr.we(0)='1' then
                    for index in 0 to 63 loop
                        if Bank1RegWr.be(index/8)='1' then
                            ctrlreg(index)  <= Bank1RegWr.data(index);
                        end if;
                    end loop;
                end if;
            end if;
        end if;
    end process;

    Bank1RegRd  <= (0=>ctrlreg,1=>x"1111111111111111",2=>x"2222222222222222",others=>(others=>'0'));
    Bank2RegRd  <= (others=>(others=>'0'));

end arch;
