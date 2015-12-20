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

entity SoCKit_Reg is
    port (

        -------------------
        -- HPS Interface --
        -------------------

--      reset_reset_n                           : in    std_logic;
--      clk_clk                                 : in    std_logic;
        memory_mem_a                            : out   std_logic_vector(14 downto 0);
        memory_mem_ba                           : out   std_logic_vector(2 downto 0);
        memory_mem_ck                           : out   std_logic;
        memory_mem_ck_n                         : out   std_logic;
        memory_mem_cke                          : out   std_logic;
        memory_mem_cs_n                         : out   std_logic;
        memory_mem_ras_n                        : out   std_logic;
        memory_mem_cas_n                        : out   std_logic;
        memory_mem_we_n                         : out   std_logic;
        memory_mem_reset_n                      : out   std_logic;
        memory_mem_dq                           : inout std_logic_vector(31 downto 0);
        memory_mem_dqs                          : inout std_logic_vector(3 downto 0);
        memory_mem_dqs_n                        : inout std_logic_vector(3 downto 0);
        memory_mem_odt                          : out   std_logic;
        memory_mem_dm                           : out   std_logic_vector(3 downto 0);
        memory_oct_rzqin                        : in    std_logic;
        hps_0_hps_io_hps_io_emac1_inst_TX_CLK   : out   std_logic;
        hps_0_hps_io_hps_io_emac1_inst_TXD0     : out   std_logic;
        hps_0_hps_io_hps_io_emac1_inst_TXD1     : out   std_logic;
        hps_0_hps_io_hps_io_emac1_inst_TXD2     : out   std_logic;
        hps_0_hps_io_hps_io_emac1_inst_TXD3     : out   std_logic;
        hps_0_hps_io_hps_io_emac1_inst_RXD0     : in    std_logic;
        hps_0_hps_io_hps_io_emac1_inst_MDIO     : inout std_logic;
        hps_0_hps_io_hps_io_emac1_inst_MDC      : out   std_logic;
        hps_0_hps_io_hps_io_emac1_inst_RX_CTL   : in    std_logic;
        hps_0_hps_io_hps_io_emac1_inst_TX_CTL   : out   std_logic;
        hps_0_hps_io_hps_io_emac1_inst_RX_CLK   : in    std_logic;
        hps_0_hps_io_hps_io_emac1_inst_RXD1     : in    std_logic;
        hps_0_hps_io_hps_io_emac1_inst_RXD2     : in    std_logic;
        hps_0_hps_io_hps_io_emac1_inst_RXD3     : in    std_logic;
        hps_0_hps_io_hps_io_qspi_inst_IO0       : inout std_logic;
        hps_0_hps_io_hps_io_qspi_inst_IO1       : inout std_logic;
        hps_0_hps_io_hps_io_qspi_inst_IO2       : inout std_logic;
        hps_0_hps_io_hps_io_qspi_inst_IO3       : inout std_logic;
        hps_0_hps_io_hps_io_qspi_inst_SS0       : out   std_logic;
        hps_0_hps_io_hps_io_qspi_inst_CLK       : out   std_logic;
        hps_0_hps_io_hps_io_sdio_inst_CMD       : inout std_logic;
        hps_0_hps_io_hps_io_sdio_inst_D0        : inout std_logic;
        hps_0_hps_io_hps_io_sdio_inst_D1        : inout std_logic;
        hps_0_hps_io_hps_io_sdio_inst_CLK       : out   std_logic;
        hps_0_hps_io_hps_io_sdio_inst_D2        : inout std_logic;
        hps_0_hps_io_hps_io_sdio_inst_D3        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D0        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D1        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D2        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D3        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D4        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D5        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D6        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_D7        : inout std_logic;
        hps_0_hps_io_hps_io_usb1_inst_CLK       : in    std_logic;
        hps_0_hps_io_hps_io_usb1_inst_STP       : out   std_logic;
        hps_0_hps_io_hps_io_usb1_inst_DIR       : in    std_logic;
        hps_0_hps_io_hps_io_usb1_inst_NXT       : in    std_logic;
        hps_0_hps_io_hps_io_spim0_inst_CLK      : out   std_logic;
        hps_0_hps_io_hps_io_spim0_inst_MOSI     : out   std_logic;
        hps_0_hps_io_hps_io_spim0_inst_MISO     : in    std_logic;
        hps_0_hps_io_hps_io_spim0_inst_SS0      : out   std_logic;
        hps_0_hps_io_hps_io_spim1_inst_CLK      : out   std_logic;
        hps_0_hps_io_hps_io_spim1_inst_MOSI     : out   std_logic;
        hps_0_hps_io_hps_io_spim1_inst_MISO     : in    std_logic;
        hps_0_hps_io_hps_io_spim1_inst_SS0      : out   std_logic;
        hps_0_hps_io_hps_io_uart0_inst_RX       : in    std_logic;
        hps_0_hps_io_hps_io_uart0_inst_TX       : out   std_logic;
        hps_0_hps_io_hps_io_i2c1_inst_SDA       : inout std_logic;
        hps_0_hps_io_hps_io_i2c1_inst_SCL       : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO00    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO09    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO35    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO48    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO53    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO54    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO55    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO56    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO61    : inout std_logic;
        hps_0_hps_io_hps_io_gpio_inst_GPIO62    : inout std_logic;

        --------------------
        -- FPGA Interface --
        --------------------

        -- Clocks
        clk_100m_fpga               : in    std_logic;                  -- 2.5V     100.00 MHz
        clk_50m_fpga                : in    std_logic;                  -- 2.5V      50.00 MHz
        clk_top1                    : in    std_logic;                  -- 2.5V     156.25 MHz adjustable
        clk_bot1                    : in    std_logic;                  -- 1.5V     100.00 MHz adjustable
        fpga_resetn                 : in    std_logic;                  -- 2.5V     FPGA Reset Pushbutton   

        -- SiLabs Clock Generator
        clk_i2c_sclk                : inout std_logic;                  -- I2C Clock 
        clk_i2c_sdat                : inout std_logic;                  -- I2C Data 
                 
        -- FPGA User I/O : 14 pins
        user_dipsw_fpga             : in    std_logic_vector(3 downto 0);
        user_led_fpga               : out   std_logic_vector(3 downto 0);
        user_pb_fpga                : in    std_logic_vector(3 downto 0);
        irda_rxd                    : in    std_logic;                  -- IRDA Receive LED   
        fan_ctrl                    : out   std_logic;                  -- control for fan
    
--      -- FPGA DDR3 400MHzx32bit I/O : 74 pins
--      ddr3_fpga_a                 : out   std_logic_vector(14:0);     -- SSTL15  --Address
--      ddr3_fpga_ba                : out   std_logic_vector(2:0) ;     -- SSTL15  --Bank Address
--      ddr3_fpga_casn              : out   std_logic;                  -- SSTL15  --Column Address Strobe
--      ddr3_fpga_cke               : out   std_logic;                  -- SSTL15  --Clock Enable
--      ddr3_fpga_clk_n             : out   std_logic;                  -- SSTL15  --Diff Clock - Neg
--      ddr3_fpga_clk_p             : out   std_logic;                  -- SSTL15  --Diff Clock - Pos
--      ddr3_fpga_csn               : out   std_logic;                  -- SSTL15  --Chip Select
--      ddr3_fpga_dm                : out   std_logic_vector(3:0) ;     -- SSTL15  --Data Write Mask
--      ddr3_fpga_dq                : inout std_logic_vector(31:0);     -- SSTL15  --Data Bus
--      ddr3_fpga_dqs_n             : inout std_logic_vector(3:0) ;     -- SSTL15  --Diff Data Strobe - Neg
--      ddr3_fpga_dqs_p             : inout std_logic_vector(3:0) ;     -- SSTL15  --Diff Data Strobe - Pos
--      ddr3_fpga_odt               : out   std_logic;                  -- SSTL15  --On-Die Termination Enable
--      ddr3_fpga_rasn              : out   std_logic;                  -- SSTL15  --Row Address Strobe
--      ddr3_fpga_resetn            : out   std_logic;                  -- SSTL15  --Reset
--      ddr3_fpga_wen               : out   std_logic;                  -- SSTL15  --Write Enable
--      ddr3_fpga_rzq               : in    std_logic;                  -- OCT_rzqin --On-die termination enable
--    --oct_rdn                     :  in   std_logic;                  -- SSTL15    --On-die termination enable
--    --oct_rup                     :  in   std_logic;                  -- SSTL15    --On-die termination enable
 
        -- Temperature sensor (SPI interface)
        temp_cs_n                   : out   std_logic;                  -- Chip Select
        temp_sclk                   : out   std_logic;                  -- Slave Clock 
        temp_mosi                   : out   std_logic;                  -- Data Out 
        temp_miso                   : in    std_logic;                  -- Data In
 
        -- VGA Out : 29 pins
        vga_clk                     : out   std_logic;                      -- Video Clock
        vga_hs                      : out   std_logic;                      -- Horizontal Sync
        vga_vs                      : out   std_logic;                      -- Vertical Sync          
        vga_r                       : out   std_logic_vector(7 downto 0);   -- Red 
        vga_g                       : out   std_logic_vector(7 downto 0);   -- Green
        vga_b                       : out   std_logic_vector(7 downto 0);   -- Blue 
        vga_blank_n                 : out   std_logic;                      -- Blanking
        vga_sync_n                  : out   std_logic;                      -- Composite Sync

        -- Audio I/O : 9 pins
        aud_adcdat                  : in    std_logic;                  -- ADC Serial Data or I2C_SCLK
        aud_adclrck                 : in    std_logic;                  -- FDDR3e clock
        aud_bclk                    : in    std_logic;                  -- Bit Clock 
        aud_dacdat                  : out   std_logic;                  -- DAC Serial Data 
        aud_daclrck                 : inout std_logic;                  -- FDDR3e Clock
        aud_i2c_sclk                : out   std_logic;                                
        aud_i2c_sdat                : inout std_logic;                                   
        aud_mute                    : out   std_logic;                           
        aud_xck                     : out   std_logic;

        -- HSMC
--    --hsmc_clkin_n                : in    std_logic_vector(2 downto 1);         
--      hsmc_clkin_p                : in    std_logic_vector(2 downto 1);         
--    --hsmc_clkout_n               : out   std_logic_vector(2 downto 1);              
--      hsmc_clkout_p               : out   std_logic_vector(2 downto 1);         
--      hsmc_clk_in0                : in    std_logic;         
--      hsmc_clk_out0               : out   std_logic;              
--      hsmc_d                      : inout std_logic_vector(3 downto 0);         
--
--    --hsmc_gxb_rx_n               : in    std_logic_vector(7 downto 0);       
--      hsmc_gxb_rx_p               : in    std_logic_vector(7 downto 0);      
--    --hsmc_gxb_tx_n               : out   std_logic_vector(7 downto 0);       
--      hsmc_gxb_tx_p               : out   std_logic_vector(7 downto 0);       
--    --hsmc_ref_clk_n              : in    std_logic;       
--      hsmc_ref_clk_p              : in    std_logic;       
--
--    --hsmc_rx_n                   : in    std_logic_vector(16 downto 0); 
--      hsmc_rx_p                   : in    std_logic_vector(16 downto 0);   
--      hsmc_scl                    : out   std_logic;  
--      hsmc_sda                    : inout std_logic;  
--    --hsmc_tx_n                   : out   std_logic_vector(16 downto 0); 
--      hsmc_tx_p                   : out   std_logic_vector(16 downto 0); 


        -- QSPI Flash
        fpga_epqc_data              : inout std_logic_vector(3 downto 0) ;  -- Flash Data 
        fpga_epqc_dclk              : inout std_logic;                      -- Data Clock  
        fpga_epqc_ncso              : inout std_logic                       -- Chip Select

);

end SoCKit_Reg;

architecture arch of SoCKit_Reg is
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
        clk_clk                               => clk_bot1,
        reset_reset_n                         => hps_fpga_reset_n,
        memory_mem_a                          => memory_mem_a,
        memory_mem_ba                         => memory_mem_ba,
        memory_mem_ck                         => memory_mem_ck,
        memory_mem_ck_n                       => memory_mem_ck_n,
        memory_mem_cke                        => memory_mem_cke,
        memory_mem_cs_n                       => memory_mem_cs_n,
        memory_mem_ras_n                      => memory_mem_ras_n,
        memory_mem_cas_n                      => memory_mem_cas_n,
        memory_mem_we_n                       => memory_mem_we_n,
        memory_mem_reset_n                    => memory_mem_reset_n,
        memory_mem_dq                         => memory_mem_dq,
        memory_mem_dqs                        => memory_mem_dqs,
        memory_mem_dqs_n                      => memory_mem_dqs_n,
        memory_mem_odt                        => memory_mem_odt,
        memory_mem_dm                         => memory_mem_dm,
        memory_oct_rzqin                      => memory_oct_rzqin,
        hps_0_hps_io_hps_io_emac1_inst_TX_CLK => hps_0_hps_io_hps_io_emac1_inst_TX_CLK,
        hps_0_hps_io_hps_io_emac1_inst_TXD0   => hps_0_hps_io_hps_io_emac1_inst_TXD0,
        hps_0_hps_io_hps_io_emac1_inst_TXD1   => hps_0_hps_io_hps_io_emac1_inst_TXD1,
        hps_0_hps_io_hps_io_emac1_inst_TXD2   => hps_0_hps_io_hps_io_emac1_inst_TXD2,
        hps_0_hps_io_hps_io_emac1_inst_TXD3   => hps_0_hps_io_hps_io_emac1_inst_TXD3,
        hps_0_hps_io_hps_io_emac1_inst_RXD0   => hps_0_hps_io_hps_io_emac1_inst_RXD0,
        hps_0_hps_io_hps_io_emac1_inst_MDIO   => hps_0_hps_io_hps_io_emac1_inst_MDIO,
        hps_0_hps_io_hps_io_emac1_inst_MDC    => hps_0_hps_io_hps_io_emac1_inst_MDC,
        hps_0_hps_io_hps_io_emac1_inst_RX_CTL => hps_0_hps_io_hps_io_emac1_inst_RX_CTL,
        hps_0_hps_io_hps_io_emac1_inst_TX_CTL => hps_0_hps_io_hps_io_emac1_inst_TX_CTL,
        hps_0_hps_io_hps_io_emac1_inst_RX_CLK => hps_0_hps_io_hps_io_emac1_inst_RX_CLK,
        hps_0_hps_io_hps_io_emac1_inst_RXD1   => hps_0_hps_io_hps_io_emac1_inst_RXD1,
        hps_0_hps_io_hps_io_emac1_inst_RXD2   => hps_0_hps_io_hps_io_emac1_inst_RXD2,
        hps_0_hps_io_hps_io_emac1_inst_RXD3   => hps_0_hps_io_hps_io_emac1_inst_RXD3,
        hps_0_hps_io_hps_io_qspi_inst_IO0     => hps_0_hps_io_hps_io_qspi_inst_IO0,
        hps_0_hps_io_hps_io_qspi_inst_IO1     => hps_0_hps_io_hps_io_qspi_inst_IO1,
        hps_0_hps_io_hps_io_qspi_inst_IO2     => hps_0_hps_io_hps_io_qspi_inst_IO2,
        hps_0_hps_io_hps_io_qspi_inst_IO3     => hps_0_hps_io_hps_io_qspi_inst_IO3,
        hps_0_hps_io_hps_io_qspi_inst_SS0     => hps_0_hps_io_hps_io_qspi_inst_SS0,
        hps_0_hps_io_hps_io_qspi_inst_CLK     => hps_0_hps_io_hps_io_qspi_inst_CLK,
        hps_0_hps_io_hps_io_sdio_inst_CMD     => hps_0_hps_io_hps_io_sdio_inst_CMD,
        hps_0_hps_io_hps_io_sdio_inst_D0      => hps_0_hps_io_hps_io_sdio_inst_D0,
        hps_0_hps_io_hps_io_sdio_inst_D1      => hps_0_hps_io_hps_io_sdio_inst_D1,
        hps_0_hps_io_hps_io_sdio_inst_CLK     => hps_0_hps_io_hps_io_sdio_inst_CLK,
        hps_0_hps_io_hps_io_sdio_inst_D2      => hps_0_hps_io_hps_io_sdio_inst_D2,
        hps_0_hps_io_hps_io_sdio_inst_D3      => hps_0_hps_io_hps_io_sdio_inst_D3,
        hps_0_hps_io_hps_io_usb1_inst_D0      => hps_0_hps_io_hps_io_usb1_inst_D0,
        hps_0_hps_io_hps_io_usb1_inst_D1      => hps_0_hps_io_hps_io_usb1_inst_D1,
        hps_0_hps_io_hps_io_usb1_inst_D2      => hps_0_hps_io_hps_io_usb1_inst_D2,
        hps_0_hps_io_hps_io_usb1_inst_D3      => hps_0_hps_io_hps_io_usb1_inst_D3,
        hps_0_hps_io_hps_io_usb1_inst_D4      => hps_0_hps_io_hps_io_usb1_inst_D4,
        hps_0_hps_io_hps_io_usb1_inst_D5      => hps_0_hps_io_hps_io_usb1_inst_D5,
        hps_0_hps_io_hps_io_usb1_inst_D6      => hps_0_hps_io_hps_io_usb1_inst_D6,
        hps_0_hps_io_hps_io_usb1_inst_D7      => hps_0_hps_io_hps_io_usb1_inst_D7,
        hps_0_hps_io_hps_io_usb1_inst_CLK     => hps_0_hps_io_hps_io_usb1_inst_CLK,
        hps_0_hps_io_hps_io_usb1_inst_STP     => hps_0_hps_io_hps_io_usb1_inst_STP,
        hps_0_hps_io_hps_io_usb1_inst_DIR     => hps_0_hps_io_hps_io_usb1_inst_DIR,
        hps_0_hps_io_hps_io_usb1_inst_NXT     => hps_0_hps_io_hps_io_usb1_inst_NXT,
        hps_0_hps_io_hps_io_spim0_inst_CLK    => hps_0_hps_io_hps_io_spim0_inst_CLK,
        hps_0_hps_io_hps_io_spim0_inst_MOSI   => hps_0_hps_io_hps_io_spim0_inst_MOSI,
        hps_0_hps_io_hps_io_spim0_inst_MISO   => hps_0_hps_io_hps_io_spim0_inst_MISO,
        hps_0_hps_io_hps_io_spim0_inst_SS0    => hps_0_hps_io_hps_io_spim0_inst_SS0,
        hps_0_hps_io_hps_io_spim1_inst_CLK    => hps_0_hps_io_hps_io_spim1_inst_CLK,
        hps_0_hps_io_hps_io_spim1_inst_MOSI   => hps_0_hps_io_hps_io_spim1_inst_MOSI,
        hps_0_hps_io_hps_io_spim1_inst_MISO   => hps_0_hps_io_hps_io_spim1_inst_MISO,
        hps_0_hps_io_hps_io_spim1_inst_SS0    => hps_0_hps_io_hps_io_spim1_inst_SS0,
        hps_0_hps_io_hps_io_uart0_inst_RX     => hps_0_hps_io_hps_io_uart0_inst_RX,
        hps_0_hps_io_hps_io_uart0_inst_TX     => hps_0_hps_io_hps_io_uart0_inst_TX,
        hps_0_hps_io_hps_io_i2c1_inst_SDA     => hps_0_hps_io_hps_io_i2c1_inst_SDA,
        hps_0_hps_io_hps_io_i2c1_inst_SCL     => hps_0_hps_io_hps_io_i2c1_inst_SCL,
        hps_0_hps_io_hps_io_gpio_inst_GPIO00  => hps_0_hps_io_hps_io_gpio_inst_GPIO00,
        hps_0_hps_io_hps_io_gpio_inst_GPIO09  => hps_0_hps_io_hps_io_gpio_inst_GPIO09,
        hps_0_hps_io_hps_io_gpio_inst_GPIO35  => hps_0_hps_io_hps_io_gpio_inst_GPIO35,
        hps_0_hps_io_hps_io_gpio_inst_GPIO48  => hps_0_hps_io_hps_io_gpio_inst_GPIO48,
        hps_0_hps_io_hps_io_gpio_inst_GPIO53  => hps_0_hps_io_hps_io_gpio_inst_GPIO53,
        hps_0_hps_io_hps_io_gpio_inst_GPIO54  => hps_0_hps_io_hps_io_gpio_inst_GPIO54,
        hps_0_hps_io_hps_io_gpio_inst_GPIO55  => hps_0_hps_io_hps_io_gpio_inst_GPIO55,
        hps_0_hps_io_hps_io_gpio_inst_GPIO56  => hps_0_hps_io_hps_io_gpio_inst_GPIO56,
        hps_0_hps_io_hps_io_gpio_inst_GPIO61  => hps_0_hps_io_hps_io_gpio_inst_GPIO61,
        hps_0_hps_io_hps_io_gpio_inst_GPIO62  => hps_0_hps_io_hps_io_gpio_inst_GPIO62,

        dipsw_pio_external_connection_export  => user_dipsw_fpga,
        hps_0_h2f_reset_reset_n               => hps_fpga_reset_n,
        fpga_led_pio_export                   => fpga_led_internal,
        fpga_button_pio_export                => user_pb_fpga,
        axi_conduit_0_axi_out_axs_s1_awid     => axi_reg_awid,
        axi_conduit_0_axi_out_axs_s1_awaddr   => axi_reg_awaddr,
        axi_conduit_0_axi_out_axs_s1_awlen    => axi_reg_awlen,
        axi_conduit_0_axi_out_axs_s1_awsize   => axi_reg_awsize,
        axi_conduit_0_axi_out_axs_s1_awburst  => axi_reg_awburst,
        axi_conduit_0_axi_out_axs_s1_awvalid  => axi_reg_awvalid,
        axi_conduit_0_axi_out_axs_s1_awready  => axi_reg_awready,
        axi_conduit_0_axi_out_axs_s1_wdata    => axi_reg_wdata,
        axi_conduit_0_axi_out_axs_s1_wstrb    => axi_reg_wstrb,
        axi_conduit_0_axi_out_axs_s1_wvalid   => axi_reg_wvalid,
        axi_conduit_0_axi_out_axs_s1_wready   => axi_reg_wready,
        axi_conduit_0_axi_out_axs_s1_bid      => axi_reg_bid,
        axi_conduit_0_axi_out_axs_s1_bvalid   => axi_reg_bvalid,
        axi_conduit_0_axi_out_axs_s1_bready   => axi_reg_bready,
        axi_conduit_0_axi_out_axs_s1_arid     => axi_reg_arid,
        axi_conduit_0_axi_out_axs_s1_araddr   => axi_reg_araddr,
        axi_conduit_0_axi_out_axs_s1_arlen    => axi_reg_arlen,
        axi_conduit_0_axi_out_axs_s1_arsize   => axi_reg_arsize,
        axi_conduit_0_axi_out_axs_s1_arburst  => axi_reg_arburst,
        axi_conduit_0_axi_out_axs_s1_arvalid  => axi_reg_arvalid,
        axi_conduit_0_axi_out_axs_s1_arready  => axi_reg_arready,
        axi_conduit_0_axi_out_axs_s1_rid      => axi_reg_rid,
        axi_conduit_0_axi_out_axs_s1_rdata    => axi_reg_rdata,
        axi_conduit_0_axi_out_axs_s1_rlast    => axi_reg_rlast,
        axi_conduit_0_axi_out_axs_s1_rvalid   => axi_reg_rvalid,
        axi_conduit_0_axi_out_axs_s1_rready   => axi_reg_rready,
        axi_conduit_0_axi_out_clk_o           => axi_reg_clk,
        axi_conduit_0_axi_out_reset_o         => axi_reg_reset
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


    -- SiLabs Clock Generator
    clk_i2c_sclk    <= '1';
    clk_i2c_sdat    <= '1';

    -- FPGA User I/O
    user_led_fpga(0)    <= ctrlreg(0);
    user_led_fpga(1)    <= ctrlreg(1);
    user_led_fpga(2)    <= fpga_led_internal(2);
    user_led_fpga(3)    <= fpga_led_internal(3);
    fan_ctrl            <= '0';


    -- Audio I/O : 9 pins
    aud_dacdat      <= '0';
    aud_daclrck     <= '0';
    aud_i2c_sclk    <= '0';
    aud_i2c_sdat    <= '0';
    aud_mute        <= '0';
    aud_xck         <= '0';

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
