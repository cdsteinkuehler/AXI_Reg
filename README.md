# AXI_Reg
AXI to generic register interface

This project is intended to tie the AXI on-chip bus (as found on Altera
SoC and Xilinx Zynq parts) to a generic register interface, ultimately
supporting the use of the open source hostmot2 VHDL motion control code
from Mesa.net.

This project is licensed under a dual-clause GPL 2+ and BSD 3-clause
license, matching the existing hostmot2 code from Mesa.

ToDo:
  Convert to 32-bit interface (existing code supports 64-bit registers)
  Integrate into Altera GHRD reference design
    Add files to this project?
  Add hostmot2 files
    Convert any Xilinx-isms to Altera
    Figure out how to connect top-level pins and flexible configurations
    Connect register bus to hostmot2