# AXI_Reg
AXI to generic register interface

This project is intended to tie the AXI on-chip bus (as found on Altera
SoC and Xilinx Zynq parts) to a generic register interface, ultimately
supporting the use of the open source hostmot2 VHDL motion control code
from Mesa.net.

This project is licensed under a dual-clause GPL 2+ and BSD 3-clause
license, matching the existing hostmot2 code from Mesa.

An example project (DE0_Nano_Reg) is included to show basic usage of
the AXI to register bridge code.  The project targets the Atlas/Nano
board from Altera/Terasic and the SoC configuration is based on the
my_first_hps-fpga_base example project from the Terasic system CD:
DE0-Nano_SoC_v.1.0.9_SystemCD/Demonstrations/SoC_FPGA/my_first_hps-fpga_base/

```
To build the example project:
  Open the DE0_Nano_Reg project with Quartus 14.1
  Launch QSys and open soc_system.qsys
  Click "Generate HDL..." button
    Under Synthesis, select VHDL output
    VHDL synthesis files should be generated in soc_system/
  Compile the project as usual
```
```
ToDo:
  Convert to 32-bit interface (existing code supports 64-bit registers)
x Integrate into Altera GHRD reference design
+   Add files to this project?
  Add hostmot2 files
    Convert any Xilinx-isms to Altera
    Figure out how to connect top-level pins and flexible configurations
    Connect register bus to hostmot2
```
