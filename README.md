# The Wee O3
## An ionized O2 MiniPop
*The Wee O3* is an Arduino Nano based emulation of a [Korg Mini Pops](https://en.wikipedia.org/wiki/Korg_Mini_Pops) drum machine.  
The O3 follows Jan Ostman's O2 from 2016 with additional features and optimizations for the Korg SQ-1 sequencer.  
[Video with feature demonstration](https://youtu.be/vjLT6ayJKSc)
In addition to the standard 16 drum patterns and 8 samples the following features are provided:
## New Features
* Analog pitch control via potentiometer or CV input. Changing the pitch will change the tempo as well because pitch control is achieved by changing the interrupt timing.
* Combined CV / potentiometer input for selecting a sample (rising edge) and triggering it (falling edge). Compatible with the **littleBits OUT** of the Korg SQ-1 sequencer.
* Manual S Trigger input for triggering the selected sample.
* Bi directional clocking: Turning tempo all the way down disables internal clock and switches system to clock input.
* CV input for tempo control.
* CV input for drum pattern selection.
* Double functionality for A0-A3: Sequence counter when running, pattern number when stopped. LEDs or a display connected to these pins make pattern selection easier.
## Hardware
While the schematics in this repo provide full functionality, the circuit can be easily reduced and optimized.
## Credits
Thanks to Jan Ostman for the original code and design.
# Enjoy - and get into the O Zone !
