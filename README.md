# MPED-DIY ![mped](/pics/mped.png)


MPED DIY DCC decoders

Based on the work of Geoff Bunza, I am designing my own miniature DIY decoders, which includes both PCB design and corresponding sketch.  Programming is done using NmraDcc library with the help of its author Alex Shepherd.

So far, two decoders have been designed:

1. **Basic Accessory Turnout Decoder**: the idea is to have a miniature decoder attached to each individual turnout. The decoder is to draw power and commands directly from the turnout tracks. This is useful for those model railroaders who do not have a fixed and permanent layout so that the position of a single turnout can change.
![dt2](/pics/dt2.png)
![dt2_pcb](/pics/dt2_pcb.png)

The result can be seen on [the video](https://youtu.be/xufySOCpvhE).  For more information, please refer to [this forum entry](https://forum.mrhmag.com/post/miniature-turnout-decoder-12219236).

2. **Basic Multifunction Locomotive Decoder**: The idea is to have a miniature decoder that uses all the NMRA mandated and recommended CVs (mprimary address, Vstart, manufacturer ID and version, packet time-out value, configuration data), as well as extended address, acceleration and deceleration rate, Vmid, Vhigh, speed table and light brightness control.
![dl2](/pics/dl2.png)
![dl2_pcb](/pics/dl2_pcb.png)

The result can be seen on [the video](https://youtu.be/2htTclTV8HQ).  For more instructions, please refer to [this forum entry](https://forum.mrhmag.com/post/miniature-locomotive-decoder-12277807).
