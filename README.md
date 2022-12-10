# MPED-DIY ![mped](https://user-images.githubusercontent.com/18025812/131258018-895c02df-3534-46fb-b06c-0407fd3e26a1.png)


MPED DIY DCC decoders

Based on the work of Geoff Bunza, I am designing my own miniature DIY decoders, which includes both PCB design and corresponding sketch.  Programming is done using NmraDcc library with the help of its author Alex Shepherd.

So far, two decoders have been designed:

1. **Basic Accessory Turnout Decoder**: the idea is to have a miniature decoder attached to each individual turnout. The decoder is to draw power and commands directly from the turnout tracks. This is useful for those model railroaders who do not have a fixed and permanent layout so that the position of a single turnout can change.
![dt2](https://user-images.githubusercontent.com/18025812/206309128-0244096a-380d-45d8-8992-dc7b896f88c1.png)
![dt2_pcb](https://user-images.githubusercontent.com/18025812/206309175-2e30b9ca-da33-494c-9778-359803405835.png)

The result can be seen on [the video](https://youtu.be/xufySOCpvhE).  For more information, please refer to [this forum entry](https://forum.mrhmag.com/post/miniature-turnout-decoder-12219236).

2. **Basic Multifunction Locomotive Decoder**: The idea is to have a miniature decoder that uses all the NMRA mandated and recommended CVs (mprimary address, Vstart, manufacturer ID and version, packet time-out value, configuration data), as well as extended address, acceleration and deceleration rate, Vmid, Vhigh and speed table.
![dl1](https://user-images.githubusercontent.com/18025812/131259720-8a87d864-df40-4d0d-986d-fecaba1cd111.png)
![dl1_pcb](https://user-images.githubusercontent.com/18025812/155277658-5c986a22-a540-4ed9-aca0-de515e29ec1d.png)

For more instructions, please refer to [this forum entry](https://forum.mrhmag.com/post/miniature-locomotive-decoder-12277807).
