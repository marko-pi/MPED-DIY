# MPED-DIY ![mped](https://user-images.githubusercontent.com/18025812/131258018-895c02df-3534-46fb-b06c-0407fd3e26a1.png)


MPED DIY DCC decoders

Based on the work of Geoff Bunza, I am designing my own miniature DIY decoders, which includes both PCB design and corresponding sketch.  Programming is done using NmraDcc library with the help of its author Alex Shepherd.

So far, two decoders have been designed:

1. **Basic Accessory Turnout Decoder**: the idea is to have a miniature decoder attached to each individual turnout. The decoder is to draw power and commands directly from the turnout tracks. This is useful for those model railroaders who do not have a fixed and permanent layout so that the position of a single turnout can change.
![dt1](https://user-images.githubusercontent.com/18025812/131260893-7c2ca922-2275-4c6e-8305-605aa7da473d.png)
![dt1_pcb](https://user-images.githubusercontent.com/18025812/133302838-1e73dd06-535a-4579-9187-ca837db30098.png)

For more instructions, please refer to [this forum entry](https://model-railroad-hobbyist.com/node/43451).

2. **Basic Multifunction Locomotive Decoder**: The idea is to have a miniature decoder that uses all the NMRA mandated and recommended CVs (motor acceleration, deceleration, signal timeout, manufacturer ID), as well as CV120 for master reset.
![dl1](https://user-images.githubusercontent.com/18025812/131259720-8a87d864-df40-4d0d-986d-fecaba1cd111.png)
