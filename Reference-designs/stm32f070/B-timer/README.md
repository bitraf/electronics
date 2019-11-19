# B-Timer with STM32F070(C6)

## Making the board
The _/PCB_ directory contains KiCad files for reference. See _.pro_ or _.sch_ and _.kicad_pcb_ as starting points.

You can easily generate a _.pos_ file for use with the pick-and-place machine:

1. In the _.kicad_pcb_ file, set the "auxiliary axis origin" (_Place > Drill and Place Offset_). This point should be at the exact top left corner of the board edge, which by default shows as a thick yellow line on the "Edge.Cuts" layer.
2. Export by going to _File > Fabrication Outputs > Footprint Position (.pos) File_.

You can choose to mount the LED's in whichever direction you choose (up or down), but make sure to adjust the code accordingly. See the following section for more details.

## Flashing the board
You will need a programmer like the _ST-Link V2_ to (easily) flash your board. Make sure you connect the four pins (GND, VCC, SWDIO, SWCLK) properly.

The _/Software_ directory contains the files needed to flash your board with basic timer functionality. Assuming you are using Linux or a similar OS, the steps are:

1. Navigate to _/Software_.
1. Run `make` to compile the code.
1. Run `flash` to send it to the board.
1. If the board shows digits as expected, congrats! If it shows gibberish, check the `set_led` function in _/Software/Src/main.c_ and comment/uncomment the relevant lines (search for `@IMPORTANT`) depending on the orientation of your LED's. Remember to compile before you flash again. Lastly, if nothing shows up or something is not behaving properly, remember that the issue may be with the hardware (poorly connected pins, misaligned components, etc.).

Notes:
1. This particular implementation of `flash` in the Makefile uses `st-link`/`st-flash`. You can follow these steps ([A](https://www.youtube.com/watch?v=5nr_3o7D1To), [B](https://www.youtube.com/watch?v=pHyz2-wbDw4)) for how to install it. Alternatively, you can use `openocd` or any other software compatible with ARM chips (i.e. `avrdude` is _not_ compatible with ARM).
2. _/PCB/Software_ (not the same directory as _/Software_) contains scripts to generate digits for the display. You can safely ignore it if you are still getting familiar with this project.