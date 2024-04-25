# Pedalinator recap

## THINGS DONE
- correctly integrated the i2s module and made the notes play
- integrated DMA so CPU can do other things
- the pitch shifting algorithm works (very well also)
- keep long note without audio artifacts (also this works very well)
- have the device recognized as cdc


## TODO 1
- fix pitch shifting also with time squashing
- play the 4 notes decently as one would expect (without delays and strange artifacts)


## TODO 2
- have the device recognized as midi
- play a random midi to see if something works and is played on the PC side
- integrate a midi library to send based on the keys pressed
- make it work well and responsive to the keys pressed


## TODO 3
- use the DSP core to do interpolation calculation etc...
