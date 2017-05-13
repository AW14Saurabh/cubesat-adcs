Control Box - code for blue rectangular control box
- rfController - sends motor on/off signal + angle position of potentiometer over RF
- rfControllerPtDet - sends motor on/off + detumble/point + position of pot over RF

RWS with Ctrl - code for RWS that requires signal from Control Box
- detumbleOrPointRF - detumbles or points depending on control signal
- pointRFPot - used in conjunction to rfController - desired angle determined by angle of 
- switchDetumble - detumbles when rfController switch is turned on

RWS with no Ctrl
- point180 - RWS points to 180 from its starting position
- simpleDetumble - detumbles
