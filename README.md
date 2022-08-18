# ESPHome ADS1115 customized component

This is a customized component for the ADS1115 component to better suit my needs. The builtin component does not support setting
the SPS rate which can be desireable for some use cases over the fixed high speed rate used. There is also 
logic that adds an arbitrary delay to try and ensure the register is populated in continuous mode. When adjusting
SPS, that delay is likely not correct. In Singleshot mode we can rely on the ready bit being set and can wait for it.
