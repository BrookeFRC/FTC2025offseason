## Hello! 
my name is brooke (Patrick from school) and this is Metro's FTC code for the 2025 offseason (into the deep)

if you're here then it means i probably sent/shared you this code.
if you found this by accident, please do not out me to friends/family/school

## this code uses multiple imports and other libraries 
* FTC LIB
  * adds more code
  * PID
* FTC Dashboard
  * can run code from a laptop connected to the robot via wifi
* FTControl (this is experimental as of 5/14/25)
  * should be like FTC dash but mainly for Kotlin, may or may not use
* Pedros Pathing 
  * PP
  * mecanum auto pathing
  * unimplemented as of 5/14/25
  * plan to add it by the 2025/2026 season
  * harder setup than RR but easier usage
* Road runner
  * RR
  * motion profiling and PID
  * all implementations are commented out as of 5/14/25
  * easier setup than PP but harder usage
* Photon
  * not implimented
  * deprecated
  * should have made things faster

## slang and definitions
- commit: updates the entire project, your changed files (or the ones you select) will become the most recent official files
- PID: proportional, integral, derivative. we really only use P and D. it is a simple way to control motors.
- Telementry: sends out real time data to driver station/Dash/FTControl
- Encoder: gets the current positoin of the input (motor), not the output (shaft/arm/gearbox), in one rotation of the motor there are 28 "counts" (ticks or pulses can be used as well)
- 