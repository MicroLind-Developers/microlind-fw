# Firmware for PS/2 & RTC controller on the microLind board.

The controller uses a 32kHz crystal on the Timer2 input to trigger an interrupt every second that counts up a RTC register.
There is a 8-bit data bus, 3-bit address and 5 control signals: ENABLE, KB_IRQ, MOUSE_IRQ, RD and WR.
The register layout (seen from the microLind) is:

    $F408   Base address

    $F408	KPR	Get current key press	Send Command To Keyboard
	$F409	MR	Get current mouse byte	
	$F40A	RTCY	Current Year	Set Year
	$F40B	RTCM	Bit 0-3: Current Month.  Bit 5-7. Error Flags	Set Month
	$F40C	RTCD	Bit 0-4: Current Day.  Bit 5-7: DayOfWeek	Set  Bit 0-4: Day.  Bit 5-7: DayOfWeek
	$F40D	RTCH	Current Hour	Set Hour
	$F40E	RTCMI	Current Minute	Set Minute
	$F40F	RTCS	Current Second	Set Second