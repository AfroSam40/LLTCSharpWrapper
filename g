/////////////////////////////////////////////////
// ScanMove() FUNCTION
/////////////////////////////////////////////////
{FUNCTION ScanMove}
VAR_EXTERNAL
    R300 : LREAL; // ScanStartX
    R301 : LREAL; // ScanEndX
    R302 : LREAL; // ScanZ
    R303 : LREAL; // ScanFeed
    R304 : LREAL; // LeadInDist
    R305 : LREAL; // SafeZ
    R306 : LREAL; // RapidFeed
END_VAR

VAR
    xLeadIn : LREAL;
    xLeadOut : LREAL;
END_VAR

    // Compute lead in/out
    xLeadIn  := R300 - R304;
    xLeadOut := R301 + R304;

    // 1) Go safe
    !G90
    !G01 Z=R305 F=R306
    sync()

    // 2) Move to lead-in start
    !G01 X=xLeadIn F=R306
    sync()

    // 3) Go to scan height
    !G01 Z=R302 F=R306
    sync()

    // 4) Arm scan device (PLC handles this M-code)
    !M60
    sync()

    // 5) Do scan move (steady feed)
    !G01 X=xLeadOut F=R303
    sync()

    // 6) Disarm scan
    !M61
    sync()

    // 7) Back to safe
    !G01 Z=R305 F=R306
    sync()

END_FUNCTION}