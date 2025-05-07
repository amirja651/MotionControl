#ifndef CLI_MANAGER_H
#define CLI_MANAGER_H

#include <SimpleCLI.h>

SimpleCLI cli;
Command   cmdMotor;
Command   cmdRestart;
Command   cmdHelp;
Command   cmdMotor1Limits;   // New command for Motor 1 limits
Command   cmdMotor1RefHome;  // New command for Motor 1 reference home

void initializeCLI()
{
    cmdMotor = cli.addCmd("motor");
    cmdMotor.addArg("n", "1");      // motor number argument
    cmdMotor.addFlagArg("l");       // motor load position argument (um or deg)
    cmdMotor.addFlagArg("c");       // current position
    cmdMotor.addFlagArg("s");       // stop flag
    cmdMotor.addArg("o", "680");    // motor offset argument or target reference (pixels)
    cmdMotor.addArg("lo", "550");   // lower limit argument (pixels)
    cmdMotor.addArg("up", "-900");  // upper limit argument (pixels)
    cmdMotor.addArg("p", "0.0");    // positional argument (um or deg)

    cmdMotor.setDescription(" motor1 p 3.0 [n: motor number] [-d: degree] [-m: millimeters] [-u: micrometers] [-s: stop]");

    cmdRestart = cli.addCmd("restart");
    cmdRestart.setDescription("Restart the ESP32 system");

    cmdHelp = cli.addCmd("help");
    cmdHelp.setDescription("Show help information");
}

#endif