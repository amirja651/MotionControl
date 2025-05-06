#ifndef CLI_MANAGER_H
#define CLI_MANAGER_H

#include <SimpleCLI.h>

SimpleCLI cli;
Command   cmdMotor;
Command   cmdRestart;
Command   cmdHelp;

void initializeCLI()
{
    cmdMotor = cli.addCmd("motor");
    cmdMotor.addArg("n", "1");    // motor number argument
    cmdMotor.addArg("o", "0.0");  // motor offset argument
    cmdMotor.addArg("p", "0.0");  // positional argument
    cmdMotor.addFlagArg("d");     // degree flag
    cmdMotor.addFlagArg("u");     // um     flag
    cmdMotor.addFlagArg("s");     // stop flag
    cmdMotor.setDescription(" motor1 p 3.0 [n: motor number] [-d: degree] [-m: millimeters] [-u: micrometers] [-s: stop]");

    cmdRestart = cli.addCmd("restart");
    cmdRestart.setDescription("Restart the ESP32 system");

    cmdHelp = cli.addCmd("help");
    cmdHelp.setDescription("Show help information");
}

#endif