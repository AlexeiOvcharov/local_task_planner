#ifndef RANGEFINDER
#define RANGEFINDER

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <string>
#include <sstream>
#include <iostream>
#include <stdlib.h>

class Rangefinder {
    public:
        Rangefinder();
        ~Rangefinder();

        void open(std::string p);
        double getRange();
    private:

        int sensor;             // File descriptor for the port
        struct termios options; // Serial port configuration
        std::string port;
};

#endif /* RANGEFIDER */