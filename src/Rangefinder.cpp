#include <Rangefinder/Rangefinder.h>

Rangefinder::Rangefinder()
{}

Rangefinder::~Rangefinder()
{}

void Rangefinder::open(std::string p)
{
    port = p;
    sensor = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (sensor == -1) {
        // Could not open the port.
        std::stringstream ss;
        ss << "[Serial] Unable to open port " << port;
        perror(ss.str().c_str());
    }
    else
        fcntl(sensor, F_SETFL, 0);

    // Get current serial port settings
    tcgetattr(sensor, &options);
    // Set 9600 baud both ways
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    // 8 bits, no parity, no stop bits
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    // Canonical mode 
    options.c_lflag |= ICANON;
    // Commit the serial port settings
    tcsetattr(sensor, TCSANOW, &options);

}

double Rangefinder::getRange()
{
    double sumOfDistance = 0;
    double number = 0;
    // Write to port
    int n = write(sensor, "1\0", 1);
    if (n < 0) fputs("write() of 1 bytes failed!\n", stderr);

    char buf[64];
    /* Receive string from Arduino */
    n = read(sensor, buf, sizeof(buf));
    /* insert terminating zero in the string */
    buf[n] = 0;
 
    std::string numStr;
    for (int i = 0; i < n; ++i) {
        if (buf[i] == ' ') {
            sumOfDistance += atof(numStr.c_str());
            ++number;
            numStr = "";
            continue;
        }
        numStr += buf[i];
    }
    return sumOfDistance/number;
}