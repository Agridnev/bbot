#include <string>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "serialdatagram.h"

class BBotDriver
{
public:

    struct Data
    {
        long int x;
        long int y;
        short int th;
        short int linear;
        short int angular;
        unsigned char enabled;
        unsigned char voltage;
    };

    struct Motors
    {
        unsigned char enable;
    };

    struct Velocity
    {
        short int linear;
        short int angular;
    };

    typedef boost::function<void(const Data&)> CallbackData;
    typedef boost::function<void()> CallbackStop;

    BBotDriver(const std::string &port, unsigned int baud, const CallbackData &cbData = 0, const CallbackStop &cbStop = 0);
    void start();
    void stop();
    void setMotors(const Motors &motors);
    void setVelocity(const Velocity &velocity);

private:
    const std::string port;
    const unsigned int baud;
    const CallbackData cbData;
    const CallbackStop cbStop;
    Motors motors;
    bool motorsPending;
    Velocity velocity;

    boost::asio::io_service io;
    boost::asio::serial_port serial;

    sd::Serializer serializer;
    sd::Deserializer deserializer;

    unsigned char readBuffer[64];
    void readAsync();
    void readHandler(const boost::system::error_code& error, size_t bytesTransferred);

    boost::asio::deadline_timer writeTimer;
    void writeTimerWait();
    void writeTimerHandler();
    void writeAsyncMotors();
    void writeAsyncVelocity();
    void writeHandlerMotors(const boost::system::error_code &error, size_t bytesTransferred);
    void writeHandlerVelocity(const boost::system::error_code &error, size_t bytesTransferred);

    void run();
};

