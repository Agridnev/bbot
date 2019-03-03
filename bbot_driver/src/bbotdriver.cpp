#include "bbotdriver.h"

BBotDriver::BBotDriver(const std::string &port, unsigned int baud, const CallbackData &cbData, const CallbackStop &cbStop) :
    port(port), baud(baud), cbData(cbData), cbStop(cbStop), io(), serial(io), writeTimer(io), motors{0}, motorsPending(false), velocity{0, 0}
{
}

void BBotDriver::start()
{
    serial.open(port);
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud));

    readAsync();
    writeTimerWait();

    boost::thread t(boost::bind(&BBotDriver::run, this));
}

void BBotDriver::stop() {
    io.stop();
}

void BBotDriver::setMotors(const Motors &motors)
{
    this->motors = motors;
    motorsPending = true;
}

void BBotDriver::setVelocity(const Velocity &velocity)
{
    this->velocity = velocity;
}

void BBotDriver::readAsync()
{
    serial.async_read_some(boost::asio::buffer(readBuffer, sizeof(readBuffer)), boost::bind(&BBotDriver::readHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void BBotDriver::readHandler(const boost::system::error_code &error, size_t bytesTransferred)
{
    if (error)
    {
        io.stop();
        return;
    }
    if (bytesTransferred > 0)
    {
        deserializer.append(readBuffer, bytesTransferred);
        deserializer.deserialize();
        while (deserializer.getDLength())
        {
            if (deserializer.getDLength() == sizeof(Data))
            {
                if (cbData)
                {
                    cbData(*(const Data *) deserializer.getDData());
                }
            }
            deserializer.deserialize();
        }
    }
    readAsync();
}

void BBotDriver::writeTimerWait()
{
    writeTimer.expires_from_now(boost::posix_time::milliseconds(25));
    writeTimer.async_wait(boost::bind(&BBotDriver::writeTimerHandler, this));
}

void BBotDriver::writeTimerHandler()
{
    if (motorsPending)
    {
        motorsPending = false;
        writeAsyncMotors();
    }
    else
    {
        writeAsyncVelocity();
    }
}

void BBotDriver::writeAsyncMotors()
{
    serializer.serialize((unsigned char *) &motors, sizeof(motors));
    boost::asio::async_write(serial, boost::asio::buffer(serializer.getSData(), serializer.getSLength()),
                             boost::bind(&BBotDriver::writeHandlerMotors, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void BBotDriver::writeAsyncVelocity()
{
    serializer.serialize((unsigned char *) &velocity, sizeof(velocity));
    boost::asio::async_write(serial, boost::asio::buffer(serializer.getSData(), serializer.getSLength()),
                             boost::bind(&BBotDriver::writeHandlerVelocity, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void BBotDriver::writeHandlerMotors(const boost::system::error_code &error, size_t bytesTransferred)
{
    if (error || bytesTransferred != serializer.getSLength())
    {
        io.stop();
        return;
    }
    writeAsyncVelocity();
}

void BBotDriver::writeHandlerVelocity(const boost::system::error_code &error, size_t bytesTransferred)
{
    if (error || bytesTransferred != serializer.getSLength())
    {
        io.stop();
        return;
    }
    writeTimerWait();
}

void BBotDriver::run()
{
    io.run();
    if (cbStop)
    {
        cbStop();
    }
}

