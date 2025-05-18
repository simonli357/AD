#ifndef SERIAL_MONITOR_HPP
#define SERIAL_MONITOR_HPP

#include "mbed.h"
#include "rtos.h"
#include <map>
#include <string>
#include <array>

namespace utils::serial {
    using Callback = mbed::Callback<void(const char*, char*)>;
    using SerialSubscriberMap = std::map<std::string, Callback>;

    /**
     * @brief Threaded Serial Monitor for RTOS with debug tracing
     *
     * Reads incoming bytes, frames commands of the form:
     *    #KEY:PAYLOAD;;\r\n
     * and dispatches to registered callbacks.  Responses are sent
     * as: @KEY:RESPONSE;;\r\n
     */
    class SerialMonitor {
    public:
        SerialMonitor(BufferedSerial& serial, const SerialSubscriberMap& callbacks);
        ~SerialMonitor();

        /** Start RTOS thread */
        void start(std::chrono::milliseconds sleepDuration);
        /** Stop and join thread */
        void stop();

    private:
        void readerThreadFn();
        void processFrame();

        BufferedSerial&    serial_;             ///< UART interface
        SerialSubscriberMap  callbacks_;          ///< key->callback map
        rtos::Thread         thread_;             ///< background reader thread
        rtos::Mutex          txMutex_;            ///< protect serial writes
        std::array<char,256> frameBuf_{};         ///< frame assembly buffer
        size_t               bufIndex_ = 0;       ///< current frame index
        bool                 inFrame_ = false;    ///< assembling a frame?
        bool                 running_ = false;    ///< thread control
        std::chrono::milliseconds sleepDuration_{10};
    };
}; // namespace drivers

#endif // SERIAL_MONITOR_HPP