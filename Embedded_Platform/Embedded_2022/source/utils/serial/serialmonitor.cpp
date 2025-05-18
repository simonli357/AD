#include <utils/serial/serialmonitor.hpp>
#include <cstdio>


namespace utils::serial{

SerialMonitor::SerialMonitor(BufferedSerial& serial, const SerialSubscriberMap& callbacks)
    : serial_(serial), callbacks_(callbacks), thread_(osPriorityNormal, 2048, nullptr, "SerMon")
{
    // printf("[SerialMonitor] Initialized\n");
}

SerialMonitor::~SerialMonitor() {
    stop();
    // printf("[SerialMonitor] Stopped and destroyed\n");
}

void SerialMonitor::start(std::chrono::milliseconds sleepDuration) {
    sleepDuration_ = sleepDuration;
    running_ = true;
    thread_.start(callback(this, &SerialMonitor::readerThreadFn));
    // printf("[SerialMonitor] Thread started\n");
}

void SerialMonitor::stop() {
    running_ = false;
    thread_.join();
    // printf("[SerialMonitor] Thread joined\n");
}

void SerialMonitor::readerThreadFn() {
    // printf("[SerialMonitor] Reader thread running\n");

    char tempBuf[32];  // Temporary buffer to read multiple bytes at once

    while (running_) {
        ssize_t n = serial_.read(tempBuf, sizeof(tempBuf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; ++i) {
                char c = tempBuf[i];

                if (c == '#' && !inFrame_) {
                    inFrame_ = true;
                    bufIndex_ = 0;
                    frameBuf_[bufIndex_] = c;
                    continue;
                }

                if (inFrame_) {
                    if (bufIndex_ < (frameBuf_.size() - 5)) { // reserve for ";;\r\n" and '\0'
                        frameBuf_[++bufIndex_] = c;
                        if (bufIndex_ >= 3 &&
                            frameBuf_[bufIndex_]   == '\n' &&
                            frameBuf_[bufIndex_-1] == '\r' &&
                            frameBuf_[bufIndex_-2] == ';'  &&
                            frameBuf_[bufIndex_-3] == ';') 
                        {
                            frameBuf_[bufIndex_-3] = '\0';
                            // printf("[SerialMonitor] Complete frame: %s\n", frameBuf_.data());
                            processFrame();
                            inFrame_ = false;
                            bufIndex_ = 0;
                        }
                    } else {
                        // printf("[SerialMonitor] Buffer overflow\n");
                        inFrame_ = false;
                        bufIndex_ = 0;
                    }
                }
            }
        }
        ThisThread::sleep_for(sleepDuration_);
    }

    // printf("[SerialMonitor] Reader thread exiting\n");
}

void SerialMonitor::processFrame() {
    // printf("[SerialMonitor] Processing frame: %s\n", frameBuf_.data()); // frameBuf_ now contains e.g., "#13:0.00:0.00\0;;\r\n..."

    char keyBuf[3]  = {0};
    char msg1[256]  = {0};
    char msg2[256]  = {0};

    int parts = std::sscanf(frameBuf_.data(), "#%2[^:]:%255[^:]:%255[^\0]", keyBuf, msg1, msg2);

    std::string key(keyBuf);
    static char payloadBuf[512];

    if (parts == 3) {
        std::snprintf(payloadBuf, sizeof payloadBuf, "%s:%s", msg1, msg2);
        // printf("[SerialMonitor] Parsed 3 parts: key='%s', msg1='%s', msg2='%s'\n", key.c_str(), msg1, msg2);
    } else if (parts == 2) {
        std::snprintf(payloadBuf, sizeof payloadBuf, "%s", msg1);
        // printf("[SerialMonitor] Parsed 2 parts: key='%s', msg1='%s'\n", key.c_str(), msg1);
     } else if (parts == 1) {
        payloadBuf[0] = '\0';
        // printf("[SerialMonitor] Parsed 1 part: key='%s'\n", key.c_str());
    }
     else {
        // printf("[SerialMonitor] Parse error (sscanf returned %d parts)\n", parts);
        return; // Exit processing if parse failed
    }

    // printf("[SerialMonitor] Final parsed key='%s', payload='%s'\n", key.c_str(), payload.c_str());

    auto it = callbacks_.find(keyBuf);
    if (it != callbacks_.end()) {
        // printf("[SerialMonitor] Dispatching callback for '%s'\n", key.c_str());
        static char response[256];
        response[0] = '\0';
        it->second(payloadBuf, response);
        // printf("[SerialMonitor] Callback response: '%s'\n", response);
        // if (response[0] != '\0') { // Check if response is not empty
        //     char out[300] = {0}; // Ensure output buffer is adequate
        //     std::sn// printf(out, sizeof(out), "@%s:%s;;\r\n", keyBuf, response);
        //     // printf("[SerialMonitor] Sending response: %s", out); // Log includes newline from format
        //     txMutex_.lock();
        //     serial_.write(out, strlen(out));
        //     txMutex_.unlock();
        // } else {
        //      // printf("[SerialMonitor] Callback for '%s' provided no response to send.\n", key.c_str());
        // }
    } else {
        // printf("[SerialMonitor] No callback registered for key '%s'\n", key.c_str());
    }
}

}