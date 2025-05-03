// #include "serialmonitor.hpp"
// #include <cstdio>

// namespace drivers {

// SerialMonitor::SerialMonitor(BufferedSerial& serial, const SerialSubscriberMap& callbacks)
//     : serial_(serial), callbacks_(callbacks), thread_(osPriorityNormal, 2048, nullptr, "SerMon")
// {
//     printf("[SerialMonitor] Initialized\n");
// }

// SerialMonitor::~SerialMonitor() {
//     stop();
//     printf("[SerialMonitor] Stopped and destroyed\n");
// }

// void SerialMonitor::start() {
//     running_ = true;
//     thread_.start(callback(this, &SerialMonitor::readerThreadFn));
//     printf("[SerialMonitor] Thread started\n");
// }

// void SerialMonitor::stop() {
//     running_ = false;
//     thread_.join();
//     printf("[SerialMonitor] Thread joined\n");
// }

// void SerialMonitor::readerThreadFn() {
//     printf("[SerialMonitor] Reader thread running\n");
//     while (running_) {
//         char c;
//         ssize_t n = serial_.read(&c, 1);

//         if (n > 0) { // Byte received
//             if (c == '#' && !inFrame_) {
//                 inFrame_ = true;
//                 bufIndex_ = 0;
//                 frameBuf_[bufIndex_] = c; // Store '#' at index 0
//                 continue;
//             }

//             if (inFrame_) {
//                 if (bufIndex_ < (frameBuf_.size() - 2)) {
//                     frameBuf_[++bufIndex_] = c;
//                     if (bufIndex_ >= 3 && // Need at least 4 characters total (e.g., index 0, 1, 2, 3) for ";;\r\n"
//                         frameBuf_[bufIndex_]   == '\n' && // Current char is '\n'
//                         frameBuf_[bufIndex_-1] == '\r' && // Previous char is '\r'
//                         frameBuf_[bufIndex_-2] == ';'  && // Char before that is ';'
//                         frameBuf_[bufIndex_-3] == ';')    // Char before that is ';'
//                     {
//                         printf("[SerialMonitor] Terminator sequence \";;\\r\\n\" detected.\n");
//                         // frameBuf_[bufIndex_ - 3] = '\0';

//                         printf("[SerialMonitor] Complete frame content for processing: %s\n", frameBuf_.data());
//                         processFrame();

//                         frameBuf_[bufIndex_-3] = '\0';
//                         inFrame_ = false;
//                         bufIndex_ = 0; // Ready for a new '#'
//                         continue;
//                     } 
//                 } else {
//                     printf("[SerialMonitor] Error: Frame buffer overflow! Discarding frame.\n");
//                     inFrame_ = false;
//                     bufIndex_ = 0;
//                     // Optional: Clean buffer: frameBuf_.fill(0);
//                 }
//             }
//         } else { // n <= 0, means no byte was read
//             ThisThread::sleep_for(1ms); // Adjust sleep time if needed
//         }
//         ThisThread::sleep_for(1ms); // Prevent busy waiting
//     } 
//     printf("[SerialMonitor] Reader thread exiting\n");
// }

// void SerialMonitor::processFrame() {
//     printf("[SerialMonitor] Processing frame: %s\n", frameBuf_.data()); // frameBuf_ now contains e.g., "#13:0.00:0.00\0;;\r\n..."

//     char keyBuf[3] = {0}; // Key limited to 2 chars + null
//     char msg1[256] = {0}; // Adapt size as needed
//     char msg2[256] = {0}; // Adapt size as needed

//     int parts = std::sscanf(frameBuf_.data(), "#%2[^:]:%255[^:]:%255[^\0]", keyBuf, msg1, msg2);

//     std::string key(keyBuf);
//     std::string payload;

//     if (parts == 3) {
//         payload = std::string(msg1) + ":" + msg2;
//         printf("[SerialMonitor] Parsed 3 parts: key='%s', msg1='%s', msg2='%s'\n", key.c_str(), msg1, msg2);
//     } else if (parts == 2) {
//         payload = msg1;
//         printf("[SerialMonitor] Parsed 2 parts: key='%s', msg1='%s'\n", key.c_str(), msg1);
//      } else if (parts == 1) {
//         payload = ""; // No payload
//         printf("[SerialMonitor] Parsed 1 part: key='%s'\n", key.c_str());
//     }
//      else {
//         printf("[SerialMonitor] Parse error (sscanf returned %d parts)\n", parts);
//         return; // Exit processing if parse failed
//     }

//     printf("[SerialMonitor] Final parsed key='%s', payload='%s'\n", key.c_str(), payload.c_str());

//     auto it = callbacks_.find(key);
//     if (it != callbacks_.end()) {
//         printf("[SerialMonitor] Dispatching callback for '%s'\n", key.c_str());
//         static char response[256] = {0};
//         it->second(payload.c_str(), response); // Call the registered callback
//         printf("[SerialMonitor] Callback response: '%s'\n", response);
//         // if (response[0] != '\0') { // Check if response is not empty
//         //     char out[300] = {0}; // Ensure output buffer is adequate
//         //     std::snprintf(out, sizeof(out), "@%s:%s;;\r\n", keyBuf, response);
//         //     printf("[SerialMonitor] Sending response: %s", out); // Log includes newline from format
//         //     txMutex_.lock();
//         //     serial_.write(out, strlen(out));
//         //     txMutex_.unlock();
//         // } else {
//         //      printf("[SerialMonitor] Callback for '%s' provided no response to send.\n", key.c_str());
//         // }
//     } else {
//         printf("[SerialMonitor] No callback registered for key '%s'\n", key.c_str());
//     }
// }

// }