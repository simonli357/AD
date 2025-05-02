#include <drivers/serialmonitor.hpp>

namespace drivers {

/** @brief  CSerialMonitor class constructor
 *
 *  @param f_serialPort          reference to serial object
 *  @param f_serialSubscriberMap map with the key and the callback functions
 */
CSerialMonitor::CSerialMonitor(
        UnbufferedSerial& f_serialPort,
        CSerialSubscriberMap f_serialSubscriberMap)
    : utils::CTask(0)
    , m_serialPort(f_serialPort)
    , m_RxBuffer()
    , m_TxBuffer()
    , m_parseBuffer()
    , m_parseIt(m_parseBuffer.begin())
    , m_serialSubscriberMap(f_serialSubscriberMap)
{
    // RX interrupt-driven, non-blocking receive
    m_serialPort.attach(mbed::callback(this, &CSerialMonitor::serialRxCallback), SerialBase::RxIrq);
    // TX interrupt-driven, non-blocking transmit
    m_serialPort.attach(mbed::callback(this, &CSerialMonitor::serialTxCallback), SerialBase::TxIrq);
}

/** @brief  CSerialMonitor class destructor */
CSerialMonitor::~CSerialMonitor() {}

/** @brief  Rx callback actions (interrupt-driven) */
void CSerialMonitor::serialRxCallback() {
    __disable_irq();
    while (m_serialPort.readable() && !m_RxBuffer.isFull()) {
        char buf;
        m_serialPort.read(&buf, 1);
        m_RxBuffer.push(buf);
    }
    __enable_irq();
}

/** @brief  Tx callback actions (interrupt-driven) */
void CSerialMonitor::serialTxCallback() {
    __disable_irq();
    // Drain as many bytes as possible
    while (m_serialPort.writeable() && !m_TxBuffer.isEmpty()) {
        char c = m_TxBuffer.pop();
        m_serialPort.write(&c, 1);
    }
    __enable_irq();
}

/** @brief  Monitoring function (called periodically) */
void CSerialMonitor::_run() {
    // Process incoming data
    if (!m_RxBuffer.isEmpty()) {
        char l_c = m_RxBuffer.pop();

        // Detect start of message
        if (l_c == '#') {
            m_parseIt = m_parseBuffer.begin();
            *m_parseIt++ = l_c;
            return;
        }

        // Accumulate until end marker
        if (m_parseIt != m_parseBuffer.end()) {
            if (l_c == '\n' && *(m_parseIt - 3) == ';' && *(m_parseIt - 2) == ';' && *(m_parseIt - 1) == '\r') {
                // Parse key and payload
                char l_msgID[64] = {0};
                char l_msg[256]  = {0};
                uint32_t res = sscanf(m_parseBuffer.data(), "#%[^:]:%[^
]", l_msgID, l_msg);

                if (res == 2) {
                    auto it = m_serialSubscriberMap.find(l_msgID);
                    if (it != m_serialSubscriberMap.end()) {
                        char l_resp[256] = {0};
                        // Invoke user callback
                        it->second(l_msg, l_resp);

                        // Format response frame
                        char formattedResp[300];
                        int  len = snprintf(formattedResp, sizeof(formattedResp), "@%s:%s;;\r\n", l_msgID, l_resp);
                        if (len > 0) {
                            // Queue response into TX buffer
                            __disable_irq();
                            for (int i = 0; i < len && !m_TxBuffer.isFull(); ++i) {
                                m_TxBuffer.push(formattedResp[i]);
                            }
                            __enable_irq();

                            // Kick-start transmission if idle
                            if (m_serialPort.writeable()) {
                                serialTxCallback();
                            }
                        }
                    }
                }

                // Reset parse buffer
                m_parseIt = m_parseBuffer.begin();
            }

            // Store character
            *m_parseIt++ = l_c;
        }
    }
}

} // namespace drivers
