#include <iostream>
#include <cstring>

using namespace std;

// Define a callback function for handling received messages
void handleMessage(const char* msgID, const char* msg1, const char* msg2, char* response) {
    // Dummy function for testing, not used in this standalone test
}

void testMessageParsing(const char* message) {
    // Temporary variables to store parsed message components
    char msgID[2]; // Assuming max length of msgID is 4 characters
    char msg1[256]; // Assuming max length of msg1 is 255 characters
    char msg2[256]; // Assuming max length of msg2 is 255 characters

    // Parse the message
    int res = sscanf(message, "#%1[^:]:%255[^:]:%255[^;];;", msgID, msg1, msg2);

    // Check parsing result
    if (res == 3) {
        cout << "Parsed successafully:" << endl;
        cout << "msgID: " << msgID << endl;
        cout << "msg1: " << msg1 << endl;
        cout << "msg2: " << msg2 << endl;
        char msg[512];
        sprintf(msg, "%s:%s", msg1, msg2);
        cout << "Concatenated message: " << msg << endl;
        float float1, float2;
        res = sscanf(msg, "%f:%f", &float1, &float2);
        if (res == 2) {
            cout << "Parsed back to floats:" << endl;
            cout << "Float 1: " << float1 << endl;
            cout << "Float 2: " << float2 << endl;
        } else {
            cout << "Parsing back to floats failed." << endl;
        }
    } else {
        // Attempt parsing with one part
        res = sscanf(message, "#%1[^:]:%255[^;];;", msgID, msg1);
        if (res == 2) {
            cout << "Parsed successfully:" << endl;
            cout << "msgID: " << msgID << endl;
            cout << "msg: " << msg1 << endl;
        } else {
            cout << "Parsing failed." << endl;
        }
    }
}


int main() {
    // Test message with two parts
    const char* message1 = "#8:25.5:-10.4;;\r\n";
    cout << "Testing message 1: " << message1 << endl;
    testMessageParsing(message1);

    // Test message with one part
    const char* message2 = "#2:-10.9;;\r\n";
    cout << "Testing message 2: " << message2 << endl;
    testMessageParsing(message2);

    // Test invalid message
    const char* message3 = "#3:-10.9;;\r\n";
    cout << "Testing invalid message: " << message3 << endl;
    testMessageParsing(message3);

    return 0;
}
