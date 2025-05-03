#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

using namespace std;

string speed(float f_velocity) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_velocity);
    strs << "#" << "1" << ":" << buff;
    return strs.str();
}

string steer(float f_angle) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_angle);
    strs << "#" << "2" << ":" << buff;
    return strs.str();
}

string both(float f_velocity, float f_angle) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f:%.2f;;\r\n", f_velocity, f_angle);
    strs << "#" << "8" << ":" << buff;
    return strs.str();
}

string compute(float f_velocity, float f_angle) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f:%.2f;;\r\n", f_velocity, f_angle);
    strs << "#" << "13" << ":" << buff;
    return strs.str();
}

string setPID(float active, float kp, float ki, float kd) {
    stringstream strs;
    char buff[100];
    snprintf(buff, sizeof(buff), "%.2f:%.2f:%.2f:%.2f;;\r\n", active, kp, ki, kd);
    strs << "#" << "12" << ":" << buff;
    return strs.str();
}

vector<pair<double, double>> readFile(const string& filePath) {
    vector<pair<double, double>> data;
    ifstream file(filePath);

    if (!file) {
        cerr << "File not found!" << endl;
        return data;
    }

    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        string value1, value2;
        
        if (getline(ss, value1, ',') && getline(ss, value2, ',')) {
            try {
                double velocity = stod(value1);
                double angle = stod(value2);
                data.emplace_back(velocity, angle);
            } catch (const invalid_argument& e) {
                cerr << "Skipping invalid line: " << line << endl;
            }
        } else {
            cerr << "Skipping invalid line: " << line << endl;
        }
    }

    return data;
}

int main() {
    boost::asio::io_service io;
    boost::asio::serial_port serial(io);

    serial.open("/dev/ttyACM0");
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));

    string filepath = "/home/malo/Documents/Simulator/src/example/src/left_turn_commands0202.txt";
    vector<pair<double, double>> data = readFile(filepath);

    // Set the PID
    string pid_msg = setPID(1, 1.25, 0.625, 0.15125);
    cout << "pid msg sent: " << pid_msg << endl;
    boost::asio::write(serial, boost::asio::buffer(pid_msg));

    for (const auto& [velocity, angle] : data) {
        string message = compute(velocity, angle);
        cout << "steer msg sent: " << message << endl;
        boost::asio::write(serial, boost::asio::buffer(message));
        this_thread::sleep_for(chrono::milliseconds(100));  // 10Hz delay
    }

    // string speed_msg = speed(0.0);
    // cout << "speed msg sent: " << speed_msg << endl;
    // boost::asio::write(serial, boost::asio::buffer(speed_msg));

    // string steer_msg = steer(0.0);
    // cout << "steer msg sent: " << steer_msg << endl;
    // boost::asio::write(serial, boost::asio::buffer(steer_msg));

    string both_msg = both(0.0, 0.0);
    cout << "both msg sent: " << both_msg << endl;
    boost::asio::write(serial, boost::asio::buffer(both_msg));

    serial.close();

    return 0;
}
