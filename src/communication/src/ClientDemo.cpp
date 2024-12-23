#include <Client.hpp>
#include <unistd.h>

int main (int argc, char *argv[]) {
    Client client("127.0.0.1", 49153);
    client.initialize();
    client.send_string("TEST"); // String test
    while (true) {
        sleep(1);
    }
    return 0;
}
