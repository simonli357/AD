from server import Server
import time

server = Server(49153)
server.initialize()
server.get_client().send_string("TEST") # String test

while True:
    time.sleep(1)
