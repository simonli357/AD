from server import Server
import time

server = Server(49153)
server.initialize()
server.get_client().send_string("TEST")  # String test

while True:
    if server.get_client().strings:
        print(server.get_client().strings.pop(0))
    if server.get_client().images:
        img_msg = server.get_client().images.pop(0)
        print(f"Received Image - Height: {img_msg.height}, Width: {img_msg.width}")
    time.sleep(1)
