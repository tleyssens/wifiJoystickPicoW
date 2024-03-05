import wifi
import time
import motorAS5600

from ws_connection import ClientClosedError
from ws_server import WebSocketServer, WebSocketClient

stuurmotor = motorAS5600.MotorAS5600(0, 17, 16, 14, 15)
stuurmotor.setup(-25, 25, 30000)
deadline = time.ticks_add(time.ticks_ms(), 300)

class TestClient(WebSocketClient):
    def __init__(self, conn):
        super().__init__(conn)

    def process(self):
        try:
            msg = self.connection.read()
            if not msg:
                return
            msg = msg.decode("utf-8")
            msg = msg.split("\n")[-2]
            msg = msg.split(" ")

            deadline = time.ticks_add(time.ticks_ms(), 300)
            
            # tires.apply_power(int(msg[0]), int(msg[1]), int(msg[2]), int(msg[3]))
            zitmaaier.rijden(int(msg[0]), int(msg[1]), int(msg[2]), int(msg[3]))
        except ClientClosedError:
            print("Connection close error")
            self.connection.close()
        except Exception as e:
            print("exception:" + str(e) + "\n")
            raise e
                


class TestServer(WebSocketServer):
    def __init__(self):
        super().__init__("index.html", 100)

    def _make_client(self, conn):
        return TestClient(conn)

wifi.run()

server = TestServer()
server.start()

while True:
    zitmaaier.process()
    server.process_all()
    if time.ticks_diff(deadline, time.ticks_ms()) < 0:
        # tires.apply_power(0,0,0,0)
        zitmaaier.rijden(0,0,0,0)
        deadline = time.ticks_add(time.ticks_ms(), 100000)

server.stop()
