import wifi
import time
import motor
import as5600
import zitmaaier

from ws_connection import ClientClosedError
from ws_server import WebSocketServer, WebSocketClient
motorStuur = motor.Motor(14, 15)
motorVoorAchter = motor.Motor(10, 11)
AS5600Stuur = as5600.AS5600(0, 16, 17)
AS5600VoorAchter = as5600.AS5600(1, 18, 19)

zitmaaier = zitmaaier.Zitmaaier(motorStuur, motorVoorAchter, AS5600Stuur, AS5600VoorAchter)
zitmaaier.setMinMaxStuur(-25,25)
zitmaaier.setMaxPwmStuur(30000)
zitmaaier.setMinMaxVoorAchter(-10, 10)
zitmaaier.setMaxPwmVoorAchter(30000)
zitmaaier.motorStuur.myPID.Kp = 10000
zitmaaier.motorStuur.myPID.Ki = 7000
zitmaaier.motorStuur.myPID.Kd = 5

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
try:
    while True:
        zitmaaier.process()
        server.process_all()
        if time.ticks_diff(deadline, time.ticks_ms()) < 0:
            # tires.apply_power(0,0,0,0)
            zitmaaier.rijden(0,0,0,0)
            deadline = time.ticks_add(time.ticks_ms(), 100000)
except KeyboardInterrupt:
    zitmaaier.motorStuur.PWM = 0
    zitmaaier.motorVoorAchter.PWM = 0
    print('STOP')
    machine.reset()

server.stop()
