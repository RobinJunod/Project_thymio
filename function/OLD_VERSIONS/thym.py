from tdmclient import ClientAsync

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

with ClientAsync() as client:
    async def prog():
        with await client.lock() as node:
            await node.set_variables(motors(50, 50))
            await client.sleep(2)
            await node.set_variables(motors(0, 0))
    client.run_async_program(prog)


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

def get_data():
    thymio_data.append({"ground":list(node["prox.ground.reflected"]), 
                        "sensor":list(node["prox.ground.reflected"]),
                        "left_speed":node["motor.left.speed"],
                        "right_speed":node["motor.right.speed"]})

def aquire_data():
    aw(node.wait_for_variables()) # wait for Thymio variables values
    global rt
    rt = RepeatedTimer(0.1, get_data)

def setSpeedTH(vL,vR):
    aw(node.lock())
    node.send_set_variables(motors(vL, vR))
    aw(node.unlock())

def stopTH():
    rt.stop()