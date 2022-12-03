from tdmclient import ClientAsync

class setThymioSpeed:
    def __init__(self) -> None:
        self.client = ClientAsync()
    def motors(self, left, right):
        return {
            "motor.left.target": [left],
            "motor.right.target": [right],
        }


    async def prog(self):
        node = await self.client.wait_for_node()
        await node.lock()
        await node.set_variables(self.motors(self.speedL, self.speedR))
        await node.unlock()

    def runClas(self, vL, vR):
        self.speedL = vL
        self.speedR = vR
        self.client.run_async_program(self.prog)