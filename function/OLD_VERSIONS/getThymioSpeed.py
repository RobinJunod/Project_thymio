from tdmclient import ClientAsync
global speedl, speedr

def getSpeed():
    with ClientAsync() as client:
        async def prog():
            with await client.lock() as node:
                await node.wait_for_variables({"motor.left.speed"})
                while True:
                    speedl = node.v.motor.left.speed
                    node.flush()
                    await client.sleep(0.1)
        client.run_async_program(prog)


"""
class getThymioSpeed:
    def __init__(self) -> None:
        self.client = ClientAsync()
   
   

    def on_variables_changed(self, node, variables):
        try:
            speedr = variables["motor.right.speed"]
            speedl = variables["motor.left.speed"]
            
           
        except KeyError:
            pass  # prox.horizontal not found

    async def prog(self):
        self.node = await self.client.wait_for_node()
        await self.node.lock()
        await self.node.watch(variables=True)
        self.node.add_variables_changed_listener(self.on_variables_changed)
        #await self.client.sleep()
        await self.node.unlock()
    
    
    def getSpeed(self):
        self.client.run_async_program(self.prog)

        #return [self.speedl, self.speedr]

"""