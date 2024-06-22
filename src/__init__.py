import asyncio
import websockets
from pyee import EventEmitter
import json

webSocketAdress: str = "192.168.178.34"
webSocketPort: str = "8080"

class WebSocketHooks(EventEmitter):
    NEW_CONNECTION = "NEW_CONNECTION"

    def DispatchHook(self, hook, body):
        self.emit(hook, body)

    def SubscribeHookListener(self, hook, listener):
        self.on(hook, listener)

    def UnsubscripeListener(self, hook, listener):
        self.remove_listener(hook, listener)


class ReceivedEvent:
    def __init__(self, name: str):
        self.eventName = name
        self.data = {}

    @property
    def JSONString(self) -> str:
        return json.dumps(self.__dict__)

    def addData(self, key: str, value):
        self.data[key] = value





async def connect_to_server():
    calibrationHooks = WebSocketHooks()
    url: str = f"ws://{webSocketAdress}:{webSocketPort}"
    async with websockets.connect(url) as websocket:
        def OnTriggerCalibration(body:object):
            print(body)
        # await websocket.send("Hello, server!")

        calibrationHooks.SubscribeHookListener(hook="ON_TRIGGER_CALIBRATION",listener=OnTriggerCalibration)

        initCalibrator: ReceivedEvent = ReceivedEvent("INIT_CALIBRATOR")
        await websocket.send(initCalibrator.JSONString)

        while True:
            response = await websocket.recv()
            
            body = json.loads(response)

            print(f"{body}")

            event = body.get("eventName", "")
            data = body.get("data", {})

            print(event)
            print(data)

            calibrationHooks.DispatchHook(hook=event,body=data)

            











asyncio.get_event_loop().run_until_complete(connect_to_server())