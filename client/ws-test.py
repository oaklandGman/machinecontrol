import websocket
 
ws = websocket.WebSocket()
ws.connect("ws://192.168.2.117/ws")
 
result = ws.recv()
print(result)
 
ws.close()
