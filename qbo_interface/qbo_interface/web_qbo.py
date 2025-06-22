import asyncio
from aiohttp import web
import threading
import rclpy
from rclpy.node import Node


class QboWebServer:
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.app = web.Application()
        self.app.add_routes([
            web.get('/', self.handle_index),
            web.get('/ws', self.websocket_handler)
        ])
        self.runner = web.AppRunner(self.app)

    async def handle_index(self, request):
        return web.Response(text="""
        <html><head><title>Qbo Diagnostics</title></head>
        <body style="font-family:sans-serif">
        <h2>Diagnostics Viewer</h2>
        <div id="status"></div>
        <script>
            const ws = new WebSocket("ws://" + location.host + "/ws");
            ws.onmessage = event => {
                const data = JSON.parse(event.data);
                let html = `<h3>SMACH State: ${data.state}</h3>`;

                for (const [k,v] of Object.entries(data.errors)) {
                    html += `<div style='background:red;color:white;padding:5px;margin:5px;border-radius:8px;'>❌ ${k} : ${v[0]}</div>`;
                }
                for (const [k,v] of Object.entries(data.warnings)) {
                    html += `<div style='background:orange;color:black;padding:5px;margin:5px;border-radius:8px;'>⚠️ ${k} : ${v[0]}</div>`;
                }
                document.getElementById("status").innerHTML = html;
            };
        </script>
        </body></html>
        """, content_type='text/html')

    async def websocket_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        try:
            while True:
                await asyncio.sleep(1)
                data = {
                    'warnings': self.supervisor.get_warn_dict(),
                    'errors': self.supervisor.get_error_dict(),
                    'state': self.supervisor.current_state
                }
                await ws.send_json(data)
        except (asyncio.CancelledError, ConnectionResetError, web.WebSocketError, aiohttp.ClientConnectionError) as e:
            self.supervisor.get_logger().warn(f"[WebSocket] Connection closed: {e}")
        finally:
            await ws.close()

    async def start(self):
        await self.runner.setup()
        site = web.TCPSite(self.runner, '0.0.0.0', 8080)
        await site.start()


def run_web_server(server: QboWebServer):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server.start())
    loop.run_forever()



if __name__ == '__main__':
    print("This script is not meant to be run standalone.")
