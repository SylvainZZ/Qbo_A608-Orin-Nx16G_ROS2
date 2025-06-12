import asyncio
from aiohttp import web
import threading
import rclpy
from rclpy.node import Node


class QboWebServer:
    def __init__(self, supervisor):
        self.supervisor = supervisor
        self.app = web.Application()
        self.app.add_routes([web.get('/', self.handle_index)])
        self.runner = web.AppRunner(self.app)

    async def handle_index(self, request):
        state = self.supervisor.current_state
        return web.Response(text=f"<h1>Current SMACH State: {state}</h1>", content_type='text/html')

    async def start(self):
        await self.runner.setup()
        site = web.TCPSite(self.runner, '0.0.0.0', 8080)
        await site.start()


def run_web_server(server: QboWebServer):
    asyncio.run(server.start())
    web.run_app(server.app)  # Juste pour bloquer si besoin


class QboSupervisor(Node):
    def __init__(self):
        super().__init__('qbo_web_supervisor')
        self.current_state = 'UNKNOWN'


def main():
    rclpy.init()
    node = QboSupervisor()
    web_server = QboWebServer(node)

    # Thread parallèle pour le web
    web_thread = threading.Thread(target=run_web_server, args=(web_server,), daemon=True)
    web_thread.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
