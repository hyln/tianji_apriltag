"""Small read-only live dashboard, with no extra web dependencies."""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


class Dashboard:
    def __init__(self, cameras, host='127.0.0.1', port=23846):
        self.lock = threading.Lock()
        self.frames = {}
        self.results = {c: dict(camera=c, tags=[], status='waiting', age_s=None) for c in cameras}
        dashboard = self
        html = Path(__file__).with_name('dashboard.html').read_bytes()

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                path = urlsplit(self.path).path
                if path == '/':
                    self.respond(200, 'text/html; charset=utf-8', html)
                elif path == '/api/state':
                    self.respond(200, 'application/json', json.dumps(dashboard.snapshot(), allow_nan=False).encode())
                elif path.startswith('/camera/') and path.endswith('.jpg'):
                    try:
                        camera = int(path[len('/camera/'):-len('.jpg')])
                    except ValueError:
                        self.respond(404, 'text/plain', b'Unknown camera')
                        return
                    with dashboard.lock:
                        frame = dashboard.frames.get(camera)
                    self.respond(200 if frame else 404, 'image/jpeg' if frame else 'text/plain', frame or b'Waiting for image')
                else:
                    self.respond(404, 'text/plain', b'Not found')

            def respond(self, status, kind, body):
                self.send_response(status)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer((host, port), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def update(self, camera, result, jpeg):
        with self.lock:
            self.results[camera] = dict(result, status='live', updated_at=time.monotonic())
            self.frames[camera] = jpeg

    def error(self, camera, message):
        with self.lock:
            self.results[camera] = dict(self.results[camera], status='error', error=message)

    def snapshot(self):
        with self.lock:
            results = [dict(r) for r in self.results.values()]
        for result in results:
            updated = result.pop('updated_at', None)
            result['age_s'] = None if updated is None else round(time.monotonic() - updated, 2)
            if result['status'] == 'live' and result['age_s'] > 2:
                result['status'] = 'stale'
        return dict(cameras=results)

    def close(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
