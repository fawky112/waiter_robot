"""
Al Fanan Restaurant — WebSocket Relay Server
─────────────────────────────────────────────
Port 8000 : HTTP server  (serves kiosk HTML, payment page, logo, videos)
Port 8765 : WebSocket    (pure relay — every message is broadcast to all
                          other connected clients unchanged)

Clients that connect:
  • Kiosk screen   (waiter_kiosk.html)
  • Phone          (payment-complete.html)
  • KioskBridge    (kiosk_bridge.py — ROS2 ↔ WebSocket bridge)
  • Kitchen GUI    (kitchen_gui.html — optional)

Message flow:
  phone → "paid" → relayed to kiosk → kiosk shows confirmation
  kiosk → "payment_finalized" → relayed to bridge → publishes screen=5
  kiosk → "customer_done" → relayed to bridge → publishes screen=6
  bridge → "11", "12", etc → relayed to kiosk → shows loading screens
  bridge → "table:3" → relayed to kiosk → triggers vid1
  kiosk → order JSON → relayed to kitchen GUI

Run:
    python3 server.py

Files are served from the SAME folder as this script.
"""

import asyncio
import websockets
import http.server
import socketserver
import threading
import os
import json

# ── HTTP server ───────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

class KioskHTTPHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=BASE_DIR, **kwargs)
    
    def log_message(self, fmt, *args):
        # Only print non-200 responses
        if args[1] not in ('200', '304'):
            print(f'[HTTP] {args[1]} {args[0]}')

def run_http():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(('0.0.0.0', 8000), KioskHTTPHandler) as httpd:
        print('🌍  HTTP server  → http://0.0.0.0:8000')
        print(f'   Serving files from: {BASE_DIR}')
        httpd.serve_forever()

# ── WebSocket relay ───────────────────────────────────────
CLIENTS = set()

def log_message(websocket, direction, message):
    """Pretty-print relayed messages."""
    addr = websocket.remote_address
    client_type = "?"
    # Try to identify client type from remote port/address patterns
    if hasattr(websocket, 'path') and 'kiosk' in str(websocket.path).lower():
        client_type = "KIOSK"
    
    # Truncate long messages
    display = message[:80] + "..." if len(message) > 80 else message
    print(f'  {direction} [{addr}] → "{display}"')

async def ws_handler(websocket):
    """Handle new WebSocket connection."""
    CLIENTS.add(websocket)
    addr = websocket.remote_address
    print(f'⚡  Connected   [{addr}]  total: {len(CLIENTS)}')
    
    try:
        async for message in websocket:
            log_message(websocket, 'RX', message)
            
            # Relay to ALL OTHER connected clients (pure relay)
            others = CLIENTS - {websocket}
            if others:
                await asyncio.gather(
                    *[c.send(message) for c in others],
                    return_exceptions=True
                )
                # Log relay
                client_list = [f'{c.remote_address}' for c in others]
                print(f'  ↳ Relayed to {len(others)} client(s): {client_list}')
            else:
                print(f'  ↳ No other clients connected — message not relayed')
                
    except websockets.exceptions.ConnectionClosedError:
        pass
    except Exception as e:
        print(f'  ⚠ Error [{addr}]: {e}')
    finally:
        CLIENTS.discard(websocket)
        print(f'💤  Disconnected [{addr}]  remaining: {len(CLIENTS)}')

async def run_ws():
    print('🔗  WebSocket   → ws://0.0.0.0:8765')
    async with websockets.serve(ws_handler, '0.0.0.0', 8765, compression=None):
        await asyncio.Future()  # run forever

# ── Entry point ───────────────────────────────────────────
if __name__ == '__main__':
    print('═══════════════════════════════════')
    print('   Al Fanan Restaurant — Server    ')
    print(f'   Files: {BASE_DIR}')
    print('═══════════════════════════════════')
    print()
    
    # Start HTTP in background thread
    threading.Thread(target=run_http, daemon=True).start()
    
    try:
        asyncio.run(run_ws())
    except KeyboardInterrupt:
        print('\n🛑  Server stopped.')
