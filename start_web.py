import http.server
import socketserver
import os
import sys

PORT = 8080
DOCS_DIR = r'd:\OneDrive\project\飞控\果蝇神经网络模型\flydrone\docs'

os.chdir(DOCS_DIR)
print(f"Serving {DOCS_DIR}")
print(f"Index exists: {os.path.exists('index.html')}")
print(f"Files in live/: {os.listdir('live')}")

Handler = http.server.SimpleHTTPRequestHandler

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"\nServer running at http://localhost:{PORT}")
    print(f"Open in browser: http://localhost:{PORT}/index.html")
    print("Press Ctrl+C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")
