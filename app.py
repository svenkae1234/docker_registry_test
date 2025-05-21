from http.server import SimpleHTTPRequestHandler, HTTPServer

host = "0.0.0.0"
port = 8080

print(f"Starting server on {host}:{port}")
httpd = HTTPServer((host, port), SimpleHTTPRequestHandler)
httpd.serve_forever()
