"""Serve the Quest WebXR page over HTTPS."""

import argparse
import functools
import http.server
import ssl


parser = argparse.ArgumentParser()
parser.add_argument("--ip", default="0.0.0.0")
parser.add_argument("--port", type=int, default=8443)
parser.add_argument("--directory", default="webxr_quest")
parser.add_argument("--certfile", default="certs/quest-webxr.crt")
parser.add_argument("--keyfile", default="certs/quest-webxr.key")
args = parser.parse_args()

handler = functools.partial(
    http.server.SimpleHTTPRequestHandler,
    directory=args.directory,
)
httpd = http.server.ThreadingHTTPServer((args.ip, args.port), handler)

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(args.certfile, args.keyfile)
httpd.socket = ssl_context.wrap_socket(httpd.socket, server_side=True)

print(f"Serving HTTPS on {args.ip}:{args.port}")
print(f"Directory: {args.directory}")
httpd.serve_forever()
