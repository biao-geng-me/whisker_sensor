import socket
import struct
import json

class ConnectionManager:
    """Handles low-level TCP communication with MATLAB."""
    
    def __init__(self, host='127.0.0.1', port=65432, timeout=1.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.conn = None
        self.addr = None

    def wait_for_client(self):
        print(f"[Network] Listening for MATLAB on {self.host}:{self.port}...")
        self.conn, self.addr = self.server_socket.accept()
        # Disable Nagle and set timeout on the accepted connection
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.conn.settimeout(self.timeout)
        print(f"[Network] Connected by {self.addr}")

    def _recvall(self, n):
        """Helper to reliably receive exactly n bytes."""
        data = bytearray()
        while len(data) < n:
            packet = self.conn.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return bytes(data)

    def receive_json(self):
        """Reads a 4-byte length header, then decodes the JSON string."""
        raw_length = self._recvall(4)
        if not raw_length: return None
        msg_length = struct.unpack('>I', raw_length)[0] # Big-endian Unsigned Int
        
        raw_data = self._recvall(msg_length)
        if not raw_data: return None
        return json.loads(raw_data.decode('utf-8'))

    def send_string(self, text):
        """Sends a 4-byte length header, then the string data."""
        encoded = text.encode('utf-8')
        self.conn.sendall(struct.pack('>I', len(encoded)) + encoded)

    def receive_header(self):
        """Reads a single byte header dictating the state machine."""
        raw_byte = self._recvall(1)
        if not raw_byte: return None
        return struct.unpack('>B', raw_byte)[0] # Big-endian Unsigned Char

    def receive_doubles(self, count):
        """Reads 'count' number of big-endian doubles."""
        raw_data = self._recvall(count * 8) # 8 bytes per double
        if not raw_data: return None
        return struct.unpack(f'>{count}d', raw_data)

    def send_doubles(self, values):
        """Packs a list/tuple of floats into big-endian doubles and sends."""
        packed_data = struct.pack(f'>{len(values)}d', *values)
        self.conn.sendall(packed_data)

    def set_timeout(self, timeout):
        """Dynamically update the receive timeout."""
        if self.conn:
            self.conn.settimeout(timeout)
            print(f"[Network] Timeout set to {timeout}s")

    def close(self):
        if self.conn:
            self.conn.close()
        self.server_socket.close()
        print("[Network] Connection closed.")