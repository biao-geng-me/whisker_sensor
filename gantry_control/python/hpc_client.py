import socket
import pickle
import struct

class HPCClient:
    """Communicates with the HPC compute node via the persistent SSH Tunnel."""
    
    def __init__(self, host='127.0.0.1', port=5555):
        self.host = host
        self.port = port
        self.hpc_socket = None

    def connect(self):
        """Connects to the local end of the SSH tunnel."""
        try:
            self.hpc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.hpc_socket.connect((self.host, self.port))
            print(f"[HPC] Connected to SSH Tunnel at {self.host}:{self.port}")
            return True
        except ConnectionRefusedError:
            print(f"[HPC Error] Could not connect to {self.host}:{self.port}. Is the SSH tunnel running?")
            return False

    def sync_trajectory(self, trajectory):
        """Sends rollout data, blocks, and waits for updated weights."""
        if not self.hpc_socket:
            print("[HPC] Not connected. Skipping sync.")
            return b"dummy_weights"

        print(f"[HPC] Sending trajectory ({len(trajectory)} steps)...")
        
        # Serialize trajectory using pickle
        data_bytes = pickle.dumps(trajectory)
        
        # Send length header, then data
        self.hpc_socket.sendall(struct.pack('>I', len(data_bytes)) + data_bytes)
        
        # Block and wait for new weights (length header, then data)
        print("[HPC] Waiting for gradient update from compute node...")
        length_bytes = self._recvall(4)
        if not length_bytes:
            return None
            
        weight_length = struct.unpack('>I', length_bytes)[0]
        new_weights = self._recvall(weight_length)
        
        print("[HPC] New weights received successfully.")
        return new_weights

    def _recvall(self, n):
        """Helper to reliably receive exactly n bytes."""
        data = bytearray()
        while len(data) < n:
            packet = self.hpc_socket.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return bytes(data)
        
    def close(self):
        if self.hpc_socket:
            self.hpc_socket.close()
            print("[HPC] Disconnected.")