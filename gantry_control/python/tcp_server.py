import socket
import struct
import time

# --- Configuration ---
HOST = '127.0.0.1'  # Localhost
PORT = 65432        # Port to listen on

# Define the byte structure for communication (Big-endian '>' to match Java)
# 'd' is an 8-byte double. 
# Action: 2 doubles (e.g., 2 motor torques) = 16 bytes
ACTION_FMT = '>2d'
# State msg: 4 states, 1 reward, 1 done flag = 6 doubles = 48 bytes
STATE_FMT = '>6d'  
STATE_BYTES = struct.calcsize(STATE_FMT)

def recv_all(conn, num_bytes):
    """Receive exactly num_bytes from the socket, handling partial reads"""
    data = b''
    while len(data) < num_bytes:
        chunk = conn.recv(num_bytes - len(data))
        if not chunk:
            raise ConnectionError("Connection closed before all data received")
        data += chunk
    return data

def main():
    # 1. Create socket and configure for low latency
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        # Disable Nagle's algorithm for real-time communication
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        
        s.bind((HOST, PORT))
        s.listen()
        print(f"Waiting for MATLAB to connect on {HOST}:{PORT}...")
        
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            
            # --- Dummy RL Training Loop ---
            episodes = 1
            for ep in range(episodes):
                print(f"--- Starting Episode {ep+1} ---")
                
                # Receive initial state from MATLAB
                data = recv_all(conn, STATE_BYTES)
                if not data: break
                
                step_count = 0
                dt_tot = 0
                while True:
                    t_start = time.perf_counter()
                    
                    # 1. Unpack state data from MATLAB
                    try:
                        unpacked_data = struct.unpack(STATE_FMT, data)
                        state = unpacked_data[0:4]
                        reward = unpacked_data[4]
                        done = unpacked_data[5]
                    except struct.error as e:
                        print(f"ERROR: Failed to unpack state data: {e}")
                        print(f"  Expected {STATE_BYTES} bytes, got {len(data)} bytes")
                        print(f"  Raw bytes (first 16): {data[:16].hex()}")
                        break
                    
                    if done > 0.5: # Simple float boolean check
                        print("Episode finished.")
                        break
                    
                    # 2. Compute Action (Dummy DRL Inference goes here)
                    # For now, just send random dummy actions
                    action = [0.5, -0.5] 
                    
                    # 3. Send Action to MATLAB
                    conn.sendall(struct.pack(ACTION_FMT, *action))
                    
                    # 4. Wait for next state from MATLAB
                    data = recv_all(conn, STATE_BYTES)
                    if not data: break
                    
                    # --- Timing check ---
                    t_end = time.perf_counter()
                    dt_ms = (t_end - t_start) * 1000
                    dt_tot += dt_ms
                    
                    step_count += 1
                    if step_count % 100 == 0:
                        print(f"Step {step_count} | Latency + MATLAB Compute: {dt_ms:.2f} ms, avg: {dt_tot/step_count:.2f}")

if __name__ == "__main__":
    main()