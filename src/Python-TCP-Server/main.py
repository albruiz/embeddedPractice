import socket

# Server configuration
HOST = '192.168.31.12'  # Listen on all available interfaces (replace with actual IP if needed)
PORT = 48569         # Port to listen on

def run_server():
    """Creates a simple TCP server to receive data."""

    # Create a socket (INET, STREAMing)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the address
    try:
        server_socket.bind((HOST, PORT))
    except socket.error as e:
        print(f"Bind failed. Error: {e}")
        return

    # Listen for incoming connections
    server_socket.listen(1)  # Allow only one connection

    print(f"Server listening on {HOST}:{PORT}")

    # Accept a connection
    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    while True:
        # Receive data from the client
        try:
            data = conn.recv(1024)  # Receive up to 1024 bytes
            if not data:
                print("Client disconnected.")
                break
            message = data.decode('utf-8')  # Decode the bytes to a string
            print(f"Received: {message}")

            # Optionally, send a response back to the client
            response = f"Server received: {message}"
            conn.sendall(response.encode('utf-8'))

        except ConnectionResetError:
            print("Client disconnected unexpectedly.")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            break


    # Clean up the connection
    conn.close()
    server_socket.close()
    print("Server closed.")

if __name__ == "__main__":
    run_server()