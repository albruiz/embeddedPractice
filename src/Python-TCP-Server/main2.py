from flask import Flask, render_template, jsonify
import socket
import threading

app = Flask(__name__)

# Server configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 48569

# Global variable to store the last received message
last_message = "No message received yet."


def tcp_server():
    """Creates a TCP server to receive data and updates the last_message."""
    global last_message

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server_socket.bind((HOST, PORT))
    except socket.error as e:
        print(f"Bind failed. Error: {e}")
        return

    server_socket.listen(1)
    print(f"TCP Server listening on {HOST}:{PORT}")

    conn, addr = server_socket.accept()
    print(f"Connected by {addr}")

    while True:
        try:
            data = conn.recv(1024)
            if not data:
                print("Client disconnected.")
                break
            message = data.decode('utf-8')
            print(f"Received: {message}")
            last_message = message  # Update the global variable

            # Optionally, send a response
            response = f"Server received: {message}"
            conn.sendall(response.encode('utf-8'))

        except ConnectionResetError:
            print("Client disconnected unexpectedly.")
            break
        except Exception as e:
            print(f"Error receiving data: {e}")
            break

    conn.close()
    server_socket.close()
    print("TCP Server closed.")


@app.route("/")
def index():
    """Renders the main page with the last received message."""
    return render_template('index.html', message=last_message)


@app.route("/api/message")
def get_message():
    """API endpoint to get the last received message as JSON."""
    global last_message
    return jsonify(message=last_message)


if __name__ == "__main__":
    # Start the TCP server in a separate thread
    tcp_thread = threading.Thread(target=tcp_server)
    tcp_thread.daemon = True  # Allow the program to exit even if the thread is running
    tcp_thread.start()

    # Start the Flask app
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False) # host 0.0.0.0 for external access