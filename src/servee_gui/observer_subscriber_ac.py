import socket
from datetime import datetime
import queue
import ssl

class ClientObserver:
    def __init__(self, host, port, shared_queue=None):
        self.host = host
        self.port = port

        self.CERTFILE="./src/etc/certificates/server.crt"

        self.tls_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        self.tls_context.minimum_version = ssl.TLSVersion.TLSv1_2  # 최소 TLS 1.2
        self.tls_context.load_verify_locations(self.CERTFILE)

        self.running = True  # Control flag for the receive loop

        self.shared_queue = shared_queue
        
    def parse_message(self, message):
        result =""
        parts = message.split(',')
        if len(parts) < 3:
            print("Invalid message format")
            return

        command_type = parts[0]
        call_type = parts[1]
        instance_id = parts[2]

        try:
            if command_type in ["CREATE", "UPDATE"] and call_type in ["SE", "RV"]:
                status = parts[3]  # new_status value
                result = f"{command_type}, {call_type}, {instance_id}, {status}"
                return result
            elif command_type == "DELETE" and call_type in ["SE", "RV"]:
                result = f"{command_type}, {call_type}, {instance_id}"
                return result
            else:
                print("Unknown command type or call type")
        except Exception as e:
            print(f"An error occurred: {e}")

    def receive_updates(self):
        # Continuously receive notifications from the server
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            with self.tls_context.wrap_socket(client_socket, server_hostname=self.host) as tls_socket:
                tls_socket.connect((self.host, self.port))
                print("Secure connection established.")
                while self.running:
                    try:
                        message = client_socket.recv(1024).decode('utf-8')
                        if message:
                            result = self.parse_message(message)
                            self.shared_queue.put(result)
                    except ConnectionResetError:
                        print("Connection lost. Attempting to reconnect...")
                        break            
                        
    def send_create_command(self, call_type, order_id=None, store_id=None, table_id=None):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            with self.tls_context.wrap_socket(client_socket, server_hostname=self.host) as tls_socket:
                tls_socket.connect((self.host, self.port))
                call_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                if call_type == "SE":
                    command = f"CREATE,{call_type},{order_id},{store_id},{table_id},{call_time}"
                elif call_type == "RV":
                    store_id = 101  # Specify store_id separately if needed
                    command = f"CREATE,{call_type},{store_id},{table_id},{call_time}"
                else:
                    print("Unknown call_type provided.")
                    return

                tls_socket.sendall(command.encode('utf-8'))
                print(f"Sent to server: {command}")


    def send_update_command(self, call_type, order_id, new_status):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            with self.tls_context.wrap_socket(client_socket, server_hostname=self.host) as tls_socket:
                tls_socket.connect((self.host, self.port))
                command = f"UPDATE,{call_type},{order_id},{new_status}"
                tls_socket.sendall(command.encode('utf-8'))
                print(f"Sent to server: {command}")

    def send_delete_command(self, call_type, order_id=None, table_id=None):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            with self.tls_context.wrap_socket(client_socket, server_hostname=self.host) as tls_socket:
                tls_socket.connect((self.host, self.port))

                if call_type == "SE" and order_id is not None:
                    command = f"DELETE,{call_type},{order_id}"
                elif call_type == "RV" and table_id is not None:
                    command = f"DELETE,{call_type},{table_id}"
                else:
                    print("Invalid parameters for DELETE command.")
                    return

                tls_socket.sendall(command.encode('utf-8'))
                print(f"Sent to server: {command}")

    def stop(self):
        self.running = False  # End the receive loop