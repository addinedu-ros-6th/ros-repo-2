from observer_subscriber import ClientObserver
import threading
import time
import queue
import socket

# Custom ClientObserver with queue handling
class ClientObserverWithQueue(ClientObserver):
    def __init__(self, host, port, shared_queue):
        super().__init__(host, port, shared_queue)

    def receive_updates(self):
        # Continuously receive notifications from the server
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((self.host, self.port))
            print("Connected to server for receiving updates.")
            while self.running:
                try:
                    message = client_socket.recv(1024).decode('utf-8')
                    if message:
                        result = self.parse_message(message)
                        if self.shared_queue:  # Check if shared_queue is valid
                            self.shared_queue.put(result)
                        else:
                            print("Shared queue is not initialized.")
                except ConnectionResetError:
                    print("Connection lost. Attempting to reconnect...")
                    break
                except queue.Full:
                    print("Shared queue is full. Message dropped.")
                except Exception as e:
                    print(f"An error occurred while receiving updates: {e}")

if __name__ == "__main__":
    host = "localhost"
    port = 9998

    # Initialize the shared queue
    shared_queue = queue.Queue(maxsize=100)  # Set a max size to prevent overfilling

    client1 = ClientObserverWithQueue(host, port, shared_queue)

    # Start a thread to receive updates from the server
    receive_thread = threading.Thread(target=client1.receive_updates, daemon=True)
    receive_thread.start()

    # Example usage of available methods
    try:
        # 서빙
        # time.sleep(3)
        client1.send_create_command("SE", order_id=1234, store_id=1, table_id=4)
        # time.sleep(3)
        # client1.send_update_command("SE", order_id=1234, new_status="waiting_serverbot")

        # time.sleep(3)
        # client1.send_create_command("SE", order_id=4321, store_id=2, table_id=2)
        # time.sleep(3)
        # client1.send_update_command("SE", order_id=4321, new_status="waiting_serverbot")
        # time.sleep(3)

        # 회수
        # client1.send_create_command("RV", table_id=1)
        # time.sleep(3)
        
        # 회수 = RV,  서빙 = SE
        # client1.send_create_command("RV", table_id=2)
        # time.sleep(3)

        while True:
            if not shared_queue.empty():
                message = shared_queue.get()
                print(f"Processed message from queue: {message}")
    except KeyboardInterrupt:
        print("Stopping ClientObserver...")
        client1.stop()
        receive_thread.join()
