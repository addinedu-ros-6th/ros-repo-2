import socket
import threading
from queue import Queue
from collections import deque
from abc import ABC, abstractmethod

# ------------------- server -------------------
class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.observers = []
        self.SE_instances = {}
        self.RV_instances = {}
        self.serving_task_queue = Queue()
        self.retrieving_task_queue = Queue()
        self.lock = threading.Lock()

    def register_observer(self, observer):
        with self.lock:
            self.observers.append(observer)
    
    def unregister_observer(self, observer):
        with self.lock:
            if observer in self.observers:
                self.observers.remove(observer)

    def notify_observers(self, message):
        with self.lock:
            for observer in self.observers:
                try:
                    observer.sendall(message.encode('utf-8'))
                except (BrokenPipeError, ConnectionResetError, OSError):
                    self.observers.remove(observer)

    def manage_queue(self, instance):
        try:
            if isinstance(instance, ServingInstance):
                if instance.status == "Q":
                    self.serving_task_queue.put(instance)
                    print(f"Serving instance {instance.order_id} added to serving_task_queue.")
                elif instance.status == "DQ":
                    dequeued_instance = self.serving_task_queue.get()
                    print(f"Serving instance {dequeued_instance.order_id} removed from serving_task_queue.")

            elif isinstance(instance, RetrievalInstance):
                if instance.status == "Q":
                    self.retrieving_task_queue.put(instance)
                    print(f"Retrieval instance {instance.table_id} added to retrieving_task_queue.")
                elif instance.status == "DQ":
                    dequeued_instance = self.retrieving_task_queue.get()
                    print(f"Retrieval instance {dequeued_instance.table_id} removed from retrieving_task_queue.")
        except Exception as e:
            print(f"Error while managing queue: {e}")

    def handle_command(self, command):
        command.execute(self)

    def handle_client_connection(self, client_socket):
        self.register_observer(client_socket)
        while True:
            try:
                data = client_socket.recv(1024).decode('utf-8')
                if data:
                    parts = data.split(',')
                    command_type = parts[0]
                    if command_type == "CREATE":
                        if parts[1] == "SE" and len(parts) >= 6:
                            command = CreateInstanceCommand(call_type=parts[1], order_id=parts[2], store_id=parts[3], table_id=parts[4], call_time=parts[5])
                        elif parts[1] == "RV" and len(parts) >= 5:
                            command = CreateInstanceCommand(call_type=parts[1], store_id=parts[2], table_id=parts[3], call_time=parts[4])
                        else:
                            raise ValueError("Insufficient data for CREATE command")
                    elif command_type == "UPDATE":
                        if parts[1] == "SE" and len(parts) >= 3:
                            command = UpdateStatusCommand(call_type=parts[1], order_id=parts[2], new_status=parts[3])
                        elif parts[1] == "RV" and len(parts) >= 3:
                            command = UpdateStatusCommand(call_type=parts[1], table_id=parts[2], new_status=parts[3])
                        else:
                            raise ValueError("Insufficient data for UPDATE command")
                    elif command_type == "DELETE":
                        if parts[1] == "SE" and len(parts) >= 2:
                            command = DeleteInstanceCommand(call_type=parts[1], order_id=parts[2])
                        elif parts[1] == "RV" and len(parts) >= 2:
                            command = DeleteInstanceCommand(call_type=parts[1], table_id=parts[2])
                        else:
                            raise ValueError("Insufficient data for DELETE command")
                    else:
                        print("Unknown command received.")
                        return
                    self.handle_command(command)
                else:
                    break
            except (ConnectionResetError, ConnectionAbortedError):
                print("Client disconnected.")
                break
        self.unregister_observer(client_socket)
        client_socket.close()

    def start(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.host, self.port))
        server_socket.listen(5)
        print(f"Server running on {self.host}:{self.port}...")

        while True:
            client_socket, addr = server_socket.accept()
            print(f"Connection from {addr}")
            client_thread = threading.Thread(target=self.handle_client_connection, args=(client_socket,))
            client_thread.start()

# ------------------- commands -------------------
class Command(ABC):
    @abstractmethod
    def execute(self, server):
        pass

class CreateInstanceCommand(Command):
    def __init__(self, call_type, order_id=None, store_id=None, table_id=None, call_time=None):
        self.call_type = call_type
        self.order_id = order_id
        self.store_id = store_id
        self.table_id = table_id
        self.call_time = call_time

    def execute(self, server):
        if self.call_type == 'SE':
            instance = ServingInstance(self.order_id, self.store_id, self.table_id, self.call_time)
            server.SE_instances[self.order_id] = instance
            server.notify_observers(f"CREATE,{self.call_type},{self.order_id},{server.SE_instances[self.order_id].status}")
            print(f"SE Instance {self.order_id} created.")
        elif self.call_type == 'RV':
            instance = RetrievalInstance(self.store_id, self.table_id, self.call_time)
            server.RV_instances[self.table_id] = instance
            server.notify_observers(f"CREATE,{self.call_type},{self.table_id},{server.RV_instances[self.table_id].status}")
            print(f"RV Instance {self.table_id} created.")

class UpdateStatusCommand(Command):
    def __init__(self, new_status, call_type, order_id=None, table_id=None):
        self.call_type = call_type
        self.order_id = order_id
        self.table_id = table_id
        self.new_status = new_status

    def execute(self, server):
        if self.call_type == 'SE' and self.order_id in server.SE_instances:
            instance = server.SE_instances[self.order_id]
            instance.status = self.new_status
            server.manage_queue(instance)
            server.notify_observers(f"UPDATE,{self.call_type},{self.order_id},{self.new_status}")
            print(f"Serving instance {self.order_id} status updated to {self.new_status}.")
        elif self.call_type == 'RV' and self.table_id in server.RV_instances:
            instance = server.RV_instances[self.table_id]
            instance.status = self.new_status
            server.manage_queue(instance)
            server.notify_observers(f"UPDATE,{self.call_type},{self.table_id},{self.new_status}")
            print(f"Retrieval instance {self.table_id} status updated to {self.new_status}.")

class DeleteInstanceCommand(Command):
    def __init__(self, call_type, order_id=None, table_id=None):
        self.call_type = call_type
        self.order_id = order_id
        self.table_id = table_id

    def execute(self, server):
        if self.call_type == 'SE' and self.order_id in server.SE_instances:
            del server.SE_instances[self.order_id]
            server.notify_observers(f"DELETE,{self.call_type},{self.order_id}")
            print(f"Serving instance {self.order_id} deleted.")
        elif self.call_type == 'RV' and self.table_id in server.RV_instances:
            del server.RV_instances[self.table_id]
            server.notify_observers(f"DELETE,{self.call_type},{self.table_id}")
            print(f"Retrieval instance {self.table_id} deleted.")
        else:
            print("Instance not found for deletion.")

# ------------------- instances -------------------
class ServingInstance: # 서빙 인스턴스
    def __init__(self, order_id, store_id, table_id, call_time):
        self.order_id = order_id
        self.store_id = store_id
        self.table_id = table_id
        self.call_time = call_time
        self.status = "1" #초기 상태
        self.table_location = "0-0-0/0-0-0-0" #table_id 로 위치 접근, 추후 DB 연동
        self.store_location = "0-0-0/0-0-0-0" #store_id 로 위치 접근, 추후 DB 연동

class RetrievalInstance: # 회수 인스턴스
    def __init__(self, store_id, table_id, call_time):
        self.store_id = store_id
        self.table_id = table_id
        self.call_time = call_time
        self.status = "1" #초기 상태
        self.table_location = "0-0-0/0-0-0-0" #table_id 로 위치 접근, 추후 DB 연동
        self.retrieval_location = "0-0-0/0-0-0-0" #stoe_id 로 위치 접근, 추후 DB 연동