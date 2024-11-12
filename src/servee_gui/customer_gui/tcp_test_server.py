import socket
import sys
import os
import ast
current_dir = os.path.dirname(os.path.abspath(__file__)) # 현재 스크립트의 디렉토리를 가져오고, 프로젝트 루트로 이동하는 상대 경로를 추가
print("Current Directory:", current_dir)
relative_path = os.path.join(current_dir, '..','..')  # 상위 폴더로 이동

sys.path.append(relative_path)
import threading
import time
from etc.db.DBmanager import MySQLConnection
import queue

class Vendor():
    def __init__(self):
        self.dbm = MySQLConnection.getInstance()
        self.dbm.db_connect("localhost", 3306, "amrbase", "root", "tjdudghks1")
        self.tcp_ip = "192.168.0.130"
        self.vendor_tcp_port =9992
        self.customer_tcp_port =9993

        self.orderid_queue = queue.Queue()

        self.vendor_manager = threading.Thread(target=self.state_confirm)
        self.vendor_manager.start()  


        self.vendor_client = threading.Thread(target=self.client_send_orderid)
        self.vendor_client.start()  
        
    def db_connection(self, data):
        

        if "get_order_menu" in data:
            data = data.split('/')
            results = self.dbm.get_order_menu(data[1])

        elif "get_order_detail_menu" in data:
            data = data.split('/')   
            results = self.dbm.get_order_detail_menu(data[1],data[2])

        elif "select_store_menu_id"  in data:
            order_id = None 

            split_data = data.split('/')

            receive_data = split_data[1]

            list_result=ast.literal_eval(receive_data)

            for index ,value in enumerate(list_result):
                if index==0:
                    table_num = value

                    continue
                
                quantity = value[2]

                menu_id = self.dbm.select_store_menu_id(value[0],value[1])
                menu_id = menu_id[0][0]
                print("테이블 넘버 : ", table_num)
                print("수량 : ", quantity)
                print("메뉴아이디 : ", menu_id)
                if order_id is None:
                    order_id=self.dbm.insert_ordercalls(menu_id)
                final_order_id = order_id

                self.dbm.insert_orderdetails(final_order_id,menu_id, quantity)

        self.dbm.disconnection()
        return results    

    def client_send_orderid(self):
        try:
            order_id=self.orderid_queue.get()
            print("큐에서 전달 odr : ", order_id)
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((self.tcp_ip, self.vendor_tcp_port))
            # 정수 전송

            client.send(order_id.encode('utf-8'))
            print("서버에서 odr전달 : ", order_id)
            # 서버로부터 응답 수신
            response = client.recv(4096).decode('utf-8')
            print(response)


        except Exception as e:
            pass
        finally:
            client.close()


    def state_confirm(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((self.tcp_ip, self.customer_tcp_port))
        server.listen(5)
        print("서버가 시작되었습니다. 연결을 기다립니다...")

        while True:
            try:
                client_socket, addr = server.accept()
                if client_socket:
                    print(f"연결됨: {addr}")
                    request_data = client_socket.recv(1024)
                    
                    data = request_data.decode('utf-8')
                    result_data = f"{data}/서빙중"
                    self.orderid_queue.put(str(data))
                    print(data)
                    time.sleep(1)
                    response = result_data
                    client_socket.send(response.encode('utf-8'))
                    print("데이터 전달")
                   

            except Exception as e:
                pass
            finally:
                client_socket.close()

    #def handle_client_connection(self,client_socket):
#
    #    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #    server.bind(('localhost', 9990))
    #    server.listen(5)
    #    print("서버가 시작되었습니다. 연결을 기다립니다...")
#
    #    while True:
    #        try:
    #            client_socket, addr = server.accept()
    #            if client_socket:
    #                print(f"연결됨: {addr}")
    #                request_data = client_socket.recv(1024)
#
    #                data = request_data.decode('utf-8')
    #                data_result=self.db_connection(data)
#
    #        except Exception as e:
    #            pass
    #        finally:
    #            client_socket.close()



if __name__ == "__main__":
    Vendor()


