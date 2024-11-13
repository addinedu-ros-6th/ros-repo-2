import mysql.connector
from mysql.connector import Error
from datetime import datetime
from functools import singledispatch

class MySQLConnection:
    _instance = None

    #def __init__(self):
    #    if MySQLConnection._instance is not None:
    #        raise Exception("이 클래스는 싱글톤입니다!")
    #    else:
    #        self.connection = None
    #        MySQLConnection._instance = self
#
    #@classmethod
    #def getInstance(cls):
    #    if cls._instance is None:
    #        cls._instance = cls()
    #    return cls._instance
    def __init__(self):
        self.connection = None
        self.cursor = None
        #self.db_connect(host, port, database, user, password)



    def db_connect(self, host,port, database, user, password):
        try:
            if self.connection is None or not self.connection.is_connected():
                self.connection = mysql.connector.connect(
                    host=host,
                    port = port,
                    database=database,
                    user=user,
                    password=password
                )
                print("MySQL 데이터베이스 연결 성공")
            self.cursor = self.connection.cursor()    
        except Error as e:
            print(f"에러: {e}")
        

    def disconnection(self):
        if self.connection and self.connection.is_connected():
            self.connection.close()
            print("MySQL 연결이 닫혔습니다.")

    def get_obstacle_by_time(self,selected_start_time,selected_end_time):
        sql= f"""
        SELECT 
            COALESCE(CAST(DrivingLog.speed AS SIGNED), 'N/A') AS speed, 
            COALESCE(EventLog.category, 'N/A') AS category,  
            COALESCE(EventLog.type, 'N/A') AS type, 
            DrivingLog.time
        FROM    
            EventLog 
        RIGHT JOIN 
            DrivingLog ON DrivingLog.time = EventLog.occurtime
        WHERE 
            DrivingLog.time > '{selected_start_time}' AND 
            DrivingLog.time < '{selected_end_time}'"""
        

        print("select_data: ", sql)
        self.cursor.execute(sql)
        obstacle_results = self.cursor.fetchall()
        return obstacle_results
    
    #메뉴에서 메뉴 누르면 데이터 가져오기
    def get_order_menu(self, store_id):
        sql= f"""SELECT menu_id,store_id,name,price,menu_image_location FROM Menus where store_id='{store_id}'"""
        cursor = self.connection.cursor() 
        #print("select_data: ", sql)
        cursor.execute(sql)
        get_results = cursor.fetchall()
        return get_results
    
    def get_order_details(self, order_id):
        sql= f"""
        SELECT 
            Menus.name,
            od.quantity,
            oc.call_time
        FROM 
            OrderCalls oc
        JOIN 
            OrderDetails od ON oc.order_id = od.order_id
        JOIN 
            Menus ON od.menu_id = Menus.menu_id
        WHERE 
            oc.order_id = %s"""
        
        #print("select_data: ", sql)
        cursor = self.connection.cursor()# 새로운 커서 생성
        try:
            cursor = self.connection.cursor()
            print("db내 오더아이디: ",order_id)
            cursor.execute(sql, (order_id,))  # 파라미터 전달
            get_results =  cursor.fetchall()
            print("db내 결과: ",get_results)  # 결과를 모두 가져옴
            return get_results
            
        except Exception as e:
            pass
        
    
    def get_order_detail_menu(self, store_id, menu_id):
        sql= f"""
        SELECT 
            Stores.name , Menus.name, Menus.price
        FROM 
            Menus
        RIGHT JOIN 
            Stores ON Menus.store_id= Stores.store_id
        where 
            Menus.menu_id={menu_id} and Stores.store_id={store_id};"""
        
        cursor = self.connection.cursor()
        #print("select_data: ", sql)
        cursor.execute(sql)
        get_results = cursor.fetchall()
        return get_results
    
    def select_store_menu_id(self, store_name,menu_name):
        sql=f"""
            SELECT 
                m.menu_id
            FROM 
                `Menus` m
            JOIN 
                `Stores` s ON m.store_id = s.store_id
            WHERE 
                m.name = '{menu_name}' AND s.name = '{store_name}'
        """
        cursor = self.connection.cursor()
        cursor.execute(sql)
        get_results = cursor.fetchall()
        return get_results
    
    def insert_ordercalls(self, table_id):
       
        current_time = datetime.now()

        sql_insert=f"""
        INSERT INTO OrderCalls (table_id, call_time)
        VALUES (%s, %s);
        """
        cursor = self.connection.cursor()
        cursor.execute(sql_insert, (table_id, current_time))

        self.connection.commit()

        sql_select = f"""
            SELECT 
                order_id
            FROM 
                OrderCalls
            ORDER BY call_time DESC
            LIMIT 1;
        """
        cursor.execute(sql_select)
        get_results = cursor.fetchall()
        order_id = get_results[0][0]
        return order_id
    
    def insert_orderdetails(self,order_id, menu_id, quantity):
        
        sql=f"""
        INSERT INTO OrderDetails (order_id, menu_id, quantity)
        VALUES (%s, %s,%s);
        """
        cursor = self.connection.cursor()
        cursor.execute(sql, (order_id, menu_id,quantity))

        self.connection.commit() 

        resut=self.get_order_details(order_id)
        print("바로 적용되었는지 확인 : ", resut)
        
       
    def insert_servicecall(self,table_id):
        current_time = datetime.now()

        sql_select = f"""
            SELECT 
                foodcourt_id
            FROM 
                FoodCourt
        """
        cursor = self.connection.cursor()
        cursor.execute(sql_select)
        get_results = cursor.fetchall()
        foodcourt_id = get_results[0][0]
        
        sql=f"""
        INSERT INTO AdminCalls (foodcourt_id,table_id, time)
        VALUES (%s, %s,%s);
        """
        self.cursor.execute(sql, (int(foodcourt_id), table_id, current_time))
        self.connection.commit()     

    def insert_log(self, table_id):
        print("일단 보류")

    def update_data(self, table, columns, params, where = None):
        set_clauses = [f"{column} = %s" for column in columns]
        set_clause_str = ', '.join(set_clauses)

        sql = f"UPDATE {table} SET {set_clause_str}"

        if where:
          sql += f" WHERE {where}"

        print("update_data: ", sql)
        self.cursor.execute(sql, params)

        self.connection.commit() 
# 사용 예시
#def main():
    #db = MySQLConnection.getInstance()
    #db.db_connect("192.168.0.130",3306, "deep_project", "yhc", "1234")
    #current_time = datetime.now()

    #select query
    #result = db.select_data("LogMessage",where="id='1'")
    #if result:
    #    for row in result:
    #        print(row)
    
    
   
    #db.update_data("EventLog",("type",), ("test",), where="category='장애물'")

#    db.close_connection()
def main():
    dbm = MySQLConnection()
    dbm.db_connect("localhost", 3306, "SERVEE_DB", "root", "tjdudghks1")
    
    #dbm.insert_orderdetails(90,1,2)
    test=dbm.get_order_details(90)
   
    
if __name__ == "__main__":
    main()
    
