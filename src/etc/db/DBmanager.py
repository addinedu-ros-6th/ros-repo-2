import mysql.connector
from mysql.connector import Error
from datetime import datetime
from functools import singledispatch

class MySQLConnection:
    _instance = None

    def __init__(self):
        if MySQLConnection._instance is not None:
            raise Exception("이 클래스는 싱글톤입니다!")
        else:
            self.connection = None
            MySQLConnection._instance = self

    @classmethod
    def getInstance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

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
        
        #print("select_data: ", sql)
        self.cursor.execute(sql)
        get_results = self.cursor.fetchall()
        return get_results
    
    def get_order_detail_menu(self, store_id, menu_id):
        sql= f"""
        SELECT 
            Stores.store_name , Menus.name, Menus.price
        FROM 
            Menus
        RIGHT JOIN 
            Stores ON Menus.store_id= Stores.store_id
        where 
            Menus.menu_id={menu_id} and Stores.store_id={store_id};"""
        
        #print("select_data: ", sql)
        self.cursor.execute(sql)
        get_results = self.cursor.fetchall()
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
                m.name = '{menu_name}' AND s.store_name = '{store_name}'
        """
        self.cursor.execute(sql)
        get_results = self.cursor.fetchall()
        return get_results
    def insert_ordercalls(self, table_id):
        current_time = datetime.now()

        sql_insert=f"""
        INSERT INTO OrderCalls (table_id, ordercall_time)
        VALUES (%s, %s);
        """
        self.cursor.execute(sql_insert, (table_id, current_time))

        self.connection.commit()

        sql_select = f"""
            SELECT 
                order_id
            FROM 
                OrderCalls
            ORDER BY ordercall_time DESC
            LIMIT 1;
        """
        self.cursor.execute(sql_select)
        get_results = self.cursor.fetchall()
        order_id = get_results[0][0]
        return order_id
    
    def insert_orderdetails(self,order_id, menu_id, quantity):

        sql=f"""
        INSERT INTO OrderDetails (order_id, menu_id, quantity)
        VALUES (%s, %s,%s);
        """

        self.cursor.execute(sql, (order_id, menu_id,quantity))

        self.connection.commit() 

    def insert_servicecall(self,table_id):
        current_time = datetime.now()

        sql_select = f"""
            SELECT 
                foodcourt_id
            FROM 
                Servee_FoodCourt
        """
        self.cursor.execute(sql_select)
        get_results = self.cursor.fetchall()
        foodcourt_id = get_results[0][0]
        sql=f"""
        INSERT INTO AdminCalls (foodcourt_id,table_id, time)
        VALUES (%s, %s,%s);
        """
        self.cursor.execute(sql, (int(foodcourt_id), table_id, current_time))
        self.connection.commit()     

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
    dbm = MySQLConnection.getInstance()
    dbm.db_connect("localhost", 3306, "amrbase", "root", "tjdudghks1")
    
    dbm.insert_servicecall(1)
    

if __name__ == "__main__":
    main()
    
