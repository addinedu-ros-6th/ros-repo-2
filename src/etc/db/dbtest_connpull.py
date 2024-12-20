import mysql.connector
from mysql.connector import Error
from datetime import datetime
from functools import singledispatch
from mysql.connector import pooling
import pandas as pd

class MySQLConnection:
    _instance = None

    def __init__(self, pool_name, pool_size, host, port, database, user, password):
        if MySQLConnection._instance is not None:
            raise Exception("이 클래스는 싱글톤입니다!")
        else:
            self.pool = mysql.connector.pooling.MySQLConnectionPool(
                pool_name=pool_name,
                pool_size=pool_size,
                host=host,
                port=port,
                database=database,
                user=user,
                password=password
            )
            MySQLConnection._instance = self

    @classmethod
    def getInstance(cls):
        if cls._instance is None:
            cls._instance = cls(
                pool_name="mypool",
                pool_size=5,  # 원하는 풀의 크기 설정
                host="192.168.0.130",
                port=3306,
                database="SERVEE_DB",
                user="kjy",
                password="1234"
            )
        return cls._instance

    def get_connection(self):
        return self.pool.get_connection()

    def close_all_connections(self):
        self.pool.close()



    #def db_connect(self, host,port, database, user, password):
    #    try:
    #        if self.connection is None or not self.connection.is_connected():
    #            self.connection = mysql.connector.connect(
    #                host=host,
    #                port = port,
    #                database=database,
    #                user=user,
    #                password=password
    #            )
    #            print("MySQL 데이터베이스 연결 성공")
    #        self.cursor = self.connection.cursor()    
    #    except Error as e:
    #        print(f"에러: {e}")
        

    #def disconnection(self):
    #    if self.connection and self.connection.is_connected():
    #        self.connection.close()
    #        print("MySQL 연결이 닫혔습니다.")
    

    def get_store_info(self):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()


        sql= f"""SELECT store_id,name FROM Stores"""
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 

    def get_store_id_vendor(self, store_name):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql= f"""SELECT store_id FROM Stores where name=%s"""
        try:
            
            cursor.execute(sql, (store_name,))
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 


    #메뉴에서 메뉴 누르면 데이터 가져오기
    def get_order_menu(self, store_id):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()


        sql= f"""SELECT menu_id,store_id,name,price,menu_image_location FROM Menus where store_id='{store_id}'"""
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 

    

    def get_order_store_details(self, order_id):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()
        sql= f"""
        SELECT
            s.store_id,
            Menus.name,
            od.quantity,
            oc.call_time
        FROM 
            OrderCalls oc
        JOIN 
            OrderDetails od ON oc.order_id = od.order_id
        JOIN 
            Menus ON od.menu_id = Menus.menu_id
        JOIN 
            Stores s ON Menus.store_id = s.store_id
        WHERE 
            oc.order_id = %s

        ORDER BY 
            oc.order_id ASC"""
        
        try:
            cursor.execute(sql, (order_id,))
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close()    
    
    def get_order_details(self, order_id):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()
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
        
        try:
            cursor.execute(sql, (order_id,))
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 
        
    
    def get_order_detail_menu(self, store_id, menu_id):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()
        sql= f"""
        SELECT 
            Stores.name , Menus.name, Menus.price
        FROM 
            Menus
        RIGHT JOIN 
            Stores ON Menus.store_id= Stores.store_id
        where 
            Menus.menu_id={menu_id} and Stores.store_id={store_id};"""
        
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close()  # 커넥션 반환
    
    def select_store_menu_id(self, store_name,menu_name):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT 
                s.store_id, m.menu_id 
            FROM 
                `Menus` m
            JOIN 
                `Stores` s ON m.store_id = s.store_id
            WHERE 
                m.name = '{menu_name}' AND s.name = '{store_name}'
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 
    
    def insert_ordercalls(self, table_id):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()


        current_time = datetime.now()

        sql_insert=f"""
        INSERT INTO OrderCalls (table_id, call_time)
        VALUES (%s, %s);
        """
        #cursor = self.connection.cursor()
        #
        #cursor.execute(sql_insert, (table_id, current_time))
#
        #self.connection.commit()

        try:
            cursor.execute(sql_insert, (table_id, current_time))
            
            connection.commit()

        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close()

        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()
        sql_select = f"""
            SELECT 
                order_id
            FROM 
                OrderCalls
            ORDER BY call_time DESC
            LIMIT 1;
        """
        try:
            cursor.execute(sql_select)
            get_results = cursor.fetchall()
            order_id = get_results[0][0]
            return order_id
        
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close()  # 커넥션 반환
    
    def insert_orderdetails(self,order_id, menu_id, quantity):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
        INSERT INTO OrderDetails (order_id, menu_id, quantity)
        VALUES (%s, %s,%s);
        """
        #resut=self.get_order_details(order_id)
        #print("바로 적용되었는지 확인 : ", resut)
        
        try:
            cursor.execute(sql, (order_id, menu_id,quantity))
            connection.commit() 
  
        
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close()  # 커넥션 반환
       
    def insert_servicecall(self,table_id):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        current_time = datetime.now()

        sql_select = f"""
            SELECT 
                foodcourt_id
            FROM 
                FoodCourt
        """
        try:
            cursor = self.connection.cursor()
            cursor.execute(sql_select)
            get_results = cursor.fetchall()
            foodcourt_id = get_results[0][0]
  
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close()  # 커넥션 반환

        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
        INSERT INTO AdminCalls (foodcourt_id,table_id, time)
        VALUES (%s, %s,%s);
        """
        try:
            cursor.execute(sql, (int(foodcourt_id), table_id, current_time))
            connection.commit() 
  
        
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close()  # 커넥션 반환     

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

    ########################################
    ### Manager GUI ###

    def get_robots(self):
        '''
        등록되어 있는 robot_id와 type 반환
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT robot_id, type 
            FROM Robots;
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 
    
    def get_robot_type(self, robot_id):
        '''
        robot_id로 type 가져오기
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            sSELECT type 
            FROM Robots 
            WHERE robot_id = {robot_id};
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 
    
    def get_robot_log(self, robot_id, start_date, end_date):
        '''
        robot_id로 로봇 로그 데이터 가져오기
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT table_id, call_time from Log 
            WHERE (robot_id = {robot_id}) and (call_time BETWEEN '{start_date} 00:00:00' AND '{end_date} 23:59:59' )
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 

    def get_stores(self):
        '''
        매점 리스트
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT name 
            FROM Stores 
            WHERE name != '퇴식구1';
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 
    
    def total_orders_today(self):
        '''
        매점당 금일 주문 건수
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT M.store_id, COUNT(DISTINCT OC.order_id) AS order_count
            FROM OrderCalls OC
            JOIN OrderDetails OD ON OC.order_id = OD.order_id
            JOIN Menus M ON OD.menu_id = M.menu_id
            WHERE DATE(OC.call_time) = CURDATE()
            GROUP BY M.store_id;
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 

    def total_earning_today(self):
        '''
        매점당 금일 매출
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT M.store_id, SUM(M.price * OD.quantity) AS total_sales
            FROM OrderCalls OC
            JOIN OrderDetails OD ON OC.order_id = OD.order_id
            JOIN Menus M ON OD.menu_id = M.menu_id
            WHERE DATE(OC.call_time) = CURDATE()
            GROUP BY M.store_id;
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 
    
    def get_store_id(self, store_name):
        '''
        매점 이름으로 매점 id 얻기
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT store_id 
            FROM Stores 
            WHERE name = "{store_name}";
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 
    
    def order_status(self, store_id):
        '''
        매점당 금일 주문 현황
        '''
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()

        sql=f"""
            SELECT 
                M.name AS menu_name,
                OD.quantity,
                OC.call_time, 
                OC.order_id
            FROM 
                OrderCalls OC
            JOIN 
                OrderDetails OD ON OC.order_id = OD.order_id
            JOIN 
                Menus M ON OD.menu_id = M.menu_id
            WHERE 
                M.store_id = {store_id} and DATE(OC.call_time) = CURDATE();
        """
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()
            connection.close() 

    def get_sales_by_month(self,store_name,year):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()


        sql= f"""
        SELECT 
            SUM(m.price * od.quantity) AS total_amount,
            DATE_FORMAT(oc.call_time, '%Y-%m') AS order_month
        FROM 
            OrderCalls AS oc
        JOIN 
            OrderDetails AS od ON oc.order_id = od.order_id
        JOIN 
            Menus AS m ON od.menu_id = m.menu_id
        JOIN 
            Stores AS s ON m.store_id = s.store_id
        WHERE 
            s.name = '{store_name}'
            AND YEAR(oc.call_time) = '{year}' 
        GROUP BY 
            order_month
        ORDER BY 
            order_month;
            """
        try:
            df = pd.read_sql_query(sql, connection)
            return df
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 

    def get_sales_by_day(self,store_name,year,month):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()


        sql= f"""
        SELECT 
            SUM(m.price * od.quantity) AS total_amount,
            DATE_FORMAT(oc.call_time, '%d') AS order_date
        FROM 
            OrderCalls AS oc
        JOIN 
            OrderDetails AS od ON oc.order_id = od.order_id
        JOIN 
            Menus AS m ON od.menu_id = m.menu_id
        JOIN 
            Stores AS s ON m.store_id = s.store_id
        WHERE 
            s.name = '{store_name}'
            AND YEAR(oc.call_time) = '{year}'
            AND MONTH(oc.call_time) = '{month}'  
        GROUP BY 
            order_date
        ORDER BY 
            order_date;
            """
        try:
            df = pd.read_sql_query(sql, connection)
            return df
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 

        #임시
    def get_store_name(self):
        db_pool = MySQLConnection.getInstance()
        connection = db_pool.get_connection()
        cursor = connection.cursor()


        sql= f"""SELECT name FROM Stores"""
        try:
            cursor.execute(sql)
            results = cursor.fetchall()
            return results
        except Exception as e:
            print(f"쿼리 실행 중 오류: {e}")
        finally:
            cursor.close()  # 커서 닫기
            connection.close() 


def main():
    dbm = MySQLConnection.getInstance()
    
    
    #dbm.insert_orderdetails(90,1,2)
    test=dbm.get_sales_by_day("마파궁전","2024","11")

    print(test)
    
if __name__ == "__main__":
    main()
    
