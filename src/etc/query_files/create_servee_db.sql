-- 데이터베이스 생성
CREATE DATABASE SERVEE_DB;
USE SERVEE_DB;

-- FoodCourts 테이블 생성
CREATE TABLE FoodCourts (
    foodcourt_id INT NOT NULL AUTO_INCREMENT,
    name CHAR(32) NOT NULL,
    location VARCHAR(100) NOT NULL,
    PRIMARY KEY (foodcourt_id)
);

-- Robots 테이블 생성
CREATE TABLE Robots (
    robot_id INT NOT NULL AUTO_INCREMENT,
    foodcourt_id INT NOT NULL,
    type CHAR(32) NOT NULL CHECK (type IN ('서빙용', '회수용')),
    model CHAR(32) NOT NULL,
    PRIMARY KEY (robot_id),
    FOREIGN KEY (foodcourt_id) REFERENCES FoodCourts(foodcourt_id)
);

-- DiningTables 테이블 생성
CREATE TABLE DiningTables (
    table_id INT NOT NULL AUTO_INCREMENT,
    foodcourt_id INT NOT NULL,
    location VARCHAR(100) NOT NULL,
    arucomarker_id INT,
    PRIMARY KEY (table_id),
    FOREIGN KEY (foodcourt_id) REFERENCES FoodCourts(foodcourt_id)
);

-- Stores 테이블 생성
CREATE TABLE Stores (
    store_id INT NOT NULL AUTO_INCREMENT,
    foodcourt_id INT NOT NULL,
    location VARCHAR(100) NOT NULL,
    type CHAR(32) NOT NULL,
    name CHAR(32) NOT NULL,
    arucomarker_id INT,
    PRIMARY KEY (store_id),
    FOREIGN KEY (foodcourt_id) REFERENCES FoodCourts(foodcourt_id)
);

-- Menus 테이블 생성
CREATE TABLE Menus (
    menu_id INT NOT NULL AUTO_INCREMENT,
    store_id INT NOT NULL,
    name CHAR(32) NOT NULL,
    price INT NOT NULL,
    description VARCHAR(100),
    menu_image_location VARCHAR(100),
    PRIMARY KEY (menu_id),
    FOREIGN KEY (store_id) REFERENCES Stores(store_id)
);

-- AdminCalls 테이블 생성
CREATE TABLE AdminCalls (
    admincall_id INT NOT NULL AUTO_INCREMENT,
    foodcourt_id INT NOT NULL,
    table_id INT NOT NULL,
    time DATETIME NOT NULL,
    remarks VARCHAR(100),
    PRIMARY KEY (admincall_id),
    FOREIGN KEY (foodcourt_id) REFERENCES FoodCourts(foodcourt_id),
    FOREIGN KEY (table_id) REFERENCES DiningTables(table_id)
);

-- OrderCalls 테이블 생성
CREATE TABLE OrderCalls (
    order_id INT NOT NULL AUTO_INCREMENT,
    table_id INT NOT NULL,
    call_time DATETIME NOT NULL,
    PRIMARY KEY (order_id),
    FOREIGN KEY (table_id) REFERENCES DiningTables(table_id)
);

-- OrderDetails 테이블 생성
CREATE TABLE OrderDetails (
    order_id INT NOT NULL,
    menu_id INT NOT NULL,
    quantity INT NOT NULL,
    PRIMARY KEY (order_id, menu_id),
    FOREIGN KEY (order_id) REFERENCES OrderCalls(order_id),
    FOREIGN KEY (menu_id) REFERENCES Menus(menu_id)
);

-- Log 테이블 생성
CREATE TABLE Log (
    log_id INT NOT NULL AUTO_INCREMENT,
    robot_id INT NOT NULL,
    table_id INT NOT NULL,
    store_id INT NOT NULL,
    call_type CHAR(32) NOT NULL CHECK (call_type IN ('서빙', '회수')),
    call_time DATETIME NOT NULL,
    task_start_time DATETIME,
    task_end_time DATETIME,
    PRIMARY KEY (log_id),
    FOREIGN KEY (robot_id) REFERENCES Robots(robot_id),
    FOREIGN KEY (table_id) REFERENCES DiningTables(table_id),
    FOREIGN KEY (store_id) REFERENCES Stores(store_id)
);
