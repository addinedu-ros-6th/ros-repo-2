INSERT INTO FoodCourts (foodcourt_id, name, location) 
VALUES (1, '핑크랩 푸드코트', '서울특별시 금천구 가산동');

INSERT INTO Robots (foodcourt_id, type, model) 
VALUES 
    (1, '서빙용', 'P&A-M1'),
    (1, '서빙용', 'P&A-M1'),
    (1, '회수용', 'P&A-M1');

INSERT INTO DiningTables (foodcourt_id, location, arucomarker_id) 
VALUES 
    (1, '1층', 10),
    (1, '1층', 11),
    (1, '1층', 12),
    (1, '1층', 13);

INSERT INTO Stores (foodcourt_id, location, type, name, arucomarker_id) 
VALUES 
    (1, '1층', '중식', '마파궁전', 21),
    (1, '1층', '중식', '차이니즈 키친', 22),
    (1, '1층', '양식', '올리브 비스트로', 23);

INSERT INTO Menus (store_id, name, price, description, menu_image_location) 
VALUES 
    (1, '짜장면', 8000, '짜장면입니다', './etc/image/zza.png'),
    (1, '짬뽕', 9000, '짬뽕입니다', './etc/image/zzam.png'),
    (1, '탕수육', 20000, '탕수육입니다', './etc/image/tang.png'),
    (1, '깐풍새우', 20000, '깐풍새우입니다', './etc/image/ggan.png'),
    (2, '짜장면', 12000, '짬뽕집짜장면입니다', './etc/image/zza.png'),
    (2, '짬뽕', 13000, '짬뽕집짬뽕입니다', './etc/image/zzam.png'),
    (2, '유산슬', 30000, '짬뽕집유산슬입니다', './etc/image/you.png'),
    (2, '동파육', 35000, '동파육입니다', './etc/image/dong.png'),
    (3, '나폴리 피자', 7000, '피자입니다', './etc/image/pizza.png'),
    (3, '봉골레 파스타', 7000, '파스타입니다', './etc/image/bong.png'),
    (3, '리조또', 8000, '리조또입니다', './etc/image/rizo.png'),
    (3, '마늘빵', 2000, '마늘빵입니다', './etc/image/galic.png');
