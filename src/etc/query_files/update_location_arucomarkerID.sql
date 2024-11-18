-- Update DiningTables
UPDATE DiningTables
SET location = 'x:1.2, y:3.4, z:5.6, qx:0.1, qy:0.2, qz:0.3, qw:0.4', 
    arucomarker_id = 12
WHERE table_id = 1;

UPDATE DiningTables
SET location = 'x:2.3, y:4.5, z:6.7, qx:0.2, qy:0.3, qz:0.4, qw:0.5', 
    arucomarker_id = 13
WHERE table_id = 2;

UPDATE DiningTables
SET location = 'x:3.4, y:5.6, z:7.8, qx:0.3, qy:0.4, qz:0.5, qw:0.6', 
    arucomarker_id = 14
WHERE table_id = 3;

UPDATE DiningTables
SET location = 'x:4.5, y:6.7, z:8.9, qx:0.4, qy:0.5, qz:0.6, qw:0.7', 
    arucomarker_id = 15
WHERE table_id = 4;

-- Update Stores
UPDATE Stores
SET location = 'x:1.0, y:2.0, z:3.0, qx:0.1, qy:0.2, qz:0.3, qw:0.4', 
    arucomarker_id = 5
WHERE store_id = 1;

UPDATE Stores
SET location = 'x:2.0, y:3.0, z:4.0, qx:0.2, qy:0.3, qz:0.4, qw:0.5', 
    arucomarker_id = 6
WHERE store_id = 2;

UPDATE Stores
SET location = 'x:3.0, y:4.0, z:5.0, qx:0.3, qy:0.4, qz:0.5, qw:0.6', 
    arucomarker_id = 7
WHERE store_id = 3;

UPDATE Stores
SET
    location = 'x:10.0, y:10.0, z:10.0, qx:10.0, qy:10.0, qz:10.0, qw:10.0',
    arucomarker_id = 8
WHERE store_id = 101;
