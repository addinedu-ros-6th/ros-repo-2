# 현재 코드에서 사용 중인 모델은 MobileNetV2를 기반으로 합니다. 
# tf.keras.applications.MobileNetV2에서 weights='imagenet'를 지정하여 
# ImageNet 데이터셋으로 사전 학습된 가중치를 가져오고 있습니다. 따라서, 모델은 MobileNetV2 구조이지만,
#  초기 가중치는 ImageNet으로 학습된 값이 사용됩니다.

# 이 코드는 MobileNetV2의 사전 학습된 가중치를 활용한 이진 분류 모델입니다.

import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import os

# 모델 저장 폴더 설정
save_dir = '/home/heechun/final_ws/TableCheck'
os.makedirs(save_dir, exist_ok=True)  # 폴더가 없을 경우 생성

# 데이터 증강을 위한 ImageDataGenerator 설정 (추가 옵션 포함)
train_datagen = ImageDataGenerator(
    rescale=1.0 / 255,              # 정규화
    rotation_range=30,               # 회전 (0~30도)
    width_shift_range=0.3,           # 가로 이동 (이미지 크기의 30%)
    height_shift_range=0.3,          # 세로 이동 (이미지 크기의 30%)
    shear_range=0.2,                 # 시어 변환 (이미지 기울이기)
    zoom_range=0.2,                  # 줌 인/아웃
    horizontal_flip=True,            # 좌우 반전
    brightness_range=[0.8, 1.2],     # 밝기 조절 (0.8배 ~ 1.2배)
    fill_mode='nearest',             # 변환 시 빈 픽셀 보정
    validation_split=0.2             # 학습/검증 데이터 분할 (80% 학습, 20% 검증)
)

# 학습용 데이터 로드
train_generator = train_datagen.flow_from_directory(
    '/home/heechun/final_ws/TableCheck/record_table/data/train/',  # 데이터 경로
    target_size=(224, 224),          # 이미지 크기 조정
    batch_size=32,                   # 배치 크기
    class_mode='binary',             # 이진 분류
    subset='training'                # 학습용 데이터
)

# 검증용 데이터 로드
validation_generator = train_datagen.flow_from_directory(
    '/home/heechun/final_ws/TableCheck/record_table/data/train/',  # 동일한 경로 사용
    target_size=(224, 224),          
    batch_size=32,                   
    class_mode='binary',             
    subset='validation'              # 검증용 데이터
)

# MobileNetV2 기반 모델 구성
model = tf.keras.models.Sequential([
    tf.keras.applications.MobileNetV2(input_shape=(224, 224, 3), include_top=False, weights='imagenet'),
    tf.keras.layers.GlobalAveragePooling2D(),
    tf.keras.layers.Dense(1, activation='sigmoid')  # 이진 분류용 시그모이드
])

# 모델 컴파일
model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# 체크포인트 설정: 가장 좋은 검증 성능 모델 저장
checkpoint_path = os.path.join(save_dir, 'best_model_1031.keras')
model_checkpoint = tf.keras.callbacks.ModelCheckpoint(
    checkpoint_path, monitor='val_accuracy', save_best_only=True, verbose=1
)

# 학습 로그 저장 (TensorBoard 사용)
log_dir = os.path.join(save_dir, 'logs_1031')
tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=log_dir)

# 모델 학습
model.fit(
    train_generator,
    validation_data=validation_generator,
    epochs=20,
    callbacks=[model_checkpoint, tensorboard_callback]
)

# 데이터 증강 옵션 추가
# shear_range: 이미지를 비스듬히 기울입니다.
# zoom_range: 이미지를 확대/축소합니다.
# brightness_range: 이미지의 밝기를 조정합니다.
# fill_mode: 변환 시 생기는 빈 픽셀을 보정합니다.
# 모델 저장 경로 및 체크포인트
# os.makedirs(): 저장할 디렉터리를 생성합니다.
# ModelCheckpoint: 가장 좋은 **검증 정확도(val_accuracy)**를 기록한 모델을 best_model.h5로 저장합니다.
# TensorBoard 로그 저장
# TensorBoard를 통해 학습 과정(손실, 정확도)을 시각화할 수 있도록 로그를 기록합니다.
# tensorboard --logdir=/home/heechun/final_ws/record_table/models/logs 명령어로 TensorBoard를 실행합니다.


# 이후 모델 로드 방법
# 저장된 모델 로드
# model = tf.keras.models.load_model('/home/heechun/final_ws/TableCheck/best_model.keras')

# 실행하고 나서 localhost 뭐시기 사이트 링크 뜨면 눌러서 검증결과 볼수있음
# tensorboard --logdir=/home/heechun/final_ws/TableCheck/logs

# 10.30 -> 민강사님이 학습횟수 더 늘려봐도 될거같다고 하심 - val_accuracy가 꾸준히 올라가는 모습을 보여줘서