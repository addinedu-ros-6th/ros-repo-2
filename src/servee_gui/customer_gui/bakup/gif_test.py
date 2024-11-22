from moviepy import VideoFileClip

# 동영상 파일 경로
video_path = 'input_video.mp4'
# GIF로 저장할 경로
gif_path = 'output.gif'

# 동영상 파일 불러오기
clip = VideoFileClip(video_path)

# GIF로 변환하여 저장
clip.write_gif(gif_path)