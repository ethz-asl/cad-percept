ffmpeg -i "2d_img/frame_%d.png" -c:v libx264 -crf 10 -vf fps=30  2dfield.mp4
ffmpeg -i "3d_img/frame_%d.png" -c:v libx264 -crf 10 -vf fps=30  3dfield.mp4


