ffmpeg -i "hilo/frame_%d.png" -c:v libx264 -crf 10 -vf fps=60  rotate_hilo.mp4
ffmpeg -i "rhone/frame_%d.png" -c:v libx264 -crf 10 -vf fps=60  rotate_rhone.mp4
ffmpeg -i "curve/frame_%d.png" -c:v libx264 -crf 10 -vf fps=60  rotate_curve.mp4

