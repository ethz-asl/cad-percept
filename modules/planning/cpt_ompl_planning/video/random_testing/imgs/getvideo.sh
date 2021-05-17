ffmpeg -i "hilo/frame_%d.png" -c:v libx264 -crf 10 -vf fps=30  random_hilo.mp4
ffmpeg -i "rhone/frame_%d.png" -c:v libx264 -crf 10 -vf fps=30  random_rhone.mp4
ffmpeg -i "curve/frame_%d.png" -c:v libx264 -crf 10 -vf fps=30  random_curve.mp4

