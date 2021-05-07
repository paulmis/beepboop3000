# Takes a 1600x900 vertically-reversed image after a second and displays it over ssh using fex
raspistill -o image.jpg -w 1600 -h 900 -q 10 -t 1 -fli 50hz -vf && feh image.jpg
