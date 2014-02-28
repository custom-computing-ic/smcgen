gcc -std=c99 -lm -o gen_obsrv gen_obsrv.c
./gen_obsrv &> data_y.txt
rm gen_obsrv
