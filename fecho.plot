reset
unset key
unset tics 
unset border 
set size ratio -1
set pointsize 1.0
set term pdf enhanced 
set out 'fecho.pdf'
plot 'input2.txt' with points pt 7,\
     'fecho2.txt' with linespoints ls 7 lw 1.0

