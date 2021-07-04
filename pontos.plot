reset
unset key
unset tics 
unset border 
set size ratio -1
set pointsize 1.0
set term pdf enhanced 
set out 'pontos.pdf'
plot 'input2.txt' with points pt 7
