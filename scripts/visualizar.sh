#!/bin/bash

npts=$(head -1 input.txt); tail -$npts input.txt > input2.txt; gnuplot pontos.plot
cp fecho.txt fecho2.txt; head -1 fecho.txt >> fecho2.txt; gnuplot fecho.plot
evince fecho.pdf
