GCC = g++

all:
	$(GCC) src/quick_hull.cpp -o hull

clean:
	rm -rf hull *.txt *.pdf

genpoints:
	scripts/genpoints 1048575

view:
	scripts/visualizar.sh

run:
	./hull input.txt

default: all
