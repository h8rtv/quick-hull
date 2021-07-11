GCC = g++
FLAGS =-Wall -Wextra -Werror -pedantic -g

all:
	$(GCC) src/quick_hull.cpp -o hull $(FLAGS)

clean:
	rm -rf hull *.txt *.pdf

genpoints:
	scripts/genpoints 1000

view:
	scripts/visualizar.sh

run:
	./hull input.txt

default: all
