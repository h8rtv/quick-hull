/* Autores:
 * Heitor Tonel Ventura - 2086883
 * José Henrique Ivanchechen - 2090341
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>

// Definição de inteiro "infinito" em C++
#define INF std::numeric_limits<int>::max()

/* class Point2D
 * Utilizada para representar um ponto contendo duas dimensões inteiras.
 */
class Point2D {
public:
	// Coordenada x do ponto.
    int x;
	// Coordenada y do ponto.
    int y;

	// Construtor padrão de um ponto
    Point2D (int x = 0, int y = 0) {
        this->x = x;
        this->y = y;
    }

	// Construtor de cópia de um ponto
    Point2D (const Point2D& point) {
        x = point.x;
        y = point.y;
    }

	// Operador de atribuição de um ponto
	void operator=(const Point2D& point) {
        x = point.x;
        y = point.y;
	}
};


/* typedef Line
 * Representação de linha utilizada no algoritmo, sendo essa um par imutável
 * de dois Point2D.
 */
typedef const std::pair<Point2D, Point2D> Line;

/* Escolha das estruturas de dados:
 * Para a representação do fecho convexo, foi determinado que uma lista encadeada melhor
 * se adequa ao algoritmo pois é necessário realizar leituras sequenciais e o tamanho final
 * do fecho não pode ser determinado antes do algoritmo executar. Por isso, foi utilizada a
 * estrutura std::list<Point2D> fornecida pela STL do C++.
 * 
 * Para a representação dos pontos a serem analisados, bem como os subvetores utilizados na
 * execução das função quick_hull e quick_hull_rec, foi determinado que uma lista sequencial obterá a melhor
 * performance. O problema de usar uma lista encadeada é a quantidade de chamadas de alocação
 * dinâmica que serão feitas, principalmente na criação dos subvetores para a limitação
 * de espaço para as recursões profundas. Portanto a estrutura utilizada foi o
 * std::vector<Point2D> fornecida pelo STL do C++.
 * Foi utilizado também a função reserve para alocar a capacidade ideal e estimada dos vetores
 * e subvetores necessários.
 */

/* read_input
 * filename: std::string, nome do arquivo que contém os pontos de entrada
 * Retorna uma lista encadeada com pontos bidimensionais
 * O arquivo de entrada deve possuir o formato:
 *     <número de pontos>
 *     x1 y1
 *     x2 y2
 *     x3 y3
 *     ...
 * 
 * - Análise de complexidade:
 * - Das linhas 98 a 104, as funções executadas são constantes. O(1)
 * - A linha 105 executa 1 vez, mas sua complexidade é linear em função da capacity enviada.
 * Portanto tem complexidade da ordem O(N)
 * - A linha 108 executa enquanto não existirem mais linha no arquivo, portanto
 * executa N + 1 vezes. O(N)
 * - A linha 110 a 112 dependem do tamanho das strings das linhas do arquivo de entrada.
 * Como o tamanho de cada coordenada de entrada dos pontos é restringido a 4 algarismos,
 * temos operações com complexidade constante, que executam N vezes por estarem em um loop. O(N)
 * - Na linha 115, a criação desse objeto é de tempo constante, pois atribui duas variáveis somente.
 * Como ele é criado N vezes, temos então complexidade O(N).
 * - A linha 116 é executada uma vez para cada ponto. A operação de inserção no vetor é sempre constante pois
 * o espaço dínamico já foi priamente alocado na linha 105. O(N)
 * 
 * Portanto a função é limitada pela complexidade linear O(N).
 */
std::vector<Point2D> read_input(std::string filename) {
	std::vector<Point2D> points; // Vetor de pontos a ser retornado
	std::ifstream file(filename); // Abre um stream de dados do arquivo de entrada
	std::string line; // Variavel onde será armazenada a linha lida do arquivo
	std::getline(file, line); // Guarda a primeira linha do arquivo dentro de "line"
	
	// Reserva a quantidade de pontos dentro do vetor de pontos
	size_t capacity = stoi(line);
	points.reserve(capacity);
	
	// Laço para a leitura do arquivo
	while (std::getline(file, line)) {
		// Realiza o parse da linha, separando ela por um caractere de espaço e lendo a posição (número inteiro)
		std::size_t index = line.find(' ');
		int x = stoi(line.substr(0, index));
		int y = stoi(line.substr(index + 1));

		// Cria um objeto represetando um ponto e o adiciona no vetor resultante
		Point2D point(x, y);
		points.push_back(point);
	}

	return points;
}

/* reverse_line
 * line: Line, linha a ser reversa
 * Retorna uma nova Linha reversa ao que recebeu como parâmetro
 *
 * - Análise de complexidade:
 * - Na linha 132 é criado um novo objeto, que é de tempo constante. O(1)
 * 
 * Portanto a função é limitada pela complexidade constante O(1).
 */
Line reverse_line(Line& line) {
	return Line(line.second, line.first);
}

/* write_output
 * filename: std::string, nome do arquivo que contém os pontos de entrada
 * hull: const std::list<Point2D>&, lista que representa o fecho convexo
 * Escreve no arquivo de saída a lista de pontos na forma:
 *     x1 y1
 *     x2 y2
 *     x3 y3
 *     ...
 *
 * - Análise de complexidade:
 * - Na linha 154 é criado um objeto para escrever os dados, que é de tempo constante. O(1)
 * - A linha 155 será executada N + 1 vezes, dependendo do número de pontos recebidos dentro de hull,
 * logo, terá complexidade O(N).
 * - A linha 156 escreverá os dados no arquivo, logo será executada N vezes conforme o número de pontos.
 * Terá complexidade O(N).
 * 
 * Portanto a função é limitada pela complexidade linear O(N).
 */
void write_output(std::string filename, const std::list<Point2D>& hull) {
	std::ofstream file(filename); // Abre uma nova stream de dados de saída, para escrever os dados
	for (const Point2D& i : hull) {
		file << i.x << " " << i.y << std::endl; // Escreve um ponto no formato: "X Y", junto com uma quebra de linha
	}
}

/* get_extreme_points
 * points: std::list<Point2D>, pontos que estao dentro do fecho
 * Retorna dois pontos que formam uma linha horizontal que atravessa o fecho inteiro
 *
 * - Análise de complexidade:
 * - Nas linhas 211, 212 e 225 são criados objetos que tomam tempo constante. O(1)
 * - A linha 215 será executada N + 1 vezes, dependendo do número de pontos recebidos dentro de points,
 * logo, terá complexidade O(N).
 * - As linha 217 até 220 farão as comparações de custo O(1), mas como estão dentro de um loop, logo
 * terão complexidade O(N).
 * 
 * Portanto a função é limitada pela complexidade linear O(N).
 * 
 * Corretude:
 * 
 * Prova por loop invariante:
 * 
 * Invariante de loop: Os pontos start(S) e end(E) conterão sempre os pontos com,
 * respectivamente, o menor e o maior X dentro subvetor de pontos já analisados (Pa)
 * de um vetor de pontos P. Caso haja empate, o menor X e maior Y será escolhido, bem como
 * o maior X e menor Y será também.
 * 
 * Inicialização:
 * |Pa| é zero, e portanto os pontos start e end contém valores padrões
 * que representam sua inexistência no momento.
 * 
 * Pa = []
 * S = não existe
 * E = não existe
 * 
 * Manutenção:
 * Ao fim da iteração do laço, percebe-se que o start e o end contém os
 * os pontos com o x, respectivamente mais a esquerda e direita dos pontos já iterados(Pa),
 * pois somente se alteram se o ponto da iteração for mais a esquerda ou mais a direita que
 * start ou end. Na condição, também é considerado o empate, onde caso exista, segue a regra
 * estabelecida na invariante.
 * 
 * Pa = P[1:i]
 * S = ponto com menor x de Pa e maior y dos pontos com mesmo x
 * E = ponto com maior x de Pa e menor y dos pontos com mesmo x
 * 
 * Término:
 * Após os fim do loop, os pontos atribuidos a start e end contém os pontos extremos do vetor
 * de pontos total.
 * 
 * Pa = P[1:N]
 * S = ponto com menor x de P e maior y dos pontos com mesmo x
 * E = ponto com maior x de P e menor y dos pontos com mesmo x
 * 
 * Portanto, a linha retornada no fim do algoritmo representa a linha que liga os pontos extremos
 * do vetor de pontos recebidos.
 */
Line get_extreme_points(const std::vector<Point2D>& points) {
	Point2D start(INF, -INF); // Começo da reta, ponto mais a esquerda
	Point2D end(-INF, INF); // Começo da reta, ponto mais a direita
	
	// Laço para encontrar os pontos mais distantes dentro do fecho com base no eixo X
	for (const Point2D& i : points) {
		// SE o X for menor OU se o X for igual e o Y for maior, ENTÃO atribui o começo ao novo X
		if (i.x < start.x || (i.x == start.x && i.y > start.y))
			start = i;
		// SE o X for maior OU se o X for igual e o Y for menor, ENTÃO atribui o fim ao novo X
		if (i.x > end.x || (i.x == end.x && i.y < end.y))
			end = i;
	}

	// Retorna uma nova linha com os pontos de começo e fim
	return Line(start, end);
}

/* left
 * line: Line, par de Point2D representando uma reta
 * p3: Point2D, ponto para verificar a posicao
 * Função primitiva que retorna a posicao entre uma reta (indicada por line)
 * e um ponto (p3)
 * 1(true) caso o p3 esteja a ESQUERDA da reta line
 * 0(false) caso o p3 esteja a DIREITA ou seja COLINEAR da reta line
 * 
 * - Análise de complexidade:
 * - Nas linhas 252 a 254 são criados e calculados variáveis de tempo constante. O(1)
 * 
 * Portanto a função é limitada pela complexidade constante O(1).
 * 
 * Corretude:
 * A corretude da função é dada pela operação matemática que faz.
 * u: (p2.x, p2.y) - (p1.x, p1.y)
 * v: (p3.x, p3.y) - (p1.x, p1.y)
 * (v x u) > 0?
 * Essa, realiza um produto vetorial entre o vetor u e v, retornando assim uma área.
 * Caso a área seja positiva, significa que o vetor v está acima do vetor u e portanto
 * p3 está a esquerda da reta.
 * Caso contrário, o vetor estará colinear ou a direita da reta.
 */
bool left(Line& line, const Point2D& p3) {
	const Point2D& p1 = line.first;
	const Point2D& p2 = line.second;
    int area = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

	return area > 0;
}

/* get_point_distance_from_line
 * line: Line, par de Point2D representando uma reta
 * p3: Point2D, ponto para verificar a distancia
 * Retorna a distancia entre uma reta (indicada por line) e um ponto (p3)
 *
 * - Análise de complexidade:
 * - Nas linhas 277 a 280 são criados e calculados variáveis de tempo constante. O(1)
 * 
 * Portanto a função é limitada pela complexidade constante O(1).
 * 
 * - Corretude:
 * O algoritmo funciona com base na fórmula de cálculo da distância de um ponto
 * e uma reta representada por dois pontos.
    (|(p2.x - p1.x) * (p1.y - p3.y) - (p1.x - p3.x) * (p2.y - p1.y)|)
	/ 
    ((p2.x - p1.x)^2 + (p2.y - p1.y)^2)
 */
float get_point_distance_from_line(Line& line, const Point2D& p3) {
	const Point2D& p1 = line.first;
	const Point2D& p2 = line.second;
    int num = std::abs((p2.x - p1.x) * (p1.y - p3.y) - (p1.x - p3.x) * (p2.y - p1.y));
    int dem = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);

    return (float) num / sqrt(dem);
}


/* print_points
 * points: std::list<Point2D>&, lista encadeada de pontos a serem imprimidos
 * Imprime na tela todos os pontos da lista na forma
 * x1, y1
 * x2, y2
 * x3, y3
 * ...
 * 
 * - Análise de complexidade:
 * - A linha 302 executa N + 1 vezes, sendo N o tamanho da lista. Tem complexidade O(N).
 * - A linha 303 está limitada pelo loop dos pontos. Portanto executa linearmente
 * com base no tamanho da lista de pontos. Tem complexidade O(N).
 * 
 * A função é limitada pela complexidade linear O(N).
 */
void print_points(const std::list<Point2D>& points) {
	for (const Point2D& i : points) {
		std::cout << i.x << ", " << i.y << std::endl;
	}
}

/* get_farthest_point
 * points: std::list<Point2D>&, lista encadeada de pontos a ser analisado
 * line: Line, representando uma reta que dividirá os pontos a ser analisado
 * Retorna: o ponto mais longe da reta dentro do vetor de points
 * 
 * - Análise de complexidade:
 * - As linhas 358 e 359 executam em tempo constante. O(1)
 * - A linha 360 executa N + 1 vezes, sendo N o tamanho da lista. Tem complexidade O(N).
 * - A linha 363 chama uma função que é de complexidade constante O(1), porém 
 * está dentro do loop com complexidade O(N).
 * - Da linha 364 até 366, são executadas dentro do loop com complexidade O(N).
 * - A linha 370 que retorna o valor é de complexidade constante O(1).
 * 
 * A função é limitada pela complexidade linear O(N).
 * 
 * Corretude:
 * 
 * Prova por loop invariante:
 * 
 * Invariante de loop: O ponto farthest conterá o ponto com maior distância com base
 * no subvetor de pontos analisado e a reta recebida.
 * 
 * Inicialização:
 * |Pa| é zero, e portanto o ponto mais distante não existirá.
 * 
 * Pa = []
 * F = não existe
 * LD = não existe
 * 
 * Manutenção:
 * Ao fim da iteração do laço, percebe-se que o ponto F contém
 * o ponto mais longe dentro do subvetor analisado e LD contém sua distância
 * a partir da reta line, porque o ponto e a distância só serão alterados caso
 * o ponto da iteração tenha distância da reta recebida maior que LD.
 * 
 * Pa = P[1:i]
 * F = ponto com maior distância de Pa
 * LD = maior distância de Pa
 * 
 * Término:
 * Após os fim do loop, farthest será o ponto mais distante de P, junto com o a distância
 * armazenada em longest_distance.
 * 
 * Pa = P[1:N]
 * F = ponto com maior distância de P com base na reta recebida
 * LD = maior distância de P com base na reta recebida
 * 
 * Portanto, o ponto retornado no fim do algoritmo representa o ponto que possui a maior
 * distância dentro do conjunto de pontos recebido com base na reta também recebida.
 */
Point2D get_farthest_point(const std::vector<Point2D>& points, Line line) {
	Point2D farthest;
	float longest_distance = -INF;
	for (const Point2D& i : points) {
		// Realiza a verificação para pegar o ponto mais distante da linha
		// esse ponto será utilizado para formar a nova linha representando um limite do triângulo dos pontos dentro do fecho
		float distance = get_point_distance_from_line(line, i);
		if (distance > longest_distance) {
			longest_distance = distance;
			farthest = i;
		}
	}

	return farthest;
}

/* get_left_points
 * points: const std::vector<Point2D>&, vetor de pontos a serem analisados
 * line: Line, representando uma reta que dividirá os pontos
 * Retorna: os pontos a esquerda da reta recebida
 * 
 * - Análise de complexidade:
 * - A linhas 422 e 426 executam em tempo constante. O(1)
 * - A linha 427 executa 1 vez, mas sua complexidade é linear em função da capacity enviada.
 * Portanto tem complexidade da ordem O(N)
 * - A linha 429 executa N + 1 vezes, sendo N o tamanho da lista. Tem complexidade O(N).
 * - As linha 431 a 432 chamam funções e fazem comparações de complexidade constante O(1),
 * como estão dentro de um loop, tem com complexidade O(N).
 * - A linha 436 que retorna o valor é de complexidade constante O(1).
 * 
 * A função é portanto limitada pela complexidade linear O(N).
 * 
 * Corretude:
 * 
 * Prova por loop invariante:
 * 
 * Invariante de loop: O vetor new_points conterá os pontos dentro do subvetor de pontos analisado
 * que estão a esquerda da reta line.
 * 
 * Inicialização:
 * |Pa| é zero, com isso não há pontos a esquerda.
 * 
 * Pa = []
 * NP = []
 * 
 * Manutenção:
 * Ao fim da iteração do laço, new_points conterá os pontos dentro de points 
 * que estão a esquerda da reta line recebida, pois caso o ponto da iteração esteja a esquerda
 * da reta line, este será adicionado no vetor new_points.
 * 
 * Pa = P[1:i]
 * NP = vetor de pontos de Pa que estão a esquerda da reta line
 * 
 * Término:
 * Após os fim do loop, o vetor new_points conterá os pontos de points que estão a esquerda da
 * reta line.
 * 
 * Pa = P[1:N]
 * NP = vetor de pontos de P que estão a esquerda da reta line
 * 
 * Portanto, o vetor retornado no fim do algoritmo conterá os pontos dentro de points
 * que estão a esquerda da reta line.
 */
std::vector<Point2D> get_left_points(const std::vector<Point2D>& points, Line line) {
	// Vetor com os novos pontos que serão retornados
	std::vector<Point2D> new_points;

	// Estimativa de capacidade do vetor dos pontos a serem analisados pelas próximas iterações.
	// Utilizado para reduzir número de alocações dinâmicas usando listas encadeadas.
	size_t capacity = points.size() / 2;
	new_points.reserve(capacity);

	for (const Point2D& i : points) {
		// Se o ponto a ser verificado estiver a "esquerda" da linha
		if (left(line, i)) {
			new_points.push_back(i); // Adiciona o ponto no novo vetor que será analisado na próxima recursão
		}
	}

	return new_points;
}

/* quick_hull_rec
 * points: vector<Point2D>&, vetor contendo os pontos a serem analisados na recursão do quick_hull
 * hull: list<Point2D>&, lista contendo os pontos que determinam o fecho inteiro, em ordem anti-horária.
 * line: Line, linha representando um limite do triângulo dos pontos dentro do fecho.
 * 
 * A função adiciona na lista encadeada hull os pontos extremos a esquerda da reta line informada, de modo
 * a manter a ordem anti-horária do fecho convexo.
 * A ordem anti-horária é estabelecida uma vez que o algoritmo realiza a procura pelos pontos de forma análoga
 * a varredura de árvore binária  E - R - D. Só será adicionado um ponto uma vez que todos os outros pontos 
 * a esquerda deste já estejam sido adicionados na lista hull.
 *
 * - Análise de complexidade:
 * Como essa função é recursiva, devemos encontrar uma relação de recorrência representado o custo da árvore
 * em função do tamanho da entrada.
 * 
 * Analisando a junção e separação da divisão e conquista, temos:
 * 
 * - A linha 646 executa uma comparação de tempo constante O(1).
 * - A linha 650 realiza uma chamada de função de complexidade O(N).
 * - As linhas 654 e 656 executam em tempo constante, pois apenas alocam variáveis. O(1)
 * - A linha 655 e 657 realiza a chamada de uma função com complexidade O(N).
 * - As linhas 661 e 667 realizam a chamada recursiva, que será melhor analisado posteriormente. O(N * lg(N)) em média.
 * - A linha 665 adiciona ao vetor, que é de custo constante O(1).
 * 
 * Portanto, como custo de cada recursão, temos complexidade O(N).
 * 
 * A função depende de fatores estocásticos para definir esta relação e portanto
 * a análise será dividida em melhor caso, pior caso e caso médio.
 * 
 * - Melhor Caso:
 * Para o melhor caso, temos o algoritmo analisando metade dos pontos em cada recursão, ou seja,
 * cada recusão olhará metade dos pontos da recursão anterior, mantendo assim uma árvore de recursão
 * balanceada com altura igual a lg(n). O algoritmo se torna muito rápido quando o número de pontos é muito
 * maior que o número de pontos que pertencem ao fecho, pois esses serão eliminados da análise.
 * Um exemplo da árvore de recorrência é:
 * 
 *   				       T(n)
 * 				       /          \
 *      		  T(n/2)          T(n/2)
 * 				 /    \	          /    \
 *           T(n/4)  T(n/4)   T(n/4)  T(n/4)
 *             /  \   /  \      /  \   /  \
 *           ... ... ... ...  ... ... ... ...
 *
 * Com isso, é obtida a seguinte relação de recorrência.
 * 
 * Tb(n) = 2Tb(n/2) + O(n)
 * Como o fator de junção e substituição é polinomial, podemos utilizar o teorema mestre simplificado,
 * sendo a = 2, b = 2 e d = 1.
 * Comparemos b^d = 2^1 e a = 2.
 * Percebe-se que b^d = a, portanto temos o caso 2 do teorema mestre simplificado, que diz:
 * Tb(n) = O(n^d * log_b(n))
 * Logo, para o melhor caso, Tb(n) = O(n * lg(n))
 * 
 * 
 * - Pior Caso:
 * Para o pior caso, temos o algoritmo removendo apenas um ponto por recursão. Isso ocorre quando
 * todos os pontos no vetor são necessários para formar o fecho convexo.
 * Também, a árvore de recursão resultante deve ser desbalanceada, de modo que o próximo nó tenha
 * que analisar N - 1 pontos. Um exemplo de árvore é:
 * 
 *   				 T(n)
 * 						\
 *      			  T(n - 1)
 * 					      \
 *      			    T(n - 2) 
 * 							\
 *      				  T(n - 3)
 *                            \
 *                            ...
 * 
 * Percebe-se que a altura dessa árvore será linear, em função do tamanho do vetor de pontos analisados.
 * Portanto, o pior caso ocorre quando os pontos estão dispostos de modo que somente um lado da
 * recursão se desenvolve sempre.
 *        
 * Podemos perceber o pior caso se analisarmos pontos formando um círculo, onde o algoritmo
 * eliminaria apenas um ponto e faria outras duas recursões, onde uma receberia N - 1 pontos para analisar
 * e a outra receberia 0 pontos, pois todos eles são de responsabilidade da primeira recursão.
 * Se esse processo ocorre em todas as recursões, temos o pior caso.
 * Um exemplo disso está contido aqui: https://i.imgur.com/gagZNn4.png.
 * 
 * Com base nisso, chegamos na relação de recorrência a seguir:
 * Tw(1) = O(1)
 * Tw(n) = Tw(n - 1) + O(n)
 * 
 * Fazendo a expanção repetitiva da relação de recorrência:
 * Tw(n) = Tw(n - 1) + cn
 * Tw(n) = Tw(n - 2) + c(n - 1) + cn
 * Tw(n) = Tw(n - 3) + c(n - 2) + c(n - 1) + cn
 * Tw(n) = Tw(n - 4) + c(n - 3) + c(n - 2) + c(n - 1) + cn
 * Tw(n) = Tw(n - (n - 1)) + c(n - (n - 2)) + ... + c(n - 3) + c(n - 2) + c(n - 1) + cn
 * Tw(n) = Tw(1) + 2c + ... + c(n - 3) + c(n - 2) + c(n - 1) + cn
 * Tw(n) = c + 2c + 3c + ... + c(n - 3) + c(n - 2) + c(n - 1) + cn
 * Tw(n) = c * (1 + 2 + 3 + ... + (n - 3) + (n - 2) + (n - 1) + n) // soma de todos os inteiros até N. 
 * Tw(n) = c * (n(n + 1) / 2)
 * Tw(n) = c * (n^2 + n) / 2
 * Tw(n) = (cn^2)/2 + cn/2
 * 
 * Logo, Tw(n) = O(n^2)
 *
 * Ta(n) = 2Ta(n/2) + O(n)
 * 
 * Caso médio:
 * No caso médio, assume-se uma distribuição uniforme de pontos no espaço analisado.
 * Percebe-se uma probabilidade maior do ponto não pertencer ao fecho do que ele pertencer.
 * É observado também que uma árvore de recorrência desbalanceada, onde somente uma chamada
 * recursiva se desenvolve tendo que analisar todos os pontos da recursão anterior, também é
 * rara.
 * Portanto é assumido que o caso médio ocorre quando a árvore resultante é razoavelmente
 * balanceada e o conjunto do fecho é consideravelmente menor que o conjunto de pontos de entrada.
 * 
 * Utilizando uma árvore particionada de maneira constante, podemos entender como árvores quase
 * balanceadas são O(n * lgN) também. Dada a seguinte relação de recorrência:
 * 
 * Ta(n) = Ta(n / t) + Ta(fn/t) + O(n)
 * Onde f < t e f + 1 = t
 * 
 * Desenvolvendo a árvore
 *                                                          Custo por nível
 *   				           T(n)                              = cn
 * 				       /                 \
 *      		   T(n/t)              T(fn/t)                   = cn
 * 			      /     \	            /     \
 *           T(n/t^2)  T(fn/t^2)   T(fn/t^2)  T(n * f^2 / t^2)   = cn
 *             /  \    /  \           /  \       /  \
 *           ... ... ... ...        ... ...     ... ...          = cn 
 *            |  ... ... ...        ... ...     ... ...          = cn
 *           T(1)... ... ...        ... ...     ... ...          = cn
 *                                  ... ...     ... ...         <= cn
 *	                                            ... ...         <= cn
 *			                                        T(1)        <= cn
 * 
 * Percebe-se que a altura da árvore(h) é igual a log_(t/f)(n) // log base t/f de n
 * Também, é observado que o custo por nível (Cn) é sempre <= cn.
 * 
 * O custo total da árvore será limitado superiormente por h * Cn. Logo,
 * O(h * Cn) = O(log_(t/f)(n)*cn) = O(N * lgN)
 * 
 * Sabendo que árvores constantemente particionadas tem complexidade no tempo O(N * lgN),
 * e percebendo que as árvores quase balanceadas terão características similares,
 * pode-se concluir que o comportamento médio esperado do algoritmo será parecido com o caso
 * ótimo da execução do algoritmo, sendo esse também O(N * lgN).
 *
 * - Corretude:
 * 
 * Pode-se definir fecho convexo como a menor região convexa que contém os pontos do conjunto.
 * A região convexa é uma região onde, para todo segmento de reta formado por um par de pontos contido na região,
 * este segmento não conterá nenhum ponto externo a região.
 * Essa região é formada a partir dos pontos extremos do conjunto de pontos de entrada.
 * Um ponto extremo é um ponto que não se encontra em nenhum segmento de reta que une dois pontos quaisquer
 * dentro da região formada pelo fecho convexo.
 *
 * Para provar a corretude do algoritmo, deve-se provar que um ponto da entrada mais distante de uma reta r
 * em uma direção ortogonal é um ponto extremo, que o quick_hull_rec adiciona pontos extremos à lista encadeada do fecho
 * e seleciona todos os pontos extremos contidos na entrada.
 * 
 * 1) Provar que o ponto mais distante de uma reta R em uma direção ortogonal é um ponto extremo.
 * 
 * Dado uma reta r, o ponto mais distante P dela em uma direção (por exemplo, a esquerda dela) é considerado um ponto extremo.
 * A prova é dada pelo fato de que caso não seja um ponto extremo, haverá um segmento de reta de P a um ponto interno do fecho
 * que cruzará com as bordas do fecho convexo. 
 * 
 * 2) Provar que dado um triângulo formado por três pontos extremos, todo ponto contido na região formada por ele não poderá
 * ser um ponto extremo.
 * 
 * Prova por contradição:
 * P, conjunto de pontos analisados
 * Pe, conjunto de pontos extremos de P formando um fecho convexo
 * Pontos A, B e C pertencem a Pe e formam um triângulo T
 * 
 * Assume-se um ponto D que está contido na região de T e seja um ponto extremo.
 * Logo, haverá uma conexão de algum dos pontos A, B ou C até D. Como A, B e C formam um triângulo, percebe-se que 
 * ao adicionar o ponto D, uma concavidade será adicionada na região do fecho pois existirá um segmento de reta que
 * contém algum ponto fora da região, contradizendo a premissa de que o fecho Pe é convexo.
 * Portanto, D não pode ser um ponto extremo.
 * 
 * 3) Provar que os pontos adicionados pelo quick_hull_rec fazem parte do fecho convexo e que todos os pontos de
 * do vetor de pontos são analisados.
 *
 * Hipóteses/Invariantes:
 * a) Os pontos que formam a reta recebida na função recursiva fazem parte do fecho convexo.
 * b) Os pontos a esquerda da reta recebida serão corretamente classificados em pontos extremos, descartados ou reanalisados.
 *
 * Passo Base:
 * Se o conjunto de pontos a ser analisado for vazio, então nenhum ponto será adicionado ao fecho convexo.
 * 
 * Passo Indutivo:
 * Para cada recursão será selecionado o ponto de points mais distante da reta recebida.
 * Com base em (1), pode-se afirmar que esse ponto encontrado, farthest, é um ponto extremo. Logo faz parte 
 * do fecho convexo.
 * Após isso, são criadas duas linhas auxiliares que formam um triângulo com a linha recebida da recursão.
 * Com base em (2), percebe-se que os pontos que precisarão ser analisados estarão ou a esquerda da reta firstLine
 * ou a esquerda da reta secondLine e os outros pontos não poderão ser pontos extremos.
 * Então, get_left_points é chamado duas vezes, onde irá selecionar os pontos a esquerda dado das retas firstLine e secondLine
 * e os armazenará dentro de um vetor chamado de firstPoints e secondPoints. 
 * Por fim, a recursão é chamada duas vezes, uma com firstLine e firstPoints, e outra com secondLine e secondPoints.
 * Com isso, (a) se mantém, pois as retas são criadas a partir de pontos que são pontos extremos.
 * Também, todos os pontos recebidos na função serão analisados, sendo um classificado como extremo (farthest), dois subconjuntos de
 * pontos que estão a esquerda das novas retas que serão reanalisados e um subconjunto de pontos a serem descartados
 * pois estão a direita ou colineares as retas. Isso mantém a hipótese (b).
 * 
 * Término:
 * Todos os pontos da recursão inicial da função são analisados e são ou desconsiderados por estarem a direita/colinear de
 * alguma reta formada ou são pontos extremos.
 *
 */
void quick_hull_rec(const std::vector<Point2D>& points, std::list<Point2D>& hull, Line& line) {
	// Se não há nenhum ponto restante a ser analisado, não prossegue com a recursão
	if (points.size() == 0)
		return;

	// Retorna o ponto mais a esquerda dentro de new_points com base na reta line
	Point2D farthest = get_farthest_point(points, line);

	// Forma duas retas que ligam da reta analisada até o ponto mais longe, e também
	// extrai uma lista de pontos que estão a esquerda de points com base na primeira e segunda reta
	Line firstLine = Line(line.first, farthest);
	std::vector<Point2D> firstPoints = get_left_points(points, firstLine);
	Line secondLine = Line(farthest, line.second);
	std::vector<Point2D> secondPoints = get_left_points(points, secondLine);

	// Realiza as chamadas recursivas, passando o conjunto de pontos de entrada e saída,
	// bem como a linha que fará a divisão entre pontos da esquerda e direita.
	quick_hull_rec(firstPoints, hull, firstLine);

	// Adiciona o ponto mais longe encontrado, uma vez que ele é necessário para
	// completar os pontos dentro do fecho convexo em ordem anti-horária.
	hull.push_back(farthest);

	quick_hull_rec(secondPoints, hull, secondLine);
}

/* quick_hull
 * points: const vector<Point2D>&, vetor contendo os pontos a ser executado o algoritmo de quick_hull
 * Retorna uma nova lista contendo os pontos que determinam o fecho convexo, em ordem anti-horária.
 * 
 * - Análise de complexidade:
 * - A linha 707 executa a função get_extreme_points, que é de ordem linear, portanto O(N).
 * - As linha 713 e 727 realiza uma operação constante de inserção em lista encadeada, portanto O(1).
 * - As linha 723 e 729 realizam chamadas de quick_hull_rec, que possuem complexidade no pior caso de O(N^2)
 * e no melhor caso e caso médio de O(N*lgN).
 * - As linha 718 e 719 chamam funções que são de complexidade O(N).
 * 
 * Como a chamada quick_hull_rec é o maior custo da função, a complexidade de quick_hull será limitada pela
 * quick_hull_rec, sendo essa, portanto, O(N * lgN) ou O(N^2). 
 * 
 * - Corretude:
 * 
 * Para provar a corretude do algoritmo, deve-se provar que os pontos com menor e maior valores no eixo X
 * fazem parte do fecho convexo.
 * 
 * 1) Provar que os pontos mais a esquerda(E) e mais a direita(D), ou seja, os pontos com menor e maior
 * valor no eixo X respectivamente, fazem parte do fecho convexo.
 * 
 * Como E está mais a esquerda do conjunto de pontos, significa que não é possível que exista algum segmento de 
 * reta interno a região formada pelo fecho convexo que contenha E, pois caso exista, esta irá cruzar com a borda
 * do fecho convexo.
 * Por causa disso, E deve estar contido no conjunto de pontos que formam o fecho convexo.
 * 
 * Prova análoga ao ponto com maior valor no eixo X (D).
 * 
 * O restante do funcionamento do algoritmo se baseia em duas chamadas de quick_hull_rec, em que analisará os pontos a
 * esquerda da reta formada pelos pontos mais a esquerda e mais a direita e os pontos a esquerda da reta direcionada
 * reversa a esta.
 */
std::list<Point2D> quick_hull(const std::vector<Point2D>& points) {
	// Retorna os pontos extremos dentro do conjunto de pontos inicial
	// com base no eixo X, ou seja, forma uma linha com o menor e maior X
	// presente dentro de points
	Line line = get_extreme_points(points);
	
	// Declara a lista que conterá os pontos do fecho convexo calculado pelo
	// algoritmo recursivo. Ele começa contendo o primeiro ponto encontrado pelo get_extreme_points,
	// uma vez que o ponto mais a esquerda também faz parte do fecho convexo e é o primeiro
	// na ordem anti-horária.
	std::list<Point2D> hull { line.first };

	// Extrai uma lista de pontos que estão a esquerda de points com base na primeira e segunda reta
	// a primeira reta é dada pelos pontos extremos que atravessa o conjunto de pontos
	// a segunda reta é dada pelo inverso da primeira reta
	std::vector<Point2D> left = get_left_points(points, line);
	std::vector<Point2D> right = get_left_points(points, reverse_line(line));

	// Realiza as chamadas recursivas, passando o conjunto de pontos de entrada e saída,
	// bem como a linha que fará a divisão entre pontos da esquerda e direita.
	quick_hull_rec(left, hull, line);
	
	// Adiciona o segundo ponto extremo encontrado, uma vez que ele é necessário para
	// completar os pontos dentro do fecho convexo em ordem anti-horária.
	hull.push_back(line.second);
	
	quick_hull_rec(right, hull, reverse_line(line));

	return hull;
}

/* main
 * argc: int, quantidade de strings contidas na chamada do programa.
 * argv: char**, vetor de strings contendo as informações na chamada do programa.
 * Função inicializadora do programa.
 * Retorna 1 em caso de falha, quando o arquivo de texto de input não é informado na chamada
 * do programa.
 * Retorna 0 em caso de execução com sucesso do programa.
 * 
 * - Análise de complexidade:
 * Linhas 748 a 753 fazem chamadas constantes, O(1)
 * Linhas 754 chama a função read_input, que é da ordem de O(N)
 * Linhas 757 e 762 executam operações de relógio constantes, O(1)
 * Linhas 759 chama a função quick_hull, que pode ser O(N * lgN) ou O(N^2)
 * Linhas 764 chama a função write_output, que é da ordem de O(N)
 * Linhas 767 a 770 executam operações e chamadas de função independentes
 * do tamanho do input e portanto são constantes também, O(1).
 * 
 * Pode-se concluir que a execução completa do programa está limitado pelo
 * comportamento da função quick_hull, que pode ser O(N * lgN) ou O(N^2).
 * 
 */
int main(int argc, char* argv[]) {
	// Tratamento de erro caso o input seja fora dos padrões definidos
	if (argc < 2) {
		std::cout << "Usage: ./hull <filename>" << std::endl;
		std::cout << "Example : ./hull input.txt" << std::endl;
		return 1;
	}
	std::string filename = argv[1];
	std::vector<Point2D> points = read_input(filename);

	// Tempo do início do algoritmo
	auto start = std::chrono::system_clock::now();

	std::list<Point2D> hull = quick_hull(points);

	// Tempo do fim do algoritmo
	auto end = std::chrono::system_clock::now();

	write_output("fecho.txt", hull);

	// Calcula o tempo de execução do algoritmo
	std::chrono::duration<double> elapsed_time = end - start;
	// Adiciona precisão de 6 casas no output
	std::cout << std::fixed << std::setprecision(6);
	std::cout << elapsed_time.count() << std::endl;

	return 0;
}
