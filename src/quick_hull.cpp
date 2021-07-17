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
 * - Das linhas X a X, as funções executadas são constantes. O(1)
 * - A linha X executa 1 vez, mas sua complexidade é linear em função da capacity enviada.
 * Portanto tem complexidade da ordem O(N)
 * - A linha X executa enquanto não existirem mais linha no arquivo, portanto
 * executa N + 1 vezes. O(N)
 * - A linha X a X dependem do tamanho das strings das linhas do arquivo de entrada.
 * Como o tamanho de cada coordenada de entrada dos pontos é restringido a 4 algarismos,
 * temos operações com complexidade constante, que executam N vezes por estarem em um loop. O(N)
 * - Na linha X, a criação desse objeto é de tempo constante, pois atribui duas variáveis somente.
 * Como ele é criado N vezes, temos então complexidade O(N).
 * - A linha X é executada uma vez para cada ponto. A operação de inserção no vetor é sempre constante pois
 * o espaço dínamico já foi priamente alocado na linha Y. O(N)
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
 * - Na linha X é criado um novo objeto, que é de tempo constante. O(1)
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
 * - Na linha X é criado um objeto para escrever os dados, que é de tempo constante. O(1)
 * - A linha X será executada N + 1 vezes, dependendo do número de pontos recebidos dentro de hull,
 * logo, terá complexidade O(N).
 * - A linha X escreverá os dados no arquivo, logo será executada N vezes conforme o número de pontos.
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
 * - Nas linhas X a X são criados objetos que tomam tempo constante. O(1)
 * - A linha X será executada N + 1 vezes, dependendo do número de pontos recebidos dentro de points,
 * logo, terá complexidade O(N).
 * - As linha X até X farão as comparações que dependem da quantidade de pontos de entrada, logo
 * terá complexidade O(N).
 * 
 * Portanto a função é limitada pela complexidade linear O(N).
 * 
 * Corretude:
 * 
 * Prova por loop invariante:
 * 
 * Invariante de loop: Os pontos start(S) e end(E) conterão sempre os pontos com,
 * respectivamente, o menor e o maior X dentro subvetor de pontos já analisados (Pa)
 * de um vetor de pontos P.
 * 
 * Inicialização:
 * |Pa| é zero, e portanto os pontos start e end contém valores padrões
 * que representam sua inexistência no momento.
 * 
 * Pa = P[0:0]
 * S = não existe
 * E = não existe
 * 
 * Manutenção:
 * Ao fim da iteração do laço da linha X, percebe-se que o start e o end contém os
 * os pontos com o x, respectivamente mais a esquerda e direita dos pontos já iterados(Pa).
 * 
 * Pa = P[0:i]
 * S = ponto com menor x de Pa
 * E = ponto com maior x de Pa
 * 
 * Término:
 * Após os fim do loop, os pontos atribuidos a start e end contém os pontos extremos do vetor
 * de pontos total.
 * 
 * Pa = P[0:N]
 * S = ponto com menor x de P
 * E = ponto com maior x de P
 * 
 * Portanto, a linha retornada no fim do algoritmo representa a linha que liga os pontos extremos
 * do vetor de pontos recebidos.
 */
Line get_extreme_points(const std::vector<Point2D>& points) {
	Point2D start(INF); // Começo da reta, ponto mais a esquerda
	Point2D end(-INF); // Começo da reta, ponto mais a direita
	
	// Laço para encontrar os pontos mais distantes dentro do fecho com base no eixo X
	for (const Point2D& i : points) {
		if (i.x < start.x) // Se o X for menor, atribui o começo ao novo X
			start = i;
		if (i.x > end.x) // Se o X for maior, atribui o fim ao novo X
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
 * - Nas linhas X a X são criados e calculados variáveis de tempo constante. O(1)
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
 * - Nas linhas X a X são criados e calculados variáveis de tempo constante. O(1)
 * 
 * Portanto a função é limitada pela complexidade constante O(1).
 * 
 * - Corretude:
 * TODO
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
 * - A linha X executa N + 1 vezes, sendo N o tamanho da lista. Tem complexidade O(N).
 * - A linha X estão limitada pelo loop dos pontos. Portanto executa linearmente
 * com base no tamanho da lista de pontos. Tem complexidade O(N).
 * 
 * A função é limitada pela complexidade linear O(N).
 */
void print_points(const std::list<Point2D>& points) {
	for (const Point2D& i : points) {
		std::cout << i.x << ", " << i.y << std::endl;
	}
}

/* quick_hull_rec
 * points: vector<Point2D>&, vetor contendo os pontos a serem analisados na recursão do quick_hull
 * hull: list<Point2D>&, lista contento os pontos que determinam o fecho inteiro, em ordem anti-horária.
 * line: Line, linha representando um limite do triângulo dos pontos dentro do fecho.
 *
 * - Análise de complexidade:
 * Como essa função é recursiva, devemos encontrar uma relação de recorrência representado o custo da árvore
 * em função do tamanho da entrada.
 * 
 * Analisando a junção e separação da divisão e conquista, temos:
 * 
 * - As linhas X e X executam em tempo constante. O(1)
 * - A linha X executa 1 vez, mas sua complexidade é linear em função da capacity enviada.
 * Portanto tem complexidade da ordem O(N)
 * - As linhas X e X executam em tempo constante, pois apenas alocam variáveis. O(1)
 * - A linha X será executada N + 1 vezes, dependendo do número de pontos recebidos dentro de points,
 * logo, terá complexidade O(N).
 * - A linha X chama uma função que é de complexidade constante O(1).
 * - Da linha X até X, são executadas funções de complexidade constante O(1).
 * - A linha X executa uma comparação de tempo constante O(1).
 * - As linhas X e X criam objetos, que é de tempo constante O(1).
 * - As linhas X e X realizam a chamada recursiva, que será melhor analisado posteriormente.
 * - A linha X adiciona ao vetor, que é de custo constante O(1).
 * 
 * Portanto, como custo de cada recursão, temos complexidade O(N).
 * 
 * A função depende de fatores estocásticos para definir esta relação e portanto
 * a análise será dividida em melhor caso, pior caso e caso médio.
 * 
 * - Melhor Caso:
 * Para o melhor caso, temos o algoritmo eliminando metade dos pontos em cada recursão, ou seja,
 * cada recusão analisará metade dos pontos da recursão anterior, mantendo assim uma árvore de recursão
 * balanceada com altura igual a lg(n). Um exemplo dessa árvore é:
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
 * TODO: Pensar caso médio
 */
void quick_hull_rec(const std::vector<Point2D>& points, std::list<Point2D>& hull, Line& line) {
	// Vetor com os novos pontos que serão analisados na próxima chamada recursiva
	std::vector<Point2D> new_points;

	// Estimativa de capacidade do vetor dos pontos a serem analisados pelas próximas iterações.
	// Utilizado para reduzir número de alocações dinâmicas usando listas encadeadas.
	size_t capacity = points.size() / 1.25;
	new_points.reserve(capacity);

	// Para cada ponto no fecho, verifica os que estão a esquerda da reta, em seguida
	// verifica o que estiver mais longe a partir da reta "base"
	Point2D farthest;
	float longest_distance = -INF;
	for (const Point2D& i : points) {
		// Se o ponto a ser verificado estiver a "esquerda" da linha
		if (left(line, i)) {
			new_points.push_back(i); // Adiciona o ponto no novo vetor que será analisado na próxima recursão
			
			// Realiza a verificação para pegar o ponto mais distante da linha
			// esse ponto será utilizado para formar a nova linha representando um limite do triângulo dos pontos dentro do fecho
			float distance = get_point_distance_from_line(line, i);
			if (distance > longest_distance) {
				longest_distance = distance;
				farthest = i;
			}
		}
	}
	
	// Se não há nenhum ponto restante a ser analisado, não prossegue com a recursão
	if (new_points.size() == 0)
		return;

	// É formado duas retas que ligam da reta analisada até o ponto mais longe
	Line firstLine = Line(line.first, farthest);
	Line secondLine = Line(farthest, line.second);

	// Realiza a chamada recursiva
	quick_hull_rec(new_points, hull, firstLine);

	hull.push_back(farthest);

	quick_hull_rec(new_points, hull, secondLine);
}

/* quick_hull
 * points: const vector<Point2D>&, vetor contendo os pontos a ser executado o algoritmo de quick_hull
 * Retorna uma nova lista contento os pontos que determinam o fecho, em ordem anti-horária.
 * 
 * - Análise de complexidade:
 * - A linha X executa a função get_extreme_points, que é de ordem linear, portanto O(N).
 * - As linha X e X realiza uma operação constante de inserção em lista encadeada, portanto O(1).
 * - As linha X e X realizam chamadas recursivas, que são analisadas melhor no método específico.
 * Possuem complexidade no pior caso de O(N^2) e no melhor caso e caso médio de O(N*lgN)
 * 
 * Como a chamada recursiva é o maior custo da função, a complexidade dela será a mesma da chamada
 * recursiva, sendo essa O(N * lgN) ou O(N^2) 
 * 
 * Corretude do algoritmo:
 * 
 * - Lema 1:
 * Dado um conjunto de pontos P e uma reta R, podemos dizer que todo ponto
 * pertencente a P que é o mais distante da reta R em relação a alguma direção
 * ortogonal de R, faz parte do fecho convexo de P.
 * Prova: TODO
 *	
 * Com base no Lema 1, observamos que a etapa inicial de divisão encontra dois
 * pontos (A e B) pertencentes ao fecho, bem como uma reta AB.
 * Pode se aplicar o lema 1 novamente com a reta resultante dos pontos extremos
 * encontrados, com isso
 * TODO
 */
std::list<Point2D> quick_hull(const std::vector<Point2D>& points) {
	// Pegar os pontos extremos, formando uma reta
	Line line = get_extreme_points(points);
	
	std::list<Point2D> hull { line.first };
	
	quick_hull_rec(points, hull, line);
	hull.push_back(line.second);
	quick_hull_rec(points, hull, reverse_line(line));

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
 * Linhas X a X fazem chamadas constantes, O(1)
 * Linhas X chama a função read_input, que é da ordem de O(N)
 * Linhas X e X executam operações de relógio constantes, O(1)
 * Linhas X chama a função quick_hull, que pode ser O(N * lgN) ou O(N^2)
 * Linhas X chama a função write_output, que é da ordem de O(N)
 * Linhas X a X executam operações e chamadas de função independentes
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
