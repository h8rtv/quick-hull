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
 * execução da função quick_hull_rec, foi determinado que uma lista sequencial obterá a melhor
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
 * hull: list<Point2D>&, lista contendo os pontos que determinam o fecho inteiro, em ordem anti-horária.
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
 * Explicada em detalhes na função quick_hull.
 */
void quick_hull_rec(const std::vector<Point2D>& points, std::list<Point2D>& hull, Line& line) {
	// Vetor com os novos pontos que serão analisados na próxima chamada recursiva
	std::vector<Point2D> new_points;

	// Estimativa de capacidade do vetor dos pontos a serem analisados pelas próximas iterações.
	// Utilizado para reduzir número de alocações dinâmicas usando listas encadeadas.
	size_t capacity = points.size() / 2;
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

	// Realiza as chamadas recursivas, passando o conjunto de pontos de entrada e saída,
	// bem como a linha que fará a divisão entre pontos da esquerda e direita.
	quick_hull_rec(new_points, hull, firstLine);

	// Adiciona o ponto mais longe encontrado, uma vez que ele é necessário para
	// completar os pontos dentro do fecho convexo em ordem anti-horária.
	hull.push_back(farthest);

	quick_hull_rec(new_points, hull, secondLine);
}

/* quick_hull
 * points: const vector<Point2D>&, vetor contendo os pontos a ser executado o algoritmo de quick_hull
 * Retorna uma nova lista contendo os pontos que determinam o fecho convexo, em ordem anti-horária.
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
 * Podemos definir fecho convexo como a menor região convexa que contém os pontos do conjunto.
 * Essa região é formada a partir dos pontos extremos do conjunto de pontos de entrada.
 * Com base nisso, pode-se concluir a seguinte afirmação:
 * 
 * Dado um conjunto de pontos P e uma reta R, podemos dizer que todo ponto
 * pertencente a P que é o mais distante da reta R em relação a alguma direção
 * ortogonal de R, faz parte do fecho convexo de P.
 *  
 * Com base no Lema 1, observamos que a etapa inicial de divisão encontra dois
 * pontos (A e B) pertencentes ao fecho, bem como uma reta AB.
 * Pode se aplicar o lema 1 novamente com a reta resultante dos pontos extremos
 * encontrados, com isso
 * TODO
 * 
 * 1) Provar que os pontos mais a esquerda(E) e mais a direita(D)
 * fazem parte do fecho convexo.
 * Assumir que E não faz parte do fecho implica em E estar contido dentro
 * da área formada pelo fecho.
 * Logo, deverá existir um ponto mais a esquerda de E para formar um fecho
 * que contenha E.
 * Isso contradiz nossa suposição inicial de que E é o ponto mais a esquerda,
 * e portanto, por contradição, conclui-se que E faz parte do fecho.
 * Prova análoga para o ponto mais a direita(D).
 * 
 * 2) Provar que os pontos adicionados fazem parte do fecho convexo
 * Utilizando a definição de fecho convexo, podemos concluir que todos os pontos
 * encontrados com a maior distância entre retas formadas por pontos que fazem
 * parte do fecho
 * 
 * Podemos provar o funcionamento da parte recursiva do algoritmo usando laço invariante.
 * 
 * Hipótese:
 * Para todo subvetor de pontos analisados, o algoritmo irá adicionar os pontos que formam 
 * o fecho convexo a esquerda da reta base.
 * Para toda sublista "hull" conterá os pontos do fecho convexo, em ordem
 * anti-horária.
 * 
 * Inicialização:
 * O caso base se dá na primeira recursão, onde é recebido um conjunto de pontos e uma reta base
 * formada pelos pontos extremos dentro do conjunto de pontos incial.
 * Este faz parte do fecho pela prova 1.
 * 
 * Manutenção:
 * A cada recursão, os pontos recebidos são analisados e adicionados a um subvetor auxiliar que
 * contém somente os pontos a esquerda da reta recebida e também é encontrado o ponto mais distante
 * da reta formada pela recursão anterior.
 * 
 * Término:
 * TODO 
 * Começamos adicionando o ponto mais a esquerda do conjunto.
 * Após isso, é feita a primeira chamada recursiva, que irá executar outra recursão
 * sempre no conjunto de pontos a esquerda. Portanto, o próximo ponto a ser adicionado
 * será o segundo a esquerda.
 * Após isso, a recursão irá executar no conjunto de pontos a direita do último ponto adicionado,
 * sendo esse o terceiro a esquerda. Isso se repete, adicion
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
	
	// Realiza as chamadas recursivas, passando o conjunto de pontos de entrada e saída,
	// bem como a linha que fará a divisão entre pontos da esquerda e direita.
	quick_hull_rec(points, hull, line);
	
	// Adiciona o segundo ponto extremo encontrado, uma vez que ele é necessário para
	// completar os pontos dentro do fecho convexo em ordem anti-horária.
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
