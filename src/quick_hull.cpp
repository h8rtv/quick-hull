#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>
#include <vector>
#include <cmath>
#include <limits>
#include <string>
#include <chrono>

#define INF std::numeric_limits<int>::max()

/*

*/
class Point2D {
public:
    int x;
    int y;

    Point2D (int x = 0, int y = 0) {
        this->x = x;
        this->y = y;
    }

    Point2D (const Point2D& point) {
        x = point.x;
        y = point.y;
    }
	void operator=(const Point2D& point) {
        x = point.x;
        y = point.y;
	}
};

typedef const std::pair<Point2D, Point2D> Line;

/* read_input
 * filename: std::string, nome do arquivo que contém os pontos de entrada
 * Retorna uma lista encadeada com pontos bidimensionais
 */
std::vector<Point2D> read_input(std::string filename) {
	std::vector<Point2D> points;
	std::ifstream file(filename);
	std::string line;
	std::getline(file, line);
	size_t capacity = stoi(line);
	points.reserve(capacity);
	while (std::getline(file, line)) {
		std::size_t index = line.find(' ');
		int x = stoi(line.substr(0, index));
		int y = stoi(line.substr(index + 1));
		Point2D point(x, y);
		points.push_back(point);
	}

	return points;
}

Line reverse_line(Line line) {
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
 */
void write_output(std::string filename, const std::list<Point2D>& hull) {
	std::ofstream file(filename);
	for (const Point2D& i : hull) {
		file << i.x << " " << i.y << std::endl;
	}
}

/* get_extreme_points
 * points: std::list<Point2D>, pontos que estao dentro do fecho
 * Retorna dois pontos que formam uma linha que atravessa o fecho inteiro
 */
Line get_extreme_points(const std::vector<Point2D>& points) {
	Point2D start(INF); // Começo da reta, ponto mais a esquerda
	Point2D end(-INF); // Começo da reta, ponto mais a direita
	for (const Point2D& i : points) {
		if (i.x < start.x)
			start = i;
		if (i.x > end.x)
			end = i;
	}

	return Line(start, end);
}

/* left
 * line: Line, par de Point2D representando uma reta
 * p3: Point2D, ponto para verificar a posicao
 * Retorna a posicao entre uma reta (indicada por line) e um ponto (p3)
 * 1(true) caso o p3 esteja a ESQUERDA da reta line
 * 0(false) caso o p3 esteja a DIREITA ou seja COLINEAR da reta line
 */
bool left(Line line, const Point2D& p3) {
	const Point2D& p1 = line.first;
	const Point2D& p2 = line.second;
    int area = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

	return area > 0;
}

/* get_point_distance_from_line
 * line: Line, par de Point2D representando uma reta
 * p3: Point2D, ponto para verificar a distancia
 * Retorna a distancia entre uma reta (indicada por line) e um ponto (p3)
 */
float get_point_distance_from_line(Line line, const Point2D& p3) {
	const Point2D& p1 = line.first;
	const Point2D& p2 = line.second;
    int num = std::abs((p2.x - p1.x) * (p1.y - p3.y) - (p1.x - p3.x) * (p2.y - p1.y));
    int dem = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);

    return (float) num / sqrt(dem);
}


void print_points(const std::list<Point2D>& points) {
	for (const Point2D& i : points) {
		std::cout << i.x << ", " << i.y << std::endl;
	}
}

void quick_hull_rec(const std::vector<Point2D>& points, std::list<Point2D>& hull, Line line) {
	std::vector<Point2D> new_points;
	size_t capacity = points.size() / 2;
	new_points.reserve(capacity);

	// Para cada ponto no fecho, verificar os que estão a esquerda da reta
	// e verificar o que estiver mais longe
	Point2D farthest;
	float longest_distance = -INF;
	for (const Point2D& i : points) {
		if (left(line, i)) {
			new_points.push_back(i);

			float distance = get_point_distance_from_line(line, i);
			if (distance > longest_distance) {
				longest_distance = distance;
				farthest = i;
			}
		}
	}
	if (new_points.size() == 0)
		return;

	// Formar duas retas que ligam até o ponto mais longe
	Line firstLine = Line(line.first, farthest);
	Line secondLine = Line(farthest, line.second);

	// Repetir o loop
	quick_hull_rec(new_points, hull, firstLine);

	hull.push_back(farthest);

	quick_hull_rec(new_points, hull, secondLine);
}

std::list<Point2D> quick_hull(const std::vector<Point2D>& points) {
	// Pegar os pontos extremos, formando uma reta
	Line line = get_extreme_points(points);
	std::list<Point2D> hull { line.first };
	quick_hull_rec(points, hull, line);
	hull.push_back(line.second);
	quick_hull_rec(points, hull, reverse_line(line));

	return hull;
}

int main(int argc, char* argv[]) {
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

	std::chrono::duration<double> elapsed_time = end - start;
	std::cout << std::fixed << std::setprecision(6);
	std::cout << elapsed_time.count() << std::endl;

	return 0;
}
