#include <iostream>
#include <fstream>
#include <list>
#include <cmath>
#include <limits>

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
std::list<Point2D> read_input(std::string filename) {
	std::list<Point2D> points;
	std::ifstream file(filename);
	std::string line;
	std::getline(file, line);
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
	return std::make_pair(line.second, line.first);
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
Line get_extreme_points(const std::list<Point2D>& points) {
	Point2D start(INF); // Começo da reta, ponto mais a esquerda
	Point2D end(-INF); // Começo da reta, ponto mais a direita
	for (const Point2D& i : points) {
		if (i.x < start.x)
			start = i;
		if (i.x > end.x)
			end = i;
	}

	return std::make_pair(start, end);
}

/* left
 * line: Line, par de Point2D representando uma reta
 * p3: Point2D, ponto para verificar a posicao
 * Retorna a posicao entre uma reta (indicada por line) e um ponto (p3)
 * 1 caso o p3 esteja a ESQUERDA da reta line
 * 0 caso o p3 esteja a DIREITA ou seja COLINEAR da reta line
 */
int left(Line line, const Point2D& p3) {
	Point2D p1 = line.first;
	Point2D p2 = line.second;
    int area = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);

	return area > 0;
}

/* get_point_distance_from_line
 * line: Line, par de Point2D representando uma reta
 * p3: Point2D, ponto para verificar a distancia
 * Retorna a distancia entre uma reta (indicada por line) e um ponto (p3)
 */
float get_point_distance_from_line(Line line, const Point2D& p3) {
	Point2D p1 = line.first;
	Point2D p2 = line.second;
    int num = std::abs((p2.x - p1.x) * (p1.y - p3.y) - (p1.x - p3.x) * (p2.y - p1.y));
    int dem = (p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y);

    return (float) num / sqrt(dem);
}


void print_points(const std::list<Point2D>& points) {
	for (const Point2D& i : points) {
		std::cout << i.x << ", " << i.y << std::endl;
	}
}

void quick_hull(const std::list<Point2D>& points, std::list<Point2D>& hull, Line line) {
	std::list<Point2D> new_points;

	// Para cada ponto no fecho, verificar se está a esquerda da reta
	for (const Point2D& i : points) {
		if (left(line, i))
			new_points.push_back(i);
	}
	if (new_points.size() == 0)
		return;

	// Para todos pontos a esquerda da reta, verificar o que estiver mais longe
	Point2D farthest;
	float longest_distance = -INF;
	for (const Point2D& i : new_points) {
		float distance = get_point_distance_from_line(line, i);
		if (distance > longest_distance) {
			longest_distance = distance;
			farthest = i;
		}
	}

	// Formar duas retas que ligam até o ponto mais longe
	Line firstLine = std::make_pair(line.first, farthest);
	Line secondLine = std::make_pair(farthest, line.second);

	// Repetir o loop
	quick_hull(new_points, hull, firstLine);

	hull.push_back(farthest);

	quick_hull(new_points, hull, secondLine);
}

int main(int argc, char* argv[]) {
	if (argc < 2) {
		std::cout << "Usage: ./hull <filename>" << std::endl;
		std::cout << "Example : ./hull input.txt" << std::endl;
		return 1;
	}
	std::string filename = argv[1];
	std::list<Point2D> points = read_input(filename);

	// Pegar os pontos extremos, formando uma reta
	Line line = get_extreme_points(points);
	std::list<Point2D> hull { line.first };
	quick_hull(points, hull, line);
	hull.push_back(line.second);
	quick_hull(points, hull, reverse_line(line));

	write_output("fecho.txt", hull);

	return 0;
}
