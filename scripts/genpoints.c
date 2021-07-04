#include<stdio.h>
#include<stdlib.h>
#include<ctype.h>
#include<string.h>
#include<time.h>

#define MAX_XAXIS 1024
#define MAX_YAXIS 1024
#define MAX_POINTS MAX_XAXIS*MAX_YAXIS

/* Funcao para verificar se argumento eh inteiro positivo. */
int ehInteiroPositivo(char number[])
{
    for (int i = 0; number[i] != '\0'; i++)
    {
        if (!isdigit(number[i]))
            return 0;
    }
    return 1;
}

int main(int argc, char *argv[]){

	/* Verifica argumento. */
	if (argc != 2)
	{
		fprintf(stderr, "Uso: %s nro_pontos\n", argv[0]);
		return 1;
	}

	if (!ehInteiroPositivo(argv[1]))
	{
		fprintf(stderr, "nro de pontos (%s) nao eh valido.\n", argv[1]);
		return 1;
	}

	/* Recupera tamanho. */
	int npontos = atoi(argv[1]);
	
	/* Verifica se esta dentro dos limites dos eixos. */
	if (npontos > MAX_POINTS)
	{
		fprintf(stderr, "nro de pontos deve ser menor do que %d.\n", MAX_POINTS);
		return 1;
	}

	/* Cria matrix para verificar pontos sorteados. */
	char pontos[MAX_YAXIS][MAX_XAXIS];
	memset(pontos, 0, MAX_POINTS*sizeof(char));

	/* Salva arquivo de entrada. */
	FILE *fp = fopen("input.txt", "w");
	if (fp == NULL)
	{
		fprintf(stderr, "Falha ao criar input.txt.\n");
		return 1;
	}

	/* Cria os pontos aleatorios. */
	int xcontrole = MAX_XAXIS;
	int ycontrole = MAX_YAXIS; 
	srand(time(NULL));
	int n = 1;
	fprintf(fp, "%d\n", npontos);
	while (n <= npontos)
	{
		int x = (int)(rand()%MAX_XAXIS);
		int y = (int)(rand()%MAX_YAXIS);
		
		if (!pontos[x][y])
		{
			n++;
			pontos[x][y] = 1;
			fprintf(fp, "%d %d\n", x, y);
		}
	}

	fclose(fp);

	return 0;
}
