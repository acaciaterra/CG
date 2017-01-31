/*
  Trabalho do componente curricular Computação Gráfica,
	realizado pelos alunos Acácia dos Campos da Terra e João Pedro Winckler Bernardi.
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "imagem.h"

GLUnurbsObj *nurbs = NULL;
double cam_x = DEF_CAM_X, cam_y = DEF_CAM_Y, cam_z = DEF_CAM_Z;

/*
	Todo objeto está definido na seguinte estrutura:
	#define n*_ctrl_u :corresponde a número de control points em u
	#define n*_ctrl_v :corresponde a número de control points em v
	GLfloat knots_*_u[n*_ctrl_u + 4], knots_*_v[n*_ctrl_v + 4] : correspondem aos knots do objeto, que são definidas na função init
	GLfloat ctrlpoints_*[n*_ctrl_u][n*_ctrl_v][4] : corresponde ao vetor que contem todas as curvas que formam o objeto
*/

#define nt_ctrl_u 13
#define nt_ctrl_v 7

GLfloat knots_taca_u[nt_ctrl_u + 4], knots_taca_v[nt_ctrl_v + 4];
GLfloat ctrlpoints_taca[nt_ctrl_u][nt_ctrl_v][4] = {
	{{3.5, 8.0, 0.0, 1.0}, {3.5, 8.0, -3.5, 0.5}, {-3.5, 8.0, -3.5, 0.5}, {-3.5, 8.0, 0.0, 1.0}, {-3.5, 8.0, 3.5, 0.5}, {3.5, 8.0, 3.5, 0.5}, {3.5, 8.0, 0.0, 1.0}},
	{{3.75, 7.0, 0.0, 1.0}, {3.75, 7.0, -3.75, 0.5}, {-3.75, 7.0, -3.75, 0.5}, {-3.75, 7.0, 0.0, 1.0}, {-3.75, 7.0, 3.75, 0.5}, {3.75, 7.0, 3.75, 0.5}, {3.75, 7.0, 0.0, 1.0}},
	{{4, 6.0, 0.0, 1.0}, {4, 6.0, -4, 0.5}, {-4, 6.0, -4, 0.5}, {-4, 6.0, 0.0, 1.0}, {-4, 6.0, 4, 0.5}, {4, 6.0, 4, 0.5}, {4, 6.0, 0.0, 1.0}},
	{{4.0, 5.0, 0.0, 1.0}, {4.0, 5.0, -4.0, 0.5}, {-4.0, 5.0, -4.0, 0.5}, {-4.0, 5.0, 0.0, 1.0}, {-4.0, 5.0, 4.0, 0.5}, {4.0, 5.0, 4.0, 0.5}, {4.0, 5.0, 0.0, 1.0}},
	{{3.5, 4.0, 0.0, 1.0}, {3.5, 4.0, -3.5, 0.5}, {-3.5, 4.0, -3.5, 0.5}, {-3.5, 4.0, 0.0, 1.0}, {-3.5, 4.0, 3.5, 0.5}, {3.5, 4.0, 3.5, 0.5}, {3.5, 4.0, 0.0, 1.0}},
	{{3.0, 3.0, 0.0, 1.0}, {3.0, 3.0, -3.0, 0.5}, {-3.0, 3.0, -3.0, 0.5}, {-3.0, 3.0, 0.0, 1.0}, {-3.0, 3.0, 3.0, 0.5}, {3.0, 3.0, 3.0, 0.5}, {3.0, 3.0, 0.0, 1.0}},
	{{1.0, 2.0, 0.0, 1.0}, {1.0, 2.0, -1.0, 0.5}, {-1.0, 2.0, -1.0, 0.5}, {-1.0, 2.0, 0.0, 1.0}, {-1.0, 2.0, 1.0, 0.5}, {1.0, 2.0, 1.0, 0.5}, {1.0, 2.0, 0.0, 1.0}},
	{{3.0, 1.0, 0.0, 1.0}, {3.0, 1.0, -3.0, 0.5}, {-3.0, 1.0, -3.0, 0.5}, {-3.0, 1.0, 0.0, 1.0}, {-3.0, 1.0, 3.0, 0.5}, {3.0, 1.0, 3.0, 0.5}, {3.0, 1.0, 0.0, 1.0}},
	{{4.0, 0.0, 0.0, 1.0}, {4.0, 0.0, -4.0, 0.5}, {-4.0, 0.0, -4.0, 0.5}, {-4.0, 0.0, 0.0, 1.0}, {-4.0, 0.0, 4.0, 0.5}, {4.0, 0.0, 4.0, 0.5}, {4.0, 0.0, 0.0, 1.0}},
	{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}},
	{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}},
	{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}},
	{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 1.0}}
};


#define ns_ctrl_u 18
#define ns_ctrl_v 7

GLfloat knots_shampoo_u[ns_ctrl_u + 4], knots_shampoo_v[ns_ctrl_v + 4];
GLfloat ctrlpoints_shampoo[ns_ctrl_u][ns_ctrl_v][4] = {
	{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 1.0}},
	{{1.0, 0.0, 0.0, 1.0}, {1.0, 0.0, -1.0, 0.5}, {-1.0, 0.0, -1.0, 0.5}, {-1.0, 0.0, 0.0, 1.0}, {-1.0, 0.0, 1.0, 0.5}, {1.0, 0.0, 1.0, 0.5}, {1.0, 0.0, 0.0, 1.0}},
	{{1.0, 0.0, 0.0, 1.0}, {1.0, 0.0, -1.0, 0.5}, {-1.0, 0.0, -1.0, 0.5}, {-1.0, 0.0, 0.0, 1.0}, {-1.0, 0.0, 1.0, 0.5}, {1.0, 0.0, 1.0, 0.5}, {1.0, 0.0, 0.0, 1.0}},
	{{1.0, 1.0, 0.0, 1.0}, {1.0, 1.0, -1.0, 0.5}, {-1.0, 1.0, -1.0, 0.5}, {-1.0, 1.0, 0.0, 1.0}, {-1.0, 1.0, 1.0, 0.5}, {1.0, 1.0, 1.0, 0.5}, {1.0, 1.0, 0.0, 1.0}},
	{{1.0, 2, 0.0, 1.0}, {1.0, 2, -1.0, 0.5}, {-1.0, 2, -1.0, 0.5}, {-1.0, 2, 0.0, 1.0}, {-1.0, 2, 1.0, 0.5}, {1.0, 2, 1.0, 0.5}, {1.0, 2, 0.0, 1.0}},
	{{1.0, 3, 0.0, 1.0}, {1.0, 3, -1.0, 0.5}, {-1.0, 3, -1.0, 0.5}, {-1.0, 3, 0.0, 1.0}, {-1.0, 3, 1.0, 0.5}, {1.0, 3, 1.0, 0.5}, {1.0, 3, 0.0, 1.0}},
	{{1.0, 4, 0.0, 1.0}, {1.0, 4, -1.0, 0.5}, {-1.0, 4, -1.0, 0.5}, {-1.0, 4, 0.0, 1.0}, {-1.0, 4, 1.0, 0.5}, {1.0, 4, 1.0, 0.5}, {1.0, 4, 0.0, 1.0}},
	{{0.8, 4.25, 0.0, 1.0}, {0.8, 4.25, -0.8, 0.5}, {-0.8, 4.25, -0.8, 0.5}, {-0.8, 4.25, 0.0, 1.0}, {-0.8, 4.25, 0.8, 0.5}, {0.8, 4.25, 0.8, 0.5}, {0.8, 4.25, 0.0, 1.0}},
	{{0.65, 4.5, 0.0, 1.0}, {0.65, 4.5, -0.65, 0.5}, {-0.65, 4.5, -0.65, 0.5}, {-0.65, 4.5, 0.0, 1.0}, {-0.65, 4.5, 0.65, 0.5}, {0.65, 4.5, 0.65, 0.5}, {0.65, 4.5, 0.0, 1.0}},
	{{0.5, 4.75, 0.0, 1.0}, {0.5, 4.75, -0.5, 0.5}, {-0.5, 4.75, -0.5, 0.5}, {-0.5, 4.75, 0.0, 1.0}, {-0.5, 4.75, 0.5, 0.5}, {0.5, 4.75, 0.5, 0.5}, {0.5, 4.75, 0.0, 1.0}},
	{{0.5, 4.75, 0.0, 1.0}, {0.5, 4.75, -0.5, 0.5}, {-0.5, 4.75, -0.5, 0.5}, {-0.5, 4.75, 0.0, 1.0}, {-0.5, 4.75, 0.5, 0.5}, {0.5, 4.75, 0.5, 0.5}, {0.5, 4.75, 0.0, 1.0}},
	{{0.65, 5, 0.0, 1.0}, {0.65, 5, -0.65, 0.5}, {-0.65, 5, -0.65, 0.5}, {-0.65, 5, 0.0, 1.0}, {-0.65, 5, 0.65, 0.5}, {0.65, 5, 0.65, 0.5}, {0.65, 5, 0.0, 1.0}},
	{{0.65, 5, 0.0, 1.0}, {0.65, 5, -0.65, 0.5}, {-0.65, 5, -0.65, 0.5}, {-0.65, 5, 0.0, 1.0}, {-0.65, 5, 0.65, 0.5}, {0.65, 5, 0.65, 0.5}, {0.65, 5, 0.0, 1.0}},
	{{0.35, 5.25, 0.0, 1.0}, {0.35, 5.25, -0.35, 0.5}, {-0.35, 5.25, -0.35, 0.5}, {-0.35, 5.25, 0.0, 1.0}, {-0.35, 5.25, 0.35, 0.5}, {0.35, 5.25, 0.35, 0.5}, {0.35, 5.25, 0.0, 1.0}},
	{{0.35, 5.25, 0.0, 1.0}, {0.35, 5.25, -0.35, 0.5}, {-0.35, 5.25, -0.35, 0.5}, {-0.35, 5.25, 0.0, 1.0}, {-0.35, 5.25, 0.35, 0.5}, {0.35, 5.25, 0.35, 0.5}, {0.35, 5.25, 0.0, 1.0}},
	{{0.35, 5.60, 0.0, 1.0}, {0.35, 5.60, -0.35, 0.5}, {-0.35, 5.60, -0.35, 0.5}, {-0.35, 5.60, 0.0, 1.0}, {-0.35, 5.60, 0.35, 0.5}, {0.35, 5.60, 0.35, 0.5}, {0.35, 5.60, 0.0, 1.0}},
	{{0.0, 5.60, 0.0, 1.0}, {0.0, 5.60, -0.0, 0.5}, {-0.0, 5.60, -0.0, 0.5}, {-0.0, 5.60, 0.0, 1.0}, {-0.0, 5.60, 0.0, 0.5}, {0.0, 5.60, 0.0, 0.5}, {0.0, 5.60, 0.0, 1.0}},
	{{0.0, 5.60, 0.0, 1.0}, {0.0, 5.60, -0.0, 0.5}, {-0.0, 5.60, -0.0, 0.5}, {-0.0, 5.60, 0.0, 1.0}, {-0.0, 5.60, 0.0, 0.5}, {0.0, 5.60, 0.0, 0.5}, {0.0, 5.60, 0.0, 1.0}},
};

#define nl_ctrl_u 6
#define nl_ctrl_v 7

GLfloat knots_lamp_u[nl_ctrl_u + 4], knots_lamp_v[nl_ctrl_v + 4];
GLfloat ctrlpoints_lamp[nl_ctrl_u][nl_ctrl_v][4] = {
	{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, -0.0, 0.5}, {-0.0, 0.0, -0.0, 0.5}, {-0.0, 0.0, 0.0, 1.0}, {-0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 1.0}},
	{{0.35, 0.0, 0.0, 1.0}, {0.35, 0.0, -0.35, 0.5}, {-0.35, 0.0, -0.35, 0.5}, {-0.35, 0.0, 0.0, 1.0}, {-0.35, 0.0, 0.35, 0.5}, {0.35, 0.0, 0.35, 0.5}, {0.35, 0.0, 0.0, 1.0}},
	{{0.35, 1.0, 0.0, 1.0}, {0.35, 1.0, -0.35, 0.5}, {-0.35, 1.0, -0.35, 0.5}, {-0.35, 1.0, 0.0, 1.0}, {-0.35, 1.0, 0.35, 0.5}, {0.35, 1.0, 0.35, 0.5}, {0.35, 1.0, 0.0, 1.0}},
	{{0.35, 2.0, 0.0, 1.0}, {0.35, 2.0, -0.35, 0.5}, {-0.35, 2.0, -0.35, 0.5}, {-0.35, 2.0, 0.0, 1.0}, {-0.35, 2.0, 0.35, 0.5}, {0.35, 2.0, 0.35, 0.5}, {0.35, 2.0, 0.0, 1.0}},
	{{0.35, 3.0, 0.0, 1.0}, {0.35, 3.0, -0.35, 0.5}, {-0.35, 3.0, -0.35, 0.5}, {-0.35, 3.0, 0.0, 1.0}, {-0.35, 3.0, 0.35, 0.5}, {0.35, 3.0, 0.35, 0.5}, {0.35, 3.0, 0.0, 1.0}},
	{{0, 3.0, 0.0, 1.0}, {0, 3.0, -0, 0.5}, {-0, 3.0, -0, 0.5}, {-0, 3.0, 0.0, 1.0}, {-0, 3.0, 0, 0.5}, {0, 3.0, 0, 0.5}, {0, 3.0, 0.0, 1.0}},
};

#define nc_ctrl_u 4
#define nc_ctrl_v 28

GLfloat knots_chave_u[nc_ctrl_u + 4], knots_chave_v[nc_ctrl_v + 4];
GLfloat ctrlpoints_chave[nc_ctrl_u][nc_ctrl_v][4] = {
	{{0, -4, 0,  1},	{-3.826, -2.498, 0,  0.707},	{-4, 0, 0,  1},	{-3.463, 2.747, 0,  0.707},	{-0.597, 3.028, 0,  1},	{2.388, 2.209, 0,  0.707},	{2.747, -1.851, 0,  0.707},	{0.299, -3.344, 0,  1},	{0.119, -3.403, 0,  1}, {0.776, -9.255, 0,  1},	{0.671, -9.23, 0,  1},	{-0.336, -10.321, 0,  1},	{-1.343, -9.314, 0,  1},	{-1.51, -9.398, 0,  1},	{-0.839, -8.223, 0,  1},	{-0.839, -8.223, 0,  1},	{-1.426, -7.132, 0,  1},	{-1.51, -7.132, 0,  1},	{-0.839, -6.377, 0,  1},	{-0.755, -6.293, 0,  1},	{-1.175, -5.118, 0,  1},	{-1.259, -5.202, 0,  1},	{-1.405, -3.38, 0,  1},	{-1.424, -3.39, 0,  1},	{-3.012, -2.421, 0,  1},	{-3.271, -2.137, 0,  1},	{-3.71, -1.309, 0,  1},	{-3.341, -1.836, 0,  1}},
	{{0, 0, 0, 1},{0, 0, 0, 0.707},{0, 0, 0, 1},{0, 0, 0, 0.707},{0, 0, 0, 1},{0, 0, 0, 707},{0, 0, 0, 707},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1}},
	{{0, 0, 0, 1},{0, 0, 0, 0.707},{0, 0, 0, 1},{0, 0, 0, 0.707},{0, 0, 0, 1},{0, 0, 0, 707},{0, 0, 0, 707},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1}},
	{{0, 0, 0, 1},{0, 0, 0, 0.707},{0, 0, 0, 1},{0, 0, 0, 0.707},{0, 0, 0, 1},{0, 0, 0, 707},{0, 0, 0, 707},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1},{0, 0, 0, 1}},
};

#define nlb_ctrl_u 8
#define nlb_ctrl_v 7

GLfloat knots_basel_u[nlb_ctrl_u + 4], knots_basel_v[nlb_ctrl_v + 4];
GLfloat ctrlpoints_basel[nlb_ctrl_u][nlb_ctrl_v][4] = {
	{{1.5, 10.0, 0.0, 1.0}, {1.5, 10.0, -1.5, 0.5}, {-1.5, 10.0, -1.5, 0.5}, {-1.5, 10.0, 0.0, 1.0}, {-1.5, 10.0, 1.5, 0.5}, {1.5, 10.0, 1.5, 0.5}, {1.5, 10.0, 0.0, 1.0}},
	{{2.25, 9.0, 0.0, 1.0}, {2.25, 9.0, -2.25, 0.5}, {-2.25, 9.0, -2.25, 0.5}, {-2.25, 9.0, 0.0, 1.0}, {-2.25, 9.0, 2.25, 0.5}, {2.25, 9.0, 2.25, 0.5}, {2.25, 9.0, 0.0, 1.0}},
	{{3, 8.0, 0.0, 1.0}, {3, 8.0, -3, 0.5}, {-3, 8.0, -3, 0.5}, {-3, 8.0, 0.0, 1.0}, {-3, 8.0, 3, 0.5}, {3, 8.0, 3, 0.5}, {3, 8.0, 0.0, 1.0}},
	{{3, 7.0, 0.0, 1.0}, {3, 7.0, -3, 0.5}, {-3, 7.0, -3, 0.5}, {-3, 7.0, 0.0, 1.0}, {-3, 7.0, 3, 0.5}, {3, 7.0, 3, 0.5}, {3, 7.0, 0.0, 1.0}},
	{{3, 6.0, 0.0, 1.0}, {3, 6.0, -3, 0.5}, {-3, 6.0, -3, 0.5}, {-3, 6.0, 0.0, 1.0}, {-3, 6.0, 3, 0.5}, {3, 6.0, 3, 0.5}, {3, 6.0, 0.0, 1.0}},
	{{3, 6.0, 0.0, 1.0}, {3, 6.0, -3, 0.5}, {-3, 6.0, -3, 0.5}, {-3, 6.0, 0.0, 1.0}, {-3, 6.0, 3, 0.5}, {3, 6.0, 3, 0.5}, {3, 6.0, 0.0, 1.0}},
	{{0.0, 6.0, 0.0, 1.0}, {0.0, 6.0, 0.0, 0.5}, {0.0, 6.0, 0.0, 0.5}, {0.0, 6.0, 0.0, 1.0}, {0.0, 6.0, 0.0, 0.5}, {0.0, 6.0, 0.0, 0.5}, {0, 6.0, 0.0, 1.0}},
	{{0.0, 6.0, 0.0, 1.0}, {0.0, 6.0, 0.0, 0.5}, {0.0, 6.0, 0.0, 0.5}, {0.0, 6.0, 0.0, 1.0}, {0.0, 6.0, 0.0, 0.5}, {0.0, 6.0, 0.0, 0.5}, {0, 6.0, 0.0, 1.0}}
};

#define nr_ctrl_u 4
#define nr_ctrl_v 7

GLfloat knots_rosca_u[nr_ctrl_u + 4], knots_rosca_v[nr_ctrl_v + 4];
GLfloat ctrlpoints_rosca[nr_ctrl_u][nr_ctrl_v][4] = {
	{{1.5, 10.0, 0.0, 1.0}, {1.5, 10.0, -1.5, 0.5}, {-1.5, 10.0, -1.5, 0.5}, {-1.5, 10.0, 0.0, 1.0}, {-1.5, 10.0, 1.5, 0.5}, {1.5, 10.0, 1.5, 0.5}, {1.5, 10.0, 0.0, 1.0}},
	{{1.5, 11.0, 0.0, 1.0}, {1.5, 11.0, -1.5, 0.5}, {-1.5, 11.0, -1.5, 0.5}, {-1.5, 11.0, 0.0, 1.0}, {-1.5, 11.0, 1.5, 0.5}, {1.5, 11.0, 1.5, 0.5}, {1.5, 11.0, 0.0, 1.0}},
	{{1.5, 12.0, 0.0, 1.0}, {1.5, 12.0, -1.5, 0.5}, {-1.5, 12.0, -1.5, 0.5}, {-1.5, 12.0, 0.0, 1.0}, {-1.5, 12.0, 1.5, 0.5}, {1.5, 12.0, 1.5, 0.5}, {1.5, 12.0, 0.0, 1.0}},
	{{0, 12.0, 0.0, 1.0}, {0, 12.0, -0, 0.5}, {-0, 12.0, -0, 0.5}, {-0, 12.0, 0.0, 1.0}, {-0, 12.0, 0, 0.5}, {0, 12.0, 0, 0.5}, {0, 12.0, 0.0, 1.0}},
};

void init(void) {
	glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  const GLfloat light0_position[] = {20.0, 20.0, 0.1, 0.1};
  const GLfloat light2_position[] = {-20.0, 20.0, 0.1, 0.1};
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
  glLightfv(GL_LIGHT2, GL_POSITION, light2_position);

  const GLfloat light_ambient[] = {0.0, 0.0, 0.0, 0.5};
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT2, GL_AMBIENT, light_ambient);

  const GLfloat light_color[] = {1.0, 1.0, 1.0, 0.3};
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, light_color);
  glLightfv(GL_LIGHT2, GL_SPECULAR, light_color);

  const GLfloat light_ambient_global[] = {0.5, 0.5, 0.5, 1.0};
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambient_global);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT2);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	const GLfloat material_specular[] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
	const GLfloat material_shininess[] = {128.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, material_shininess);

	glShadeModel(GL_SMOOTH);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	nurbs = gluNewNurbsRenderer();
	gluNurbsProperty(nurbs, GLU_NURBS_MODE, GLU_NURBS_RENDERER);
	gluNurbsProperty(nurbs, GLU_SAMPLING_METHOD, GLU_PATH_LENGTH);
	gluNurbsProperty(nurbs, GLU_SAMPLING_TOLERANCE, 25.0);
	gluNurbsProperty(nurbs, GLU_DISPLAY_MODE, GLU_FILL);

	knots_taca_u[0] = 0.0;
	knots_taca_u[1] = 0.0;
	knots_taca_u[2] = 0.0;
	knots_taca_u[3] = 0.0;
	knots_taca_u[4] = 0.25;
	knots_taca_u[5] = 0.25;
	knots_taca_u[6] = 0.25;
	knots_taca_u[7] = 0.5;
	knots_taca_u[8] = 0.5;
 	knots_taca_u[9] = 0.75;
	knots_taca_u[10] = 0.75;
	knots_taca_u[11] = 0.99;
	knots_taca_u[12] = 0.99;
	knots_taca_u[13] = 1.0;
	knots_taca_u[14] = 1.0;
	knots_taca_u[15] = 1.0;
	knots_taca_u[16] = 1.0;

	knots_taca_v[0] =  0.0;
	knots_taca_v[1] =  0.0;
	knots_taca_v[2] =  0.0;
	knots_taca_v[3] =  0.0;
	knots_taca_v[4] =  0.5;
	knots_taca_v[5] =  0.5;
	knots_taca_v[6] =  0.5;
	knots_taca_v[7] =  1.0;
	knots_taca_v[8] =  1.0;
	knots_taca_v[9] =  1.0;
	knots_taca_v[10] =  1.0;

	knots_shampoo_u[0] = 0.0;
	knots_shampoo_u[1] = 0.0;
	knots_shampoo_u[2] = 0.0;
	knots_shampoo_u[3] = 0.0;

	knots_shampoo_u[4] = 0.25;
	knots_shampoo_u[5] = 0.25;
	knots_shampoo_u[6] = 0.37;
	knots_shampoo_u[7] = 0.37;
	knots_shampoo_u[8] = 0.5;
	knots_shampoo_u[9] = 0.5;
	knots_shampoo_u[10] = 0.5;
	knots_shampoo_u[11] = 0.6;

	knots_shampoo_u[12] = 0.6;
	knots_shampoo_u[13] = 0.6;
	knots_shampoo_u[14] = 0.75;
	knots_shampoo_u[15] = 0.75;
	knots_shampoo_u[16] = 0.99;
	knots_shampoo_u[17] = 0.99;
	knots_shampoo_u[18] = 1.0;
	knots_shampoo_u[19] = 1.0;
	knots_shampoo_u[20] = 1.0;
	knots_shampoo_u[21] = 1.0;

	knots_shampoo_v[0] =  0.0;
	knots_shampoo_v[1] =  0.0;
	knots_shampoo_v[2] =  0.0;
	knots_shampoo_v[3] =  0.0;
	knots_shampoo_v[4] =  0.25;
	knots_shampoo_v[5] =  0.5;
	knots_shampoo_v[6] =  0.75;
	knots_shampoo_v[7] =  1.0;
	knots_shampoo_v[8] =  1.0;
	knots_shampoo_v[9] =  1.0;
	knots_shampoo_v[10] =  1.0;

	knots_lamp_u[0] = 0.0;
	knots_lamp_u[1] = 0.0;
	knots_lamp_u[2] = 0.0;
	knots_lamp_u[3] = 0.0;
	knots_lamp_u[4] = 0.5;
	knots_lamp_u[5] = 0.5;
	knots_lamp_u[6] = 1.0;
	knots_lamp_u[7] = 1.0;
	knots_lamp_u[8] = 1.0;
	knots_lamp_u[9] = 1.0;

	knots_lamp_v[0] =  0.0;
	knots_lamp_v[1] =  0.0;
	knots_lamp_v[2] =  0.0;
	knots_lamp_v[3] =  0.0;
	knots_lamp_v[4] =  0.25;
	knots_lamp_v[5] =  0.5;
	knots_lamp_v[6] =  0.75;
	knots_lamp_v[7] =  1.0;
	knots_lamp_v[8] =  1.0;
	knots_lamp_v[9] =  1.0;
	knots_lamp_v[10] =  1.0;

	knots_basel_u[0] = 0;
	knots_basel_u[1] = 0;
	knots_basel_u[2] = 0;
	knots_basel_u[3] = 0;
	knots_basel_u[4] = 0.25;
	knots_basel_u[5] = 0.5;
	knots_basel_u[6] = 0.5;
	knots_basel_u[7] = 0.75;
	knots_basel_u[8] = 1;
	knots_basel_u[9] = 1;
	knots_basel_u[10] = 1;
	knots_basel_u[11] = 1;

	knots_basel_v[0] =  0.0;
	knots_basel_v[1] =  0.0;
	knots_basel_v[2] =  0.0;
	knots_basel_v[3] =  0.0;
	knots_basel_v[4] =  0.25;
	knots_basel_v[5] =  0.5;
	knots_basel_v[6] =  0.75;
	knots_basel_v[7] =  1.0;
	knots_basel_v[8] =  1.0;
	knots_basel_v[9] =  1.0;
	knots_basel_v[10] =  1.0;

	knots_chave_u[0] = 0;
	knots_chave_u[1] = 0.2;
	knots_chave_u[2] = 0.4;
	knots_chave_u[3] = 0.45;
	knots_chave_u[4] = 0.5;
	knots_chave_u[5] = 0.65;
	knots_chave_u[6] = 0.8;
	knots_chave_u[7] = 1;

	knots_chave_v[0] = 0;
	knots_chave_v[1] = 0.032;
	knots_chave_v[2] = 0.065;
	knots_chave_v[3] = 0.097;
	knots_chave_v[4] = 0.129;
	knots_chave_v[5] = 0.161;
	knots_chave_v[6] = 0.194;
	knots_chave_v[7] = 0.226;
	knots_chave_v[8] = 0.258;
	knots_chave_v[9] = 0.290;
	knots_chave_v[10] = 0.323;
	knots_chave_v[11] = 0.355;
	knots_chave_v[12] = 0.387;
	knots_chave_v[13] = 0.419;
	knots_chave_v[14] = 0.452;
	knots_chave_v[15] = 0.484;
	knots_chave_v[16] = 0.516;
	knots_chave_v[17] = 0.548;
	knots_chave_v[18] = 0.581;
	knots_chave_v[19] = 0.613;
	knots_chave_v[20] = 0.645;
	knots_chave_v[21] = 0.677;
	knots_chave_v[22] = 0.710;
	knots_chave_v[23] = 0.742;
	knots_chave_v[24] = 0.774;
	knots_chave_v[25] = 0.806;
	knots_chave_v[26] = 0.839;
	knots_chave_v[27] = 0.871;
	knots_chave_v[28] = 0.903;
	knots_chave_v[29] = 0.935;
	knots_chave_v[30] = 0.968;
	knots_chave_v[31] = 1;

	knots_rosca_u[0] = 0;
	knots_rosca_u[1] = 0;
	knots_rosca_u[2] = 0;
	knots_rosca_u[3] = 0;
	knots_rosca_u[4] = 1;
	knots_rosca_u[5] = 1;
	knots_rosca_u[6] = 1;
	knots_rosca_u[7] = 1;

	knots_rosca_v[0] =  0.0;
	knots_rosca_v[1] =  0.0;
	knots_rosca_v[2] =  0.0;
	knots_rosca_v[3] =  0.0;
	knots_rosca_v[4] =  0.25;
	knots_rosca_v[5] =  0.5;
	knots_rosca_v[6] =  0.75;
	knots_rosca_v[7] =  1.0;
	knots_rosca_v[8] =  1.0;
	knots_rosca_v[9] =  1.0;
	knots_rosca_v[10] =  1.0;
}

/*
	Responsável por desenhar a taça
*/
void draw_taca(void) {
	glPushMatrix();
		glColor4dv(taca_color);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, nt_ctrl_u + 4, knots_taca_u, nt_ctrl_v + 4, knots_taca_v, nt_ctrl_v * 4, 4, &ctrlpoints_taca[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
}

/*
	Responsável por desenhar a chave
*/
void draw_chave(void) {
	glPushMatrix();
		glColor3ubv(chave_color);
		glRotated(60, 0, 1, 0);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, nc_ctrl_u + 4, knots_chave_u, nc_ctrl_v + 4, knots_chave_v, nc_ctrl_v * 4, 4, &ctrlpoints_chave[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
}

/*
	Responsável por desenhar o shampoo
*/
void draw_shampoo(void) {
	glPushMatrix();
		glColor4ubv(shampoo_color);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, ns_ctrl_u + 4, knots_shampoo_u, ns_ctrl_v + 4, knots_shampoo_v, ns_ctrl_v * 4, 4, &ctrlpoints_shampoo[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
}

/*
	Responsável por desenhar a base da lâmpada
*/
void draw_basel(void) {
	glPushMatrix();
		glColor4ubv(lamp_color);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, nlb_ctrl_u + 4, knots_basel_u, nlb_ctrl_v + 4, knots_basel_v, nlb_ctrl_v * 4, 4, &ctrlpoints_basel[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
}

/*
	Responsável por desenhar a rosca da lâmpada
*/
void draw_rosca(void) {
	glPushMatrix();
		glColor4ubv(lamp1_color);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, nr_ctrl_u + 4, knots_rosca_u, nr_ctrl_v + 4, knots_rosca_v, nr_ctrl_v * 4, 4, &ctrlpoints_rosca[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
}

/*
	Responsável por desenhar um dos arcos de onde sai a luz da lâmpada
*/
void draw_lamp(void) {
	glPushMatrix();
		glScalef(1, 1.5, 1);
		glColor4ubv(lamp_color);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, nl_ctrl_u + 4, knots_lamp_u, nl_ctrl_v + 4, knots_lamp_v, nl_ctrl_v * 4, 4, &ctrlpoints_lamp[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
	glPushMatrix();
		glScalef(1, 1.5, 1);
		glColor4ubv(lamp_color);
		glTranslated(1, 0, 0);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, nl_ctrl_u + 4, knots_lamp_u, nl_ctrl_v + 4, knots_lamp_v, nl_ctrl_v * 4, 4, &ctrlpoints_lamp[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
	glPushMatrix();
	glRotated(90, 1, 0, 0);
	glPushMatrix();
		glColor4ubv(lamp_color);
		glRotated(90, 0, 0, 1);
		glTranslated(0, -1.25, -0.3);
		glScalef(0.8, 0.5, 1);
		gluBeginSurface(nurbs);
		gluNurbsSurface(nurbs, nl_ctrl_u + 4, knots_lamp_u, nl_ctrl_v + 4, knots_lamp_v, nl_ctrl_v * 4, 4, &ctrlpoints_lamp[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
		gluEndSurface(nurbs);
	glPopMatrix();
	glPopMatrix();
}

/*
	Responsável por desenhar os três arcos de onde sai a luz da lâmpada
*/
void draw_lampall(void) {
	glPushMatrix();
	draw_lamp();
	glPopMatrix();

	glPushMatrix();
	glRotated(120, 0, 1, 0);
	glTranslated(-2, 0, -1);
	draw_lamp();
	glPopMatrix();

	glPushMatrix();
	glRotated(240, 0, 1, 0);
	glTranslated(0, 0, -1.85);
	draw_lamp();
	glPopMatrix();
}

/*
	Responsável por desenhar a lâmpada inteira
*/
void draw_lamp1() {
	glPushMatrix();
	glScalef(1.7, 2.5, 1.7);
	glTranslated(-0.5, 0, 0);
	glTranslated(0, 0, -1.1);
	glTranslated(0, -1.5, 0);
	draw_lampall();
	glPopMatrix();

	glPushMatrix();
	draw_basel();
	glPopMatrix();

	glPushMatrix();
	draw_rosca();
	glPopMatrix();
}

/*
	Função onde tudo é desenhado
*/
void display(void) {
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClearColor(0, 0, 0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	gluLookAt(cam_x, cam_y, cam_z, CENTER_X, CENTER_Y, CENTER_Z, 0.0, 1.0, 0.0);

	glPushMatrix();
		glTranslated(0, 0, -10);
		glTranslated(-10, 0, 0);
		glTranslated(0, -2.6, 0);
		glScalef(3, 3, 3);
		glRotated(90, 1, 0, 0);
		glRotated(90, 0, 1, 0);
		glRotated(180, 0, 1, 0);
		glRotated(30, 0, 1, 0);
		draw_chave();
	glPopMatrix();

	glPushMatrix();
		glTranslated(0, -2.9, 0);
		glTranslated(0, 0, -12);
		glTranslated(10, 0, 0);
		glScalef(3.5, 3.5, 3.5);
		draw_shampoo();
	glPopMatrix();

	glPushMatrix();
		glTranslated(0.0, 0.0, -20);
		glRotated(30, 0, 1, 0);
		glRotated(90, 1, 0, 0);
		draw_lamp1();
	glPopMatrix();

	glPushMatrix();
		glColor3ubv(chao_color);
		glTranslated(0, -3.6, -10);
		glScalef(30, 1, 30);
		glutSolidCube(1);
	glPopMatrix();

	glPushMatrix();
		glTranslated(0.0, -2.8, 0.0);
		glScalef(0.75f, 1.2f, 1.0f);
		draw_taca();
	glPopMatrix();

	glutSwapBuffers();
}

void reshape(int w, int h) {
	if (h == 0) h = 1;
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double) w / h, 0.1, 150.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void idle(void) {
	glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y) {
  switch (key) {
		case 27: // ESC
			exit(0);
		case '+':
			cam_y = 30.0;
			break;
		case '-':
			cam_y = DEF_CAM_Y;
			break;
    case '1':
			cam_x = DEF_CAM_X;
			cam_z = DEF_CAM_Z;
			break;
    case '2':
			cam_x = 50.0;
			cam_z = 0.0;
			break;
		case '3':
			cam_x = 50.0;
			cam_z = -40;
			break;
    case '4':
			cam_x = -50;
			cam_z = 40;
			break;
    case '5':
			cam_x = -40.0;
			cam_z = -40.0;
			break;
  }
}


int main(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(720, 720);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("T2 - Acacia e Joao");
	init();
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutMainLoop();
	return 0;
}
