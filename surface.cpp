/* Arquivo: surface.cpp
   Autores: Gabriel Batista Galli e Vladimir Belinski
   Descrição: o presente arquivo faz parte da resolução do Trabalho II do CCR Computação Gráfica, 2016-2, do curso de
              Ciência da Computação da Universidade Federal da Fronteira Sul - UFFS, o qual consiste na renderização
              de uma imagem utilizando-se NURBS.
              --> surface.cpp é o principal arquivo do trabalho. Nele são realizadas as configurações de visibilidade,
              iluminação, posicionamento de câmera, aplicação de textura, definição e chamada das funções de desenho,
							gerenciamento de teclado, entre outros.
*/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "surface.h"

unsigned int ih = 0, iw = 0;
unsigned char * counter_texture = NULL;
GLuint *textures = new GLuint[MAXTEXTURES];
float ctex_ix = 0.0, ctex_iy = 0.0, ctex_fx = 1.0, ctex_fy = 1.0;
unsigned char* loadBMP_custom(const char*, unsigned int&, unsigned int&);

GLUnurbsObj *nurbs = NULL;
// the position the camera points to
double center_x = 0.0, center_y = 0.0, center_z = 0.0;
// the position of the camera
double cam_x = DEF_CAM_X, cam_y = DEF_CAM_Y, cam_z = DEF_CAM_Z;

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


#define ns_ctrl_u 10
#define ns_ctrl_v 7

GLfloat knots_test_u[ns_ctrl_u + 4], knots_test_v[ns_ctrl_v + 4];
GLfloat ctrlpoints_test[ns_ctrl_u][ns_ctrl_v][4] = {
	{{0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 0.5}, {0.0, 0.0, 0.0, 1.0}},
	{{1.0, 0.0, 0.0, 1.0}, {1.0, 0.0, -1.0, 0.5}, {-1.0, 0.0, -1.0, 0.5}, {-1.0, 0.0, 0.0, 1.0}, {-1.0, 0.0, 1.0, 0.5}, {1.0, 0.0, 1.0, 0.5}, {1.0, 0.0, 0.0, 1.0}},
	{{1.0, 1.0, 0.0, 1.0}, {1.0, 1.0, -1.0, 0.5}, {-1.0, 1.0, -1.0, 0.5}, {-1.0, 1.0, 0.0, 1.0}, {-1.0, 1.0, 1.0, 0.5}, {1.0, 1.0, 1.0, 0.5}, {1.0, 1.0, 0.0, 1.0}},
	{{1.0, 2, 0.0, 1.0}, {1.0, 2, -1.0, 0.5}, {-1.0, 2, -1.0, 0.5}, {-1.0, 2, 0.0, 1.0}, {-1.0, 2, 1.0, 0.5}, {1.0, 2, 1.0, 0.5}, {1.0, 2, 0.0, 1.0}},
	{{1.0, 3, 0.0, 1.0}, {1.0, 3, -1.0, 0.5}, {-1.0, 3, -1.0, 0.5}, {-1.0, 3, 0.0, 1.0}, {-1.0, 3, 1.0, 0.5}, {1.0, 3, 1.0, 0.5}, {1.0, 3, 0.0, 1.0}},
	{{1.0, 4, 0.0, 1.0}, {1.0, 4, -1.0, 0.5}, {-1.0, 4, -1.0, 0.5}, {-1.0, 4, 0.0, 1.0}, {-1.0, 4, 1.0, 0.5}, {1.0, 4, 1.0, 0.5}, {1.0, 4, 0.0, 1.0}},
	{{0.75, 4.25, 0.0, 1.0}, {0.75, 4.25, -0.75, 0.5}, {-0.75, 4.25, -0.75, 0.5}, {-0.75, 4.25, 0.0, 1.0}, {-0.75, 4.25, 0.75, 0.5}, {0.75, 4.25, 0.75, 0.5}, {0.75, 4.25, 0.0, 1.0}},
	{{0.5, 4.5, 0.0, 1.0}, {0.5, 4.5, -0.5, 0.5}, {-0.5, 4.5, -0.5, 0.5}, {-0.5, 4.5, 0.0, 1.0}, {-0.5, 4.5, 0.5, 0.5}, {0.5, 4.5, 0.5, 0.5}, {0.5, 4.5, 0.0, 1.0}},
	{{0.35, 4.75, 0.0, 1.0}, {0.35, 4.75, -0.35, 0.35}, {-0.35, 4.75, -0.35, 0.35}, {-0.35, 4.75, 0.0, 1.0}, {-0.35, 4.75, 0.35, 0.35}, {0.35, 4.75, 0.35, 0.35}, {0.35, 4.75, 0.0, 1.0}},
	{{0.35, 4.75, 0.0, 1.0}, {0.35, 4.75, -0.35, 0.35}, {-0.35, 4.75, -0.35, 0.35}, {-0.35, 4.75, 0.0, 1.0}, {-0.35, 4.75, 0.35, 0.35}, {0.35, 4.75, 0.35, 0.35}, {0.35, 4.75, 0.0, 1.0}},
	// {{0.5, 5, 0.0, 1.0}, {0.5, 5, -0.5, 0.5}, {-0.5, 5, -0.5, 0.5}, {-0.5, 5, 0.0, 1.0}, {-0.5, 5, 0.5, 0.5}, {0.5, 5, 0.5, 0.5}, {0.5, 5, 0.0, 1.0}},
	// {{0.5, 5, 0.0, 1.0}, {0.5, 5, -0.5, 0.5}, {-0.5, 5, -0.5, 0.5}, {-0.5, 5, 0.0, 1.0}, {-0.5, 5, 0.5, 0.5}, {0.5, 5, 0.5, 0.5}, {0.5, 5, 0.0, 1.0}},
	// {{0.5, 5.25, 0.0, 1.0}, {0.5, 5.25, -0.5, 0.5}, {-0.5, 5.25, -0.5, 0.5}, {-0.5, 5.25, 0.0, 1.0}, {-0.5, 5.25, 0.5, 0.5}, {0.5, 5.25, 0.5, 0.5}, {0.5, 5.25, 0.0, 1.0}},
	// {{0.35, 5.25, 0.0, 1.0}, {0.35, 5.25, -0.35, 0.35}, {-0.35, 5.25, -0.35, 0.35}, {-0.35, 5.25, 0.0, 1.0}, {-0.35, 5.25, 0.35, 0.35}, {0.35, 5.25, 0.35, 0.35}, {0.35, 5.25, 0.0, 1.0}},
	// {{0.35, 8, 0.0, 1.0}, {0.35, 8, -0.35, 0.35}, {-0.35, 8, -0.35, 0.35}, {-0.35, 8, 0.0, 1.0}, {-0.35, 8, 0.35, 0.35}, {0.35, 8, 0.35, 0.35}, {0.35, 8, 0.0, 1.0}},
	// {{0, 8, 0.0, 1.0}, {0, 8, -0, 0}, {-0, 8, -0, 0}, {-0, 8, 0.0, 1.0}, {-0, 8, 0, 0}, {0, 8, 0, 0}, {0, 8, 0.0, 1.0}}
};

void nurbsError(GLenum errorCode) {
	const GLubyte *estring = NULL;
	estring = gluErrorString(errorCode);
	fprintf (stderr, "NURBS Error: %s\n", estring);
	exit(-1);
}

// all OpenGL-related initialization
void init(void) {
	// generation function of textures
	glGenTextures(1, textures);
	// enabling the use of tetxure
	glEnable(GL_TEXTURE_2D);
	// definig the storage form of pixels in the texture (1 means alignment per byte)
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	counter_texture = loadBMP_custom("marble.bmp", iw, ih);
	// association function
	glBindTexture(GL_TEXTURE_2D, textures[0]);
	// definition of the color mix
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	// void glTexImage2D(GLenum target, GLint level, GLint internalFormat, GLsizei width, GLsizei height, GLint border, GLenum format, GLenum type, const GLvoid *data);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, iw, ih, 0, GL_RGB, GL_UNSIGNED_BYTE, counter_texture);
	// GLint gluBuild2DMipmaps(GLenum target, GLint internalFormat, GLsizei width,	GLsizei height, GLenum format, GLenum type, const void *data);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, iw, ih, GL_RGB, GL_UNSIGNED_BYTE, counter_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
	// definition of the color interpolation scheme
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// treatment of points outside the texture space
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	delete counter_texture;

	glMatrixMode(GL_MODELVIEW);
  	glLoadIdentity();

  const GLfloat light0_position[] = {0.0, 5.0, 5.0, 1.0};
  glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

  const GLfloat light_ambient[] = {0.0, 0.0, 0.0, 1.0};
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);

  const GLfloat light_color[] = {1.0, 1.0, 1.0, 1.0};
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);

  const GLfloat light_ambient_global[] = {0.5, 0.5, 0.5, 1.0}; // default is 0.2, 0.2, 0.2, 1.0
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_ambient_global);

  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);

	// "Enable GL_COLOR_MATERIAL and set glColorMaterial to GL_AMBIENT_AND_DIFFUSE.
	// This means that glMaterial will control the polygon's specular and emission
	// colours and the ambient and diffuse will both be set using glColor."
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	const GLfloat material_specular[] = {1.0, 1.0, 1.0, 1.0};
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
	const GLfloat material_shininess[] = {128.0}; // 0 to 128. The higher, the "thinner" the "little white glow"
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, material_shininess);

	glShadeModel(GL_SMOOTH); // GL_SMOOTH is the default

	// definition of how perspective must be treated
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	glEnable(GL_AUTO_NORMAL);
	glEnable(GL_NORMALIZE);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	nurbs = gluNewNurbsRenderer();
	/* value should be set to be either GLU_NURBS_RENDERER or GLU_NURBS_TESSELLATOR.
	 * When set to GLU_NURBS_RENDERER, NURBS objects are tessellated into OpenGL
	 * primitives and sent to the pipeline for rendering. When set to GLU_NURBS_TESSELLATOR,
	 * NURBS objects are tessellated into OpenGL primitives but the vertices, normals, colors,
	 * and/or textures are retrieved back through a callback interface (see gluNurbsCallback).
	 * This allows the user to cache the tessellated results for further processing.
	 * The initial value is GLU_NURBS_RENDERER.
	 **/
	//gluNurbsProperty(nurbs, GLU_NURBS_MODE, GLU_NURBS_RENDERER);
	/* When set to GLU_PATH_LENGTH, the surface is rendered so that the maximum length, in pixels,
	 * of the edges of the tessellation polygons is no greater than what is specified by GLU_SAMPLING_TOLERANCE.
	 * The initial value is GLU_PATH_LENGTH.
	 **/
	gluNurbsProperty(nurbs, GLU_SAMPLING_METHOD, GLU_PATH_LENGTH);
	/* Specifies the maximum length, in pixels or in object space length unit, to use when the
	 * sampling method is set to GLU_PATH_LENGTH or GLU_OBJECT_PATH_LENGTH. The NURBS code is
	 * conservative when rendering a curve or surface, so the actual length can be somewhat shorter.
	 * The initial value is 50.0 pixels.
	 **/
	gluNurbsProperty(nurbs, GLU_SAMPLING_TOLERANCE, 25.0);
	/* value can be set to GLU_OUTLINE_POLYGON, GLU_FILL, or GLU_OUTLINE_PATCH.
	 * When GLU_NURBS_MODE is set to be GLU_NURBS_RENDERER, value defines how a
	 * & NURBS surface should be rendered. When value is set to GLU_FILL, the surface
	 * is rendered as a set of polygons. When value is set to GLU_OUTLINE_POLYGON,
	 * the NURBS library draws only the outlines of the polygons created by tessellation.
	 * When value is set to GLU_OUTLINE_PATCH just the outlines of patches and trim
	 * curves defined by the user are drawn.
	 * The initial value is GLU_FILL.
	 **/
	gluNurbsProperty(nurbs, GLU_DISPLAY_MODE, GLU_FILL);
	gluNurbsCallback(nurbs, GLU_ERROR, (GLvoid (*)()) nurbsError);

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
	knots_taca_u[12] = 1.0;
	knots_taca_u[13] = 1.0;
	knots_taca_u[14] = 1.0;
	knots_taca_u[15] = 1.0;

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

	knots_test_u[0] = 0.0;
	knots_test_u[1] = 0.0;
	knots_test_u[2] = 0.0;
	knots_test_u[3] = 0.0;

	knots_test_u[4] = 0.25;
	knots_test_u[5] = 0.25;
	knots_test_u[6] = 0.5;
	knots_test_u[7] = 0.5;
	knots_test_u[8] = 0.75;
	knots_test_u[9] = 0.75;
	knots_test_u[10] = 1.0;
	knots_test_u[11] = 1.0;

	knots_test_u[12] = 1.0;
	knots_test_u[13] = 1.0;

	knots_test_v[0] =  0.0;
	knots_test_v[1] =  0.0;
	knots_test_v[2] =  0.0;
	knots_test_v[3] =  0.0;
	knots_test_v[4] =  0.25;
	knots_test_v[5] =  0.5;
	knots_test_v[6] =  0.75;
	knots_test_v[7] =  1.0;
	knots_test_v[8] =  1.0;
	knots_test_v[9] =  1.0;
	knots_test_v[10] =  1.0;
}

void draw_taca(void) {
	glPushMatrix();
		glColor3ub(0, 255, 0);
		 glScalef(0.75f, 1.2f, 1.0f);
		gluNurbsSurface(nurbs, nt_ctrl_u + 4, knots_taca_u, nt_ctrl_v + 4, knots_taca_v, nt_ctrl_v * 4, 4, &ctrlpoints_taca[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
	glPopMatrix();
}

void draw_test(void) {
	glPushMatrix();
		glColor3ub(255, 0, 0);
		gluNurbsSurface(nurbs, ns_ctrl_u + 4, knots_test_u, ns_ctrl_v + 4, knots_test_v, ns_ctrl_v * 4, 4, &ctrlpoints_test[0][0][0], 4, 4, GL_MAP2_VERTEX_3);
	glPopMatrix();
}


void display(void) {
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	gluLookAt(cam_x, cam_y, cam_z, center_x, center_y, center_z, 0.0, 1.0, 0.0);

//	glPushMatrix();
//		draw_taca();
//	glPopMatrix();

	glPushMatrix();
		draw_test();
	glPopMatrix();

	glutSwapBuffers();
}

void reshape(int w, int h) {
	// Avoid division by 0
	if (h == 0) h = 1;
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0 /* angle of view */, (double) w / h /* aspect ratio */ , 0.1, 150.0);

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
		case 'y':
			cam_y = -10.0;
			break;
		case 'Y':
			cam_y = DEF_CAM_Y;
			break;
		case '0':
			cam_x = DEF_CAM_X;
			cam_z = DEF_CAM_Z;
			break;
    case '1':
			cam_x = 30.0;
			cam_z = DEF_CAM_Z;
			break;
    case '2':
			cam_x = 50.0;
			cam_z = 0.0;
			break;
		case '3':
			cam_x = 30.0;
			cam_z = -DEF_CAM_Z;
			break;
		case '4':
			cam_x = DEF_CAM_X;
			cam_z = -DEF_CAM_Z;
			break;
    case '5':
			cam_x = -30.0;
			cam_z = -DEF_CAM_Z;
			break;
    case '6':
			cam_x = -50.0;
			cam_z = 0.0;
			break;
		case '7':
			cam_x = -30.0;
			cam_z = DEF_CAM_Z;
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
