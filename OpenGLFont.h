#ifndef OPENGLFONT_H
#define OPENGLFONT_H

#define FTGL_OUTLINE 0
#define FTGL_POLYGON 1
#define FTGL_TEXTURE 2
#define FTGL_BITMAP 3
#define FTGL_PIXMAP 4
#define FTGL_EXTRUDE 5


#include "FTGLOutlineFont.h"
#include "FTGLPolygonFont.h"
#include "FTGLBitmapFont.h"
#include "FTGLTextureFont.h"
#include "FTGLPixmapFont.h"
#include "FTGLExtrdFont.h"

#define DEFAULT_FONT "C:\\WINDOWS\\Fonts\\arial.ttf"
using namespace std;

class OpenGLFont {
public:

	OpenGLFont();
	~OpenGLFont();

	char *font_filename;
	FTFont* fonts2D[6];
	int    currentFont2D;

	FTFont* fonts3D[6];
	int    currentFont3D;

	unsigned int    textureID;
	int width;
	int height;

	void DrawFont2D(char *string,float x, float y,float *color,float scale);

	void SetUpLighting();
	void DrawFont3D(char *string,float x, float y, float z, float *color,float scale);
	void SetCurrentFont2D(int font) { currentFont2D = font; };

private:


};

#endif