#include "stdafx.h"

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <stdlib.h> 

#include <vector>
#include <list>

#include "OpenGLFont.h"

OpenGLFont g_GLFont;


static float texture[] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

OpenGLFont::OpenGLFont()
{

	font_filename = DEFAULT_FONT;

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	// New 2D Font
    fonts2D[FTGL_OUTLINE] = new FTGLOutlineFont( font_filename );
    fonts2D[FTGL_POLYGON] = new FTGLPolygonFont( font_filename );
    fonts2D[FTGL_TEXTURE] = new FTGLTextureFont( font_filename );
    fonts2D[FTGL_BITMAP]  = new FTGLBitmapFont( font_filename );
    fonts2D[FTGL_PIXMAP]  = new FTGLPixmapFont( font_filename );
	fonts2D[FTGL_EXTRUDE] = new FTGLExtrdFont( font_filename );

   
	for (int i=0; i< 6; i++) {
		if( fonts2D[i]->Error())
		{
			MessageBox(NULL,_T("Failed to open font !"),_T("Error"),NULL);
		}
		
        int point_size = 64;
        if (!fonts2D[i]->FaceSize(point_size)) {
             MessageBox(NULL,_T("Failed to open font size!"),_T("Error"),NULL);
        }
    }

	currentFont2D = FTGL_POLYGON;


	// New 3D Font
    fonts3D[FTGL_OUTLINE] = new FTGLOutlineFont( font_filename );
    fonts3D[FTGL_POLYGON] = new FTGLPolygonFont( font_filename );
    fonts3D[FTGL_TEXTURE] = new FTGLTextureFont( font_filename );
    fonts3D[FTGL_BITMAP]  = new FTGLBitmapFont( font_filename );
    fonts3D[FTGL_PIXMAP]  = new FTGLPixmapFont( font_filename );
	fonts3D[FTGL_EXTRUDE] = new FTGLExtrdFont( font_filename );

   
	for (int i=0; i< 6; i++) {
		if( fonts3D[i]->Error())
		{
			MessageBox(NULL,_T("Failed to open font !"),_T("Error"),NULL);
		}
		
        int point_size = 1;
        if (!fonts3D[i]->FaceSize(point_size)) {
             MessageBox(NULL,_T("Failed to open font size!"),_T("Error"),NULL);
        }
		fonts3D[i]->Depth(0.2);
		fonts3D[i]->CharMap(ft_encoding_unicode);
    }

	currentFont3D = FTGL_EXTRUDE;

	glGenTextures(1, (GLuint*)&textureID);
    glBindTexture(GL_TEXTURE_2D, (GLuint)textureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 4, 4, 0, GL_RGB, GL_FLOAT, texture);

	SetUpLighting();
	
}

OpenGLFont::~OpenGLFont()
{
	SAFE_DELETE(fonts2D[FTGL_OUTLINE]);
	SAFE_DELETE(fonts2D[FTGL_POLYGON]);
	SAFE_DELETE(fonts2D[FTGL_TEXTURE]);
	SAFE_DELETE(fonts2D[FTGL_BITMAP]);
	SAFE_DELETE(fonts2D[FTGL_PIXMAP]);
	SAFE_DELETE(fonts2D[FTGL_EXTRUDE]);

	SAFE_DELETE(fonts3D[FTGL_OUTLINE]);
	SAFE_DELETE(fonts3D[FTGL_POLYGON]);
	SAFE_DELETE(fonts3D[FTGL_TEXTURE]);
	SAFE_DELETE(fonts3D[FTGL_BITMAP]);
	SAFE_DELETE(fonts3D[FTGL_PIXMAP]);
	SAFE_DELETE(fonts3D[FTGL_EXTRUDE]);
}

void OpenGLFont::DrawFont2D(char *string,float x, float y,float *color, float scale)
{
   /* Set up some strings with the characters to draw. */
   unsigned int count = 0;
   //int i;

   glColor3f(color[0], color[1], color[2]);

   if (currentFont2D >= FTGL_BITMAP) {
      glRasterPos2f(x, y);
      fonts2D[currentFont2D]->Render(string);
    }
    else {
      if (currentFont2D == FTGL_TEXTURE) {
         glEnable(GL_TEXTURE_2D);
         glEnable(GL_BLEND);
         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
       }

        glPushMatrix(); 
			glTranslatef(x, y, 0.0);
			glScalef(scale,scale,1.0);
            fonts2D[currentFont2D]->Render(string);
         glPopMatrix();

         if (currentFont2D == FTGL_TEXTURE) {
             glDisable(GL_TEXTURE_2D);
             glDisable(GL_BLEND);
      }
   }
}


void OpenGLFont::SetUpLighting()
{
   // Set up lighting.
	/*
   float light1_ambient[4]  = { 1.0, 1.0, 1.0, 1.0 };
   float light1_diffuse[4]  = { 1.0, 0.9, 0.9, 1.0 };
   float light1_specular[4] = { 1.0, 0.7, 0.7, 1.0 };
   float light1_position[4] = { -1.0, 1.0, 1.0, 0.0 };
   glLightfv(GL_LIGHT1, GL_AMBIENT,  light1_ambient);
   glLightfv(GL_LIGHT1, GL_DIFFUSE,  light1_diffuse);
   glLightfv(GL_LIGHT1, GL_SPECULAR, light1_specular);
   glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
   glEnable(GL_LIGHT1);

   float light2_ambient[4]  = { 0.2, 0.2, 0.2, 1.0 };
   float light2_diffuse[4]  = { 0.9, 0.9, 0.9, 1.0 };
   float light2_specular[4] = { 0.7, 0.7, 0.7, 1.0 };
   float light2_position[4] = { 1.0, -1.0, -1.0, 0.0 };
   glLightfv(GL_LIGHT2, GL_AMBIENT,  light2_ambient);
   glLightfv(GL_LIGHT2, GL_DIFFUSE,  light2_diffuse);
   glLightfv(GL_LIGHT2, GL_SPECULAR, light2_specular);
   glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
   glEnable(GL_LIGHT2);
*/
   float front_emission[4] = { 0.3, 0.2, 0.1, 0.0 };
   float front_ambient[4]  = { 0.2, 0.2, 0.2, 0.0 };
   float front_diffuse[4]  = { 0.95, 0.95, 0.8, 0.0 };
   float front_specular[4] = { 0.6, 0.6, 0.6, 0.0 };
   glMaterialfv(GL_FRONT, GL_EMISSION, front_emission);
   glMaterialfv(GL_FRONT, GL_AMBIENT, front_ambient);
   glMaterialfv(GL_FRONT, GL_DIFFUSE, front_diffuse);
   glMaterialfv(GL_FRONT, GL_SPECULAR, front_specular);
   glMaterialf(GL_FRONT, GL_SHININESS, 16.0);
   glColor4fv(front_diffuse);

   glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
   glColorMaterial(GL_FRONT, GL_DIFFUSE);
   glEnable(GL_COLOR_MATERIAL);


   //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//	glClearColor( 0.13, 0.17, 0.32, 0.0);
//	glColor3f( 1.0, 1.0, 1.0);
	
	glEnable( GL_CULL_FACE);
	glFrontFace( GL_CCW);
	
	glEnable( GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);

	glEnable( GL_POLYGON_OFFSET_LINE);
	glPolygonOffset( 1.0, 1.0); // ????

}


void OpenGLFont::DrawFont3D(char *string,float x, float y, float z, float *color, float scale)
{
  // glEnable(GL_LIGHTING);

   switch( currentFont3D)
	{
		case FTGL_BITMAP:
		case FTGL_PIXMAP:
		case FTGL_OUTLINE:
			break;
		case FTGL_POLYGON:
            glEnable( GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, textureID);
			glDisable( GL_BLEND);
			SetUpLighting();
			break;
		case FTGL_EXTRUDE:
			glEnable( GL_DEPTH_TEST);
			glDisable( GL_BLEND);
            glEnable( GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, textureID);
			SetUpLighting();
			break;
		case FTGL_TEXTURE:
			glEnable( GL_TEXTURE_2D);
			glDisable( GL_DEPTH_TEST);
			SetUpLighting();
			glNormal3f( 0.0, 0.0, 1.0);
			break;

	}


   glColor3f(color[0], color[1], color[2]);
   glPushMatrix();
        glScalef(scale,scale,scale);
	    glTranslatef(x, y, z);
		fonts3D[currentFont3D]->Render( string);
   glPopMatrix();
 
   //glDisable(GL_LIGHTING);
   glEnable(GL_BLEND);
   glDisable(GL_DEPTH_TEST);
}
