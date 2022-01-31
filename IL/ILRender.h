/* $Id: ILRender.h,v 1.50 2005/10/17 10:12:09 ovidiom Exp $ */

/* forward declarations */
namespace ILines { class ILRender; }

#ifndef _ILRENDER_H_
#define _ILRENDER_H_

#if defined(WIN32) || defined(__CYGWIN__)
#include <windows.h>
#endif

#include <cstring>
#include <algorithm>

#include <GL/gl.h>

#include "glExtensions.h"
#include "ILTexture.h"
#include "ILLightingModel.h"
#include "ILUtilities.h"
#include "ShaderProgram.h"
#include "Vector.h"


namespace ILines
{
	extern const char *IL_cylinder_blinn_vp;
	extern const char *IL_cylinder_blinn_tangent_vp;
	extern const char *IL_cylinder_blinn_fp;


	/**
	 * @brief Provides functions for rendering illuminated lines.
	 *
	 * The ILRender class provides an interface for rendering
	 * illuminated lines.
	 */
	class ILRender
	{
	public:
		/** @brief Type definition for hiding the actual type of the identifier. */
		typedef void *				ILIdentifier;

		/** @brief Constant uniquely representing an invalid identifier. */
		static const ILIdentifier	IL_INVALID_IDENTIFIER;

		/** @brief Default constructor. Initializes some variables. */
		ILRender();

		/** @brief Destructor. Releases all acquired resources. */
		~ILRender();

		/** @brief Precomputes and prepares the textures for illuminating lines. */
		void setupTextures(float ka, float kd, float ks, float n, int texDim,
		                   ILLightingModel::Model lightingModel,
		                   bool stretch = false, const float *L = NULL);

		/** @brief Draws a set of line strips. */
		void multiDrawArrays(GLint *first,
		                     GLsizei *vertCount, GLsizei lineCount);

		/** @brief Draws a set of line strips. */
		void multiDrawArrays(ILIdentifier ilID);

		/** @brief Prepares a set of line strips for efficient rendering. */
		static ILIdentifier prepareMultiDrawArrays(GLint *first,
		                                           GLsizei *vertCount,
		                                           GLsizei lineCount);

		/** @brief Invalidates the identifier and release resources. */
		static void deleteIdentifier(ILIdentifier ilID);

		/** @brief Returns the ID of the ambient and diffuse lighting texture. */
		GLuint getTexIDDiff() const;

		/** @brief Returns the ID of the specular lighting texture. */
		GLuint getTexIDSpec() const;

		/** @brief Enables or disables z-sorting. */
		void enableZSort(bool doZSort);

		/** @brief Returns whether z-sorting is enabled. */
		bool isEnabledZSort() const;

		/** @brief Checks whether the passed lighting model is supported. */
		static bool isLightingModelSupported(ILLightingModel::Model model);

		/** @name Public error handling functions. */
		/*@{*/
		/** @brief Class containing possible errors generated. */
		enum ILError
		{
			/** @brief Value signaling that no error occurred. */
			IL_NO_ERROR,

			/** @brief Error generated if trying to render without having
			 *         set up the lighting textures. */
			IL_NOT_INITIALIZED,

			/** @brief Error generated if an OpenGL error occurred. */
			IL_GL_ERROR,

			/** @brief Error generated if an invalid lighting model is supplied. */
			IL_INVALID_LIGHTING_MODEL,

			/** @brief Error generated if the GL_ARB_multitexture extension
			 *         is needed but not supported. */
			IL_NO_ARB_MULTITEXTURE_EXTENSION,

			/** @brief Error generated if the GL_EXT_multi_draw_arrays extension
			 *         is needed but not supported. */
			IL_NO_EXT_MULTI_DRAW_ARRAYS_EXTENSION,

			/** @brief Error generated if the GL_ARB_vertex_program extension
			 *         is needed but not supported. */
			IL_NO_ARB_VERTEX_PROGRAM_EXTENSION,

			/** @brief Error generated if the GL_ARB_fragment_program extension
			 *         is needed but not supported. */
			IL_NO_ARB_FRAGMENT_PROGRAM_EXTENSION
		};

		/** @brief Function pointer type for error callback function. */
		typedef void(*ILErrorCallback)(ILRender *ilRender);

		/** @brief Returns the currently set error callback function. */
		ILErrorCallback getErrorCallback() const;

		/** @brief Sets the error callback function to be used. */
		void setErrorCallback(ILErrorCallback errorCallback);

		/** @brief Returns the error generated by the last operation. */
		ILError getError();

		/** @brief Returns the OpenGL error generated by the last operation. */
		GLenum getGLError();

		/** @brief Returns a description of the passed error. */
		static const char *errorString(ILError err);
		/*@}*/

	private:
		/** @brief Structure for internally representing a set of line strips. */
		struct ILInfo;

		/** @brief The lighting model being used. */
		ILLightingModel::Model	lightingModel;

		/** @brief The gloss exponent for the specular lighting. */
		GLfloat		gloss;

		/** @brief Texture ID for the ambient and diffuse illumination. */
		GLuint		texIDDiff;

		/** @brief Texture ID for the specular illumination. */
		GLuint		texIDSpec;

		/** @brief The used texture matrix. */
		GLfloat		textureMatrix[16];

		/** @brief Flag whether the lighting textures have been set up. */
		bool		isInitialized;

		/** @brief Flag whether to perform z-sorting. */
		bool		doZSort;

		/** @name OpenGL state variables. */
		/*@{*/
		GLenum		stateMatrixMode;
		GLenum		stateActiveTexture;
		GLenum		stateClientActiveTexture;
		GLdouble	stateTextureMatrixDiff[16];
		GLdouble	stateTextureMatrixSpec[16];
		GLfloat		stateSpotExponent;
		/*@}*/

		/** @name Flags signaling whether OpenGL extensions are supported. */
		/*@{*/
		/** @brief Flag whether the GL_ARB_multitexture extension is supported. */
		static bool	extMultitexture;

		/** @brief Flag whether the GL_EXT_multi_draw_arrays extension is supported. */
		static bool	extMultiDrawArrays;

		/** @brief Flag whether the GL_ARB_vertex_buffer_object extension is supported. */
		static bool	extVertexBufferObject;

		/** @brief Flag whether the GL_ARB_vertex_program extension is supported. */
		static bool	extVertexProgram;

		/** @brief Flag whether the GL_ARB_fragment_program extension is supported. */
		static bool	extFragmentProgram;
		/*@}*/

		/** @name Function pointers to OpenGL extension functions. */
		/*@{*/
		static PFNGLACTIVETEXTUREPROC			pglActiveTexture;
		static PFNGLCLIENTACTIVETEXTUREPROC		pglClientActiveTexture;
		static PFNGLMULTIDRAWARRAYSPROC			pglMultiDrawArrays;
		static PFNGLBINDBUFFERPROC				pglBindBuffer;
		static PFNGLGENBUFFERSPROC				pglGenBuffers;
		static PFNGLDELETEBUFFERSPROC			pglDeleteBuffers;
		static PFNGLMAPBUFFERPROC				pglMapBuffer;
		static PFNGLUNMAPBUFFERPROC				pglUnmapBuffer;
		static PFNGLGETBUFFERPARAMETERIVPROC	pglGetBufferParameteriv;
		static PFNGLGETBUFFERPOINTERVPROC		pglGetBufferPointerv;
		static PFNGLBUFFERDATAPROC				pglBufferData;
		static PFNGLPROGRAMSTRINGPROC			pglProgramString;
		static PFNGLBINDPROGRAMPROC				pglBindProgram;
		static PFNGLDELETEPROGRAMSPROC			pglDeletePrograms;
		static PFNGLGENPROGRAMSPROC				pglGenPrograms;
		/*@}*/

		/** @brief Vertex program for the Phong/Blinn lighting model.
		 *         See IL_cylinder_blinn_vp.cpp. */
		ShaderProgram	blinnVertProg;

		/** @brief Vertex program for the Phong/Blinn lighting model.
		 *         See IL_cylinder_blinn_tangent_vp.cpp. */
		ShaderProgram	blinnTangentVertProg;

		/** @brief Fragment program for the Phong/Blinn lighting model.
		 *         See IL_cylinder_blinn_fp.cpp. */
		ShaderProgram	blinnFragProg;

		/** @brief The error generated by the last operation. */
		ILError			lastError;

		/** @brief The OpenGL error generated by the last operation. */
		GLenum			lastGLError;

		/** @brief The currently used error function or NULL, if none is used. */
		ILErrorCallback	errorCallback;

		/** @brief Builds the needed texture matrix. */
		void buildTextureMatrix(Vector3f L, Vector3f V);

		/** @brief Sets up the needed texture coordinate arrays. */
		void setupTexCoordArrays(GLint size, GLenum type, GLsizei stride,
		                         const GLvoid *texCoords) const;

		/** @brief Enables the given texture mode and disables all other. */
		static void enableTexMode(GLenum textureMode);

		/** @brief Sets up the texture units for multitexturing. */
		void setupTexUnits(GLint size, GLenum type, GLsizei stride,
		                   const GLvoid *texCoordsDiff, const GLvoid *texCoordsSpec,
		                   GLuint texCoordsVBO) const;

		/** @brief Returns the stride of the vertices passed to OpenGL. */
		static GLsizei getVertexStride();

		/** @brief Saves the rendering state. */
		void saveRenderingState();

		/** @brief Restores the previously saved rendering state. */
		void restoreRenderingState();

		/** @brief Makes the OpenGL extensions available. */
		static void getExtensions();

		/** @brief Checks for the presence of an OpenGL extension. */
		static bool isExtensionSupported(const char *extension);

		/** @brief Initializes a given texture. */
		static void initTexture(GLuint *texID, int texDim, const float *texture);

		/** @brief Convertes the OpenGL vertices to a homogeneous format. */
		static float *getHomogeneous(int *first, int *vertCount, int lineCount,
		                             int arrSize);

		/** @brief Creates an array of homogeneous vertex coordinates. */
		template <typename T>
		static float *toHomogeneous(int *first, int *vertCount, int lineCount,
		                            int size, int stride,
		                            int arrSize, const T *vertices);

		/** @brief Computes the tangents for all line strips. */
		static float *computeTangents(int *first, int *vertCount, int lineCount,
		                              int arrSize, const float *homVert);

		/** @brief Does the actual rendering. */
		void render(int *first, int *vertCount, int lineCount,
		            const float *vertices,
		            const float *tangents, GLuint tangentsVBO = 0) const;

		/** @brief Does the actual rendering. */
		void renderLines(int *first, int *vertCount, int lineCount,
		                 const float *vertices) const;

		/** @brief Extracts the vertices supplied to OpenGL by the user. */
		static GLvoid *getVertices(GLuint &mappedVBO);

		/** @brief Releases previously mapped vertex buffer object data. */
		static void releaseVertices(GLuint mappedVBO);

		/** @brief Sets an error that occurred. */
		void setError(ILError err);

		/** @brief Checks whether the current lighting model is supported. */
		ILError catchLightingModelErrors();

		/** @brief Checks for occurred OpenGL errors. */
		void catchGLErrors();

		/** @brief Releases all acquired resources. */
		void releaseResources();
	};
}

#endif /* _ILRENDER_H_ */

