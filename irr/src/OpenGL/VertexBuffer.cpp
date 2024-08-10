#include "OpenGL/VertexBuffer.h"
#include "mt_opengl.h"
#include "vendor/gl.h"

namespace irr
{
namespace scene
{

COpenGL3VertexBuffer::COpenGL3VertexBuffer()
	: IVertexBuffer()
{
	GL.GenBuffers(1, &VertexBufferId);
	GL.GenBuffers(1, &IndexBufferId);

	PrimitiveType = scene::EPT_TRIANGLES;
}

COpenGL3VertexBuffer::~COpenGL3VertexBuffer()
{
	GL.DeleteBuffers(1, &VertexBufferId);
	GL.DeleteBuffers(1, &IndexBufferId);
}

void COpenGL3VertexBuffer::uploadVertexData(u32 vertexCount, const void *data)
{
	VertexCount = vertexCount;

	GL.BindBuffer(GL_ARRAY_BUFFER, VertexBufferId);

	size_t bufferSize = VertexCount * getVertexPitchFromType(VertexType);

	GLenum usage = GL_STATIC_DRAW;
	if (MappingHint_Vertex == scene::EHM_STREAM)
		usage = GL_STREAM_DRAW;
	else if (MappingHint_Vertex == scene::EHM_DYNAMIC)
		usage = GL_DYNAMIC_DRAW;

	GL.BufferData(GL_ARRAY_BUFFER, bufferSize, data, usage);

	GL.BindBuffer(GL_ARRAY_BUFFER, 0);
}

void COpenGL3VertexBuffer::uploadIndexData(u32 indexCount, const void *data)
{
	IndexCount = indexCount;

	GL.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferId);

	size_t bufferSize = IndexCount * getIndexPitchFromType(IndexType);

	GLenum usage = GL_STATIC_DRAW;
	if (MappingHint_Index == scene::EHM_STREAM)
		usage = GL_STREAM_DRAW;
	else if (MappingHint_Index == scene::EHM_DYNAMIC)
		usage = GL_DYNAMIC_DRAW;

	GL.BufferData(GL_ELEMENT_ARRAY_BUFFER, bufferSize, data, usage);

	GL.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void COpenGL3VertexBuffer::bind() const
{
	GL.BindBuffer(GL_ARRAY_BUFFER, VertexBufferId);
	GL.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferId);
}

void COpenGL3VertexBuffer::unbind() const
{
	GL.BindBuffer(GL_ARRAY_BUFFER, 0);
	GL.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void COpenGL3VertexBuffer::draw(video::IVideoDriver *driver, const video::SMaterial &last_material) const
{
	GLenum indexSize = 0;

	switch (IndexType) {
    case (video::EIT_16BIT): {
		indexSize = GL_UNSIGNED_SHORT;
		break;
	}
    case (video::EIT_32BIT): {
		indexSize = GL_UNSIGNED_INT;
//#ifdef GL_OES_element_index_uint
//#ifndef GL_UNSIGNED_INT
//#define GL_UNSIGNED_INT 0x1405
//#endif
		//if (driver->queryDriverFeature(COGLESCoreExtensionHandler::IRR_GL_OES_element_index_uint))
		//	indexSize = GL_UNSIGNED_INT;
		//else
//#endif
		//	indexSize = GL_UNSIGNED_SHORT;
		break;
	}
	}

	u32 primitiveCount = getPrimitiveCount();

	switch (PrimitiveType) {
	case scene::EPT_POINTS:
	case scene::EPT_POINT_SPRITES:
		GL.DrawArrays(GL_POINTS, 0, primitiveCount);
		break;
	case scene::EPT_LINE_STRIP:
		GL.DrawElements(GL_LINE_STRIP, primitiveCount + 1, indexSize, 0);
		break;
	case scene::EPT_LINE_LOOP:
		GL.DrawElements(GL_LINE_LOOP, primitiveCount, indexSize, 0);
		break;
	case scene::EPT_LINES:
		GL.DrawElements(GL_LINES, primitiveCount * 2, indexSize, 0);
		break;
	case scene::EPT_TRIANGLE_STRIP:
		GL.DrawElements(GL_TRIANGLE_STRIP, primitiveCount + 2, indexSize, 0);
		break;
	case scene::EPT_TRIANGLE_FAN:
		GL.DrawElements(GL_TRIANGLE_FAN, primitiveCount + 2, indexSize, 0);
		break;
	case scene::EPT_TRIANGLES:
		GL.DrawElements((last_material.Wireframe) ? GL_LINES : (last_material.PointCloud) ? GL_POINTS
																						: GL_TRIANGLES,
				primitiveCount * 3, indexSize, 0);
		break;
	default:
		break;
	}
}

} // end namespace scene
} // end namespace irr
