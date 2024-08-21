#include "OpenGL/VertexBuffer.h"
#include "OpenGL/VertexType.h"
#include "mt_opengl.h"
#include "vendor/gl.h"

namespace irr
{
namespace scene
{

GLenum to_gl_usage(E_HARDWARE_MAPPING mapping)
{
	GLenum usage = GL_STATIC_DRAW;
	if (mapping == scene::EHM_STREAM)
		usage = GL_STREAM_DRAW;
	else if (mapping == scene::EHM_DYNAMIC)
		usage = GL_DYNAMIC_DRAW;

	return usage;
}

COpenGL3VertexBuffer::COpenGL3VertexBuffer()
	: IVertexBuffer()
{
	GL.GenVertexArrays(1, &VertexArrayId);
	GL.GenBuffers(1, &VertexBufferId);
}

COpenGL3VertexBuffer::~COpenGL3VertexBuffer()
{
	GL.DeleteVertexArrays(1, &VertexArrayId);
	GL.DeleteBuffers(1, &VertexBufferId);

	if (IndexBufferId != 0)
		GL.DeleteBuffers(1, &IndexBufferId);
}

void COpenGL3VertexBuffer::uploadData(u32 vertexCount, const void *vertexData,
	u32 indexCount, const void *indexData)
{
	if (vertexCount == 0 || vertexData == nullptr) {
		os::Printer::log("Failed to upload the data in the vertex buffer (no vertices provided)");
		return;
	}

	GL.BindVertexArray(VertexArrayId);

	VertexCount = vertexCount;

	GL.BindBuffer(GL_ARRAY_BUFFER, VertexBufferId);

    size_t bufferSize = VertexCount * video::getVertexPitchFromType(VertexType);

	GL.BufferData(GL_ARRAY_BUFFER, bufferSize, vertexData, to_gl_usage(MappingHint_Vertex));

	if (indexCount > 0 && indexData != nullptr) {
		if (IndexBufferId == 0)
			GL.GenBuffers(1, &IndexBufferId);

		IndexCount = indexCount;

		GL.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferId);

        bufferSize = IndexCount * video::getIndexPitchFromType(IndexType);

		GL.BufferData(GL_ELEMENT_ARRAY_BUFFER, bufferSize, indexData, to_gl_usage(MappingHint_Index));
	}

    setAttributes();

	GL.BindVertexArray(0);
}

void COpenGL3VertexBuffer::bind() const
{
	GL.BindVertexArray(VertexArrayId);
}

void COpenGL3VertexBuffer::unbind() const
{
	GL.BindVertexArray(0);
}

void COpenGL3VertexBuffer::setAttributes()
{
    auto &vertexAttribs = video::getVertexTypeDescription(VertexType);
    for (auto &attr : vertexAttribs.Attributes) {
		GL.EnableVertexAttribArray(attr.Index);
		switch (attr.mode) {
        case video::VertexAttribute::Mode::Regular:
            GL.VertexAttribPointer(attr.Index, attr.ComponentCount, (GLenum)attr.ComponentType, GL_FALSE, vertexAttribs.VertexSize, (void*)attr.Offset);
			break;
        case video::VertexAttribute::Mode::Normalized:
            GL.VertexAttribPointer(attr.Index, attr.ComponentCount, (GLenum)attr.ComponentType, GL_TRUE, vertexAttribs.VertexSize, (void*)attr.Offset);
			break;
        case video::VertexAttribute::Mode::Integral:
            GL.VertexAttribIPointer(attr.Index, attr.ComponentCount, (GLenum)attr.ComponentType, vertexAttribs.VertexSize, (void*)attr.Offset);
			break;
		}
	}
}

void COpenGL3VertexBuffer::draw(video::IVideoDriver *driver, const video::SMaterial &last_material,
	scene::E_PRIMITIVE_TYPE primitive_type) const
{
	GLenum indexSize = 0;

	switch (IndexType) {
    case (video::EIT_16BIT): {
		indexSize = GL_UNSIGNED_SHORT;
		break;
	}
    case (video::EIT_32BIT): {
		indexSize = GL_UNSIGNED_INT;
		break;
	}
	}

	u32 primitiveCount = getPrimitiveCount(primitive_type);

	switch (primitive_type) {
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
