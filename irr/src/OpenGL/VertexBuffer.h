#pragma once

#include "IVertexBuffer.h"

namespace irr
{
namespace scene
{

class COpenGL3VertexBuffer : public IVertexBuffer
{
public:
    COpenGL3VertexBuffer();

	~COpenGL3VertexBuffer();

	void uploadVertexData(u32 vertexCount, const void *data) override;

	void uploadIndexData(u32 indexCount, const void *data) override;

	void bind() const override;

	void unbind() const override;

	void draw(video::IVideoDriver *driver, const video::SMaterial &last_material) const override;
};

} // end namespace scene
} // end namespace irr
