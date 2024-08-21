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

    void uploadData(u32 vertexCount, const void *vertexData,
		u32 indexCount, const void *indexData) override;

	void bind() const override;

	void unbind() const override;

    void setAttributes() override;

	void draw(video::IVideoDriver *driver, const video::SMaterial &last_material,
        scene::E_PRIMITIVE_TYPE primitive_type = scene::EPT_TRIANGLES) const override;
};

} // end namespace scene
} // end namespace irr
