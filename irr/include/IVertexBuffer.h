// Copyright (C) 2008-2012 Nikolaus Gebhardt
// This file is part of the "Irrlicht Engine".
// For conditions of distribution and use, see copyright notice in irrlicht.h

#pragma once

#include "IReferenceCounted.h"
#include "S3DVertex.h"
#include "SVertexIndex.h"
#include "EHardwareBufferFlags.h"
#include "EPrimitiveTypes.h"
#include "os.h"
#include "SMaterial.h"
#include <cassert>

namespace irr
{

namespace video
{
	class IVideoDriver;
}

namespace scene
{

class IVertexBuffer
{
public:
	IVertexBuffer()
		: VertexBufferId(0), IndexBufferId(0)
	{}
	virtual ~IVertexBuffer() {}

    void formatBuffer(video::E_VERTEX_TYPE vertexType = video::EVT_STANDARD,
		scene::E_HARDWARE_MAPPING mappingHintVertex = scene::EHM_STATIC,
		video::E_INDEX_TYPE indexType = video::EIT_32BIT,
		scene::E_HARDWARE_MAPPING mappingHintIndex = scene::EHM_STATIC)
	{
		assert(mappingHintVertex != scene::EHM_NEVER);
		VertexType = vertexType;
		MappingHint_Vertex = mappingHintVertex;

		assert(mappingHintIndex != scene::EHM_NEVER);
		IndexType = indexType;
		MappingHint_Index = mappingHintIndex;
	}

	bool canBeDrawn() const
	{
		if (VertexCount == 0 || IndexCount == 0) {
			os::Printer::log("Can not render an empty vertex buffer (vertex count or index count is null)");
			return false;
		}

		return true;
	}

	virtual void uploadVertexData(u32 vertexCount, const void *data) = 0;

	virtual void uploadIndexData(u32 indexCount, const void *data) = 0;

	virtual void bind() const = 0;

	virtual void unbind() const = 0;

	virtual void draw(video::IVideoDriver *driver, const video::SMaterial &last_material) const  = 0;

	video::E_VERTEX_TYPE getVertexType() const { return VertexType; }

	bool operator==(const IVertexBuffer *other_buffer)
	{
		return (VertexBufferId == other_buffer->VertexBufferId && IndexBufferId == other_buffer->IndexBufferId);
	}

	u32 VertexBufferId;
	u32 IndexBufferId;

protected:
	u32 getPrimitiveCount() const {
		switch (PrimitiveType) {
		case scene::EPT_POINTS:
			return IndexCount;
		case scene::EPT_LINE_STRIP:
			return IndexCount - 1;
		case scene::EPT_LINE_LOOP:
			return IndexCount;
		case scene::EPT_LINES:
			return IndexCount / 2;
		case scene::EPT_TRIANGLE_STRIP:
			return (IndexCount - 2);
		case scene::EPT_TRIANGLE_FAN:
			return (IndexCount - 2);
		case scene::EPT_TRIANGLES:
			return IndexCount / 3;
		case scene::EPT_POINT_SPRITES:
			return IndexCount;
		}
		return 0;
	}

	video::E_VERTEX_TYPE VertexType;
	video::E_INDEX_TYPE IndexType;

	E_HARDWARE_MAPPING MappingHint_Vertex;
	E_HARDWARE_MAPPING MappingHint_Index;

	u32 VertexCount;
	u32 IndexCount;

	scene::E_PRIMITIVE_TYPE PrimitiveType;
};

} // end namespace scene
} // end namespace irr
