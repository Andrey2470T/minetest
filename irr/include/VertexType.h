#pragma once

#include <vector>
#include <cassert>
#include <cstddef>
#include "S3DVertex.h"
#include "EVertexAttributes.h"
#include "mt_opengl.h"

namespace irr
{
namespace video
{
struct VertexAttribute
{
	enum class Mode
	{
		Regular,
		Normalized,
		Integral,
	};
	int Index;
	int ComponentCount;
	GLenum ComponentType;
	Mode mode;
	int Offset;
};

struct VertexType
{
	int VertexSize;
	std::vector<VertexAttribute> Attributes;
};

const VertexAttribute *begin(const VertexType &type)
{
	return type.Attributes.data();
}

const VertexAttribute *end(const VertexType &type)
{
	return type.Attributes.data() + type.Attributes.size();
}

const VertexType vtStandard = {
		sizeof(S3DVertex),
		{
				{EVA_POSITION, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex, Pos)},
				{EVA_NORMAL, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex, Normal)},
				{EVA_COLOR, 4, GLubyte, VertexAttribute::Mode::Normalized, offsetof(S3DVertex, Color)},
				{EVA_TCOORD0, 2, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex, TCoords)},
		},
};

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"

const VertexType vt2TCoords = {
		sizeof(S3DVertex2TCoords),
		{
				{EVA_POSITION, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex2TCoords, Pos)},
				{EVA_NORMAL, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex2TCoords, Normal)},
				{EVA_COLOR, 4, GLubyte, VertexAttribute::Mode::Normalized, offsetof(S3DVertex2TCoords, Color)},
				{EVA_TCOORD0, 2, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex2TCoords, TCoords)},
				{EVA_TCOORD1, 2, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex2TCoords, TCoords2)},
		},
};

const VertexType vtTangents = {
		sizeof(S3DVertexTangents),
		{
				{EVA_POSITION, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertexTangents, Pos)},
				{EVA_NORMAL, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertexTangents, Normal)},
				{EVA_COLOR, 4, GLubyte, VertexAttribute::Mode::Normalized, offsetof(S3DVertexTangents, Color)},
				{EVA_TCOORD0, 2, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertexTangents, TCoords)},
				{EVA_TANGENT, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertexTangents, Tangent)},
				{EVA_BINORMAL, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertexTangents, Binormal)},
		},
};

#pragma GCC diagnostic pop

const VertexType &getVertexTypeDescription(E_VERTEX_TYPE type)
{
	switch (type) {
	case EVT_STANDARD:
		return vtStandard;
	case EVT_2TCOORDS:
		return vt2TCoords;
	case EVT_TANGENTS:
		return vtTangents;
	default:
		assert(false);
	}
}

const VertexType vt2DImage = {
		sizeof(S3DVertex),
		{
				{EVA_POSITION, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex, Pos)},
				{EVA_COLOR, 4, GLubyte, VertexAttribute::Mode::Normalized, offsetof(S3DVertex, Color)},
				{EVA_TCOORD0, 2, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex, TCoords)},
		},
};

const VertexType vtPrimitive = {
		sizeof(S3DVertex),
		{
				{EVA_POSITION, 3, GLfloat, VertexAttribute::Mode::Regular, offsetof(S3DVertex, Pos)},
				{EVA_COLOR, 4, GLubyte, VertexAttribute::Mode::Normalized, offsetof(S3DVertex, Color)},
		},
};
}
}
