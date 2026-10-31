#pragma once

#include "CoreMinimal.h"
#include <assimp/material.h>

struct FPmxMaterialProperties
{
	FLinearColor Color = FLinearColor::White;
	FLinearColor EdgeColor = FLinearColor::Black;
	float EdgeSize = 0.f;
	int SphereMode = 0;
	bool bTwoSided = false;
	bool bEdgeEnabled = false;
	bool bToonEnabled = true;
};

inline FPmxMaterialProperties ReadPmxMaterialProperties(const aiMaterial& Material)
{
	FPmxMaterialProperties Result;
	aiColor3D Diffuse(1.f, 1.f, 1.f);
	float Opacity = 1.f;
	Material.Get(AI_MATKEY_COLOR_DIFFUSE, Diffuse);
	Material.Get(AI_MATKEY_OPACITY, Opacity);
	Result.Color = FLinearColor(Diffuse.r, Diffuse.g, Diffuse.b, FMath::Clamp(Opacity, 0.f, 1.f));
	Material.Get(AI_MATKEY_TWOSIDED, Result.bTwoSided);
	Material.Get("$mat.mmd.edgeEnabled", 0, 0, Result.bEdgeEnabled);
	Material.Get("$mat.mmd.toonEnabled", 0, 0, Result.bToonEnabled);
	Material.Get("$mat.mmd.edgeSize", 0, 0, Result.EdgeSize);
	aiColor4D EdgeColor(0.f, 0.f, 0.f, 1.f);
	Material.Get("$mat.mmd.edgeColor", 0, 0, EdgeColor);
	Result.EdgeColor = FLinearColor(EdgeColor.r, EdgeColor.g, EdgeColor.b, EdgeColor.a);
	Material.Get("$mat.mmd.sphereMode", 0, 0, Result.SphereMode);
	if (Result.SphereMode != 1 && Result.SphereMode != 2) Result.SphereMode = 0;
	return Result;
}
