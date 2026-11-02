// Released under the MIT License; see LICENSE_UMT and the VRM4U LICENSE.
#pragma once

#include "CoreMinimal.h"

struct FReferenceSkeleton;
class UVrmPmxPhysicsData;

// One binding/frame state per model/proxy. A frame evaluates before, then after physics.
class VRM4U_API FVrmPmxBoneTransform
{
public:
    FVrmPmxBoneTransform();
    ~FVrmPmxBoneTransform();
    FVrmPmxBoneTransform(const FVrmPmxBoneTransform&) = delete;
    FVrmPmxBoneTransform& operator=(const FVrmPmxBoneTransform&) = delete;
    bool Initialize(const UVrmPmxPhysicsData& Data, const FReferenceSkeleton& Ref, FString& Error);
    bool EvaluateStage(bool bAfterPhysics, bool bPhysicsApplied,
        TArray<FTransform>& InOutComponentPose, FString& Error,
        const TMap<int32, FQuat>* ComponentRotationTargets = nullptr);

private:
    struct FRule
    {
        int32 MeshIndex = INDEX_NONE, SourceMeshIndex = INDEX_NONE, SourceIndex = INDEX_NONE;
        int32 Layer = 0, Flags = 0;
        float Weight = 0;
    };
    TArray<FRule> Bones;
    TArray<FTransform> ReferenceLocal, ReferenceComponent;
    TArray<int32> Parents;
    TArray<bool> PhysicsDriven, IsPmxBone, Seen;
    TArray<FQuat> RotationValues;
    TArray<FVector> TranslationValues;
    TArray<FTransform> WorldValues;
    bool bInitialized = false, bFrameStarted = false;
};
