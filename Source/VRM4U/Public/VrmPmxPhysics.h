// Released under the MIT License; see the VRM4U LICENSE.
#pragma once

#include "CoreMinimal.h"

struct FReferenceSkeleton;
class UVrmPmxPhysicsData;

// One instance per model; all calls must belong to the same pose-evaluation owner.
class VRM4U_API FVrmPmxPhysics
{
public:
    FVrmPmxPhysics();
    ~FVrmPmxPhysics();
    FVrmPmxPhysics(const FVrmPmxPhysics&) = delete;
    FVrmPmxPhysics& operator=(const FVrmPmxPhysics&) = delete;

    bool Initialize(const UVrmPmxPhysicsData& Data, const FReferenceSkeleton& Ref,
        const FTransform& ComponentToWorld, FString& Error);
    bool Reset(const TArray<FTransform>& BoneComponentPose, const FTransform& ComponentToWorld);
    // Full mesh-bone-indexed component pose in and out, before converting to compact/local pose.
    // Dynamic display poses interpolate one fixed step behind; constraint feedback uses current native states.
    bool Simulate(float DeltaSeconds, const FTransform& ComponentToWorld,
        TArray<FTransform>& BoneComponentPose, FString& Error);
    bool IsInitialized() const;

private:
#if WITH_DEV_AUTOMATION_TESTS
    friend class FVrmPmxMode2ParentTest;
    friend class FVrmPmxSubstepTimingTest;
    friend class FVrmPmxRenderInterpolationTest;
#endif
    struct FImpl;
    TUniquePtr<FImpl> Impl;
};
