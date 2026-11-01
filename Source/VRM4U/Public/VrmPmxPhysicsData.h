// Released under the MIT License; see the VRM4U LICENSE.
#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "VrmPmxPhysicsData.generated.h"

// Values retain the source PMX basis, units, indices and raw collision mask.
USTRUCT()
struct VRM4U_API FVrmPmxRigidBodyData
{
    GENERATED_BODY()
    UPROPERTY() FString Name;
    UPROPERTY() FName BoneName;
    UPROPERTY() int32 SourceBoneIndex = INDEX_NONE;
    UPROPERTY() int32 Group = 0;
    // Raw PMX uint16: a set bit permits collision with that group; a cleared bit excludes it.
    UPROPERTY() int32 CollisionMask = 65535;
    UPROPERTY() int32 Shape = 0;
    UPROPERTY() int32 Mode = 0;
    UPROPERTY() FVector Size = FVector::ZeroVector;
    UPROPERTY() FVector Position = FVector::ZeroVector;
    UPROPERTY() FVector Rotation = FVector::ZeroVector;
    UPROPERTY() float Mass = 0;
    UPROPERTY() float LinearDamping = 0;
    UPROPERTY() float AngularDamping = 0;
    UPROPERTY() float Restitution = 0;
    UPROPERTY() float Friction = 0;
};

USTRUCT()
struct VRM4U_API FVrmPmxJointData
{
    GENERATED_BODY()
    UPROPERTY() FString Name;
    UPROPERTY() int32 Type = 0;
    UPROPERTY() int32 BodyA = INDEX_NONE;
    UPROPERTY() int32 BodyB = INDEX_NONE;
    UPROPERTY() FVector Position = FVector::ZeroVector;
    UPROPERTY() FVector Rotation = FVector::ZeroVector;
    UPROPERTY() FVector TranslationMin = FVector::ZeroVector;
    UPROPERTY() FVector TranslationMax = FVector::ZeroVector;
    UPROPERTY() FVector RotationMin = FVector::ZeroVector;
    UPROPERTY() FVector RotationMax = FVector::ZeroVector;
    UPROPERTY() FVector SpringTranslation = FVector::ZeroVector;
    UPROPERTY() FVector SpringRotation = FVector::ZeroVector;
};

UCLASS(BlueprintType)
class VRM4U_API UVrmPmxPhysicsData : public UObject
{
    GENERATED_BODY()
public:
    UPROPERTY() float ModelScale = 1;
    UPROPERTY() TArray<FVrmPmxRigidBodyData> RigidBodies;
    UPROPERTY() TArray<FVrmPmxJointData> Joints;
};
