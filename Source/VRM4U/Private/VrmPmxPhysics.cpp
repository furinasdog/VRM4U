// Released under the MIT License; see the VRM4U LICENSE.
#include "VrmPmxPhysics.h"
#include "VrmPmxPhysicsData.h"
#include "ReferenceSkeleton.h"

#ifndef WITH_VRM4U_PMX_BULLET
#define WITH_VRM4U_PMX_BULLET 0
#endif

#if WITH_VRM4U_PMX_BULLET
THIRD_PARTY_INCLUDES_START
#include "NativeContext.h"
#include "API.h"
THIRD_PARTY_INCLUDES_END
static_assert(sizeof(MMDBulletPhysics::RigidBodySimulationData) == 288, "UMT rigid-body ABI changed.");
static_assert(sizeof(MMDBulletPhysics::NativeJointData) == 212, "UMT joint ABI changed.");
static_assert(sizeof(MMDBulletPhysics::NativeConfig) == 36, "UMT configuration ABI changed.");
#endif

namespace
{
    // Keep the MMD fixed step; long frames are bounded/reset rather than fed into Bullet.
    constexpr float FixedStep = 1.f / 60.f;
    constexpr int32 MaximumSteps = 8;

    bool ValidTransform(const FTransform& T)
    {
        return !T.ContainsNaN() && T.GetRotation().IsNormalized();
    }

    bool UniformScale(const FTransform& T, float& Scale)
    {
        if (!ValidTransform(T)) return false;
        const FVector S = T.GetScale3D();
        Scale = float(S.X);
        return FMath::IsFinite(Scale) && Scale > SMALL_NUMBER && FMath::IsNearlyEqual(S.X, S.Y, 1.e-5)
            && FMath::IsNearlyEqual(S.X, S.Z, 1.e-5);
    }

    // The simulator uses Y-up metres; the mesh uses (PMX x, -z, y) centimetres.
    FVector SwapYZ(const FVector& V) { return FVector(V.X, V.Z, V.Y); }
    FQuat SwapYZ(const FQuat& Q) { return FQuat(-Q.X, -Q.Z, -Q.Y, Q.W); }

    FTransform PmxComponentTransform(const FVector& P, const FVector& R, float Scale)
    {
        const FQuat PhysicsRotation = (FQuat(FVector::YAxisVector, -R.Y)
            * FQuat(FVector::XAxisVector, -R.X) * FQuat(FVector::ZAxisVector, R.Z)).GetNormalized();
        return FTransform(SwapYZ(PhysicsRotation), FVector(P.X, -P.Z, P.Y) * (100.f * Scale));
    }

    bool WriteNative(const FTransform& UEWorld, float* Out)
    {
        const FTransform Physics(SwapYZ(UEWorld.GetRotation()), SwapYZ(UEWorld.GetLocation()) / 100.);
        const FMatrix M = Physics.ToMatrixNoScale();
        for (int32 Axis = 0; Axis < 4; ++Axis)
            for (int32 Component = 0; Component < 4; ++Component)
            {
                const float Value = float(M.M[Axis][Component]);
                if (!FMath::IsFinite(Value)) return false;
                Out[Axis * 4 + Component] = Value;
            }
        return true;
    }

    bool ReadNative(const float* In, FTransform& UEWorld)
    {
        FMatrix M;
        for (int32 Axis = 0; Axis < 4; ++Axis)
            for (int32 Component = 0; Component < 4; ++Component)
            {
                const float V = In[Axis * 4 + Component];
                if (!FMath::IsFinite(V)) return false;
                M.M[Axis][Component] = V;
            }
        const FTransform Physics(M);
        if (!ValidTransform(Physics)) return false;
        UEWorld = FTransform(SwapYZ(Physics.GetRotation()).GetNormalized(), SwapYZ(Physics.GetLocation()) * 100.);
        return ValidTransform(UEWorld);
    }

    bool ValidateData(const UVrmPmxPhysicsData& Data, FString& Error)
    {
        if (!FMath::IsFinite(Data.ModelScale) || Data.ModelScale <= SMALL_NUMBER
            || Data.RigidBodies.IsEmpty() || Data.RigidBodies.Num() > 16384 || Data.Joints.Num() > 32768)
        { Error = TEXT("Invalid PMX physics scale or object count."); return false; }
        for (const auto& B : Data.RigidBodies)
        {
            if (B.Group < 0 || B.Group > 15 || B.CollisionMask < 0 || B.CollisionMask > 65535
                || B.Shape < 0 || B.Shape > 2 || B.Mode < 0 || B.Mode > 2
                || B.Position.ContainsNaN() || B.Rotation.ContainsNaN() || B.Size.ContainsNaN()
                || B.Size.X <= 0 || (B.Shape == 1 && (B.Size.Y <= 0 || B.Size.Z <= 0)) || (B.Shape == 2 && B.Size.Y < 0)
                || !FMath::IsFinite(B.Mass) || B.Mass < 0 || !FMath::IsFinite(B.Friction) || B.Friction < 0
                || !FMath::IsFinite(B.LinearDamping) || !FMath::IsFinite(B.AngularDamping)
                || !FMath::IsFinite(B.Restitution) || B.Restitution < 0 || B.Restitution > 1)
            { Error = FString::Printf(TEXT("Invalid PMX rigid body: %s"), *B.Name); return false; }
        }
        for (const auto& J : Data.Joints)
        {
            if (J.Type < 0 || J.Type > 1)
            { Error = FString::Printf(TEXT("PMX joint type %d is not supported by this Bullet ABI: %s"), J.Type, *J.Name); return false; }
            if (!Data.RigidBodies.IsValidIndex(J.BodyA) || !Data.RigidBodies.IsValidIndex(J.BodyB) || J.BodyA == J.BodyB
                || J.Position.ContainsNaN() || J.Rotation.ContainsNaN() || J.TranslationMin.ContainsNaN() || J.TranslationMax.ContainsNaN()
                || J.RotationMin.ContainsNaN() || J.RotationMax.ContainsNaN() || J.SpringTranslation.ContainsNaN() || J.SpringRotation.ContainsNaN()
                || J.SpringTranslation.GetMin() < 0 || J.SpringRotation.GetMin() < 0)
            { Error = FString::Printf(TEXT("Invalid PMX joint: %s"), *J.Name); return false; }
        }
        return true;
    }
}

struct FVrmPmxPhysics::FImpl
{
    TArray<FVrmPmxRigidBodyData> Bodies;
    TArray<int32> Bones, Parents, AllIndices, KinematicIndices, DynamicIndices;
    TArray<FTransform> BodyOffsets, BodyRest, PreviousPose;
    FTransform PreviousComponentToWorld;
    float ComponentScale = 1;
    float Accumulator = 0;
#if WITH_VRM4U_PMX_BULLET
    MMDBulletPhysics::NativeContext* Context = nullptr;
    ~FImpl() { if (Context) MMDBulletPhysicsDestroy(Context); }

    FTransform AnimatedBody(int32 I, const TArray<FTransform>& Pose, const FTransform& ToWorld) const
    {
        return (Bones[I] == INDEX_NONE ? BodyRest[I] : BodyOffsets[I] * Pose[Bones[I]]) * ToWorld;
    }
    FTransform BoneFromBody(int32 I, const FTransform& BodyWorld, const FTransform& ToWorld) const
    {
        FTransform BodyComponent = BodyWorld.GetRelativeTransform(ToWorld);
        // Collider dimensions already contain the uniform world scale; native rigid poses have unit scale.
        BodyComponent.SetScale3D(FVector::OneVector);
        return BodyOffsets[I].Inverse() * BodyComponent;
    }
    bool ValidPose(const TArray<FTransform>& Pose, const FTransform& ToWorld) const
    {
        float Scale;
        if (Pose.Num() != Parents.Num() || !UniformScale(ToWorld, Scale) || !FMath::IsNearlyEqual(Scale, ComponentScale, 1.e-5f)) return false;
        for (const FTransform& T : Pose) if (!ValidTransform(T)) return false;
        return true;
    }
    bool ReadBodies(const TArray<int32>& Indices, TArray<FTransform>& Result) const
    {
        TArray<float> Matrices;
        Matrices.SetNumUninitialized(Indices.Num() * 16);
        // Position correction writes the current simulation body, so its source snapshot must use the same time.
        MMDBulletPhysicsGetRigidBodyWorldTransforms(Context, Indices.Num(), Indices.GetData(), Matrices.GetData());
        Result.SetNum(Indices.Num());
        for (int32 I = 0; I < Indices.Num(); ++I)
            if (!ReadNative(Matrices.GetData() + I * 16, Result[I])) return false;
        return true;
    }
#endif
};

FVrmPmxPhysics::FVrmPmxPhysics() = default;
FVrmPmxPhysics::~FVrmPmxPhysics() = default;

bool FVrmPmxPhysics::IsInitialized() const
{
#if WITH_VRM4U_PMX_BULLET
    return Impl && Impl->Context;
#else
    return false;
#endif
}

bool FVrmPmxPhysics::Initialize(const UVrmPmxPhysicsData& Data, const FReferenceSkeleton& Ref,
    const FTransform& ComponentToWorld, FString& Error)
{
    Impl.Reset(); Error.Reset();
#if !WITH_VRM4U_PMX_BULLET
    Error = TEXT("This build does not include the PMX Bullet runtime."); return false;
#else
    if (!ValidateData(Data, Error)) return false;
    TUniquePtr<FImpl> State = MakeUnique<FImpl>();
    if (!UniformScale(ComponentToWorld, State->ComponentScale) || Ref.GetNum() == 0)
    { Error = TEXT("PMX physics needs a valid skeleton and positive uniform component scale."); return false; }
    TArray<FTransform> RefPose;
    for (int32 I = 0; I < Ref.GetNum(); ++I)
    {
        const int32 Parent = Ref.GetParentIndex(I);
        if (Parent < INDEX_NONE || Parent >= I || !ValidTransform(Ref.GetRefBonePose()[I]))
        { Error = TEXT("Invalid PMX reference skeleton."); return false; }
        State->Parents.Add(Parent);
        RefPose.Add(Parent == INDEX_NONE ? Ref.GetRefBonePose()[I] : Ref.GetRefBonePose()[I] * RefPose[Parent]);
    }
    State->Bodies = Data.RigidBodies;
    const float LengthScale = Data.ModelScale * State->ComponentScale;
    if (!FMath::IsFinite(LengthScale) || LengthScale <= SMALL_NUMBER)
    { Error = TEXT("PMX physics unit scale is outside the supported range."); return false; }
    TArray<MMDBulletPhysics::RigidBodySimulationData> NativeBodies;
    for (int32 I = 0; I < Data.RigidBodies.Num(); ++I)
    {
        const auto& B = Data.RigidBodies[I];
        const int32 Bone = B.BoneName.IsNone() ? INDEX_NONE : Ref.FindBoneIndex(B.BoneName);
        if ((B.SourceBoneIndex >= 0 || !B.BoneName.IsNone()) && Bone == INDEX_NONE)
        { Error = FString::Printf(TEXT("PMX rigid body bone is missing: %s"), *B.Name); return false; }
        const FTransform Rest = PmxComponentTransform(B.Position, B.Rotation, Data.ModelScale);
        State->Bones.Add(Bone); State->BodyRest.Add(Rest);
        State->BodyOffsets.Add(Bone == INDEX_NONE ? Rest : Rest.GetRelativeTransform(RefPose[Bone]));
        State->AllIndices.Add(I);
        (B.Mode == 0 ? State->KinematicIndices : State->DynamicIndices).Add(I);
        auto& N = NativeBodies.AddDefaulted_GetRef();
        N.rigidBodyIndex = I; N.relatedBoneIndex = Bone; N.noRelatedBone = Bone == INDEX_NONE;
        N.groupIndex = uint8(B.Group);
        // PMX and Bullet both use set bits for allowed collision groups.
        N.collisionGroupMask = int16(uint16(B.CollisionMask));
        N.shape = MMDBulletPhysics::RigidBodyShape(B.Shape); N.mode = MMDBulletPhysics::RigidBodyMode(B.Mode);
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            N.size[Axis] = float(B.Size[Axis]) * LengthScale;
            if (!FMath::IsFinite(N.size[Axis])) { Error = TEXT("PMX collider dimensions overflow native precision."); return false; }
        }
        if (N.size[0] <= 0 || (B.Shape == 1 && (N.size[1] <= 0 || N.size[2] <= 0)))
        { Error = TEXT("PMX collider dimensions underflow native precision."); return false; }
        N.mass = B.Mass;
        // Bullet setDamping clamps finite inputs; preserve the authored raw values in Data.
        N.linearDamping = FMath::Clamp(B.LinearDamping, 0.f, 1.f);
        N.angularDamping = FMath::Clamp(B.AngularDamping, 0.f, 1.f);
        N.restitution = B.Restitution; N.friction = B.Friction;
        if (!WriteNative(Rest * ComponentToWorld, N.restWorldTransform))
        { Error = TEXT("PMX body transform overflows native precision."); return false; }
    }
    TArray<MMDBulletPhysics::NativeJointData> NativeJoints;
    for (const auto& J : Data.Joints)
    {
        auto& N = NativeJoints.AddDefaulted_GetRef();
        N.type = MMDBulletPhysics::JointType(J.Type); N.rigidBodyAIndex = J.BodyA; N.rigidBodyBIndex = J.BodyB;
        FTransform Joint = PmxComponentTransform(J.Position, J.Rotation, Data.ModelScale) * ComponentToWorld;
        FTransform A = State->BodyRest[J.BodyA] * ComponentToWorld;
        FTransform B = State->BodyRest[J.BodyB] * ComponentToWorld;
        Joint.SetScale3D(FVector::OneVector); A.SetScale3D(FVector::OneVector); B.SetScale3D(FVector::OneVector);
        // Converting a relative UE rigid frame uses the same reflected local basis.
        if (!WriteNative(Joint.GetRelativeTransform(A), N.frameInA) || !WriteNative(Joint.GetRelativeTransform(B), N.frameInB))
        { Error = TEXT("PMX joint frame overflows native precision."); return false; }
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            N.translationLimitMin[Axis] = float(Axis == 2 ? -J.TranslationMax[Axis] : J.TranslationMin[Axis]) * LengthScale;
            N.translationLimitMax[Axis] = float(Axis == 2 ? -J.TranslationMin[Axis] : J.TranslationMax[Axis]) * LengthScale;
            N.rotationLimitMin[Axis] = float(Axis == 2 ? J.RotationMin[Axis] : -J.RotationMax[Axis]);
            N.rotationLimitMax[Axis] = float(Axis == 2 ? J.RotationMax[Axis] : -J.RotationMin[Axis]);
            N.springTranslation[Axis] = float(J.SpringTranslation[Axis]);
            // Spring2 matches MMD Tools' authored stiffness; the old motor's scale^2 conversion does not apply.
            N.springRotation[Axis] = float(J.SpringRotation[Axis]);
            if (!FMath::IsFinite(N.translationLimitMin[Axis]) || !FMath::IsFinite(N.translationLimitMax[Axis])
                || !FMath::IsFinite(N.springRotation[Axis]))
            { Error = TEXT("PMX joint parameters overflow native precision."); return false; }
        }
    }
    const float Gravity[3] = {0, -9.8f, 0};
    State->Context = MMDBulletPhysicsCreate(Gravity, 80, 1, FixedStep);
    if (!State->Context) { Error = TEXT("Cannot create PMX physics context."); return false; }
    MMDBulletPhysics::NativeConfig Config{};
    Config.groundNormal[1] = 1;
    Config.mmdUnitToUnityUnit = LengthScale;
    Config.convexMargin = 1.e-6f;
    Config.springTranslationDamping = .5f;
    Config.springRotationDamping = .5f;
    Config.lockedTranslationReinforce = 1;
    MMDBulletPhysicsSetConfig(State->Context, &Config);
    MMDBulletPhysicsBuildRigidBodies(State->Context, NativeBodies.GetData(), NativeBodies.Num());
    MMDBulletPhysicsBuildJoints(State->Context, NativeJoints.GetData(), NativeJoints.Num());
    Impl = MoveTemp(State);
    if (!Reset(RefPose, ComponentToWorld)) { Impl.Reset(); Error = TEXT("Cannot initialize PMX body transforms."); return false; }
    return true;
#endif
}

bool FVrmPmxPhysics::Reset(const TArray<FTransform>& BoneComponentPose, const FTransform& ComponentToWorld)
{
#if WITH_VRM4U_PMX_BULLET
    if (!IsInitialized() || !Impl->ValidPose(BoneComponentPose, ComponentToWorld)) return false;
    TArray<float> Matrices;
    Matrices.SetNumUninitialized(Impl->Bodies.Num() * 16);
    for (int32 I = 0; I < Impl->Bodies.Num(); ++I)
        if (!WriteNative(Impl->AnimatedBody(I, BoneComponentPose, ComponentToWorld), Matrices.GetData() + I * 16)) return false;
    MMDBulletPhysicsSetRigidBodyTransforms(Impl->Context, Impl->AllIndices.Num(), Matrices.GetData(), Impl->AllIndices.GetData(), true);
    MMDBulletPhysicsReset(Impl->Context, 0);
    Impl->PreviousPose = BoneComponentPose; Impl->PreviousComponentToWorld = ComponentToWorld; Impl->Accumulator = 0;
    return true;
#else
    return false;
#endif
}

bool FVrmPmxPhysics::Simulate(float DeltaSeconds, const FTransform& ComponentToWorld,
    TArray<FTransform>& BoneComponentPose, FString& Error)
{
    Error.Reset();
#if !WITH_VRM4U_PMX_BULLET
    Error = TEXT("This build does not include the PMX Bullet runtime."); return false;
#else
    if (!IsInitialized() || !FMath::IsFinite(DeltaSeconds) || DeltaSeconds < 0 || !Impl->ValidPose(BoneComponentPose, ComponentToWorld))
    { Error = TEXT("Invalid PMX physics frame input."); return false; }
    if (DeltaSeconds > .25f) return Reset(BoneComponentPose, ComponentToWorld);
    const TArray<FTransform> Animated = BoneComponentPose;
    Impl->Accumulator += FMath::Min(DeltaSeconds, FixedStep * MaximumSteps);
    const int32 Steps = FMath::Min(FMath::FloorToInt(Impl->Accumulator / FixedStep), MaximumSteps);
    Impl->Accumulator -= Steps * FixedStep;
    TArray<float> KinematicMatrices;
    KinematicMatrices.SetNumUninitialized(Impl->KinematicIndices.Num() * 16);
    for (int32 Step = 0; Step < Steps; ++Step)
    {
        const float Alpha = float(Step + 1) / Steps;
        for (int32 K = 0; K < Impl->KinematicIndices.Num(); ++K)
        {
            const int32 I = Impl->KinematicIndices[K];
            FTransform Interpolated;
            Interpolated.Blend(Impl->AnimatedBody(I, Impl->PreviousPose, Impl->PreviousComponentToWorld),
                Impl->AnimatedBody(I, Animated, ComponentToWorld), Alpha);
            if (!WriteNative(Interpolated, KinematicMatrices.GetData() + K * 16))
            { Error = TEXT("PMX animation transform overflows native precision."); return false; }
        }
        MMDBulletPhysicsSetRigidBodyTransforms(Impl->Context, Impl->KinematicIndices.Num(), KinematicMatrices.GetData(), Impl->KinematicIndices.GetData(), false);
        MMDBulletPhysicsStepSimulation(Impl->Context, FixedStep);
        TArray<FTransform> Dynamic;
        if (!Impl->ReadBodies(Impl->DynamicIndices, Dynamic))
        { Reset(Animated, ComponentToWorld); Error = TEXT("Non-finite PMX body result; simulation reset."); return false; }
    }
    TArray<FTransform> Dynamic;
    if (!Impl->ReadBodies(Impl->DynamicIndices, Dynamic))
    { Reset(Animated, ComponentToWorld); Error = TEXT("Non-finite PMX body result; simulation reset."); return false; }
    TMap<int32, FTransform> Driven;
    TMap<int32, int32> DynamicSources;
    for (int32 K = 0; K < Impl->DynamicIndices.Num(); ++K)
    {
        const int32 I = Impl->DynamicIndices[K], Bone = Impl->Bones[I];
        if (Bone == INDEX_NONE) continue;
        FTransform Target = Impl->BoneFromBody(I, Dynamic[K], ComponentToWorld);
        Target.SetScale3D(Animated[Bone].GetScale3D());
        if (!ValidTransform(Target) || Target.GetLocation().SizeSquared() > 1.e14)
        { Reset(Animated, ComponentToWorld); Error = TEXT("Invalid PMX bone result; simulation reset."); return false; }
        Driven.Add(Bone, Target); DynamicSources.Add(Bone, K);
    }
    TArray<FTransform> Resolved;
    Resolved.SetNum(Animated.Num());
    for (int32 Bone = 0; Bone < Animated.Num(); ++Bone)
    {
        const int32 Parent = Impl->Parents[Bone];
        const FTransform Local = Parent == INDEX_NONE ? Animated[Bone] : Animated[Bone].GetRelativeTransform(Animated[Parent]);
        Resolved[Bone] = Parent == INDEX_NONE ? Local : Local * Resolved[Parent];
        if (const auto* Target = Driven.Find(Bone))
        {
            const int32 K = DynamicSources[Bone], I = Impl->DynamicIndices[K];
            if (Impl->Bodies[I].Mode == 2)
            {
                // Preserve the animated local position in the resolved physical parent frame.
                Resolved[Bone].SetRotation(Target->GetRotation());
                const FTransform DesiredBody = Impl->BodyOffsets[I] * Resolved[Bone] * ComponentToWorld;
                // Match permanent MMD Mode2 semantics: align position while retaining the solver velocity.
                const FVector Delta = SwapYZ(Dynamic[K].GetLocation() - DesiredBody.GetLocation()) / 100.;
                const float Shift[3] = {float(Delta.X), float(Delta.Y), float(Delta.Z)};
                if (!FMath::IsFinite(Shift[0]) || !FMath::IsFinite(Shift[1]) || !FMath::IsFinite(Shift[2]))
                { Reset(Animated, ComponentToWorld); Error = TEXT("PMX aligned-body position overflows native precision."); return false; }
                MMDBulletPhysicsShiftRigidBodyPosition(Impl->Context, I, Shift);
            }
            else Resolved[Bone] = *Target;
        }
        if (!ValidTransform(Resolved[Bone]))
        { Reset(Animated, ComponentToWorld); Error = TEXT("Invalid PMX descendant pose; simulation reset."); return false; }
    }
    BoneComponentPose = MoveTemp(Resolved);
    Impl->PreviousPose = Animated; Impl->PreviousComponentToWorld = ComponentToWorld;
    return true;
#endif
}

#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Animation/Skeleton.h"
#include <limits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxDampingTest, "VRM4U.PMX.DampingCompatibility",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxDampingTest::RunTest(const FString& Parameters)
{
#if WITH_VRM4U_PMX_BULLET
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier Modifier(Skeleton);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("pendant"), TEXT("pendant"), 0), FTransform(FVector(50, 0, 100)));
    }
    auto* Data = NewObject<UVrmPmxPhysicsData>(); Data->ModelScale = .1f;
    auto& Anchor = Data->RigidBodies.AddDefaulted_GetRef();
    Anchor.BoneName = TEXT("root"); Anchor.SourceBoneIndex = 0; Anchor.Mode = 0; Anchor.Shape = 0;
    Anchor.Size = FVector(.2, 0, 0); Anchor.Position = FVector(0, 10, 0); Anchor.CollisionMask = 0;
    auto& Pendant = Data->RigidBodies.AddDefaulted_GetRef();
    Pendant.BoneName = TEXT("pendant"); Pendant.SourceBoneIndex = 1; Pendant.Mode = 1; Pendant.Shape = 0;
    Pendant.Size = FVector(.2, 0, 0); Pendant.Position = FVector(5, 10, 0); Pendant.Mass = 1; Pendant.CollisionMask = 0;
    auto& Joint = Data->Joints.AddDefaulted_GetRef();
    Joint.BodyA = 0; Joint.BodyB = 1; Joint.Position = FVector(0, 10, 0);
    Joint.RotationMin = FVector(-PI); Joint.RotationMax = FVector(PI);
    const TArray<FTransform> Animated{FTransform::Identity, FTransform(FVector(50, 0, 100))};
    struct FCase { float Linear, Angular; };
    const FCase Cases[] = {{1, 2}, {2, 1}, {-.25f, -.75f}, {.3f, .8f}};
    FString Error;
    for (const auto& Case : Cases)
    {
        Data->RigidBodies[1].LinearDamping = Case.Linear; Data->RigidBodies[1].AngularDamping = Case.Angular;
        FVrmPmxPhysics Raw, Clamped;
        if (!TestTrue(TEXT("Finite authored damping outside [0,1] initializes like Bullet"), Raw.Initialize(*Data, Skeleton->GetReferenceSkeleton(), FTransform::Identity, Error))) return false;
        TestEqual(TEXT("Initialization preserves raw linear damping metadata"), Data->RigidBodies[1].LinearDamping, Case.Linear);
        TestEqual(TEXT("Initialization preserves raw angular damping metadata"), Data->RigidBodies[1].AngularDamping, Case.Angular);
        Data->RigidBodies[1].LinearDamping = FMath::Clamp(Case.Linear, 0.f, 1.f);
        Data->RigidBodies[1].AngularDamping = FMath::Clamp(Case.Angular, 0.f, 1.f);
        if (!TestTrue(TEXT("Explicitly clamped reference initializes"), Clamped.Initialize(*Data, Skeleton->GetReferenceSkeleton(), FTransform::Identity, Error))) return false;
        TArray<FTransform> RawPose, ClampedPose;
        for (int32 Frame = 0; Frame < 120; ++Frame)
        {
            RawPose = ClampedPose = Animated;
            if (!Raw.Simulate(1.f / 60.f, FTransform::Identity, RawPose, Error)
                || !Clamped.Simulate(1.f / 60.f, FTransform::Identity, ClampedPose, Error)) { AddError(Error); return false; }
        }
        TestTrue(TEXT("Authored damping reproduces Bullet's clamped pendulum motion"), RawPose[1].Equals(ClampedPose[1], .001));
    }
    const float Invalid[] = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()};
    FVrmPmxPhysics Rejected;
    for (const float Value : Invalid)
    {
        Data->RigidBodies[1].LinearDamping = Value; Data->RigidBodies[1].AngularDamping = .5f;
        TestFalse(TEXT("Non-finite linear damping is rejected before Bullet"), Rejected.Initialize(*Data, Skeleton->GetReferenceSkeleton(), FTransform::Identity, Error));
        Data->RigidBodies[1].LinearDamping = .5f; Data->RigidBodies[1].AngularDamping = Value;
        TestFalse(TEXT("Non-finite angular damping is rejected before Bullet"), Rejected.Initialize(*Data, Skeleton->GetReferenceSkeleton(), FTransform::Identity, Error));
    }
#else
    AddWarning(TEXT("Bullet is not linked in this build; damping assertions were skipped."));
#endif
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxCollisionGroupTest, "VRM4U.PMX.CollisionGroups",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxCollisionGroupTest::RunTest(const FString& Parameters)
{
#if WITH_VRM4U_PMX_BULLET
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier Modifier(Skeleton);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("sphere"), TEXT("sphere"), 0), FTransform(FVector(0, 0, 100)));
    }
    auto* Data = NewObject<UVrmPmxPhysicsData>(); Data->ModelScale = .1f;
    auto& Sphere = Data->RigidBodies.AddDefaulted_GetRef();
    Sphere.BoneName = TEXT("sphere"); Sphere.SourceBoneIndex = 1; Sphere.Group = 9;
    Sphere.Mode = 1; Sphere.Shape = 0; Sphere.Size = FVector(.5, 0, 0); Sphere.Position = FVector(0, 10, 0); Sphere.Mass = 1;
    auto& Floor = Data->RigidBodies.AddDefaulted_GetRef();
    Floor.BoneName = TEXT("root"); Floor.SourceBoneIndex = 0; Floor.Group = 3;
    Floor.Mode = 0; Floor.Shape = 1; Floor.Size = FVector(50, 1, 50);
    struct FCase { const TCHAR* Name; int32 SphereMask, FloorMask; bool bCollides; };
    const FCase Cases[] = {
        {TEXT("Groups9and3MutuallyAllowed"), (1 << 3) | (1 << 12), (1 << 9) | (1 << 5), true},
        {TEXT("SphereRejectsFloorButAllowsGroup12"), 1 << 12, (1 << 9) | (1 << 5), false},
        {TEXT("FloorRejectsSphereButAllowsGroup5"), (1 << 3) | (1 << 12), 1 << 5, false}
    };
    const TArray<FTransform> Animated{FTransform::Identity, FTransform(FVector(0, 0, 100))};
    FString Error; FVrmPmxPhysics Physics;
    for (const auto& Case : Cases)
    {
        Data->RigidBodies[0].CollisionMask = Case.SphereMask; Data->RigidBodies[1].CollisionMask = Case.FloorMask;
        if (!TestTrue(FString(Case.Name) + TEXT(" initializes raw asymmetric masks"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), FTransform::Identity, Error))) return false;
        TArray<FTransform> Pose;
        for (int32 Frame = 0; Frame < 180; ++Frame)
        {
            Pose = Animated;
            if (!Physics.Simulate(1.f / 60.f, FTransform::Identity, Pose, Error)) { AddError(Error); return false; }
        }
        if (Case.bCollides) TestTrue(FString(Case.Name) + TEXT(" rests on the authored floor"), FMath::IsNearlyEqual(Pose[1].GetLocation().Z, 15., .5));
        else TestTrue(FString(Case.Name) + TEXT(" falls through because both masks must permit the pair"), Pose[1].GetLocation().Z < -1000);
    }
#else
    AddWarning(TEXT("Bullet is not linked in this build; collision assertions were skipped."));
#endif
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxMode2ParentTest, "VRM4U.PMX.Mode2ParentChain",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxMode2ParentTest::RunTest(const FString& Parameters)
{
#if WITH_VRM4U_PMX_BULLET
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier Modifier(Skeleton);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("parent"), TEXT("parent"), 0), FTransform(FVector(0, 0, 100)));
        Modifier.Add(FMeshBoneInfo(TEXT("aligned"), TEXT("aligned"), 1), FTransform(FVector(20, 0, 10)));
        Modifier.Add(FMeshBoneInfo(TEXT("tip"), TEXT("tip"), 2), FTransform(FVector(0, 0, 10)));
    }
    auto* Data = NewObject<UVrmPmxPhysicsData>(); Data->ModelScale = .1f;
    auto& Parent = Data->RigidBodies.AddDefaulted_GetRef();
    Parent.BoneName = TEXT("parent"); Parent.SourceBoneIndex = 1; Parent.Mode = 1; Parent.Shape = 0;
    Parent.Size = FVector(.2, 0, 0); Parent.Position = FVector(0, 10, 0); Parent.Mass = 1; Parent.CollisionMask = 0;
    auto& Child = Data->RigidBodies.AddDefaulted_GetRef();
    Child.BoneName = TEXT("aligned"); Child.SourceBoneIndex = 2; Child.Mode = 2; Child.Shape = 0;
    Child.Size = FVector(.2, 0, 0); Child.Position = FVector(3, 11, 0); Child.Mass = 1; Child.CollisionMask = 0;
    const TArray<FTransform> Animated{FTransform::Identity, FTransform(FVector(0, 0, 100)),
        FTransform(FVector(20, 0, 110)), FTransform(FVector(20, 0, 120))};
    const FTransform ChildLocal = Animated[2].GetRelativeTransform(Animated[1]);
    const FTransform TipLocal = Animated[3].GetRelativeTransform(Animated[2]);
    const FTransform ParentTargets[] = {FTransform(FRotator(12, 35, -8), FVector(30, -15, 130)),
        FTransform(FRotator(-20, -30, 10), FVector(-25, 40, 170))};
    for (const int32 ParentMode : {1, 2})
        for (const float Scale : {1.f, 2.f})
        {
            Data->RigidBodies[0].Mode = ParentMode;
            const FTransform World(FRotator(0, -90, 0), FVector(100, 200, 300), FVector(Scale));
            FString Error; FVrmPmxPhysics Physics;
            if (!TestTrue(TEXT("A physical parent and offset mode2 child initialize"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), World, Error))) return false;
            for (const FTransform& ParentTarget : ParentTargets)
            {
                const FTransform ChildTarget(FRotator(-4, 15, 22), FVector(160, -80, 70));
                const FTransform Targets[] = {Physics.Impl->BodyOffsets[0] * ParentTarget * World,
                    Physics.Impl->BodyOffsets[1] * ChildTarget * World};
                float Native[32];
                if (!WriteNative(Targets[0], Native) || !WriteNative(Targets[1], Native + 16)) return false;
                // Seed actual dynamic transforms; the zero-time production readback isolates parent resolution from gravity/interpolation.
                MMDBulletPhysicsSetRigidBodyTransforms(Physics.Impl->Context, 2, Native, Physics.Impl->AllIndices.GetData(), true);
                TArray<FTransform> Pose = Animated;
                if (!TestTrue(TEXT("Zero-time mode2 parent resolution succeeds"), Physics.Simulate(0, World, Pose, Error))) return false;
                FTransform ExpectedParent = ParentTarget;
                if (ParentMode == 2) ExpectedParent.SetLocation(Animated[1].GetLocation());
                TestTrue(TEXT("The physical parent rotation and mode-specific position are retained"), Pose[1].Equals(ExpectedParent, .001));
                TestTrue(TEXT("Mode2 local position follows the current resolved physical parent"),
                    Pose[2].GetRelativeTransform(Pose[1]).GetLocation().Equals(ChildLocal.GetLocation(), .001));
                TestTrue(TEXT("Mode2 retains its independent dynamic rotation"), Pose[2].GetRotation().Equals(ChildTarget.GetRotation(), .001));
                TestTrue(TEXT("Unsimulated descendants follow the resolved mode2 child"), Pose[3].Equals(TipLocal * Pose[2], .001));
                TArray<FTransform> Bodies;
                if (!TestTrue(TEXT("Read back the real aligned native bodies"), Physics.Impl->ReadBodies(Physics.Impl->AllIndices, Bodies))) return false;
                for (int32 I = 0; I < 2; ++I)
                {
                    const FTransform NativeBone = Physics.Impl->BoneFromBody(I, Bodies[I], World);
                    TestTrue(TEXT("Native body position and output bone agree after parent-chain alignment"), NativeBone.GetLocation().Equals(Pose[I + 1].GetLocation(), .001));
                }
                for (int32 Frame = 0; Frame < 30; ++Frame)
                {
                    Pose = Animated;
                    if (!TestTrue(TEXT("A real fixed step retains mode2 parent-chain attachment"), Physics.Simulate(1.f / 60.f, World, Pose, Error))) return false;
                    TestTrue(TEXT("Stepped mode2 child retains its animated local position under the physical parent"),
                        Pose[2].GetRelativeTransform(Pose[1]).GetLocation().Equals(ChildLocal.GetLocation(), .001));
                    if (!Physics.Impl->ReadBodies(Physics.Impl->AllIndices, Bodies)) return false;
                    const FTransform SteppedChild = Physics.Impl->BoneFromBody(1, Bodies[1], World);
                    TestTrue(TEXT("Current-world mode2 correction has no interpolated-snapshot position feedback"),
                        SteppedChild.GetLocation().Equals(Pose[2].GetLocation(), .001));
                }
            }
        }
#else
    AddWarning(TEXT("Bullet is not linked in this build; mode2 parent assertions were skipped."));
#endif
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxBulletTest, "VRM4U.PMX.BulletRuntime",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxBulletTest::RunTest(const FString& Parameters)
{
    const FTransform World(FRotator(0, -90, 0), FVector(100, 200, 300));
    float Native[16]; FTransform RoundTrip;
    WriteNative(World, Native);
    TestTrue(TEXT("Physics-space matrices round trip rotated and translated components"), ReadNative(Native, RoundTrip) && World.Equals(RoundTrip, .001f));
    const FTransform Pmx = PmxComponentTransform(FVector(1, 2, 3), FVector::ZeroVector, .1f);
    TestTrue(TEXT("Imported PMX component basis includes the negative depth axis"), Pmx.GetLocation().Equals(FVector(10, -30, 20), .001f));
#if WITH_VRM4U_PMX_BULLET
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier Modifier(Skeleton);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("pendant"), TEXT("pendant"), 0), FTransform(FVector(0, 0, 100)));
        Modifier.Add(FMeshBoneInfo(TEXT("tip"), TEXT("tip"), 1), FTransform(FVector(0, 0, 10)));
    }
    auto* Data = NewObject<UVrmPmxPhysicsData>();
    Data->ModelScale = .1f;
    auto& Body = Data->RigidBodies.AddDefaulted_GetRef();
    Body.BoneName = TEXT("pendant"); Body.SourceBoneIndex = 1; Body.Mode = 1; Body.Shape = 0;
    Body.Size = FVector(.5, 0, 0); Body.Position = FVector(0, 10, 0); Body.Mass = 1;
    const TArray<FTransform> Animated{FTransform::Identity, FTransform(FVector(0, 0, 100)), FTransform(FVector(0, 0, 110))};
    TArray<FTransform> Pose = Animated;
    FString Error;
    FVrmPmxPhysics Physics;
    if (!TestTrue(TEXT("Build a real Bullet context"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), World, Error))) return false;
    for (int32 I = 0; I < 60; ++I)
    {
        Pose = Animated;
        TestTrue(TEXT("Finite Bullet step succeeds"), Physics.Simulate(1.f / 60.f, World, Pose, Error));
    }
    TestTrue(TEXT("Gravity lowers the dynamic bone in component Z"), Pose[1].GetLocation().Z < 0);
    TestTrue(TEXT("Unsimulated descendants inherit their simulated parent"), FMath::IsNearlyEqual(Pose[2].GetLocation().Z - Pose[1].GetLocation().Z, 10., .001));
    TestTrue(TEXT("The root is not overwritten by secondary physics"), Pose[0].Equals(Animated[0]));
    Pose = Animated;
    TestTrue(TEXT("Reset restores bodies to the current animation"), Physics.Reset(Pose, World));
    TestTrue(TEXT("Zero-time read after reset succeeds"), Physics.Simulate(0, World, Pose, Error));
    TestTrue(TEXT("Reset does not keep a fallen body pose"), Pose[1].GetLocation().Equals(FVector(0, 0, 100), .001f));
    TestFalse(TEXT("Non-finite delta never reaches the native solver"), Physics.Simulate(std::numeric_limits<float>::quiet_NaN(), World, Pose, Error));
    TestTrue(TEXT("Rejected frame leaves the animation intact"), Pose[1].Equals(Animated[1], .001f));
    Data->RigidBodies[0].Position.X = 1;
    FTransform ScaledWorld = World; ScaledWorld.SetScale3D(FVector(2));
    TestTrue(TEXT("Uniform actor scale initializes with an offset collider"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), ScaledWorld, Error));
    Pose = Animated;
    TestTrue(TEXT("Scaled body readback succeeds"), Physics.Simulate(0, ScaledWorld, Pose, Error));
    TestTrue(TEXT("Uniform actor scale does not move the offset bone"), Pose[1].Equals(Animated[1], .001f));
    Data->RigidBodies[0].Mode = 2;
    TestTrue(TEXT("Mode 2 initializes as dynamic bone-aligned"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), World, Error));
    for (int32 I = 0; I < 30; ++I)
    {
        Pose = Animated;
        TestTrue(TEXT("Mode 2 step succeeds"), Physics.Simulate(1.f / 60.f, World, Pose, Error));
        TestTrue(TEXT("Mode 2 retains the animated bone position"), Pose[1].GetLocation().Equals(Animated[1].GetLocation(), .001f));
    }
    Data->Joints.AddDefaulted_GetRef().Type = 5;
    TestFalse(TEXT("Unsupported PMX2.1 joint fails without calling native abort"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), World, Error));
    Data->Joints.Reset();
    Data->RigidBodies[0].Mode = 1; Data->RigidBodies[0].Group = 1;
    auto& Floor = Data->RigidBodies.AddDefaulted_GetRef();
    Floor.BoneName = TEXT("root"); Floor.SourceBoneIndex = 0; Floor.Shape = 1; Floor.Mode = 0;
    Floor.Size = FVector(50, 1, 50);
    TestTrue(TEXT("A PMX box and dynamic sphere initialize"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), World, Error));
    for (int32 I = 0; I < 180; ++I)
    {
        Pose = Animated;
        if (!Physics.Simulate(1.f / 60.f, World, Pose, Error)) { AddError(Error); return false; }
    }
    TestTrue(TEXT("PMX shapes collide at their authored scaled dimensions"), FMath::IsNearlyEqual(Pose[1].GetLocation().Z, 15., .5));
    Data->RigidBodies[0].CollisionMask = 1 << 1;
    TestTrue(TEXT("PMX raw mask initializes with floor group bit disabled"), Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), World, Error));
    for (int32 I = 0; I < 180; ++I)
    {
        Pose = Animated;
        if (!Physics.Simulate(1.f / 60.f, World, Pose, Error)) { AddError(Error); return false; }
    }
    TestTrue(TEXT("The excluded floor does not collide with the sphere"), Pose[1].GetLocation().Z < -1000);
#else
    AddWarning(TEXT("Bullet is not linked in this build; native simulation assertions were skipped."));
#endif
    return true;
}
#endif
