// Adapted from Unity MMD Tools MMDBoneTransform.BoneMath, MIT.
// Copyright (c) 2025-present Candidum Games. See LICENSE_UMT.
// https://github.com/CandidumGames/UnityMMDTools
#include "VrmPmxBoneTransform.h"
#include "VrmPmxPhysicsData.h"
#include "ReferenceSkeleton.h"

namespace
{
    constexpr int32 LocalAppend = 0x0080, AppendRotation = 0x0100, AppendTranslation = 0x0200;
    constexpr int32 AfterPhysics = 0x1000;

    bool ValidPmxBoneTransform(const FTransform& T)
    {
        const FVector Scale = T.GetScale3D();
        return !T.ContainsNaN() && T.GetRotation().IsNormalized()
            && FMath::Abs(Scale.X) > SMALL_NUMBER && FMath::Abs(Scale.Y) > SMALL_NUMBER && FMath::Abs(Scale.Z) > SMALL_NUMBER;
    }

    FQuat WeightedRotation(FQuat Rotation, float Weight)
    {
        Rotation.Normalize();
        // Explicit inversion keeps negative weights on the same shortest arc as UMT.
        if (Weight < 0) { Rotation = Rotation.Inverse(); Weight = -Weight; }
        return FQuat::Slerp(FQuat::Identity, Rotation, Weight).GetNormalized();
    }
}

FVrmPmxBoneTransform::FVrmPmxBoneTransform() = default;
FVrmPmxBoneTransform::~FVrmPmxBoneTransform() = default;

bool FVrmPmxBoneTransform::Initialize(const UVrmPmxPhysicsData& Data, const FReferenceSkeleton& Ref, FString& Error)
{
    bInitialized = false; bFrameStarted = false; Bones.Reset(); Parents.Reset(); ReferenceComponent.Reset(); PhysicsDriven.Reset(); IsPmxBone.Reset();
    ReferenceLocal = Ref.GetRefBonePose();
    if (ReferenceLocal.IsEmpty() || Data.BoneTransforms.Num() > 65535)
    { Error = TEXT("Invalid PMX append skeleton size."); return false; }
    PhysicsDriven.Init(false, Ref.GetNum()); IsPmxBone.Init(false, Ref.GetNum());
    for (int32 I = 0; I < Ref.GetNum(); ++I)
    {
        const int32 Parent = Ref.GetParentIndex(I);
        if (Parent < INDEX_NONE || Parent >= I || !ValidPmxBoneTransform(ReferenceLocal[I]))
        { Error = TEXT("Invalid PMX append reference hierarchy/transform."); return false; }
        Parents.Add(Parent);
        ReferenceComponent.Add(Parent == INDEX_NONE ? ReferenceLocal[I] : ReferenceLocal[I] * ReferenceComponent[Parent]);
    }
    const int32 Count = Data.BoneTransforms.Num();
    TArray<int32> MeshIndices; MeshIndices.Init(INDEX_NONE, Count);
    TSet<int32> Used;
    for (int32 I = 0; I < Count; ++I)
    {
        const auto& Bone = Data.BoneTransforms[I];
        const int32 Index = Ref.FindBoneIndex(Bone.BoneName);
        if (Index == INDEX_NONE || Used.Contains(Index) || Bone.Flags < 0 || Bone.Flags > 65535
            || Bone.SourceParentIndex < INDEX_NONE || Bone.SourceParentIndex >= Count || !FMath::IsFinite(Bone.AppendWeight))
        { Error = FString::Printf(TEXT("Invalid PMX append binding at source bone %d (%s)."), I, *Bone.BoneName.ToString()); return false; }
        MeshIndices[I] = Index; Used.Add(Index); IsPmxBone[Index] = true;
        if ((Bone.Flags & (AppendRotation | AppendTranslation)) && (Bone.AppendSourceIndex < INDEX_NONE || Bone.AppendSourceIndex >= Count))
        { Error = FString::Printf(TEXT("Invalid PMX append source at bone %d."), I); return false; }
    }
    for (int32 I = 0; I < Count; ++I)
    {
        const auto& Bone = Data.BoneTransforms[I];
        FRule& Rule = Bones.AddDefaulted_GetRef();
        Rule.MeshIndex = MeshIndices[I]; Rule.SourceMeshIndex = !(Bone.Flags & (AppendRotation | AppendTranslation)) || Bone.AppendSourceIndex == INDEX_NONE ? INDEX_NONE : MeshIndices[Bone.AppendSourceIndex];
        Rule.SourceIndex = I; Rule.Layer = Bone.DeformLayer; Rule.Flags = Bone.Flags; Rule.Weight = Bone.AppendWeight;
    }
    Bones.Sort([](const FRule& A, const FRule& B) { return A.Layer != B.Layer ? A.Layer < B.Layer : A.SourceIndex < B.SourceIndex; });
    for (const auto& Body : Data.RigidBodies)
        if (Body.Mode != 0)
        {
            const int32 Index = Ref.FindBoneIndex(Body.BoneName);
            if (Index != INDEX_NONE) PhysicsDriven[Index] = true;
        }
    bInitialized = true;
    return true;
}

bool FVrmPmxBoneTransform::EvaluateStage(bool bAfterPhysics, bool bPhysicsApplied,
    TArray<FTransform>& InOutComponentPose, FString& Error,
    const TMap<int32, FQuat>* ComponentRotationTargets)
{
    const auto Fail = [&](const TCHAR* Message) { Error = Message; bFrameStarted = false; return false; };
    if (!bInitialized || InOutComponentPose.Num() != Parents.Num())
        return Fail(TEXT("PMX append pose does not match its initialized skeleton."));
    if (bAfterPhysics && !bFrameStarted)
        return Fail(TEXT("PMX append after-physics evaluation requires a before-physics frame."));
    if (ComponentRotationTargets)
        for (const auto& Target : *ComponentRotationTargets)
            if (Target.Key < 0 || Target.Key >= Parents.Num() || Target.Value.ContainsNaN() || !Target.Value.IsNormalized())
                return Fail(TEXT("Invalid PMX semantic component-rotation target."));
    TArray<FTransform> Pose = InOutComponentPose, Local;
    Local.SetNum(Pose.Num());
    const auto ParentReference = [&](int32 Index) -> const FTransform& {
        return Parents[Index] == INDEX_NONE ? FTransform::Identity : ReferenceComponent[Parents[Index]];
    };
    const auto ReadDeltas = [&](int32 Index) {
        const FTransform& Basis = ParentReference(Index);
        const FQuat Delta = Local[Index].GetRotation() * ReferenceLocal[Index].GetRotation().Inverse();
        RotationValues[Index] = (Basis.GetRotation() * Delta * Basis.GetRotation().Inverse()).GetNormalized();
        TranslationValues[Index] = Basis.TransformVector(Local[Index].GetLocation() - ReferenceLocal[Index].GetLocation());
    };
    for (int32 I = 0; I < Pose.Num(); ++I)
    {
        if (!ValidPmxBoneTransform(Pose[I])) return Fail(TEXT("PMX append received a non-finite or singular pose."));
        Local[I] = Parents[I] == INDEX_NONE ? Pose[I] : Pose[I].GetRelativeTransform(Pose[Parents[I]]);
        Local[I].NormalizeRotation();
    }
    if (!bAfterPhysics)
    {
        bFrameStarted = true;
        RotationValues.Init(FQuat::Identity, Pose.Num()); TranslationValues.Init(FVector::ZeroVector, Pose.Num());
        Seen.Init(false, Pose.Num()); WorldValues.SetNum(Pose.Num());
        for (int32 I = 0; I < Pose.Num(); ++I)
        {
            // UMT resets pending world matrices to identity (model rotation/position).
            // Reference rotations only change the UE basis, not the neutral PMX rotation.
            WorldValues[I] = FTransform(ReferenceComponent[I].GetRotation(), FVector::ZeroVector);
            if (!IsPmxBone[I]) { Seen[I] = true; ReadDeltas(I); WorldValues[I] = Pose[I]; }
        }
    }
    else
    {
        for (int32 I = 0; I < Pose.Num(); ++I)
        {
            if (bPhysicsApplied && PhysicsDriven[I]) Seen[I] = true;
            if (Seen[I])
            {
                ReadDeltas(I);
                if (bPhysicsApplied) WorldValues[I] = Pose[I];
            }
        }
    }
    const auto Recompose = [&]() {
        for (int32 I = 0; I < Pose.Num(); ++I)
        {
            const bool bPhysical = bAfterPhysics && bPhysicsApplied && PhysicsDriven[I];
            const FQuat* Goal = ComponentRotationTargets ? ComponentRotationTargets->Find(I) : nullptr;
            if (bPhysical)
            {
                Pose[I] = InOutComponentPose[I];
                Local[I] = Parents[I] == INDEX_NONE ? Pose[I] : Pose[I].GetRelativeTransform(Pose[Parents[I]]);
                Local[I].NormalizeRotation();
            }
            else
            {
                if (Goal)
                {
                    const FQuat Parent = Parents[I] == INDEX_NONE ? FQuat::Identity : Pose[Parents[I]].GetRotation();
                    Local[I].SetRotation((Parent.Inverse() * *Goal).GetNormalized());
                }
                Pose[I] = Parents[I] == INDEX_NONE ? Local[I] : Local[I] * Pose[Parents[I]];
            }
            if (!ValidPmxBoneTransform(Pose[I])) return false;
            // Absolute retarget goals are resolved against the current animated parent.
            // Pending bones stay pending; they cannot leak into an earlier deform layer.
            if ((Goal || bPhysical) && Seen[I]) { ReadDeltas(I); WorldValues[I] = Pose[I]; }
        }
        return true;
    };
    if (!Recompose()) return Fail(TEXT("PMX append could not resolve the semantic rotation targets."));
    for (const FRule& Bone : Bones)
    {
        if (!!(Bone.Flags & AfterPhysics) != bAfterPhysics) continue;
        const int32 I = Bone.MeshIndex, Source = Bone.SourceMeshIndex;
        if (bAfterPhysics && bPhysicsApplied && PhysicsDriven[I]) continue;
        const FTransform& Basis = ParentReference(I);
        const bool bAppend = (Bone.Flags & (AppendRotation | AppendTranslation)) && Source != INDEX_NONE && Bone.Weight != 0;
        if (bAppend)
        {
            // Snapshot before any writes also makes self/cyclic references bounded:
            // each bone is evaluated once, in authored layer/index order, every frame.
            const FQuat SourceRotation = Bone.Flags & LocalAppend
                ? (WorldValues[Source].GetRotation() * ReferenceComponent[Source].GetRotation().Inverse()).GetNormalized()
                : RotationValues[Source];
            const FVector SourceTranslation = Bone.Flags & LocalAppend
                ? WorldValues[Source].GetLocation() - ReferenceComponent[Source].GetLocation() : TranslationValues[Source];
            if (Bone.Flags & AppendRotation)
            {
                const FQuat Weighted = WeightedRotation(SourceRotation, Bone.Weight);
                const FQuat LocalWeighted = Basis.GetRotation().Inverse() * Weighted * Basis.GetRotation();
                Local[I].SetRotation((LocalWeighted * Local[I].GetRotation()).GetNormalized());
            }
            if (Bone.Flags & AppendTranslation)
                Local[I].AddToTranslation(Basis.InverseTransformVector(SourceTranslation * Bone.Weight));
            if (!Recompose()) return Fail(TEXT("PMX append produced an invalid transform; input pose retained."));
        }
        ReadDeltas(I); Seen[I] = true;
        const FQuat* Goal = ComponentRotationTargets ? ComponentRotationTargets->Find(I) : nullptr;
        if (Goal) WorldValues[I] = Pose[I];
        else
        {
            const int32 Parent = Parents[I];
            const FTransform ParentWorld = Parent == INDEX_NONE ? FTransform::Identity : WorldValues[Parent];
            WorldValues[I] = Parent == INDEX_NONE ? Local[I] : Local[I] * ParentWorld;
        }
    }
    InOutComponentPose = MoveTemp(Pose);
    if (bAfterPhysics) bFrameStarted = false;
    return true;
}

#if WITH_DEV_AUTOMATION_TESTS
#include "Animation/Skeleton.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxAppendSpaceTest, "VRM4U.PMX.AppendSpacesAndStages",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxAppendSpaceTest::RunTest(const FString& Parameters)
{
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier M(Skeleton);
        M.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform(FRotator(11, 23, -9), FVector(5, 7, 3)));
        M.Add(FMeshBoneInfo(TEXT("sourceParent"), TEXT("sourceParent"), 0), FTransform(FRotator(7, -13, 5), FVector(3, 1, 4)));
        M.Add(FMeshBoneInfo(TEXT("source"), TEXT("source"), 1), FTransform(FRotator(-6, 17, 3), FVector(2, 5, 1)));
        M.Add(FMeshBoneInfo(TEXT("destParent"), TEXT("destParent"), 0), FTransform(FRotator(14, -8, -11), FVector(-2, 6, 4)));
        M.Add(FMeshBoneInfo(TEXT("dest"), TEXT("dest"), 3), FTransform(FRotator(8, 19, -5), FVector(1, 2, 3)));
        M.Add(FMeshBoneInfo(TEXT("child"), TEXT("child"), 4), FTransform(FVector(4, 3, 2)));
    }
    const auto& Ref = Skeleton->GetReferenceSkeleton();
    const auto Rest = Ref.GetRefBonePose();
    const auto Components = [&](const TArray<FTransform>& Local) {
        TArray<FTransform> C;
        for (int32 I = 0; I < Local.Num(); ++I) C.Add(Ref.GetParentIndex(I) == INDEX_NONE ? Local[I] : Local[I] * C[Ref.GetParentIndex(I)]);
        return C;
    };
    const auto Bind = Components(Rest);
    auto* Data = NewObject<UVrmPmxPhysicsData>();
    // Deliberately not mesh/topological order: original PMX indices must survive binding.
    const int32 Order[]{4, 2, 0, 3, 1, 5};
    const int32 SourceParents[]{3, 4, -1, 2, 2, 0};
    for (int32 I = 0; I < 6; ++I)
    {
        auto& B = Data->BoneTransforms.AddDefaulted_GetRef();
        B.BoneName = Ref.GetBoneName(Order[I]); B.SourceParentIndex = SourceParents[I];
    }
    Data->BoneTransforms[2].DeformLayer = -3;
    Data->BoneTransforms[3].DeformLayer = -2;
    Data->BoneTransforms[4].DeformLayer = -2;
    Data->BoneTransforms[1].DeformLayer = -1;
    Data->BoneTransforms[5].DeformLayer = 8;
    auto& Rule = Data->BoneTransforms[0]; Rule.Flags = 0x300; Rule.AppendSourceIndex = 1; Rule.DeformLayer = 7;
    const FQuat ParentMotion = FRotator(21, -16, 12).Quaternion(), SourceMotion = FRotator(-26, 31, 17).Quaternion();
    const FQuat OwnMotion = FRotator(19, -14, 8).Quaternion(), OtherMotion = FRotator(-11, 28, 16).Quaternion();
    const FVector SourceMove(2, -1, 3);
    const auto Animate = [&](TArray<FTransform>& L, int32 I, FQuat Q, FVector T = FVector::ZeroVector) {
        const int32 P = Ref.GetParentIndex(I);
        const FTransform Basis = P == INDEX_NONE ? FTransform::Identity : Bind[P];
        L[I].SetRotation((Basis.GetRotation().Inverse() * Q * Basis.GetRotation() * Rest[I].GetRotation()).GetNormalized());
        L[I].AddToTranslation(Basis.InverseTransformVector(T));
    };
    TArray<FTransform> Local = Rest;
    Animate(Local, 1, ParentMotion); Animate(Local, 2, SourceMotion, SourceMove);
    Animate(Local, 3, OtherMotion); Animate(Local, 4, OwnMotion);
    const TArray<FTransform> Animated = Components(Local);
    FString Error;
    for (const bool bLocal : {false, true}) for (const float Ratio : {-.6f, .5f, 1.4f})
    {
        Rule.Flags = 0x300 | (bLocal ? 0x80 : 0); Rule.AppendWeight = Ratio;
        FVrmPmxBoneTransform Solver;
        if (!TestTrue(TEXT("Reference rotations and reordered PMX indices bind"), Solver.Initialize(*Data, Ref, Error))) return false;
        auto Pose = Animated;
        TestTrue(TEXT("Before-physics append evaluates"), Solver.EvaluateStage(false, false, Pose, Error));
        const FQuat Source = bLocal ? (Animated[2].GetRotation() * Bind[2].GetRotation().Inverse()).GetNormalized() : SourceMotion;
        const FVector Move = bLocal ? Animated[2].GetLocation() - Bind[2].GetLocation() : SourceMove;
        auto ExpectedLocal = Local;
        const FQuat Weighted = Ratio < 0 ? FQuat::Slerp(FQuat::Identity, Source.Inverse(), -Ratio) : FQuat::Slerp(FQuat::Identity, Source, Ratio);
        ExpectedLocal[4].SetRotation((Bind[3].GetRotation().Inverse() * Weighted * Bind[3].GetRotation() * Local[4].GetRotation()).GetNormalized());
        ExpectedLocal[4].AddToTranslation(Bind[3].InverseTransformVector(Move * Ratio));
        const auto Expected = Components(ExpectedLocal);
        TestTrue(TEXT("Local/nonlocal rotation, negative weight and extrapolation match UMT in the model basis"), Pose[4].GetRotation().AngularDistance(Expected[4].GetRotation()) < .0001);
        TestTrue(TEXT("Local/nonlocal translation includes the correct source motion"), Pose[4].GetLocation().Equals(Expected[4].GetLocation(), .0001));
        TestTrue(TEXT("Non-driven descendants follow the appended pose"), Pose[5].Equals(Expected[5], .0001));
    }
    Rule.Flags = 0x1380; Rule.AppendWeight = -.6f;
    auto& Physical = Data->RigidBodies.AddDefaulted_GetRef(); Physical.BoneName = TEXT("source"); Physical.Mode = 1;
    FVrmPmxBoneTransform After;
    TestTrue(TEXT("After-physics binding succeeds"), After.Initialize(*Data, Ref, Error));
    auto Pose = Animated;
    After.EvaluateStage(false, false, Pose, Error);
    TestTrue(TEXT("After-physics append is not applied early"), Pose[4].Equals(Animated[4], .0001));
    Pose[2].SetRotation((FRotator(32, -27, 18).Quaternion() * Bind[2].GetRotation()).GetNormalized());
    Pose[2].AddToTranslation(FVector(3, 7, -2));
    const auto PhysicsSource = Pose[2];
    TestTrue(TEXT("After-physics reads actual current physical transforms"), After.EvaluateStage(true, true, Pose, Error));
    TestTrue(TEXT("Physics-owned source remains pinned in component space"), Pose[2].Equals(PhysicsSource, .0001));
    TestFalse(TEXT("The late target receives physical source motion"), Pose[4].Equals(Animated[4], .0001));
    const auto Input = Pose;
    Rule.AppendSourceIndex = 6;
    FVrmPmxBoneTransform Invalid;
    TestFalse(TEXT("Out-of-range append sources are rejected without evaluation"), Invalid.Initialize(*Data, Ref, Error));
    TestFalse(TEXT("An invalid binding does not write any output"), Invalid.EvaluateStage(false, false, Pose, Error));
    TestTrue(TEXT("Failed append retains the input pose"), Pose[4].Equals(Input[4]));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxAppendOrderTest, "VRM4U.PMX.AppendOrderAndPhysicsOwnership",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxAppendOrderTest::RunTest(const FString& Parameters)
{
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier M(Skeleton);
        M.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        for (const TCHAR* Name : {TEXT("source"), TEXT("middle"), TEXT("last")})
            M.Add(FMeshBoneInfo(Name, Name, 0), FTransform(FVector(0, 0, 10)));
        M.Add(FMeshBoneInfo(TEXT("tip"), TEXT("tip"), 3), FTransform(FVector(4, 0, 0)));
    }
    const auto& Ref = Skeleton->GetReferenceSkeleton();
    auto* Data = NewObject<UVrmPmxPhysicsData>();
    for (int32 I = 0; I < Ref.GetNum(); ++I)
    {
        auto& B = Data->BoneTransforms.AddDefaulted_GetRef();
        B.BoneName = Ref.GetBoneName(I); B.SourceParentIndex = Ref.GetParentIndex(I);
    }
    auto& Middle = Data->BoneTransforms[2]; Middle.Flags = 0x100; Middle.AppendSourceIndex = 1; Middle.AppendWeight = .5f; Middle.DeformLayer = -2;
    auto& Last = Data->BoneTransforms[3]; Last.Flags = 0x100; Last.AppendSourceIndex = 2; Last.AppendWeight = -.3f; Last.DeformLayer = 5;
    Data->BoneTransforms[0].DeformLayer = -4; Data->BoneTransforms[1].DeformLayer = -3; Data->BoneTransforms[4].DeformLayer = 6;
    const FQuat A = FRotator(27, -31, 18).Quaternion(), B = FRotator(-19, 12, 33).Quaternion(), C = FRotator(16, 24, -21).Quaternion();
    auto Input = Ref.GetRefBonePose(); Input[1].SetRotation(A); Input[2].SetRotation(B); Input[3].SetRotation(C);
    Input[4] = Input[4] * Input[3];
    FString Error;
    FVrmPmxBoneTransform Solver; TestTrue(TEXT("Chain binds"), Solver.Initialize(*Data, Ref, Error));
    auto Pose = Input; TestTrue(TEXT("Ordered chain evaluates"), Solver.EvaluateStage(false, false, Pose, Error));
    const FQuat ExpectedMiddle = (FQuat::Slerp(FQuat::Identity, A, .5f) * B).GetNormalized();
    const FQuat ExpectedLast = (FQuat::Slerp(FQuat::Identity, ExpectedMiddle.Inverse(), .3f) * C).GetNormalized();
    TestTrue(TEXT("Append source carries its own animated rotation plus prior grants"), Pose[2].GetRotation().AngularDistance(ExpectedMiddle) < .0001);
    TestTrue(TEXT("Noncommuting rotations and negative partial weights preserve multiplication order"), Pose[3].GetRotation().AngularDistance(ExpectedLast) < .0001);
    Last.DeformLayer = -3;
    FVrmPmxBoneTransform Reordered; Reordered.Initialize(*Data, Ref, Error);
    auto Early = Input; Reordered.EvaluateStage(false, false, Early, Error);
    TestTrue(TEXT("A future source has reset identity accumulation until its authored layer is reached"), Early[3].GetRotation().AngularDistance(C) < .0001);
    Last.Flags |= 0x1000;
    auto& Body = Data->RigidBodies.AddDefaulted_GetRef(); Body.BoneName = TEXT("middle"); Body.Mode = 2;
    FVrmPmxBoneTransform Physics; Physics.Initialize(*Data, Ref, Error);
    auto Physical = Input; Physics.EvaluateStage(false, false, Physical, Error);
    const FQuat ActualPhysics = FRotator(-34, 41, 26).Quaternion(); Physical[2].SetRotation(ActualPhysics);
    Physics.EvaluateStage(true, true, Physical, Error);
    const FQuat ExpectedPhysical = (FQuat::Slerp(FQuat::Identity, ActualPhysics.Inverse(), .3f) * C).GetNormalized();
    TestTrue(TEXT("After-physics append uses the physics result rather than stale animation"), Physical[3].GetRotation().AngularDistance(ExpectedPhysical) < .0001);
    TestTrue(TEXT("After-physics preserves the physical source"), Physical[2].GetRotation().AngularDistance(ActualPhysics) < .0001);
    Body.BoneName = TEXT("last");
    FVrmPmxBoneTransform Pinned; Pinned.Initialize(*Data, Ref, Error);
    auto Owned = Input; Pinned.EvaluateStage(false, false, Owned, Error); Owned[3].SetRotation(ActualPhysics); Pinned.EvaluateStage(true, true, Owned, Error);
    TestTrue(TEXT("An after-physics rule cannot overwrite its physics-owned target"), Owned[3].GetRotation().AngularDistance(ActualPhysics) < .0001);
    Last.Flags = 0x100; Last.AppendSourceIndex = 3; Last.AppendWeight = 0;
    FVrmPmxBoneTransform Zero; TestTrue(TEXT("A disabled zero-weight self-reference is harmless"), Zero.Initialize(*Data, Ref, Error));
    Last.AppendWeight = -.3f;
    FVrmPmxBoneTransform Self; TestTrue(TEXT("Self references use bounded ordered snapshots"), Self.Initialize(*Data, Ref, Error));
    auto SelfPose = Input; Self.EvaluateStage(false, false, SelfPose, Error);
    TestTrue(TEXT("A self source reads reset accumulation rather than recursively feeding back"), SelfPose[3].GetRotation().AngularDistance(C) < .0001);
    auto SecondFrame = Input; Self.EvaluateStage(false, false, SecondFrame, Error);
    TestTrue(TEXT("Snapshot cycles cannot accumulate across frames"), SecondFrame[3].GetRotation().AngularDistance(C) < .0001);
    return true;
}
#endif
