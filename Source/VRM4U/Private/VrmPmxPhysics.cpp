// Released under the MIT License; see the VRM4U LICENSE.
#include "VrmPmxPhysics.h"
#include "VrmPmxPhysicsData.h"
#include "ReferenceSkeleton.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Serialization/Archive.h"
#include "Serialization/MemoryWriter.h"
#endif

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
    // Leave headroom for rendering instead of accumulating expensive catch-up steps.
    constexpr float FixedStep = 1.f / 30.f;
    constexpr int32 SolverIterations = 64;
    // Drop excess elapsed time instead of turning one missed deadline into several costly steps.
    constexpr int32 MaximumSteps = 1;

#if WITH_DEV_AUTOMATION_TESTS
    bool QaStepSettings(const TCHAR* Command, float& Step, int32& Iterations)
    {
        float Hz = 1.f / Step;
        FParse::Value(Command, TEXT("PmxQaHz="), Hz);
        FParse::Value(Command, TEXT("PmxQaIterations="), Iterations);
        if (!FMath::IsFinite(Hz) || Hz < 15.f || Hz > 120.f || Iterations < 8 || Iterations > 160) return false;
        Step = 1.f / Hz;
        return true;
    }
#endif

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
    // Display snapshots never participate in Mode2 alignment or solver feedback.
    TArray<FTransform> RenderPreviousBodies, RenderCurrentBodies;
    TArray<FTransform> RenderPreviousAnimation, RenderCurrentAnimation;
    bool bRenderHistory = false;
    FTransform PreviousComponentToWorld;
    float ComponentScale = 1;
    float Accumulator = 0;
    float StepSeconds = FixedStep;
    int32 Iterations = SolverIterations;
#if WITH_VRM4U_PMX_BULLET
    MMDBulletPhysics::NativeContext* Context = nullptr;
    ~FImpl()
    {
#if WITH_DEV_AUTOMATION_TESTS
        QaEventsFinish(0);
#endif
        if (Context) MMDBulletPhysicsDestroy(Context);
    }

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
#if WITH_DEV_AUTOMATION_TESTS
    bool bQaTrace = false;
    uint32 QaFrame = 0;
    TArray<FName> QaBoneNames;
    TArray<FTransform> QaAnimated, QaOutput;
    TArray<float> QaTargets;
    FString QaDumpPath;
    double QaElapsed = 0;
    TArray<MMDBulletPhysics::RigidBodySimulationData> QaNativeBodies;
    TArray<MMDBulletPhysics::NativeJointData> QaNativeJoints;
    MMDBulletPhysics::NativeConfig QaConfig{};
    TArray<FTransform> QaReference;
    FTransform QaInitialToWorld;
    // Explicit development-only capture of calls crossing the native ABI, not a new simulation path.
    FString QaEventsPath;
    TUniquePtr<FArchive> QaEventsWriter;
    TArray<uint8> QaEventsBuffer;
    uint64 QaEventsBytes = 0;
    uint32 QaEventsCount = 0, QaEventsFrames = 0, QaEventsEnded = 0;
    double QaEventsSeconds = 0;
    static constexpr uint64 QaEventsMaxBytes = 128ull * 1024 * 1024;
    static constexpr double QaEventsMaxSeconds = 90.;
    static void QaEventsAppend(TArray<uint8>& Bytes, const void* Data, SIZE_T Size)
    {
        if (Size) Bytes.Append(static_cast<const uint8*>(Data), int32(Size));
    }
    void QaEventsFinish(uint32 Status)
    {
        if (!QaEventsWriter) return;
        if (Status == 0 && QaEventsFrames != QaEventsEnded) Status = 4;
        const uint32 Header[] = {7, 20};
        const uint32 Counts[] = {Status, QaEventsFrames, QaEventsEnded, QaEventsCount};
        const float Seconds = float(QaEventsSeconds);
        QaEventsWriter->Serialize(const_cast<uint32*>(Header), sizeof(Header));
        QaEventsWriter->Serialize(const_cast<uint32*>(Counts), sizeof(Counts));
        QaEventsWriter->Serialize(const_cast<float*>(&Seconds), sizeof(Seconds));
        QaEventsBytes += sizeof(Header) + sizeof(Counts) + sizeof(Seconds);
        const bool bMemoryValid = QaEventsWriter->Close() && !QaEventsWriter->IsError();
        QaEventsWriter.Reset();
        const bool bSaved = bMemoryValid && FFileHelper::SaveArrayToFile(QaEventsBuffer, *QaEventsPath);
        QaEventsBuffer.Empty();
        if (bSaved && Status == 0)
        {
            UE_LOG(LogTemp, Display, TEXT("PMX_QA_EVENTS saved=1 status=%u frames=%u ended=%u events=%u bytes=%llu elapsed=%.6f path=\"%s\""),
                Status, QaEventsFrames, QaEventsEnded, QaEventsCount, QaEventsBytes, QaEventsSeconds, *QaEventsPath);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("PMX_QA_EVENTS saved=%d status=%u frames=%u ended=%u events=%u bytes=%llu elapsed=%.6f path=\"%s\""),
                bSaved, Status, QaEventsFrames, QaEventsEnded, QaEventsCount, QaEventsBytes, QaEventsSeconds, *QaEventsPath);
        }
    }
    bool QaEventsRecord(uint32 Type, const void* Data, uint32 Size)
    {
        if (!QaEventsWriter) return false;
        // Reserve the footer so a bounded capture is distinguishable from a truncated file.
        if (QaEventsBytes + 8ull + Size + 28ull > QaEventsMaxBytes)
        { QaEventsFinish(1); return false; }
        uint32 Header[] = {Type, Size};
        QaEventsWriter->Serialize(Header, sizeof(Header));
        if (Size) QaEventsWriter->Serialize(const_cast<void*>(Data), Size);
        QaEventsBytes += sizeof(Header) + Size;
        ++QaEventsCount;
        if (QaEventsWriter->IsError()) { QaEventsFinish(3); return false; }
        return true;
    }
    void QaEventsStart(const TArray<MMDBulletPhysics::RigidBodySimulationData>& NativeBodies,
        const TArray<MMDBulletPhysics::NativeJointData>& NativeJoints, const MMDBulletPhysics::NativeConfig& Config,
        const TArray<FTransform>& Reference, const FTransform& ToWorld)
    {
        if (QaEventsPath.IsEmpty()) return;
        TArray<uint8> Bytes;
        const uint32 Header[] = {1, 0x01020304, uint32(Bodies.Num()), uint32(Parents.Num()), uint32(NativeJoints.Num()),
            sizeof(MMDBulletPhysics::RigidBodySimulationData), sizeof(MMDBulletPhysics::NativeJointData), sizeof(MMDBulletPhysics::NativeConfig),
            16, 1, uint32(Iterations), 1, MaximumSteps, uint32(QaEventsMaxBytes), uint32(QaEventsMaxSeconds), 0};
        const float Scalars[] = {StepSeconds, ComponentScale, 0, -9.8f, 0};
        QaEventsAppend(Bytes, Header, sizeof(Header)); QaEventsAppend(Bytes, Scalars, sizeof(Scalars));
        QaEventsAppend(Bytes, &Config, sizeof(Config));
        QaEventsAppend(Bytes, NativeBodies.GetData(), NativeBodies.Num() * sizeof(MMDBulletPhysics::RigidBodySimulationData));
        QaEventsAppend(Bytes, NativeJoints.GetData(), NativeJoints.Num() * sizeof(MMDBulletPhysics::NativeJointData));
        QaEventsAppend(Bytes, Parents.GetData(), Parents.Num() * sizeof(int32));
        QaEventsAppend(Bytes, Bones.GetData(), Bones.Num() * sizeof(int32));
        const auto Matrices = [&](const TArray<FTransform>& Values)
        {
            for (const FTransform& T : Values)
            {
                float M[16];
                if (!WriteNative(T, M)) return false;
                QaEventsAppend(Bytes, M, sizeof(M));
            }
            return true;
        };
        if (!Matrices(Reference) || !Matrices(BodyOffsets) || !Matrices({ToWorld}))
        { UE_LOG(LogTemp, Warning, TEXT("PMX_QA_EVENTS saved=0 invalid_metadata path=\"%s\""), *QaEventsPath); return; }
        if (uint64(Bytes.Num()) + 44ull > QaEventsMaxBytes
            || !IFileManager::Get().MakeDirectory(*FPaths::GetPath(QaEventsPath), true))
        { UE_LOG(LogTemp, Warning, TEXT("PMX_QA_EVENTS saved=0 metadata_or_directory path=\"%s\""), *QaEventsPath); return; }
        // Reserve once before the first step; motion capture performs no per-frame disk writes.
        QaEventsBuffer.Reset();
        QaEventsBuffer.Reserve(int32(QaEventsMaxBytes));
        QaEventsWriter = MakeUnique<FMemoryWriter>(QaEventsBuffer, true);
        ANSICHAR Magic[8] = {'P','M','X','U','E','E','0','1'};
        QaEventsWriter->Serialize(Magic, sizeof(Magic)); QaEventsBytes = sizeof(Magic);
        if (QaEventsRecord(0, Bytes.GetData(), Bytes.Num()))
            UE_LOG(LogTemp, Display, TEXT("PMX_QA_EVENTS recording=1 max_bytes=%llu max_seconds=90 path=\"%s\""), QaEventsMaxBytes, *QaEventsPath);
    }
    void QaEventsSet(int32 Count, const float* Matrices, const int32* Indices, bool bReset)
    {
        if (!QaEventsWriter) return;
        TArray<uint8> Bytes;
        const uint32 Header[] = {uint32(Count), uint32(bReset)};
        QaEventsAppend(Bytes, Header, sizeof(Header)); QaEventsAppend(Bytes, Indices, Count * sizeof(int32));
        QaEventsAppend(Bytes, Matrices, Count * 16 * sizeof(float));
        QaEventsRecord(1, Bytes.GetData(), Bytes.Num());
    }
    void QaEventsReset(uint32 Seed) { QaEventsRecord(2, &Seed, sizeof(Seed)); }
    void QaEventsStep(float Delta) { QaEventsRecord(3, &Delta, sizeof(Delta)); }
    void QaEventsShift(int32 Index, const float* Delta)
    {
        if (!QaEventsWriter) return;
        TArray<uint8> Bytes;
        QaEventsAppend(Bytes, &Index, sizeof(Index)); QaEventsAppend(Bytes, Delta, 3 * sizeof(float));
        QaEventsRecord(4, Bytes.GetData(), Bytes.Num());
    }
    void QaEventsBegin(float Delta)
    {
        if (!QaEventsWriter) return;
        if (QaEventsSeconds + Delta > QaEventsMaxSeconds) { QaEventsFinish(2); return; }
        QaEventsSeconds += Delta;
        ++QaEventsFrames;
        TArray<uint8> Bytes;
        QaEventsAppend(Bytes, &QaEventsFrames, sizeof(QaEventsFrames)); QaEventsAppend(Bytes, &Delta, sizeof(Delta));
        QaEventsRecord(5, Bytes.GetData(), Bytes.Num());
    }
    void QaEventsEnd(int32 Steps, uint32 Reason)
    {
        if (!QaEventsWriter) return;
        TArray<float> Snapshot;
        Snapshot.SetNumUninitialized(AllIndices.Num() * 16);
        MMDBulletPhysicsGetRigidBodyWorldTransforms(Context, AllIndices.Num(), AllIndices.GetData(), Snapshot.GetData());
        TArray<uint8> Bytes;
        const uint32 Header[] = {QaEventsFrames, uint32(Steps), Reason, uint32(AllIndices.Num())};
        QaEventsAppend(Bytes, Header, sizeof(Header)); QaEventsAppend(Bytes, Snapshot.GetData(), Snapshot.Num() * sizeof(float));
        if (QaEventsRecord(6, Bytes.GetData(), Bytes.Num())) ++QaEventsEnded;
    }

    void QaDump(const TArray<FTransform>& Animated, const FTransform& ToWorld)
    {
        if (QaDumpPath.IsEmpty() || QaElapsed < 14.) return;
        const FString Path = MoveTemp(QaDumpPath);
        TArray<uint8> Bytes;
        const auto Append = [&](const void* Data, int32 Size)
        {
            const int32 Start = Bytes.AddUninitialized(Size);
            if (Size) FMemory::Memcpy(Bytes.GetData() + Start, Data, Size);
        };
        const auto Matrices = [&](const TArray<FTransform>& Transforms)
        {
            for (const FTransform& T : Transforms)
            {
                float Matrix[16];
                if (!WriteNative(T, Matrix)) return false;
                Append(Matrix, sizeof(Matrix));
            }
            return true;
        };
        const ANSICHAR Magic[8] = {'P','M','X','U','E','Q','0','1'};
        // Header flags: component matrices use native reflected basis/metres; final body snapshot is world-space after Mode2 correction.
        const uint32 Header[] = {1, 0x01020304, uint32(Bodies.Num()), uint32(Parents.Num()), uint32(QaNativeJoints.Num()),
            sizeof(MMDBulletPhysics::RigidBodySimulationData), sizeof(MMDBulletPhysics::NativeJointData), sizeof(MMDBulletPhysics::NativeConfig), 16, 1, uint32(Iterations), MaximumSteps};
        const float Scalars[] = {StepSeconds, ComponentScale};
        Append(Magic, sizeof(Magic)); Append(Header, sizeof(Header)); Append(Scalars, sizeof(Scalars)); Append(&QaElapsed, sizeof(QaElapsed));
        Append(&QaConfig, sizeof(QaConfig));
        Append(QaNativeBodies.GetData(), QaNativeBodies.Num() * sizeof(MMDBulletPhysics::RigidBodySimulationData));
        Append(QaNativeJoints.GetData(), QaNativeJoints.Num() * sizeof(MMDBulletPhysics::NativeJointData));
        Append(Parents.GetData(), Parents.Num() * sizeof(int32)); Append(Bones.GetData(), Bones.Num() * sizeof(int32));
        bool bValid = Matrices(QaReference) && Matrices(BodyOffsets) && Matrices({QaInitialToWorld, ToWorld}) && Matrices(Animated);
        TArray<float> Snapshot;
        Snapshot.SetNumUninitialized(AllIndices.Num() * 16);
        MMDBulletPhysicsGetRigidBodyWorldTransforms(Context, AllIndices.Num(), AllIndices.GetData(), Snapshot.GetData());
        Append(Snapshot.GetData(), Snapshot.Num() * sizeof(float));
        const bool bSaved = bValid && IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true)
            && FFileHelper::SaveArrayToFile(Bytes, *Path);
        UE_LOG(LogTemp, Display, TEXT("PMX_QA_DUMP saved=%d elapsed=%.6f bytes=%d bodies=%d joints=%d bones=%d path=\"%s\""),
            bSaved, QaElapsed, Bytes.Num(), Bodies.Num(), QaNativeJoints.Num(), Parents.Num(), *Path);
        QaNativeBodies.Reset(); QaNativeJoints.Reset(); QaReference.Reset();
    }
    struct FQaPoseDelta { double Position = 0, Angle = 0; int32 PositionBone = INDEX_NONE, AngleBone = INDEX_NONE; };
    struct FQaTargetDelta { double Position = 0, Basis = 0; int32 PositionBody = INDEX_NONE, BasisBody = INDEX_NONE; };
    FQaTargetDelta QaSentDelta, QaActualDelta;
    FQaPoseDelta QaMeasurePose(const TArray<FTransform>& Now, const TArray<FTransform>& Before, bool bLocal) const
    {
        FQaPoseDelta Result;
        if (Now.Num() != Before.Num()) return Result;
        for (int32 I = 0; I < Now.Num(); ++I)
        {
            const int32 P = Parents[I];
            const FTransform A = bLocal && P != INDEX_NONE ? Now[I].GetRelativeTransform(Now[P]) : Now[I];
            const FTransform B = bLocal && P != INDEX_NONE ? Before[I].GetRelativeTransform(Before[P]) : Before[I];
            const double Position = FVector::Distance(A.GetLocation(), B.GetLocation());
            const FQuat Relative = (A.GetRotation().GetNormalized() * B.GetRotation().GetNormalized().Inverse()).GetNormalized();
            const double Angle = FMath::RadiansToDegrees(2. * FMath::Atan2(FVector(Relative.X, Relative.Y, Relative.Z).Length(), FMath::Abs(Relative.W)));
            if (Position > Result.Position) { Result.Position = Position; Result.PositionBone = I; }
            if (Angle > Result.Angle) { Result.Angle = Angle; Result.AngleBone = I; }
        }
        return Result;
    }
    void QaMeasureTargets(const TArray<float>& Target)
    {
        TArray<float> Actual;
        Actual.SetNumUninitialized(Target.Num());
        MMDBulletPhysicsGetRigidBodyWorldTransforms(Context, KinematicIndices.Num(), KinematicIndices.GetData(), Actual.GetData());
        const auto Measure = [&](const TArray<float>& Before, FQaTargetDelta& Result)
        {
            if (Before.Num() != Target.Num()) return;
            for (int32 K = 0; K < KinematicIndices.Num(); ++K)
            {
                const float* A = Target.GetData() + K * 16, * B = Before.GetData() + K * 16;
                const double Position = FVector(double(A[12]) - B[12], double(A[13]) - B[13], double(A[14]) - B[14]).Length() * 100.;
                double Basis = 0;
                for (int32 Row = 0; Row < 3; ++Row)
                    Basis = FMath::Max(Basis, FVector(double(A[Row]) - B[Row], double(A[Row + 4]) - B[Row + 4], double(A[Row + 8]) - B[Row + 8]).Length());
                if (Position > Result.Position) { Result.Position = Position; Result.PositionBody = KinematicIndices[K]; }
                if (Basis > Result.Basis) { Result.Basis = Basis; Result.BasisBody = KinematicIndices[K]; }
            }
        };
        Measure(QaTargets, QaSentDelta); Measure(Actual, QaActualDelta);
        QaTargets = Target;
    }
    void QaReport(float Delta, int32 Steps, const TCHAR* ResetReason, const TArray<FTransform>& Animated, const TArray<FTransform>& Output)
    {
        if (!bQaTrace) return;
        const FQaPoseDelta A = QaMeasurePose(Animated, QaAnimated, false), O = QaMeasurePose(Output, QaOutput, true);
        const auto Name = [&](int32 Index) { return QaBoneNames.IsValidIndex(Index) ? QaBoneNames[Index].ToString() : FString(TEXT("-")); };
        UE_LOG(LogTemp, Display, TEXT("PMX_QA frame=%u dt=%.9g steps=%d reset=%s anim_cm=%.9g anim_p=%d anim_pname=\"%s\" anim_deg=%.9g anim_r=%d anim_rname=\"%s\" sent_cm=%.9g sent_pbody=%d sent_basis=%.9g sent_rbody=%d actual_cm=%.9g actual_pbody=%d actual_basis=%.9g actual_rbody=%d out_cm=%.9g out_p=%d out_pname=\"%s\" out_deg=%.9g out_r=%d out_rname=\"%s\""),
            ++QaFrame, Delta, Steps, ResetReason, A.Position, A.PositionBone, *Name(A.PositionBone), A.Angle, A.AngleBone, *Name(A.AngleBone),
            QaSentDelta.Position, QaSentDelta.PositionBody, QaSentDelta.Basis, QaSentDelta.BasisBody,
            QaActualDelta.Position, QaActualDelta.PositionBody, QaActualDelta.Basis, QaActualDelta.BasisBody,
            O.Position, O.PositionBone, *Name(O.PositionBone), O.Angle, O.AngleBone, *Name(O.AngleBone));
        QaAnimated = Animated; QaOutput = Output;
    }
#endif
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
#if WITH_DEV_AUTOMATION_TESTS
    if (!QaStepSettings(FCommandLine::Get(), State->StepSeconds, State->Iterations))
    { Error = TEXT("Invalid PMX QA rate or iteration count."); return false; }
    State->bQaTrace = FParse::Param(FCommandLine::Get(), TEXT("PmxQaTrace"));
    FParse::Value(FCommandLine::Get(), TEXT("PmxQaEvents="), State->QaEventsPath);
    if (!State->QaEventsPath.IsEmpty() && (FPaths::IsRelative(State->QaEventsPath) || !State->QaEventsPath.EndsWith(TEXT(".bin"))))
    {
        UE_LOG(LogTemp, Warning, TEXT("PMX_QA_EVENTS rejected: an absolute .bin path is required."));
        State->QaEventsPath.Reset();
    }
    FParse::Value(FCommandLine::Get(), TEXT("PmxQaDump="), State->QaDumpPath);
    if (!State->QaDumpPath.IsEmpty() && (FPaths::IsRelative(State->QaDumpPath) || !State->QaDumpPath.EndsWith(TEXT(".bin"))))
    {
        UE_LOG(LogTemp, Warning, TEXT("PMX_QA_DUMP rejected: an absolute .bin path is required."));
        State->QaDumpPath.Reset();
    }
#endif
    if (!UniformScale(ComponentToWorld, State->ComponentScale) || Ref.GetNum() == 0)
    { Error = TEXT("PMX physics needs a valid skeleton and positive uniform component scale."); return false; }
    TArray<FTransform> RefPose;
    for (int32 I = 0; I < Ref.GetNum(); ++I)
    {
        const int32 Parent = Ref.GetParentIndex(I);
        if (Parent < INDEX_NONE || Parent >= I || !ValidTransform(Ref.GetRefBonePose()[I]))
        { Error = TEXT("Invalid PMX reference skeleton."); return false; }
        State->Parents.Add(Parent);
#if WITH_DEV_AUTOMATION_TESTS
        if (State->bQaTrace) State->QaBoneNames.Add(Ref.GetBoneName(I));
#endif
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
        FMemory::Memzero(&N, sizeof(N));
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
        FMemory::Memzero(&N, sizeof(N));
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
    State->Context = MMDBulletPhysicsCreate(Gravity, State->Iterations, 1, State->StepSeconds);
    if (!State->Context) { Error = TEXT("Cannot create PMX physics context."); return false; }
#if WITH_DEV_AUTOMATION_TESTS
    UE_LOG(LogTemp, Display, TEXT("PMX_QA_SETTINGS hz=%.3f iterations=%d"), 1.f / State->StepSeconds, State->Iterations);
#endif
    MMDBulletPhysics::NativeConfig Config{};
    Config.groundNormal[1] = 1;
    Config.mmdUnitToUnityUnit = LengthScale;
    Config.convexMargin = 1.e-6f;
    Config.springTranslationDamping = .5f;
    Config.springRotationDamping = .5f;
    Config.lockedTranslationReinforce = 1;
    MMDBulletPhysicsSetConfig(State->Context, &Config);
#if WITH_DEV_AUTOMATION_TESTS
    if (!State->QaDumpPath.IsEmpty())
    {
        State->QaNativeBodies = NativeBodies; State->QaNativeJoints = NativeJoints;
        State->QaReference = RefPose; State->QaInitialToWorld = ComponentToWorld; State->QaConfig = Config;
    }
#endif
    MMDBulletPhysicsBuildRigidBodies(State->Context, NativeBodies.GetData(), NativeBodies.Num());
    MMDBulletPhysicsBuildJoints(State->Context, NativeJoints.GetData(), NativeJoints.Num());
#if WITH_DEV_AUTOMATION_TESTS
    State->QaEventsStart(NativeBodies, NativeJoints, Config, RefPose, ComponentToWorld);
#endif
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
#if WITH_DEV_AUTOMATION_TESTS
    Impl->QaEventsSet(Impl->AllIndices.Num(), Matrices.GetData(), Impl->AllIndices.GetData(), true);
#endif
    MMDBulletPhysicsSetRigidBodyTransforms(Impl->Context, Impl->AllIndices.Num(), Matrices.GetData(), Impl->AllIndices.GetData(), true);
#if WITH_DEV_AUTOMATION_TESTS
    Impl->QaEventsReset(0);
#endif
    MMDBulletPhysicsReset(Impl->Context, 0);
    Impl->PreviousPose = BoneComponentPose; Impl->PreviousComponentToWorld = ComponentToWorld; Impl->Accumulator = 0;
    if (!Impl->ReadBodies(Impl->DynamicIndices, Impl->RenderCurrentBodies)) return false;
    Impl->RenderPreviousBodies = Impl->RenderCurrentBodies; Impl->bRenderHistory = false;
    Impl->RenderCurrentAnimation = BoneComponentPose;
    for (FTransform& Bone : Impl->RenderCurrentAnimation) Bone *= ComponentToWorld;
    Impl->RenderPreviousAnimation = Impl->RenderCurrentAnimation;
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
#if WITH_DEV_AUTOMATION_TESTS
    Impl->QaEventsBegin(DeltaSeconds);
    if (!Impl->QaDumpPath.IsEmpty()) Impl->QaElapsed += DeltaSeconds;
    if (Impl->bQaTrace) { Impl->QaSentDelta = {}; Impl->QaActualDelta = {}; }
#endif
    if (DeltaSeconds > .25f)
    {
        const bool bReset = Reset(BoneComponentPose, ComponentToWorld);
#if WITH_DEV_AUTOMATION_TESTS
        Impl->QaEventsEnd(0, bReset ? 1u : 2u);
        Impl->QaReport(DeltaSeconds, 0, bReset ? TEXT("long_frame") : TEXT("long_frame_failed"), BoneComponentPose, BoneComponentPose);
#endif
        return bReset;
    }
    const TArray<FTransform> Animated = BoneComponentPose;
    const float StepSeconds = Impl->StepSeconds;
    const float AccumulatorBefore = Impl->Accumulator;
    const float AcceptedDelta = FMath::Min(DeltaSeconds, StepSeconds * MaximumSteps);
    Impl->Accumulator += AcceptedDelta;
    const int32 Steps = FMath::Min(FMath::FloorToInt(Impl->Accumulator / StepSeconds), MaximumSteps);
    Impl->Accumulator -= Steps * StepSeconds;
    TArray<float> KinematicMatrices;
    KinematicMatrices.SetNumUninitialized(Impl->KinematicIndices.Num() * 16);
    for (int32 Step = 0; Step < Steps; ++Step)
    {
        // Sample animation at the actual fixed-step time, including a partially accumulated frame.
        const float Alpha = AcceptedDelta > SMALL_NUMBER
            ? FMath::Clamp(((Step + 1) * StepSeconds - AccumulatorBefore) / AcceptedDelta, 0.f, 1.f) : 1.f;
        Impl->RenderPreviousAnimation = MoveTemp(Impl->RenderCurrentAnimation);
        Impl->RenderCurrentAnimation.SetNum(Animated.Num());
        for (int32 Bone = 0; Bone < Animated.Num(); ++Bone)
            Impl->RenderCurrentAnimation[Bone].Blend(Impl->PreviousPose[Bone] * Impl->PreviousComponentToWorld,
                Animated[Bone] * ComponentToWorld, Alpha);
        for (int32 K = 0; K < Impl->KinematicIndices.Num(); ++K)
        {
            const int32 I = Impl->KinematicIndices[K];
            FTransform Interpolated;
            Interpolated.Blend(Impl->AnimatedBody(I, Impl->PreviousPose, Impl->PreviousComponentToWorld),
                Impl->AnimatedBody(I, Animated, ComponentToWorld), Alpha);
            if (!WriteNative(Interpolated, KinematicMatrices.GetData() + K * 16))
            { Error = TEXT("PMX animation transform overflows native precision."); return false; }
        }
#if WITH_DEV_AUTOMATION_TESTS
        if (Impl->bQaTrace) Impl->QaMeasureTargets(KinematicMatrices);
#endif
#if WITH_DEV_AUTOMATION_TESTS
        Impl->QaEventsSet(Impl->KinematicIndices.Num(), KinematicMatrices.GetData(), Impl->KinematicIndices.GetData(), false);
#endif
        MMDBulletPhysicsSetRigidBodyTransforms(Impl->Context, Impl->KinematicIndices.Num(), KinematicMatrices.GetData(), Impl->KinematicIndices.GetData(), false);
#if WITH_DEV_AUTOMATION_TESTS
        Impl->QaEventsStep(StepSeconds);
#endif
        MMDBulletPhysicsStepSimulation(Impl->Context, StepSeconds);
        TArray<FTransform> Dynamic;
        if (!Impl->ReadBodies(Impl->DynamicIndices, Dynamic))
        { Reset(Animated, ComponentToWorld); Error = TEXT("Non-finite PMX body result; simulation reset."); return false; }
        Impl->RenderPreviousBodies = MoveTemp(Impl->RenderCurrentBodies);
        Impl->RenderCurrentBodies = MoveTemp(Dynamic);
        Impl->bRenderHistory = true;
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
#if WITH_DEV_AUTOMATION_TESTS
                Impl->QaEventsShift(I, Shift);
#endif
                MMDBulletPhysicsShiftRigidBodyPosition(Impl->Context, I, Shift);
            }
            else Resolved[Bone] = *Target;
        }
        if (!ValidTransform(Resolved[Bone]))
        { Reset(Animated, ComponentToWorld); Error = TEXT("Invalid PMX descendant pose; simulation reset."); return false; }
    }
    if (Impl->bRenderHistory && Impl->RenderPreviousBodies.Num() == Dynamic.Num())
    {
        // Head/body animation must share the physical display time; native contacts and Mode2 remain current.
        const float RenderAlpha = FMath::Clamp(Impl->Accumulator / StepSeconds, 0.f, 1.f);
        TArray<FTransform> DisplayAnimation;
        DisplayAnimation.SetNum(Animated.Num());
        for (int32 Bone = 0; Bone < Animated.Num(); ++Bone)
        {
            FTransform World;
            World.Blend(Impl->RenderPreviousAnimation[Bone], Impl->RenderCurrentAnimation[Bone], RenderAlpha);
            DisplayAnimation[Bone] = World.GetRelativeTransform(ComponentToWorld);
        }
        Driven.Reset();
        for (int32 K = 0; K < Impl->DynamicIndices.Num(); ++K)
        {
            const int32 I = Impl->DynamicIndices[K], Bone = Impl->Bones[I];
            if (Bone == INDEX_NONE) continue;
            FTransform Interpolated; Interpolated.Blend(Impl->RenderPreviousBodies[K], Dynamic[K], RenderAlpha);
            FTransform Target = Impl->BoneFromBody(I, Interpolated, ComponentToWorld);
            Target.SetScale3D(Animated[Bone].GetScale3D());
            Driven.Add(Bone, Target);
        }
        for (int32 Bone = 0; Bone < Animated.Num(); ++Bone)
        {
            const int32 Parent = Impl->Parents[Bone];
            const FTransform Local = Parent == INDEX_NONE ? DisplayAnimation[Bone] : DisplayAnimation[Bone].GetRelativeTransform(DisplayAnimation[Parent]);
            Resolved[Bone] = Parent == INDEX_NONE ? Local : Local * Resolved[Parent];
            if (const auto* Target = Driven.Find(Bone))
            {
                const int32 I = Impl->DynamicIndices[DynamicSources[Bone]];
                if (Impl->Bodies[I].Mode == 2) Resolved[Bone].SetRotation(Target->GetRotation());
                else Resolved[Bone] = *Target;
            }
            if (!ValidTransform(Resolved[Bone]))
            { Reset(Animated,ComponentToWorld); Error=TEXT("Invalid PMX display pose; simulation reset."); return false; }
        }
    }
#if WITH_DEV_AUTOMATION_TESTS
    Impl->QaEventsEnd(Steps, 0);
    Impl->QaDump(Animated, ComponentToWorld);
    Impl->QaReport(DeltaSeconds, Steps, TEXT("none"), Animated, Resolved);
#endif
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

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxQaStepSettingsTest, "VRM4U.PMX.QaStepSettings",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxQaStepSettingsTest::RunTest(const FString&)
{
    float Step = FixedStep;
    int32 Iterations = SolverIterations;
    TestTrue(TEXT("No override retains defaults"), QaStepSettings(TEXT(""), Step, Iterations));
    TestEqual(TEXT("Default timestep is unchanged"), Step, FixedStep);
    TestEqual(TEXT("Default iterations are unchanged"), Iterations, SolverIterations);
    TestTrue(TEXT("QA can select a measured rate and budget"),
        QaStepSettings(TEXT("-PmxQaHz=30 -PmxQaIterations=64"), Step, Iterations));
    TestEqual(TEXT("Actual step is 30 Hz"), Step, 1.f / 30.f);
    TestEqual(TEXT("Actual solver budget is retained"), Iterations, 64);
    for (const TCHAR* Invalid : {TEXT("-PmxQaHz=0"), TEXT("-PmxQaHz=121"),
        TEXT("-PmxQaHz=nan"), TEXT("-PmxQaIterations=0"), TEXT("-PmxQaIterations=161")})
    {
        Step = FixedStep; Iterations = SolverIterations;
        TestFalse(FString(TEXT("Invalid QA settings rejected: ")) + Invalid, QaStepSettings(Invalid, Step, Iterations));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxSubstepTimingTest, "VRM4U.PMX.SubstepTiming",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxSubstepTimingTest::RunTest(const FString&)
{
#if WITH_VRM4U_PMX_BULLET
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier Modifier(Skeleton);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("head"), TEXT("head"), 0), FTransform(FVector(0,0,100)));
    }
    auto* Data = NewObject<UVrmPmxPhysicsData>(); Data->ModelScale = .1f;
    auto& Head = Data->RigidBodies.AddDefaulted_GetRef();
    Head.BoneName = TEXT("head"); Head.SourceBoneIndex = 1; Head.Mode = 0; Head.Shape = 0;
    Head.Size = FVector(.2,0,0); Head.Position = FVector(0,10,0); Head.CollisionMask = 0;
    TArray<TArray<float>> Cases;
    for (int32 Rate : {24,30,60,72,90,120,144}) Cases.Add({1.f / Rate});
    Cases.Add({1.f/144,1.f/90,0.f,1.f/24,1.f/60,.15f});
    for (int32 Case = 0; Case < Cases.Num(); ++Case)
    {
        FString Error; FVrmPmxPhysics Physics;
        if (!TestTrue(TEXT("A real native kinematic head initializes"),Physics.Initialize(*Data,Skeleton->GetReferenceSkeleton(),FTransform::Identity,Error))) return false;
        double Time = 0, MaxPosition = 0, MaxAngle = 0;
        int32 Completed = 0;
        for (int32 Frame = 0; Frame < 240; ++Frame)
        {
            const float Delta = Cases[Case][Frame % Cases[Case].Num()];
            Time += FMath::Min(Delta,FixedStep * MaximumSteps);
            const FTransform Target(FRotator(0,Time*60.,0),FVector(Time*30.,0,100));
            TArray<FTransform> Pose{FTransform::Identity,Target};
            const float AccumulatorBefore = Physics.Impl->Accumulator;
            Completed += FMath::Min(FMath::FloorToInt((AccumulatorBefore + FMath::Min(Delta,FixedStep*MaximumSteps))/FixedStep),MaximumSteps);
            if (!Physics.Simulate(Delta,FTransform::Identity,Pose,Error)) { AddError(Error); return false; }
            TArray<FTransform> Bodies;
            if (!Physics.Impl->ReadBodies(Physics.Impl->AllIndices,Bodies) || Bodies.Num()!=1) { AddError(TEXT("Native head readback failed")); return false; }
            const FTransform Actual = Physics.Impl->BoneFromBody(0,Bodies[0],FTransform::Identity);
            const double SampleTime = double(Completed) * FixedStep;
            const FTransform Expected(FRotator(0,SampleTime*60.,0),FVector(SampleTime*30.,0,100));
            MaxPosition = FMath::Max(MaxPosition,FVector::Distance(Actual.GetLocation(),Expected.GetLocation()));
            MaxAngle = FMath::Max(MaxAngle,FMath::RadiansToDegrees(Actual.GetRotation().AngularDistance(Expected.GetRotation())));
        }
        AddInfo(FString::Printf(TEXT("PMX_SUBSTEP_TIMING case=%d max_cm=%.6f max_deg=%.6f"),Case,MaxPosition,MaxAngle));
        TestTrue(FString::Printf(TEXT("Case %d samples the actual fixed-step position"),Case),MaxPosition < .005);
        TestTrue(FString::Printf(TEXT("Case %d samples the actual fixed-step rotation"),Case),MaxAngle < .02);
    }
#else
    AddError(TEXT("Substep timing requires the native Bullet runtime."));
#endif
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxRenderInterpolationTest, "VRM4U.PMX.RenderInterpolation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxRenderInterpolationTest::RunTest(const FString&)
{
#if WITH_VRM4U_PMX_BULLET
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier Modifier(Skeleton);
        Modifier.Add(FMeshBoneInfo(TEXT("root"),TEXT("root"),INDEX_NONE),FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("dynamic"),TEXT("dynamic"),0),FTransform(FVector(0,0,100)));
    }
    auto* Data=NewObject<UVrmPmxPhysicsData>(); Data->ModelScale=.1f;
    auto& Body=Data->RigidBodies.AddDefaulted_GetRef();
    Body.BoneName=TEXT("dynamic"); Body.SourceBoneIndex=1; Body.Mode=1; Body.Shape=0;
    Body.Size=FVector(.2,0,0); Body.Position=FVector(0,10,0); Body.Mass=1; Body.CollisionMask=0;
    const TArray<FTransform> Animated{FTransform::Identity,FTransform(FVector(0,0,100))};
    FString Error; FVrmPmxPhysics HighRate, Reference;
    if (!HighRate.Initialize(*Data,Skeleton->GetReferenceSkeleton(),FTransform::Identity,Error)
        || !Reference.Initialize(*Data,Skeleton->GetReferenceSkeleton(),FTransform::Identity,Error)) { AddError(Error); return false; }
    int32 Completed=0, MovingZeroFrames=0;
    double PreviousOutput=100;
    TArray<FTransform> Native, PreviousNative;
    for (int32 Frame=0;Completed<60 && Frame<400;++Frame)
    {
        const int32 Steps=FMath::FloorToInt((HighRate.Impl->Accumulator + 1.f/144.f)/FixedStep);
        auto Pose=Animated;
        if (!HighRate.Simulate(1.f/144.f,FTransform::Identity,Pose,Error) || !HighRate.Impl->ReadBodies(HighRate.Impl->AllIndices,Native))
        { AddError(Error); return false; }
        if (Frame>10 && Steps==0 && PreviousNative.Num()==Native.Num())
        {
            TestTrue(TEXT("Zero-step display leaves the real native body untouched"),Native[0].Equals(PreviousNative[0],1.e-8));
            if (FMath::Abs(Pose[1].GetLocation().Z-PreviousOutput)>1.e-6) ++MovingZeroFrames;
        }
        Completed+=Steps; PreviousOutput=Pose[1].GetLocation().Z; PreviousNative=Native;
    }
    TestEqual(TEXT("High-rate playback completes sixty physics steps"),Completed,60);
    TestTrue(TEXT("High-rate frames interpolate the visible body while physics is idle"),MovingZeroFrames>50);
    for (int32 Step=0;Step<Completed;++Step)
    {
        auto Pose=Animated;
        if (!Reference.Simulate(FixedStep,FTransform::Identity,Pose,Error)) { AddError(Error); return false; }
    }
    TArray<FTransform> ReferenceNative;
    if (!Reference.Impl->ReadBodies(Reference.Impl->AllIndices,ReferenceNative)) return false;
    TestTrue(TEXT("Display interpolation does not change the native fixed-step trajectory"),
        Native[0].Equals(ReferenceNative[0],.0001));
    TestTrue(TEXT("Reset discards old display history"),HighRate.Reset(Animated,FTransform::Identity) && !HighRate.Impl->bRenderHistory);
    auto Rest=Animated;
    TestTrue(TEXT("Zero-time post-reset output is the current pose"),HighRate.Simulate(0,FTransform::Identity,Rest,Error) && Rest[1].Equals(Animated[1],.001));
#else
    AddError(TEXT("Display interpolation regression requires native Bullet."));
#endif
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxDisplayAttachmentTest, "VRM4U.PMX.DisplayAttachment",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FVrmPmxDisplayAttachmentTest::RunTest(const FString&)
{
#if WITH_VRM4U_PMX_BULLET
    auto* Skeleton = NewObject<USkeleton>();
    {
        FReferenceSkeletonModifier Modifier(Skeleton);
        Modifier.Add(FMeshBoneInfo(TEXT("root"), TEXT("root"), INDEX_NONE), FTransform::Identity);
        Modifier.Add(FMeshBoneInfo(TEXT("head"), TEXT("head"), 0), FTransform(FVector(0, 0, 100)));
        Modifier.Add(FMeshBoneInfo(TEXT("follower"), TEXT("follower"), 1), FTransform(FVector(20, 0, 0)));
    }
    auto* Data = NewObject<UVrmPmxPhysicsData>(); Data->ModelScale = .1f;
    for (int32 I = 0; I < 2; ++I)
    {
        auto& B = Data->RigidBodies.AddDefaulted_GetRef();
        B.BoneName = I ? TEXT("follower") : TEXT("head"); B.SourceBoneIndex = I + 1;
        B.Mode = I; B.Shape = 0; B.Size = FVector(.2, 0, 0); B.Mass = 1;
        B.Position = FVector(I * 2, 10, 0); B.CollisionMask = 0;
    }
    auto& Joint = Data->Joints.AddDefaulted_GetRef();
    Joint.BodyA = 0; Joint.BodyB = 1; Joint.Position = FVector(0, 10, 0);
    const FTransform Offset(FVector(20, 0, 0));
    FString Error;
    for (float Fps : {30.f, 60.f, 90.f, 144.f})
    {
        FVrmPmxPhysics Physics;
        if (!Physics.Initialize(*Data, Skeleton->GetReferenceSkeleton(), FTransform::Identity, Error))
        { AddError(Error); return false; }
        double MaximumGap = 0;
        for (int32 Frame = 0; Frame < 180; ++Frame)
        {
            const FTransform Head(FRotator(0, 0, 30 * FMath::Sin(Frame / Fps * 3)), FVector(0, 0, 100));
            TArray<FTransform> Pose{FTransform::Identity, Head, Offset * Head};
            if (!Physics.Simulate(1.f / Fps, FTransform::Identity, Pose, Error))
            { AddError(Error); return false; }
            if (Frame > 10) MaximumGap = FMath::Max(MaximumGap,
                FVector::Distance(Pose[2].GetLocation(), (Offset * Pose[1]).GetLocation()));
        }
        TestTrue(FString::Printf(TEXT("%.0f FPS: animated head and physical follower share display time (gap %.6f cm)"), Fps, MaximumGap), MaximumGap < .04);
    }
#else
    AddError(TEXT("Display attachment regression requires native Bullet."));
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
                // External native-state injection starts a new display history.
                if (!Physics.Reset(Animated, World)) return false;
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
                    FTransform CurrentParent = Physics.Impl->BoneFromBody(0, Bodies[0], World);
                    if (ParentMode == 2) CurrentParent.SetLocation(Animated[1].GetLocation());
                    TestTrue(TEXT("Current-world mode2 correction has no display-interpolation feedback"),
                        SteppedChild.GetLocation().Equals((ChildLocal * CurrentParent).GetLocation(), .001));
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
