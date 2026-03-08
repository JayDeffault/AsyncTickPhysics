#include "WheelComponent.h"

#include "AsyncTickFunctions.h"
#include "CollisionQueryParams.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include <cmath>

UWheelComponent::UWheelComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	CurrentSuspensionLength = SuspensionUpperLimit + SuspensionLowerLimit;
}

void UWheelComponent::BeginPlay()
{
	Super::BeginPlay();
	EnsureCollisionMesh();
	EnsureVisualMesh();
}


void UWheelComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bTickWheelInGameThread || !bDebugVehicle)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const FColor SweepColor = bLastSweepHadBlockingHit ? FColor::Green : FColor::Red;
	DrawDebugLine(World, LastSweepStart, LastSweepEnd, SweepColor, false, DeltaTime, 0, 1.5f);
	DrawDebugPoint(World, LastWheelWorldLocation, 8.0f, FColor::White, false, DeltaTime, 0);

	if (bIsGrounded)
	{
		DrawDebugPoint(World, ContactPoint, 10.0f, FColor::Yellow, false, DeltaTime, 0);
		DrawDebugLine(World, ContactPoint, ContactPoint + ContactNormal * 30.0f, FColor::Cyan, false, DeltaTime, 0, 1.0f);
	}
}


void UWheelComponent::EnsureVisualMesh()
{
	AActor* OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		return;
	}

	if (!VisualWheelMesh)
	{
		VisualWheelMesh = NewObject<UStaticMeshComponent>(OwnerActor, NAME_None, RF_Transactional);
		if (!VisualWheelMesh)
		{
			return;
		}

		OwnerActor->AddInstanceComponent(VisualWheelMesh);
		VisualWheelMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		VisualWheelMesh->SetSimulatePhysics(false);
		VisualWheelMesh->SetGenerateOverlapEvents(false);
		VisualWheelMesh->RegisterComponent();
	}

	if (!VisualWheelMesh->GetStaticMesh())
	{
		if (UStaticMesh* DefaultWheelVisual = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder")))
		{
			VisualWheelMesh->SetStaticMesh(DefaultWheelVisual);
		}
	}

	if (VisualWheelMesh->GetAttachParent() != this)
	{
		VisualWheelMesh->AttachToComponent(this, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	}

	VisualWheelMesh->SetRelativeLocation(FVector::ZeroVector);
	VisualWheelMesh->SetRelativeRotation(FRotator::ZeroRotator);
	// Визуальный меш не масштабируем от параметров физики колеса (Radius/Width).
	// Сохраняем масштаб из ассета/Blueprint.
}

void UWheelComponent::UpdateVisualWheel(float DeltaTime, float SteeringAngleDeg, const FTransform& WheelWorldTransform)
{
	EnsureVisualMesh();
	if (!VisualWheelMesh)
	{
		return;
	}

	VisualSpinAngleDeg = FMath::Fmod(VisualSpinAngleDeg + FMath::RadiansToDegrees(CachedVisualAngularVelocity) * DeltaTime, 360.0f);

	const float SteerDeg = bIsSteerWheel ? SteeringAngleDeg : 0.0f;
	const FVector SpinAxisLocal = VisualWheelRotationAxis.GetSafeNormal();
	const FQuat SteerLocalQuat(FVector::UpVector, FMath::DegreesToRadians(SteerDeg));
	const FQuat SpinLocalQuat(SpinAxisLocal, FMath::DegreesToRadians(VisualSpinAngleDeg));
	const FQuat VisualLocalQuat = (SteerLocalQuat * SpinLocalQuat).GetNormalized();

	const float RestLength = SuspensionUpperLimit + SuspensionLowerLimit;
	const float CompressionOffset = FMath::Clamp(RestLength - CurrentSuspensionLength, 0.0f, RestLength);
	VisualWheelMesh->SetRelativeLocation(FVector(0.0f, 0.0f, -CompressionOffset));
	VisualWheelMesh->SetRelativeRotation(VisualLocalQuat.Rotator());
}

void UWheelComponent::UpdateVisualFromTick(float DeltaTime, float SteeringAngleDeg, const FTransform& BodyWorldTransform)
{
	const FTransform WheelWorldTransform = GetRelativeTransform() * BodyWorldTransform;
	UpdateVisualWheel(DeltaTime, SteeringAngleDeg, WheelWorldTransform);
}

void UWheelComponent::EnsureCollisionMesh()
{
	AActor* OwnerActor = GetOwner();
	if (!OwnerActor)
	{
		return;
	}

	if (!WheelCollisionMesh)
	{
		WheelCollisionMesh = NewObject<UStaticMeshComponent>(OwnerActor, NAME_None, RF_Transactional);
		if (!WheelCollisionMesh)
		{
			return;
		}

		OwnerActor->AddInstanceComponent(WheelCollisionMesh);
		WheelCollisionMesh->SetHiddenInGame(true);
		WheelCollisionMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
		WheelCollisionMesh->SetCollisionResponseToAllChannels(ECR_Ignore);
		WheelCollisionMesh->SetCollisionResponseToChannel(ECC_WorldStatic, ECR_Block);
		WheelCollisionMesh->SetCollisionResponseToChannel(ECC_WorldDynamic, ECR_Block);
		WheelCollisionMesh->SetCollisionResponseToChannel(ECC_PhysicsBody, ECR_Ignore);
		WheelCollisionMesh->SetSimulatePhysics(false);
		WheelCollisionMesh->SetGenerateOverlapEvents(false);
		WheelCollisionMesh->RegisterComponent();
	}

	if (!WheelCollisionMesh->GetStaticMesh())
	{
		if (UStaticMesh* DefaultWheelMesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere")))
		{
			WheelCollisionMesh->SetStaticMesh(DefaultWheelMesh);
		}
	}

	if (WheelCollisionMesh->GetAttachParent() != this)
	{
		WheelCollisionMesh->AttachToComponent(this, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	}

	WheelCollisionMesh->SetRelativeLocation(FVector::ZeroVector);
	WheelCollisionMesh->SetRelativeRotation(FRotator::ZeroRotator);
	const float RadiusScale = FMath::Max(0.1f, WheelRadius / 50.0f);
	WheelCollisionMesh->SetRelativeScale3D(FVector(RadiusScale, RadiusScale, RadiusScale));

	if (UPrimitiveComponent* OwnerRootPrimitive = Cast<UPrimitiveComponent>(OwnerActor->GetRootComponent()))
	{
		WheelCollisionMesh->IgnoreComponentWhenMoving(OwnerRootPrimitive, true);
		OwnerRootPrimitive->IgnoreComponentWhenMoving(WheelCollisionMesh, true);
	}
}

void UWheelComponent::UpdateContact(float DeltaTime, UPrimitiveComponent* BodyMesh, const FTransform& WheelWorldTransform)
{
	bIsGrounded = false;
	LastTotalForce = FVector::ZeroVector;
	const FVector WheelLocation = WheelWorldTransform.GetLocation();
	const FVector WheelUp = WheelWorldTransform.GetUnitAxis(EAxis::Z);
	LastWheelWorldLocation = WheelLocation;

	ContactPoint = WheelLocation - WheelUp * SuspensionLowerLimit;
	ContactNormal = FVector::UpVector;
	CompressionRatio = 0.0f;
	CachedNormalLoad = 0.0f;
	bLastSweepHadBlockingHit = false;

	if (!WheelCollisionMesh)
	{
		return;
	}

	const FVector OutStart = WheelLocation + WheelUp * SuspensionUpperLimit;
	const FVector OutEnd = WheelLocation - WheelUp * SuspensionLowerLimit;
	LastSweepStart = OutStart;
	LastSweepEnd = OutEnd;

	TArray<FHitResult> SweepHits;
	FComponentQueryParams QueryParams;
	QueryParams.AddIgnoredActor(GetOwner());
	if (BodyMesh)
	{
		QueryParams.AddIgnoredComponent(BodyMesh);
	}

	TArray<UWheelComponent*> Wheels;
	if (AActor* OwnerActor = GetOwner())
	{
		OwnerActor->GetComponents(Wheels);
		for (UWheelComponent* Wheel : Wheels)
		{
			if (Wheel && Wheel->WheelCollisionMesh)
			{
				QueryParams.AddIgnoredComponent(Wheel->WheelCollisionMesh);
			}
		}
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const bool bHits = World->ComponentSweepMulti(
		SweepHits,
		WheelCollisionMesh,
		OutStart,
		OutEnd,
		WheelWorldTransform.GetRotation(),
		QueryParams);

	if (!bHits)
	{
		return;
	}

	float ClosestDistSq = TNumericLimits<float>::Max();
	FHitResult BestHit;
	for (const FHitResult& Hit : SweepHits)
	{
		if (!Hit.bBlockingHit)
		{
			continue;
		}

		const float DistSq = FVector::DistSquared(OutStart, Hit.ImpactPoint);
		if (DistSq < ClosestDistSq)
		{
			ClosestDistSq = DistSq;
			BestHit = Hit;
		}
	}

	if (ClosestDistSq == TNumericLimits<float>::Max())
	{
		return;
	}

	bIsGrounded = true;
	bLastSweepHadBlockingHit = true;
	ContactPoint = BestHit.ImpactPoint;
	ContactNormal = BestHit.ImpactNormal.GetSafeNormal();

	const float RestLength = SuspensionUpperLimit + SuspensionLowerLimit;
	const float SuspensionTravel = FVector::DotProduct(OutStart - ContactPoint, WheelUp);
	CurrentSuspensionLength = FMath::Max(0.0f, SuspensionTravel - WheelRadius);
	CurrentSuspensionLength = FMath::Clamp(CurrentSuspensionLength, 0.0f, RestLength);

	const float Compression = FMath::Max(0.0f, RestLength - CurrentSuspensionLength);
	CompressionRatio = (RestLength > KINDA_SMALL_NUMBER) ? FMath::Clamp(Compression / RestLength, 0.0f, 1.0f) : 0.0f;

	(void)DeltaTime;
}

FWheelForces UWheelComponent::SimulateWheel(float DeltaTime, UPrimitiveComponent* BodyMesh, float DriveForce, float BrakeForce, float SteeringAngleDeg, const FTransform& WheelWorldTransform)
{
	FWheelForces Forces;
	if (!BodyMesh || DeltaTime <= KINDA_SMALL_NUMBER)
	{
		LastTotalForce = FVector::ZeroVector;
		return Forces;
	}

	UpdateContact(DeltaTime, BodyMesh, WheelWorldTransform);
	if (!bIsGrounded)
	{
		// In air: keep wheel spin inertia with gentle damping.
		WheelAngularVelocity *= FMath::Pow(FMath::Clamp(1.0f - AirborneWheelSpinDamping * 0.05f, 0.85f, 1.0f), DeltaTime * 60.0f);
		CachedVisualAngularVelocity = FMath::FInterpTo(CachedVisualAngularVelocity, WheelAngularVelocity, DeltaTime, 3.0f);
		LastTotalForce = FVector::ZeroVector;
		LinearSpeedAlongWheelForward = 0.0f;
		SlipRatioLong = 0.0f;
		SlipAngleLat = 0.0f;
		return Forces;
	}

	const FVector WheelUp = WheelWorldTransform.GetUnitAxis(EAxis::Z);
	const float RestLength = SuspensionUpperLimit + SuspensionLowerLimit;
	const float Compression = FMath::Clamp(RestLength - CurrentSuspensionLength, 0.0f, RestLength);
	const FVector PointVel = UAsyncTickFunctions::ATP_GetLinearVelocityAtPoint(BodyMesh, ContactPoint);
	const float VelocityAlongSuspension = FVector::DotProduct(PointVel, WheelUp);

	const float SpringForce = SuspensionStiffness * Compression;
	const float DamperForce = SuspensionDamping * (-VelocityAlongSuspension);
	CachedNormalLoad = FMath::Max(0.0f, SpringForce + DamperForce);
	Forces.SuspensionForce = WheelUp * CachedNormalLoad;

	const FVector TireUp = ContactNormal.IsNearlyZero() ? WheelUp : ContactNormal.GetSafeNormal();
	const FQuat SteerRot(TireUp, FMath::DegreesToRadians(SteeringAngleDeg));
	FVector WheelForward = SteerRot.RotateVector(WheelWorldTransform.GetUnitAxis(EAxis::X));
	WheelForward = FVector::VectorPlaneProject(WheelForward, TireUp).GetSafeNormal();
	const FVector WheelRight = FVector::CrossProduct(TireUp, WheelForward).GetSafeNormal();

	float VLong = FVector::DotProduct(PointVel, WheelForward);
	float VLat = FVector::DotProduct(PointVel, WheelRight);
	LinearSpeedAlongWheelForward = VLong;

	if (bUseRestStabilization && FMath::Abs(VLong) < 1.0f) VLong = 0.0f;
	if (bUseRestStabilization && FMath::Abs(VLat) < 1.0f) VLat = 0.0f;

	const float AbsLong = FMath::Abs(VLong);
	SlipAngleLat = FMath::Atan2(VLat, FMath::Max(AbsLong, 40.0f));

	const float RadiusM = FMath::Max(WheelRadius * 0.01f, 0.05f);
	const float WheelSurfaceSpeed = WheelAngularVelocity * RadiusM;
	SlipRatioLong = (AbsLong > 30.0f) ? (WheelSurfaceSpeed - VLong) / AbsLong : 0.0f;

	CachedSlipAngle = FMath::FInterpTo(CachedSlipAngle, SlipAngleLat, DeltaTime, SlipRelaxationSpeed);
	CachedSlipRatio = FMath::FInterpTo(CachedSlipRatio, SlipRatioLong, DeltaTime, SlipRelaxationSpeed);

	const float ForceFadeFloor = FMath::Clamp(MinLowSpeedForceFade, 0.0f, 1.0f);
	const float Fade = FMath::GetMappedRangeValueClamped(FVector2D(LowSpeedLateralFadeStart, LowSpeedLateralFadeEnd), FVector2D(ForceFadeFloor, 1.0f), AbsLong);

	const float LoadScale = FMath::Clamp(1.0f - LoadSensitivity + LoadSensitivity * FMath::Sqrt(FMath::Max(CachedNormalLoad, 0.0f) / 4000.0f), 0.6f, 1.4f);
	const float Mu = TireFriction * LoadScale;
	const float MaxTireForce = FMath::Max(0.0f, Mu * CachedNormalLoad);

	const float PeakSlipAngle = FMath::DegreesToRadians(FMath::Max(1.0f, SlipAnglePeakDeg));
	const float PeakSlipRatio = FMath::Max(0.03f, SlipRatioPeak);

	const float NormSA = FMath::Clamp(CachedSlipAngle / PeakSlipAngle, -3.0f, 3.0f);
	const float NormSR = FMath::Clamp(CachedSlipRatio / PeakSlipRatio, -3.0f, 3.0f);

	float FLatScalar = -std::tanh(NormSA * CorneringStiffness) * MaxTireForce * LateralFrictionScale * Fade;
	const float VelLatDamping = -VLat * LateralVelocityDamping * FMath::GetMappedRangeValueClamped(FVector2D(0.0f, 35.0f), FVector2D(1.0f, 0.5f), FMath::Abs(SteeringAngleDeg));
	FLatScalar += FMath::Clamp(VelLatDamping, -MaxTireForce * MaxLateralVelocityForceRatio, MaxTireForce * MaxLateralVelocityForceRatio);

	const float FLongSlip = -std::tanh(NormSR * LongitudinalStiffness) * MaxTireForce * LongitudinalFrictionScale * Fade;
	float FLongScalar = DriveForce - BrakeForce + FLongSlip;

	// Passive resistances
	FLongScalar += FMath::Clamp(-VLong * RollingResistanceCoeff, -MaxTireForce * 0.5f, MaxTireForce * 0.5f);
	FLatScalar += FMath::Clamp(-VLat * SideSlipDampingCoeff, -MaxTireForce * 0.6f, MaxTireForce * 0.6f);

	if (BrakeForce > 1.0f)
	{
		const float HBAlpha = FMath::Clamp(BrakeForce / 8000.0f, 0.0f, 1.0f);
		FLongScalar += FMath::Clamp(-VLong * RestVelocityDamping * HandbrakeLongitudinalDragMultiplier * HBAlpha, -MaxTireForce, MaxTireForce);
		FLatScalar *= FMath::Lerp(1.0f, HandbrakeLateralGripMultiplier, HBAlpha);
	}

	if (bUseRestStabilization && FMath::Abs(VLong) < RestSpeedThreshold && FMath::Abs(VLat) < RestSpeedThreshold && FMath::Abs(DriveForce) < 2.0f && FMath::Abs(BrakeForce) < 2.0f)
	{
		FLongScalar += FMath::Clamp(-VLong * RestVelocityDamping, -MaxTireForce * 0.35f, MaxTireForce * 0.35f);
		FLatScalar += FMath::Clamp(-VLat * RestLateralDamping, -MaxTireForce * 0.35f, MaxTireForce * 0.35f);
	}

	// Friction ellipse (combined slip)
	const float Nx = FLongScalar / FMath::Max(MaxTireForce, 1.0f);
	const float Ny = FLatScalar / FMath::Max(MaxTireForce, 1.0f);
	const float Combined = FMath::Sqrt(Nx * Nx + Ny * Ny);
	if (Combined > 1.0f)
	{
		const float Scale = 1.0f / Combined;
		FLongScalar *= Scale;
		FLatScalar *= Scale;
	}

	if (bEnableStaticFrictionLock)
	{
		const float SpeedSq = VLong * VLong + VLat * VLat;
		const bool bNearStatic = SpeedSq < StaticLockSpeedThreshold * StaticLockSpeedThreshold;
		const bool bNoInputForLock = (FMath::Abs(DriveForce) < 2.0f) && (FMath::Abs(BrakeForce) < 2.0f) && (FMath::Abs(SteeringAngleDeg) < 0.3f) && (FMath::Abs(WheelAngularVelocity) < 2.0f);
		if (bNearStatic && bNoInputForLock)
		{
			FLongScalar = FMath::Clamp(-VLong * StaticLongitudinalDamping, -MaxTireForce, MaxTireForce);
			FLatScalar = FMath::Clamp(-VLat * StaticLateralDamping, -MaxTireForce, MaxTireForce);
			if (FMath::Abs(VLong) < 1.0f) FLongScalar = 0.0f;
			if (FMath::Abs(VLat) < 1.0f) FLatScalar = 0.0f;
		}
	}

	Forces.LongitudinalForce = WheelForward * FLongScalar;
	Forces.LateralForce = WheelRight * FLatScalar;
	LastTotalForce = Forces.TotalForce();

	const float WheelInertia = FMath::Max(0.8f, WheelRadius * 0.02f);
	const float WheelAngularAccel = (FLongScalar * RadiusM) / WheelInertia;
	WheelAngularVelocity += WheelAngularAccel * DeltaTime;

	const float TargetOmega = VLong / RadiusM;
	const bool bNoDriveOrBrakeInput = (FMath::Abs(DriveForce) < 2.0f && FMath::Abs(BrakeForce) < 2.0f);
	const float SyncRate = bNoDriveOrBrakeInput ? FreeRollingAngularSync : 5.0f;
	WheelAngularVelocity = FMath::FInterpTo(WheelAngularVelocity, TargetOmega, DeltaTime, SyncRate);

	if (bUseRestStabilization && FMath::Abs(VLong) < RestSpeedThreshold && FMath::Abs(VLat) < RestSpeedThreshold && bNoDriveOrBrakeInput)
	{
		WheelAngularVelocity = FMath::FInterpTo(WheelAngularVelocity, 0.0f, DeltaTime, 8.0f);
	}

	const float VisualTargetOmega = FMath::Lerp(TargetOmega, WheelAngularVelocity, 0.35f);
	CachedVisualAngularVelocity = FMath::FInterpTo(CachedVisualAngularVelocity, VisualTargetOmega, DeltaTime, VisualGroundSpinInterp);

	return Forces;
}
