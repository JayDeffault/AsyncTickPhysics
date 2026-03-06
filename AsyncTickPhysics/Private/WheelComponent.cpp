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

	VisualWheelMesh->SetRelativeLocation(FVector(0.0f, 0.0f, -CurrentSuspensionLength));
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
	UpdateContact(DeltaTime, BodyMesh, WheelWorldTransform);

	if (!bIsGrounded || !BodyMesh)
	{
		// В воздухе сохраняем инерцию вращения колеса с мягким затуханием.
		WheelAngularVelocity = FMath::FInterpTo(WheelAngularVelocity, 0.0f, DeltaTime, AirborneWheelSpinDamping);
		CachedVisualAngularVelocity = WheelAngularVelocity;
		CachedLateralForceScalar = FMath::FInterpTo(CachedLateralForceScalar, 0.0f, DeltaTime, LateralForceSmoothing);
		LastTotalForce = FVector::ZeroVector;
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
	const FQuat SteerRot = FQuat(TireUp, FMath::DegreesToRadians(SteeringAngleDeg));
	FVector WheelForward = SteerRot.RotateVector(WheelWorldTransform.GetUnitAxis(EAxis::X));
	WheelForward = FVector::VectorPlaneProject(WheelForward, TireUp).GetSafeNormal();
	const FVector WheelRight = FVector::CrossProduct(TireUp, WheelForward).GetSafeNormal();

	float VLat = FVector::DotProduct(PointVel, WheelRight);
	float VLong = FVector::DotProduct(PointVel, WheelForward);
	const bool bNoDriveOrBrakeInput = FMath::Abs(DriveForce) < 1.0f && FMath::Abs(BrakeForce) < 1.0f;

	if (bUseRestStabilization && FMath::Abs(VLat) < RestSpeedThreshold && FMath::Abs(VLong) < RestSpeedThreshold)
	{
		if (FMath::Abs(VLat) < 1.0f)
		{
			VLat = 0.0f;
		}
		if (FMath::Abs(VLong) < 1.0f)
		{
			VLong = 0.0f;
		}
	}

	SlipAngleLat = FMath::Atan2(VLat, FMath::Max(FMath::Abs(VLong), 1.0f));
	const float WheelSurfaceSpeed = WheelAngularVelocity * WheelRadius;
	SlipRatioLong = (FMath::Abs(VLong) > 1.0f) ? (WheelSurfaceSpeed - VLong) / FMath::Abs(VLong) : 0.0f;

	CachedSlipAngle = FMath::FInterpTo(CachedSlipAngle, SlipAngleLat, DeltaTime, SlipRelaxationSpeed);
	CachedSlipRatio = FMath::FInterpTo(CachedSlipRatio, SlipRatioLong, DeltaTime, SlipRelaxationSpeed);

	const float AbsLongSpeed = FMath::Abs(VLong);
	const float LateralFade = FMath::GetMappedRangeValueClamped(
		FVector2D(LowSpeedLateralFadeStart, LowSpeedLateralFadeEnd),
		FVector2D(0.0f, 1.0f),
		AbsLongSpeed);
	const float LongitudinalFade = FMath::GetMappedRangeValueClamped(
		FVector2D(LowSpeedLateralFadeStart, LowSpeedLateralFadeEnd),
		FVector2D(0.0f, 1.0f),
		AbsLongSpeed);

	const float LoadScale = FMath::Clamp(1.0f - LoadSensitivity + LoadSensitivity * FMath::Sqrt(FMath::Max(CachedNormalLoad, 0.0f) / 4000.0f), 0.6f, 1.4f);
	const float Mu = TireFriction * LoadScale;
	const float MaxTireForce = Mu * CachedNormalLoad;

	const float PeakSlipAngle = FMath::DegreesToRadians(FMath::Max(0.5f, SlipAnglePeakDeg));
	const float NormSlipAngle = FMath::Clamp(CachedSlipAngle / PeakSlipAngle, -3.0f, 3.0f);
	float TargetFLatScalar = -std::tanh(NormSlipAngle * CorneringStiffness) * MaxTireForce * LateralFrictionScale * LateralFade;

	const float PeakSlipRatio = FMath::Max(0.02f, SlipRatioPeak);
	const float NormSlipRatio = FMath::Clamp(CachedSlipRatio / PeakSlipRatio, -3.0f, 3.0f);
	float TireLongFromSlip = std::tanh(NormSlipRatio * LongitudinalStiffness) * MaxTireForce * LongitudinalFade;

	if (bNoDriveOrBrakeInput)
	{
		// Без входа от водителя не используем продольную slip-тягу: только сопротивление/демпфирование.
		TireLongFromSlip = 0.0f;
		// И отключаем нелинейную боковую "тягу" от кэшированного slip, оставляя только демпфирование боковой скорости.
		TargetFLatScalar = 0.0f;
	}

	if (bUseRestStabilization && AbsLongSpeed < RestSpeedThreshold && bNoDriveOrBrakeInput)
	{
		// На малой скорости без входа не даем продольной "пружине" шины раскачивать кузов назад/вперед.
		TireLongFromSlip = 0.0f;
		CachedSlipRatio = FMath::FInterpTo(CachedSlipRatio, 0.0f, DeltaTime, SlipRelaxationSpeed);
	}

	float FLongScalar = (DriveForce - BrakeForce) + TireLongFromSlip;

	if (BrakeForce > 1.0f)
	{
		const float HandbrakeAlpha = FMath::Clamp(BrakeForce / 8000.0f, 0.0f, 1.0f);
		const float ExtraLongDrag = -VLong * RestVelocityDamping * HandbrakeLongitudinalDragMultiplier * HandbrakeAlpha;
		const float MaxExtraDrag = MaxTireForce * FMath::Lerp(0.6f, 1.2f, HandbrakeAlpha);
		FLongScalar += FMath::Clamp(ExtraLongDrag, -MaxExtraDrag, MaxExtraDrag);

		if (FMath::Abs(VLong) < RestSpeedThreshold * 2.0f && FMath::Abs(DriveForce) < 1.0f)
		{
			WheelAngularVelocity = FMath::FInterpTo(WheelAngularVelocity, 0.0f, DeltaTime, HandbrakeWheelStopInterp);
		}
	}

	// Пассивное сопротивление качению/боковому скольжению, чтобы кузов не ехал и не вращался как по льду.
	const float RollingResistanceForce = -VLong * RollingResistanceCoeff;
	const float SideSlipDampingForce = -VLat * SideSlipDampingCoeff;
	FLongScalar += FMath::Clamp(RollingResistanceForce, -MaxTireForce * 0.75f, MaxTireForce * 0.75f);
	TargetFLatScalar += FMath::Clamp(SideSlipDampingForce, -MaxTireForce * 0.75f, MaxTireForce * 0.75f);

	if (bUseRestStabilization && FMath::Abs(VLong) < RestSpeedThreshold)
	{
		TargetFLatScalar *= RestLateralGripMultiplier;
		if (bNoDriveOrBrakeInput)
		{
			TargetFLatScalar += -VLat * RestLateralDamping;
		}
	}

	TargetFLatScalar = FMath::Clamp(TargetFLatScalar, -MaxTireForce, MaxTireForce);

	const float MaxDeltaLat = LateralForceRateLimit * DeltaTime;
	const float RateLimitedLat = CachedLateralForceScalar + FMath::Clamp(TargetFLatScalar - CachedLateralForceScalar, -MaxDeltaLat, MaxDeltaLat);
	float FLatScalar = FMath::FInterpTo(CachedLateralForceScalar, RateLimitedLat, DeltaTime, LateralForceSmoothing);

	if (bNoDriveOrBrakeInput)
	{
		// Не допускаем смену знака сил демпфирования: сила должна только гасить скорость, а не разгонять в обратную сторону.
		if (FMath::Abs(VLat) < 1.0f || FMath::Sign(FLatScalar) == FMath::Sign(VLat))
		{
			FLatScalar = 0.0f;
		}
		if (FMath::Abs(VLong) < 1.0f || FMath::Sign(FLongScalar) == FMath::Sign(VLong))
		{
			FLongScalar = 0.0f;
		}

		if (FMath::Abs(VLong) < RestSpeedThreshold && FMath::Abs(VLat) < RestSpeedThreshold)
		{
			CachedLateralForceScalar = FMath::FInterpTo(CachedLateralForceScalar, 0.0f, DeltaTime, LateralForceSmoothing * 2.0f);
		}
	}

	CachedLateralForceScalar = FLatScalar;

	if (bUseRestStabilization && FMath::Abs(VLong) < RestSpeedThreshold && FMath::Abs(WheelAngularVelocity) < RestAngularSpeedThreshold && bNoDriveOrBrakeInput)
	{
		FLongScalar += -VLong * RestVelocityDamping;
	}

	const float FrictionCircle = MaxTireForce;
	const float MaxLongFromCircle = FMath::Sqrt(FMath::Max(0.0f, FrictionCircle * FrictionCircle - FLatScalar * FLatScalar));
	FLongScalar = FMath::Clamp(FLongScalar, -MaxLongFromCircle, MaxLongFromCircle);

	const float MaxLatFromCircle = FMath::Sqrt(FMath::Max(0.0f, FrictionCircle * FrictionCircle - FLongScalar * FLongScalar));
	FLatScalar = FMath::Clamp(FLatScalar, -MaxLatFromCircle, MaxLatFromCircle);

	if (bEnableStaticFrictionLock)
	{
		const float SpeedSq = VLong * VLong + VLat * VLat;
		const float LockSpeedSq = StaticLockSpeedThreshold * StaticLockSpeedThreshold;
		const bool bNearStatic = SpeedSq < LockSpeedSq;
		const bool bNoInputForLock = bNoDriveOrBrakeInput;

		if (bNearStatic && bNoInputForLock)
		{
			const float MaxStaticLong = FMath::Max(MaxTireForce * 0.95f, KINDA_SMALL_NUMBER);
			const float MaxStaticLat = FMath::Max(MaxTireForce * 0.95f, KINDA_SMALL_NUMBER);

			const float StaticLong = FMath::Clamp(-VLong * StaticLongitudinalDamping, -MaxStaticLong, MaxStaticLong);
			const float StaticLat = FMath::Clamp(-VLat * StaticLateralDamping, -MaxStaticLat, MaxStaticLat);

			FLongScalar = StaticLong;
			FLatScalar = StaticLat;

			if (FMath::Abs(VLong) < 1.0f)
			{
				FLongScalar = 0.0f;
			}
			if (FMath::Abs(VLat) < 1.0f)
			{
				FLatScalar = 0.0f;
			}

			if (FMath::Abs(WheelAngularVelocity) < StaticLockAngularThreshold)
			{
				WheelAngularVelocity = 0.0f;
			}
		}
	}

	Forces.LateralForce = WheelRight * FLatScalar;
	Forces.LongitudinalForce = WheelForward * FLongScalar;
	LastTotalForce = Forces.TotalForce();

	WheelAngularVelocity += (FLongScalar / FMath::Max(WheelRadius, 1.0f)) * DeltaTime;

	if (bNoDriveOrBrakeInput)
	{
		const float WheelCircumferenceForSync = 2.0f * PI * FMath::Max(WheelRadius, 1.0f);
		const float TargetFreeRollingOmega = (VLong / WheelCircumferenceForSync) * 2.0f * PI;
		WheelAngularVelocity = FMath::FInterpTo(WheelAngularVelocity, TargetFreeRollingOmega, DeltaTime, FreeRollingAngularSync);
	}

	if (bUseRestStabilization && FMath::Abs(VLong) < RestSpeedThreshold && FMath::Abs(VLat) < RestSpeedThreshold && bNoDriveOrBrakeInput)
	{
		WheelAngularVelocity = FMath::FInterpTo(WheelAngularVelocity, 0.0f, DeltaTime, 8.0f);
	}

	const float WheelCircumference = 2.0f * PI * FMath::Max(WheelRadius, 1.0f);
	const float WheelTurnsPerSecond = VLong / WheelCircumference;
	const float RollingOmega = WheelTurnsPerSecond * 2.0f * PI;
	const float SlipBlend = FMath::Clamp(1.0f - FMath::Abs(CachedSlipRatio) * 0.35f, 0.25f, 1.0f);
	const float VisualTargetOmega = FMath::Lerp(WheelAngularVelocity, RollingOmega, SlipBlend);
	CachedVisualAngularVelocity = FMath::FInterpTo(CachedVisualAngularVelocity, VisualTargetOmega, DeltaTime, VisualGroundSpinInterp);

	return Forces;
}
