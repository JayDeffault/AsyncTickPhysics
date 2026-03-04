#include "WheelComponent.h"

#include "AsyncTickFunctions.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"

UWheelComponent::UWheelComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	CurrentSuspensionLength = SuspensionUpperLimit + SuspensionLowerLimit;
}

void UWheelComponent::BeginPlay()
{
	Super::BeginPlay();
	EnsureCollisionMesh();
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
		WheelCollisionMesh->SetSimulatePhysics(false);
		WheelCollisionMesh->SetGenerateOverlapEvents(false);
		WheelCollisionMesh->RegisterComponent();
	}

	if (WheelCollisionMesh->GetAttachParent() != this)
	{
		WheelCollisionMesh->AttachToComponent(this, FAttachmentTransformRules::SnapToTargetNotIncludingScale);
	}

	WheelCollisionMesh->SetRelativeLocation(FVector::ZeroVector);
	WheelCollisionMesh->SetRelativeRotation(FRotator::ZeroRotator);
}

void UWheelComponent::UpdateContact(float DeltaTime, UPrimitiveComponent* BodyMesh)
{
	bIsGrounded = false;
	ContactPoint = GetComponentLocation() - GetUpVector() * SuspensionLowerLimit;
	ContactNormal = FVector::UpVector;
	CompressionRatio = 0.0f;
	CachedNormalLoad = 0.0f;
	bLastSweepHadBlockingHit = false;

	EnsureCollisionMesh();
	if (!WheelCollisionMesh)
	{
		return;
	}

	const FVector OutStart = GetComponentLocation() + GetUpVector() * SuspensionUpperLimit;
	const FVector OutEnd = GetComponentLocation() - GetUpVector() * SuspensionLowerLimit;
	LastSweepStart = OutStart;
	LastSweepEnd = OutEnd;

	TArray<FHitResult> SweepHits;
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(WheelSweep), false);
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
		WheelCollisionMesh->GetComponentQuat(),
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
	const float SuspensionTravel = FVector::DotProduct(OutStart - ContactPoint, GetUpVector());
	CurrentSuspensionLength = FMath::Max(0.0f, SuspensionTravel - WheelRadius);
	CurrentSuspensionLength = FMath::Clamp(CurrentSuspensionLength, 0.0f, RestLength);

	const float Compression = FMath::Max(0.0f, RestLength - CurrentSuspensionLength);
	CompressionRatio = (RestLength > KINDA_SMALL_NUMBER) ? FMath::Clamp(Compression / RestLength, 0.0f, 1.0f) : 0.0f;

	(void)DeltaTime;
}

FWheelForces UWheelComponent::SimulateWheel(float DeltaTime, UPrimitiveComponent* BodyMesh, float DriveForce, float BrakeForce, float SteeringAngleDeg)
{
	FWheelForces Forces;
	UpdateContact(DeltaTime, BodyMesh);

	if (!bIsGrounded || !BodyMesh)
	{
		return Forces;
	}

	const FVector WheelUp = GetUpVector();
	const float RestLength = SuspensionUpperLimit + SuspensionLowerLimit;
	const float Compression = FMath::Clamp(RestLength - CurrentSuspensionLength, 0.0f, RestLength);
	const FVector PointVel = UAsyncTickFunctions::ATP_GetLinearVelocityAtPoint(BodyMesh, ContactPoint);
	const float VelocityAlongSuspension = FVector::DotProduct(PointVel, WheelUp);

	const float SpringForce = SuspensionStiffness * Compression;
	const float DamperForce = SuspensionDamping * (-VelocityAlongSuspension);
	CachedNormalLoad = FMath::Max(0.0f, SpringForce + DamperForce);

	Forces.SuspensionForce = WheelUp * CachedNormalLoad;

	const FQuat SteerRot = FQuat(WheelUp, FMath::DegreesToRadians(SteeringAngleDeg));
	const FVector WheelForward = SteerRot.RotateVector(GetForwardVector()).GetSafeNormal();
	const FVector WheelRight = FVector::CrossProduct(WheelUp, WheelForward).GetSafeNormal();

	const float VLat = FVector::DotProduct(PointVel, WheelRight);
	const float VLong = FVector::DotProduct(PointVel, WheelForward);

	SlipAngleLat = FMath::Atan2(VLat, FMath::Max(FMath::Abs(VLong), 1.0f));
	const float WheelSurfaceSpeed = WheelAngularVelocity * WheelRadius;
	SlipRatioLong = (FMath::Abs(VLong) > 1.0f) ? (WheelSurfaceSpeed - VLong) / FMath::Abs(VLong) : 0.0f;

	float FLatScalar = -VLat * LateralFrictionScale * TireFriction * CachedNormalLoad;
	float FLongScalar = DriveForce - BrakeForce;

	const float FrictionCircle = TireFriction * CachedNormalLoad;
	const float Combined = FMath::Sqrt(FLatScalar * FLatScalar + FLongScalar * FLongScalar);
	if (Combined > FrictionCircle && Combined > KINDA_SMALL_NUMBER)
	{
		const float Scale = FrictionCircle / Combined;
		FLatScalar *= Scale;
		FLongScalar *= Scale;
	}

	Forces.LateralForce = WheelRight * FLatScalar;
	Forces.LongitudinalForce = WheelForward * FLongScalar;

	WheelAngularVelocity += (FLongScalar / FMath::Max(WheelRadius, 1.0f)) * DeltaTime;

	return Forces;
}
