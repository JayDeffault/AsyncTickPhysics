#include "WheelComponent.h"

#include "CollisionQueryParams.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"

UWheelComponent::UWheelComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	CurrentSuspensionLength = SuspensionUpperLimit + SuspensionLowerLimit;
}

void UWheelComponent::BeginPlay()
{
	Super::BeginPlay();
	EnsureCollisionMesh();
}


void UWheelComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	if (!bTickWheelInGameThread)
	{
		return;
	}

	UPrimitiveComponent* BodyMesh = nullptr;
	if (AActor* OwnerActor = GetOwner())
	{
		BodyMesh = Cast<UPrimitiveComponent>(OwnerActor->GetRootComponent());
	}

	if (!BodyMesh)
	{
		return;
	}

	const FTransform BodyWorldTransform = BodyMesh->GetComponentTransform();
	const FTransform WheelWorldTransform = GetRelativeTransform() * BodyWorldTransform;
	UpdateContact(DeltaTime, BodyMesh, WheelWorldTransform);

	if (bDebugVehicle)
	{
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

	EnsureCollisionMesh();
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
		LastTotalForce = FVector::ZeroVector;
		return Forces;
	}

	const FVector WheelUp = WheelWorldTransform.GetUnitAxis(EAxis::Z);
	const float RestLength = SuspensionUpperLimit + SuspensionLowerLimit;
	const float Compression = FMath::Clamp(RestLength - CurrentSuspensionLength, 0.0f, RestLength);
	const FVector PointVel = BodyMesh->GetPhysicsLinearVelocityAtPoint(ContactPoint);
	const float VelocityAlongSuspension = FVector::DotProduct(PointVel, WheelUp);

	const float SpringForce = SuspensionStiffness * Compression;
	const float DamperForce = SuspensionDamping * (-VelocityAlongSuspension);
	CachedNormalLoad = FMath::Max(0.0f, SpringForce + DamperForce);

	Forces.SuspensionForce = WheelUp * CachedNormalLoad;

	const FQuat SteerRot = FQuat(WheelUp, FMath::DegreesToRadians(SteeringAngleDeg));
	const FVector WheelForward = SteerRot.RotateVector(WheelWorldTransform.GetUnitAxis(EAxis::X)).GetSafeNormal();
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
	LastTotalForce = Forces.TotalForce();

	WheelAngularVelocity += (FLongScalar / FMath::Max(WheelRadius, 1.0f)) * DeltaTime;

	return Forces;
}
