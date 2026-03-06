#include "BaseVehicle.h"

#include "AsyncTickFunctions.h"
#include "Components/InputComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "WheelComponent.h"

ABaseVehicle::ABaseVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bStartWithTickEnabled = true;

	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	SetRootComponent(BodyMesh);
	BodyMesh->SetSimulatePhysics(true);
	BodyMesh->SetCollisionProfileName(TEXT("PhysicsActor"));

	EngineComponent = CreateDefaultSubobject<UEngineComponent>(TEXT("EngineComponent"));

	UWheelComponent* FrontLeft = CreateDefaultSubobject<UWheelComponent>(TEXT("Wheel_FrontLeft"));
	FrontLeft->SetupAttachment(BodyMesh);
	FrontLeft->SetRelativeLocation(FVector(120.0f, -75.0f, -40.0f));
	FrontLeft->bIsSteerWheel = true;
	FrontLeft->bIsDrivenWheel = true;

	UWheelComponent* FrontRight = CreateDefaultSubobject<UWheelComponent>(TEXT("Wheel_FrontRight"));
	FrontRight->SetupAttachment(BodyMesh);
	FrontRight->SetRelativeLocation(FVector(120.0f, 75.0f, -40.0f));
	FrontRight->bIsSteerWheel = true;
	FrontRight->bIsDrivenWheel = true;

	UWheelComponent* RearLeft = CreateDefaultSubobject<UWheelComponent>(TEXT("Wheel_RearLeft"));
	RearLeft->SetupAttachment(BodyMesh);
	RearLeft->SetRelativeLocation(FVector(-120.0f, -75.0f, -40.0f));
	RearLeft->bIsSteerWheel = false;
	RearLeft->bIsDrivenWheel = true;

	UWheelComponent* RearRight = CreateDefaultSubobject<UWheelComponent>(TEXT("Wheel_RearRight"));
	RearRight->SetupAttachment(BodyMesh);
	RearRight->SetRelativeLocation(FVector(-120.0f, 75.0f, -40.0f));
	RearRight->bIsSteerWheel = false;
	RearRight->bIsDrivenWheel = true;

	Wheels = {FrontLeft, FrontRight, RearLeft, RearRight};
}

void ABaseVehicle::BeginPlay()
{
	Super::BeginPlay();
	Health = MaxHealth;
	SetActorTickEnabled(true);

	if (BodyMesh)
	{
		BodyMesh->OnComponentHit.AddDynamic(this, &ABaseVehicle::OnBodyHit);
	}
}


void ABaseVehicle::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (BodyMesh)
	{
		const FTransform BodyWorldTransform = BodyMesh->GetComponentTransform();
		for (UWheelComponent* Wheel : Wheels)
		{
			if (!Wheel)
			{
				continue;
			}

			const float SteerAngle = Wheel->bIsSteerWheel ? CurrentSteerAngle : 0.0f;
			Wheel->UpdateVisualFromTick(DeltaTime, SteerAngle, BodyWorldTransform);
		}
	}

	DrawVehicleDebug(DeltaTime);
}

void ABaseVehicle::NativeAsyncTick(float DeltaTime)
{
	Super::NativeAsyncTick(DeltaTime);
	SimulateVehicle(DeltaTime);
}

void ABaseVehicle::SimulateVehicle(float DeltaTime)
{
	if (!BodyMesh || !EngineComponent)
	{
		return;
	}

	CurrentSteerAngle = FMath::FInterpTo(CurrentSteerAngle, SteeringInput * MaxSteerAngleDeg, DeltaTime, SteerSpeed);

	TMap<UWheelComponent*, float> DriveForces;
	TMap<UWheelComponent*, float> BrakeForces;
	EngineComponent->SetThrottle(ThrottleInput);
	EngineComponent->SetBrake(BrakeInput);
	EngineComponent->SetHandbrake(HandbrakeInput);
	EngineComponent->Simulate(DeltaTime, Wheels, DriveForces, BrakeForces);

	const FTransform BodyWorldTransform = UAsyncTickFunctions::ATP_GetTransform(BodyMesh);

	for (UWheelComponent* Wheel : Wheels)
	{
		if (!Wheel)
		{
			continue;
		}

		const FTransform WheelWorldTransform = Wheel->GetRelativeTransform() * BodyWorldTransform;
		Wheel->bDebugVehicle = bDebugVehicle;
		const float DriveForce = DriveForces.FindRef(Wheel);
		const float BrakeForce = BrakeForces.FindRef(Wheel);
		const float SteerAngle = Wheel->bIsSteerWheel ? CurrentSteerAngle : 0.0f;

		const FWheelForces WheelForces = Wheel->SimulateWheel(DeltaTime, BodyMesh, DriveForce, BrakeForce, SteerAngle, WheelWorldTransform);
		if (Wheel->bIsGrounded)
		{
			UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, Wheel->ContactPoint, WheelForces.TotalForce());
		}
	}

	ApplyAntiRollBar();
}

void ABaseVehicle::ApplyAntiRollBar()
{
	if (!BodyMesh || Wheels.Num() < 4)
	{
		return;
	}

	auto ApplyPair = [this](UWheelComponent* Left, UWheelComponent* Right)
	{
		if (!Left || !Right)
		{
			return;
		}

		if (!Left->bIsGrounded || !Right->bIsGrounded)
		{
			return;
		}

		const float RollDelta = Left->CompressionRatio - Right->CompressionRatio;
		if (FMath::Abs(RollDelta) < 0.01f)
		{
			return;
		}
		const FVector BodyUp = UAsyncTickFunctions::ATP_GetTransform(BodyMesh).GetUnitAxis(EAxis::Z);
		const FVector LeftForce = BodyUp * (-RollDelta * AntiRollBarStiffness);
		const FVector RightForce = BodyUp * (RollDelta * AntiRollBarStiffness);

		UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, Left->ContactPoint, LeftForce);
		UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, Right->ContactPoint, RightForce);
	};

	ApplyPair(Wheels[0], Wheels[1]);
	ApplyPair(Wheels[2], Wheels[3]);

	const FVector AngularVelocity = UAsyncTickFunctions::ATP_GetAngularVelocity(BodyMesh);
	FVector TotalAngularDamping = -AngularVelocity * 15.0f;

	if (bUseArcadeStabilityAssist)
	{
		const FTransform BodyTransform = UAsyncTickFunctions::ATP_GetTransform(BodyMesh);
		const FVector BodyForward = BodyTransform.GetUnitAxis(EAxis::X);
		const FVector BodyRight = BodyTransform.GetUnitAxis(EAxis::Y);
		const FVector BodyLocation = BodyTransform.GetLocation();

		const FVector LinearVelocity = UAsyncTickFunctions::ATP_GetLinearVelocity(BodyMesh);
		const float VLong = FVector::DotProduct(LinearVelocity, BodyForward);
		const float VLat = FVector::DotProduct(LinearVelocity, BodyRight);

		FVector StabilizationForce = (-BodyForward * VLong * ArcadeLongitudinalDamping) + (-BodyRight * VLat * ArcadeLateralDamping);
		const float MaxStabilizationForce = 280000.0f;
		StabilizationForce = StabilizationForce.GetClampedToMaxSize(MaxStabilizationForce);
		UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, BodyLocation, StabilizationForce);

		const FVector YawOnlyAngularVelocity(0.0f, 0.0f, AngularVelocity.Z);
		TotalAngularDamping += -YawOnlyAngularVelocity * ArcadeYawDamping;
		TotalAngularDamping += -AngularVelocity * ArcadeAngularDamping;
	}

	UAsyncTickFunctions::ATP_AddTorque(BodyMesh, TotalAngularDamping, false);
}


void ABaseVehicle::DrawVehicleDebug(float DeltaTime)
{
	if (!bDebugVehicle)
	{
		return;
	}

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	for (const UWheelComponent* Wheel : Wheels)
	{
		if (!Wheel)
		{
			continue;
		}

		DrawDebugPoint(World, Wheel->LastWheelWorldLocation, 8.0f, FColor::White, false, DeltaTime, 0);

		const FColor SweepColor = Wheel->bLastSweepHadBlockingHit ? FColor::Green : FColor::Red;
		DrawDebugLine(World, Wheel->LastSweepStart, Wheel->LastSweepEnd, SweepColor, false, DeltaTime, 0, 1.5f);
		DrawDebugLine(World, Wheel->LastWheelWorldLocation, Wheel->LastSweepEnd, FColor::Purple, false, DeltaTime, 0, 1.0f);

		if (Wheel->bIsGrounded)
		{
			DrawDebugPoint(World, Wheel->ContactPoint, 12.0f, FColor::Yellow, false, DeltaTime, 0);
			DrawDebugLine(World, Wheel->ContactPoint, Wheel->ContactPoint + Wheel->ContactNormal * 35.0f, FColor::Cyan, false, DeltaTime, 0, 1.0f);
			DrawDebugLine(World, Wheel->LastWheelWorldLocation, Wheel->ContactPoint, FColor::Orange, false, DeltaTime, 0, 1.0f);
			DrawDebugLine(World, Wheel->ContactPoint, Wheel->ContactPoint + Wheel->LastTotalForce * 0.001f, FColor::Blue, false, DeltaTime, 0, 1.5f);
		}
		else
		{
			DrawDebugPoint(World, Wheel->LastSweepEnd, 10.0f, FColor::Red, false, DeltaTime, 0);
		}
	}
}

void ABaseVehicle::SetThrottle(float Value)
{
	ThrottleInput = FMath::Clamp(Value, -1.0f, 1.0f);
}

void ABaseVehicle::SetBrake(float Value)
{
	BrakeInput = FMath::Clamp(Value, 0.0f, 1.0f);
}

void ABaseVehicle::SetHandbrake(float Value)
{
	HandbrakeInput = FMath::Clamp(Value, 0.0f, 1.0f);
}

void ABaseVehicle::SetSteering(float Value)
{
	SteeringInput = FMath::Clamp(Value, -1.0f, 1.0f);
}

void ABaseVehicle::ApplyCollisionDamage(float ImpactStrength)
{
	Health -= ImpactStrength * CollisionDamageMult;
	if (Health <= 0.0f)
	{
		DestroyVehicle();
	}
}

void ABaseVehicle::ApplyWeaponDamage(float Damage)
{
	Health -= Damage * WeaponDamageMult;
	if (Health <= 0.0f)
	{
		DestroyVehicle();
	}
}

void ABaseVehicle::DestroyVehicle()
{
	Destroy();
}

void ABaseVehicle::OnBodyHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit)
{
	ApplyCollisionDamage(NormalImpulse.Size());
}

void ABaseVehicle::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	if (!PlayerInputComponent)
	{
		return;
	}

	PlayerInputComponent->BindAxis(TEXT("Throttle"), this, &ABaseVehicle::SetThrottle);
	PlayerInputComponent->BindAxis(TEXT("Brake"), this, &ABaseVehicle::SetBrake);
	PlayerInputComponent->BindAxis(TEXT("Steering"), this, &ABaseVehicle::SetSteering);
	PlayerInputComponent->BindAxis(TEXT("Handbrake"), this, &ABaseVehicle::SetHandbrake);
}
