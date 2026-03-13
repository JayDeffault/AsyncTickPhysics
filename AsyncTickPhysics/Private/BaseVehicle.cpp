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

	int32 GroundedWheels = 0;
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
			++GroundedWheels;
			UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, Wheel->ContactPoint, WheelForces.TotalForce());
		}
	}

	if (FMath::Abs(ThrottleInput) > 0.1f && BrakeInput < 0.1f && HandbrakeInput < 0.1f && GroundedWheels > 0)
	{
		const FVector BodyForward = BodyWorldTransform.GetUnitAxis(EAxis::X);
		const float ForwardSpeed = FVector::DotProduct(UAsyncTickFunctions::ATP_GetLinearVelocity(BodyMesh), BodyForward);
		if (FMath::Abs(ForwardSpeed) < LowSpeedLaunchAssistMaxSpeed)
		{
			const float LaunchScale = 1.0f - FMath::Clamp(FMath::Abs(ForwardSpeed) / LowSpeedLaunchAssistMaxSpeed, 0.0f, 1.0f);
			const FVector AssistForce = BodyForward * (ThrottleInput * LowSpeedLaunchAssistForce * LaunchScale);
			UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, BodyWorldTransform.GetLocation(), AssistForce);
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
		if (!Left || !Right || !Left->bIsGrounded || !Right->bIsGrounded)
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
	FVector TotalTorque = -AngularVelocity * BaseAngularDamping;

	if (bUseArcadeStabilityAssist)
	{
		const FTransform BodyTransform = UAsyncTickFunctions::ATP_GetTransform(BodyMesh);
		const FVector BodyForward = BodyTransform.GetUnitAxis(EAxis::X);
		const FVector BodyRight = BodyTransform.GetUnitAxis(EAxis::Y);
		const FVector BodyLocation = BodyTransform.GetLocation();
		const FVector V = UAsyncTickFunctions::ATP_GetLinearVelocity(BodyMesh);
		const float VLong = FVector::DotProduct(V, BodyForward);
		const float VLat = FVector::DotProduct(V, BodyRight);
		const float Speed = V.Size();

		const float SpeedBlend = FMath::GetMappedRangeValueClamped(FVector2D(StabilityAssistMinSpeed, StabilityAssistFullSpeed), FVector2D(0.0f, 1.0f), Speed);
		const float SteeringDemand = FMath::Clamp(FMath::Abs(SteeringInput), 0.0f, 1.0f);
		const float SteerRelax = FMath::Lerp(1.0f, FMath::Clamp(1.0f - SteeringStabilityReduction, 0.2f, 1.0f), SteeringDemand);
		const float Assist = SpeedBlend * SteerRelax;

		const FVector StabilizationForce = (-BodyForward * VLong * ArcadeLongitudinalDamping - BodyRight * VLat * ArcadeLateralDamping) * Assist;
		UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, BodyLocation, StabilizationForce.GetClampedToMaxSize(220000.0f));

		const float DesiredYawRate = SteeringInput * FMath::GetMappedRangeValueClamped(FVector2D(0.0f, 2600.0f), FVector2D(0.85f, 0.25f), FMath::Abs(VLong));
		const float YawRateError = DesiredYawRate - AngularVelocity.Z;
		const float YawControlTorque = FMath::Clamp(YawRateError * YawControlGain * Assist, -MaxYawControlTorque, MaxYawControlTorque);
		TotalTorque += FVector(0.0f, 0.0f, YawControlTorque);

		TotalTorque += FVector(0.0f, 0.0f, -AngularVelocity.Z * ArcadeYawDamping * Assist);
		TotalTorque += -AngularVelocity * ArcadeAngularDamping * Assist;

		if (bEnableStandstillLock)
		{
			const bool bNoDriverInput = FMath::Abs(ThrottleInput) < 0.05f && FMath::Abs(SteeringInput) < 0.05f && BrakeInput < 0.05f;
			const bool bParkingMode = bNoDriverInput || (HandbrakeInput > 0.2f && FMath::Abs(ThrottleInput) < 0.1f && FMath::Abs(SteeringInput) < 0.2f);
			if (bParkingMode && Speed < StandstillLinearSpeedThreshold)
			{
				UAsyncTickFunctions::ATP_AddForceAtPosition(BodyMesh, BodyLocation, (-V * StandstillLinearDamping).GetClampedToMaxSize(180000.0f));
				TotalTorque += (-AngularVelocity * StandstillAngularDamping).GetClampedToMaxSize(220000.0f);
			}
		}
	}

	UAsyncTickFunctions::ATP_AddTorque(BodyMesh, TotalTorque, false);
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
	BrakeInput = (FMath::Abs(Value) < 0.05f) ? 0.0f : FMath::Clamp(Value, 0.0f, 1.0f);
}

void ABaseVehicle::SetHandbrake(float Value)
{
	HandbrakeInput = (FMath::Abs(Value) < 0.05f) ? 0.0f : FMath::Clamp(Value, 0.0f, 1.0f);
}

void ABaseVehicle::SetSteering(float Value)
{
	const float Clamped = FMath::Clamp(Value, -1.0f, 1.0f);
	SteeringInput = bInvertSteeringInput ? -Clamped : Clamped;
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
