#include "ArcadeVehiclePawn.h"

#include "Components/InputComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/Engine.h"

AArcadeVehiclePawn::AArcadeVehiclePawn()
{
	PrimaryActorTick.bCanEverTick = true;
	BodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BodyMesh"));
	SetRootComponent(BodyMesh);
	BodyMesh->SetSimulatePhysics(true);
	BodyMesh->SetCollisionProfileName(TEXT("PhysicsActor"));

	WheelSetups.SetNum(4);
	WheelSetups[0].Name = TEXT("FL"); WheelSetups[0].LocalOffset = FVector(125,-78,-38); WheelSetups[0].bSteerWheel = true; WheelSetups[0].bDrivenWheel = true;
	WheelSetups[1].Name = TEXT("FR"); WheelSetups[1].LocalOffset = FVector(125,78,-38); WheelSetups[1].bSteerWheel = true; WheelSetups[1].bDrivenWheel = true;
	WheelSetups[2].Name = TEXT("RL"); WheelSetups[2].LocalOffset = FVector(-125,-78,-38); WheelSetups[2].bDrivenWheel = true; WheelSetups[2].bHandbrakeWheel = true; WheelSetups[2].CorneringStiffness = 6.8f; WheelSetups[2].LongitudinalStiffness = 9.2f;
	WheelSetups[3].Name = TEXT("RR"); WheelSetups[3].LocalOffset = FVector(-125,78,-38); WheelSetups[3].bDrivenWheel = true; WheelSetups[3].bHandbrakeWheel = true; WheelSetups[3].CorneringStiffness = 6.8f; WheelSetups[3].LongitudinalStiffness = 9.2f;
	WheelStates.SetNumZeroed(WheelSetups.Num());
}

void AArcadeVehiclePawn::SetThrottle(float Value) { ThrottleInput = FMath::Clamp(Value, -1.f, 1.f); }
void AArcadeVehiclePawn::SetBrake(float Value) { BrakeInput = FMath::Clamp(Value, 0.f, 1.f); }
void AArcadeVehiclePawn::SetHandbrake(float Value) { HandbrakeInput = FMath::Clamp(Value, 0.f, 1.f); }
void AArcadeVehiclePawn::SetSteering(float Value) { SteeringInput = FMath::Clamp(Value, -1.f, 1.f); }

void AArcadeVehiclePawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	if (!PlayerInputComponent) return;
	PlayerInputComponent->BindAxis(TEXT("Throttle"), this, &AArcadeVehiclePawn::SetThrottle);
	PlayerInputComponent->BindAxis(TEXT("Brake"), this, &AArcadeVehiclePawn::SetBrake);
	PlayerInputComponent->BindAxis(TEXT("Handbrake"), this, &AArcadeVehiclePawn::SetHandbrake);
	PlayerInputComponent->BindAxis(TEXT("Steering"), this, &AArcadeVehiclePawn::SetSteering);
}

float AArcadeVehiclePawn::ComputeTorqueFromCurve() const
{
	const float N = FMath::Clamp((EngineRPM - Engine.IdleRPM) / FMath::Max(Engine.MaxRPM - Engine.IdleRPM, 1.f), 0.f, 1.f);
	if (const FRichCurve* C = Engine.TorqueCurve.GetRichCurveConst(); C && C->GetNumKeys() > 0)
	{
		return C->Eval(N);
	}
	// fallback broad midrange curve
	return Engine.MaxTorqueNm * (1.0f + 0.35f * FMath::Exp(-FMath::Square((N - 0.55f) / 0.22f))) * FMath::Lerp(1.f, 0.72f, FMath::Clamp((N - 0.82f) / 0.18f, 0.f, 1.f));
}

bool AArcadeVehiclePawn::IsDrivenByLayout(int32 WheelIndex) const
{
	if (!WheelSetups.IsValidIndex(WheelIndex)) return false;
	const bool bFront = WheelSetups[WheelIndex].LocalOffset.X > 0.f;
	switch (Transmission.DriveLayout)
	{
	case EArcadeDriveLayout::FWD: return bFront;
	case EArcadeDriveLayout::RWD: return !bFront;
	default: return true;
	}
}

void AArcadeVehiclePawn::UpdateTransmission(float Dt, float DrivenOmega)
{
	if (ShiftTimer > 0.f) ShiftTimer -= Dt;
	if (ShiftTimer <= 0.f && Transmission.bAutomatic)
	{
		if (EngineRPM > Transmission.UpshiftRPM && CurrentGear < Transmission.ForwardGearRatios.Num())
		{
			++CurrentGear; ShiftTimer = Transmission.ShiftTime;
		}
		else if (EngineRPM < Transmission.DownshiftRPM && CurrentGear > 1)
		{
			--CurrentGear; ShiftTimer = Transmission.ShiftTime;
		}
	}
	CurrentGear = FMath::Clamp(CurrentGear, 1, FMath::Max(1, Transmission.ForwardGearRatios.Num()));
	const float GearRatio = Transmission.ForwardGearRatios.IsValidIndex(CurrentGear - 1) ? Transmission.ForwardGearRatios[CurrentGear - 1] : 1.f;
	const float TargetRPM = FMath::Max(Engine.IdleRPM, FMath::Abs(DrivenOmega * GearRatio * Transmission.FinalDrive * 9.5493f));
	EngineRPM = FMath::FInterpTo(EngineRPM, TargetRPM, Dt, 1.f / FMath::Max(Engine.EngineInertia, 0.05f));
	EngineRPM = FMath::Clamp(EngineRPM, Engine.IdleRPM, Engine.MaxRPM);
}

void AArcadeVehiclePawn::UpdateSteering(float Dt, float SpeedCm)
{
	const float SpeedMps = SpeedCm * 0.01f;
	const float Reduction = FMath::Lerp(1.f, 1.f - Handling.HighSpeedSteerReduction, FMath::Clamp(SpeedMps / 55.f, 0.f, 1.f));
	const float Target = SteeringInput * Handling.MaxSteerDeg * Reduction;
	const float Rate = (FMath::Abs(SteeringInput) > 0.01f) ? Handling.SteerSpeed : Handling.SteerReturn;
	CurrentSteerDeg = FMath::FInterpTo(CurrentSteerDeg, Target, Dt, Rate);
}

void AArcadeVehiclePawn::SimulateWheel(int32 WheelIndex, float Dt, float DriveTorquePerWheel)
{
	if (!BodyMesh || !WheelSetups.IsValidIndex(WheelIndex) || !WheelStates.IsValidIndex(WheelIndex)) return;
	const FTransform BodyXf = BodyMesh->GetComponentTransform();
	const FArcadeWheelSetup& Setup = WheelSetups[WheelIndex];
	FArcadeWheelState& State = WheelStates[WheelIndex];

	const FVector WheelHub = BodyXf.TransformPosition(Setup.LocalOffset);
	const FVector BodyUp = BodyXf.GetUnitAxis(EAxis::Z);
	const FVector Start = WheelHub + BodyUp * Setup.MinLengthCm;
	const FVector End = WheelHub - BodyUp * Setup.MaxLengthCm;

	FCollisionQueryParams Params(SCENE_QUERY_STAT(ArcadeWheelTrace), false, this);
	FHitResult Hit;
	State.bGrounded = GetWorld()->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility, Params);
	State.Hit = Hit;
	State.SteerAngleDeg = Setup.bSteerWheel ? CurrentSteerDeg : 0.f;
	if (!State.bGrounded)
	{
		State.AngularVelocity = FMath::FInterpTo(State.AngularVelocity, 0.f, Dt, 0.35f);
		State.Fx = State.Fy = State.NormalLoad = 0.f;
		return;
	}

	State.SuspensionLengthCm = Hit.Distance;
	const float Compression = FMath::Clamp(Setup.RestLengthCm - State.SuspensionLengthCm, 0.f, Setup.RestLengthCm);
	const FVector ContactV = BodyMesh->GetPhysicsLinearVelocityAtPoint(Hit.ImpactPoint);
	const float SuspV = FVector::DotProduct(ContactV, BodyUp);
	State.SpringForce = Compression * Setup.SpringRate;
	State.DamperForce = -SuspV * (SuspV < 0.f ? Setup.DamperCompression : Setup.DamperRebound);
	State.NormalLoad = FMath::Clamp(State.SpringForce + State.DamperForce, 0.f, 1.8f * BodyMesh->GetMass() * 980.f);

	const FVector TireUp = Hit.ImpactNormal.GetSafeNormal();
	const FQuat SteerQ(TireUp, FMath::DegreesToRadians(State.SteerAngleDeg));
	FVector Forward = SteerQ.RotateVector(BodyXf.GetUnitAxis(EAxis::X));
	Forward = FVector::VectorPlaneProject(Forward, TireUp).GetSafeNormal();
	const FVector Right = FVector::CrossProduct(TireUp, Forward).GetSafeNormal();
	const float VLong = FVector::DotProduct(ContactV, Forward);
	const float VLat = FVector::DotProduct(ContactV, Right);

	const float Circ = 2.f * PI * FMath::Max(Setup.RadiusCm, 1.f);
	const float WheelLinear = (State.AngularVelocity / (2.f * PI)) * Circ;
	State.SlipRatio = FMath::Clamp((WheelLinear - VLong) / (FMath::Abs(VLong) + 5.f), -2.f, 2.f);
	State.SlipAngle = FMath::Clamp(FMath::Atan2(VLat, FMath::Abs(VLong) + 5.f), -1.2f, 1.2f);

	const float Mu = Setup.FrictionScale;
	const float FxMax = Mu * State.NormalLoad;
	float FyMax = Mu * State.NormalLoad;
	if (Setup.bHandbrakeWheel && HandbrakeInput > 0.01f)
	{
		FyMax *= FMath::Lerp(1.f, Handling.DriftRearGripScale, HandbrakeInput);
	}

	float Fx = std::tanh(State.SlipRatio * Setup.LongitudinalStiffness) * FxMax;
	float Fy = -std::tanh(State.SlipAngle * Setup.CorneringStiffness) * FyMax;

	float BrakeTorque = BrakeInput * Handling.BrakeTorque;
	if (Setup.bHandbrakeWheel) BrakeTorque += HandbrakeInput * Handling.HandbrakeTorque;
	float DriveTorque = (IsDrivenByLayout(WheelIndex) && Setup.bDrivenWheel) ? DriveTorquePerWheel : 0.f;

	if (FMath::Abs(ThrottleInput) > 0.1f && FMath::Abs(State.SlipRatio) > 0.25f)
	{
		DriveTorque *= FMath::Clamp(1.f - Handling.TractionControl, 0.4f, 1.f);
	}
	if (BrakeInput > 0.1f && FMath::Abs(State.SlipRatio) > 0.35f)
	{
		BrakeTorque *= FMath::Clamp(1.f - Handling.ABSAssist, 0.5f, 1.f);
	}

	const float RadiusM = FMath::Max(Setup.RadiusCm * 0.01f, 0.05f);
	Fx += DriveTorque / RadiusM;
	Fx -= FMath::Sign(VLong) * (BrakeTorque / RadiusM);

	const float Ellipse = FMath::Square(Fx / FMath::Max(FxMax, 1.f)) + FMath::Square(Fy / FMath::Max(FyMax, 1.f));
	if (Ellipse > 1.f)
	{
		const float Scale = 1.f / FMath::Sqrt(Ellipse);
		Fx *= Scale; Fy *= Scale;
	}
	if (FMath::Abs(VLong) < Handling.LowSpeedJitterThreshold && FMath::Abs(VLat) < Handling.LowSpeedJitterThreshold && FMath::Abs(ThrottleInput) < 0.05f)
	{
		Fx = Fy = 0.f;
	}

	State.Fx = FMath::FInterpTo(State.Fx, Fx, Dt, 16.f);
	State.Fy = FMath::FInterpTo(State.Fy, Fy, Dt, 16.f);

	const FVector Force = Forward * State.Fx + Right * State.Fy + TireUp * State.NormalLoad;
	BodyMesh->AddForceAtLocation(Force, Hit.ImpactPoint);

	const float TireTorque = -State.Fx * RadiusM;
	const float NetTorque = DriveTorque - BrakeTorque * FMath::Sign(State.AngularVelocity) + TireTorque - Engine.EngineBraking * State.AngularVelocity;
	State.AngularVelocity += (NetTorque / 1.15f) * Dt;
	State.WheelRPM = State.AngularVelocity * 9.5493f;
}

void AArcadeVehiclePawn::TickVehicle(float Dt)
{
	if (!BodyMesh || WheelSetups.Num() == 0) return;
	SmoothedThrottle = FMath::FInterpTo(SmoothedThrottle, ThrottleInput, Dt, Engine.ThrottleResponse);

	int32 DrivenCount = 0;
	float OmegaSum = 0.f;
	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		if (IsDrivenByLayout(i) && WheelSetups[i].bDrivenWheel)
		{
			++DrivenCount;
			OmegaSum += WheelStates.IsValidIndex(i) ? WheelStates[i].AngularVelocity : 0.f;
		}
	}
	DrivenCount = FMath::Max(DrivenCount, 1);
	const float AvgOmega = OmegaSum / DrivenCount;
	UpdateTransmission(Dt, AvgOmega);
	UpdateSteering(Dt, BodyMesh->GetPhysicsLinearVelocity().Size());

	const float GearRatio = Transmission.ForwardGearRatios.IsValidIndex(CurrentGear - 1) ? Transmission.ForwardGearRatios[CurrentGear - 1] : 1.f;
	const float DriveTorquePerWheel = ComputeTorqueFromCurve() * SmoothedThrottle * GearRatio * Transmission.FinalDrive / DrivenCount;

	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		SimulateWheel(i, Dt, DriveTorquePerWheel);
	}

	// Anti-roll
	for (int32 i = 0; i + 1 < WheelStates.Num(); i += 2)
	{
		if (!WheelStates[i].bGrounded || !WheelStates[i + 1].bGrounded) continue;
		const bool bFront = WheelSetups[i].LocalOffset.X > 0.f;
		const float ARB = bFront ? Handling.AntiRollFront : Handling.AntiRollRear;
		const float L = FMath::Clamp(WheelSetups[i].RestLengthCm - WheelStates[i].SuspensionLengthCm, 0.f, WheelSetups[i].RestLengthCm);
		const float R = FMath::Clamp(WheelSetups[i + 1].RestLengthCm - WheelStates[i + 1].SuspensionLengthCm, 0.f, WheelSetups[i + 1].RestLengthCm);
		const float Delta = (L - R) / FMath::Max(WheelSetups[i].RestLengthCm, 1.f);
		const FVector Up = BodyMesh->GetUpVector();
		BodyMesh->AddForceAtLocation(-Up * Delta * ARB, WheelStates[i].Hit.ImpactPoint);
		BodyMesh->AddForceAtLocation(Up * Delta * ARB, WheelStates[i + 1].Hit.ImpactPoint);
	}

	const FVector V = BodyMesh->GetPhysicsLinearVelocity();
	const float Speed = V.Size();
	if (Speed > 1.f)
	{
		const FVector Dir = V / Speed;
		const FVector Drag = -Dir * Handling.Drag * Speed * Speed;
		const FVector Rolling = -Dir * Handling.RollingResistance * Speed;
		const FVector Down = -BodyMesh->GetUpVector() * Handling.Downforce * Speed * Speed;
		BodyMesh->AddForce(Drag + Rolling + Down);
	}

	const FVector Ang = BodyMesh->GetPhysicsAngularVelocityInRadians();
	const float YawT = -Ang.Z * FMath::Abs(Ang.Z) * 28.f;
	BodyMesh->AddTorqueInRadians(FVector(0,0,YawT));
}

void AArcadeVehiclePawn::DrawDebugData(float Dt) const
{
	if (!bDebug || !GetWorld()) return;
	float Y = 20.f;
	for (int32 i = 0; i < WheelStates.Num(); ++i)
	{
		const FArcadeWheelState& S = WheelStates[i];
		if (S.bGrounded)
		{
			DrawDebugLine(GetWorld(), S.Hit.TraceStart, S.Hit.ImpactPoint, FColor::Green, false, Dt, 0, 1.5f);
		}
		GEngine->AddOnScreenDebugMessage((uint64)(100+i), 0.f, FColor::White,
			FString::Printf(TEXT("W%d G:%d Comp:%.1f SlipR:%.2f SlipA:%.2f Fx:%.0f Fy:%.0f"), i, S.bGrounded?1:0, WheelSetups[i].RestLengthCm-S.SuspensionLengthCm, S.SlipRatio, S.SlipAngle, S.Fx, S.Fy));
		Y += 12.f;
	}
	GEngine->AddOnScreenDebugMessage(99, 0.f, FColor::Cyan,
		FString::Printf(TEXT("RPM: %.0f Gear:%d Speed: %.1f km/h"), EngineRPM, CurrentGear, BodyMesh ? BodyMesh->GetPhysicsLinearVelocity().Size() * 0.036f : 0.f));
}

void AArcadeVehiclePawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	TickVehicle(DeltaSeconds);
	DrawDebugData(DeltaSeconds);
}
