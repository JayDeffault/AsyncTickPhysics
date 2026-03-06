#include "SimplifiedVehiclePhysics.h"

#include "Algo/ForEach.h"

void VehiclePhysics::InitializeFromSetups(const TArray<FWheelSetup>& InSetups)
{
	WheelSetups = InSetups;
	WheelStates.SetNumZeroed(WheelSetups.Num());
}

void TirePhysics::ComputeSlip(const FWheelSetup& Setup, FWheelState& State, float LongitudinalSpeed, float LateralSpeed, float DeltaTime)
{
	const float Epsilon = 1.0f;
	const float WheelSurfaceSpeed = State.WheelAngularSpeed * Setup.Radius;
	float SlipRatio = (WheelSurfaceSpeed - LongitudinalSpeed) / (FMath::Abs(LongitudinalSpeed) + Epsilon);
	float SlipAngle = FMath::Atan2(LateralSpeed, FMath::Abs(LongitudinalSpeed) + Epsilon);

	if (FMath::Abs(LongitudinalSpeed) < Setup.LowSpeedThreshold)
	{
		const float LowSpeedAlpha = FMath::Clamp(FMath::Abs(LongitudinalSpeed) / FMath::Max(Setup.LowSpeedThreshold, 1.0f), 0.0f, 1.0f);
		SlipRatio *= LowSpeedAlpha;
		SlipAngle *= LowSpeedAlpha;
	}

	State.SlipRatio = FMath::FInterpTo(State.SlipRatio, SlipRatio, DeltaTime, Setup.RelaxationRate);
	State.SlipAngle = FMath::FInterpTo(State.SlipAngle, SlipAngle, DeltaTime, Setup.RelaxationRate);
}

FTireForces TirePhysics::ComputeTireForces(const FWheelSetup& Setup, FWheelState& State, float EngineForce, float BrakeForce, float DeltaTime)
{
	FTireForces OutForces;
	if (!State.bIsGrounded || State.NormalLoad <= KINDA_SMALL_NUMBER)
	{
		State.RelaxedLongitudinalForce = FMath::FInterpTo(State.RelaxedLongitudinalForce, 0.0f, DeltaTime, Setup.RelaxationRate);
		State.RelaxedLateralForce = FMath::FInterpTo(State.RelaxedLateralForce, 0.0f, DeltaTime, Setup.RelaxationRate);
		return OutForces;
	}

	const float PeakSlipRatio = FMath::Max(Setup.SlipRatioPeak, 0.02f);
	const float PeakSlipAngleRad = FMath::DegreesToRadians(FMath::Max(Setup.SlipAnglePeakDeg, 0.5f));

	const float NormalizedSlipRatio = FMath::Clamp(State.SlipRatio / PeakSlipRatio, -3.0f, 3.0f);
	const float NormalizedSlipAngle = FMath::Clamp(State.SlipAngle / PeakSlipAngleRad, -3.0f, 3.0f);

	const float LoadScale = FMath::Clamp(
		(1.0f - Setup.LoadSensitivity) + Setup.LoadSensitivity * FMath::Sqrt(FMath::Max(State.NormalLoad, 0.0f) / 4000.0f),
		0.6f,
		1.4f);

	const float Mu = Setup.TireFriction * LoadScale;
	const float FMax = Mu * State.NormalLoad;

	const float SlipBasedFx = FMath::TanH(NormalizedSlipRatio * Setup.LongitudinalStiffness) * FMax;
	const float SlipBasedFy = -FMath::TanH(NormalizedSlipAngle * Setup.CorneringStiffness) * FMax;

	float TargetFx = SlipBasedFx + EngineForce - BrakeForce;
	float TargetFy = SlipBasedFy;

	State.RelaxedLongitudinalForce = FMath::FInterpTo(State.RelaxedLongitudinalForce, TargetFx, DeltaTime, Setup.RelaxationRate);
	State.RelaxedLateralForce = FMath::FInterpTo(State.RelaxedLateralForce, TargetFy, DeltaTime, Setup.RelaxationRate);

	float Fx = State.RelaxedLongitudinalForce;
	float Fy = State.RelaxedLateralForce;

	const float LongLimit = FMath::Max(FMax * Setup.LongitudinalGripScale, KINDA_SMALL_NUMBER);
	const float LatLimit = FMath::Max(FMax * Setup.LateralGripScale, KINDA_SMALL_NUMBER);

	const float EllipseNorm = FMath::Square(Fx / LongLimit) + FMath::Square(Fy / LatLimit);
	if (EllipseNorm > 1.0f)
	{
		const float Scale = 1.0f / FMath::Sqrt(EllipseNorm);
		Fx *= Scale;
		Fy *= Scale;
	}

	if (FMath::Abs(State.SlipRatio) < 0.005f && FMath::Abs(State.SlipAngle) < FMath::DegreesToRadians(0.5f))
	{
		Fx *= 0.25f;
		Fy *= 0.25f;
	}

	OutForces.Longitudinal = Fx;
	OutForces.Lateral = Fy;
	OutForces.Vertical = State.NormalLoad;
	return OutForces;
}

void VehiclePhysics::UpdateWheel(
	int32 WheelIndex,
	const FTransform& BodyTransform,
	const FVector& BodyUp,
	float SteeringAngleDeg,
	float EngineForce,
	float BrakeForce,
	float DeltaTime,
	FSuspensionSweepFunc SweepFunc,
	FVelocityAtPointFunc VelocityAtPointFunc,
	FApplyForceFunc ApplyForceFunc)
{
	if (!WheelSetups.IsValidIndex(WheelIndex) || !WheelStates.IsValidIndex(WheelIndex))
	{
		return;
	}

	const FWheelSetup& Setup = WheelSetups[WheelIndex];
	FWheelState& State = WheelStates[WheelIndex];

	const FVector WheelCenter = BodyTransform.TransformPosition(Setup.LocalPosition);
	const FVector Up = BodyUp.GetSafeNormal();

	const FVector SweepStart = WheelCenter + Up * Setup.SuspensionMaxRaise;
	const FVector SweepEnd = WheelCenter - Up * Setup.SuspensionMaxDrop;

	FHitResult Hit;
	State.bIsGrounded = SweepFunc(SweepStart, SweepEnd, Setup.Radius, Hit) && Hit.bBlockingHit;
	if (!State.bIsGrounded)
	{
		State.ContactPoint = SweepEnd;
		State.ContactNormal = Up;
		State.NormalLoad = 0.0f;
		const FTireForces Forces = TirePhysics::ComputeTireForces(Setup, State, 0.0f, 0.0f, DeltaTime);
		ApplyForces(WheelIndex, BodyTransform.GetUnitAxis(EAxis::X), BodyTransform.GetUnitAxis(EAxis::Y), Up, Forces, ApplyForceFunc);
		return;
	}

	State.ContactPoint = Hit.ImpactPoint;
	State.ContactNormal = Hit.ImpactNormal.GetSafeNormal();

	const float RestLength = Setup.SuspensionMaxDrop + Setup.SuspensionMaxRaise;
	const float SuspensionTravel = FVector::DotProduct(SweepStart - State.ContactPoint, Up);
	const float PreviousCompression = State.Compression;
	State.Compression = FMath::Clamp(SuspensionTravel - Setup.Radius, 0.0f, RestLength);
	State.CompressionRatio = (RestLength > KINDA_SMALL_NUMBER) ? State.Compression / RestLength : 0.0f;
	State.CompressionVelocity = (State.Compression - PreviousCompression) / FMath::Max(DeltaTime, KINDA_SMALL_NUMBER);

	const float SpringForce = Setup.SuspensionStiffness * State.Compression;
	const float DamperForce = Setup.SuspensionDamping * State.CompressionVelocity;
	State.NormalLoad = FMath::Max(0.0f, SpringForce + DamperForce);

	const FQuat SteerRot(Up, FMath::DegreesToRadians(Setup.bIsSteerWheel ? SteeringAngleDeg : 0.0f));
	const FVector WheelForward = SteerRot.RotateVector(BodyTransform.GetUnitAxis(EAxis::X)).GetSafeNormal();
	const FVector WheelRight = FVector::CrossProduct(Up, WheelForward).GetSafeNormal();

	const FVector ContactVelocity = VelocityAtPointFunc(State.ContactPoint);
	const float VLong = FVector::DotProduct(ContactVelocity, WheelForward);
	const float VLat = FVector::DotProduct(ContactVelocity, WheelRight);

	TirePhysics::ComputeSlip(Setup, State, VLong, VLat, DeltaTime);

	const float WheelEngineForce = Setup.bIsDrivenWheel ? EngineForce : 0.0f;
	const FTireForces TireForces = TirePhysics::ComputeTireForces(Setup, State, WheelEngineForce, BrakeForce, DeltaTime);

	ApplyForces(WheelIndex, WheelForward, WheelRight, Up, TireForces, ApplyForceFunc);

	const float InertiaProxy = 0.6f;
	const float AngularAccel = TireForces.Longitudinal / FMath::Max(Setup.Radius * InertiaProxy, 1.0f);
	State.WheelAngularSpeed += AngularAccel * DeltaTime;

	if (FMath::Abs(VLong) < 5.0f && FMath::Abs(WheelEngineForce) < 1.0f && FMath::Abs(BrakeForce) < 1.0f)
	{
		State.WheelAngularSpeed = FMath::FInterpTo(State.WheelAngularSpeed, 0.0f, DeltaTime, 8.0f);
	}
}

void VehiclePhysics::ApplyForces(
	int32 WheelIndex,
	const FVector& WheelForward,
	const FVector& WheelRight,
	const FVector& WheelUp,
	const FTireForces& TireForces,
	FApplyForceFunc ApplyForceFunc)
{
	if (!WheelStates.IsValidIndex(WheelIndex))
	{
		return;
	}

	FWheelState& State = WheelStates[WheelIndex];
	const FVector TotalForce =
		(WheelForward * TireForces.Longitudinal) +
		(WheelRight * TireForces.Lateral) +
		(WheelUp * TireForces.Vertical);

	State.LastAppliedForce = TotalForce;
	if (State.bIsGrounded)
	{
		ApplyForceFunc(TotalForce, State.ContactPoint);
	}
}
