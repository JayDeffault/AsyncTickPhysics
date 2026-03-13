#include "SimcadeVehiclePhysics.h"

#include <cmath>

namespace Simcade
{

static constexpr float CmToM = 0.01f;
static constexpr float MToCm = 100.0f;
static constexpr float Gravity = 980.0f; // cm/s^2

float TireModel::ApplyRelaxation(float Previous, float Target, float RelaxRate, float DeltaTime) const
{
	const float Alpha = 1.0f - FMath::Exp(-FMath::Max(RelaxRate, 0.01f) * DeltaTime);
	return FMath::Lerp(Previous, Target, Alpha);
}

void TireModel::ComputeTireSlip(const VehicleTuning& Tuning, WheelState& State, float VLong, float VLat, float DeltaTime) const
{
	const float Epsilon = 0.5f;
	const float LowSpeed = FMath::Max(0.1f, Tuning.LowSpeedThresholdMps * MToCm);
	const float WheelLinear = State.WheelOmega * 34.0f;
	float SlipRatio = (WheelLinear - VLong) / (FMath::Abs(VLong) + Epsilon);
	float SlipAngle = FMath::Atan2(VLat, FMath::Abs(VLong) + Epsilon);

	SlipRatio = FMath::Clamp(SlipRatio, -2.5f, 2.5f);
	SlipAngle = FMath::Clamp(SlipAngle, -1.4f, 1.4f);

	if (FMath::Abs(VLong) < LowSpeed)
	{
		const float Fade = FMath::Clamp(FMath::Abs(VLong) / LowSpeed, 0.0f, 1.0f);
		SlipRatio *= Fade;
		SlipAngle *= Fade;
	}

	State.SmoothedSlipRatio = ApplyRelaxation(State.SmoothedSlipRatio, SlipRatio, 14.0f, DeltaTime);
	State.SmoothedSlipAngle = ApplyRelaxation(State.SmoothedSlipAngle, SlipAngle, 14.0f, DeltaTime);
}

TireForceResult TireModel::ComputeTireForces(
	const VehicleTuning& Tuning,
	const WheelSetup& Setup,
	WheelState& State,
	const FVector& WheelForward,
	const FVector& WheelRight,
	const FVector& ContactVelocity,
	float DeltaTime,
	bool bInDrift) const
{
	TireForceResult Result;
	Result.Fz = FMath::Max(0.0f, State.Suspension.NormalLoadN);
	if (!State.Contact.bHasContact || Result.Fz <= KINDA_SMALL_NUMBER)
	{
		return Result;
	}

	const float VLong = FVector::DotProduct(ContactVelocity, WheelForward);
	const float VLat = FVector::DotProduct(ContactVelocity, WheelRight);
	ComputeTireSlip(Tuning, State, VLong, VLat, DeltaTime);

	const bool bFront = Setup.AxleIndex == 0;
	const float CornerStiff = bFront ? Tuning.CorneringStiffnessFront : Tuning.CorneringStiffnessRear;
	const float LongStiff = bFront ? Tuning.LongitudinalStiffnessFront : Tuning.LongitudinalStiffnessRear;

	const float LoadNorm = FMath::Clamp(Result.Fz / (Tuning.Mass * Gravity * 0.25f), 0.15f, 3.0f);
	const float EffectiveMu = Tuning.BaseFriction * FMath::Pow(LoadNorm, Tuning.LoadSensitivity);

	const float SurfMu = State.Contact.Surface.FrictionCoefficient;
	const float MuLong = EffectiveMu * SurfMu * State.Contact.Surface.LongitudinalGripMultiplier;
	const float MuLatBase = EffectiveMu * SurfMu * State.Contact.Surface.LateralGripMultiplier;

	const float SlipRatio = State.SmoothedSlipRatio;
	const float SlipAngle = State.SmoothedSlipAngle;

	const float FxLinear = LongStiff * SlipRatio;
	const float FyLinear = -CornerStiff * SlipAngle;

	const float FxSat = std::tanh(FxLinear) * (MuLong * Result.Fz);
	float PostPeakLat = Tuning.PostPeakLateralGrip;
	if (bInDrift)
	{
		PostPeakLat *= Tuning.DriftGripScale;
	}

	const float FySat = std::tanh(FyLinear) * (MuLatBase * Result.Fz);
	const float FyPostPeak = FySat * FMath::Lerp(1.0f, PostPeakLat, FMath::Clamp((FMath::Abs(SlipAngle) - 0.22f) / 0.35f, 0.0f, 1.0f));

	float FxTarget = FxSat;
	float FyTarget = FyPostPeak;

	const float FxMax = MuLong * Result.Fz;
	const float FyMax = MuLatBase * Result.Fz;
	float EllipseNorm = 0.0f;
	if (FxMax > KINDA_SMALL_NUMBER && FyMax > KINDA_SMALL_NUMBER)
	{
		EllipseNorm = FMath::Square(FxTarget / FxMax) + FMath::Square(FyTarget / FyMax);
	}
	if (EllipseNorm > 1.0f)
	{
		const float Scale = 1.0f / FMath::Sqrt(EllipseNorm);
		FxTarget *= Scale;
		FyTarget *= Scale;
	}

	State.SmoothedFx = ApplyRelaxation(State.SmoothedFx, FxTarget, 18.0f, DeltaTime);
	State.SmoothedFy = ApplyRelaxation(State.SmoothedFy, FyTarget, 18.0f, DeltaTime);

	Result.SlipRatio = SlipRatio;
	Result.SlipAngleRad = SlipAngle;
	Result.Fx = State.SmoothedFx;
	Result.Fy = State.SmoothedFy;
	Result.ForceWorld = WheelForward * Result.Fx + WheelRight * Result.Fy;
	return Result;
}

void SuspensionModel::UpdateSuspensionForWheel(const VehicleTuning& Tuning, const WheelSetup& Setup, WheelState& State, float DeltaTime) const
{
	SuspensionState& S = State.Suspension;
	const float Rest = Tuning.SuspensionRestLength;
	const float MinL = FMath::Max(0.1f, Setup.SuspensionMinLengthCm);
	const float MaxL = FMath::Max(MinL + 0.1f, Setup.SuspensionMaxLengthCm);
	const float Length = FMath::Clamp(State.Contact.HitDistanceCm - Setup.RadiusCm, MinL, MaxL);

	S.PrevLengthCm = S.CurrentLengthCm;
	S.CurrentLengthCm = Length;
	const float Compression = (Rest + Setup.SuspensionPreloadCm) - Length;
	const float TotalTravel = FMath::Max(1.0f, MaxL - MinL);
	S.CompressionRatio = FMath::Clamp((Compression - MinL) / TotalTravel, 0.0f, 1.0f);
	S.CompressionVelocity = (S.CurrentLengthCm - S.PrevLengthCm) / FMath::Max(DeltaTime, 0.001f);

	S.SpringForceN = FMath::Max(0.0f, Compression * Tuning.SuspensionStiffness);
	const bool bCompressing = S.CompressionVelocity < 0.0f;
	const float DamperCoeff = bCompressing ? Tuning.SuspensionDampingCompression : Tuning.SuspensionDampingRebound;
	S.DamperForceN = -S.CompressionVelocity * DamperCoeff;

	S.BumpStopForceN = 0.0f;
	if (Length <= (MinL + 3.0f))
	{
		const float BumpRatio = FMath::Clamp((MinL + 3.0f - Length) / 3.0f, 0.0f, 1.0f);
		S.BumpStopForceN = Tuning.BumpStopStiffness * BumpRatio * BumpRatio;
	}

	S.ReboundStopForceN = 0.0f;
	if (Length >= (MaxL - 2.0f))
	{
		const float ReboundRatio = FMath::Clamp((Length - (MaxL - 2.0f)) / 2.0f, 0.0f, 1.0f);
		S.ReboundStopForceN = -Tuning.BumpStopStiffness * 0.35f * ReboundRatio * ReboundRatio;
	}

	const float Total = S.SpringForceN + S.DamperForceN + S.BumpStopForceN + S.ReboundStopForceN;
	S.NormalLoadN = FMath::Clamp(Total, 0.0f, Tuning.Mass * Gravity * 1.7f);
}

void DrivetrainModel::ApplyDrivetrain(
	const VehicleTuning& Tuning,
	const VehicleInput& Input,
	const TArray<WheelSetup>& Setups,
	TArray<WheelState>& States,
	float VehicleSpeedMps,
	float DeltaTime)
{
	SmoothedThrottle = FMath::FInterpTo(SmoothedThrottle, Input.Throttle, DeltaTime, Tuning.ThrottleResponse);
	const float SpeedFactor = FMath::Clamp(1.0f - (VehicleSpeedMps / FMath::Max(Tuning.MaxSpeedSoftLimit, 1.0f)), 0.2f, 1.0f);
	const float EngineTorqueNm = SmoothedThrottle * Tuning.EngineTorque * SpeedFactor;

	int32 DrivenCount = 0;
	for (const WheelSetup& W : Setups)
	{
		const bool bDriven = (DriveType == EDriveType::AWD) || (DriveType == EDriveType::FWD && W.AxleIndex == 0) || (DriveType == EDriveType::RWD && W.AxleIndex > 0);
		if (bDriven)
		{
			++DrivenCount;
		}
	}

	for (int32 i = 0; i < States.Num(); ++i)
	{
		WheelState& S = States[i];
		const WheelSetup& W = Setups[i];
		const bool bDriven = (DriveType == EDriveType::AWD) || (DriveType == EDriveType::FWD && W.AxleIndex == 0) || (DriveType == EDriveType::RWD && W.AxleIndex > 0);
		const float BaseDrive = (bDriven && DrivenCount > 0) ? (EngineTorqueNm / DrivenCount) : 0.0f;

		const float SlipAssist = FMath::Clamp(1.0f - FMath::Abs(S.SmoothedSlipRatio) * Tuning.TractionAssist, 0.35f, 1.0f);
		S.DriveTorque = BaseDrive * SlipAssist;

		float WheelBrake = Input.Brake * Tuning.BrakePower;
		if (W.bHandbrakeWheel)
		{
			WheelBrake += Input.Handbrake * Tuning.HandbrakePower;
		}

		if (FMath::Abs(Input.Throttle) < 0.05f)
		{
			WheelBrake += Tuning.EngineBrake;
		}

		if (Input.Brake > 0.1f && FMath::Abs(S.SmoothedSlipRatio) > 0.3f)
		{
			WheelBrake *= FMath::Clamp(1.0f - Tuning.AbsAssist * 0.5f, 0.55f, 1.0f);
		}
		S.BrakeTorque = WheelBrake;
	}
}

void VehicleStabilityAssist::ApplyStabilityHelpers(
	const VehicleTuning& Tuning,
	const VehicleInput& Input,
	IVehiclePhysicsBody& Body,
	const TArray<WheelSetup>&,
	TArray<WheelState>& States,
	float DeltaTime) const
{
	const FVector Angular = Body.GetAngularVelocity();
	const float YawRate = Angular.Z;
	const bool bPlayerWantsStraight = FMath::Abs(Input.Steering) < 0.2f;
	if (bPlayerWantsStraight)
	{
		const float Assist = FMath::Clamp(FMath::Abs(YawRate) * Tuning.YawAssist, 0.0f, 1.0f);
		Body.AddTorque(FVector(0.0f, 0.0f, -YawRate * Assist * 18000.0f));
	}

	for (WheelState& S : States)
	{
		if (FMath::Abs(S.Tire.SlipAngleRad) > 0.35f)
		{
			S.SmoothedFy = FMath::FInterpTo(S.SmoothedFy, S.SmoothedFy * Tuning.PostPeakLateralGrip, DeltaTime, 4.0f);
		}
	}
}

void VehiclePhysics::Initialize(const VehicleMassProperties& InMassProps, const VehicleTuning& InTuning, const TArray<WheelSetup>& InWheels)
{
	MassProps = InMassProps;
	Tuning = InTuning;
	WheelSetups = InWheels;
	WheelStates.SetNumZeroed(WheelSetups.Num());

	Axles.Reset();
	TMap<int32, AxleState> AxleMap;
	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		const WheelSetup& W = WheelSetups[i];
		AxleState& A = AxleMap.FindOrAdd(W.AxleIndex);
		if (W.bLeft) { A.LeftWheelIndex = i; } else { A.RightWheelIndex = i; }
		A.AntiRollStiffness = (W.AxleIndex == 0) ? Tuning.AntiRollFront : Tuning.AntiRollRear;
	}
	AxleMap.GenerateValueArray(Axles);

	if (SurfaceMap.Num() == 0)
	{
		SurfaceMap.Add(ESurfaceType::Asphalt, SurfaceData{1.05f, 1.0f, 1.0f, 1.0f, 1.0f});
		SurfaceMap.Add(ESurfaceType::WetAsphalt, SurfaceData{0.78f, 0.86f, 0.90f, 1.1f, 1.1f});
		SurfaceMap.Add(ESurfaceType::Dirt, SurfaceData{0.72f, 0.80f, 0.84f, 1.22f, 1.12f});
		SurfaceMap.Add(ESurfaceType::Gravel, SurfaceData{0.65f, 0.75f, 0.80f, 1.25f, 1.15f});
		SurfaceMap.Add(ESurfaceType::Grass, SurfaceData{0.52f, 0.62f, 0.68f, 1.35f, 1.22f});
		SurfaceMap.Add(ESurfaceType::Sand, SurfaceData{0.46f, 0.58f, 0.63f, 1.55f, 1.25f});
	}
}

void VehiclePhysics::ReadInput(float DeltaTime)
{
	FilteredInput.Throttle = FMath::FInterpTo(FilteredInput.Throttle, RawInput.Throttle, DeltaTime, Tuning.ThrottleResponse);
	FilteredInput.Brake = FMath::FInterpTo(FilteredInput.Brake, RawInput.Brake, DeltaTime, 12.0f);
	FilteredInput.Handbrake = FMath::FInterpTo(FilteredInput.Handbrake, RawInput.Handbrake, DeltaTime, 20.0f);
	FilteredInput.Steering = FMath::FInterpTo(FilteredInput.Steering, RawInput.Steering, DeltaTime, 10.0f);
}

void VehiclePhysics::UpdateSteering(float DeltaTime, const IVehiclePhysicsBody& Body)
{
	const float SpeedMps = Body.GetLinearVelocity().Size() * CmToM;
	const float Reduction = FMath::Lerp(1.0f, 1.0f - Tuning.HighSpeedSteerReduction, FMath::Clamp(SpeedMps / 45.0f, 0.0f, 1.0f));
	const float TargetSteerDeg = FilteredInput.Steering * Tuning.SteeringMaxAngle * Reduction;
	SteerVisual = FMath::FInterpTo(SteerVisual, TargetSteerDeg, DeltaTime, Tuning.SteeringSpeed);

	const float Wheelbase = FMath::Max(MassProps.WheelbaseCm, 1.0f);
	const float Track = FMath::Max(MassProps.TrackFrontCm, 1.0f);
	const float BaseRad = FMath::DegreesToRadians(SteerVisual);
	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		WheelState& S = WheelStates[i];
		const WheelSetup& W = WheelSetups[i];
		if (!W.bSteer)
		{
			S.SteerAngleRad = 0.0f;
			continue;
		}

		const float Sign = (SteerVisual >= 0.0f) ? 1.0f : -1.0f;
		const float AbsBase = FMath::Abs(BaseRad);
		const float TanV = FMath::Tan(AbsBase);
		const float Inner = FMath::Atan2(Wheelbase, (Wheelbase / FMath::Max(TanV, 0.001f)) - Track * 0.5f);
		const float Outer = FMath::Atan2(Wheelbase, (Wheelbase / FMath::Max(TanV, 0.001f)) + Track * 0.5f);
		const bool bInnerWheel = (Sign > 0.0f) ? W.bLeft : !W.bLeft;
		S.SteerAngleRad = Sign * (bInnerWheel ? Inner : Outer);
	}
}

void VehiclePhysics::PerformWheelQueries(IVehiclePhysicsBody& Body, const FWheelQueryCallback& QueryCallback)
{
	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		QueryWheelContact(i, Body, QueryCallback);
	}
}

bool VehiclePhysics::QueryWheelContact(int32 WheelIndex, IVehiclePhysicsBody& Body, const FWheelQueryCallback& QueryCallback)
{
	if (!WheelSetups.IsValidIndex(WheelIndex) || !WheelStates.IsValidIndex(WheelIndex))
	{
		return false;
	}

	WheelState& S = WheelStates[WheelIndex];
	S.Contact = WheelContactData();
	if (!QueryCallback)
	{
		return false;
	}

	const FTransform BodyXf = Body.GetWorldTransform();
	const bool bHit = QueryCallback(QueryMode, WheelSetups[WheelIndex], BodyXf, S.Contact);
	S.Contact.bHasContact = bHit && S.Contact.bHasContact;
	if (const SurfaceData* Found = SurfaceMap.Find(S.Contact.SurfaceType))
	{
		S.Contact.Surface = *Found;
	}
	else
	{
		S.Contact.Surface = SurfaceData();
	}

	return S.Contact.bHasContact;
}

void VehiclePhysics::UpdateSuspension(float DeltaTime)
{
	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		if (!WheelStates[i].Contact.bHasContact)
		{
			WheelStates[i].Suspension.NormalLoadN = 0.0f;
			continue;
		}
		Suspension.UpdateSuspensionForWheel(Tuning, WheelSetups[i], WheelStates[i], DeltaTime);
	}
}

void VehiclePhysics::ComputeLoadTransfer(const IVehiclePhysicsBody& Body, float DeltaTime)
{
	const FVector Vel = Body.GetLinearVelocity();
	const FVector Accel = (Vel - PrevLinearVelocity) / FMath::Max(DeltaTime, 0.001f);
	PrevLinearVelocity = Vel;

	const FTransform Xf = Body.GetWorldTransform();
	const FVector Forward = Xf.GetUnitAxis(EAxis::X);
	const FVector Right = Xf.GetUnitAxis(EAxis::Y);

	const float Ax = FVector::DotProduct(Accel, Forward);
	const float Ay = FVector::DotProduct(Accel, Right);

	const float Mass = Body.GetMassKg();
	const float H = MassProps.CGHeightCm;
	const float L = FMath::Max(50.0f, MassProps.WheelbaseCm);
	const float Tf = FMath::Max(50.0f, MassProps.TrackFrontCm);
	const float Tr = FMath::Max(50.0f, MassProps.TrackRearCm);

	const float LongTransfer = Mass * Ax * (H / L);
	const float LatTransferFront = Mass * Ay * (H / Tf) * 0.5f;
	const float LatTransferRear = Mass * Ay * (H / Tr) * 0.5f;

	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		WheelState& S = WheelStates[i];
		const WheelSetup& W = WheelSetups[i];
		float DeltaLoad = 0.0f;
		DeltaLoad += (W.AxleIndex == 0) ? -LongTransfer * 0.5f : LongTransfer * 0.5f;
		if (W.AxleIndex == 0)
		{
			DeltaLoad += W.bLeft ? -LatTransferFront : LatTransferFront;
		}
		else
		{
			DeltaLoad += W.bLeft ? -LatTransferRear : LatTransferRear;
		}
		S.Suspension.NormalLoadN = FMath::Max(0.0f, S.Suspension.NormalLoadN + DeltaLoad);
	}
}


void VehiclePhysics::ApplyDrivetrain(float DeltaTime, IVehiclePhysicsBody& Body)
{
	const float SpeedMps = Body.GetLinearVelocity().Size() * CmToM;
	Drivetrain.ApplyDrivetrain(Tuning, FilteredInput, WheelSetups, WheelStates, SpeedMps, DeltaTime);
}

void VehiclePhysics::ApplyBraking(float DeltaTime)
{
	for (int32 i = 0; i < WheelStates.Num(); ++i)
	{
		WheelState& S = WheelStates[i];
		const WheelSetup& W = WheelSetups[i];
		if (FilteredInput.Brake > 0.1f && FMath::Abs(S.SmoothedSlipRatio) > 0.3f)
		{
			S.BrakeTorque = FMath::Lerp(S.BrakeTorque, S.BrakeTorque * (1.0f - Tuning.AbsAssist), DeltaTime * 10.0f);
		}
		if (W.bHandbrakeWheel && FilteredInput.Handbrake > 0.05f)
		{
			S.BrakeTorque += FilteredInput.Handbrake * Tuning.HandbrakePower;
		}
	}
}

void VehiclePhysics::ComputeTireForces(float DeltaTime, IVehiclePhysicsBody& Body)
{
	const FTransform Xf = Body.GetWorldTransform();
	const FVector Up = Xf.GetUnitAxis(EAxis::Z);
	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		WheelState& S = WheelStates[i];
		const WheelSetup& W = WheelSetups[i];
		if (!S.Contact.bHasContact)
		{
			S.Tire = TireForceResult();
			continue;
		}

		const FQuat Steer(Up, S.SteerAngleRad);
		const FVector WheelForward = Steer.RotateVector(Xf.GetUnitAxis(EAxis::X)).GetSafeNormal();
		const FVector WheelRight = FVector::CrossProduct(Up, WheelForward).GetSafeNormal();
		const FVector ContactVel = Body.GetLinearVelocityAtPoint(S.Contact.ContactPoint);

		const bool bRear = W.AxleIndex > 0;
		const bool bDrift = bRear && (FilteredInput.Handbrake > 0.1f || (FilteredInput.Throttle > 0.6f && FMath::Abs(S.SmoothedSlipAngle) > 0.25f));

		S.Tire = Tire.ComputeTireForces(Tuning, W, S, WheelForward, WheelRight, ContactVel, DeltaTime, bDrift);

		// Drivetrain and braking as longitudinal tire force requests.
		const float RadiusM = FMath::Max(W.RadiusCm * CmToM, 0.05f);
		float DriveFx = S.DriveTorque / RadiusM;
		float BrakeFx = S.BrakeTorque / RadiusM;

		// GTA-like: handbrake reduces rear lateral grip and makes rotation controllable.
		if (W.bHandbrakeWheel && FilteredInput.Handbrake > 0.01f)
		{
			S.Tire.Fy *= FMath::Lerp(1.0f, 0.38f, FilteredInput.Handbrake);
			DriveFx *= 0.85f;
		}

		const float FxTotal = S.Tire.Fx + DriveFx - FMath::Sign(FVector::DotProduct(ContactVel, WheelForward)) * BrakeFx;
		S.Tire.Fx = FxTotal;
		S.Tire.ForceWorld = WheelForward * S.Tire.Fx + WheelRight * S.Tire.Fy + Up * S.Tire.Fz;
	}
}

void VehiclePhysics::ApplyAntiRollBars(IVehiclePhysicsBody& Body)
{
	for (const AxleState& A : Axles)
	{
		if (!WheelStates.IsValidIndex(A.LeftWheelIndex) || !WheelStates.IsValidIndex(A.RightWheelIndex))
		{
			continue;
		}
		WheelState& L = WheelStates[A.LeftWheelIndex];
		WheelState& R = WheelStates[A.RightWheelIndex];
		if (!L.Contact.bHasContact || !R.Contact.bHasContact)
		{
			continue;
		}

		const float Delta = L.Suspension.CompressionRatio - R.Suspension.CompressionRatio;
		const float Force = Delta * A.AntiRollStiffness;
		Body.AddForceAtPoint(FVector::UpVector * -Force, L.Contact.ContactPoint);
		Body.AddForceAtPoint(FVector::UpVector * Force, R.Contact.ContactPoint);
	}
}

void VehiclePhysics::ApplyAeroDrag(IVehiclePhysicsBody& Body)
{
	ApplyAeroAndRollingResistance(Body);
}

void VehiclePhysics::ApplyStabilityAssist(float DeltaTime, IVehiclePhysicsBody& Body)
{
	StabilityAssist.ApplyStabilityHelpers(Tuning, FilteredInput, Body, WheelSetups, WheelStates, DeltaTime);
}

void VehiclePhysics::ApplyAeroAndRollingResistance(IVehiclePhysicsBody& Body)
{
	const FVector V = Body.GetLinearVelocity();
	const float Speed = V.Size();
	if (Speed <= KINDA_SMALL_NUMBER)
	{
		return;
	}

	const FVector Dir = V / Speed;
	const float DragMag = Tuning.DragCoefficient * Speed * Speed;
	Body.AddForceAtPoint(-Dir * DragMag, Body.GetWorldTransform().GetLocation());

	const float RollingMag = Tuning.RollingResistance * Speed;
	Body.AddForceAtPoint(-Dir * RollingMag, Body.GetWorldTransform().GetLocation());

	const float Downforce = Tuning.DownforceCoefficient * Speed * Speed;
	Body.AddForceAtPoint(-Body.GetWorldTransform().GetUnitAxis(EAxis::Z) * Downforce, Body.GetWorldTransform().GetLocation());
}

void VehiclePhysics::ApplyWheelForces(IVehiclePhysicsBody& Body)
{
	for (const WheelState& S : WheelStates)
	{
		if (!S.Contact.bHasContact)
		{
			continue;
		}
		Body.AddForceAtPoint(S.Tire.ForceWorld, S.Contact.ContactPoint);
	}
}

void VehiclePhysics::UpdateWheelAngularVelocity(float DeltaTime)
{
	for (int32 i = 0; i < WheelSetups.Num(); ++i)
	{
		WheelState& S = WheelStates[i];
		const WheelSetup& W = WheelSetups[i];
		const float RadiusM = FMath::Max(W.RadiusCm * CmToM, 0.05f);
		const float Inertia = FMath::Max(Tuning.WheelInertia, 0.05f);
		const float TireTorque = -S.Tire.Fx * RadiusM;
		const float NetTorque = S.DriveTorque - S.BrakeTorque * FMath::Sign(S.WheelOmega) + TireTorque;
		const float AngularAcc = NetTorque / Inertia;
		S.WheelOmega += AngularAcc * DeltaTime;

		if (FMath::Abs(S.WheelOmega) < 0.1f && FMath::Abs(S.Tire.Fx) < 40.0f)
		{
			S.WheelOmega = 0.0f;
		}
		S.WheelAngle = FMath::Fmod(S.WheelAngle + S.WheelOmega * DeltaTime, TWO_PI);
	}
}

void VehiclePhysics::UpdateVehicle(float DeltaTime, IVehiclePhysicsBody& Body, const FWheelQueryCallback& QueryCallback)
{
	if (DeltaTime <= 0.0f)
	{
		return;
	}

	ReadInput(DeltaTime);
	UpdateSteering(DeltaTime, Body);
	PerformWheelQueries(Body, QueryCallback);
	UpdateSuspension(DeltaTime);
	ComputeLoadTransfer(Body, DeltaTime);
	ApplyDrivetrain(DeltaTime, Body);
	ApplyBraking(DeltaTime);
	ComputeTireForces(DeltaTime, Body);
	ApplyAntiRollBars(Body);
	ApplyAeroDrag(Body);
	ApplyStabilityAssist(DeltaTime, Body);
	ApplyWheelForces(Body);
	UpdateWheelAngularVelocity(DeltaTime);
}

} // namespace Simcade
