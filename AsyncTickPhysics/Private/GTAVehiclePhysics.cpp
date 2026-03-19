#include "GTAVehiclePhysics.h"

#include <cmath>

namespace GTAVehicle
{

static constexpr float CmToM = 0.01f;
static constexpr float GravityCm = 980.0f;

void FVehicleSimulator::Initialize(const FVehicleHandling& InHandling, const FPowertrainSetup& InPowertrain, const TArray<FWheelSetup>& InWheels)
{
	Handling = InHandling;
	Powertrain = InPowertrain;
	WheelsSetup = InWheels;
	WheelsState.SetNumZeroed(WheelsSetup.Num());
	EngineRPM = Powertrain.IdleRPM;
	CurrentGear = 1;
}

void FVehicleSimulator::SetInput(const FVehicleInputState& InInput)
{
	Input.Throttle = FMath::Clamp(InInput.Throttle, -1.0f, 1.0f);
	Input.Brake = FMath::Clamp(InInput.Brake, 0.0f, 1.0f);
	Input.Handbrake = FMath::Clamp(InInput.Handbrake, 0.0f, 1.0f);
	Input.Steering = FMath::Clamp(InInput.Steering, -1.0f, 1.0f);
}

bool FVehicleSimulator::IsDrivenWheel(int32 Index) const
{
	if (!WheelsSetup.IsValidIndex(Index))
	{
		return false;
	}

	const FWheelSetup& W = WheelsSetup[Index];
	if (Powertrain.DriveLayout == EDriveLayout::AWD)
	{
		return true;
	}
	const bool bFront = W.LocalOffset.X > 0.0f;
	return (Powertrain.DriveLayout == EDriveLayout::FWD) ? bFront : !bFront;
}

float FVehicleSimulator::EvalEngineTorque(float NormalizedRPM) const
{
	// Простая GTA-like кривая момента: сильный средний диапазон, мягкий спад на верхах.
	const float MidBoost = 1.0f + 0.35f * FMath::Exp(-FMath::Square((NormalizedRPM - 0.52f) / 0.22f));
	const float HighDrop = FMath::Lerp(1.0f, 0.72f, FMath::Clamp((NormalizedRPM - 0.82f) / 0.18f, 0.0f, 1.0f));
	return 460.0f * MidBoost * HighDrop;
}

void FVehicleSimulator::UpdatePowertrain(float DeltaTime)
{
	SmoothedThrottle = FMath::FInterpTo(SmoothedThrottle, Input.Throttle, DeltaTime, Powertrain.ThrottleResponse);

	const float ThrottleSign = FMath::Sign(Input.Throttle);
	if (ThrottleSign < -0.1f)
	{
		CurrentGear = -1;
	}
	else if (ThrottleSign > 0.1f && CurrentGear <= 0)
	{
		CurrentGear = 1;
	}

	if (CurrentGear > 0)
	{
		if (CurrentGear < Powertrain.GearRatios.Num() - 1 && EngineRPM > Powertrain.ShiftUpRPM)
		{
			++CurrentGear;
		}
		if (CurrentGear > 1 && EngineRPM < Powertrain.ShiftDownRPM)
		{
			--CurrentGear;
		}
		CurrentGear = FMath::Clamp(CurrentGear, 1, FMath::Max(1, Powertrain.GearRatios.Num() - 1));
	}
}

void FVehicleSimulator::UpdateSteering(float DeltaTime, const FVector& LinearVelocity)
{
	const float SpeedMps = LinearVelocity.Size() * CmToM;
	const float Reduction = FMath::Lerp(1.0f, 1.0f - Handling.HighSpeedSteerReduction, FMath::Clamp(SpeedMps / 55.0f, 0.0f, 1.0f));
	const float TargetSteer = Input.Steering * Handling.MaxSteerDeg * Reduction;
	const float Interp = (FMath::Abs(Input.Steering) > 0.01f) ? Handling.SteerSpeed : Handling.SteerReturnSpeed;
	CurrentSteer = FMath::FInterpTo(CurrentSteer, TargetSteer, DeltaTime, Interp);

	for (int32 i = 0; i < WheelsSetup.Num(); ++i)
	{
		WheelsState[i].SteerAngleDeg = WheelsSetup[i].bSteering ? CurrentSteer : 0.0f;
	}
}

void FVehicleSimulator::UpdateWheelContacts(const FTransform& BodyTransform, const FWheelQueryFn& QueryFn)
{
	for (int32 i = 0; i < WheelsSetup.Num(); ++i)
	{
		FWheelContact Contact;
		if (QueryFn)
		{
			QueryFn(i, BodyTransform, Contact);
		}
		WheelsState[i].Contact = Contact;
	}
}

void FVehicleSimulator::ComputeSuspensionLoads(float DeltaTime, const FVector& LinearVelocity, const FVector& BodyUp)
{
	const FVector VelDir = LinearVelocity;
	const float Speed = VelDir.Size();

	for (int32 i = 0; i < WheelsSetup.Num(); ++i)
	{
		FWheelState& WS = WheelsState[i];
		const FWheelSetup& Setup = WheelsSetup[i];
		if (!WS.Contact.bHasContact)
		{
			WS.NormalLoad = 0.0f;
			continue;
		}

		const float Rest = Setup.SuspensionRestCm;
		const float Length = FMath::Clamp(WS.Contact.SuspensionLengthCm, Setup.SuspensionMinCm, Setup.SuspensionMaxCm);
		const float Compression = FMath::Clamp(Rest - Length, 0.0f, Rest);
		const float SpringForce = Compression * Setup.SpringRate;
		const float DampingSign = FVector::DotProduct(VelDir, BodyUp);
		const float Damping = (DampingSign < 0.0f ? Setup.DamperCompression : Setup.DamperRebound) * (-DampingSign);

		float Load = SpringForce + Damping;
		if (Length <= Setup.SuspensionMinCm + 2.0f)
		{
			const float BumpAlpha = FMath::Clamp((Setup.SuspensionMinCm + 2.0f - Length) / 2.0f, 0.0f, 1.0f);
			Load += Setup.SpringRate * 2.8f * BumpAlpha * BumpAlpha;
		}

		Load = FMath::Clamp(Load, 0.0f, Handling.MassKg * GravityCm * 1.8f);
		WS.NormalLoad = Load;
	}

	(void)Speed;
	(void)DeltaTime;
}

void FVehicleSimulator::ComputeTireForces(float DeltaTime, IVehiclePhysicsInterface& Body, const FTransform& BodyTransform)
{
	int32 DrivenCount = 0;
	for (int32 i = 0; i < WheelsSetup.Num(); ++i)
	{
		if (IsDrivenWheel(i) && WheelsState[i].Contact.bHasContact)
		{
			++DrivenCount;
		}
	}
	DrivenCount = FMath::Max(DrivenCount, 1);

	float Ratio = 0.0f;
	if (CurrentGear > 0)
	{
		Ratio = Powertrain.GearRatios.IsValidIndex(CurrentGear) ? Powertrain.GearRatios[CurrentGear] : 1.0f;
	}
	else if (CurrentGear < 0)
	{
		Ratio = Powertrain.ReverseGearRatio;
	}
	const float NormalizedRPM = FMath::Clamp((EngineRPM - Powertrain.IdleRPM) / FMath::Max(Powertrain.MaxRPM - Powertrain.IdleRPM, 1.0f), 0.0f, 1.0f);
	const float EngineTorque = EvalEngineTorque(NormalizedRPM);
	const float TotalDriveTorque = EngineTorque * SmoothedThrottle * Ratio * Powertrain.FinalDrive;
	const float DriveTorquePerWheel = TotalDriveTorque / DrivenCount;

	float DrivenOmegaSum = 0.0f;
	int32 DrivenOmegaCount = 0;

	for (int32 i = 0; i < WheelsSetup.Num(); ++i)
	{
		FWheelState& WS = WheelsState[i];
		const FWheelSetup& Setup = WheelsSetup[i];
		WS.TotalForce = FVector::ZeroVector;

		if (!WS.Contact.bHasContact)
		{
			// В воздухе колесо крутится по инерции.
			WS.AngularSpeed = FMath::FInterpTo(WS.AngularSpeed, 0.0f, DeltaTime, 0.35f);
			continue;
		}

		const FVector WheelUp = WS.Contact.Normal.GetSafeNormal();
		const FQuat SteerQ(WheelUp, FMath::DegreesToRadians(WS.SteerAngleDeg));
		FVector WheelForward = SteerQ.RotateVector(BodyTransform.GetUnitAxis(EAxis::X));
		WheelForward = FVector::VectorPlaneProject(WheelForward, WheelUp).GetSafeNormal();
		const FVector WheelRight = FVector::CrossProduct(WheelUp, WheelForward).GetSafeNormal();

		const FVector ContactVel = Body.GetVelocityAtPoint(WS.Contact.Point);
		const float VLong = FVector::DotProduct(ContactVel, WheelForward);
		const float VLat = FVector::DotProduct(ContactVel, WheelRight);

		const float RadiusCm = FMath::Max(Setup.RadiusCm, 1.0f);
		const float Circumference = 2.0f * PI * RadiusCm;
		const float WheelLinear = (WS.AngularSpeed / (2.0f * PI)) * Circumference;
		const float SlipRatio = (WheelLinear - VLong) / (FMath::Abs(VLong) + 5.0f);
		const float SlipAngle = FMath::Atan2(VLat, FMath::Abs(VLong) + 5.0f);

		WS.SlipRatio = FMath::FInterpTo(WS.SlipRatio, FMath::Clamp(SlipRatio, -2.0f, 2.0f), DeltaTime, 12.0f);
		WS.SlipAngle = FMath::FInterpTo(WS.SlipAngle, FMath::Clamp(SlipAngle, -1.2f, 1.2f), DeltaTime, 12.0f);

		const float SurfaceMu = Setup.TireFriction * WS.Contact.Surface.Friction;
		const float FxMax = SurfaceMu * WS.NormalLoad * WS.Contact.Surface.LongitudinalGrip;
		float FyMax = SurfaceMu * WS.NormalLoad * WS.Contact.Surface.LateralGrip;

		if (Input.Handbrake > 0.01f && Setup.bHandbrakeWheel)
		{
			FyMax *= FMath::Lerp(1.0f, Handling.DriftGripScale, Input.Handbrake);
		}

		const float FxRaw = std::tanh(WS.SlipRatio * Setup.LongitudinalStiffness) * FxMax;
		const float FyRaw = -std::tanh(WS.SlipAngle * Setup.CorneringStiffness) * FyMax;

		float BrakeTorque = Input.Brake * Handling.BrakeTorque;
		if (Setup.bHandbrakeWheel)
		{
			BrakeTorque += Input.Handbrake * Handling.HandbrakeTorque;
		}

		float DriveTorque = IsDrivenWheel(i) ? DriveTorquePerWheel : 0.0f;
		if (Handling.bEnableTractionControl && FMath::Abs(Input.Throttle) > 0.1f && FMath::Abs(WS.SlipRatio) > 0.25f)
		{
			DriveTorque *= FMath::Clamp(1.0f - Handling.TractionAssist, 0.4f, 1.0f);
		}

		if (Handling.bEnableABS && Input.Brake > 0.1f && FMath::Abs(WS.SlipRatio) > 0.35f)
		{
			BrakeTorque *= FMath::Clamp(1.0f - Handling.ABSSensitivity, 0.5f, 1.0f);
		}

		const float RadiusM = RadiusCm * CmToM;
		float Fx = FxRaw + (DriveTorque / FMath::Max(RadiusM, 0.05f));
		Fx -= FMath::Sign(VLong) * (BrakeTorque / FMath::Max(RadiusM, 0.05f));
		float Fy = FyRaw;

		const float Ellipse = FMath::Square(Fx / FMath::Max(FxMax, 1.0f)) + FMath::Square(Fy / FMath::Max(FyMax, 1.0f));
		if (Ellipse > 1.0f)
		{
			const float Scale = 1.0f / FMath::Sqrt(Ellipse);
			Fx *= Scale;
			Fy *= Scale;
		}

		if (FMath::Abs(VLong) < Handling.LowSpeedJitterThreshold && FMath::Abs(VLat) < Handling.LowSpeedJitterThreshold && FMath::Abs(Input.Throttle) < 0.05f)
		{
			Fx = 0.0f;
			Fy = 0.0f;
		}

		WS.SmoothedFx = FMath::FInterpTo(WS.SmoothedFx, Fx, DeltaTime, 16.0f);
		WS.SmoothedFy = FMath::FInterpTo(WS.SmoothedFy, Fy, DeltaTime, 16.0f);

		WS.TotalForce = WheelForward * WS.SmoothedFx + WheelRight * WS.SmoothedFy + WheelUp * WS.NormalLoad;
		Body.AddForceAtPoint(WS.TotalForce, WS.Contact.Point);

		const float WheelInertia = 1.15f;
		const float TireTorque = -WS.SmoothedFx * RadiusM;
		const float NetTorque = DriveTorque - BrakeTorque * FMath::Sign(WS.AngularSpeed) + TireTorque - Powertrain.EngineBrakeTorque * WS.AngularSpeed;
		WS.AngularSpeed += (NetTorque / WheelInertia) * DeltaTime;

		if (FMath::Abs(VLong) > 80.0f)
		{
			const float RollingOmega = (VLong / Circumference) * 2.0f * PI;
			WS.AngularSpeed = FMath::FInterpTo(WS.AngularSpeed, RollingOmega, DeltaTime, 6.0f);
		}

		if (IsDrivenWheel(i))
		{
			DrivenOmegaSum += WS.AngularSpeed;
			++DrivenOmegaCount;
		}
	}

	const float AvgOmega = (DrivenOmegaCount > 0) ? (DrivenOmegaSum / DrivenOmegaCount) : 0.0f;
	const float TargetRPM = FMath::Max(Powertrain.IdleRPM, FMath::Abs(AvgOmega * Ratio * Powertrain.FinalDrive * 9.5493f));
	EngineRPM = FMath::FInterpTo(EngineRPM, TargetRPM, DeltaTime, 5.0f);
	EngineRPM = FMath::Clamp(EngineRPM, Powertrain.IdleRPM, Powertrain.MaxRPM);
}

void FVehicleSimulator::ApplyAntiRoll(IVehiclePhysicsInterface& Body, const FTransform& BodyTransform)
{
	for (int32 i = 0; i + 1 < WheelsSetup.Num(); i += 2)
	{
		FWheelState& L = WheelsState[i];
		FWheelState& R = WheelsState[i + 1];
		if (!L.Contact.bHasContact || !R.Contact.bHasContact)
		{
			continue;
		}

		const bool bFront = WheelsSetup[i].LocalOffset.X > 0.0f;
		const float ARB = bFront ? Handling.AntiRollFront : Handling.AntiRollRear;
		const float LComp = FMath::Clamp(WheelsSetup[i].SuspensionRestCm - L.Contact.SuspensionLengthCm, 0.0f, WheelsSetup[i].SuspensionRestCm);
		const float RComp = FMath::Clamp(WheelsSetup[i + 1].SuspensionRestCm - R.Contact.SuspensionLengthCm, 0.0f, WheelsSetup[i + 1].SuspensionRestCm);
		const float RollDelta = (LComp - RComp) / FMath::Max(WheelsSetup[i].SuspensionRestCm, 1.0f);

		const FVector Up = BodyTransform.GetUnitAxis(EAxis::Z);
		Body.AddForceAtPoint(-Up * RollDelta * ARB, L.Contact.Point);
		Body.AddForceAtPoint(Up * RollDelta * ARB, R.Contact.Point);
	}
}

void FVehicleSimulator::ApplyAero(IVehiclePhysicsInterface& Body, const FTransform& BodyTransform)
{
	const FVector V = Body.GetLinearVelocity();
	const float Speed = V.Size();
	if (Speed < 1.0f)
	{
		return;
	}

	const FVector Dir = V / Speed;
	const FVector Drag = -Dir * Handling.DragCoeff * Speed * Speed;
	const FVector Rolling = -Dir * Handling.RollingResistanceCoeff * Speed;
	const FVector Downforce = -BodyTransform.GetUnitAxis(EAxis::Z) * Handling.DownforceCoeff * Speed * Speed;
	const FVector CoM = BodyTransform.GetLocation();

	Body.AddForceAtPoint(Drag + Rolling + Downforce, CoM);
}

void FVehicleSimulator::ApplyStabilityAssist(IVehiclePhysicsInterface& Body, const FTransform& BodyTransform, const FVector& LinearVelocity, const FVector& AngularVelocity)
{
	const FVector Forward = BodyTransform.GetUnitAxis(EAxis::X);
	const FVector Right = BodyTransform.GetUnitAxis(EAxis::Y);
	const float VLong = FVector::DotProduct(LinearVelocity, Forward);
	const float VLat = FVector::DotProduct(LinearVelocity, Right);

	if (Handling.bEnableStabilityAssist && FMath::Abs(Input.Steering) < 0.2f)
	{
		const FVector Assist = (-Right * VLat * 150.0f - Forward * VLong * 25.0f) * Handling.StabilityAssist;
		Body.AddForceAtPoint(Assist, BodyTransform.GetLocation());
	}

	const float YawTorque = -AngularVelocity.Z * FMath::Abs(AngularVelocity.Z) * Handling.DriftYawDamping;
	Body.AddTorque(FVector(0.0f, 0.0f, YawTorque));
}

void FVehicleSimulator::Tick(float DeltaTime, IVehiclePhysicsInterface& Body, const FWheelQueryFn& QueryFn)
{
	if (DeltaTime <= 0.0f)
	{
		return;
	}

	const FTransform BodyXf = Body.GetTransform();
	const FVector LinearV = Body.GetLinearVelocity();
	const FVector AngularV = Body.GetAngularVelocity();

	UpdatePowertrain(DeltaTime);
	UpdateSteering(DeltaTime, LinearV);
	UpdateWheelContacts(BodyXf, QueryFn);
	ComputeSuspensionLoads(DeltaTime, LinearV, BodyXf.GetUnitAxis(EAxis::Z));
	ComputeTireForces(DeltaTime, Body, BodyXf);
	ApplyAntiRoll(Body, BodyXf);
	ApplyAero(Body, BodyXf);
	ApplyStabilityAssist(Body, BodyXf, LinearV, AngularV);

	for (FWheelState& WS : WheelsState)
	{
		WS.RotationAngle = FMath::Fmod(WS.RotationAngle + WS.AngularSpeed * DeltaTime, TWO_PI);
	}
}

} // namespace GTAVehicle
