#include "EngineComponent.h"

#include "WheelComponent.h"

UEngineComponent::UEngineComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
	GearRatios = {0.0f, 3.2f, 2.1f, 1.5f, 1.18f, 1.0f, 0.82f};
}

void UEngineComponent::SetThrottle(float Value)
{
	ThrottleInput = FMath::Clamp(Value, -1.0f, 1.0f);
}

void UEngineComponent::SetBrake(float Value)
{
	BrakeInput = (FMath::Abs(Value) < 0.05f) ? 0.0f : FMath::Clamp(Value, 0.0f, 1.0f);
}

void UEngineComponent::SetHandbrake(float Value)
{
	HandbrakeInput = (FMath::Abs(Value) < 0.05f) ? 0.0f : FMath::Clamp(Value, 0.0f, 1.0f);
}

void UEngineComponent::UpdateAutomaticGear()
{
	if (!bAutomatic || ShiftTimer > 0.0f)
	{
		return;
	}

	if (RPM > ShiftUpRPM && CurrentGear < NumGears)
	{
		++CurrentGear;
		ShiftTimer = ShiftCooldown;
	}
	else if (RPM < ShiftDownRPM && CurrentGear > 1)
	{
		--CurrentGear;
		ShiftTimer = ShiftCooldown;
	}
}

bool UEngineComponent::IsWheelDriven(const UWheelComponent* Wheel, int32 WheelIndex, int32 WheelCount) const
{
	if (!Wheel || !Wheel->bIsDrivenWheel)
	{
		return false;
	}

	const bool bFront = WheelIndex < WheelCount / 2;
	if (DriveType == EDriveType::AWD)
	{
		return true;
	}
	if (DriveType == EDriveType::FWD)
	{
		return bFront;
	}
	return !bFront;
}

void UEngineComponent::Simulate(float DeltaTime, const TArray<UWheelComponent*>& Wheels, TMap<UWheelComponent*, float>& OutDriveForce, TMap<UWheelComponent*, float>& OutBrakeForce)
{
	OutDriveForce.Reset();
	OutBrakeForce.Reset();

	ShiftTimer = FMath::Max(0.0f, ShiftTimer - DeltaTime);

	const float ThrottleRate = (FMath::Abs(ThrottleInput) > FMath::Abs(SmoothedThrottle)) ? ThrottleRiseRate : ThrottleFallRate;
	SmoothedThrottle = FMath::FInterpTo(SmoothedThrottle, ThrottleInput, DeltaTime, FMath::Max(0.1f, ThrottleRate));

	float AvgDrivenWheelOmega = 0.0f;
	float AvgDrivenGroundSpeed = 0.0f;
	int32 AvgCount = 0;
	for (int32 Idx = 0; Idx < Wheels.Num(); ++Idx)
	{
		const UWheelComponent* Wheel = Wheels[Idx];
		if (!IsWheelDriven(Wheel, Idx, Wheels.Num()))
		{
			continue;
		}

		AvgDrivenWheelOmega += Wheel->WheelAngularVelocity;
		AvgDrivenGroundSpeed += Wheel->LinearSpeedAlongWheelForward;
		++AvgCount;
	}

	const float DrivenOmega = (AvgCount > 0) ? (AvgDrivenWheelOmega / static_cast<float>(AvgCount)) : 0.0f;
	const float ForwardSpeed = (AvgCount > 0) ? (AvgDrivenGroundSpeed / static_cast<float>(AvgCount)) : 0.0f;

	bool bReverseDrive = false;
	if (bUseAutoReverse && SmoothedThrottle < -0.1f && FMath::Abs(ForwardSpeed) < AutoReverseSpeedThreshold)
	{
		bReverseDrive = true;
	}

	UpdateAutomaticGear();

	float GearRatio = 0.0f;
	if (bReverseDrive)
	{
		GearRatio = -FMath::Abs(ReverseGearRatio);
	}
	else
	{
		GearRatio = GearRatios.IsValidIndex(CurrentGear) ? GearRatios[CurrentGear] : 0.0f;
	}

	const float NormalizedRPM = FMath::Clamp((RPM - IdleRPM) / FMath::Max(MaxRPM - IdleRPM, 1.0f), 0.0f, 1.0f);
	float EngineTorque = 450.0f;
	if (const FRichCurve* RichCurve = TorqueCurve.GetRichCurveConst())
	{
		EngineTorque = RichCurve->GetNumKeys() > 0 ? RichCurve->Eval(NormalizedRPM) : 450.0f;
	}

	const float RequestedThrottle = bReverseDrive ? FMath::Abs(SmoothedThrottle) : FMath::Max(0.0f, SmoothedThrottle);
	float WheelTorque = EngineTorque * RequestedThrottle * GearRatio * FinalDriveRatio;

	if (RequestedThrottle < 0.05f && FMath::Abs(ForwardSpeed) > 10.0f)
	{
		WheelTorque -= FMath::Sign(ForwardSpeed) * EngineBrakingTorque * FMath::Abs(GearRatio) * FinalDriveRatio;
	}

	int32 NumDrivenWheels = 0;
	for (int32 Idx = 0; Idx < Wheels.Num(); ++Idx)
	{
		if (IsWheelDriven(Wheels[Idx], Idx, Wheels.Num()))
		{
			++NumDrivenWheels;
		}
	}
	NumDrivenWheels = FMath::Max(1, NumDrivenWheels);

	const float TorquePerWheel = WheelTorque / static_cast<float>(NumDrivenWheels);

	for (int32 Idx = 0; Idx < Wheels.Num(); ++Idx)
	{
		UWheelComponent* Wheel = Wheels[Idx];
		if (!Wheel)
		{
			continue;
		}

		const bool bDriven = IsWheelDriven(Wheel, Idx, Wheels.Num());
		float AppliedWheelTorque = bDriven ? TorquePerWheel : 0.0f;
		float GripScale = 1.0f;
		if (bDriven)
		{
			const float SlipAbs = FMath::Abs(Wheel->SlipRatioLong);
			if (SlipAbs > SlipThreshold)
			{
				const float SlipAlpha = FMath::Clamp((SlipAbs - SlipThreshold) / FMath::Max(SlipThreshold, 0.01f), 0.0f, 1.0f);
				AppliedWheelTorque *= FMath::Lerp(1.0f, 1.0f - TractionControlStrength, SlipAlpha);
				GripScale = FMath::Lerp(1.0f, LowGripFrictionScale, SlipAlpha);
			}
		}

		const float WheelRadiusMeters = FMath::Max(Wheel->WheelRadius * 0.01f, 0.05f);
		const float DriveForce = (AppliedWheelTorque * GripScale) / WheelRadiusMeters;

		float TotalBrake = BrakeInput * BrakeTorque;
		const bool bRearWheel = Idx >= (Wheels.Num() / 2);
		if (HandbrakeInput > 0.0f && bRearWheel)
		{
			TotalBrake += HandbrakeInput * HandbrakeTorque;
		}

		if (RequestedThrottle > 0.1f && BrakeInput < 0.1f && HandbrakeInput < 0.05f)
		{
			TotalBrake = 0.0f;
		}

		OutDriveForce.Add(Wheel, DriveForce);
		OutBrakeForce.Add(Wheel, TotalBrake / WheelRadiusMeters);
	}

	const float TargetRPMFromWheels = FMath::Max(IdleRPM, FMath::Abs(DrivenOmega * FMath::Max(0.2f, FMath::Abs(GearRatio)) * FinalDriveRatio * 9.5493f));
	const float FreeRevTarget = FMath::Lerp(IdleRPM, MaxRPM, RequestedThrottle);
	const float Coupling = FMath::Clamp(static_cast<float>(AvgCount) / 2.0f, 0.25f, 1.0f);
	const float TargetRPM = FMath::Lerp(FreeRevTarget, TargetRPMFromWheels, Coupling);

	RPM = FMath::FInterpTo(RPM, TargetRPM, DeltaTime, FMath::Max(0.1f, EngineInertia));
	RPM = FMath::Clamp(RPM, IdleRPM, MaxRPM);
}
