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
	BrakeInput = FMath::Clamp(Value, 0.0f, 1.0f);
}

void UEngineComponent::SetHandbrake(float Value)
{
	HandbrakeInput = FMath::Clamp(Value, 0.0f, 1.0f);
}

void UEngineComponent::UpdateAutomaticGear()
{
	if (!bAutomatic)
	{
		return;
	}

	if (RPM > ShiftUpRPM && CurrentGear < NumGears)
	{
		++CurrentGear;
	}
	else if (RPM < ShiftDownRPM && CurrentGear > 1)
	{
		--CurrentGear;
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

	UpdateAutomaticGear();

	const float GearRatio = GearRatios.IsValidIndex(CurrentGear) ? GearRatios[CurrentGear] : 0.0f;
	const float NormalizedRPM = FMath::Clamp((RPM - IdleRPM) / FMath::Max(MaxRPM - IdleRPM, 1.0f), 0.0f, 1.0f);
	const float EngineTorque = TorqueCurve.GetRichCurveConst() ? TorqueCurve.GetRichCurveConst()->Eval(NormalizedRPM) : 450.0f;
	float WheelTorque = EngineTorque * ThrottleInput * GearRatio * FinalDriveRatio;

	int32 NumDrivenWheels = 0;
	for (int32 Idx = 0; Idx < Wheels.Num(); ++Idx)
	{
		if (IsWheelDriven(Wheels[Idx], Idx, Wheels.Num()))
		{
			++NumDrivenWheels;
		}
	}

	if (NumDrivenWheels <= 0)
	{
		NumDrivenWheels = 1;
	}

	const float TorquePerWheel = WheelTorque / static_cast<float>(NumDrivenWheels);
	float AvgDrivenWheelOmega = 0.0f;
	int32 AvgCount = 0;

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
				const float TCSFactor = FMath::Lerp(1.0f, 1.0f - TractionControlStrength, SlipAlpha);
				AppliedWheelTorque *= TCSFactor;
				GripScale = FMath::Lerp(1.0f, LowGripFrictionScale, SlipAlpha);
			}
		}

		const float DriveForce = (AppliedWheelTorque * GripScale) / FMath::Max(Wheel->WheelRadius, 1.0f);
		float TotalBrake = BrakeInput * BrakeTorque;
		if (HandbrakeInput > 0.0f && !IsWheelDriven(Wheel, Idx, Wheels.Num()))
		{
			TotalBrake += HandbrakeInput * HandbrakeTorque;
		}

		OutDriveForce.Add(Wheel, DriveForce);
		OutBrakeForce.Add(Wheel, TotalBrake / FMath::Max(Wheel->WheelRadius, 1.0f));

		if (bDriven)
		{
			AvgDrivenWheelOmega += Wheel->WheelAngularVelocity;
			++AvgCount;
		}
	}

	const float DrivenOmega = (AvgCount > 0) ? (AvgDrivenWheelOmega / static_cast<float>(AvgCount)) : 0.0f;
	const float TargetRPM = FMath::Max(IdleRPM, FMath::Abs(DrivenOmega * GearRatio * FinalDriveRatio * 9.5493f));
	RPM = FMath::FInterpTo(RPM, TargetRPM, DeltaTime, FMath::Max(EngineInertia, 0.05f));
	RPM = FMath::Clamp(RPM, IdleRPM, MaxRPM);
}
