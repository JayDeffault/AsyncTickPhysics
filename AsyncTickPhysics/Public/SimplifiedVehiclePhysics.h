#pragma once

#include "CoreMinimal.h"
#include "SimplifiedVehiclePhysics.generated.h"

USTRUCT(BlueprintType)
struct ASYNCTICKPHYSICS_API FWheelSetup
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	FName WheelName = NAME_None;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	FVector LocalPosition = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float Radius = 34.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float SuspensionMaxDrop = 35.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float SuspensionMaxRaise = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float SuspensionStiffness = 35000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float SuspensionDamping = 4500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float TireFriction = 1.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float CorneringStiffness = 80000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float LongitudinalStiffness = 90000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float LateralGripScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float LongitudinalGripScale = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float LoadSensitivity = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float RelaxationRate = 12.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float LowSpeedThreshold = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float SlipRatioPeak = 0.12f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	float SlipAnglePeakDeg = 8.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	bool bIsDrivenWheel = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel")
	bool bIsSteerWheel = false;
};

USTRUCT(BlueprintType)
struct ASYNCTICKPHYSICS_API FWheelState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	bool bIsGrounded = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	FVector ContactPoint = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	FVector ContactNormal = FVector::UpVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float Compression = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float CompressionRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float CompressionVelocity = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float NormalLoad = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float WheelAngularSpeed = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float SlipRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float SlipAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float RelaxedLongitudinalForce = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	float RelaxedLateralForce = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel")
	FVector LastAppliedForce = FVector::ZeroVector;
};

struct ASYNCTICKPHYSICS_API FTireForces
{
	float Longitudinal = 0.0f;
	float Lateral = 0.0f;
	float Vertical = 0.0f;
};

class ASYNCTICKPHYSICS_API TirePhysics
{
public:
	static void ComputeSlip(
		const FWheelSetup& Setup,
		FWheelState& State,
		float LongitudinalSpeed,
		float LateralSpeed,
		float DeltaTime);

	static FTireForces ComputeTireForces(
		const FWheelSetup& Setup,
		FWheelState& State,
		float EngineForce,
		float BrakeForce,
		float DeltaTime);
};

class ASYNCTICKPHYSICS_API VehiclePhysics
{
public:
	using FSuspensionSweepFunc = TFunctionRef<bool(const FVector& Start, const FVector& End, float Radius, FHitResult& OutHit)>;
	using FVelocityAtPointFunc = TFunctionRef<FVector(const FVector& WorldPoint)>;
	using FApplyForceFunc = TFunctionRef<void(const FVector& Force, const FVector& WorldPoint)>;

	TArray<FWheelSetup> WheelSetups;
	TArray<FWheelState> WheelStates;

	void InitializeFromSetups(const TArray<FWheelSetup>& InSetups);

	void UpdateWheel(
		int32 WheelIndex,
		const FTransform& BodyTransform,
		const FVector& BodyUp,
		float SteeringAngleDeg,
		float EngineForce,
		float BrakeForce,
		float DeltaTime,
		FSuspensionSweepFunc SweepFunc,
		FVelocityAtPointFunc VelocityAtPointFunc,
		FApplyForceFunc ApplyForceFunc);

	void ApplyForces(
		int32 WheelIndex,
		const FVector& WheelForward,
		const FVector& WheelRight,
		const FVector& WheelUp,
		const FTireForces& TireForces,
		FApplyForceFunc ApplyForceFunc);
};
