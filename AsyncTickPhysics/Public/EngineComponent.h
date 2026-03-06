#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Curves/CurveFloat.h"
#include "EngineComponent.generated.h"

class UWheelComponent;

UENUM(BlueprintType)
enum class EDriveType : uint8
{
	FWD UMETA(DisplayName = "Front Wheel Drive"),
	RWD UMETA(DisplayName = "Rear Wheel Drive"),
	AWD UMETA(DisplayName = "All Wheel Drive")
};

UCLASS(ClassGroup=(Vehicle), BlueprintType, Blueprintable, meta=(BlueprintSpawnableComponent))
class ASYNCTICKPHYSICS_API UEngineComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UEngineComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
	float RPM = 900.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
	float IdleRPM = 900.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
	float MaxRPM = 7200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
	float EngineInertia = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Engine")
	FRuntimeFloatCurve TorqueCurve;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transmission")
	int32 NumGears = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transmission")
	TArray<float> GearRatios;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transmission")
	float FinalDriveRatio = 3.9f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transmission")
	int32 CurrentGear = 1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transmission")
	bool bAutomatic = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transmission")
	float ShiftUpRPM = 6500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Transmission")
	float ShiftDownRPM = 1800.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Braking")
	float BrakeTorque = 4500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Braking")
	float HandbrakeTorque = 8000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drive")
	EDriveType DriveType = EDriveType::AWD;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traction")
	float SlipThreshold = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traction")
	float TractionControlStrength = 0.75f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Traction")
	float LowGripFrictionScale = 0.6f;

	UFUNCTION(BlueprintCallable, Category = "Input")
	void SetThrottle(float Value);

	UFUNCTION(BlueprintCallable, Category = "Input")
	void SetBrake(float Value);

	UFUNCTION(BlueprintCallable, Category = "Input")
	void SetHandbrake(float Value);

	void Simulate(float DeltaTime, const TArray<UWheelComponent*>& Wheels, TMap<UWheelComponent*, float>& OutDriveForce, TMap<UWheelComponent*, float>& OutBrakeForce);

private:
	float ThrottleInput = 0.0f;
	float BrakeInput = 0.0f;
	float HandbrakeInput = 0.0f;

	void UpdateAutomaticGear();
	bool IsWheelDriven(const UWheelComponent* Wheel, int32 WheelIndex, int32 WheelCount) const;
};
