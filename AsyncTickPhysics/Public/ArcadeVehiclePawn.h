#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "ArcadeVehiclePawn.generated.h"

class UStaticMeshComponent;

UENUM(BlueprintType)
enum class EArcadeDriveLayout : uint8
{
	FWD,
	RWD,
	AWD
};

USTRUCT(BlueprintType)
struct FArcadeWheelSetup
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel") FName Name;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel") FVector LocalOffset = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel") float RadiusCm = 34.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel") float WidthCm = 24.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Suspension") float RestLengthCm = 35.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Suspension") float MinLengthCm = 8.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Suspension") float MaxLengthCm = 45.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Suspension") float SpringRate = 36000.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Suspension") float DamperCompression = 4200.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Suspension") float DamperRebound = 5200.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Tire") float CorneringStiffness = 7.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Tire") float LongitudinalStiffness = 9.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel|Tire") float FrictionScale = 1.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel") bool bSteerWheel = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel") bool bDrivenWheel = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Wheel") bool bHandbrakeWheel = false;
};

USTRUCT(BlueprintType)
struct FArcadeWheelState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") bool bGrounded = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") FHitResult Hit;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float SuspensionLengthCm = 35.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float SpringForce = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float DamperForce = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float NormalLoad = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float AngularVelocity = 0.f; // rad/s
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float WheelRPM = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float SlipRatio = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float SlipAngle = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float Fx = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float Fy = 0.f;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Wheel") float SteerAngleDeg = 0.f;
};

USTRUCT(BlueprintType)
struct FArcadeEngineConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Engine") float MaxTorqueNm = 460.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Engine") float IdleRPM = 900.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Engine") float MaxRPM = 7300.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Engine") float EngineInertia = 0.25f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Engine") float ThrottleResponse = 6.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Engine") float EngineBraking = 180.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Engine") FRuntimeFloatCurve TorqueCurve;
};

USTRUCT(BlueprintType)
struct FArcadeTransmissionConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") bool bAutomatic = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") TArray<float> ForwardGearRatios = {3.2f, 2.2f, 1.6f, 1.25f, 1.0f, 0.82f};
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") float ReverseGearRatio = -2.9f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") float FinalDrive = 3.9f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") float UpshiftRPM = 6700.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") float DownshiftRPM = 1900.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") float ShiftTime = 0.18f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Transmission") EArcadeDriveLayout DriveLayout = EArcadeDriveLayout::RWD;
};

USTRUCT(BlueprintType)
struct FArcadeHandlingConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float Drag = 0.42f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float Downforce = 0.0018f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float RollingResistance = 15.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float MaxSteerDeg = 34.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float SteerSpeed = 8.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float SteerReturn = 10.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float HighSpeedSteerReduction = 0.58f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float BrakeTorque = 6500.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float HandbrakeTorque = 12000.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float AntiRollFront = 22000.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float AntiRollRear = 16000.f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float DriftRearGripScale = 0.78f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float YawAssist = 0.2f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float TractionControl = 0.35f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float ABSAssist = 0.3f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Handling") float LowSpeedJitterThreshold = 45.f;
};

UCLASS(BlueprintType)
class ASYNCTICKPHYSICS_API AArcadeVehiclePawn : public APawn
{
	GENERATED_BODY()

public:
	AArcadeVehiclePawn();
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle") UStaticMeshComponent* BodyMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle") TArray<FArcadeWheelSetup> WheelSetups;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Vehicle") TArray<FArcadeWheelState> WheelStates;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle") FArcadeEngineConfig Engine;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle") FArcadeTransmissionConfig Transmission;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle") FArcadeHandlingConfig Handling;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Vehicle|Debug") bool bDebug = true;

	UFUNCTION(BlueprintCallable) void SetThrottle(float Value);
	UFUNCTION(BlueprintCallable) void SetBrake(float Value);
	UFUNCTION(BlueprintCallable) void SetHandbrake(float Value);
	UFUNCTION(BlueprintCallable) void SetSteering(float Value);

private:
	void TickVehicle(float Dt);
	void UpdateTransmission(float Dt, float DrivenWheelOmega);
	void UpdateSteering(float Dt, float SpeedCm);
	void SimulateWheel(int32 WheelIndex, float Dt, float DriveTorquePerWheel);
	float ComputeTorqueFromCurve() const;
	bool IsDrivenByLayout(int32 WheelIndex) const;
	void DrawDebugData(float Dt) const;

private:
	float ThrottleInput = 0.f;
	float BrakeInput = 0.f;
	float HandbrakeInput = 0.f;
	float SteeringInput = 0.f;
	float SmoothedThrottle = 0.f;
	float EngineRPM = 900.f;
	int32 CurrentGear = 1;
	float ShiftTimer = 0.f;
	float CurrentSteerDeg = 0.f;
};
