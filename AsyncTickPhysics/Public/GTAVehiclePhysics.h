#pragma once

#include "CoreMinimal.h"

namespace GTAVehicle
{

enum class EDriveLayout : uint8
{
	FWD,
	RWD,
	AWD
};

struct FSurfacePreset
{
	float Friction = 1.0f;
	float LongitudinalGrip = 1.0f;
	float LateralGrip = 1.0f;
	float RollingResistance = 1.0f;
};

struct FVehicleInputState
{
	float Throttle = 0.0f;   // [-1..1]
	float Brake = 0.0f;      // [0..1]
	float Handbrake = 0.0f;  // [0..1]
	float Steering = 0.0f;   // [-1..1]
};

struct FWheelSetup
{
	FName Name;
	FVector LocalOffset = FVector::ZeroVector;
	float RadiusCm = 34.0f;
	float WidthCm = 24.0f;
	float SuspensionRestCm = 35.0f;
	float SuspensionMinCm = 10.0f;
	float SuspensionMaxCm = 45.0f;
	float SpringRate = 36000.0f;
	float DamperCompression = 4200.0f;
	float DamperRebound = 5200.0f;
	float CorneringStiffness = 7.0f;
	float LongitudinalStiffness = 9.0f;
	float TireFriction = 1.15f;
	bool bSteering = false;
	bool bDriven = false;
	bool bHandbrakeWheel = false;
};

struct FPowertrainSetup
{
	EDriveLayout DriveLayout = EDriveLayout::RWD;
	float IdleRPM = 900.0f;
	float MaxRPM = 7300.0f;
	float ShiftUpRPM = 6700.0f;
	float ShiftDownRPM = 1900.0f;
	float FinalDrive = 3.9f;
	float EngineBrakeTorque = 140.0f;
	float DifferentialBias = 0.25f;
	float ThrottleResponse = 6.0f;
	TArray<float> GearRatios = {0.0f, 3.2f, 2.2f, 1.6f, 1.25f, 1.0f, 0.82f};
};

struct FVehicleHandling
{
	float MassKg = 1550.0f;
	float GravityScale = 1.0f;
	float WheelBaseCm = 270.0f;
	float TrackFrontCm = 160.0f;
	float TrackRearCm = 160.0f;
	float CenterOfMassHeightCm = 50.0f;
	float MaxSteerDeg = 34.0f;
	float SteerSpeed = 8.0f;
	float SteerReturnSpeed = 10.0f;
	float HighSpeedSteerReduction = 0.58f;
	float BrakeTorque = 6500.0f;
	float HandbrakeTorque = 12000.0f;
	float DragCoeff = 0.42f;
	float RollingResistanceCoeff = 15.0f;
	float DownforceCoeff = 0.0018f;
	float AntiRollFront = 22000.0f;
	float AntiRollRear = 16000.0f;
	float DriftGripScale = 0.8f;
	float DriftYawDamping = 28.0f;
	float StabilityAssist = 0.22f;
	float TractionAssist = 0.35f;
	float ABSSensitivity = 0.30f;
	float LowSpeedJitterThreshold = 40.0f; // cm/s
};

struct FWheelContact
{
	bool bHasContact = false;
	FVector Point = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	FSurfacePreset Surface;
	float SuspensionLengthCm = 35.0f;
};

struct FWheelState
{
	FWheelContact Contact;
	float SteerAngleDeg = 0.0f;
	float AngularSpeed = 0.0f; // rad/s
	float RotationAngle = 0.0f;
	float SlipRatio = 0.0f;
	float SlipAngle = 0.0f;
	float SmoothedFx = 0.0f;
	float SmoothedFy = 0.0f;
	float NormalLoad = 0.0f;
	FVector TotalForce = FVector::ZeroVector;
};

class IVehiclePhysicsInterface
{
public:
	virtual ~IVehiclePhysicsInterface() = default;
	virtual FTransform GetTransform() const = 0;
	virtual FVector GetLinearVelocity() const = 0;
	virtual FVector GetAngularVelocity() const = 0;
	virtual FVector GetVelocityAtPoint(const FVector& WorldPoint) const = 0;
	virtual void AddForceAtPoint(const FVector& Force, const FVector& Point) = 0;
	virtual void AddTorque(const FVector& Torque) = 0;
};

using FWheelQueryFn = TFunction<bool(int32 WheelIndex, const FTransform& BodyTransform, FWheelContact& OutContact)>;

class FVehicleSimulator
{
public:
	void Initialize(const FVehicleHandling& InHandling, const FPowertrainSetup& InPowertrain, const TArray<FWheelSetup>& InWheels);
	void SetInput(const FVehicleInputState& InInput);
	void Tick(float DeltaTime, IVehiclePhysicsInterface& Body, const FWheelQueryFn& QueryFn);

	const TArray<FWheelState>& GetWheels() const { return WheelsState; }
	float GetEngineRPM() const { return EngineRPM; }
	int32 GetCurrentGear() const { return CurrentGear; }

private:
	void UpdateSteering(float DeltaTime, const FVector& LinearVelocity);
	void UpdatePowertrain(float DeltaTime);
	void UpdateWheelContacts(const FTransform& BodyTransform, const FWheelQueryFn& QueryFn);
	void ComputeSuspensionLoads(float DeltaTime, const FVector& LinearVelocity, const FVector& BodyUp);
	void ComputeTireForces(float DeltaTime, IVehiclePhysicsInterface& Body, const FTransform& BodyTransform);
	void ApplyAntiRoll(IVehiclePhysicsInterface& Body, const FTransform& BodyTransform);
	void ApplyAero(IVehiclePhysicsInterface& Body, const FTransform& BodyTransform);
	void ApplyStabilityAssist(IVehiclePhysicsInterface& Body, const FTransform& BodyTransform, const FVector& LinearVelocity, const FVector& AngularVelocity);
	float EvalEngineTorque(float NormalizedRPM) const;
	bool IsDrivenWheel(int32 Index) const;

private:
	FVehicleHandling Handling;
	FPowertrainSetup Powertrain;
	FVehicleInputState Input;
	TArray<FWheelSetup> WheelsSetup;
	TArray<FWheelState> WheelsState;

	float SmoothedThrottle = 0.0f;
	float EngineRPM = 900.0f;
	int32 CurrentGear = 1;
	float CurrentSteer = 0.0f;
};

} // namespace GTAVehicle
