#pragma once

#include "CoreMinimal.h"
namespace Simcade
{

enum class EDriveType : uint8
{
	FWD,
	RWD,
	AWD
};

enum class EWheelQueryMode : uint8
{
	Raycast,
	SphereSweep,
	CapsuleSweep
};

enum class ESurfaceType : uint8
{
	Asphalt,
	Dirt,
	Grass,
	Gravel,
	WetAsphalt,
	Sand
};

struct VehicleInput
{
	float Throttle = 0.0f;     // [-1..1]
	float Brake = 0.0f;        // [0..1]
	float Handbrake = 0.0f;    // [0..1]
	float Steering = 0.0f;     // [-1..1]
};

struct SurfaceData
{
	float FrictionCoefficient = 1.0f;
	float LateralGripMultiplier = 1.0f;
	float LongitudinalGripMultiplier = 1.0f;
	float RollingResistanceMultiplier = 1.0f;
	float DampingMultiplier = 1.0f;
};

struct VehicleMassProperties
{
	float MassKg = 1600.0f;
	FVector CenterOfMassOffset = FVector(0.0f, 0.0f, -20.0f);
	float WheelbaseCm = 275.0f;
	float TrackFrontCm = 162.0f;
	float TrackRearCm = 160.0f;
	float CGHeightCm = 52.0f;
	float InertiaYawScale = 1.2f;
	float InertiaPitchScale = 1.1f;
	float InertiaRollScale = 1.15f;
};

struct VehicleTuning
{
	// Base body/mass.
	float Mass = 1600.0f;
	float GravityScale = 1.0f;

	// Wheel.
	float WheelRadius = 34.0f;
	float WheelMass = 18.0f;
	float WheelInertia = 1.2f;

	// Suspension.
	float SuspensionStiffness = 38000.0f;
	float SuspensionDampingCompression = 4200.0f;
	float SuspensionDampingRebound = 5500.0f;
	float SuspensionRestLength = 34.0f;
	float BumpStopStiffness = 100000.0f;
	float AntiRollFront = 18000.0f;
	float AntiRollRear = 12000.0f;

	// Tire and load.
	float BaseFriction = 1.18f;
	float CorneringStiffnessFront = 7.0f;
	float CorneringStiffnessRear = 6.2f;
	float LongitudinalStiffnessFront = 8.8f;
	float LongitudinalStiffnessRear = 9.3f;
	float LoadSensitivity = 0.23f;
	float PostPeakGrip = 0.72f;

	// Steering.
	float SteeringMaxAngle = 35.0f;
	float SteeringSpeed = 6.0f;
	float SteeringReturnSpeed = 8.0f;
	float HighSpeedSteerReduction = 0.55f;

	// Helpers.
	float YawAssist = 0.16f;
	float TractionAssist = 0.45f;
	float AbsAssist = 0.35f;
	float DriftAssist = 0.30f;

	// Input response.
	float ThrottleResponse = 6.0f;
	float BrakePower = 14500.0f;
	float HandbrakePower = 22000.0f;

	// Engine/transmission.
	float EngineTorque = 520.0f;
	float EngineBrake = 1700.0f;
	float DragCoefficient = 0.39f;
	float RollingResistance = 18.0f;
	float DownforceCoefficient = 0.0018f;
	float MaxSpeedSoftLimit = 72.0f; // m/s

	float InertiaYawScale = 1.2f;
	float InertiaPitchScale = 1.1f;
	float InertiaRollScale = 1.15f;

	// Drift/game-feel controls.
	float DriftGripScale = 0.86f;
	float PostPeakLateralGrip = 0.68f;
	float YawDampingInDrift = 22.0f;
	float CounterSteerAssist = 0.18f;
	float ThrottleOversteerFactor = 0.32f;
	float LowSpeedThresholdMps = 0.6f;
};

struct WheelSetup
{
	FName Name;
	FVector LocalAttachPoint = FVector::ZeroVector;
	float RadiusCm = 34.0f;
	float WidthCm = 24.0f;
	float SuspensionMinLengthCm = 8.0f;
	float SuspensionMaxLengthCm = 45.0f;
	float SuspensionPreloadCm = 3.0f;
	bool bSteer = false;
	bool bDriven = true;
	bool bHandbrakeWheel = false;
	int32 AxleIndex = 0;
	bool bLeft = true;
};

struct WheelContactData
{
	bool bHasContact = false;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	FVector QueryStart = FVector::ZeroVector;
	FVector QueryEnd = FVector::ZeroVector;
	float HitDistanceCm = 0.0f;
	float SurfaceFriction = 1.0f;
	ESurfaceType SurfaceType = ESurfaceType::Asphalt;
	SurfaceData Surface;
};

struct SuspensionState
{
	float CurrentLengthCm = 0.0f;
	float PrevLengthCm = 0.0f;
	float CompressionRatio = 0.0f;
	float CompressionVelocity = 0.0f;
	float SpringForceN = 0.0f;
	float DamperForceN = 0.0f;
	float BumpStopForceN = 0.0f;
	float ReboundStopForceN = 0.0f;
	float NormalLoadN = 0.0f;
};

struct TireForceResult
{
	float SlipRatio = 0.0f;
	float SlipAngleRad = 0.0f;
	float Fx = 0.0f;
	float Fy = 0.0f;
	float Fz = 0.0f;
	FVector ForceWorld = FVector::ZeroVector;
};

struct WheelState
{
	WheelContactData Contact;
	SuspensionState Suspension;
	TireForceResult Tire;
	float WheelOmega = 0.0f;
	float WheelAngle = 0.0f;
	float SmoothedSlipRatio = 0.0f;
	float SmoothedSlipAngle = 0.0f;
	float SmoothedFx = 0.0f;
	float SmoothedFy = 0.0f;
	float DriveTorque = 0.0f;
	float BrakeTorque = 0.0f;
	float SteerAngleRad = 0.0f;
};

struct AxleState
{
	int32 LeftWheelIndex = INDEX_NONE;
	int32 RightWheelIndex = INDEX_NONE;
	float AntiRollStiffness = 0.0f;
};

class IVehiclePhysicsBody
{
public:
	virtual ~IVehiclePhysicsBody() = default;
	virtual FTransform GetWorldTransform() const = 0;
	virtual FVector GetLinearVelocityAtPoint(const FVector& WorldPoint) const = 0;
	virtual FVector GetLinearVelocity() const = 0;
	virtual FVector GetAngularVelocity() const = 0;
	virtual float GetMassKg() const = 0;
	virtual void AddForceAtPoint(const FVector& ForceN, const FVector& WorldPoint) = 0;
	virtual void AddTorque(const FVector& TorqueNm) = 0;
};

using FWheelQueryCallback = TFunction<bool(EWheelQueryMode QueryMode, const WheelSetup&, const FTransform&, WheelContactData&)>;

class TireModel
{
public:
	TireForceResult ComputeTireForces(
		const VehicleTuning& Tuning,
		const WheelSetup& Setup,
		WheelState& State,
		const FVector& WheelForward,
		const FVector& WheelRight,
		const FVector& ContactVelocity,
		float DeltaTime,
		bool bInDrift) const;

private:
	void ComputeTireSlip(const VehicleTuning& Tuning, WheelState& State, float VLong, float VLat, float DeltaTime) const;
	float ApplyRelaxation(float Previous, float Target, float RelaxRate, float DeltaTime) const;
};

class SuspensionModel
{
public:
	void UpdateSuspensionForWheel(const VehicleTuning& Tuning, const WheelSetup& Setup, WheelState& State, float DeltaTime) const;
};

class DrivetrainModel
{
public:
	void SetDriveType(EDriveType InDriveType) { DriveType = InDriveType; }
	void ApplyDrivetrain(
		const VehicleTuning& Tuning,
		const VehicleInput& Input,
		const TArray<WheelSetup>& Setups,
		TArray<WheelState>& States,
		float VehicleSpeedMps,
		float DeltaTime);

private:
	EDriveType DriveType = EDriveType::RWD;
	float SmoothedThrottle = 0.0f;
};

class VehicleStabilityAssist
{
public:
	void ApplyStabilityHelpers(
		const VehicleTuning& Tuning,
		const VehicleInput& Input,
		IVehiclePhysicsBody& Body,
		const TArray<WheelSetup>& Setups,
		TArray<WheelState>& States,
		float DeltaTime) const;
};

class VehiclePhysics
{
public:
	void Initialize(const VehicleMassProperties& InMassProps, const VehicleTuning& InTuning, const TArray<WheelSetup>& InWheels);
	void SetQueryMode(EWheelQueryMode InMode) { QueryMode = InMode; }
	void SetInput(const VehicleInput& InInput) { RawInput = InInput; }
	void SetSurfaceTable(const TMap<ESurfaceType, SurfaceData>& InSurfaces) { SurfaceMap = InSurfaces; }

	void UpdateVehicle(float DeltaTime, IVehiclePhysicsBody& Body, const FWheelQueryCallback& QueryCallback);

	const TArray<WheelState>& GetWheelStates() const { return WheelStates; }

private:
	void ReadInput(float DeltaTime);
	void UpdateSteering(float DeltaTime, const IVehiclePhysicsBody& Body);
	void PerformWheelQueries(IVehiclePhysicsBody& Body, const FWheelQueryCallback& QueryCallback);
	bool QueryWheelContact(int32 WheelIndex, IVehiclePhysicsBody& Body, const FWheelQueryCallback& QueryCallback);
	void UpdateSuspension(float DeltaTime);
	void ComputeLoadTransfer(const IVehiclePhysicsBody& Body, float DeltaTime);
	void ComputeTireForces(float DeltaTime, IVehiclePhysicsBody& Body);
	void ApplyDrivetrain(float DeltaTime, IVehiclePhysicsBody& Body);
	void ApplyBraking(float DeltaTime);
	void ApplyAeroDrag(IVehiclePhysicsBody& Body);
	void ApplyAeroAndRollingResistance(IVehiclePhysicsBody& Body);
	void ApplyStabilityAssist(float DeltaTime, IVehiclePhysicsBody& Body);
	void ApplyAntiRollBars(IVehiclePhysicsBody& Body);
	void ApplyWheelForces(IVehiclePhysicsBody& Body);
	void UpdateWheelAngularVelocity(float DeltaTime);

private:
	VehicleMassProperties MassProps;
	VehicleTuning Tuning;
	VehicleInput RawInput;
	VehicleInput FilteredInput;
	TArray<WheelSetup> WheelSetups;
	TArray<WheelState> WheelStates;
	TArray<AxleState> Axles;
	TMap<ESurfaceType, SurfaceData> SurfaceMap;

	FVector PrevLinearVelocity = FVector::ZeroVector;
	float SteerVisual = 0.0f;
	EWheelQueryMode QueryMode = EWheelQueryMode::SphereSweep;

	TireModel Tire;
	SuspensionModel Suspension;
	DrivetrainModel Drivetrain;
	VehicleStabilityAssist StabilityAssist;
};

} // namespace Simcade
