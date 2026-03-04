#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"
#include "WheelComponent.generated.h"

class UStaticMeshComponent;

USTRUCT(BlueprintType)
struct FWheelForces
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Wheel|Forces")
	FVector SuspensionForce = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Wheel|Forces")
	FVector LateralForce = FVector::ZeroVector;

	UPROPERTY(BlueprintReadOnly, Category = "Wheel|Forces")
	FVector LongitudinalForce = FVector::ZeroVector;

	FORCEINLINE FVector TotalForce() const
	{
		return SuspensionForce + LateralForce + LongitudinalForce;
	}
};

UCLASS(ClassGroup=(Vehicle), BlueprintType, Blueprintable, meta=(BlueprintSpawnableComponent))
class ASYNCTICKPHYSICS_API UWheelComponent : public USceneComponent
{
	GENERATED_BODY()

public:
	UWheelComponent();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Setup")
	UStaticMeshComponent* WheelCollisionMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Setup")
	float WheelRadius = 34.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Setup")
	float WheelWidth = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Suspension")
	float SuspensionStiffness = 35000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Suspension")
	float SuspensionDamping = 4500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Suspension")
	float SuspensionUpperLimit = 25.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Suspension")
	float SuspensionLowerLimit = 35.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Tire")
	float TireFriction = 1.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Tire")
	float LateralFrictionScale = 0.35f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	bool bIsDrivenWheel = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	bool bIsSteerWheel = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	bool bIsGrounded = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	float CompressionRatio = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	float CurrentSuspensionLength = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	float WheelAngularVelocity = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	float SlipRatioLong = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	float SlipAngleLat = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	FVector ContactPoint = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|State")
	FVector ContactNormal = FVector::UpVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Wheel|Debug")
	bool bDebugVehicle = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel|Debug")
	FVector LastWheelWorldLocation = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel|Debug")
	FVector LastSweepStart = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel|Debug")
	FVector LastSweepEnd = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel|Debug")
	bool bLastSweepHadBlockingHit = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Wheel|Debug")
	FVector LastTotalForce = FVector::ZeroVector;

	void UpdateContact(float DeltaTime, UPrimitiveComponent* BodyMesh, const FTransform& WheelWorldTransform);

	FWheelForces SimulateWheel(
		float DeltaTime,
		UPrimitiveComponent* BodyMesh,
		float DriveForce,
		float BrakeForce,
		float SteeringAngleDeg,
		const FTransform& WheelWorldTransform);

	float GetNormalLoad() const { return CachedNormalLoad; }

protected:
	virtual void BeginPlay() override;

private:
	void EnsureCollisionMesh();

	float CachedNormalLoad = 0.0f;
};
