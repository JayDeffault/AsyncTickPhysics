#pragma once

#include "CoreMinimal.h"
#include "AsyncTickPawn.h"
#include "EngineComponent.h"
#include "BaseVehicle.generated.h"

class UStaticMeshComponent;
class UWheelComponent;

UENUM(BlueprintType)
enum class EVehicleClass : uint8
{
	Cars,
	Jeeps,
	Buses,
	Vans,
	Trucks
};

UCLASS(BlueprintType)
class ASYNCTICKPHYSICS_API ABaseVehicle : public AAsyncTickPawn
{
	GENERATED_BODY()

public:
	ABaseVehicle();

	virtual void NativeAsyncTick(float DeltaTime) override;
	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Components")
	UStaticMeshComponent* BodyMesh;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Components")
	UEngineComponent* EngineComponent;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Components")
	TArray<UWheelComponent*> Wheels;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Meta")
	EVehicleClass VehicleClass = EVehicleClass::Cars;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Meta")
	FName VehicleId = TEXT("Vehicle_01");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Meta")
	FText VehicleName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Damage")
	float MaxHealth = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Damage")
	float Health = 1000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Damage")
	float CollisionDamageMult = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Damage")
	float WeaponDamageMult = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Steering")
	float MaxSteerAngleDeg = 35.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Steering")
	float SteerSpeed = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|AntiRoll")
	float AntiRollBarStiffness = 10000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vehicle|Debug")
	bool bDebugVehicle = true;

	UFUNCTION(BlueprintCallable, Category = "Vehicle|Input")
	void SetThrottle(float Value);

	UFUNCTION(BlueprintCallable, Category = "Vehicle|Input")
	void SetBrake(float Value);

	UFUNCTION(BlueprintCallable, Category = "Vehicle|Input")
	void SetHandbrake(float Value);

	UFUNCTION(BlueprintCallable, Category = "Vehicle|Input")
	void SetSteering(float Value);

	UFUNCTION(BlueprintCallable, Category = "Vehicle|Damage")
	void ApplyCollisionDamage(float ImpactStrength);

	UFUNCTION(BlueprintCallable, Category = "Vehicle|Damage")
	void ApplyWeaponDamage(float Damage);

	UFUNCTION(BlueprintCallable, Category = "Vehicle|Damage")
	void DestroyVehicle();

protected:
	virtual void BeginPlay() override;

	UFUNCTION()
	void OnBodyHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComp, FVector NormalImpulse, const FHitResult& Hit);

private:
	float ThrottleInput = 0.0f;
	float BrakeInput = 0.0f;
	float HandbrakeInput = 0.0f;
	float SteeringInput = 0.0f;
	float CurrentSteerAngle = 0.0f;

	void ApplyAntiRollBar();
	void DrawVehicleDebug(float DeltaTime);
};
