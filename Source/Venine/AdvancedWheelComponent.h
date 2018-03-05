// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/TextRenderComponent.h"
#include "AdvancedWheelComponent.generated.h"

USTRUCT()
struct FTireImpact {
	GENERATED_BODY()

	UPROPERTY()
	FVector StartPoint = FVector::ZeroVector;
	UPROPERTY()
	FVector EndPoint = FVector::ZeroVector;
	UPROPERTY()
	float Compression = 0;
	UPROPERTY()
	bool Hit = false;
	UPROPERTY()
	FVector LastSpring = FVector::ZeroVector;
	UPROPERTY()
	FVector LastDamping = FVector::ZeroVector;
	UPROPERTY()
	FVector LastFriction = FVector::ZeroVector;

	UPROPERTY()
	FHitResult HitResult;

	UPROPERTY()
	int32 LockTime = 0;
	UPROPERTY()
	bool Locked = false;
};

UCLASS( ClassGroup=(Custom), meta=(BlueprintSpawnableComponent) )
class VENINE_API UAdvancedWheelComponent : public UActorComponent
{
	GENERATED_BODY()

public:	
	// Sets default values for this component's properties
	UAdvancedWheelComponent();
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void SubstepTick(float DeltaTime, FBodyInstance* BodyInstance);
	FTireImpact *GetImpact(uint32 TraceIndex);
	FTireImpact *GetImpact(uint32 TorI, uint32 PolI);
	void TraceImpacts();

	FCalculateCustomPhysics OnCalculateCustomPhysics;
	physx::PxRigidBody* WRigidBody = NULL;
	physx::PxRigidBody* BRigidBody = NULL;

	AActor* Owner;
	UStaticMeshComponent* WheelMesh;
	UTextRenderComponent* TextRender;
	FString CompositedText;

	TArray<FTireImpact> TireImpacts;

	TArray<TTuple<FString, FVector>> VectorRecord;
	bool Crashed = false;

	

	//UPhysicsConstraintComponent* WheelConstraint;

	UPROPERTY(EditAnywhere)
		FString WheelComponentName;
	UPROPERTY(EditAnywhere)
		FString DebugTextComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		bool DebugDraws = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		bool DrawTraceSpheres = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		bool DebugLogs = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		bool OptimizedTracing = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireInnerRadius = 25;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireRadius = 7;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		int32 WheelToroidalDensity = 50;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
		int32 WheelPoloidalDensity;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
		float SphereTraceRadius;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireKp = 50000000;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireKd = 10000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTirePressurePower = 1.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTirePressurePreload = 0.2;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)

		float WheelToroidalStartAngle = 0;	
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelToroidalAngularSpan = 360;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelPoloidalAngularSpan = 270;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireFrictionLockMul = 100000.;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireFrictionVelMul = 200000.;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireFrictionDeltaMax = 20000.;
};