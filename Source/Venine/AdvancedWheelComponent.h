// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Components/TextRenderComponent.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "AdvancedWheelComponent.generated.h"

USTRUCT()
struct FTireImpact {
	GENERATED_BODY()

	UPROPERTY()
	FVector Position = FVector::ZeroVector;
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
	FCalculateCustomPhysics OnCalculateCustomPhysics;
	void SubstepTick(float DeltaTime, FBodyInstance* BodyInstance);

	physx::PxRigidBody* WRigidBody = NULL;
	physx::PxRigidBody* BRigidBody = NULL;

	AActor* Owner;
	UStaticMeshComponent* WheelMesh;
	UTextRenderComponent* TextRender;
	FString CompositedText;

	TArray<FTireImpact> TireImpacts;

	void RecordVector(FString Name, FVector Vec);
	void CleanRecord();

	TArray<TTuple<FString, FVector>> VectorRecord;
	bool Crashed = false;

	

	//UPhysicsConstraintComponent* WheelConstraint;

	UPROPERTY(EditAnywhere)
		FString WheelComponentName;
	UPROPERTY(EditAnywhere)
		FString DebugTextComponentName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		bool BikeStabilization;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		bool DebugDraws;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireInnerRadius = 30;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireRadius = 5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		int32 WheelToroidalDensity = 180;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		int32 WheelPoloidalDensity = 9;

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
		float WheelPoloidalAngularSpan = 180;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)

		float WheelTireFrictionLockMul = 100000.;
	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireFrictionVelMul = 200000.;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
		float WheelTireFrictionDeltaMax = 20000.;

	FORCEINLINE FVector  GetCurrentLocation();
	FORCEINLINE FRotator GetCurrentRotation();
	FORCEINLINE FVector  GetCurrentVelocity();
	FORCEINLINE FVector  GetCurrentAngularVelocity();
		
	
};