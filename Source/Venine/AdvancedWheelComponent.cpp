// Fill out your copyright notice in the Description page of Project Settings.

#include "AdvancedWheelComponent.h"
#include "DrawDebugHelpers.h"

#include "PhysXIncludes.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"
#include "Runtime/Engine/Private/PhysicsEngine/PhysXSupport.h"

#define LOG(format, ...) UE_LOG(LogTemp, Log, TEXT(format), __VA_ARGS__)
#define LOGW(format, ...) UE_LOG(LogTemp, Warning, TEXT(format), __VA_ARGS__)
#define LOGE(format, ...) UE_LOG(LogTemp, Error, TEXT(format), __VA_ARGS__)

void UAdvancedWheelComponent::RecordVector(FString Name, FVector Vec) {
	VectorRecord.Add(TTuple<FString,FVector>(Name,Vec));
}

void UAdvancedWheelComponent::CleanRecord() {
	VectorRecord.Empty();
}

UAdvancedWheelComponent::UAdvancedWheelComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;

	OnCalculateCustomPhysics.BindUObject(this, &UAdvancedWheelComponent::SubstepTick);
}

void UAdvancedWheelComponent::BeginPlay()
{
	Super::BeginPlay();

	Owner = GetOwner();

	TArray<UStaticMeshComponent*> SMComponents;
	TArray<UTextRenderComponent*> TRComponents;
	Owner->GetComponents<UStaticMeshComponent>(SMComponents);
	Owner->GetComponents<UTextRenderComponent>(TRComponents);

	for (auto *Component : SMComponents) {
		if (Component->GetName().Equals(WheelComponentName)) {
			WheelMesh = Component;
			LOG("FOUND WHEEL");
		}
	}
	if (IsValid(WheelMesh)) {
		WRigidBody = WheelMesh->GetBodyInstance()->GetPxRigidBody_AssumesLocked();
		if(WRigidBody){
			LOGW("Found appropriate Rigid Body for mesh");
		}
		else {
			LOGE("COULDN'T FIND RIGID BODY FOR MESH");
		}
	} else {
		LOGE("Wheel Component needs an appropriate wheel mesh!")
	}

	for (auto *Component : TRComponents) {
		if (Component->GetName().Equals(DebugTextComponentName)) {
			TextRender = Component;
			LOG("FOUND TEXTRENDER");
		}
	}

	BRigidBody = Cast<UStaticMeshComponent>(GetOwner()->GetRootComponent())->GetBodyInstance()->GetPxRigidBody_AssumesLocked();

	TireImpacts.SetNum(WheelToroidalDensity*WheelPoloidalDensity);
}

void UAdvancedWheelComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	GetWorld();
	UWorld *World = GetWorld();;
	
	if (WheelMesh) {
		WheelMesh->GetBodyInstance()->AddCustomPhysics(OnCalculateCustomPhysics);
	}
	
	if (TextRender) {
		TextRender->SetText(FText::FromString(CompositedText));
	}



	//DrawDebugLine(World, Vertices[i], Vertices[(i + 1) % Vertices.Num()], Color, true, 0, 0, LineSize);

	//LOGE("TICK on delta %f", DeltaTime);
}

void UAdvancedWheelComponent::SubstepTick(float DeltaTime, FBodyInstance* BodyInstance)
{

	if (P2UVector(WRigidBody->getGlobalPose().p).ContainsNaN() && !Crashed) {

		LOGE("PHYSX BROKE!\n");

		for (auto Record : VectorRecord) {
			LOGW("%s : %s", *(Record.Get<0>()), *(Record.Get<1>().ToString()));
		}


		Crashed = true;
	}

	CleanRecord();


	float TotalTraceDensity = WheelToroidalDensity * WheelPoloidalDensity;

	FTransform BodyTransform = P2UTransform(BRigidBody->getGlobalPose());
	FVector BodyPosition = BodyTransform.GetLocation();
	FVector BodyRight = BodyTransform.TransformVector(FVector::RightVector);
	FVector BodyUp = BodyTransform.TransformVector(FVector::UpVector);
	FVector BodyForward = BodyTransform.TransformVector(FVector::ForwardVector);

	FTransform WheelTransform = P2UTransform(WRigidBody->getGlobalPose());
	FVector WheelPosition = WheelTransform.GetLocation();
	FVector WheelRight = WheelTransform.TransformVector(FVector::RightVector);
	//FVector WheelUp = WheelTransform.TransformVector(FVector::UpVector);
	FVector WheelUp = FVector::VectorPlaneProject(FVector::UpVector, WheelRight).GetSafeNormal();
	FVector WheelForward = WheelTransform.TransformVector(FVector::ForwardVector);
	//FVector WheelForward = FVector::CrossProduct(-WheelRight, WheelUp);

	//WheelForward.Normalize();
	//FVector WheelUp = FVector::CrossProduct(WheelRight, WheelForward);
	//WheelUp.Normalize()

	float TotalSpringForce = 0.;
	float TotalDampForce = 0.;
	float TotalFrictionForce = 0.;
	FVector TotalFrictionVelocityForce = FVector::ZeroVector;
	FVector TotalFrictionLockForce = FVector::ZeroVector;
	uint32 TotalTraceHit = 0;
	float TotalGripStrength = 0;

	TArray<TTuple<PxVec3, PxVec3>> FrictionStack;

	for (uint32 TorN = 0; TorN < (uint32)WheelToroidalDensity; TorN++) {
		FVector ToroidalVector = WheelForward.RotateAngleAxis(((float)TorN / (uint32)WheelToroidalDensity) * WheelToroidalAngularSpan + WheelToroidalStartAngle, WheelRight);
		for (uint32 PolN = 0; PolN < (uint32)WheelPoloidalDensity; PolN++) {

			uint32 TraceIndex = TorN * WheelPoloidalDensity + PolN;

			FTireImpact TireImpact = TireImpacts[TraceIndex];

			FVector PoloidalRotationAxis = FVector::CrossProduct(ToroidalVector, WheelRight);
			PoloidalRotationAxis.Normalize();

			FVector StartPoint = WheelPosition + ToroidalVector * WheelTireInnerRadius;
			FVector PoloidalVector = ToroidalVector.RotateAngleAxis(((float)PolN / ((uint32)WheelPoloidalDensity-1) - 0.5) * WheelPoloidalAngularSpan, PoloidalRotationAxis);
			FVector EndPoint = StartPoint + PoloidalVector * WheelTireRadius;

			/* LINETRACE */
			FHitResult HitResult;
			FCollisionQueryParams HitParams = FCollisionQueryParams::DefaultQueryParam;
			HitParams.AddIgnoredComponent(WheelMesh);

			bool HitBool = GetWorld()->LineTraceSingleByChannel(HitResult, StartPoint, EndPoint, ECC_Visibility, HitParams);
			FVector HitImpact = HitResult.ImpactPoint;
			/*************/

			/* PARAMS */
			const float LockMul = WheelTireFrictionLockMul;
			const float VelocityMul = WheelTireFrictionVelMul;
			const float LockBreakDistance = 10.;
			const float AcceptableLockDistance = SMALL_NUMBER; // Setting this to 0 **will** cause crashes
			/**********/

			/* TIREPRESSURE */

			float Compression = 0;
			float GripStrength = 0;
			if (HitBool) {
				if (HitImpact.Equals(FVector(0.))) {
					LOGE("WRONG IMPACT");
				}

				Compression = FMath::Clamp(1. - HitResult.Distance / WheelTireRadius, 0., 1.);
				//GripStrength = 0.5 + .5*Compression;
				GripStrength = Compression;
				TotalGripStrength += GripStrength;

				FVector LockForce = FVector::ZeroVector;

				/* TIRELOCK */
				if (TireImpact.Locked) {

					FVector Delta = HitImpact - TireImpact.Position;

					// Don't bother if we're already close enough to target
					if (Delta.Size() <= AcceptableLockDistance) break;

					LockForce = (Delta * -LockMul * GripStrength) / (TotalTraceDensity);

					if ((TotalFrictionLockForce + LockForce).Size() > 1000000) {
						LOGE("ABBERANT FORCE AT %d", TraceIndex);
					}

					TotalFrictionLockForce += LockForce;

					// Break AFTER. This could induce unwanted DOWNWARDS and REVERSE forces.
					if (Delta.Size() > LockBreakDistance) {
						TireImpact.Position = HitImpact - (Delta.GetSafeNormal())*LockBreakDistance;
						if (DebugDraws) DrawDebugPoint(GetWorld(), HitImpact, 20, FColor::Red);
					}
				} 
				// Lock tire if compressed enough
				else if (Compression > .0001) {
					if (DebugDraws) DrawDebugPoint(GetWorld(), HitImpact, 20, FColor::Green);
					TireImpact.Position = HitImpact;
					TireImpact.Locked = true;
				}
				/************/

				/* TIREDAMP */

				FVector Velocity = FVector::VectorPlaneProject(P2UVector(PxRigidBodyExt::getVelocityAtPos(*WRigidBody, U2PVector(HitImpact))), PoloidalVector);
				FVector VelocityForce = (Velocity * -VelocityMul * GripStrength) / (TotalTraceDensity);
				TotalFrictionVelocityForce += VelocityForce;
				/************/

				if (VelocityForce.Size() > 100000 || LockForce.Size() > 100000) {
					LOGE("ABBERANT FORCES DETECTED");
				}

				// Add both forces to the friction stack
				FrictionStack.Add(TTuple<PxVec3, PxVec3>(U2PVector(LockForce + VelocityForce), U2PVector(HitImpact)));

				float TireCompression = FMath::Pow(Compression,WheelTirePressurePower)*(1.-WheelTirePressurePreload)+ WheelTirePressurePreload;

				FVector PoloidalDelta = P2UVector(PxRigidBodyExt::getVelocityAtPos(*WRigidBody, U2PVector((HitImpact)))).ProjectOnTo(PoloidalVector);

				FVector PressureForce = PoloidalVector * (-WheelTireKp / TotalTraceDensity) * TireCompression;
				FVector DampingForce = PoloidalDelta * (-WheelTireKd / TotalTraceDensity) * TireCompression;
	
				PxRigidBodyExt::addForceAtPos(*WRigidBody, U2PVector(PressureForce + DampingForce), U2PVector(HitImpact));

				TotalSpringForce += PressureForce.Size();
				TotalDampForce += DampingForce.Size();

				TotalTraceHit++;
			}else if (TireImpact.Locked) {
				TireImpact.Locked = false;
				//LOGW("IMPACT RELEASE [ %d ]", TraceIndex);
				if(DebugDraws) DrawDebugPoint(GetWorld(), TireImpact.Position, 30, FColor::Yellow);
			}

			TireImpacts[TraceIndex] = TireImpact;

			/****************/

			// DEBUG DRAWING
			FVector DrawCorrection = FVector::ZeroVector;// ToroidalVector * -10;
			if (DebugDraws) DrawDebugLine(GetWorld(), StartPoint + DrawCorrection, EndPoint + DrawCorrection, FColor((1 - Compression) * 255, Compression * 255, 0), false, 0, 0, Compression*5.+.1);
		}
	}

	if (TotalFrictionLockForce.Size() > 100000.) {
		LOGE("ABERRANT FORCES");
	}

	// Apply friction stack
	if (TotalTraceHit > 0) {
		for (auto Tuple : FrictionStack) {
			PxRigidBodyExt::addForceAtPos(*WRigidBody, Tuple.Get<0>() / (TotalGripStrength), Tuple.Get<1>());
		}
	}

	if (BikeStabilization) {
		FTransform BodyTransform = P2UTransform(BRigidBody->getGlobalPose());
		FVector BodyRight = BodyTransform.TransformVector(FVector::RightVector);
		FVector BodyUp = BodyTransform.TransformVector(FVector::UpVector);
		FVector BodyForward = BodyTransform.TransformVector(FVector::ForwardVector);

		FVector StabilizationForce = FVector::CrossProduct(BodyUp, FVector::UpVector).GetSafeNormal().ProjectOnTo(BodyForward)*10000000.;
		BRigidBody->addTorque(U2PVector(StabilizationForce));
	}

	CompositedText = *WheelComponentName + 
					 FString("\nHITS : ") + FString::FromInt(TotalTraceHit) + 
					 FString("\nSPRNG : ") + FString::FromInt(TotalSpringForce) + 
					 FString("\nDAMP : ") + FString::FromInt(TotalDampForce) + 
					 FString("\nFRIC : ") + FString::FromInt((TotalFrictionLockForce+TotalFrictionVelocityForce).Size()/(TotalGripStrength));

	//LOGW("[%s] \t %d HITS \t SPRING %d \t DAMP %d", *WheelComponentName, TotalTraceHit, (uint32)TotalSpringForce, (uint32) TotalDampForce);

	//LOG("SUBTICK on delta %f", DeltaTime);
}

FVector UAdvancedWheelComponent::GetCurrentLocation() {
	PxTransform PTransform = WRigidBody->getGlobalPose();
	return FVector(PTransform.p.x, PTransform.p.y, PTransform.p.z);
}

FRotator UAdvancedWheelComponent::GetCurrentRotation() {
	PxTransform PTransform = WRigidBody->getGlobalPose();
	return FRotator(FQuat(PTransform.q.x, PTransform.q.y, PTransform.q.z, PTransform.q.w));
}

FVector UAdvancedWheelComponent::GetCurrentAngularVelocity() {
	PxVec3 PAngVelocity = WRigidBody->getAngularVelocity();
	return FMath::RadiansToDegrees(FVector(PAngVelocity.x, PAngVelocity.y, PAngVelocity.z));
}

FVector UAdvancedWheelComponent::GetCurrentVelocity() {
	PxVec3 PVelocity = WRigidBody->getLinearVelocity();
	return FVector(PVelocity.x, PVelocity.y, PVelocity.z);
}
