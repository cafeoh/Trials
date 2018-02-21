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
		PRigidBody = WheelMesh->GetBodyInstance()->GetPxRigidBody_AssumesLocked();
		if(PRigidBody){
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

	float TotalTraceDensity = WheelToroidalDensity * WheelPoloidalDensity;

	FTransform WheelTransform = P2UTransform(PRigidBody->getGlobalPose());

	/*FVector WheelCenter = WheelMesh->GetComponentLocation();
	FVector WheelRight = WheelMesh->GetRightVector();
	FVector WheelForward = FVector::CrossProduct(-WheelRight, Owner->GetActorUpVector());
	WheelForward.Normalize();
	FVector WheelUp = FVector::CrossProduct(WheelRight, WheelForward);
	WheelUp.Normalize();*/

	FVector WheelCenter = WheelTransform.GetLocation();
	FVector WheelRight = WheelTransform.TransformVector(FVector::RightVector);
	FVector WheelUp = WheelTransform.TransformVector(FVector::UpVector);
	FVector WheelForward = WheelTransform.TransformVector(FVector::ForwardVector);
	//FVector WheelForward = FVector::CrossProduct(-WheelRight, WheelUp);

	//WheelForward.Normalize();
	//FVector WheelUp = FVector::CrossProduct(WheelRight, WheelForward);
	//WheelUp.Normalize()

	float TotalSpringForce = 0.;
	float TotalDampForce = 0.;
	float TotalFrictionForce = 0.;
	uint32 TotalTraceHit = 0;
	float TotalGripStrength = 0;

	TArray<TTuple<PxVec3, PxVec3>> FrictionStack;

	FVector AverageContactPoint;
	FVector AverageFrictionDelta;

	for (uint32 TorN = 0; TorN < (uint32)WheelToroidalDensity; TorN++) {
		FVector ToroidalVector = WheelForward.RotateAngleAxis(((float)TorN / (uint32)WheelToroidalDensity) * WheelToroidalAngularSpan + WheelToroidalStartAngle, WheelRight);
		for (uint32 PolR = 0; PolR < (uint32)WheelPoloidalDensity; PolR++) {
			FVector PoloidalRotationAxis = FVector::CrossProduct(ToroidalVector, WheelRight);
			PoloidalRotationAxis.Normalize();

			FVector StartPoint = WheelCenter + ToroidalVector * WheelTireInnerRadius;
			FVector PoloidalVector = ToroidalVector.RotateAngleAxis(((float)PolR / ((uint32)WheelPoloidalDensity-1) - 0.5) * WheelPoloidalAngularSpan, PoloidalRotationAxis);
			FVector EndPoint = StartPoint + PoloidalVector * WheelTireRadius;

			/* LINETRACE */
			FHitResult HitResult;
			FCollisionQueryParams HitParams = FCollisionQueryParams::DefaultQueryParam;
			HitParams.AddIgnoredComponent(WheelMesh);

			bool HitBool = GetWorld()->LineTraceSingleByChannel(HitResult, StartPoint, EndPoint, ECC_Visibility, HitParams);
			FVector HitImpact = HitResult.ImpactPoint;
			/*************/

			/* TIREPRESSURE */

			float Compression = 0;
			float GripStrength = 0;
			if (HitBool) {
				Compression = FMath::Clamp(1. - HitResult.Distance / FVector::Distance(StartPoint, EndPoint), 0., 1.);

				GripStrength = 0.8 + .2*Compression;
				TotalGripStrength += GripStrength;

				// Averaging
				AverageContactPoint += HitImpact * GripStrength;

				Compression = FMath::Pow(Compression,WheelTirePressurePower)*(1.-WheelTirePressurePreload)+ WheelTirePressurePreload;

				FVector PoloidalDelta = P2UVector(PxRigidBodyExt::getVelocityAtPos(*PRigidBody, U2PVector((HitImpact)))).ProjectOnTo(PoloidalVector);

				FVector FrictionDelta = P2UVector(PxRigidBodyExt::getVelocityAtPos(*PRigidBody, U2PVector((HitImpact)))) - PoloidalDelta;

				AverageFrictionDelta += FrictionDelta * GripStrength;

				FrictionDelta = FMath::Pow(FrictionDelta.Size(), WheelTireFrictionDeltaPower) * (FrictionDelta/FrictionDelta.Size());

				FVector PressureForce = PoloidalVector * Compression * (-WheelTireKp / TotalTraceDensity);
				FVector DampingForce = PoloidalDelta * (-WheelTireKd / TotalTraceDensity) * Compression;

				FVector FrictionDeltaLateral = FrictionDelta.ProjectOnTo(WheelRight);
				FVector FrictionDeltaPlanar = FrictionDelta - FrictionDeltaLateral;

				FVector FrictionForce = GripStrength * (FrictionDeltaPlanar * -WheelTireFrictionDeltaMultiplier.X + FrictionDeltaLateral * -WheelTireFrictionDeltaMultiplier.Y) / TotalTraceDensity;

				//FrictionForce = (FrictionForce / FrictionForce.Size())*(FMath::Min<float>(FrictionForce.Size(), 2000.));
					
				PxRigidBodyExt::addForceAtPos(*PRigidBody, U2PVector(PressureForce + DampingForce), U2PVector(HitImpact));

				//PxRigidBodyExt::addForceAtPos(*PRigidBody, U2PVector(FrictionForce), U2PVector(HitImpact));
				FrictionStack.Add(TTuple<PxVec3, PxVec3>(U2PVector(FrictionForce), U2PVector(HitImpact)));

				TotalSpringForce += PressureForce.Size();
				TotalDampForce += DampingForce.Size();
				TotalFrictionForce += FrictionForce.Size();
				TotalTraceHit++;
			}


			/****************/

			// DEBUG DRAWING
			FVector DrawCorrection = FVector::ZeroVector;// ToroidalVector * -10;
			DrawDebugLine(GetWorld(), StartPoint + DrawCorrection, EndPoint + DrawCorrection, FColor((1 - Compression) * 255, Compression * 255, 0), false, 0, 0, Compression+.1);
		}
	}

	AverageContactPoint = AverageContactPoint / TotalGripStrength;
	AverageFrictionDelta /= TotalGripStrength;

	float FrictionDiv = FMath::Max<float>(1.,(TotalFrictionForce /(WheelTireFrictionDeltaMax*TotalGripStrength)));
	//TotalFrictionComp = FMath::Max<float>(1., TotalFrictionComp);

	DrawDebugPoint(GetWorld(), AverageContactPoint, 10, FColor::Cyan);
	
	if (TotalTraceHit > 0 && true) {
		// OR try using velocity delta to SCALE average delta
		FVector VelocityDelta = P2UVector(PxRigidBodyExt::getVelocityAtPos(*PRigidBody, U2PVector(AverageContactPoint)));
		FVector VelocityToAdd = VelocityDelta * 0.;

		LOG("(grip %f) %s -> %s", TotalGripStrength, *(AverageContactPoint.ToString()) ,*(VelocityToAdd.ToString()));

		PxRigidBodyExt::addForceAtPos(*PRigidBody, PxVec3(0.,0.,0.), U2PVector(AverageContactPoint));
	}



	if (TotalTraceHit > 0 && true) {

		// FMath::Max(1., TotalFrictionForce / 100000.);

		for (auto Tuple : FrictionStack) {
			PxRigidBodyExt::addForceAtPos(*PRigidBody, Tuple.Get<0>() / (FrictionDiv*TotalGripStrength), Tuple.Get<1>());
		}

		LOGW("TotalGripStrength : %f", TotalGripStrength);
		if (FrictionDiv > 1.01) LOGE("MAXFRIC");
	}

	/*PRigidBody->addForce(TotalLinearImpulse*000000., physx::PxForceMode::eFORCE);
	PRigidBody->addTorque(TotalAngularImpulse*000000., physx::PxForceMode::eFORCE);*/

	//LOG("=== > %s : %s", *(P2UVector(TotalLinearImpulse).ToString()), *(P2UVector(TotalAngularImpulse).ToString()));

	CompositedText = *WheelComponentName + FString("\nHITS : ") + FString::FromInt(TotalTraceHit) + FString("\nSPRNG : ") + FString::FromInt(TotalSpringForce) + FString("\nDAMP : ") + FString::FromInt(TotalDampForce) + FString("\nFRIC : ") + FString::FromInt(TotalFrictionForce/(FrictionDiv*TotalGripStrength));

	//LOGW("[%s] \t %d HITS \t SPRING %d \t DAMP %d", *WheelComponentName, TotalTraceHit, (uint32)TotalSpringForce, (uint32) TotalDampForce);

	//LOG("SUBTICK on delta %f", DeltaTime);
}

FVector UAdvancedWheelComponent::GetCurrentLocation() {
	PxTransform PTransform = PRigidBody->getGlobalPose();
	return FVector(PTransform.p.x, PTransform.p.y, PTransform.p.z);
}

FRotator UAdvancedWheelComponent::GetCurrentRotation() {
	PxTransform PTransform = PRigidBody->getGlobalPose();
	return FRotator(FQuat(PTransform.q.x, PTransform.q.y, PTransform.q.z, PTransform.q.w));
}

FVector UAdvancedWheelComponent::GetCurrentAngularVelocity() {
	PxVec3 PAngVelocity = PRigidBody->getAngularVelocity();
	return FMath::RadiansToDegrees(FVector(PAngVelocity.x, PAngVelocity.y, PAngVelocity.z));
}

FVector UAdvancedWheelComponent::GetCurrentVelocity() {
	PxVec3 PVelocity = PRigidBody->getLinearVelocity();
	return FVector(PVelocity.x, PVelocity.y, PVelocity.z);
}
