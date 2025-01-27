// Fill out your copyright notice in the Description page of Project Settings.

#include "AdvancedWheelComponent.h"
#include "DrawDebugHelpers.h"
#include "WorldCollision.h"
#include "Components/ActorComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include "PhysXIncludes.h"
#include "PhysicsPublic.h"
#include "PhysXPublic.h"
#include "Runtime/Engine/Private/PhysicsEngine/PhysXSupport.h"

#define LOG(format, ...) UE_LOG(LogTemp, Log, TEXT(format), __VA_ARGS__)
#define LOGW(format, ...) UE_LOG(LogTemp, Warning, TEXT(format), __VA_ARGS__)
#define LOGE(format, ...) UE_LOG(LogTemp, Error, TEXT(format), __VA_ARGS__)

FDebugFloatHistory SpringHistory;
FDebugFloatHistory DampHistory;
FDebugFloatHistory FrictionHistory;

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
			return;
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

	float ToroidalPerimeter = 6.283 * (WheelTireInnerRadius+WheelTireRadius);
	float PoloidalPerimeter = 6.283 * (WheelTireRadius * WheelPoloidalAngularSpan / 360.);

	SphereTraceRadius = (ToroidalPerimeter / WheelToroidalDensity) / 2.;
	WheelPoloidalDensity = FMath::RoundToInt(PoloidalPerimeter / SphereTraceRadius);

	TireImpacts.SetNum(WheelToroidalDensity*WheelPoloidalDensity);

	LOGE("Setting spheretrace radius to %f", SphereTraceRadius);
	LOGE("Setting Poloidal density to %d", WheelPoloidalDensity);
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

	FVector HistoryForward = FVector::CrossProduct(GetOwner()->GetActorRightVector(), FVector::UpVector).GetSafeNormal();
	FTransform HistoryTransform = GetOwner()->GetActorTransform();

	if (DrawTraceSpheres) {

		for(FTireImpact TireImpact : TireImpacts){

			//DrawDebugSphere(GetWorld(), TireImpact.EndPoint - (TireImpact.EndPoint-TireImpact.StartPoint).GetSafeNormal()*SphereTraceRadius, SphereTraceRadius, 8, FColor::Yellow, false, 0, 0, .2);
			DrawDebugPoint(GetWorld(), TireImpact.EndPoint, SphereTraceRadius, FColor::Yellow, false, 0, 0);
		}
	}

	if(DebugDraws){
		DrawDebugFloatHistory(*GetWorld(), SpringHistory, GetOwner()->GetActorLocation() + HistoryForward *-200 + FVector(0, 0, 200), FVector2D(400, 50), FColor(0,255,0,150), 0, 0, 0);
		DrawDebugFloatHistory(*GetWorld(), DampHistory, GetOwner()->GetActorLocation() + HistoryForward *-200 + FVector(0, 0, 150), FVector2D(400, 50), FColor(0, 0, 255, 150), 0, 0, 0);
		DrawDebugFloatHistory(*GetWorld(), FrictionHistory, GetOwner()->GetActorLocation() + HistoryForward *-200 + FVector(0, 0, 100), FVector2D(400, 50), FColor(255, 0, 0, 150), 0, 0, 0);
	}

	SubstepIndex = 0;

	//DrawDebugLine(World, Vertices[i], Vertices[(i + 1) % Vertices.Num()], Color, true, 0, 0, LineSize);

	//LOGE("TICK on delta %f", DeltaTime);

	//WheelTransformedLocation = WheelMesh->GetAttachParent()->GetComponentTransform().InverseTransformPosition(WheelMesh->GetComponentLocation());

}


void UAdvancedWheelComponent::GenerateTransforms()
{
	WheelTransform = WheelMesh->GetRelativeTransform() * WheelMesh->GetBodyInstance()->GetUnrealWorldTransform();
	WheelPosition = WheelTransform.GetLocation();
	WheelRight = WheelTransform.TransformVector(FVector::RightVector);
	WheelUp = FVector::VectorPlaneProject(FVector::UpVector, WheelRight).GetSafeNormal();
	if (!WheelUp.IsUnit()) LOGE("UNSAFENORMAL WHEELUP");
	WheelForward = WheelTransform.TransformVector(FVector::ForwardVector);

}

FTireImpact *UAdvancedWheelComponent::GetImpact(uint32 TraceIndex) {
	return &(TireImpacts[TraceIndex%(WheelToroidalDensity*WheelPoloidalDensity)]);
}

FTireImpact *UAdvancedWheelComponent::GetImpact(uint32 TorI, uint32 PolI) {
	TorI %= WheelToroidalDensity;
	PolI %= WheelPoloidalDensity;
	return &TireImpacts[TorI * WheelPoloidalDensity + PolI];
}

FVector UAdvancedWheelComponent::GetPatchNormal(uint32 TraceIndex)
{
	FVector PatchNormal;
	/* GETTING TIREPATCH NORMAL 

		Methods :
	(1) Poloidal Normal
	(2) Impact Derivative (using adjacent impacts)
	(3) Ground Impact Normal
	(4) Center to Impact Normal (for sphere sweep)

		Method 1
	FVector PatchNormal = (TireImpact->EndPoint - TireImpact->StartPoint).GetSafeNormal();

		Method 2
	FTireImpact *PatchL = GetImpact(TraceIndex - 1);
	FTireImpact *PatchR = GetImpact(TraceIndex + 1);
	FTireImpact *PatchF = GetImpact(TraceIndex + WheelPoloidalDensity);
	FTireImpact *PatchB = GetImpact(TraceIndex - WheelPoloidalDensity);
	FVector PatchNormal = FVector::CrossProduct(PatchL->EndPoint - PatchR->EndPoint, GetImpact(TraceIndex + WheelPoloidalDensity)->EndPoint - GetImpact(TraceIndex - WheelPoloidalDensity)->EndPoint).GetSafeNormal();

		Method 3
	FVector PatchNormal = -TireImpact->HitResult.ImpactNormal;
	*/

	//  Method 4
	FTireImpact *TireImpact = GetImpact(TraceIndex);
	PatchNormal = (TireImpact->HitResult.ImpactPoint - TireImpact->HitResult.Location).GetSafeNormal();

	return PatchNormal;
}

int32 UAdvancedWheelComponent::GetDebugData(FString Key){
	return DebugData.Contains(Key) ? DebugData[Key] : 0;
}

void UAdvancedWheelComponent::SetDebugData(FString Key, int32 Value)
{
	DebugData.Add(Key, Value);
}

void UAdvancedWheelComponent::AddDebugData(FString Key, int32 Value){
	SetDebugData(Key, GetDebugData(Key) + Value);
}

void UAdvancedWheelComponent::TraceImpacts(){

	/* TRACING SETUP */
	FCollisionQueryParams HitParams = FCollisionQueryParams::DefaultQueryParam;
	HitParams.AddIgnoredActor(GetOwner());
	/*****************/

	/* COLLECT ALL TRACES */
	for (uint32 TorN = 0; TorN < (uint32)WheelToroidalDensity; TorN++) {
		FVector ToroidalVector = WheelForward.RotateAngleAxis(((float)TorN / (uint32)WheelToroidalDensity) * WheelToroidalAngularSpan + WheelToroidalStartAngle, WheelRight);

		FVector StartPoint = WheelPosition + ToroidalVector * WheelTireInnerRadius;
		bool SkipTrace = false;

		// SKIP IF USELESS
		if (OptimizedTracing && !GetWorld()->OverlapAnyTestByChannel(StartPoint, FQuat::Identity, ECC_Visibility, FCollisionShape::MakeSphere(WheelTireRadius*1.2), HitParams)) {
			SkipTrace = true;
		}

		for (uint32 PolN = 0; PolN < (uint32)WheelPoloidalDensity; PolN++) {

			FTireImpact *TireImpact = GetImpact(TorN, PolN);

			FVector PoloidalRotationAxis = FVector::CrossProduct(ToroidalVector, WheelRight).GetSafeNormal();
			if (PoloidalRotationAxis.IsNearlyZero()) LOGE("UNSAFENORMAL POLOIDALROTATIONAXIS");
			FVector PoloidalVector = ToroidalVector.RotateAngleAxis(((float)PolN / ((uint32)WheelPoloidalDensity - 1) - 0.5) * WheelPoloidalAngularSpan, PoloidalRotationAxis);

			TireImpact->StartPoint = StartPoint - ToroidalVector * WheelTireRadius*1.2;
			TireImpact->EndPoint = StartPoint + PoloidalVector * WheelTireRadius;

			float LineLength = (TireImpact->EndPoint - TireImpact->StartPoint).Size();



			// Optimize out unnecessary traces
			if (SkipTrace) {
				TireImpact->LastDamping = FVector::ZeroVector;
				TireImpact->Hit = false;
				TireImpact->Compression = 0.;
				continue;
			}

			AddDebugData("TotalTraces", 1);

			if (true) {
				TireImpact->Hit = GetWorld()->SweepSingleByChannel(TireImpact->HitResult, TireImpact->StartPoint, TireImpact->EndPoint - PoloidalVector * SphereTraceRadius, FQuat::Identity, ECC_Visibility, FCollisionShape::MakeSphere(SphereTraceRadius), HitParams);
			}
			else {
				TireImpact->Hit = GetWorld()->LineTraceSingleByChannel(TireImpact->HitResult, TireImpact->StartPoint, TireImpact->EndPoint, ECC_Visibility, HitParams);
			}

			if (TireImpact->Hit) {

				// ENSURING GOOD NORMAL TO PREVENT PHYSX LOCK
				if (TireImpact->HitResult.Distance < KINDA_SMALL_NUMBER) {
					TireImpact->Compression = 1. - KINDA_SMALL_NUMBER;
					TireImpact->EndPoint = FMath::Lerp<FVector>(TireImpact->StartPoint, TireImpact->EndPoint, KINDA_SMALL_NUMBER);
				} else {
					TireImpact->EndPoint = TireImpact->HitResult.ImpactPoint;
					TireImpact->Compression = FMath::Clamp(1. - (TireImpact->EndPoint - TireImpact->StartPoint).Size() / LineLength, 0., 1.);
				}
			}
		}
	}
}

void UAdvancedWheelComponent::SubstepTick(float DeltaTime, FBodyInstance* BodyInstance)
{

	GenerateTransforms();

	// First substep of the tick
	if(SubstepIndex == 0){
		// Clean debug data
		SetDebugData("TotalTraces", 0);
		SetDebugData("TotalHitTraces", 0);
	}

	if (!GetOwner()->GetRootComponent()->IsSimulatingPhysics() || !WheelMesh->IsSimulatingPhysics()) {
		return;
	}

	if (P2UVector(WRigidBody->getGlobalPose().p).ContainsNaN() && !Crashed) {

		LOGE("PHYSX BROKE!\n");
		Crashed = true;
	}

	float MaxFrictionLockForce = 0;
	float MaxFrictionVelForce = 0;
	float MaxTireSpringForce = 0;
	float MaxTireDampForce = 0;
	float MaxVelocity = 0;

	float TotalTraceDensity = WheelToroidalDensity * WheelPoloidalDensity;

	float TotalSpringForce = 0.;
	float TotalDampForce = 0.;
	float TotalFrictionForce = 0.;
	float TotalFrictionVelocityForce = 0.;
	float TotalGripStrength = 0;

	uint32 TotalTraceHit = 0;

	TArray<TTuple<PxVec3, PxVec3>> FrictionStack;
	TArray<TTuple<PxVec3, PxVec3>> SpringStack;


	/* PARAMS */
	const float VelocityMul = WheelTireFrictionVelMul;
	const float StaticVelCap = 1000000;
	const float SpringCap = 200000;
	const float DampCap = 500000;

	const float InnerTireOffset = 1.; // 0 starts at center, 1 starts at edges of tire (general case value), any value significantly beyond is probably meaningless
	/**********/

	bool Slipping = false;

	// Trace all impacts
	TraceImpacts();
	
	/******************/

	for (uint32 TraceIndex = 0 ; TraceIndex < TotalTraceDensity ; TraceIndex++){

		Slipping = false;

		FTireImpact *TireImpact = GetImpact(TraceIndex);
		
		float GripStrength = 0;
		if (TireImpact->Hit) {
			//float TireCompression = (1-FMath::Pow(1-TireImpact->Compression,WheelTirePressurePower))*(1.-WheelTirePressurePreload)+ WheelTirePressurePreload;
			GripStrength = (1. - FMath::Pow(1 - TireImpact->Compression, 5.))*.8 + .2;
			TotalGripStrength += GripStrength;




			/* GETTING TIREPATCH NORMAL */
			FVector PatchNormal = GetPatchNormal(TraceIndex);

			if (TireImpact->HitResult.Distance < KINDA_SMALL_NUMBER) {
				LOGE("TIREIMPACT ERROR : %s", *((TireImpact->EndPoint - TireImpact->StartPoint).ToString()));
			}
			
			if (PatchNormal.IsNearlyZero()) {
				LOGE("AVERTED PATCHNORMAL CRISIS");
				continue;
				//PatchNormal = (GetImpact(TraceIndex)->EndPoint - GetImpact(TraceIndex)->StartPoint).GetSafeNormal();
			}
			/****************************/




			/************ FRICTION ************/
			FVector LateralVector = FVector::VectorPlaneProject(WheelRight, PatchNormal).GetSafeNormal();
			if (LateralVector.ContainsNaN()) LOGE("LATERAL VECTOR IS NAN"); // TOFIX
			FVector ForwardVector = FVector::CrossProduct(PatchNormal, LateralVector).GetSafeNormal();
			if (ForwardVector.ContainsNaN()) LOGE("FORWARD VECTOR IS NAN"); // TOFIX
			FVector LateralVelocity = P2UVector(PxRigidBodyExt::getVelocityAtPos(*WRigidBody, U2PVector(TireImpact->EndPoint))).ProjectOnToNormal(LateralVector);
			FVector ForwardVelocity = P2UVector(PxRigidBodyExt::getVelocityAtPos(*WRigidBody, U2PVector(TireImpact->EndPoint))).ProjectOnToNormal(ForwardVector);
			
			FVector FrictionForce = (LateralVelocity * -VelocityMul) + (ForwardVector * EnginePower) + (ForwardVelocity * -(BreakPower + 5.));
			//FrictionForce = FrictionForce.GetClampedToMaxSize(StaticVelCap);

// 			if (FrictionForce.Size() > 1000000) {
// 				FrictionForce *= .1;
// 				Slipping = true;
// 			}

			MaxVelocity = FMath::Max<float>(LateralVelocity.Size(), MaxVelocity);
			TotalFrictionVelocityForce += FrictionForce.Size();

			// FILTERING FRICTION STACK
			FrictionForce = FMath::Lerp<FVector>(TireImpact->LastFriction, FrictionForce, .2);
			TireImpact->LastFriction = FrictionForce;
			FrictionStack.Add(TTuple<PxVec3, PxVec3>(U2PVector(FrictionForce), U2PVector(TireImpact->EndPoint)));
			/*********************************/



			/************ TIRE PUSH ************/
			float TireCompression = (1-FMath::Pow(1-TireImpact->Compression,WheelTirePressurePower))*(1.-WheelTirePressurePreload)+ WheelTirePressurePreload;

			FVector PoloidalDelta = P2UVector(PxRigidBodyExt::getVelocityAtPos(*WRigidBody, U2PVector((TireImpact->EndPoint)))).ProjectOnTo(PatchNormal);

			FVector PressureForce = PatchNormal   * (-WheelTireKp / TotalTraceDensity) * TireCompression;
			FVector DampingForce  = PoloidalDelta * (-WheelTireKd / TotalTraceDensity) * TireCompression;

			if (FVector::DotProduct(DampingForce, PoloidalDelta) > 0) {
				DampingForce *= 1.;
			}

			PressureForce = PressureForce.GetClampedToMaxSize(SpringCap);
			DampingForce = DampingForce.GetClampedToMaxSize(DampCap);

			if (DebugLogs && PressureForce.Size() == SpringCap) LOG("SPRING HIT CAP");
			if (DebugLogs && DampingForce.Size() == DampCap) LOG("DAMP HIT CAP");

			// FILTERING SPRING STACK
			PressureForce = FMath::Lerp<FVector>(TireImpact->LastSpring, PressureForce, 0.5);
			DampingForce = FMath::Lerp<FVector>(TireImpact->LastDamping, DampingForce, 0.5);
			
			TireImpact->LastSpring = PressureForce;
			TireImpact->LastDamping = DampingForce;

			SpringStack.Add(TTuple<PxVec3, PxVec3>(U2PVector(PressureForce + DampingForce), U2PVector(TireImpact->EndPoint)));
			/************************************/

			TotalSpringForce += PressureForce.Size();
			TotalDampForce += DampingForce.Size();

			TotalTraceHit++;

			if (DebugDraws) {
				FColor LoadColor = FColor::Black;
				FColor FrictionColor = FColor::Black;

				LoadColor += FColor(0, 255*(1 - TireImpact->Compression), 0);
				LoadColor += FColor(TireImpact->Compression * 255, 0, 0);

				if (Slipping) FrictionColor = FColor::Blue;
				else         FrictionColor = FColor::Orange;

							

				DrawDebugLine(GetWorld(), TireImpact->EndPoint, TireImpact->EndPoint + PatchNormal * -WheelTireRadius * (TireImpact->Compression + .1), LoadColor, false, 0, 0, TireImpact->Compression*1. + .1);

				DrawDebugLine(GetWorld(), TireImpact->EndPoint, TireImpact->EndPoint - FrictionForce.GetSafeNormal()* (FMath::Pow(FrictionForce.Size(),1/2.)/10.), FrictionColor, false, 0, 0, 0.2);
			}
		}

		/****************/

		// DEBUG DRAWING

	}

	AddDebugData("TotalHitTraces", TotalTraceHit);

	MaxTireSpringForce = FMath::Max<float>(TotalSpringForce, MaxTireSpringForce);
	MaxTireDampForce = FMath::Max<float>(TotalDampForce, MaxTireDampForce);
	MaxFrictionVelForce = FMath::Max<float>(TotalFrictionVelocityForce, MaxFrictionVelForce * (TotalGripStrength/TotalTraceHit));

	SpringHistory.AddSample(TotalSpringForce);
	DampHistory.AddSample(TotalDampForce);
	FrictionHistory.AddSample(TotalFrictionVelocityForce);

	if (SubstepIndex == 0 && DebugLogs) {
		LOGE("%s SLIP : %d", *(WheelMesh->GetName()), (uint32)MaxVelocity);
  		LOGW("TOTSPRING : %d\t\tTOTDAMP : %d   \tTOTFRIC : %d", (uint32)(MaxTireSpringForce), (uint32)(MaxTireDampForce), (uint32)(MaxFrictionVelForce));

		
		MaxFrictionLockForce = 0;
		MaxFrictionVelForce = 0;
		MaxTireDampForce = 0;
		MaxTireSpringForce = 0;
		MaxVelocity = 0.;
	}

	// Apply friction stack
	if (TotalTraceHit > 0) {
		for (auto Tuple : FrictionStack) {
			PxRigidBodyExt::addForceAtPos(*WRigidBody, Tuple.Get<0>()/ TotalTraceHit, Tuple.Get<1>());
		}

		for (auto Tuple : SpringStack) {
			PxRigidBodyExt::addForceAtPos(*WRigidBody, Tuple.Get<0>(), Tuple.Get<1>());
		}
	}

	CompositedText = *WheelComponentName + 
					 FString("\nHITS : ") + FString::FromInt(TotalTraceHit) + 
					 FString("\nSPRNG : ") + FString::FromInt(TotalSpringForce) + 
					 FString("\nDAMP : ") + FString::FromInt(TotalDampForce) + 
					 FString("\nFRIC : ") + FString::FromInt((TotalFrictionVelocityForce/TotalGripStrength));
	
	SubstepIndex++;
}
