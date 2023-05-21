// Fill out your copyright notice in the Description page of Project Settings.


#include "MySceneComponent.h"
#include "Components/LightComponent.h"

// Sets default values for this component's properties
UMySceneComponent::UMySceneComponent()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;

	// ...
}


// Called when the game starts
void UMySceneComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
	
}


// Called every frame
void UMySceneComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{

	UPROPERTY(EditAnywhere)
	float intense = 1.0f;

	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	FVector vec = GetRelativeLocation();
	vec.Z = 20;
	this->SetRelativeLocation(vec);
	ULightComponent* lightning = this->GetOwner()->FindComponentByClass<ULightComponent>();
	lightning->SetIntensity(intense);

//	this->GetOwner()->SetActorLocation(FVector(0.f, 0.f, -2000.f));
	// ...
}

