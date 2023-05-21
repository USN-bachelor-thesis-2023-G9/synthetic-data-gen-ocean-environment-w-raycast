// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Async/AsyncWork.h"
#include "DetectionFetcher.generated.h"


class FetcherTask : public FNonAbandonableTask{
friend class FAsyncTask<FetcherTask>;
public:
	ADetectionFetcher* ref;

	FetcherTask(ADetectionFetcher* ref);
	~FetcherTask();

	//Required by UE4
	FORCEINLINE TStatId GetStatId() const {
		RETURN_QUICK_DECLARE_CYCLE_STAT(FetcherTask, STATGROUP_ThreadPoolAsyncTasks)
	}

	void DoWork();
};

UCLASS()
class SIMPLEENVIRONMENT_API ADetectionFetcher : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADetectionFetcher();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable)
	void RunFetcherTask();

	UPROPERTY(BlueprintReadOnly)
	bool Running;

	UPROPERTY(BlueprintReadOnly)
	bool Completed;

	UPROPERTY(BlueprintReadOnly)
	TArray<FVector2D> Detections;

	FAsyncTask<FetcherTask>* runner;

};