// Fill out your copyright notice in the Description page of Project Settings.


#include "DetectionFetcher.h"
#include "api/RpcLibClientBase.hpp"
#include<vector>

// Sets default values
ADetectionFetcher::ADetectionFetcher()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADetectionFetcher::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ADetectionFetcher::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	if(Running && !Completed && runner->IsDone()){
		Running = false;
		Completed = true;
		delete runner;
	}
}

void ADetectionFetcher::RunFetcherTask()
{
	Detections.Empty();
	runner = new FAsyncTask<FetcherTask>(this);
	runner->StartBackgroundTask();
	Running = true;
	Completed = false;
}

FetcherTask::FetcherTask(ADetectionFetcher* ref)
{
	UE_LOG(LogTemp, Warning, TEXT("Fetcher task constructed"));
	this->ref = ref;
}

FetcherTask::~FetcherTask()
{
	UE_LOG(LogTemp, Warning, TEXT("Fetcher task destructed"));
}


void FetcherTask::DoWork()
{
	msr::airlib::RpcLibClientBase client;
	client.confirmConnection();
	const std::string camera_name = "0";
	msr::airlib::ImageCaptureBase::ImageType detectionImageType = msr::airlib::ImageCaptureBase::ImageType::Scene;

	std::vector<msr::airlib::DetectionInfo> infoList = client.simGetDetections("0", msr::airlib::ImageCaptureBase::ImageType::Scene, "", false);
	for (auto i = infoList.begin(); i != infoList.end(); ++i){
		msr::airlib::Box2D box = i->box2D;
		msr::airlib::Vector2r min = box.min;
		msr::airlib::Vector2r max = box.max;
		ref->Detections.Add(FVector2D(min[0],min[1]));
		ref->Detections.Add(FVector2D(max[0],max[1]));
	}
}
