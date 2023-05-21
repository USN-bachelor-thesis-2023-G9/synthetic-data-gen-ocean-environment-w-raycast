// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "FileWriter.generated.h"

/**
 * 
 */
UCLASS()
class SIMPLEENVIRONMENT_API UFileWriter : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

	UFUNCTION(BlueprintCallable, Category = "Custom", meta = (Keywords = "Files"))
	static bool WriteResults(FString FileDirectory, FString MetaDirectory, FString FileName, TArray<FString> ActorName, TArray<float> Values, bool AllowOverwrite);
};
