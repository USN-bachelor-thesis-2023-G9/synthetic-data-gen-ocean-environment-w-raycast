// Fill out your copyright notice in the Description page of Project Settings.


#include "FileWriter.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformFileManager.h"
#include "api/RpcLibClientBase.hpp"

const bool test = false;

bool UFileWriter::WriteResults(FString FileDirectory, FString MetaDirectory, FString FileName, TArray<FString> ActorName, TArray<float> Values, bool AllowOverwrite = false) {
	FString MetaFile =	MetaDirectory + "\\" + FileName + ".csv";
	FileDirectory += "\\";
	FileDirectory += FileName;

	//return true; //UNCOMMENT WHEN WORKING WITH BLUEPRINTS

	if (!AllowOverwrite) {
		if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*FileDirectory)) {
			return false;
		}
	}

	TArray<FString> validActors;
	if (test) {
		validActors.Add("Cube");
		validActors.Add("Cylinder");
		validActors.Add("Cone");
		validActors.Add("Sphere");
	}
	else {
		validActors.Add("human");
		validActors.Add("boat");
	}

	FString FinalString = "";
	FString MetaString = "annotation_id,depth";
	MetaString += LINE_TERMINATOR;
	int actors = ActorName.Num();
	int annotationCount = 0;
	for (int i = 0; i < actors; i++) {
		FString name = ActorName[i];

		for (int j = 0; j < validActors.Num(); j++) {
			if (name.StartsWith(validActors[j])) {
				FinalString.AppendInt(j);
				break;
			}
		}
		if (FinalString.IsEmpty() || FinalString.EndsWith(LINE_TERMINATOR)) {
			UE_LOG(LogTemp, Warning, TEXT("Could not infer type. Skipping."));
			continue;
		}

		int nums = Values.Num() / actors;
		for (int j = nums * i; j < nums*(i+1)-1; j++) {
			FinalString += " ";
			FinalString += std::to_string(Values[j]).c_str();
		}

		MetaString += std::to_string(annotationCount).c_str();
		MetaString += ",";
		MetaString += std::to_string(Values[nums * (i + 1) - 1]).c_str();
		MetaString += LINE_TERMINATOR;
		FinalString += LINE_TERMINATOR;
		annotationCount++;
	}

	if (!FFileHelper::SaveStringToFile(FinalString, *FileDirectory)) {
		return false;
	}

	if (!FFileHelper::SaveStringToFile(MetaString, *MetaFile)) {
		return false;
	}

	msr::airlib::RpcLibClientBase client;
	client.simPause(true);

	return true;
}

