#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Templates/SubclassOf.h"
#include "BaselineEditorTools.generated.h"

UCLASS()
class ARUCOLANDINGBASELINE_API UBaselineEditorTools : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    // EditorLevelLibrary's placement path dereferences a viewport in UE4.27
    // commandlets. This direct UWorld path is deterministic and headless-safe.
    UFUNCTION(BlueprintCallable, Category = "Aruco Landing|Editor")
    static AActor* SpawnActorDirect(
        UObject* WorldContextObject,
        TSubclassOf<AActor> ActorClass,
        FVector Location,
        FRotator Rotation
    );

    UFUNCTION(BlueprintCallable, Category = "Aruco Landing|Editor")
    static bool DestroyActorDirect(AActor* Actor);
};
