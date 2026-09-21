#include "BaselineEditorTools.h"

#include "Engine/World.h"
#include "GameFramework/Actor.h"

AActor* UBaselineEditorTools::SpawnActorDirect(
    UObject* WorldContextObject,
    TSubclassOf<AActor> ActorClass,
    FVector Location,
    FRotator Rotation
)
{
    if (!WorldContextObject || !ActorClass)
    {
        return nullptr;
    }
    UWorld* World = WorldContextObject->GetWorld();
    if (!World)
    {
        return nullptr;
    }
    FActorSpawnParameters Parameters;
    Parameters.OverrideLevel = World->GetCurrentLevel();
    Parameters.ObjectFlags |= RF_Transactional;
    return World->SpawnActor<AActor>(ActorClass, Location, Rotation, Parameters);
}

bool UBaselineEditorTools::DestroyActorDirect(AActor* Actor)
{
    return Actor && Actor->Destroy();
}
