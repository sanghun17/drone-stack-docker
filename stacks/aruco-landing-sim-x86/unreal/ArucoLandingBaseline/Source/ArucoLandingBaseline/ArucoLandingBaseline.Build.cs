using UnrealBuildTool;

public class ArucoLandingBaseline : ModuleRules
{
    public ArucoLandingBaseline(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        PublicDependencyModuleNames.AddRange(
            new string[] { "Core", "CoreUObject", "Engine", "InputCore", "AirSim" }
        );
    }
}
