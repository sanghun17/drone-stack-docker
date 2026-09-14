using UnrealBuildTool;
using System.Collections.Generic;

public class ArucoLandingBaselineTarget : TargetRules
{
    public ArucoLandingBaselineTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.Add("ArucoLandingBaseline");
        if (Target.Platform == UnrealTargetPlatform.Linux)
            bUsePCHFiles = false;
    }
}
