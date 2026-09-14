using UnrealBuildTool;
using System.Collections.Generic;

public class ArucoLandingBaselineEditorTarget : TargetRules
{
    public ArucoLandingBaselineEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V2;
        ExtraModuleNames.Add("ArucoLandingBaseline");
    }
}
