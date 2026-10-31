#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "VrmPmxMaterial.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVrmPmxMaterialTest, "VRM4U.PMX.MaterialColorAndOpacity",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FVrmPmxMaterialTest::RunTest(const FString& Parameters)
{
	aiMaterial Material;
	TestEqual(TEXT("Missing properties stay white and opaque"), ReadPmxMaterialProperties(Material).Color, FLinearColor::White);
	aiColor3D Diffuse(0.2f, 0.4f, 0.6f);
	Material.AddProperty(&Diffuse, 1, AI_MATKEY_COLOR_DIFFUSE);
	for (float Opacity : {0.f, 0.5f, 1.f})
	{
		Material.AddProperty(&Opacity, 1, AI_MATKEY_OPACITY);
		TestEqual(TEXT("Authored diffuse and hidden/translucent/opaque values survive"),
			ReadPmxMaterialProperties(Material).Color, FLinearColor(0.2f, 0.4f, 0.6f, Opacity));
	}
	for (int Enabled : {0, 1})
	{
		Material.AddProperty(&Enabled, 1, AI_MATKEY_TWOSIDED);
		Material.AddProperty(&Enabled, 1, "$mat.mmd.edgeEnabled", 0, 0);
		TestEqual(TEXT("Authored culling survives"), ReadPmxMaterialProperties(Material).bTwoSided, Enabled != 0);
		TestEqual(TEXT("Authored outline switch survives"), ReadPmxMaterialProperties(Material).bEdgeEnabled, Enabled != 0);
	}
	float EdgeSize = 0.618f;
	aiColor4D EdgeColor(0.1f, 0.2f, 0.3f, 0.4f);
	Material.AddProperty(&EdgeSize, 1, "$mat.mmd.edgeSize", 0, 0);
	Material.AddProperty(&EdgeColor, 1, "$mat.mmd.edgeColor", 0, 0);
	TestEqual(TEXT("Authored outline size survives"), ReadPmxMaterialProperties(Material).EdgeSize, EdgeSize);
	TestEqual(TEXT("Authored outline color survives"), ReadPmxMaterialProperties(Material).EdgeColor, FLinearColor(0.1f, 0.2f, 0.3f, 0.4f));
	TestEqual(TEXT("Missing sphere map is disabled"), ReadPmxMaterialProperties(Material).SphereMode, 0);
	for (int Mode : {0, 1, 2, 3, -1})
	{
		Material.AddProperty(&Mode, 1, "$mat.mmd.sphereMode", 0, 0);
		TestEqual(TEXT("Only supported sphere blend modes are exposed"), ReadPmxMaterialProperties(Material).SphereMode,
			Mode == 1 || Mode == 2 ? Mode : 0);
	}
	return true;
}
#endif
