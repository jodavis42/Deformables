#include "SpringsPrecompiled.hpp"

ZilchDefineExternalBaseType(IntegrationMethod::Enum, Zilch::TypeCopyMode::ValueType, builder, type)
{
  ZilchFullBindEnum(builder, type, Zilch::SpecialType::Enumeration);
  ZilchBindEnumValues(IntegrationMethod);
}

//***************************************************************************
ZilchDefineStaticLibraryAndPlugin(SpringsLibrary, SpringsPlugin, ZilchDependencyStub(Core) ZilchDependencyStub(ZeroEngine))
{
  ZilchInitializeType(ClothSpringSystem);
  ZilchInitializeType(ClothSpringSystemEvent);
  ZilchInitializeType(ClothWind);
  ZilchInitializeEnum(IntegrationMethod);
  // Auto Initialize (used by Visual Studio plugins, do not remove this line)
}

//***************************************************************************
void SpringsPlugin::Initialize()
{
  // One time startup logic goes here
  // This runs after our plugin library/reflection is built
  Zilch::Console::WriteLine("SpringsPlugin::Initialize");
}

//***************************************************************************
void SpringsPlugin::Uninitialize()
{
  // One time shutdown logic goes here
  Zilch::Console::WriteLine("SpringsPlugin::Uninitialize");
}
