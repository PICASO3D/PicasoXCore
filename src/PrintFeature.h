//Copyright (c) 2020 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#ifndef PRINT_FEATURE
#define PRINT_FEATURE

namespace cura
{

enum class PathConfigFeature : unsigned char
{
	Inset0 = 0,
	Inset1 = 1,
	InsetX = 2,

	BridgeInset0 = 3,
	BridgeInset1 = 4,
	BridgeInsetX = 5,
	BridgeSkin1 = 6,
	BridgeSkin2 = 7,
	BridgeSkin3 = 8,

	Skin = 9,
	Roofing = 10,
	Infill = 11,
	Ironing = 12,
	PerimeterGap = 13,

	RaftBase = 14,
	RaftInterface = 15,
	RaftSurface = 16,

	ExtruderTravel = 17, // all movement with/out retracted E axis
	ExtruderSkirtBrim = 18,
	ExtruderPrimeTower = 19,

	SupportRoof = 20,
	SupportInfill = 21,
	SupportBottom = 22,
	SupportUnderRoof = 23,
	SupportAboveBottom = 24,

	MoveRetraction = 25, // E axis movement

	NoneType = 26,
};

enum class PrintFeatureType: unsigned char
{
    NoneType = 0, // used to mark unspecified jumps in polygons. libArcus depends on it
    OuterWall = 1,
    InnerWall = 2,
    Skin = 3,
    Support = 4,
    SkirtBrim = 5,
    Infill = 6,
    SupportInfill = 7,
    MoveCombing = 8,
    MoveRetraction = 9,
    SupportInterface = 10,
	PrimeTower = 11,
    Retract = 12,
    ZHopp = 13,

    NumPrintFeatureTypes = 14 // this number MUST be the last one because other modules will
                              // use this symbol to get the total number of types, which can
                              // be used to create an array or so
};

enum class PicasoSpeedProfile : unsigned char
{
    Undefined = 0,              // G0 Retract/ZHopp movements and mil layer time wait
    Perimeter = 1,              // Outer perimeters
    Loops = 2,                  // Inner perimeters
    Support = 3,                // support
    InterfaceSupport = 4,       // interface support
    Infill = 5,                 // infill
	Travel = 6,                 // Travel movies
    Counters = 7,               // !!! counters, not a profile

    NumPicasoSpeedProfiles = 8, // this number MUST be the last one
};

enum class PicasoPrintMode : unsigned char
{
    HighQuality = 0,
    Standart = 1,
    Fast = 2,
    Draft = 3,

    NumPicasoPrintModes = 4, // this number MUST be the last one
};


} // namespace cura

#endif // PRINT_FEATURE
