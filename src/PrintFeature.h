//Copyright (c) 2021 PICASO 3D
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

	Skin = 9, // Skin Top/Bottom 
	Roofing = 10, // Surface Roofing
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

	Flooring = 26, // Surface Flooring

	Inset0SeamCrossStart = 27,
	Inset0SeamCrossFinish = 28,

	NoneType = 29,

	NumPathConfigFeatures = 30, // this number MUST be the last one
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


    NumPrintFeatureTypes = 12 // this number MUST be the last one because other modules will
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

    NumPicasoSpeedProfiles = 7, // this number MUST be the last one
};

enum class PicasoPrintMode : unsigned char
{
    HighQuality = 0,
    Standart = 1,
    Fast = 2,
    Draft = 3,

    NumPicasoPrintModes = 4, // this number MUST be the last one
};



class PrintFeatureConverter
{
private:
	PrintFeatureConverter() {}

public:

	static PrintFeatureType toPrintFeatureType(const PathConfigFeature& feature)
	{
		switch (feature)
		{
		case PathConfigFeature::Inset0: return PrintFeatureType::OuterWall;
		case PathConfigFeature::Inset1: return PrintFeatureType::OuterWall;
		case PathConfigFeature::InsetX: return PrintFeatureType::InnerWall;

		case PathConfigFeature::Inset0SeamCrossStart: return PrintFeatureType::OuterWall;
		case PathConfigFeature::Inset0SeamCrossFinish: return PrintFeatureType::OuterWall;

		case PathConfigFeature::BridgeInset0: return PrintFeatureType::OuterWall;
		case PathConfigFeature::BridgeInset1: return PrintFeatureType::OuterWall;
		case PathConfigFeature::BridgeInsetX: return PrintFeatureType::InnerWall;
		case PathConfigFeature::BridgeSkin1: return PrintFeatureType::Skin;
		case PathConfigFeature::BridgeSkin2: return PrintFeatureType::Skin;
		case PathConfigFeature::BridgeSkin3: return PrintFeatureType::Skin;

		case PathConfigFeature::Skin: return PrintFeatureType::Skin;
		case PathConfigFeature::Roofing: return PrintFeatureType::Skin;
		case PathConfigFeature::Flooring: return PrintFeatureType::Skin;
		case PathConfigFeature::Infill: return PrintFeatureType::Infill;
		case PathConfigFeature::Ironing: return PrintFeatureType::Skin;
		case PathConfigFeature::PerimeterGap: return PrintFeatureType::Skin;

		case PathConfigFeature::RaftBase: return PrintFeatureType::SupportInterface;
		case PathConfigFeature::RaftInterface: return PrintFeatureType::Support;
		case PathConfigFeature::RaftSurface: return PrintFeatureType::SupportInterface;

		case PathConfigFeature::ExtruderTravel: return PrintFeatureType::MoveCombing;
		case PathConfigFeature::ExtruderSkirtBrim: return PrintFeatureType::SkirtBrim;
		case PathConfigFeature::ExtruderPrimeTower: return PrintFeatureType::PrimeTower;

		case PathConfigFeature::SupportRoof: return PrintFeatureType::SupportInterface;
		case PathConfigFeature::SupportInfill: return PrintFeatureType::Support;
		case PathConfigFeature::SupportBottom: return PrintFeatureType::SupportInterface;
		case PathConfigFeature::SupportUnderRoof: return PrintFeatureType::SupportInterface;
		case PathConfigFeature::SupportAboveBottom: return PrintFeatureType::SupportInterface;

		case PathConfigFeature::MoveRetraction: return PrintFeatureType::MoveRetraction;

		default:
			return PrintFeatureType::NoneType;
		}
	}

	static PicasoSpeedProfile toPicasoSpeedProfile(const PathConfigFeature& feature)
	{
		switch (feature)
		{
		case PathConfigFeature::Inset0: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::Inset1: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::InsetX: return PicasoSpeedProfile::Loops;

		case PathConfigFeature::Inset0SeamCrossStart: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::Inset0SeamCrossFinish: return PicasoSpeedProfile::Perimeter;

		case PathConfigFeature::BridgeInset0: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::BridgeInset1: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::BridgeInsetX: return PicasoSpeedProfile::Loops;
		case PathConfigFeature::BridgeSkin1: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::BridgeSkin2: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::BridgeSkin3: return PicasoSpeedProfile::Perimeter;

		case PathConfigFeature::Skin: return PicasoSpeedProfile::Loops;
		case PathConfigFeature::Roofing: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::Flooring: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::Infill: return PicasoSpeedProfile::Infill;
		case PathConfigFeature::Ironing: return PicasoSpeedProfile::Perimeter;
		case PathConfigFeature::PerimeterGap: return PicasoSpeedProfile::Perimeter;

		case PathConfigFeature::RaftBase: return PicasoSpeedProfile::InterfaceSupport;
		case PathConfigFeature::RaftInterface: return PicasoSpeedProfile::Support;
		case PathConfigFeature::RaftSurface: return PicasoSpeedProfile::InterfaceSupport;

		case PathConfigFeature::ExtruderTravel: return PicasoSpeedProfile::Travel;
		case PathConfigFeature::ExtruderSkirtBrim: return PicasoSpeedProfile::Loops;
		case PathConfigFeature::ExtruderPrimeTower: return PicasoSpeedProfile::Support;

		case PathConfigFeature::SupportRoof: return PicasoSpeedProfile::InterfaceSupport;
		case PathConfigFeature::SupportInfill: return PicasoSpeedProfile::Support;
		case PathConfigFeature::SupportBottom: return PicasoSpeedProfile::InterfaceSupport;
		case PathConfigFeature::SupportUnderRoof: return PicasoSpeedProfile::InterfaceSupport;
		case PathConfigFeature::SupportAboveBottom: return PicasoSpeedProfile::InterfaceSupport;

		case PathConfigFeature::MoveRetraction: return PicasoSpeedProfile::Travel;

		case PathConfigFeature::NoneType:
		case PathConfigFeature::NumPathConfigFeatures:
		default: 
			return PicasoSpeedProfile::Undefined;
		}
	}

	static PicasoSpeedProfile toPicasoSpeedProfile(const PrintFeatureType& type)
	{
		switch (type)
		{
		case PrintFeatureType::OuterWall:
		case PrintFeatureType::Skin:
			return PicasoSpeedProfile::Perimeter;

		case PrintFeatureType::InnerWall:
		case PrintFeatureType::SkirtBrim:
			return PicasoSpeedProfile::Loops;

		case PrintFeatureType::Support:
		case PrintFeatureType::SupportInfill:
			return PicasoSpeedProfile::Support;

		case PrintFeatureType::SupportInterface:
			return PicasoSpeedProfile::InterfaceSupport;

		case PrintFeatureType::Infill:
			return PicasoSpeedProfile::Infill;

		case PrintFeatureType::MoveCombing:
		case PrintFeatureType::MoveRetraction:
			return PicasoSpeedProfile::Travel;

		case PrintFeatureType::NoneType:
		case PrintFeatureType::NumPrintFeatureTypes:
		default:
			return PicasoSpeedProfile::Undefined;
		}
	}

	static bool allowSpeedScale(const PathConfigFeature& feature)
	{
		switch (feature)
		{
		case PathConfigFeature::Inset0: return true;
		case PathConfigFeature::Inset1: return true;
		case PathConfigFeature::InsetX: return true;

		case PathConfigFeature::Inset0SeamCrossStart: return true;
		case PathConfigFeature::Inset0SeamCrossFinish: return true;

		case PathConfigFeature::BridgeInset0: return false;
		case PathConfigFeature::BridgeInset1: return false;
		case PathConfigFeature::BridgeInsetX: return false;
		case PathConfigFeature::BridgeSkin1: return false;
		case PathConfigFeature::BridgeSkin2: return false;
		case PathConfigFeature::BridgeSkin3: return false;

		case PathConfigFeature::Skin: return true;
		case PathConfigFeature::Roofing: return true;
		case PathConfigFeature::Flooring: return true;
		case PathConfigFeature::Infill: return true;
		case PathConfigFeature::Ironing: return false;
		case PathConfigFeature::PerimeterGap: return true;

		case PathConfigFeature::RaftBase: return false;
		case PathConfigFeature::RaftInterface: return false;
		case PathConfigFeature::RaftSurface: return false;

		case PathConfigFeature::ExtruderTravel: return false;
		case PathConfigFeature::ExtruderSkirtBrim: return false;
		case PathConfigFeature::ExtruderPrimeTower: return true;

		case PathConfigFeature::SupportRoof: return true;
		case PathConfigFeature::SupportInfill: return true;
		case PathConfigFeature::SupportBottom: return true;
		case PathConfigFeature::SupportUnderRoof: return true;
		case PathConfigFeature::SupportAboveBottom: return true;

		default: return false;
		}
	}

};

} // namespace cura

#endif // PRINT_FEATURE
