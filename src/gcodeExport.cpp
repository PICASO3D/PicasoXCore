//Copyright (c) 2020 Ultimaker B.V.
//Copyright (c) 2021 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#include <assert.h>
#include <cmath>
#include <iomanip>
#include <ctime>
#include <stdarg.h>

#include "Application.h" //To send layer view data.
#include "ExtruderTrain.h"
#include "gcodeExport.h"
#include "PrintFeature.h"
#include "RetractionConfig.h"
#include "Slice.h"
#include "raft.h"
#include "sliceDataStorage.h"
#include "communication/Communication.h" //To send layer view data.
#include "settings/types/LayerIndex.h"
#include "utils/Date.h"
#include "utils/logoutput.h"
#include "utils/string.h" // MMtoStream, PrecisionedDouble
#include "utils/md5.h" // MD5
#include "WipeScriptConfig.h"

#ifndef PICASO_HW_RETRACT
#define PICASO_HW_RETRACT
#endif

#ifndef PICASO_HW_ZHOPP
#define PICASO_HW_ZHOPP
#endif

namespace cura {

std::string transliterate(const std::string& text)
{
    // For now, just replace all non-ascii characters with '?'.
    // This function can be expaned if we need more complex transliteration.
    std::ostringstream stream;
    for (const char& c : text)
    {
        stream << static_cast<char>((c >= 0) ? c : '?');
    }
    return stream.str();
}

GCodeExport::GCodeExport()
: output_stream(&std::cout)
, currentPosition(0,0,MM2INT(20))
, layer_nr(0)
, zero_based_layer_nr(-1)
, relative_extrusion(false)
{
    *output_stream << std::fixed;

    current_e_value = 0;
    current_extruder = 0;
    current_fan_speed = -1;

    total_print_times = std::vector<Duration>(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);

    currentSpeed = 1;
    current_print_acceleration = -1;
    current_travel_acceleration = -1;
    current_jerk = -1;

    is_z_hopped = 0;
    setFlavor(EGCodeFlavor::MARLIN);
    initial_bed_temp = 0;
    build_volume_temperature = 0;
    machine_heated_build_volume = false;

    fan_number = 0;
    use_extruder_offset_to_offset_coords = false;
    machine_name = "";
    machine_buildplate_type = "";
    relative_extrusion = false;
    new_line = "\n";

    total_bounding_box = AABB3D();

    // HighQuality
    picasoPrintModeSpeed[(int)PicasoPrintMode::HighQuality][(int)PicasoSpeedProfile::Perimeter] = 20.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::HighQuality][(int)PicasoSpeedProfile::Loops] = 20.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::HighQuality][(int)PicasoSpeedProfile::Infill] = 30.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::HighQuality][(int)PicasoSpeedProfile::Support] = 20.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::HighQuality][(int)PicasoSpeedProfile::InterfaceSupport] = 20.0;
    // Standart
    picasoPrintModeSpeed[(int)PicasoPrintMode::Standart][(int)PicasoSpeedProfile::Perimeter] = 20.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Standart][(int)PicasoSpeedProfile::Loops] = 60.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Standart][(int)PicasoSpeedProfile::Infill] = 60.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Standart][(int)PicasoSpeedProfile::Support] = 60.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Standart][(int)PicasoSpeedProfile::InterfaceSupport] = 20.0;
    // Fast
    picasoPrintModeSpeed[(int)PicasoPrintMode::Fast][(int)PicasoSpeedProfile::Perimeter] = 30.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Fast][(int)PicasoSpeedProfile::Loops] = 80.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Fast][(int)PicasoSpeedProfile::Infill] = 80.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Fast][(int)PicasoSpeedProfile::Support] = 80.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Fast][(int)PicasoSpeedProfile::InterfaceSupport] = 30.0;
    // Draft
    picasoPrintModeSpeed[(int)PicasoPrintMode::Draft][(int)PicasoSpeedProfile::Perimeter] = 60.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Draft][(int)PicasoSpeedProfile::Loops] = 80.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Draft][(int)PicasoSpeedProfile::Infill] = 80.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Draft][(int)PicasoSpeedProfile::Support] = 80.0;
    picasoPrintModeSpeed[(int)PicasoPrintMode::Draft][(int)PicasoSpeedProfile::InterfaceSupport] = 60.0;
}

GCodeExport::~GCodeExport()
{
}

void GCodeExport::preSetup(const size_t start_extruder)
{
    current_extruder = start_extruder;

    const Scene& scene = Application::getInstance().current_slice->scene;
    std::vector<MeshGroup>::iterator mesh_group = scene.current_mesh_group;
    setFlavor(mesh_group->settings.get<EGCodeFlavor>("machine_gcode_flavor"));

    firmware_retract = mesh_group->settings.get<bool>("machine_firmware_retract");
    firmware_zhopp = mesh_group->settings.get<bool>("machine_firmware_zhopp");
    unretract_after_change = mesh_group->settings.get<bool>("machine_unretract_after_change");

    use_extruder_offset_to_offset_coords = mesh_group->settings.get<bool>("machine_use_extruder_offset_to_offset_coords");
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();

    hop_when_extruder_switch = mesh_group->settings.get<bool>("retraction_hop_when_extruder_switch");

    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = scene.extruders[extruder_nr];
        setFilamentDiameter(extruder_nr, train.settings.get<coord_t>("material_diameter"));

        extruder_attr[extruder_nr].last_retraction_prime_speed = train.settings.get<Velocity>("retraction_prime_speed"); // the alternative would be switch_extruder_prime_speed, but dual extrusion might not even be configured...
        extruder_attr[extruder_nr].fan_number = train.settings.get<size_t>("machine_extruder_cooling_fan_number");

        if (getFlavor() == EGCodeFlavor::PICASO) {
            // pre defined all extruders start with single retraction (on tool change T10/11)
            extruder_attr[extruder_nr].retraction_e_amount_current = 1.0;
        }
    }

    machine_name = mesh_group->settings.get<std::string>("machine_name");
    machine_buildplate_type = mesh_group->settings.get<std::string>("machine_buildplate_type");

    int comment_ind = 0;
    while (comment_ind < 10)
    {
        std::ostringstream comment_name;
        comment_name << "machine_comment_" << comment_ind;
        const std::string comment_value = mesh_group->settings.get<std::string>(comment_name.str());
        //logAlways(comment_name.str().c_str());
        //logAlways(comment_value.c_str());
        if (comment_value == "" || (comment_ind + 1) >= 10)
            break;
        custom_comment.push_back(comment_value);
        comment_ind++;
    }

    relative_extrusion = mesh_group->settings.get<bool>("relative_extrusion");
    always_write_active_tool = mesh_group->settings.get<bool>("machine_always_write_active_tool");

    if (flavor == EGCodeFlavor::BFB)
    {
        new_line = "\r\n";
    }
    else 
    {
        new_line = "\n";
    }

	total_print_times_pathConfig.ReInit(mesh_group->settings.get<size_t>("machine_extruder_count"));

    estimateCalculator.setFirmwareDefaults(mesh_group->settings);
}

void GCodeExport::setInitialAndBuildVolumeTemps(const unsigned int start_extruder_nr)
{
    const Scene& scene = Application::getInstance().current_slice->scene;
    const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
    for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
    {
        const ExtruderTrain& train = scene.extruders[extruder_nr];

        const Temperature print_temp_0 = train.settings.get<Temperature>("material_print_temperature_layer_0");
        const Temperature print_temp_here = (print_temp_0 != 0)? print_temp_0 : train.settings.get<Temperature>("material_print_temperature");
        const Temperature temp = (extruder_nr == start_extruder_nr)? print_temp_here : train.settings.get<Temperature>("material_standby_temperature");
        setInitialTemp(extruder_nr, temp);
    }

    initial_bed_temp = scene.current_mesh_group->settings.get<Temperature>("material_bed_temperature_layer_0");
    machine_heated_build_volume = scene.current_mesh_group->settings.get<bool>("machine_heated_build_volume");
    build_volume_temperature = machine_heated_build_volume ? scene.current_mesh_group->settings.get<Temperature>("build_volume_temperature") : Temperature(0);
}

void GCodeExport::setInitialTemp(int extruder_nr, double temp)
{
    extruder_attr[extruder_nr].initial_temp = temp;
    if (flavor == EGCodeFlavor::GRIFFIN || flavor == EGCodeFlavor::ULTIGCODE)
    {
        extruder_attr[extruder_nr].currentTemperature = temp;
    }
}

const std::string GCodeExport::flavorToString(const EGCodeFlavor& flavor) const
{
    switch (flavor)
    {
        case EGCodeFlavor::BFB:
            return "BFB";
        case EGCodeFlavor::MACH3:
            return "Mach3";
        case EGCodeFlavor::MAKERBOT:
            return "Makerbot";
        case EGCodeFlavor::ULTIGCODE:
            return "UltiGCode";
        case EGCodeFlavor::MARLIN_VOLUMATRIC:
            return "Marlin(Volumetric)";
        case EGCodeFlavor::GRIFFIN:
            return "Griffin";
        case EGCodeFlavor::REPETIER:
            return "Repetier";
        case EGCodeFlavor::REPRAP:
            return "RepRap";
        case EGCodeFlavor::PICASO:
            return "Picaso";
        case EGCodeFlavor::MARLIN:
        default:
            return "Marlin";
    }
}

std::string GCodeExport::getFileHeader(const std::vector<bool>& extruder_is_used, const Duration* print_time, const std::vector<double>& filament_used, const std::vector<std::string>& mat_ids)
{
	std::ostringstream prefix;

	const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();
	switch (flavor)
	{
	case EGCodeFlavor::GRIFFIN:
	{
		prefix << ";START_OF_HEADER" << new_line;
		prefix << ";HEADER_VERSION:0.1" << new_line;
		prefix << ";FLAVOR:" << flavorToString(flavor) << new_line;
		prefix << ";GENERATOR.NAME:PicasoXCore" << new_line;
		prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
		prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
		prefix << ";TARGET_MACHINE.NAME:" << transliterate(machine_name) << new_line;

		for (size_t extr_nr = 0; extr_nr < extruder_count; extr_nr++)
		{
			if (!extruder_is_used[extr_nr])
			{
				continue;
			}
			prefix << ";EXTRUDER_TRAIN." << extr_nr << ".INITIAL_TEMPERATURE:" << extruder_attr[extr_nr].initial_temp << new_line;
			if (filament_used.size() == extruder_count)
			{
				prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.VOLUME_USED:" << static_cast<int>(filament_used[extr_nr]) << new_line;
			}
			if (mat_ids.size() == extruder_count && mat_ids[extr_nr] != "")
			{
				prefix << ";EXTRUDER_TRAIN." << extr_nr << ".MATERIAL.GUID:" << mat_ids[extr_nr] << new_line;
			}
			const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extr_nr].settings;
			prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.DIAMETER:" << extruder_settings.get<double>("machine_nozzle_size") << new_line;
			prefix << ";EXTRUDER_TRAIN." << extr_nr << ".NOZZLE.NAME:" << extruder_settings.get<std::string>("machine_nozzle_id") << new_line;
		}
		prefix << ";BUILD_PLATE.TYPE:" << machine_buildplate_type << new_line;
		prefix << ";BUILD_PLATE.INITIAL_TEMPERATURE:" << initial_bed_temp << new_line;

        if (machine_heated_build_volume)
        {
            prefix << ";BUILD_VOLUME.TEMPERATURE:" << build_volume_temperature << new_line;
        }

		if (print_time)
		{
			prefix << ";PRINT.TIME:" << static_cast<int>(*print_time) << new_line;
		}

		prefix << ";PRINT.GROUPS:" << Application::getInstance().current_slice->scene.mesh_groups.size() << new_line;

		if (total_bounding_box.min.x > total_bounding_box.max.x) //We haven't encountered any movement (yet). This probably means we're command-line slicing.
		{
			//Put some small default in there.
			total_bounding_box.min = Point3(0, 0, 0);
			total_bounding_box.max = Point3(10, 10, 10);
		}
		prefix << ";PRINT.SIZE.MIN.X:" << INT2MM(total_bounding_box.min.x) << new_line;
		prefix << ";PRINT.SIZE.MIN.Y:" << INT2MM(total_bounding_box.min.y) << new_line;
		prefix << ";PRINT.SIZE.MIN.Z:" << INT2MM(total_bounding_box.min.z) << new_line;
		prefix << ";PRINT.SIZE.MAX.X:" << INT2MM(total_bounding_box.max.x) << new_line;
		prefix << ";PRINT.SIZE.MAX.Y:" << INT2MM(total_bounding_box.max.y) << new_line;
		prefix << ";PRINT.SIZE.MAX.Z:" << INT2MM(total_bounding_box.max.z) << new_line;
		prefix << ";END_OF_HEADER" << new_line;
	}
	break;

	case EGCodeFlavor::PICASO:
	{
		PicasoPrintTaskStats print_task_stats;
		print_task_stats.extruder_total_count = extruder_count;
		std::vector<PicasoExtruderStats>& extruder_stats = print_task_stats.extruder_stats;

		const Scene& scene = Application::getInstance().current_slice->scene;

		prefix << ";FLAVOR:" << flavorToString(flavor) << new_line;
		if (print_time)
		{
			prefix << ";PRINT.TIME:" << static_cast<int>(*print_time) << new_line;
		}
		prefix << ";GENERATOR.NAME:PicasoXCore" << new_line;
		prefix << ";GENERATOR.VERSION:" << VERSION << new_line;
		prefix << ";GENERATOR.BUILD_DATE:" << Date::getDate().toStringDashed() << new_line;
		prefix << ";MACHINE.NAME:" << machine_name << new_line;

		// <start_extruder_nr>
		size_t start_extruder_nr = 0;
		if (scene.mesh_groups.size() > 0 && scene.mesh_groups[0].meshes.size() > 0)
		{
			// First Model Extruder
			start_extruder_nr = scene.mesh_groups[0].meshes[0].settings.get<size_t>("extruder_nr");
		}
		if (scene.settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::NONE)
		{
			// Adhesion Extruder
			start_extruder_nr = scene.settings.get<size_t>("adhesion_extruder_nr");
		}
		print_task_stats.extruder_start = start_extruder_nr;
		// </start_extruder_nr>

		size_t used_extruders = 0;
		for (size_t extr_nr = 0; extr_nr < extruder_count; extr_nr++)
		{
			PicasoExtruderStats single_extruder_stats;
			single_extruder_stats.extruder_nr = extr_nr;
			single_extruder_stats.is_used = extruder_is_used[extr_nr];
			extruder_stats.push_back(single_extruder_stats);

			if (!extruder_is_used[extr_nr])
			{
				continue;
			}
			used_extruders++;

			prefix << ";EXTRUDER_" << extr_nr << "_TEMPERATURE=" << extruder_attr[extr_nr].initial_temp << new_line;
			if (filament_used.size() == extruder_count)
			{
				prefix << ";EXTRUDER_" << extr_nr << "_FILLAMENT=" << static_cast<int>(filament_used[extr_nr]) << new_line;
				extruder_stats[extr_nr].filament_volume = static_cast<int>(filament_used[extr_nr]);
			}
			if (getExtruderIsUsed(extr_nr))
			{
				const Settings& extruder_settings = scene.extruders[extr_nr].settings;
				prefix << ";EXTRUDER_" << extr_nr << "_DIAMETER=" << extruder_settings.get<double>("machine_nozzle_size") << new_line;
				extruder_stats[extr_nr].nozzle_size = static_cast<float>(extruder_settings.get<double>("machine_nozzle_size"));
			}
		}
		print_task_stats.extruder_used_count = used_extruders;

		for (unsigned int c_ind = 0; c_ind < custom_comment.size(); c_ind++) {
			const std::string comment_value = custom_comment[c_ind];
			//logAlways(comment_value.c_str());
			prefix << ";" << comment_value << new_line;
		}

		// <TIME_FEATURES>
		const PicasoPrintModeEstimate& total_estimates = getPicasoPrintModeEstimate(scene, total_print_times_pathConfig);
		std::vector<Duration> total_estimates_all_extruders(static_cast<unsigned char>(PicasoPrintMode::NumPicasoPrintModes), 0.0);

        for (auto it = total_estimates.durations.begin(); it != total_estimates.durations.end(); ++it)
        {
            size_t extruder_nr = it->first;
            PicasoExtruderStats& single_extruder_stats = extruder_stats[extruder_nr];

            //std::vector<PicasoProfileDurationStats> duration_stats;
            for (size_t n = 0; n < it->second.size(); n++)
            {
                //prefix << ";M1112 T" << extruder_nr
                //    << " S" << n
                //    << " P" << PrecisionedDouble{ 0, it->second[n] } << new_line;

                PicasoProfileDurationStats speed_profile_duration;
                speed_profile_duration.profile = n;
                speed_profile_duration.duration_sec = static_cast<size_t>(static_cast<double>(it->second[n]));
                single_extruder_stats.durations.push_back(speed_profile_duration);

                total_estimates_all_extruders[n] += it->second[n];
            }
        }

        print_task_stats.zhopp_count = total_estimates.zhopp_count;
		print_task_stats.retract_count = total_estimates.retract_count;

		// Old Format combined extruders
		for (size_t i = 0; i < (int)PicasoPrintMode::NumPicasoPrintModes; i++)
		{
			prefix << ";TASK_TIME_POLYGON_" << i << "=" << (int)((total_estimates_all_extruders[i] + total_estimates.extra_time) / 60) << new_line; // in minutes
		}

        if (scene.settings.get<bool>("cmd_enable_m1114")) // Report Layer Additional Counters
        {
            prefix << ";M1114"
                << " Z" << total_estimates.zhopp_count
                << " E" << total_estimates.retract_count
                << " P" << PrecisionedDouble{ 0, total_estimates.extra_time } << new_line;
        }

        // Format: FW 6.0 estimates
        if (scene.settings.get<bool>("cmd_enable_m1115")) // Report Total Durations
        {
            for (auto it = total_print_times_pathConfig.durations.begin(); it != total_print_times_pathConfig.durations.end(); ++it) // Nozzle
            {
                for (size_t n = 0; n < it->second.size(); n++) // PathConfigFeature
                {
                    const PathConfigFeature feature = static_cast<PathConfigFeature>(n);
                    const Duration duration(it->second[n]);
                    prefix << getFileDurationEstimateReport(it->first, feature, duration);
                }
            }
        }
		// </TIME_FEATURES>

		assert(used_extruders > 0 && extruder_count > 0 && extruder_count <= 2);

		// TID - TaskIdentifier
		std::string tid_value = scene.settings.get<std::string>("machine_tid");
		if (tid_value == "") {
			std::ostringstream hash_string;
			hash_string << "PicasoXCore_";
			auto t = std::time(nullptr);
			auto tm = *std::localtime(&t);
			hash_string << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");

			tid_value = md5(hash_string.str(), true);
		}
		prefix << ";TID: " << tid_value << new_line;
		print_task_stats.task_id = tid_value;

		if (extruder_count == 1)
		{
			prefix << ";POLYGON_ACTIVE_SOPLO=3" << new_line;
		}
		else // == 2
		{
			if (used_extruders == 2)
			{
				prefix << ";POLYGON_ACTIVE_SOPLO=" << (start_extruder_nr + 1) << new_line; // when 2 extruders select start extruder
			}
			else
			{
				prefix << ";POLYGON_ACTIVE_SOPLO=" << (start_extruder_nr + 3) << new_line; // when single extruder select 3/4
			}
		}

		prefix << ";POLYGON_FIRST_LAYER_HEIGHT=" << scene.current_mesh_group->settings.get<double>("layer_height_0") << new_line;
		print_task_stats.first_layer_height = static_cast<float>(scene.current_mesh_group->settings.get<double>("layer_height_0"));

		prefix << ";POLYGON_LAYER_HEIGHT=" << scene.current_mesh_group->settings.get<double>("layer_height") << new_line;
		print_task_stats.layer_height = static_cast<float>(scene.current_mesh_group->settings.get<double>("layer_height"));

		//sb.Append($";POLYGON_SOLUBLE=0{new_line}");
		//prefix << ";POLYGON_BASIC_FEEDRATE=" << Application::getInstance().current_slice->scene.current_mesh_group->settings.get<double>("speed_print") << new_line;
		prefix << ";POLYGON_END_PARAMS" << new_line;
		prefix << new_line;

        if (scene.settings.get<bool>("cmd_enable_m1109")) // M1109 config
        {
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Inset0, "speed_wall_0", "acceleration_wall_0", "jerk_wall_0");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Inset1, "speed_wall_1", "acceleration_wall_1", "jerk_wall_1");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::InsetX, "speed_wall_x", "acceleration_wall_x", "jerk_wall_x");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::BridgeInset0, "bridge_wall_speed", "acceleration_wall_0", "jerk_wall_0");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::BridgeInset1, "bridge_wall_speed", "acceleration_wall_1", "jerk_wall_1");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::BridgeInsetX, "bridge_wall_speed", "acceleration_wall_x", "jerk_wall_x");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::BridgeSkin1, "bridge_skin_speed", "acceleration_topbottom", "jerk_topbottom");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::BridgeSkin2, "bridge_skin_speed_2", "acceleration_topbottom", "jerk_topbottom");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::BridgeSkin3, "bridge_skin_speed_3", "acceleration_topbottom", "jerk_topbottom");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Roofing, "speed_roofing", "acceleration_roofing", "jerk_roofing");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Skin, "speed_topbottom", "acceleration_topbottom", "jerk_topbottom");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Flooring, "speed_flooring", "acceleration_flooring", "jerk_flooring");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Infill, "speed_infill", "acceleration_infill", "jerk_infill");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Ironing, "speed_ironing", "acceleration_ironing", "jerk_ironing");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::PerimeterGap, "speed_topbottom", "acceleration_topbottom", "jerk_topbottom");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::RaftBase, "raft_base_speed", "raft_base_acceleration", "raft_base_jerk");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::RaftInterface, "raft_interface_speed", "raft_interface_acceleration", "raft_interface_jerk");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::RaftSurface, "raft_surface_speed", "raft_surface_acceleration", "raft_surface_jerk");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::ExtruderTravel, "speed_travel", "acceleration_travel", "jerk_travel");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::ExtruderSkirtBrim, "skirt_brim_speed", "acceleration_skirt_brim", "jerk_skirt_brim");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::ExtruderPrimeTower, "speed_prime_tower", "acceleration_prime_tower", "jerk_prime_tower");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::SupportRoof, "speed_support_roof", "acceleration_support_roof", "jerk_support_roof");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::SupportInfill, "speed_support_infill", "acceleration_support_infill", "jerk_support_infill");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::SupportBottom, "speed_support_bottom", "acceleration_support_bottom", "jerk_support_bottom");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::SupportUnderRoof, "speed_support_under_roof", "acceleration_support_roof", "jerk_support_roof");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::SupportAboveBottom, "speed_support_above_bottom", "acceleration_support_bottom", "jerk_support_bottom");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::MoveRetraction, "speed_travel", "acceleration_travel", "jerk_travel");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Flooring, "speed_flooring", "acceleration_flooring", "jerk_flooring");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Inset0SeamCrossStart, "speed_wall_0", "acceleration_wall_0", "jerk_wall_0");
            prefix << getPathConfigFeatureConfig(scene, PathConfigFeature::Inset0SeamCrossFinish, "speed_wall_0", "acceleration_wall_0", "jerk_wall_0");
        }

		if (!Application::getInstance().communication->isSequential() && scene.settings.get<bool>("machine_use_task_stats"))
		{
			Application::getInstance().communication->sendPicasoPrintTaskStats(print_task_stats);
		}
	}
	break;

	default:
	{
		prefix << ";FLAVOR:" << flavorToString(flavor) << new_line;
		prefix << ";TIME:" << ((print_time) ? static_cast<int>(*print_time) : 6666) << new_line;
		if (flavor == EGCodeFlavor::ULTIGCODE)
		{
			prefix << ";MATERIAL:" << ((filament_used.size() >= 1) ? static_cast<int>(filament_used[0]) : 6666) << new_line;
			prefix << ";MATERIAL2:" << ((filament_used.size() >= 2) ? static_cast<int>(filament_used[1]) : 0) << new_line;

			prefix << ";NOZZLE_DIAMETER:" << Application::getInstance().current_slice->scene.extruders[0].settings.get<double>("machine_nozzle_size") << new_line;
		}
		else if (flavor == EGCodeFlavor::REPRAP || flavor == EGCodeFlavor::MARLIN || flavor == EGCodeFlavor::MARLIN_VOLUMATRIC)
		{
			prefix << ";Filament used: ";
			if (filament_used.size() > 0)
			{
				for (unsigned i = 0; i < filament_used.size(); ++i)
				{
					if (i > 0)
					{
						prefix << ", ";
					}
					if (flavor != EGCodeFlavor::MARLIN_VOLUMATRIC)
					{
						prefix << filament_used[i] / (1000 * extruder_attr[i].filament_area) << "m";
					}
					else //Use volumetric filament used.
					{
						prefix << filament_used[i] << "mm3";
					}
				}
			}
			else
			{
				prefix << "0m";
			}
			prefix << new_line;
			prefix << ";Layer height: " << Application::getInstance().current_slice->scene.current_mesh_group->settings.get<double>("layer_height") << new_line;
		}
		prefix << ";MINX:" << INT2MM(total_bounding_box.min.x) << new_line;
		prefix << ";MINY:" << INT2MM(total_bounding_box.min.y) << new_line;
		prefix << ";MINZ:" << INT2MM(total_bounding_box.min.z) << new_line;
		prefix << ";MAXX:" << INT2MM(total_bounding_box.max.x) << new_line;
		prefix << ";MAXY:" << INT2MM(total_bounding_box.max.y) << new_line;
		prefix << ";MAXZ:" << INT2MM(total_bounding_box.max.z) << new_line;
	}
	break;
	}

	return prefix.str();
}

std::string GCodeExport::getPathConfigFeatureConfig(
	const Scene& scene,
	const PathConfigFeature& feature,
	const std::string& key_speed,
	const std::string& key_acceleration,
	const std::string& key_jerk)
{
	std::ostringstream prefix;
	prefix << "M1109 S" << static_cast<int>(feature);
	prefix << " P" << static_cast<int>(PrintFeatureConverter::toPicasoSpeedProfile(feature));
	prefix << " V" << PrecisionedDouble{ 1, scene.settings.get<Velocity>(key_speed) };
	prefix << " A" << PrecisionedDouble{ 1, scene.settings.get<Acceleration>(key_acceleration) };
	prefix << " J" << PrecisionedDouble{ 1, scene.settings.get<Velocity>(key_jerk) };
	prefix << " I" << (PrintFeatureConverter::allowSpeedScale(feature) ? 1 : 0) << new_line;
	return prefix.str();
}

std::string GCodeExport::getFileDurationEstimateReport(
    const size_t extruder,
    const PathConfigFeature& feature,
    const Duration& duration)
{
    std::ostringstream prefix;
    prefix << ";M1115 T" << extruder;
    prefix << " S" << static_cast<int>(feature);
    prefix << " P" << PrecisionedDouble{ 0, duration } << new_line;
    return prefix.str();
}


void GCodeExport::setLayerNr(unsigned int layer_nr_) {
	layer_nr = layer_nr_;
}

void GCodeExport::incZeroBasedLayerNr()
{
#pragma omp critical
	zero_based_layer_nr++;
}

void GCodeExport::setLastOverhangState(const bool state) {
    last_overhang_state = state;
}

bool GCodeExport::getLastOverhangState() const {
    return last_overhang_state;
}

void GCodeExport::setLastModelOnSupportState(const bool state) {
    last_model_on_support_state = state;
}

bool GCodeExport::getLastModelOnSupportState() const {
    return last_model_on_support_state;
}

void GCodeExport::setOutputStream(std::ostream* stream)
{
    output_stream = stream;
    *output_stream << std::fixed;
}

bool GCodeExport::getExtruderIsUsed(const int extruder_nr) const
{
    assert(extruder_nr >= 0);
    assert(extruder_nr < MAX_EXTRUDERS);
    return extruder_attr[extruder_nr].is_used;
}

Point GCodeExport::getGcodePos(const coord_t x, const coord_t y, const int extruder_train) const
{
    if (use_extruder_offset_to_offset_coords)
    {
        const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[extruder_train].settings;
        return Point(x - extruder_settings.get<coord_t>("machine_nozzle_offset_x"), y - extruder_settings.get<coord_t>("machine_nozzle_offset_y"));
    }
    else
    {
        return Point(x, y);
    }
}


void GCodeExport::setFlavor(EGCodeFlavor flavor)
{
    this->flavor = flavor;
    if (flavor == EGCodeFlavor::MACH3)
    {
        for(int n=0; n<MAX_EXTRUDERS; n++)
        {
            extruder_attr[n].extruderCharacter = 'A' + n;
        }
    }
    else
    {
        for(int n=0; n<MAX_EXTRUDERS; n++)
        {
            extruder_attr[n].extruderCharacter = 'E';
        }
    }
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::MARLIN_VOLUMATRIC)
    {
        is_volumetric = true;
    }
    else
    {
        is_volumetric = false;
    }
}

EGCodeFlavor GCodeExport::getFlavor() const
{
    return flavor;
}

void GCodeExport::setZ(int z)
{
    current_layer_z = z;
}

void GCodeExport::setFirstMovementOnLayer()
{
    first_movement_on_layer = true;
}

void GCodeExport::setFlowRateExtrusionSettings(double max_extrusion_offset, double extrusion_offset_factor)
{
    this->max_extrusion_offset = max_extrusion_offset;
    this->extrusion_offset_factor = extrusion_offset_factor;
}

Point3 GCodeExport::getPosition() const
{
    return currentPosition;
}
Point GCodeExport::getPositionXY() const
{
    return Point(currentPosition.x, currentPosition.y);
}

int GCodeExport::getPositionZ() const
{
    return currentPosition.z;
}

int GCodeExport::getExtruderNr() const
{
    return current_extruder;
}

int GCodeExport::getExtrudersUsed() const
{
	const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();

	// From SliceDataStorage
	std::vector<bool> ret;
	ret.resize(extruder_count, false);
	const Settings& mesh_group_settings = Application::getInstance().current_slice->scene.current_mesh_group->settings;
	if (mesh_group_settings.get<EPlatformAdhesion>("adhesion_type") != EPlatformAdhesion::NONE)
	{
		ret[mesh_group_settings.get<ExtruderTrain&>("adhesion_extruder_nr").extruder_nr] = true;
	}
	// support is presupposed to be present...
	for (const MeshGroup& mesh_group : Application::getInstance().current_slice->scene.mesh_groups)
	{
		for (const Mesh& mesh : mesh_group.meshes)
		{
			if (mesh.settings.get<bool>("support_enable") || mesh.settings.get<bool>("support_mesh"))
			{
				ret[mesh.settings.get<ExtruderTrain&>("support_extruder_nr_layer_0").extruder_nr] = true;
				ret[mesh.settings.get<ExtruderTrain&>("support_infill_extruder_nr").extruder_nr] = true;
				if (mesh.settings.get<bool>("support_roof_enable"))
				{
					ret[mesh.settings.get<ExtruderTrain&>("support_roof_extruder_nr").extruder_nr] = true;
				}
				if (mesh.settings.get<bool>("support_under_roof_enable"))
				{
					ret[mesh.settings.get<ExtruderTrain&>("support_under_roof_extruder_nr").extruder_nr] = true;
				}
				if (mesh.settings.get<bool>("support_above_bottom_enable"))
				{
					ret[mesh.settings.get<ExtruderTrain&>("support_above_bottom_extruder_nr").extruder_nr] = true;
				}
				if (mesh.settings.get<bool>("support_bottom_enable"))
				{
					ret[mesh.settings.get<ExtruderTrain&>("support_bottom_extruder_nr").extruder_nr] = true;
				}
			}

			ret[mesh.settings.get<ExtruderTrain&>("wall_0_extruder_nr").extruder_nr] = true;
			ret[mesh.settings.get<ExtruderTrain&>("wall_1_extruder_nr").extruder_nr] = true;
			ret[mesh.settings.get<ExtruderTrain&>("wall_x_extruder_nr").extruder_nr] = true;
			ret[mesh.settings.get<ExtruderTrain&>("top_bottom_extruder_nr").extruder_nr] = true;
			ret[mesh.settings.get<ExtruderTrain&>("infill_extruder_nr").extruder_nr] = true;
		}
	}

	size_t used_extruders = 0;
	for (size_t extr_nr = 0; extr_nr < extruder_count; extr_nr++)
	{
		if (!ret[extr_nr])
		{
			continue;
		}
		used_extruders++;
	}

	return used_extruders;
}

void GCodeExport::setFilamentDiameter(const size_t extruder, const coord_t diameter)
{
    const double r = INT2MM(diameter) / 2.0;
    const double area = M_PI * r * r;
    extruder_attr[extruder].filament_area = area;
}

double GCodeExport::getCurrentExtrudedVolume() const
{
    double extrusion_amount = current_e_value;
    const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if (!extruder_settings.get<bool>("machine_firmware_retract"))
    { // no E values are changed to perform a retraction
        extrusion_amount -= extruder_attr[current_extruder].retraction_e_amount_at_e_start; // subtract the increment in E which was used for the first unretraction instead of extrusion
        extrusion_amount += extruder_attr[current_extruder].retraction_e_amount_current; // add the decrement in E which the filament is behind on extrusion due to the last retraction
    }
    if (is_volumetric)
    {
        return extrusion_amount;
    }
    else
    {
        return extrusion_amount * extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::eToMm(double e)
{
    if (is_volumetric)
    {
        return e / extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return e;
    }
}

double GCodeExport::mm3ToE(double mm3)
{
    if (is_volumetric)
    {
        return mm3;
    }
    else
    {
        return mm3 / extruder_attr[current_extruder].filament_area;
    }
}

double GCodeExport::mmToE(double mm)
{
    if (is_volumetric)
    {
        return mm * extruder_attr[current_extruder].filament_area;
    }
    else
    {
        return mm;
    }
}

double GCodeExport::eToMm3(double e, size_t extruder)
{
    if (is_volumetric)
    {
        return e;
    }
    else
    {
        return e * extruder_attr[extruder].filament_area;
    }
}

double GCodeExport::getTotalFilamentUsed(size_t extruder_nr)
{
    if (extruder_nr == current_extruder)
        return extruder_attr[extruder_nr].totalFilament + getCurrentExtrudedVolume();
    return extruder_attr[extruder_nr].totalFilament;
}

std::vector<Duration> GCodeExport::getTotalPrintTimePerFeature()
{
    return total_print_times;
}


PicasoPrintModeEstimate GCodeExport::getPicasoPrintModeEstimate(const Scene& scene, const TimeEstimateResult& estimateResult)
{
	PicasoPrintModeEstimate result(estimateResult.extruder_count, estimateResult.extra_time, estimateResult.retract_count, estimateResult.zhopp_count);

	// Prepare SpeedScales
	std::map<size_t, PicasoSpeedScale> speed_scales;
	for (size_t i = 0; i < static_cast<size_t>(PathConfigFeature::NumPathConfigFeatures); i++)
	{
		const PathConfigFeature feature = static_cast<PathConfigFeature>(i);
		const Velocity base_speed = getFeatureSpeed(scene, feature);
		const bool allow_scale = PrintFeatureConverter::allowSpeedScale(feature);

		PicasoSpeedScale speed_scale(base_speed, allow_scale);
		speed_scales[i] = speed_scale;
	}

	// Calculate PicasoPrintModeEstimate
	for (auto it = estimateResult.durations.begin(); it != estimateResult.durations.end(); ++it)
	{
		auto result_it = result.durations.find(it->first);
		if (result_it != result.durations.end())
		{
			for (size_t n = 0; n < it->second.size(); n++)
			{
				const PathConfigFeature feature = static_cast<PathConfigFeature>(n);
				const PicasoSpeedProfile profile = PrintFeatureConverter::toPicasoSpeedProfile(feature);

				const PicasoSpeedScale& speed_scale = speed_scales[n];

				switch (profile)
				{
				case PicasoSpeedProfile::Perimeter:
				case PicasoSpeedProfile::Loops:
				case PicasoSpeedProfile::Support:
				case PicasoSpeedProfile::InterfaceSupport:
				case PicasoSpeedProfile::Infill:
				case PicasoSpeedProfile::Travel:
				{
					Duration value(it->second[n]);

					for (size_t m = 0; m < static_cast<size_t>(PicasoPrintMode::NumPicasoPrintModes); m++)
					{
						if (speed_scale.allow_scale)
						{
							Duration speed_koef = picasoPrintModeSpeed[m][static_cast<size_t>(profile)] / speed_scale.base_speed;
							result_it->second[m] += value / speed_koef;
						}
						else
						{
							result_it->second[m] += value;
						}
					}
				}
				break;

				case PicasoSpeedProfile::Undefined:
				default:
					break;
				}
			}
		}
	}

	return result;
}

double GCodeExport::getSumTotalPrintTimes()
{
    double sum = 0.0;
    for (size_t i = 0; i < total_print_times.size(); i++)
    {
        sum += total_print_times[i];
    }
    return sum;
}

void GCodeExport::resetTotalPrintTimeAndFilament()
{
    for(size_t i = 0; i < total_print_times.size(); i++)
    {
        total_print_times[i] = 0.0;
    }

	total_print_times_pathConfig.Reset();

    for(unsigned int e=0; e<MAX_EXTRUDERS; e++)
    {
        extruder_attr[e].totalFilament = 0.0;
        extruder_attr[e].currentTemperature = 0;
        extruder_attr[e].waited_for_temperature = false;
    }
    current_e_value = 0.0;
    estimateCalculator.reset();
}

void GCodeExport::updateTotalPrintTime()
{
	TimeEstimateResult timeEstimates = estimateCalculator.getTimeEstimateResult();

	total_print_times_pathConfig.Add(timeEstimates);
	std::vector<Duration> estimates = timeEstimates.getPrintFeatureTypes();

	for (size_t i = 0; i < estimates.size(); i++)
	{
		total_print_times[i] += estimates[i];
	}
	estimateCalculator.reset();

	if (getFlavor() == EGCodeFlavor::PICASO)
	{
		const Scene& scene = Application::getInstance().current_slice->scene;

		PicasoPrintModeEstimate layerEstimates = getPicasoPrintModeEstimate(scene, timeEstimates);
		// <Layer>
        if (scene.settings.get<bool>("cmd_enable_m1111")) // Report Layer Durations
        {
            for (auto it = layerEstimates.durations.begin(); it != layerEstimates.durations.end(); ++it)
            {
                for (size_t n = 0; n < it->second.size(); n++)
                {
                    *output_stream << "M1111 T" << it->first
                        << " L" << this->zero_based_layer_nr
                        << " S" << n
                        << " P" << PrecisionedDouble{ 0, it->second[n] } << new_line;
                }
            }
        }
        if (scene.settings.get<bool>("cmd_enable_m1113")) // Report Layer Additional Counters
        {
            *output_stream << "M1113 L" << this->zero_based_layer_nr
                << " Z" << layerEstimates.zhopp_count
                << " E" << layerEstimates.retract_count
                << " P" << PrecisionedDouble{ 0, layerEstimates.extra_time } << new_line;
        }
        if (scene.settings.get<bool>("cmd_enable_m1116")) // Report Total Durations
        {
            for (auto it = total_print_times_pathConfig.durations.begin(); it != total_print_times_pathConfig.durations.end(); ++it) // Nozzle
            {
                for (size_t n = 0; n < it->second.size(); n++) // PathConfigFeature
                {
                    const PathConfigFeature feature = static_cast<PathConfigFeature>(n);
                    const Duration duration(it->second[n]);

                    *output_stream << ";M1116 L" << this->zero_based_layer_nr
                        << " T" << it->first
                        << " S" << static_cast<int>(feature)
                        << " P" << PrecisionedDouble{ 0, duration } << new_line;
                }
            }
        }
		// </Layer>


		PicasoPrintModeEstimate totalEstimates = getPicasoPrintModeEstimate(scene, total_print_times_pathConfig);
		// <Total>
		// Report Total Durations
        if (scene.settings.get<bool>("cmd_enable_m1112"))
        {
            for (auto it = totalEstimates.durations.begin(); it != totalEstimates.durations.end(); ++it)
            {
                for (size_t n = 0; n < it->second.size(); n++)
                {
                    *output_stream << "M1112 T" << it->first
                        << " S" << n
                        << " P" << PrecisionedDouble{ 0, it->second[n] } << new_line;
                }
            }
        }
        if (scene.settings.get<bool>("cmd_enable_m1114"))
        {
            // Report Layer Additional Counters
            *output_stream << "M1114"
                << " Z" << totalEstimates.zhopp_count
                << " E" << totalEstimates.retract_count
                << " P" << PrecisionedDouble{ 0, totalEstimates.extra_time } << new_line;
        }
		// </Total>
	}

	writeTimeComment(getSumTotalPrintTimes());
}

void GCodeExport::writeComment(const std::string& unsanitized_comment)
{
    const std::string comment = transliterate(unsanitized_comment);

    *output_stream << ";";
    for (unsigned int i = 0; i < comment.length(); i++)
    {
        if (comment[i] == '\n')
        {
            *output_stream << new_line << ";";
        }
        else
        {
            *output_stream << comment[i];
        }
    }
    *output_stream << new_line;
}

void GCodeExport::writeTimeComment(std::string feature, const double time)
{
    //output_stream.Write($";TIME_{feature}:{time}{new_line}");
    *output_stream << ";TIME_" << feature << ":" << time << new_line;
}

void GCodeExport::writeTimeComment(const Duration time)
{
    *output_stream << ";TIME_ELAPSED:" << time << new_line;
}

void GCodeExport::writeTypeComment(const PrintFeatureType& type)
{
    switch (type)
    {
        case PrintFeatureType::OuterWall:
            *output_stream << ";TYPE:WALL-OUTER" << new_line;
            break;
        case PrintFeatureType::InnerWall:
            *output_stream << ";TYPE:WALL-INNER" << new_line;
            break;
        case PrintFeatureType::Skin:
            *output_stream << ";TYPE:SKIN" << new_line;
            break;
        case PrintFeatureType::Support:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::SkirtBrim:
            *output_stream << ";TYPE:SKIRT" << new_line;
            break;
        case PrintFeatureType::Infill:
            *output_stream << ";TYPE:FILL" << new_line;
            break;
        case PrintFeatureType::SupportInfill:
            *output_stream << ";TYPE:SUPPORT" << new_line;
            break;
        case PrintFeatureType::SupportInterface:
            *output_stream << ";TYPE:SUPPORT-INTERFACE" << new_line;
			break;
        case PrintFeatureType::PrimeTower:
            *output_stream << ";TYPE:PRIME-TOWER" << new_line;
            break;
        case PrintFeatureType::MoveCombing:
        case PrintFeatureType::MoveRetraction:
        case PrintFeatureType::NoneType:
        case PrintFeatureType::NumPrintFeatureTypes:
            // do nothing
            break;
    }
}

Velocity GCodeExport::getFeatureSpeed(const Scene& scene, const PathConfigFeature& feature)
{
	switch (feature)
	{
	case PathConfigFeature::Inset0: return scene.settings.get<Velocity>("speed_wall_0");
	case PathConfigFeature::Inset1: return scene.settings.get<Velocity>("speed_wall_1");
	case PathConfigFeature::InsetX: return scene.settings.get<Velocity>("speed_wall_x");
	case PathConfigFeature::BridgeInset0: return scene.settings.get<Velocity>("bridge_wall_speed");
	case PathConfigFeature::BridgeInset1: return scene.settings.get<Velocity>("bridge_wall_speed");
	case PathConfigFeature::BridgeInsetX: return scene.settings.get<Velocity>("bridge_wall_speed");
	case PathConfigFeature::BridgeSkin1: return scene.settings.get<Velocity>("bridge_skin_speed");
	case PathConfigFeature::BridgeSkin2: return scene.settings.get<Velocity>("bridge_skin_speed_2");
	case PathConfigFeature::BridgeSkin3: return scene.settings.get<Velocity>("bridge_skin_speed_3");
	case PathConfigFeature::Skin: return scene.settings.get<Velocity>("speed_topbottom");
	case PathConfigFeature::Roofing: return scene.settings.get<Velocity>("speed_roofing");
	case PathConfigFeature::Flooring: return scene.settings.get<Velocity>("speed_flooring");
	case PathConfigFeature::Infill: return scene.settings.get<Velocity>("speed_infill");
	case PathConfigFeature::Ironing: return scene.settings.get<Velocity>("speed_ironing");
	case PathConfigFeature::PerimeterGap: return scene.settings.get<Velocity>("speed_topbottom");
	case PathConfigFeature::RaftBase: return scene.settings.get<Velocity>("raft_base_speed");
	case PathConfigFeature::RaftInterface: return scene.settings.get<Velocity>("raft_interface_speed");
	case PathConfigFeature::RaftSurface: return scene.settings.get<Velocity>("raft_surface_speed");
	case PathConfigFeature::ExtruderTravel: return scene.settings.get<Velocity>("speed_travel");
	case PathConfigFeature::ExtruderSkirtBrim: return scene.settings.get<Velocity>("skirt_brim_speed");
	case PathConfigFeature::ExtruderPrimeTower: return scene.settings.get<Velocity>("speed_prime_tower");
	case PathConfigFeature::SupportRoof: return scene.settings.get<Velocity>("speed_support_roof");
	case PathConfigFeature::SupportInfill: return scene.settings.get<Velocity>("speed_support_infill");
	case PathConfigFeature::SupportBottom: return scene.settings.get<Velocity>("speed_support_bottom");
	case PathConfigFeature::SupportUnderRoof: return scene.settings.get<Velocity>("speed_support_under_roof");
	case PathConfigFeature::SupportAboveBottom: return scene.settings.get<Velocity>("speed_support_above_bottom");
	default: return Velocity(1.0);
	}
}

std::string GCodeExport::picasoSpeedProfileToKissComment(const PicasoSpeedProfile& type)
{
    switch (type)
    {
    case PicasoSpeedProfile::Perimeter: return "; 'Perimeter Path'";
    case PicasoSpeedProfile::Loops: return "; 'Loop Path'";
    case PicasoSpeedProfile::Support: return "; 'Support'";
    case PicasoSpeedProfile::InterfaceSupport: return "; 'Support Interface Path'";
    case PicasoSpeedProfile::Infill: return "; 'Solid Path'";
	case PicasoSpeedProfile::Travel: return "; 'Travel'";
    case PicasoSpeedProfile::Undefined:
    default: return "; Undefined";
    }
}

std::string GCodeExport::picasoSpeedProfileToString(const PicasoSpeedProfile& type)
{
    switch (type)
    {
    case PicasoSpeedProfile::Perimeter: return "Perimeter";
    case PicasoSpeedProfile::Loops: return "Loops";
    case PicasoSpeedProfile::Support: return "Support";
    case PicasoSpeedProfile::InterfaceSupport: return "InterfaceSupport";
    case PicasoSpeedProfile::Infill: return "Infill";
    case PicasoSpeedProfile::Travel: return "Travel";
    case PicasoSpeedProfile::Undefined:
    default: return "Undefined";
    }
}

std::string GCodeExport::printFeatureTypeToString(const PrintFeatureType& type)
{
    switch (type)
    {
    case PrintFeatureType::NoneType: return "NoneType";
    case PrintFeatureType::OuterWall: return "OuterWall";
    case PrintFeatureType::InnerWall: return "InnerWall";
    case PrintFeatureType::Skin: return "Skin";
    case PrintFeatureType::Support: return "Support";
    case PrintFeatureType::SkirtBrim: return "SkirtBrim";
    case PrintFeatureType::Infill: return "Infill";
    case PrintFeatureType::SupportInfill: return "SupportInfill";
    case PrintFeatureType::MoveCombing: return "MoveCombing";
    case PrintFeatureType::MoveRetraction: return "MoveRetraction";
    case PrintFeatureType::SupportInterface: return "SupportInterface";
    default: return "Undefined";
    }
}

void GCodeExport::writeLayerComment(const LayerIndex layer_nr)
{
	if (this->flavor == EGCodeFlavor::PICASO)
	{
		*output_stream << ";LAYER:" << this->zero_based_layer_nr << new_line;
		*output_stream << "M532 L" << this->zero_based_layer_nr << new_line;
	}
	else
	{
		*output_stream << ";LAYER:" << layer_nr << new_line;
	}
}

void GCodeExport::writeLayerCountComment(const size_t layer_count)
{
	if (this->flavor == EGCodeFlavor::PICASO)
	{
		size_t real_layer_count = layer_count + Raft::getTotalExtraLayers();

		*output_stream << ";LAYER_COUNT:" << real_layer_count << new_line;
		*output_stream << "M530 S1 L" << real_layer_count << new_line;
	}
	else
	{
		*output_stream << ";LAYER_COUNT:" << layer_count << new_line;
	}
}

void GCodeExport::writeLine(const char* line)
{
    *output_stream << line << new_line;
}

void GCodeExport::writeExtrusionMode(bool set_relative_extrusion_mode)
{
    if (set_relative_extrusion_mode)
    {
        *output_stream << "M83 ;relative extrusion mode" << new_line;
    }
    else
    {
        *output_stream << "M82 ;absolute extrusion mode" << new_line;
    }
}

void GCodeExport::resetExtrusionValue()
{
    if (!relative_extrusion)
    {
        *output_stream << "G92 " << extruder_attr[current_extruder].extruderCharacter << "0" << new_line;
    }
    double current_extruded_volume = getCurrentExtrudedVolume();
    extruder_attr[current_extruder].totalFilament += current_extruded_volume;
    for (double& extruded_volume_at_retraction : extruder_attr[current_extruder].extruded_volume_at_previous_n_retractions)
    { // update the extruded_volume_at_previous_n_retractions only of the current extruder, since other extruders don't extrude the current volume
        extruded_volume_at_retraction -= current_extruded_volume;
    }
    current_e_value = 0.0;
    extruder_attr[current_extruder].retraction_e_amount_at_e_start = extruder_attr[current_extruder].retraction_e_amount_current;
}

void GCodeExport::writeDelay(const Duration& time_amount)
{
    *output_stream << "G4 P" << int(time_amount * 1000) << new_line;
    estimateCalculator.addTime(time_amount);
}

void GCodeExport::writeTravel(const Point& p, const Velocity& speed)
{
    writeTravel(Point3(p.X, p.Y, current_layer_z), speed);
}
void GCodeExport::writeExtrusion(const Point& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, PathConfigFeature featureType, bool update_extrusion_offset)
{
    writeExtrusion(Point3(p.X, p.Y, current_layer_z), speed, extrusion_mm3_per_mm, feature, featureType, update_extrusion_offset);
}

void GCodeExport::writeTravel(const Point3& p, const Velocity& speed)
{
    switch (flavor)
    {
    case EGCodeFlavor::BFB:
        writeMoveBFB(p.x, p.y, p.z + is_z_hopped, speed, 0.0, PrintFeatureType::MoveCombing, PathConfigFeature::ExtruderTravel);
        break;

#ifdef PICASO_HW_ZHOPP
    case EGCodeFlavor::PICASO:
        writeTravel(p.x, p.y, p.z + (firmware_zhopp ? 0 : is_z_hopped), speed);
        break;
#endif

    default:
        writeTravel(p.x, p.y, p.z + is_z_hopped, speed);
        break;
    }
}

void GCodeExport::writeExtrusion(const Point3& p, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, PathConfigFeature featureType, bool update_extrusion_offset)
{
    if (flavor == EGCodeFlavor::BFB)
    {
        writeMoveBFB(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature, featureType);
        return;
    }
    writeExtrusion(p.x, p.y, p.z, speed, extrusion_mm3_per_mm, feature, featureType, update_extrusion_offset);
}

void GCodeExport::writeMoveBFB(const int x, const int y, const int z, const Velocity& speed, double extrusion_mm3_per_mm, PrintFeatureType feature, PathConfigFeature featureType)
{
    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }
    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    Point gcode_pos = getGcodePos(x,y, current_extruder);

    //For Bits From Bytes machines, we need to handle this completely differently. As they do not use E values but RPM values.
    float fspeed = speed * 60;
    float rpm = extrusion_per_mm * speed * 60;
    const float mm_per_rpm = 4.0; //All BFB machines have 4mm per RPM extrusion.
    rpm /= mm_per_rpm;
    if (rpm > 0)
    {
        if (extruder_attr[current_extruder].retraction_e_amount_current)
        {
            if (currentSpeed != double(rpm))
            {
                //fprintf(f, "; %f e-per-mm %d mm-width %d mm/s\n", extrusion_per_mm, lineWidth, speed);
                //fprintf(f, "M108 S%0.1f\r\n", rpm);
                *output_stream << "M108 S" << PrecisionedDouble{1, rpm} << new_line;
                currentSpeed = double(rpm);
            }
            //Add M101 or M201 to enable the proper extruder.
            *output_stream << "M" << int((current_extruder + 1) * 100 + 1) << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
        }
        //Fix the speed by the actual RPM we are asking, because of rounding errors we cannot get all RPM values, but we have a lot more resolution in the feedrate value.
        // (Trick copied from KISSlicer, thanks Jonathan)
        fspeed *= (rpm / (roundf(rpm * 100) / 100));

        //Increase the extrusion amount to calculate the amount of filament used.
        Point3 diff = Point3(x,y,z) - getPosition();
        
        current_e_value += extrusion_per_mm * diff.vSizeMM();
    }
    else
    {
        //If we are not extruding, check if we still need to disable the extruder. This causes a retraction due to auto-retraction.
        if (!extruder_attr[current_extruder].retraction_e_amount_current)
        {
            *output_stream << "M103" << new_line;
            extruder_attr[current_extruder].retraction_e_amount_current = 1.0; // 1.0 used as stub; BFB doesn't use the actual retraction amount; it performs retraction on the firmware automatically
        }
    }
    *output_stream << "G1 X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y} << " Z" << MMtoStream{z};
    *output_stream << " F" << PrecisionedDouble{1, fspeed} << new_line;
    
    currentPosition = Point3(x, y, z);
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), speed, feature, featureType, current_extruder);
}

void GCodeExport::writeTravel(const coord_t& x, const coord_t& y, const coord_t& z, const Velocity& speed)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
    {
        return;
    }

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
#endif //ASSERT_INSANE_OUTPUT

    const PrintFeatureType travel_move_type = extruder_attr[current_extruder].retraction_e_amount_current ? PrintFeatureType::MoveRetraction : PrintFeatureType::MoveCombing;
	const PathConfigFeature travel_feature_type = PathConfigFeature::ExtruderTravel;
    const int display_width = extruder_attr[current_extruder].retraction_e_amount_current ? MM2INT(0.2) : MM2INT(0.1);
    const double layer_height = Application::getInstance().current_slice->scene.current_mesh_group->settings.get<double>("layer_height");
    Application::getInstance().communication->sendLineTo(travel_move_type, Point(x, y), display_width, layer_height, speed);

    if (first_movement_on_layer)
    {
        if (flavor == EGCodeFlavor::PICASO)
        {
            if (is_z_hopped > 0)
            {
                *output_stream << "G0";
                writeFXYZE(speed, x, y, currentPosition.z, current_e_value, travel_move_type, travel_feature_type);
                writeZhopEnd();
                first_movement_on_layer = false;
                return;
            }
        }
        first_movement_on_layer = false;
    }

    *output_stream << "G0";
    writeFXYZE(speed, x, y, z, current_e_value, travel_move_type, travel_feature_type);
}

void GCodeExport::writeExtrusion(const int x, const int y, const int z, const Velocity& speed, const double extrusion_mm3_per_mm, const PrintFeatureType& feature, const PathConfigFeature& featureType, const bool update_extrusion_offset)
{
    if (currentPosition.x == x && currentPosition.y == y && currentPosition.z == z)
    {
        return;
    }

#ifdef ASSERT_INSANE_OUTPUT
    assert(speed < 400 && speed > 1); // normal F values occurring in UM2 gcode (this code should not be compiled for release)
    assert(currentPosition != no_point3);
    assert(Point3(x, y, z) != no_point3);
    assert((Point3(x,y,z) - currentPosition).vSize() < MM2INT(1000)); // no crazy positions (this code should not be compiled for release)
    assert(extrusion_mm3_per_mm >= 0.0);
#endif //ASSERT_INSANE_OUTPUT
#ifdef DEBUG
    if (std::isinf(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is infinite!");
        assert(false && "Infinite extrusion move!");
        std::exit(1);
    }

    if (std::isnan(extrusion_mm3_per_mm))
    {
        logError("Extrusion rate is not a number!");
        assert(false && "NaN extrusion move!");
        std::exit(1);
    }

    if (extrusion_mm3_per_mm < 0.0)
    {
        logWarning("Warning! Negative extrusion move!\n");
    }
#endif

    const double extrusion_per_mm = mm3ToE(extrusion_mm3_per_mm);

    first_movement_on_layer = false;

    if (is_z_hopped > 0)
    {
        writeZhopEnd();
    }

    const Point3 diff = Point3(x,y,z) - currentPosition;
    const double diff_length = diff.vSizeMM();

    writeUnretractionAndPrime();

    //flow rate compensation
    double extrusion_offset = 0;
    if(diff_length)
    {
        extrusion_offset = speed * extrusion_mm3_per_mm * extrusion_offset_factor;
        if (extrusion_offset > max_extrusion_offset)
        {
            extrusion_offset = max_extrusion_offset;
        }
    }
    // write new value of extrusion_offset, which will be remembered.
    if (update_extrusion_offset && (extrusion_offset != current_e_offset))
    {
        current_e_offset = extrusion_offset;
        *output_stream << ";FLOW_RATE_COMPENSATED_OFFSET = " << current_e_offset << new_line;
    }

    extruder_attr[current_extruder].last_e_value_after_wipe += extrusion_per_mm * diff_length;
    const double new_e_value = current_e_value + extrusion_per_mm * diff_length;

	//*output_stream << ";!!!!! DBG " << new_line;
	//*output_stream << ";current_e_value = " << current_e_value << new_line;
	//*output_stream << ";new_e_value = " << new_e_value << new_line;
	//*output_stream << ";last_e_value_after_wipe = " << extruder_attr[current_extruder].last_e_value_after_wipe << new_line;

    *output_stream << "G1";
    writeFXYZE(speed, x, y, z, new_e_value, feature, featureType);
}

void GCodeExport::writeFXYZE(const Velocity& speed, const int x, const int y, const int z, const double e, const PrintFeatureType& feature, const PathConfigFeature& featureType)
{
    if (currentSpeed != speed)
    {
        *output_stream << " F" << PrecisionedDouble{1, speed * 60};
        currentSpeed = speed;
    }

    Point gcode_pos = getGcodePos(x, y, current_extruder);
    total_bounding_box.include(Point3(gcode_pos.X, gcode_pos.Y, z));

    *output_stream << " X" << MMtoStream{gcode_pos.X} << " Y" << MMtoStream{gcode_pos.Y};
    if (z != currentPosition.z)
    {
        *output_stream << " Z" << MMtoStream{z};
    }
    if (e + current_e_offset != current_e_value)
    {
        const double output_e = (relative_extrusion)? e + current_e_offset - current_e_value : e + current_e_offset;
        *output_stream << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, output_e};
    }

#ifdef ADD_GCODE_COMMENTS
    *output_stream << " ;T" << current_extruder << "; TYPE:" << printFeatureTypeToString(feature);
#endif // ADD_GCODE_COMMENTS

    *output_stream << new_line;
    
    currentPosition = Point3(x, y, z);
    current_e_value = e;
    estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(x), INT2MM(y), INT2MM(z), eToMm(e)), speed, feature, featureType, current_extruder);
}

void GCodeExport::writeUnretractionAndPrime()
{
    const double prime_volume = extruder_attr[current_extruder].prime_volume;
    const double prime_volume_e = mm3ToE(prime_volume);
    current_e_value += prime_volume_e;
    if (extruder_attr[current_extruder].retraction_e_amount_current)
    {
        const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
        if (extruder_settings.get<bool>("machine_firmware_retract"))
        { // note that BFB is handled differently
#ifdef PICASO_HW_RETRACT
            if (flavor == EGCodeFlavor::PICASO)
            {
                current_e_value += extruder_attr[current_extruder].retraction_e_amount_current;
                const double output_e = (relative_extrusion) ? extruder_attr[current_extruder].retraction_e_amount_current + prime_volume_e : current_e_value;
                // E-axis fillament down
                *output_stream << "G11 E" << PrecisionedDouble{ 5, output_e };

				//*output_stream << " ;prime_volume: " << prime_volume;
				//*output_stream << " ;current_e_value: " << current_e_value;
				//*output_stream << " ;retraction_e_amount_current: " << extruder_attr[current_extruder].retraction_e_amount_current;

                if (prime_volume > 0)
                {
                    *output_stream << ";prime_volume: " << prime_volume;
                }
                currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
                estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction, PathConfigFeature::MoveRetraction, current_extruder);
            }
            else
#endif
            {
                *output_stream << "G11";
                //Assume default UM2 retraction settings.
                if (prime_volume != 0)
                {
                    *output_stream << new_line;
                    const double output_e = (relative_extrusion) ? prime_volume_e : current_e_value;
                    *output_stream << "G1 F" << PrecisionedDouble{ 1, extruder_attr[current_extruder].last_retraction_prime_speed * 60 } << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{ 5, output_e };
                    currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
                }
                estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), 25.0, PrintFeatureType::MoveRetraction, PathConfigFeature::MoveRetraction, current_extruder);
            }
        }
        else
        {
            current_e_value += extruder_attr[current_extruder].retraction_e_amount_current;
            const double output_e = (relative_extrusion)? extruder_attr[current_extruder].retraction_e_amount_current + prime_volume_e : current_e_value;
            *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter << PrecisionedDouble{5, output_e};
            currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction, PathConfigFeature::MoveRetraction, current_extruder);
        }

#ifdef ADD_GCODE_COMMENTS
        *output_stream << " ;RETRACT:END";
#endif // ADD_GCODE_COMMENTS

        estimateCalculator.addRetract(false);
        *output_stream << new_line;

        if (getCurrentExtrudedVolume() > 10000.0 && flavor != EGCodeFlavor::BFB && flavor != EGCodeFlavor::MAKERBOT) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
        {
            resetExtrusionValue();
        }
        extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
    }
    else if (prime_volume != 0.0)
    {
        const double output_e = (relative_extrusion)? prime_volume_e : current_e_value;
        *output_stream << "G1 F" << PrecisionedDouble{1, extruder_attr[current_extruder].last_retraction_prime_speed * 60} << " " << extruder_attr[current_extruder].extruderCharacter;
        *output_stream << PrecisionedDouble{5, output_e} << new_line;
        currentSpeed = extruder_attr[current_extruder].last_retraction_prime_speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::NoneType, PathConfigFeature::NoneType, current_extruder);
    }
    extruder_attr[current_extruder].prime_volume = 0.0;
    
    if (getCurrentExtrudedVolume() > 10000.0 && flavor != EGCodeFlavor::BFB && flavor != EGCodeFlavor::MAKERBOT) //According to https://github.com/Ultimaker/CuraEngine/issues/14 having more then 21m of extrusion causes inaccuracies. So reset it every 10m, just to be sure.
    {
        resetExtrusionValue();
    }
    if (extruder_attr[current_extruder].retraction_e_amount_current)
    {
        extruder_attr[current_extruder].retraction_e_amount_current = 0.0;
    }
}

void GCodeExport::writeRetraction(const RetractionConfig& config, bool force, bool extruder_switch)
{
    ExtruderTrainAttributes& extr_attr = extruder_attr[current_extruder];

    if(flavor == EGCodeFlavor::BFB) //BitsFromBytes does automatic retraction.
    {
        if(extruder_switch)
        {
            if(!extr_attr.retraction_e_amount_current)
            {
                *output_stream << "M103" << new_line;
            }
            extr_attr.retraction_e_amount_current = 1.0; // 1.0 is a stub; BFB doesn't use the actual retracted amount; retraction is performed by firmware
        }
        return;
    }

    double old_retraction_e_amount = extr_attr.retraction_e_amount_current;
    double new_retraction_e_amount = mmToE(config.distance);
    double retraction_diff_e_amount = old_retraction_e_amount - new_retraction_e_amount;
    if(std::abs(retraction_diff_e_amount) < 0.000001)
    {
        return;
    }

    { // handle retraction limitation
        double current_extruded_volume = getCurrentExtrudedVolume();
        std::deque<double>& extruded_volume_at_previous_n_retractions = extr_attr.extruded_volume_at_previous_n_retractions;
        while (extruded_volume_at_previous_n_retractions.size() > config.retraction_count_max && !extruded_volume_at_previous_n_retractions.empty()) 
        {
            // extruder switch could have introduced data which falls outside the retraction window
            // also the retraction_count_max could have changed between the last retraction and this
            extruded_volume_at_previous_n_retractions.pop_back();
        }
        if(!force && config.retraction_count_max <= 0)
        {
            return;
        }
        if(!force && extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max
            && current_extruded_volume < extruded_volume_at_previous_n_retractions.back() + config.retraction_extrusion_window * extr_attr.filament_area) 
        {
            return;
        }
        extruded_volume_at_previous_n_retractions.push_front(current_extruded_volume);
        if(extruded_volume_at_previous_n_retractions.size() == config.retraction_count_max + 1) 
        {
            extruded_volume_at_previous_n_retractions.pop_back();
        }
    }

    const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if(extruder_settings.get<bool>("machine_firmware_retract"))
    {
#ifdef PICASO_HW_RETRACT
        if (flavor == EGCodeFlavor::PICASO)
        {
            double speed = ((retraction_diff_e_amount < 0.0) ? config.speed : extr_attr.last_retraction_prime_speed) * 60;
            current_e_value += retraction_diff_e_amount;
            const double output_e = (relative_extrusion) ? retraction_diff_e_amount : current_e_value;
            currentSpeed = speed;
            // E-axis fillament up
            *output_stream << "G10 E" << PrecisionedDouble{ 5, output_e };
#ifdef ADD_GCODE_COMMENTS
            if (extruder_switch)
            {
                // lift because liquid form can leak
                *output_stream << " ;extruder_switch";
            }
#endif
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction, PathConfigFeature::MoveRetraction, current_extruder);
            extr_attr.last_retraction_prime_speed = config.primeSpeed;
        }
        else
#endif
        {
            if (extruder_switch && extr_attr.retraction_e_amount_current)
            {
                return;
            }
            *output_stream << "G10";
            if (extruder_switch && flavor == EGCodeFlavor::REPETIER)
            {
                *output_stream << " S1";
            }
            //Assume default UM2 retraction settings.
            estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value + retraction_diff_e_amount)), 25, PrintFeatureType::MoveRetraction, PathConfigFeature::MoveRetraction, current_extruder); // TODO: hardcoded values!
        }
    }
    else
    {
        double speed = ((retraction_diff_e_amount < 0.0)? config.speed : extr_attr.last_retraction_prime_speed);
        current_e_value += retraction_diff_e_amount;
        const double output_e = (relative_extrusion)? retraction_diff_e_amount : current_e_value;
        *output_stream << "G1 F" << PrecisionedDouble{1, speed * 60} << " " << extr_attr.extruderCharacter << PrecisionedDouble{5, output_e};
        currentSpeed = speed;
        estimateCalculator.plan(TimeEstimateCalculator::Position(INT2MM(currentPosition.x), INT2MM(currentPosition.y), INT2MM(currentPosition.z), eToMm(current_e_value)), currentSpeed, PrintFeatureType::MoveRetraction, PathConfigFeature::MoveRetraction, current_extruder);
        extr_attr.last_retraction_prime_speed = config.primeSpeed;
    }

#ifdef ADD_GCODE_COMMENTS
    *output_stream << " ;RETRACT:START";
#endif // ADD_GCODE_COMMENTS

    estimateCalculator.addRetract(true);
    *output_stream << new_line;

	//*output_stream << ";!!!!! DBG !!!!!" << new_line;
	//*output_stream << ";current_e_value = " << current_e_value << new_line;
	//*output_stream << ";old_retraction_e_amount = " << old_retraction_e_amount << new_line;
	//*output_stream << ";new_retraction_e_amount = " << new_retraction_e_amount << new_line;
	//*output_stream << ";retraction_e_amount_current = " << extr_attr.retraction_e_amount_current << new_line;
	//*output_stream << ";config.prime_volume = " << config.prime_volume << new_line;
	//*output_stream << ";config.distance = " << config.distance << new_line;

    extr_attr.retraction_e_amount_current = new_retraction_e_amount; // suppose that for UM2 the retraction amount in the firmware is equal to the provided amount
    extr_attr.prime_volume += config.prime_volume;
}

void GCodeExport::writeZhopStart(const coord_t hop_height, Velocity speed/*= 0*/)
{
    if (hop_height > 0 && (is_z_hopped != hop_height))
    {
        if (speed == 0)
        {
            const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[current_extruder];
            speed = extruder.settings.get<Velocity>("speed_z_hop");
        }
        is_z_hopped = hop_height;
        currentSpeed = speed;

#ifdef PICASO_HW_ZHOPP
        if (firmware_zhopp && flavor == EGCodeFlavor::PICASO)
        {
            // Z-axis move down
            *output_stream << "G10 F" << PrecisionedDouble{ 1, speed * 60 } << " Z" << MMtoStream{ current_layer_z + is_z_hopped };
        }
        else
#endif // PICASO_HW_ZHOPP
        {
            *output_stream << "G1 F" << PrecisionedDouble{ 1, speed * 60 } << " Z" << MMtoStream{ current_layer_z + is_z_hopped };
        }

#ifdef ADD_GCODE_COMMENTS
        *output_stream << " ;ZHOPP:START";
#endif // ADD_GCODE_COMMENTS

        estimateCalculator.addZHopp(true);
        *output_stream << new_line;

        total_bounding_box.includeZ(current_layer_z + is_z_hopped);
        assert(speed > 0.0 && "Z hop speed should be positive.");
    }
}

void GCodeExport::writeZhopEnd(Velocity speed/*= 0*/)
{
    if (is_z_hopped)
    {
        if (speed == 0)
        {
            const ExtruderTrain& extruder = Application::getInstance().current_slice->scene.extruders[current_extruder];
            speed = extruder.settings.get<Velocity>("speed_z_hop");
        }
        is_z_hopped = 0;
        currentPosition.z = current_layer_z;
        currentSpeed = speed;

#ifdef PICASO_HW_ZHOPP
        if (firmware_zhopp && flavor == EGCodeFlavor::PICASO)
        {
            // Z-axis move down
            *output_stream << "G11 F" << PrecisionedDouble{ 1, speed * 60 } << " Z" << MMtoStream{ current_layer_z };
        }
        else
#endif
        {
            *output_stream << "G1 F" << PrecisionedDouble{ 1, speed * 60 } << " Z" << MMtoStream{ current_layer_z };
        }

#ifdef ADD_GCODE_COMMENTS
        *output_stream << " ;ZHOPP:END";
#endif // ADD_GCODE_COMMENTS

        estimateCalculator.addZHopp(false);
        *output_stream << new_line;

        assert(speed > 0.0 && "Z hop speed should be positive.");
    }
}

void GCodeExport::setExtruderUsed(const size_t extruder_id)
{
    extruder_attr[extruder_id].is_used = true;
}

void GCodeExport::startExtruder(const size_t new_extruder)
{
    extruder_attr[new_extruder].is_used = true;
    if (new_extruder != current_extruder) // wouldn't be the case on the very first extruder start if it's extruder 0
    {
        switch (flavor)
        {
        case EGCodeFlavor::MAKERBOT:
            *output_stream << "M135 T" << new_extruder << new_line;
            break;

        case EGCodeFlavor::PICASO:
            *output_stream << "T1" << new_extruder << new_line; // T0 mean select tool, T10 mean select tool and wipe clean it

#ifdef PICASO_HW_RETRACT
            if (unretract_after_change)
            {
                // !!! compensate kisslicer auto retract
                // E-axis fillament down 
                *output_stream << "G11 E ;extruder_switch ;RETRACT:END" << new_line;
            }
#endif
            break;

        default:
            *output_stream << "T" << new_extruder << new_line;
            break;
        }
    }

    current_extruder = new_extruder;

    assert(getCurrentExtrudedVolume() == 0.0 && "Just after an extruder switch we haven't extruded anything yet!");
    resetExtrusionValue(); // zero the E value on the new extruder, just to be sure

    const std::string start_code = Application::getInstance().current_slice->scene.extruders[new_extruder].settings.get<std::string>("machine_extruder_start_code");

    if(!start_code.empty())
    {
        if (relative_extrusion)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the start gcode
        }

        writeCode(start_code.c_str());

        if (relative_extrusion)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }

    Application::getInstance().communication->setExtruderForSend(Application::getInstance().current_slice->scene.extruders[new_extruder]);
    Application::getInstance().communication->sendCurrentPosition(getPositionXY());

    if (flavor != EGCodeFlavor::PICASO)
    {
        //Change the Z position so it gets re-written again. We do not know if the switch code modified the Z position.
        currentPosition.z += 1;
    }

    setExtruderFanNumber(new_extruder);
}

void GCodeExport::switchExtruder(size_t new_extruder, 
	const RetractionConfig& retraction_config_old_extruder, 
	const RetractionConfig& retraction_config_new_extruder,
	coord_t perform_z_hop /*= 0*/)
{
    if (current_extruder == new_extruder)
    {
        return;
    }

    const Settings& old_extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if(old_extruder_settings.get<bool>("retraction_enable"))
    {
        constexpr bool force = true;
        constexpr bool extruder_switch = true;
        writeRetraction(retraction_config_old_extruder, force, extruder_switch);
    }

    if (perform_z_hop > 0)
    {
        writeZhopStart(perform_z_hop);
    }

    resetExtrusionValue(); // zero the E value on the old extruder, so that the current_e_value is registered on the old extruder

	if (flavor == EGCodeFlavor::PICASO)
	{
		extruder_attr[new_extruder].retraction_e_amount_current = retraction_config_new_extruder.distance;
	}

    const std::string end_code = old_extruder_settings.get<std::string>("machine_extruder_end_code");

    if(!end_code.empty())
    {
        if (relative_extrusion)
        {
            writeExtrusionMode(false); // ensure absolute extrusion mode is set before the end gcode
        }

        writeCode(end_code.c_str());

        if (relative_extrusion)
        {
            writeExtrusionMode(true); // restore relative extrusion mode
        }
    }

    if (hop_when_extruder_switch)
    {
        writeZhopStart(retraction_config_old_extruder.zHop);
    }

    startExtruder(new_extruder);
}

void GCodeExport::writeCode(const char* str)
{
    *output_stream << str << new_line;
}

void GCodeExport::writePrimeTrain(const Velocity& travel_speed)
{
    if (extruder_attr[current_extruder].is_primed)
    { // extruder is already primed once!
        return;
    }
    const Settings& extruder_settings = Application::getInstance().current_slice->scene.extruders[current_extruder].settings;
    if (extruder_settings.get<bool>("prime_blob_enable"))
    { // only move to prime position if we do a blob/poop
        // ideally the prime position would be respected whether we do a blob or not,
        // but the frontend currently doesn't support a value function of an extruder setting depending on an fdmprinter setting,
        // which is needed to automatically ignore the prime position for the printer when blob is disabled
        Point3 prime_pos(extruder_settings.get<coord_t>("extruder_prime_pos_x"), extruder_settings.get<coord_t>("extruder_prime_pos_y"), extruder_settings.get<coord_t>("extruder_prime_pos_z"));
        if (!extruder_settings.get<bool>("extruder_prime_pos_abs"))
        {
            // currentPosition.z can be already z hopped
            prime_pos += Point3(currentPosition.x, currentPosition.y, current_layer_z);
        }
        writeTravel(prime_pos, travel_speed);
    }

    if (flavor == EGCodeFlavor::GRIFFIN)
    {
        bool should_correct_z = false;
        
        std::string command = "G280";
        if (!extruder_settings.get<bool>("prime_blob_enable"))
        {
            command += " S1";  // use S1 to disable prime blob
            should_correct_z = true;
        }
        *output_stream << command << new_line;

        // There was an issue with the S1 strategy parameter, where it would only change the material-position,
        //   as opposed to 'be a prime-blob maneuvre without actually printing the prime blob', as we assumed here.
        // After a chat, the firmware-team decided to change the S1 strategy behaviour,
        //   but since people don't update their firmware at each opportunity, it was decided to fix it here as well.
        if (should_correct_z)
        {
            // Can't output via 'writeTravel', since if this is needed, the value saved for 'current height' will not be correct.
            // For similar reasons, this isn't written to the front-end via command-socket.
            *output_stream << "G0 Z" << MMtoStream{getPositionZ()} << new_line;
        }
    }
    else
    {
        // there is no prime gcode for other firmware versions...
    }

    extruder_attr[current_extruder].is_primed = true;
}

void GCodeExport::setExtruderFanNumber(int extruder)
{
    if (extruder_attr[extruder].fan_number != fan_number)
    {
        fan_number = extruder_attr[extruder].fan_number;
        current_fan_speed = -1; // ensure fan speed gcode gets output for this fan
    }
}

void GCodeExport::writeFanCommand(double speed)
{
    if (std::abs(current_fan_speed - speed) < 0.1)
    {
        return;
    }
    if(flavor == EGCodeFlavor::MAKERBOT)
    {
        if(speed >= 50)
        {
            *output_stream << "M126 T0" << new_line; //Makerbot cannot PWM the fan speed...
        }
        else
        {
            *output_stream << "M127 T0" << new_line;
        }
    }
    else if (speed > 0)
    {
        *output_stream << "M106 S" << PrecisionedDouble{1, speed * 255 / 100};
        if (fan_number)
        {
            *output_stream << " P" << fan_number;
        }
        *output_stream << new_line;
    }
    else
    {
        *output_stream << "M107";
        if (fan_number)
        {
            *output_stream << " P" << fan_number;
        }
        *output_stream << new_line;
    }

    current_fan_speed = speed;
}

void GCodeExport::writeTemperatureCommand(const size_t extruder, const Temperature& temperature, const bool wait)
{
    if (flavor == EGCodeFlavor::PICASO)
    { // The PICASO family doesn't support temperature commands (they are fixed in the firmware)
        return;
    }

    const ExtruderTrain& extruder_train = Application::getInstance().current_slice->scene.extruders[extruder];

    if (!extruder_train.settings.get<bool>("machine_nozzle_temp_enabled"))
    {
        return;
    }

    if (extruder_train.settings.get<bool>("machine_extruders_share_heater"))
    {
        // extruders share a single heater
        if (extruder != current_extruder)
        {
            // ignore all changes to the non-current extruder
            return;
        }

        // sync all extruders with the change to the current extruder
        const size_t extruder_count = Application::getInstance().current_slice->scene.extruders.size();

        for (size_t extruder_nr = 0; extruder_nr < extruder_count; extruder_nr++)
        {
            if (extruder_nr != extruder)
            {
                // only reset the other extruders' waited_for_temperature state when the new temperature
                // is greater than the old temperature
                if (wait || temperature > extruder_attr[extruder_nr].currentTemperature)
                {
                    extruder_attr[extruder_nr].waited_for_temperature = wait;
                }
                extruder_attr[extruder_nr].currentTemperature = temperature;
            }
        }
    }

    if ((!wait || extruder_attr[extruder].waited_for_temperature) && extruder_attr[extruder].currentTemperature == temperature)
    {
        return;
    }

    if (wait && flavor != EGCodeFlavor::MAKERBOT)
    {
        if(flavor == EGCodeFlavor::MARLIN)
        {
            *output_stream << "M105" << new_line; // get temperatures from the last update, the M109 will not let get the target temperature
        }
        *output_stream << "M109";
        extruder_attr[extruder].waited_for_temperature = true;
    }
    else
    {
        *output_stream << "M104";
        extruder_attr[extruder].waited_for_temperature = false;
    }
    if (extruder != current_extruder)
    {
        *output_stream << " T" << extruder;
    }
#ifdef ASSERT_INSANE_OUTPUT
    assert(temperature >= 0);
#endif // ASSERT_INSANE_OUTPUT
    *output_stream << " S" << PrecisionedDouble{1, temperature} << new_line;
    if (extruder != current_extruder && always_write_active_tool)
    {
        //Some firmwares (ie Smoothieware) change tools every time a "T" command is read - even on a M104 line, so we need to switch back to the active tool.
        *output_stream << "T" << current_extruder << new_line;
    }
    if (wait && flavor == EGCodeFlavor::MAKERBOT)
    {
        //Makerbot doesn't use M109 for heat-and-wait. Instead, use M104 and then wait using M116.
        *output_stream << "M116" << new_line;
    }
    extruder_attr[extruder].currentTemperature = temperature;
}

void GCodeExport::writeBedTemperatureCommand(const Temperature& temperature, const bool wait)
{
    if (flavor == EGCodeFlavor::PICASO)
    { // The PICASO family doesn't support temperature commands (they are fixed in the firmware)
        return;
    }
    if (flavor == EGCodeFlavor::ULTIGCODE)
    { // The UM2 family doesn't support temperature commands (they are fixed in the firmware)
        return;
    }

    if (wait)
    {
        if(flavor == EGCodeFlavor::MARLIN)
        {
            *output_stream << "M140 S"; // set the temperature, it will be used as target temperature from M105
            *output_stream << PrecisionedDouble{1, temperature} << new_line;
            *output_stream << "M105" << new_line;
        }

        *output_stream << "M190 S";
    }
    else
        *output_stream << "M140 S";
    *output_stream << PrecisionedDouble{1, temperature} << new_line;
}

void GCodeExport::writeBuildVolumeTemperatureCommand(const Temperature& temperature, const bool wait)
{
    if (flavor == EGCodeFlavor::PICASO)
    { // The PICASO family doesn't support build volume temperature commands.
        return;
    }
    if (flavor == EGCodeFlavor::ULTIGCODE || flavor == EGCodeFlavor::GRIFFIN)
    {
        //Ultimaker printers don't support build volume temperature commands.
        return;
    }
    if (wait)
    {
        *output_stream << "M191 S";
    }
    else
    {
        *output_stream << "M141 S";
    }
    *output_stream << PrecisionedDouble{1, temperature} << new_line;
}

void GCodeExport::writePrintAcceleration(const Acceleration& acceleration)
{
    switch (getFlavor())
    {
        case EGCodeFlavor::REPETIER:
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M201 X" << PrecisionedDouble{0, acceleration} << " Y" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::REPRAP:
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M204 P" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::PICASO:
            // acceleration embeded in firmware
            break;
        default:
            // MARLIN, etc. only have one acceleration for both print and travel
            if (current_print_acceleration != acceleration)
            {
                *output_stream << "M204 S" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
    }
    current_print_acceleration = acceleration;
    estimateCalculator.setAcceleration(acceleration);
}

void GCodeExport::writeTravelAcceleration(const Acceleration& acceleration)
{
    switch (getFlavor())
    {
        case EGCodeFlavor::REPETIER:
            if (current_travel_acceleration != acceleration)
            {
                *output_stream << "M202 X" << PrecisionedDouble{0, acceleration} << " Y" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::REPRAP:
            if (current_travel_acceleration != acceleration)
            {
                *output_stream << "M204 T" << PrecisionedDouble{0, acceleration} << new_line;
            }
            break;
        case EGCodeFlavor::PICASO:
            // acceleration embeded in firmware
            break;
        default:
            // MARLIN, etc. only have one acceleration for both print and travel
            writePrintAcceleration(acceleration);
            break;
    }
    current_travel_acceleration = acceleration;
    estimateCalculator.setAcceleration(acceleration);
}

void GCodeExport::writeJerk(const Velocity& jerk)
{
    if (current_jerk != jerk)
    {
        switch (getFlavor())
        {
            case EGCodeFlavor::REPETIER:
                *output_stream << "M207 X" << PrecisionedDouble{2, jerk} << new_line;
                break;
            case EGCodeFlavor::REPRAP:
                *output_stream << "M566 X" << PrecisionedDouble{2, jerk * 60} << " Y" << PrecisionedDouble{2, jerk * 60} << new_line;
                break;
			case EGCodeFlavor::PICASO:
				// jerk embeded in firmware
				break;
            default:
                *output_stream << "M205 X" << PrecisionedDouble{2, jerk} << " Y" << PrecisionedDouble{2, jerk} << new_line;
                break;
        }
        current_jerk = jerk;
        estimateCalculator.setMaxXyJerk(jerk);
    }
}

void GCodeExport::finalize(const char* endCode)
{
    writeFanCommand(0);
    writeCode(endCode);
    int64_t print_time = getSumTotalPrintTimes();
    int mat_0 = getTotalFilamentUsed(0);
    log("Print time (s): %d\n", print_time);
    log("Print time (hr|min|s): %dh %dm %ds\n", int(print_time / 60 / 60), int((print_time / 60) % 60), int(print_time % 60));
    log("Filament (mm^3): %d\n", mat_0);
    for(int n=1; n<MAX_EXTRUDERS; n++)
        if (getTotalFilamentUsed(n) > 0)
            log("Filament%d: %d\n", n + 1, int(getTotalFilamentUsed(n)));

    if (flavor == EGCodeFlavor::PICASO)
    {
        std::ostringstream lines;

        lines << "; Print time: " << print_time << new_line;
        lines << "; Print time (readable): " << int(print_time / 60 / 60) << "h " << int((print_time / 60) % 60) << "m " << int(print_time % 60) << "s" << new_line;
        lines << "; Filament[0]: " << PrecisionedDouble{ 2, getTotalFilamentUsed(0) } << new_line;
        for (int n = 1; n < MAX_EXTRUDERS; n++)
            if (getTotalFilamentUsed(n) > 0)
                lines << "; Filament[" << n << "]: " << PrecisionedDouble{ 2, getTotalFilamentUsed(n) } << new_line;
        writeCode(lines.str().c_str());
    }
    output_stream->flush();
}

double GCodeExport::getExtrudedVolumeAfterLastWipe(size_t extruder)
{
    return eToMm3(extruder_attr[extruder].last_e_value_after_wipe, extruder);
}

void GCodeExport::ResetLastEValueAfterWipe(size_t extruder)
{
    extruder_attr[extruder].last_e_value_after_wipe = 0;
}

void GCodeExport::insertWipeScript(const WipeScriptConfig& wipe_config)
{
    const Point3 prev_position = currentPosition;
    writeComment("WIPE_SCRIPT_BEGIN");

    if (wipe_config.retraction_enable)
    {
        writeRetraction(wipe_config.retraction_config);
    }

    if (wipe_config.hop_enable)
    {
        writeZhopStart(wipe_config.hop_amount, wipe_config.hop_speed);
    }

    writeTravel(Point(wipe_config.brush_pos_x, currentPosition.y), wipe_config.move_speed);
    for (size_t i = 0; i < wipe_config.repeat_count; ++i)
    {
        coord_t x = currentPosition.x + (i % 2 ? -wipe_config.move_distance : wipe_config.move_distance);
        writeTravel(Point(x, currentPosition.y), wipe_config.move_speed);
    }

    writeTravel(prev_position, wipe_config.move_speed);

    if (wipe_config.hop_enable)
    {
        writeZhopEnd(wipe_config.hop_speed);
    }

    if (wipe_config.retraction_enable)
    {
        writeUnretractionAndPrime();
    }

    if (wipe_config.pause > 0)
    {
        writeDelay(wipe_config.pause);
    }

    writeComment("WIPE_SCRIPT_END");
}

}//namespace cura

