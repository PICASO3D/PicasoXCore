//Copyright (c) 2018 Ultimaker B.V.
//Copyright (c) 2022 PICASO 3D
//PicasoXCore is released under the terms of the AGPLv3 or higher

#ifndef TIME_ESTIMATE_H
#define TIME_ESTIMATE_H

#include <stdint.h>
#include <vector>
#include <map>
#include <unordered_map>

#include "PrintFeature.h"
#include "settings/types/Duration.h" //Print time estimates.
#include "settings/types/Velocity.h" //Speeds and accelerations at which we print.

namespace cura
{

class Ratio;
class Settings;

class PicasoSpeedScale
{
public:
	Velocity base_speed;
	bool allow_scale;

	PicasoSpeedScale()
		: base_speed(0)
		, allow_scale(false)
	{
	}

	PicasoSpeedScale(Velocity base_speed, bool allow_scale)
		: base_speed(base_speed)
		, allow_scale(allow_scale)
	{
	}
};

class PicasoPrintModeEstimate
{
public:
	std::map<size_t, std::vector<Duration>> durations;
	Duration extra_time;
	size_t extruder_count;
	size_t retract_count;
	size_t zhopp_count;

	PicasoPrintModeEstimate()
		: durations()
		, extruder_count(0)
		, extra_time(0)
		, retract_count(0)
		, zhopp_count(0)
	{
	}

	PicasoPrintModeEstimate(size_t extruder_count, Duration extra_time, size_t retract_count, size_t zhopp_count)
		: durations()
		, extruder_count(extruder_count)
		, extra_time(extra_time)
		, retract_count(retract_count)
		, zhopp_count(zhopp_count)
	{
		ReInit(extruder_count);
	}

	void ReInit(size_t extruder_count)
	{
		this->extruder_count = extruder_count;
		for (size_t n = 0; n < extruder_count; n++)
		{
			auto it = durations.find(n);
			if (it == durations.end())
			{
				std::vector<Duration> totals(static_cast<unsigned char>(PicasoPrintMode::NumPicasoPrintModes), 0.0);
				durations[n] = totals;
			}
			else
			{
				for (size_t i = 0; i < it->second.size(); i++)
				{
					it->second[i] = 0.0;
				}
			}
		}
	}
};


class TimeEstimateResult
{
public:
	std::map<size_t, std::vector<Duration>> durations;
	Duration extra_time;
	size_t extruder_count;
	size_t retract_count;
	size_t zhopp_count;

	TimeEstimateResult()
		: durations()
		, extruder_count(0)
		, extra_time(0)
		, retract_count(0)
		, zhopp_count(0)
	{
	}

	void ReInit(size_t extruder_count)
	{
		this->extruder_count = extruder_count;
		for (size_t n = 0; n < extruder_count; n++)
		{
			auto it = durations.find(n);
			if (it == durations.end())
			{
				std::vector<Duration> totals(static_cast<unsigned char>(PathConfigFeature::NumPathConfigFeatures), 0.0);
				durations[n] = totals;
			}
			else
			{
				for (size_t i = 0; i < it->second.size(); i++)
				{
					it->second[i] = 0.0;
				}
			}
		}
	}

	TimeEstimateResult(size_t extruder_count, Duration extra_time, size_t retract_count, size_t zhopp_count)
		: durations()
		, extruder_count(extruder_count)
		, extra_time(extra_time)
		, retract_count(retract_count)
		, zhopp_count(zhopp_count)
	{ 
		ReInit(extruder_count);
	}

	// for ArcusCommunication::sendPrintTimeMaterialEstimates()
	std::vector<Duration> getPrintFeatureTypes()
	{
		std::vector<Duration> result(static_cast<unsigned char>(PrintFeatureType::NumPrintFeatureTypes), 0.0);

		// Extra time (pause for minimum layer time, etc) is marked as NoneType
		result[static_cast<unsigned char>(PrintFeatureType::NoneType)] = extra_time;

		for (auto it = durations.begin(); it != durations.end(); ++it)
		{
			for (size_t n = 0; n < it->second.size(); n++)
			{
				PrintFeatureType pft = PrintFeatureConverter::toPrintFeatureType(static_cast<PathConfigFeature>(n));

				result[static_cast<unsigned char>(pft)] += it->second[n];
			}
		}

		return result;
	}

	void Reset()
	{
		this->extra_time = 0;
		this->retract_count = 0;
		this->zhopp_count = 0;

		for (auto it = durations.begin(); it != durations.end(); ++it)
		{
			for (size_t i = 0; i < it->second.size(); i++)
			{
				it->second[i] = 0.0;
			}
		}
	}

	void Add(const TimeEstimateResult& other)
	{
		extra_time += other.extra_time;
		retract_count += other.retract_count;
		zhopp_count += other.zhopp_count;

		for (auto it = durations.begin(); it != durations.end(); ++it)
		{
			auto other_it = other.durations.find(it->first);
			if (other_it != other.durations.end())
			{
				for (size_t i = 0; i < it->second.size(); i++)
				{
					it->second[i] += other_it->second[i];
				}
			}
		}
	}
};

/*!
 *  The TimeEstimateCalculator class generates a estimate of printing time calculated with acceleration in mind.
 *  Some of this code has been adapted from the Marlin sources.
 */

class TimeEstimateCalculator
{
public:
    constexpr static unsigned int NUM_AXIS = 4;
    constexpr static unsigned int X_AXIS = 0;
    constexpr static unsigned int Y_AXIS = 1;
    constexpr static unsigned int Z_AXIS = 2;
    constexpr static unsigned int E_AXIS = 3;


    class Position
    {
    public:
        Position() {for(unsigned int n=0;n<NUM_AXIS;n++) axis[n] = 0;}
        Position(double x, double y, double z, double e) {axis[0] = x;axis[1] = y;axis[2] = z;axis[3] = e;}
        double axis[NUM_AXIS];
        
        double& operator[](const int n) { return axis[n]; }
    };

    class Block
    {
    public:
        bool recalculate_flag;
        
        double accelerate_until;
        double decelerate_after;
        Velocity initial_feedrate;
        Velocity final_feedrate;

        Velocity entry_speed;
        Velocity max_entry_speed;
        bool nominal_length_flag;
        
        Velocity nominal_feedrate;
        double maxTravel;
        double distance;
        Acceleration acceleration;
        Position delta;
        Position absDelta;

        PrintFeatureType feature;

		PathConfigFeature featureType;
		size_t extruder;
    };

private:
    Velocity max_feedrate[NUM_AXIS] = {600, 600, 40, 25}; // mm/s
    Velocity minimumfeedrate = 0.01;
    Acceleration acceleration = 3000;
    Acceleration max_acceleration[NUM_AXIS] = {9000, 9000, 100, 10000};
    Velocity max_xy_jerk = 20.0;
    Velocity max_z_jerk = 0.4;
    Velocity max_e_jerk = 5.0;
    Duration extra_time = 0.0;
	size_t extruder_count = 1;

    int retract_start = 0;
    int retract_end = 0;
    int zhopp_start = 0;
    int zhopp_end = 0;
    
    Position previous_feedrate;
    Velocity previous_nominal_feedrate;

    Position currentPosition;

    std::vector<Block> blocks;
public:
    /*!
     * \brief Set the movement configuration of the firmware.
     * \param settings_base Where to get the settings from.
     */
    void setFirmwareDefaults(const Settings& settings);
    void setPosition(Position newPos);
    void plan(Position newPos, Velocity feedRate, PrintFeatureType feature, PathConfigFeature featureType, size_t extruder);
    void addTime(const Duration& time);
    void setAcceleration(const Acceleration& acc); //!< Set the default acceleration to \p acc
    void setMaxXyJerk(const Velocity& jerk); //!< Set the max xy jerk to \p jerk

    void addRetract(bool start);
    void addZHopp(bool start);

    double getMaxXFeedrate();

    void reset();
    
    std::vector<Duration> calculate();
	TimeEstimateResult getTimeEstimateResult();
private:
    void reverse_pass();
    void forward_pass();
    void recalculate_trapezoids();

    void calculate_trapezoid_for_block(Block *block, const Ratio entry_factor, const Ratio exit_factor);
    void planner_reverse_pass_kernel(Block *previous, Block *current, Block *next);
    void planner_forward_pass_kernel(Block *previous, Block *current, Block *next);
};

}//namespace cura
#endif//TIME_ESTIMATE_H
