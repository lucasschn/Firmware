/**
 * @file UtilityFunctions.cpp
 */

#include "UtilityFunctions.hpp"

bool highEnoughForLandingGear(const float alt_threshold, const float reference_alt)
{

	return  reference_alt > alt_threshold;

}
