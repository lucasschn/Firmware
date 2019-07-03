/**
 * @file UtilityFunctions.hpp
 *
 * This file is used to some utility functions.
 */

#pragma once

/** Return true if the current height allows landing gear operation */
bool highEnoughForLandingGear(const float alt_threshold, const float reference_alt);
