"""
Tests that can be applied to real test-flights as well
as simulated test-flights
"""

from pyulgresample import attitude
from pyulgresample import localposition as lpos
from pyulgresample import ulogconv
from pyulgresample import loginfo
import pyulog
import os
import numpy as np
import pytest


def setup_dataframe(filepath, dfulgClass, topics):
    try:
        dfulg = dfulgClass.create(filepath, topics)
        return dfulg

    except Exception:
        pytest.skip("Could not create dfulg object")


class TestAttitude:
    """
    Test Attitude related constraints
    """

    @classmethod
    def setup_class(cls):
        cls._dfulgClass = attitude.dfUlgAttitude
        cls._topics = attitude.dfUlgAttitude.get_required_topics()
        cls._topics.append("vehicle_status")

    def test_tilt_desired(self, filepath):
        dfulg = setup_dataframe(filepath, self._dfulgClass, self._topics)

        # During Manual / Stabilized and Altitude, the tilt threshdol should not exceed
        # MPC_MAN_TILT_MAX
        attitude.add_desired_tilt(dfulg.df)
        man_tilt = (
            loginfo.get_param(dfulg.ulog, "MPC_MAN_TILT_MAX", 0) * np.pi / 180
        )
        assert dfulg.df[
            (
                (dfulg.df.T_vehicle_status_0__F_nav_state == 0)
                | (dfulg.df.T_vehicle_status_0__F_nav_state == 1)
            )
            & (
                dfulg.df.T_vehicle_attitude_setpoint_0__NF_tilt_desired
                > man_tilt
            )
        ].empty


class TestRTLHeight:
    # The return to home height changes with the distance from home
    # mode was triggered
    # check the height above ground while the drone returns to home. compare it with
    # the allowed maximum or minimum heights, until the drone has reached home and motors have been turned off

    @classmethod
    def setup_class(cls):
        cls._dfulgClass = attitude.dfUlgAttitude
        cls._topics = attitude.dfUlgAttitude.get_required_topics()
        cls._topics.extend(["vehicle_local_position", "vehicle_status"])

    def test_rtl(self, filepath):
        dfulg = setup_dataframe(filepath, self._dfulgClass, self._topics)

        # drone parameters: below rtl_min_dist, the drone follows different rules than outside of it.
        rtl_min_dist = loginfo.get_param(dfulg.ulog, "RTL_MIN_DIST", 0)
        rtl_return_alt = loginfo.get_param(dfulg.ulog, "RTL_RETURN_ALT", 0)
        rtl_cone_dist = loginfo.get_param(dfulg.ulog, "RTL_CONE_DIST", 0)

        NAVIGATION_STATE_AUTO_RTL = (
            5
        )  # see https://github.com/PX4/Firmware/blob/master/msg/vehicle_status.msg
        thresh = 1  # Threshold for position inaccuracies, in meters

        lpos.add_horizontal_distance(dfulg.df)

        # Run the test every time that RTL was triggered
        dfulg.df["T_vehicle_status_0__F_nav_state_group2"] = (
            dfulg.df.T_vehicle_status_0__F_nav_state
            != dfulg.df.T_vehicle_status_0__F_nav_state.shift()
        ).cumsum()
        state_group = dfulg.df.groupby(
            ["T_vehicle_status_0__F_nav_state_group2"]
        )
        for g, d in state_group:
            # Check that RTL was actually triggered
            # at least two consecutive T_vehicle_status_0__F_nav_state values have to
            # be equal to NAVIGATION_STATE_AUTO_RTL in order to confirm that RTL has been triggered
            if (
                d.T_vehicle_status_0__F_nav_state.count() > 1
                and d.T_vehicle_status_0__F_nav_state[0]
                == NAVIGATION_STATE_AUTO_RTL
            ):
                height_at_RTL = abs(d.T_vehicle_local_position_0__F_z[0])
                distance_at_RTL = d.T_vehicle_local_position_0__NF_abs_horizontal_dist[
                    0
                ]
                max_height_during_RTL = abs(
                    max(d.T_vehicle_local_position_0__F_z)
                )

                if rtl_cone_dist > 0:
                    # Drone should not rise higher than height defined by a cone (definition taken from rtl.cpp file in firmware)
                    max_height_within_RTL_MIN_DIST = 2 * distance_at_RTL
                else:
                    # If no cone is defined, drone should not rise at all within certain radius around home
                    max_height_within_RTL_MIN_DIST = height_at_RTL

                # check if a value of the z position after triggering RTL is larger than allowed value
                if (distance_at_RTL < rtl_min_dist) & (
                    height_at_RTL < max_height_within_RTL_MIN_DIST
                ):
                    assert (
                        max_height_during_RTL
                        < max_height_within_RTL_MIN_DIST + thresh
                    )

                elif (distance_at_RTL < rtl_min_dist) & (
                    height_at_RTL >= max_height_within_RTL_MIN_DIST
                ):
                    assert max_height_during_RTL < height_at_RTL + thresh

                elif (distance_at_RTL >= rtl_min_dist) & (
                    height_at_RTL < rtl_return_alt
                ):
                    assert max_height_during_RTL < rtl_return_alt + thresh

                elif (distance_at_RTL >= rtl_min_dist) & (
                    height_at_RTL > rtl_return_alt
                ):
                    assert max_height_during_RTL < height_at_RTL + thresh
