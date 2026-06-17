#!/usr/bin/env python3

import math
import os

import rospy
import yaml
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import Waypoint, WaypointList
from mavros_msgs.srv import WaypointClear, WaypointPull, WaypointPush, WaypointSetCurrent

try:
    from airsim_ros_pkgs.msg import GPSYaw
except ImportError:
    GPSYaw = None


class MissionWaypointUploader:
    EARTH_RADIUS_M = 6378137.0
    MAV_CMD_NAV_WAYPOINT = 16

    def __init__(self):
        self.waypoint_file = rospy.get_param("~waypoint_file", "")
        self.ready_param = rospy.get_param("~mission_ready_param", "/mission_auto_avoid/mission_ready")
        self.start_wp_seq = int(rospy.get_param("~start_wp_seq", 0))
        self.set_current_after_upload = rospy.get_param("~set_current_after_upload", False)
        self.prepend_home_placeholder = rospy.get_param("~prepend_home_placeholder", True)
        self.waypoint_hold_time = max(0.0, float(rospy.get_param("~waypoint_hold_time", 3.0)))
        self.origin_timeout = rospy.get_param("~origin_timeout", 15.0)
        self.service_timeout = rospy.get_param("~service_timeout", 15.0)
        self.pull_verify_timeout = rospy.get_param("~pull_verify_timeout", 5.0)

        self.origin = None
        self.latest_waypoint_list = None

        rospy.set_param(self.ready_param, False)

        rospy.Subscriber("/mavros/global_position/gp_origin", GeoPointStamped, self._geo_point_origin_callback, queue_size=1)
        if GPSYaw is not None:
            rospy.Subscriber("/airsim_node/origin_geo_point", GPSYaw, self._airsim_origin_callback, queue_size=1)
        else:
            rospy.logwarn("mission_auto_avoid: airsim_ros_pkgs is unavailable; skipping AirSim origin topic.")
        rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self._waypoint_list_callback, queue_size=1)

        self.clear_client = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
        self.pull_client = rospy.ServiceProxy("/mavros/mission/pull", WaypointPull)
        self.push_client = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
        self.set_current_client = rospy.ServiceProxy("/mavros/mission/set_current", WaypointSetCurrent)

        self._run()

    def _run(self):
        local_waypoints = self._load_waypoints(self.waypoint_file)
        if not local_waypoints:
            rospy.logerr("mission_auto_avoid: no mission waypoints loaded, cannot upload FCU mission.")
            return

        if not self._wait_for_origin():
            rospy.logerr("mission_auto_avoid: no geographic origin received, cannot upload FCU mission.")
            return

        if not self._wait_for_services():
            rospy.logerr("mission_auto_avoid: mavros mission services are unavailable, cannot upload FCU mission.")
            return

        mission_waypoints = self._build_global_waypoints(local_waypoints)
        if not mission_waypoints:
            rospy.logerr("mission_auto_avoid: failed to build FCU mission waypoints.")
            return

        upload_waypoints = self._build_upload_waypoints(mission_waypoints)
        if not upload_waypoints:
            rospy.logerr("mission_auto_avoid: failed to build FCU upload waypoint list.")
            return

        if self.start_wp_seq < 0 or self.start_wp_seq >= len(mission_waypoints):
            rospy.logerr(
                "mission_auto_avoid: start_wp_seq=%d is out of range for %d mission waypoints.",
                self.start_wp_seq,
                len(mission_waypoints),
            )
            return

        fcu_start_wp_seq = self.start_wp_seq + 1 if self.prepend_home_placeholder else self.start_wp_seq

        self._log_planned_waypoints(local_waypoints, mission_waypoints)
        self._log_upload_waypoint_strategy(upload_waypoints, fcu_start_wp_seq)

        try:
            clear_resp = self.clear_client()
        except rospy.ServiceException as exc:
            rospy.logerr("mission_auto_avoid: failed to clear FCU mission: %s", exc)
            return

        if not clear_resp.success:
            rospy.logerr("mission_auto_avoid: FCU rejected mission clear.")
            return

        try:
            push_resp = self.push_client(start_index=0, waypoints=upload_waypoints)
        except rospy.ServiceException as exc:
            rospy.logerr("mission_auto_avoid: failed to push FCU mission: %s", exc)
            return

        if not push_resp.success or push_resp.wp_transfered != len(upload_waypoints):
            rospy.logerr(
                "mission_auto_avoid: FCU mission push failed. success=%s transferred=%d expected=%d",
                push_resp.success,
                push_resp.wp_transfered,
                len(upload_waypoints),
            )
            return

        self._verify_uploaded_mission()

        if self.set_current_after_upload:
            if self.start_wp_seq == 0:
                rospy.logwarn(
                    "mission_auto_avoid: set_current_after_upload=true with start_wp_seq=0. "
                    "FCU current waypoint request will use seq=%d.",
                    fcu_start_wp_seq,
                )
            try:
                set_current_resp = self.set_current_client(wp_seq=fcu_start_wp_seq)
            except rospy.ServiceException as exc:
                rospy.logerr("mission_auto_avoid: failed to set current FCU mission waypoint: %s", exc)
                return

            if not set_current_resp.success:
                rospy.logerr("mission_auto_avoid: FCU rejected current mission waypoint selection.")
                return

            rospy.logwarn(
                "mission_auto_avoid: uploaded %d FCU mission waypoints from %s and explicitly requested current waypoint %d.",
                len(upload_waypoints),
                self.waypoint_file,
                fcu_start_wp_seq,
            )
        else:
            rospy.logwarn(
                "mission_auto_avoid: uploaded %d FCU mission waypoints from %s without calling set_current. "
                "This avoids ArduPilot mission seq 0 ambiguity.",
                len(upload_waypoints),
                self.waypoint_file,
            )

        rospy.set_param(self.ready_param, True)

    def _load_waypoints(self, path):
        if not path or not os.path.isfile(path):
            rospy.logerr("mission_auto_avoid: waypoint file not found: %s", path)
            return []

        try:
            with open(path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except Exception as exc:
            rospy.logerr("mission_auto_avoid: failed to read waypoint file %s: %s", path, exc)
            return []

        mission_cfg = data.get("mission", data) if isinstance(data, dict) else {}
        raw_waypoints = mission_cfg.get("waypoints", []) if isinstance(mission_cfg, dict) else []
        if not isinstance(raw_waypoints, list):
            rospy.logerr("mission_auto_avoid: mission.waypoints must be a list in %s", path)
            return []

        waypoints = []
        for idx, item in enumerate(raw_waypoints):
            waypoint = self._parse_waypoint_item(item, idx, path)
            if waypoint is not None:
                waypoints.append(waypoint)
        return waypoints

    def _parse_waypoint_item(self, item, idx, path):
        if isinstance(item, dict):
            x = item.get("x")
            y = item.get("y")
            z = item.get("z")
        elif isinstance(item, (list, tuple)) and len(item) >= 3:
            x, y, z = item[0], item[1], item[2]
        else:
            rospy.logwarn("mission_auto_avoid: invalid waypoint #%d in %s", idx, path)
            return None

        try:
            return (float(x), float(y), float(z))
        except (TypeError, ValueError):
            rospy.logwarn("mission_auto_avoid: waypoint #%d has non-numeric coordinates in %s", idx, path)
            return None

    def _geo_point_origin_callback(self, msg):
        if self.origin is None:
            self.origin = (
                float(msg.position.latitude),
                float(msg.position.longitude),
                float(msg.position.altitude),
            )

    def _airsim_origin_callback(self, msg):
        if self.origin is None:
            self.origin = (
                float(msg.latitude),
                float(msg.longitude),
                float(msg.altitude),
            )

    def _waypoint_list_callback(self, msg):
        self.latest_waypoint_list = msg

    def _wait_for_origin(self):
        deadline = rospy.Time.now() + rospy.Duration(self.origin_timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.origin is None:
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return self.origin is not None

    def _wait_for_services(self):
        services = [
            "/mavros/mission/clear",
            "/mavros/mission/pull",
            "/mavros/mission/push",
            "/mavros/mission/set_current",
        ]
        timeout = self.service_timeout
        for service_name in services:
            try:
                rospy.wait_for_service(service_name, timeout=timeout)
            except rospy.ROSException:
                return False
        return True

    def _build_global_waypoints(self, local_waypoints):
        mission_waypoints = []
        for idx, local_waypoint in enumerate(local_waypoints):
            latitude, longitude = self._enu_to_geodetic(local_waypoint[0], local_waypoint[1])

            waypoint = Waypoint()
            waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            waypoint.command = self.MAV_CMD_NAV_WAYPOINT
            waypoint.is_current = False
            waypoint.autocontinue = True
            waypoint.param1 = self.waypoint_hold_time
            waypoint.param2 = 0.5
            waypoint.param3 = 0.0
            waypoint.param4 = float("nan")
            waypoint.x_lat = latitude
            waypoint.y_long = longitude
            waypoint.z_alt = local_waypoint[2]
            mission_waypoints.append(waypoint)
        return mission_waypoints

    def _build_upload_waypoints(self, mission_waypoints):
        if not mission_waypoints:
            return []

        upload_waypoints = []
        if self.prepend_home_placeholder:
            # ArduPilot reserves seq=0 for HOME and overwrites the first uploaded item.
            # Insert a placeholder so the real mission starts at seq=1.
            home_placeholder = Waypoint()
            home_placeholder.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            home_placeholder.command = self.MAV_CMD_NAV_WAYPOINT
            home_placeholder.is_current = False
            home_placeholder.autocontinue = True
            home_placeholder.param1 = self.waypoint_hold_time
            home_placeholder.param2 = 0.5
            home_placeholder.param3 = 0.0
            home_placeholder.param4 = float("nan")
            home_placeholder.x_lat = self.origin[0]
            home_placeholder.y_long = self.origin[1]
            home_placeholder.z_alt = mission_waypoints[0].z_alt
            upload_waypoints.append(home_placeholder)

        upload_waypoints.extend(mission_waypoints)
        return upload_waypoints

    def _log_planned_waypoints(self, local_waypoints, mission_waypoints):
        for idx, (local_wp, mission_wp) in enumerate(zip(local_waypoints, mission_waypoints)):
            rospy.loginfo(
                "mission_auto_avoid: mission plan idx=%d local=(%.2f, %.2f, %.2f) global=(%.7f, %.7f, %.2f)",
                idx,
                local_wp[0],
                local_wp[1],
                local_wp[2],
                mission_wp.x_lat,
                mission_wp.y_long,
                mission_wp.z_alt,
            )

    def _log_upload_waypoint_strategy(self, upload_waypoints, fcu_start_wp_seq):
        rospy.loginfo(
            "mission_auto_avoid: upload_waypoints=%d prepend_home_placeholder=%s fcu_start_wp_seq=%d",
            len(upload_waypoints),
            str(self.prepend_home_placeholder),
            fcu_start_wp_seq,
        )

    def _verify_uploaded_mission(self):
        try:
            pull_resp = self.pull_client()
        except rospy.ServiceException as exc:
            rospy.logwarn("mission_auto_avoid: failed to pull FCU mission for verification: %s", exc)
            return

        if not pull_resp.success:
            rospy.logwarn("mission_auto_avoid: FCU mission pull reported failure during verification.")
            return

        try:
            waypoint_list = rospy.wait_for_message(
                "/mavros/mission/waypoints",
                WaypointList,
                timeout=self.pull_verify_timeout,
            )
        except rospy.ROSException:
            waypoint_list = self.latest_waypoint_list

        if waypoint_list is None:
            rospy.logwarn("mission_auto_avoid: no /mavros/mission/waypoints message received after pull.")
            return

        rospy.loginfo(
            "mission_auto_avoid: FCU mission verification current_seq=%d count=%d.",
            waypoint_list.current_seq,
            len(waypoint_list.waypoints),
        )
        for idx, waypoint in enumerate(waypoint_list.waypoints):
            rospy.loginfo(
                "mission_auto_avoid: FCU mission idx=%d current=%s frame=%d cmd=%d lat=%.7f lon=%.7f alt=%.2f",
                idx,
                str(waypoint.is_current),
                waypoint.frame,
                waypoint.command,
                waypoint.x_lat,
                waypoint.y_long,
                waypoint.z_alt,
            )

    def _enu_to_geodetic(self, east_m, north_m):
        lat0_deg, lon0_deg, _alt0 = self.origin
        lat0_rad = math.radians(lat0_deg)

        d_lat = north_m / self.EARTH_RADIUS_M
        d_lon = east_m / (self.EARTH_RADIUS_M * max(math.cos(lat0_rad), 1e-9))

        latitude = lat0_deg + math.degrees(d_lat)
        longitude = lon0_deg + math.degrees(d_lon)
        return latitude, longitude


if __name__ == "__main__":
    rospy.init_node("mission_waypoint_uploader")
    MissionWaypointUploader()
    rospy.spin()
