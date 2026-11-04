# SPDX-License-Identifier: LicenseRef-EasyFly-Restricted-1.0
# SPDX-FileCopyrightText: 2026 贾国源
# License edition: EF-LICENSE-20261103-02; bundle: share/easyfly_msgs/licenses/.
# This revision offers no Apache-2.0 option for the listed original code.
cmake_minimum_required(VERSION 3.8)
project(easyfly_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

set(msg_files
  "msg/PerceptionContextV2.msg"
  "msg/ObservationRefV2.msg"
  "msg/PixelBoxV2.msg"
  "msg/TrackKeyV2.msg"
  "msg/ClassifiedObstacleArrayV2.msg"
  "msg/DetectionV2.msg"
  "msg/DetectionArrayV2.msg"
  "msg/ImageTrackV2.msg"
  "msg/ImageTrackArrayV2.msg"
  "msg/WorldTargetV2.msg"
  "msg/WorldTargetArrayV2.msg"
  "msg/TargetPredictionV2.msg"
  "msg/TargetPredictionArrayV2.msg"
  "msg/SingleTrackStateV2.msg"
  "msg/VehicleIdentityV2.msg"
  "msg/VehicleIdentityArrayV2.msg"

  "msg/VoxelGrid.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/SepticTrajectory.msg"
  "msg/NavigationSession.msg"
  "msg/UAVState.msg"
  "msg/FlightStatus.msg"
  "msg/UAVCommand.msg"
  "msg/DetectionInfo.msg"
  "msg/DetectionInfoArray.msg"
  "msg/EllipseObservation.msg"
  "msg/EllipseObservationArray.msg"
  "msg/ColorLineObservation.msg"
  "msg/ColorLineObservationArray.msg"
  "msg/TrackInfo.msg"
  "msg/TrackInfoArray.msg"
  "msg/VehicleIdentity.msg"
  "msg/VehicleIdentityArray.msg"
  "msg/Target3D.msg"
  "msg/Target3DArray.msg"
  "msg/TargetPrediction.msg"
  "msg/TargetPredictionArray.msg"
  "msg/TimeSyncStatus.msg"
  "msg/LocalizationStatus.msg"
  "msg/TagObservation.msg"
  "msg/TagObservationArray.msg"
  "msg/LandingTarget.msg"
  "msg/LandingGuidanceTarget.msg"
  "msg/SingleTrackState.msg"
  "msg/ClassifiedObstacle.msg"
  "msg/ClassifiedObstacleArray.msg"
  "msg/ObstacleInfo.msg"
  "msg/ObstacleInfoArray.msg"
  "msg/GimbalState.msg"
  "msg/SwarmInfo.msg"
  "msg/MissionStatus.msg"
  "msg/MissionStepFeedback.msg"
)

set(srv_files
  "srv/GetPerceptionV2.srv"
  "srv/ManageSingleTrackV2.srv"
  "srv/SetExplorationView.srv"
  "srv/ReserveNavigation.srv"
  "srv/GetPerception.srv"
  "srv/GetCapabilities.srv"
  "srv/GetTrajectoryState.srv"
  "srv/ManageMission.srv"
  "srv/GetMissionDetails.srv"
  "srv/ManageReviewedMission.srv"
  "srv/ImportMission.srv"
  "srv/GetActionOwnership.srv"
  "srv/ManageSingleTrack.srv"
  "srv/SetNavigationSession.srv"
  "srv/StopNavigation.srv"
  "srv/ArmMotors.srv"
  "srv/SetFlightMode.srv"
  "srv/GetObstacles.srv"
  "srv/TriggerLanding.srv"
  "srv/SetGimbalAngle.srv"
  "srv/SetGimbalTracking.srv"
  "srv/QueryParams.srv"
)

set(action_files
  "action/Explore.action"
  "action/Takeoff.action"
  "action/Land.action"
  "action/PrecisionLand.action"
  "action/GoToWaypoint.action"
  "action/TrackTarget.action"
  "action/FollowTrajectory.action"
  "action/NavigateTo.action"
  "action/SearchArea.action"
  "action/ExecuteMission.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces unique_identifier_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# EasyFly license bundle: copied from this package, with no workspace-root dependency.
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")

ament_package()
