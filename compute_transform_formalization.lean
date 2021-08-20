import .phys.time.time
import .phys.time_series.geom3d
import .standards.time_std
import .standards.geom3d_std
noncomputable theory

/-
We need to assume a physical interpretation of the data
representing our coordinate system on time. We derive a new ACS from 
"coordinated_universal_time_in_seconds" - see time_std.lean 
for a more details on the coordinate system and physical interpretation
(note : this is a conventional UTC ACS expressed with units in seconds)


This example does not require any specialized timespaces beyond a simple
world time space.


(1) ORIGIN: We move the origin up to 1629311979

(2) BASIS VECTORS
    basis0 
      - points to the future
      - unit length is 1 second (as in UTC)
(3) ACS is given by [Origin, b0]
-/

def August18thTwoFortyPMTimestamp : scalar := 1629311979

def current_time_in_UTC : time_space _ := 
  let origin := mk_time coordinated_universal_time_in_seconds 1629311979 in
  let basis := mk_duration coordinated_universal_time_in_seconds 1 in
  mk_time_space (mk_time_frame origin basis)

/-
We need to assume a physical interpretation of the data
representing our coordinate system on geom3d. See geom3d_std.lean
for more details on the coordinate system and physical interpretation.
-/
def geometry3d_acs : geom3d_space _ := rice420_acs

/-
We define a base_link frame to indicate the position of an arbitrary robot
in terms of the "world" frame - geometry3d_acs. 
We derive from the Rice Hall 420 standard,
aliased in this file as "geometry3d_acs".

We define it to be 2 meters to the right and two meters forward from the
back-left wall.

-/
def base_link_acs : geom3d_space _ := 
 let origin := mk_position3d geometry3d_acs 2 2 0 in
 let basis0 := mk_displacement3d geometry3d_acs 1 0 0 in
 let basis1 := mk_displacement3d geometry3d_acs 0 1 0 in
 let basis2 := mk_displacement3d geometry3d_acs 0 0 1 in
 let fr := mk_geom3d_frame origin basis0 basis1 basis2 in
  mk_geom3d_space fr

/-
We define a UTM coordinate space. UTM (Universal Transverse Mercator)
is a 2-D coordinate system on the globe that separates the globe into 
slices separated into longitudinal strips. 

Andrew - update these coordinates with more accurate representations.
-/

def UTM_acs : geom3d_space _  := 
 let origin := mk_position3d geometry3d_acs 3 2 1 in
 let basis0 := mk_displacement3d geometry3d_acs 3 2 1 in
 let basis1 := mk_displacement3d geometry3d_acs 4 4 4 in
 let basis2 := mk_displacement3d geometry3d_acs 4 5 5 in
 let fr := mk_geom3d_frame origin basis0 basis1 basis2 in
  mk_geom3d_space fr

/-
We define a base_link_yaw_only frame. This is intended to represent
the base link frame, however, projected onto a 2-D coordinate sytem
(where there is only a yaw).

-/
def base_link_yaw_only_acs : geom3d_space _ := 
 let origin := mk_position3d geometry3d_acs 2 2 0 in
 let basis0 := mk_displacement3d geometry3d_acs 0.54 (-0.84) 0 in
 let basis1 := mk_displacement3d geometry3d_acs (0.84) (0.54) 0 in
 let basis2 := mk_displacement3d geometry3d_acs 0 0 1 in
 let fr := mk_geom3d_frame origin basis0 basis1 basis2 in
  mk_geom3d_space fr


/-

    //! @brief Holds the cartesian (UTM or local ENU) pose that is used to compute the transform
    //!
    tf2::Transform transform_cartesian_pose_;
-/

/-
-/
axiom transform_cartesian_pose_ : pose3d geometry3d_acs

/-

    //! @brief Latest IMU orientation
    //!
    tf2::Transform transform_world_pose_;

-/

axiom transform_world_pose_ : geom3d_transform geometry3d_acs UTM_acs


/-


    //! @brief Parameter that specifies the magnetic declination for the robot's environment.
    //!
    double magnetic_declination_;

    //! @brief UTM's meridian convergence
    //!
    //! Angle between projected meridian (True North) and UTM's grid Y-axis.
    //! For UTM projection (Ellipsoidal Transverse Mercator) it is zero on the equator and non-zero everywhere else.
    //! It increases as the poles are approached or as we're getting farther from central meridian.
    //!
    double utm_meridian_convergence_;

    //! @brief IMU's yaw offset
    //!
    //! Your IMU should read 0 when facing *magnetic* north. If it doesn't, this (parameterized) value gives the offset
    //! (NOTE: if you have a magenetic declination, use the parameter setting for that).
    //!
    double yaw_offset_;
-/

axiom magnetic_declination_ : scalar 
axiom utm_meridian_convergence_ : scalar 
axiom yaw_offset_ : scalar

/-
  void NavSatTransform::computeTransform()
  {
-/


def getRobotOriginCartesianPose 
  : geom3d_transform geometry3d_acs UTM_acs → pose3d geometry3d_acs → time current_time_in_UTC → punit
  := 
  λ gps_cartesian_pose, λ robot_cartesian_pose, λ transform_time, punit.star


def getRPY : scalar → scalar → scalar → punit := 
  λ s1 s2 s3, punit.star

/-

  void NavSatTransform::getRobotOriginCartesianPose(const tf2::Transform &gps_cartesian_pose,
                                                    tf2::Transform &robot_cartesian_pose,
                                                    const ros::Time &transform_time)
  {
    robot_cartesian_pose.setIdentity();

    // Get linear offset from origin for the GPS
    tf2::Transform offset;
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 ros::Duration(transform_timeout_),
                                                                 offset);

    if (can_transform)
    {
      // Get the orientation we'll use for our Cartesian->world transform
      tf2::Quaternion cartesian_orientation = transform_orientation_;
      tf2::Matrix3x3 mat(cartesian_orientation);

      // Add the offsets
      double roll;
      double pitch;
      double yaw;
      mat.getRPY(roll, pitch, yaw);
      yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);
      cartesian_orientation.setRPY(roll, pitch, yaw);

      // Rotate the GPS linear offset by the orientation
      // Zero out the orientation, because the GPS orientation is meaningless, and if it's non-zero, it will make the
      // the computation of robot_cartesian_pose erroneous.
      offset.setOrigin(tf2::quatRotate(cartesian_orientation, offset.getOrigin()));
      offset.setRotation(tf2::Quaternion::getIdentity());

      // Update the initial pose
      robot_cartesian_pose = offset.inverse() * gps_cartesian_pose;
    }
    else
    {
      if (gps_frame_id_ != "")
      {
        ROS_WARN_STREAM_ONCE("Unable to obtain " << base_link_frame_id_ << "->" << gps_frame_id_ <<
          " transform. Will assume navsat device is mounted at robot's origin");
      }

      robot_cartesian_pose = gps_cartesian_pose;
    }
  }
-/

axiom imu_roll : scalar
axiom imu_pitch : scalar
axiom imu_yaw : scalar

def compute_transform : punit := 
  --let transform_world_pose_ : geom3d_transform_expr World BaseLink := ((|World.value.mk_geom3d_transform_to BaseLink.value|:geom3d_transform_expr World BaseLink)) in
  --let cartesian_world_transform_ : geom3d_transform_expr World UTM := ((|World.value.mk_geom3d_transform_to UTM.value|:geom3d_transform_expr World UTM)) in
  --let cartesian_pose_with_orientation : geom3d_transform_expr UTM BaseLinkYawOnly := ((|UTM.value.mk_geom3d_transform_to BaseLinkYawOnly.value|:geom3d_transform_expr UTM BaseLinkYawOnly)) in
  --let cartesian_world_transform_0 : geom3d_transform_expr World UTM := ((transform_world_pose_ : geom3d_transform_expr World BaseLink)).value∘((((cartesian_pose_with_orientation : geom3d_transform_expr UTM BaseLinkYawOnly))⁻¹:geom3d_transform_expr BaseLinkYawOnly UTM)).value in
  let transform_cartesian_pose_corrected : pose3d UTM_acs := inhabited.default _ in
  let getRobotOriginCartesianPoseCall : punit 
    := getRobotOriginCartesianPose transform_cartesian_pose_ transform_cartesian_pose_corrected (mk_time current_time_in_UTC 0) in

  let mat : orientation3d geometry3d_acs := inhabited.default _ in 

  let imu_roll : scalar := imu_roll in
  let imu_pitch : scalar := imu_pitch in 
  let imu_yaw : scalar := imu_yaw in 

  let getRPYCall : punit := getRPY imu_roll imu_pitch imu_yaw in 
  let imu_yaw0 := imu_yaw + magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_ in 

  let imu_quat : orientation3d base_link_acs := inhabited.default _ in 


/-

      cartesian_pose_with_orientation.setOrigin(transform_cartesian_pose_corrected.getOrigin());
      cartesian_pose_with_orientation.setRotation(imu_quat);

      We have no way to formalize these two lines
-/
  let cartesian_pose_with_orientation : geom3d_transform geometry3d_acs base_link_acs := 
    (geometry3d_acs.mk_geom3d_transform_to base_link_acs) in 

  let cartestian_world_transform_ := transform_world_pose_∘cartesian_pose_with_orientation in 


  punit.star
/-
  void NavSatTransform::computeTransform()
  {
    if (!transform_good_ &&
        has_transform_odom_ &&
        has_transform_gps_ &&
        has_transform_imu_)
    {
      tf2::Transform transform_cartesian_pose_corrected;
      if (!use_manual_datum_)
      {
        getRobotOriginCartesianPose(transform_cartesian_pose_, transform_cartesian_pose_corrected, ros::Time(0));
      }
      else
      {
        transform_cartesian_pose_corrected = transform_cartesian_pose_;
      }

      tf2::Matrix3x3 mat(transform_orientation_);

      double imu_roll;
      double imu_pitch;
      double imu_yaw;
      mat.getRPY(imu_roll, imu_pitch, imu_yaw);

      imu_yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);

      tf2::Quaternion imu_quat;
      imu_quat.setRPY(0.0, 0.0, imu_yaw);

      tf2::Transform cartesian_pose_with_orientation;
      cartesian_pose_with_orientation.setOrigin(transform_cartesian_pose_corrected.getOrigin());
      cartesian_pose_with_orientation.setRotation(imu_quat);

      cartesian_world_transform_.mult(transform_world_pose_, cartesian_pose_with_orientation.inverse());

      cartesian_world_transform_world_pose_trans_inverse_ = cartesian_world_transform_.inverse();

      transform_good_ = true;

      if (broadcast_cartesian_transform_)
      {
        geometry_msgs::TransformStamped cartesian_transform_stamped;
        cartesian_transform_stamped.header.stamp = ros::Time::now();
        std::string cartesian_frame_id = (use_local_cartesian_ ? "local_enu" : "utm");
        cartesian_transform_stamped.header.frame_id = (broadcast_cartesian_transform_as_parent_frame_ ?
                                                       cartesian_frame_id : world_frame_id_);
        cartesian_transform_stamped.child_frame_id = (broadcast_cartesian_transform_as_parent_frame_ ?
                                                      world_frame_id_ : cartesian_frame_id);
        cartesian_transform_stamped.transform = (broadcast_cartesian_transform_as_parent_frame_ ?
                                             tf2::toMsg(cartesian_world_trans_inverse_) :
                                             tf2::toMsg(cartesian_world_transform_));
        cartesian_transform_stamped.transform.translation.z = (zero_altitude_ ?
                                                           0.0 : cartesian_transform_stamped.transform.translation.z);
        cartesian_broadcaster_.sendTransform(cartesian_transform_stamped);
      }
    }
  }-/