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
(where there is only a yaw rotation).

Andrew - update these coordinates with more accurate representations.
-/
def base_link_yaw_only_acs : geom3d_space _ := 
 let origin := mk_position3d geometry3d_acs 2 2 0 in
 let basis0 := mk_displacement3d geometry3d_acs 0.54 (-0.84) 0 in
 let basis1 := mk_displacement3d geometry3d_acs (0.84) (0.54) 0 in
 let basis2 := mk_displacement3d geometry3d_acs 0 0 1 in
 let fr := mk_geom3d_frame origin basis0 basis1 basis2 in
  mk_geom3d_space fr



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

    //! @brief Latest IMU orientation
    //!
    tf2::Quaternion transform_orientation_;

    //! @brief Latest IMU orientation
    //!
    tf2::Transform transform_world_pose_;

    //! @brief Holds the Cartesian->odom transform
    //!
    tf2::Transform cartesian_world_transform_;

    //! @brief Holds the cartesian (UTM or local ENU) pose that is used to compute the transform
    //!
    tf2::Transform transform_cartesian_pose_;

-/

/-
This is a transpiled version of the class containing the critical code, named "NavSatTransform". It contains (only)
the relevant class properties that are referenced in the methods relevant to where the isolated bug is.

-/

structure NavSatTransform := 
 (magnetic_declination_ : scalar)
 (utm_meridian_convergence_ : scalar)
 (yaw_offset_ : scalar)
 (transform_orientation_ : orientation3d geometry3d_acs)
 (transform_world_pose_  : geom3d_transform geometry3d_acs base_link_acs)
 (transform_cartesian_pose_ : geom3d_transform geometry3d_acs UTM_acs)

/-
    void NavSatTransform::getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                                tf2::Transform &robot_odom_pose,
                                                const ros::Time &transform_time)
  {
    robot_odom_pose.setIdentity();
    tf2::Transform gps_offset_rotated;
    bool can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                                 base_link_frame_id_,
                                                                 gps_frame_id_,
                                                                 transform_time,
                                                                 transform_timeout_,
                                                                 gps_offset_rotated);

    if (can_transform)
    {
      tf2::Transform robot_orientation;
      can_transform = RosFilterUtilities::lookupTransformSafe(tf_buffer_,
                                                              world_frame_id_,
                                                              base_link_frame_id_,
                                                              transform_time,
                                                              transform_timeout_,
                                                              robot_orientation);
      if (can_transform)
      {
        gps_offset_rotated.setOrigin(tf2::quatRotate(robot_orientation.getRotation(), gps_offset_rotated.getOrigin()));
        gps_offset_rotated.setRotation(tf2::Quaternion::getIdentity());
        robot_odom_pose = gps_offset_rotated.inverse() * gps_odom_pose;
      }
      else
      {
      }
    }
    else
    {
    }
  }

-/

/-
Transpilation of "tf::Transform::setIdentity"
Notice that this method is "bound" to pose3d in Lean, so there is an interesting subtlety in how these would need to be 
printed off (possibly with duplicates).

The other aspect to notice is that the purpose of this method, in C++, is an in-place assignment. We lose a particular aspect of the 
original semantics here. We can see a re-assignment of the argument, p, below. However, the original semantics require an in-place assignment.
Thus, the caller expects that the member argument, p, has been re-assigned, but, as it is modeled, 
the member argument is assigned *into* p0, and the enclosing scope does not have access to the updated value of p. 
This is a broader issue and does not have a simple solution, such as forcibly convering this method and all call sites as assignments.
-/
def pose3d.setIdentity {f : geom3d_frame} {sp : geom3d_space f} (p : pose3d sp) : punit := 
  let p0 : pose3d sp := {
    position := inhabited.default _, 
    orientation := inhabited.default _
  } in
  punit.star


/-

Just as above, transpilation of "getRPY" call from matrices or quaternions in ROS. 
The intended semantics of this method should perform an in-place assignment of the arguments, 
(roll pitch and yaw angles). Unfortunately, we face the same challenge as above. 

(also, I haven't implemented the Lean orientation call to retrieve these values...will do so soon)
-/
def orientation3d.getRPY {f : geom3d_frame} {sp : geom3d_space f } 
  (o : orientation3d sp) 
  : scalar → scalar → scalar → punit := 
  λ s1 s2 s3,
  punit.star 

/-

    void NavSatTransform::getRobotOriginWorldPose(const tf2::Transform &gps_odom_pose,
                                                tf2::Transform &robot_odom_pose,
                                                const ros::Time &transform_time)

This is the formalization of a helper function reference in the function "computeTransform",
formalized below, which describes the error of interest to the issue. There are no physical
type errors in this method, although formalizing raised some interesting issues. 

It is defined as a "Lean-esque member function" of NavSatTransform.

For example, Poses and Transforms share the same type in C++, whereas they are modeled differently 
-/
def NavSatTransform.getRobotOriginCartesianPose (nst : NavSatTransform)
  : geom3d_transform geometry3d_acs UTM_acs → pose3d geometry3d_acs → time current_time_in_UTC → punit
  := 
    λ gps_cartesian_pose, λ robot_cartesian_pose, λ transform_time, 
    --robot_cartesian_pose.setIdentity();
    let setIdentityCall := robot_cartesian_pose.setIdentity in 
    --tf2::Quaternion cartesian_orientation = transform_orientation_;
    let cartesian_orientation : orientation3d geometry3d_acs := nst.transform_orientation_ in 
    --tf2::Matrix3x3 mat(cartesian_orientation);
    let mat : orientation3d geometry3d_acs := cartesian_orientation in 

    /-
    double roll;
    double pitch;
    double yaw;
    -/
    let roll : scalar := inhabited.default _ in 
    let pitch : scalar := inhabited.default _ in 
    let yaw : scalar := inhabited.default _ in 

    --mat.getRPY(roll, pitch, yaw);
    let getRPYCall := mat.getRPY roll pitch yaw in 
    let yaw0 := yaw + nst.magnetic_declination_ + nst.yaw_offset_ + nst.utm_meridian_convergence_ in 
    --yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);
    /-
    cartesian_orientation.setRPY(roll, pitch, yaw);
    A note here: A more "literal" translation of the above line would involve a call to "setRPY",
    which, again, is an in-place assignment to cartesian_orientation. Fortunately, if we don't
    attempt that route, we can directly assign to the variable (SSA-style) using "mk_orientation3d_from_euler_angles",
    which carries the same semantics as the original C++.
    -/
    let cartesian_orientation0 := mk_orientation3d_from_euler_angles geometry3d_acs roll pitch yaw in

    /-
    offset.setOrigin(tf2::quatRotate(cartesian_orientation, offset.getOrigin()));
    offset.setRotation(tf2::Quaternion::getIdentity());
    
    robot_cartesian_pose = offset.inverse() * gps_cartesian_pose;

    We have no way of formalizing this code. "offset" is intended to be a transform - but we provide 
    no way to set the "origin" or "rotation" of a Transform, and, it's not a simple fix, it's a bit beyond the scope of 
    our formalization currently.
    -/

    punit.star

/-
void NavSatTransform::computeTransform()

This is the principal function in which the error of interest is modeled. 
It is a member function of the C++ class "NavSatTransform", and here, in Lean, it is modeled
as a "member function" of the structure NavSatTransform. 

The gist of the function and error, below, is that we will first "construct" 
  the Transform from UTM → Base Link Yaw Only, T₁, and then we will "use" the member transform_world_pose_, T₂,
  of NavSatTransform, which is a transform from World → Base Link, in order to construct a transform from 
  World → UTM, by taking T₂∘(T₁⁻¹). However, the domain of T₁⁻¹ does not match the codomain of T₂,
  yielding a type error in Lean that was not captured in the original code.
-/
def NavSatTransform.compute_transform 
  (nst : NavSatTransform)
  : punit := 
  /-
      tf2::Transform transform_cartesian_pose_corrected;
  -/
  let transform_cartesian_pose_corrected : pose3d geometry3d_acs := inhabited.default _ in
  /-
  if (!use_manual_datum_)
      {
        getRobotOriginCartesianPose(transform_cartesian_pose_, transform_cartesian_pose_corrected, ros::Time(0));
      }
      else
      {
        transform_cartesian_pose_corrected = transform_cartesian_pose_;
      }

  Notice here, we simply assume an execution path. We assume that the former branch triggers, and thus, a call
  to the previously formalized member function of NavSatTransform is called
  -/
  let getRobotOriginCartesianPoseCall : punit 
    := nst.getRobotOriginCartesianPose nst.transform_cartesian_pose_ transform_cartesian_pose_corrected (mk_time current_time_in_UTC 0) in
  /-
      tf2::Matrix3x3 mat(transform_orientation_);
  -/
  let mat : orientation3d geometry3d_acs := inhabited.default _ in 

  /-
      double imu_roll;
      double imu_pitch;
      double imu_yaw;
      mat.getRPY(imu_roll, imu_pitch, imu_yaw);
  -/
  let imu_roll : scalar := inhabited.default _ in
  let imu_pitch : scalar := inhabited.default _ in 
  let imu_yaw : scalar := inhabited.default _ in 
  /-
  This does not correctly model the semantics of in-place assignment, as described earlier.
  -/
  let getRPYCall : punit := mat.getRPY imu_roll imu_pitch imu_yaw in 

  --imu_yaw += (magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_);
  /-
  
  -/
  let imu_yaw0 := imu_yaw + nst.magnetic_declination_ + nst.yaw_offset_ + nst.utm_meridian_convergence_ in 

  /-
  tf2::Quaternion imu_quat;
  imu_quat.setRPY(0.0, 0.0, imu_yaw);

  -/
  let imu_quat : orientation3d base_link_acs := inhabited.default _ in 
  let imu_quat0 : orientation3d base_link_acs := mk_orientation3d_from_euler_angles _ imu_roll imu_pitch imu_yaw in

/-

      cartesian_pose_with_orientation.setOrigin(transform_cartesian_pose_corrected.getOrigin());
      cartesian_pose_with_orientation.setRotation(imu_quat);

      We have no way to formalize these two lines. As described earlier, in our current architecture, a 
      Transform is defined simply by providing two coordinate spaces, and we receive the *fixed* transform
      between those two spaces. In ROS, coordinate spaces *vary* over time, and thus, transforms are not necessarily 
      fixed, and we can assign to their properties in an ad hoc manner, which often represents a transform at a particular time.
      Time Series, at least as we've currently modeled them, do not provide a solution to this limitation.

      To capture the semantics, I ignore the constructed transform_cartesian_pose_corrected origin and imu_quat, and
      construct the transform from UTM → Base Link Yaw Only as we conventionally construct it.
-/
  let cartesian_pose_with_orientation : geom3d_transform _ _ := 
    (UTM_acs.mk_geom3d_transform_to base_link_yaw_only_acs) in 

  /-
  Here is where the error occurs: We attempt to compose a transform from World → Base Link, with the inverse of a transform from 
  UTM → Base Link Yaw Only, by taking T₂∘(T₁⁻¹). However, the domain of T₁⁻¹ does not match the codomain of T₂,
  yielding a type error in Lean that was not captured in the original code.
  -/
  let cartestian_world_transform_ := (nst.transform_world_pose_.trans (cartesian_pose_with_orientation.symm)) in


  punit.star
