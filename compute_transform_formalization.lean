import .phys.time.time
import .phys.time_series.geom3d
import .std.time_std
import .std.geom3d_std
noncomputable theory

-- TODO: Should come from resp. std libraries and be distributed to them accordingly
namespace std
def time (p : K) : time time_std_space := mk_time time_std_space p
def duration (d : K) : duration time_std_space := mk_duration _ d
def position (x y z : K) : position3d geom3d_std_space := mk_position3d _ x y z
def displacement (x y z : K) : displacement3d geom3d_std_space := mk_displacement3d _ x y z
end std

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

namespace utc
def origin := std.time 0   -- origin; first instant of January 1, 1970
def basis := std.duration 1    -- basis; "second;" the smallest non-variable unit in UTC
def frame := mk_time_frame origin basis -- recall why "time" is part of the constructor name? factor out?
def coords := mk_space frame  -- "cosys"?
def time (t : K) := mk_time coords t
def duration (d : K) := mk_duration coords d
end utc

/-
We need to assume a physical interpretation of the data
representing our coordinate system on geom3d. See geom3d_std.lean
for more details on the coordinate system and physical interpretation.
-/
-- Geometric world
namespace world  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := std.position 0 0 0      -- looking in from doorway, the back lower left corner  
def basis_0 := std.displacement 1 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := std.displacement 0 1 0 -- to door along weset wall; 1m; right
def basis_2 := std.displacement 0 0 1 -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : K) := mk_position3d coords x y z
def displacement (x y z : K) := mk_displacement3d coords x y z
end world


/-
We define a base link ACS, which is intended to represent the ACS
centered at the robot, in terms of the world frame, and oriented at the robot,
in terms of the world frame.

We define the robot to be 3 meters to the right of the left wall, 4 meters
in north of the bottom wall, and 1 meter above the ground. It's rotated 1 radian
around it's yaw axis relative to the world's orientation.
-/

namespace base_link   
def origin := std.position 3 4 1       
def basis_0 := std.displacement 1 0 0
def basis_1 := std.displacement 0 1 0
def basis_2 := std.displacement 0 0 1
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end base_link 

/-
We define a UTM coordinate space. UTM (Universal Transverse Mercator)
is a 2-D coordinate system on the globe that separates the globe into 
slices separated into longitudinal strips. 

Andrew - update these coordinates with more accurate representations.
-/

namespace UTM  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := world.position 3 2 0      -- looking in from doorway, the back lower left corner  
def basis_0 := world.displacement 1 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := world.displacement 0 1 0 -- to door along weset wall; 1m; right
def basis_2 := world.displacement 0 0 0 -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : K) := mk_position3d coords x y z
def displacement (x y z : K) := mk_displacement3d coords x y z
end UTM

/-
We define a base_link_yaw_only frame. This is intended to represent
the base link frame, however, projected onto a 2-D coordinate sytem
(where there is only a yaw rotation).

Andrew - update these coordinates with more accurate representations.

This should, perhaps, derive from the UTC coordinates.
-/
namespace base_link_yaw_only  
def origin := world.position 2 2 0       
def basis_0 := world.displacement 0.54 (-0.84) 0 
def basis_1 := world.displacement (0.84) (0.54) 0
def basis_2 := world.displacement 0 0 0
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end base_link_yaw_only 


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
 (transform_orientation_ : orientation3d world.coords)
 (transform_world_pose_  : geom3d_transform world.coords base_link.coords)
 (transform_cartesian_pose_ : geom3d_transform world.coords UTM.coords)

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
  : geom3d_transform world.coords UTM.coords → pose3d world.coords → time utc.coords → punit
  := 
    λ gps_cartesian_pose, λ robot_cartesian_pose, λ transform_time, 
    --robot_cartesian_pose.setIdentity();
    let setIdentityCall := robot_cartesian_pose.setIdentity in 
    --tf2::Quaternion cartesian_orientation = transform_orientation_;
    let cartesian_orientation : orientation3d world.coords := nst.transform_orientation_ in 
    --tf2::Matrix3x3 mat(cartesian_orientation);
    let mat : orientation3d world.coords := cartesian_orientation in 

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
    let cartesian_orientation0 := mk_orientation3d_from_euler_angles world.coords roll pitch yaw in

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
  let transform_cartesian_pose_corrected : pose3d world.coords := inhabited.default _ in
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
    := nst.getRobotOriginCartesianPose nst.transform_cartesian_pose_ transform_cartesian_pose_corrected (mk_time utc.coords 0) in
  /-
      tf2::Matrix3x3 mat(transform_orientation_);
  -/
  let mat : orientation3d world.coords := inhabited.default _ in 

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
  let imu_quat : orientation3d base_link.coords := inhabited.default _ in 
  let imu_quat0 : orientation3d base_link.coords := mk_orientation3d_from_euler_angles _ imu_roll imu_pitch imu_yaw in

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
    (UTM.coords.mk_geom3d_transform_to base_link_yaw_only.coords) in 

  /-
  Here is where the error occurs: We attempt to compose a transform from World → Base Link, with the inverse of a transform from 
  UTM → Base Link Yaw Only, by taking T₂∘(T₁⁻¹). However, the domain of T₁⁻¹ does not match the codomain of T₂,
  yielding a type error in Lean that was not captured in the original code.
  -/
  let cartestian_world_transform_ := (nst.transform_world_pose_.trans (cartesian_pose_with_orientation.symm)) in


  punit.star
