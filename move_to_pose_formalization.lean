import .phys.time.time
import .phys.time_series.geom3d
import .std.time_std
import .std.geom3d_std
import .phys.time_series.geom3d
import data.real.basic
noncomputable theory

open classical
local attribute [instance] prop_decidable


-- TODO: Should come from resp. std libraries and be distributed to them accordingly
namespace std
def time (p : K) : time time_std_space := mk_time time_std_space p
def duration (d : K) : duration time_std_space := mk_duration _ d
def position (x y z : K) : position3d geom3d_std_space := mk_position3d _ x y z
def displacement (x y z : K) : displacement3d geom3d_std_space := mk_displacement3d _ x y z
end std

/-
The issue in question refers to the issue as occurring specifically
when a "target pose" (which has some initial frame and is transformed to a planning frame), 
is transformed from the "world" frame to the "base link" frame.

Thus, we are interpreting one of our standard spaces - rice440 - as the coordinate ACS
for our "world" space
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
def position (x y z : K) := mk_position3d coords x y z
def displacement (x y z : K) := mk_displacement3d coords x y z
end base_link  

-- https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/#Tracking_Sensor_Origin_and_CS


/-
We need to assume a physical interpretation of the data
representing our coordinate system on time. We derive a new ACS from 
"coordinated_universal_time_in_seconds" - see time_std.lean 
for a more details on the coordinate system and physical interpretation
(note : this is a conventional UTC ACS expressed with units in seconds)

The gist of this interpretation of our "world time" is that we have formalized
some fixed time at which our application is run, expressed in terms of UTC. 
To do so, we have simply translated the UTC origin to the time at which this was written,
August 18th at ~240 PM. Note: any arbitrary time does not hold any impact on our formal model
(although presumably if we set the origin to the stone age, 
  there should not be a robotics camera running there).
We do not change the basis - which is a standard 
basis whose unit vectors are expressed in seconds.


(1) ORIGIN: We move the origin up to 1629311979

(2) BASIS VECTORS
    basis0 
      - points to the future
      - unit length is 1 second (as in UTC)
      - Presumably no dilation occur, since this is supposed to represent "empirical"
        (atomically-sampled-and-average-weighted) time
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

  Eigen::Isometry3d tf_moveit_to_robot_cmd_frame_;

  Helper class used in relevant call to formalize error - in particular, contains a member function of interest, "getCommandFrameTransform" that is called
-/
structure ServoCalcs := 
  (tf_moveit_to_robot_cmd_frame_ : timestamped utc.coords (pose3d world.coords))

/-
Formalized Eigen::Isometry3d::isZero
-/
def timestamped.isZero : timestamped utc.coords (pose3d world.coords) → bool := 
  λ tp,
  tp.value.orientation.to_orientation.to_vectr_basis.basis_vectrs 0 ≠ 0 ∧ 
  tp.value.orientation.to_orientation.to_vectr_basis.basis_vectrs 1 ≠ 0 ∧ 
  tp.value.orientation.to_orientation.to_vectr_basis.basis_vectrs 2 ≠ 0
/-
bool ServoCalcs::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  const std::lock_guard<std::mutex> lock(input_mutex_);
  transform = tf_moveit_to_robot_cmd_frame_;

  // All zeros means the transform wasn't initialized, so return false
  return !transform.matrix().isZero(0);
}

  This call unfortunately represents an in-place assignment, so we lose some of the semantics,
  although, in terms of type checking, it's not a serious problem.
-/
def ServoCalcs.getCommandFrameTransform (sc : ServoCalcs) : timestamped utc.coords (pose3d world.coords) → bool := 
  λtransform,
  let transform0 := sc.tf_moveit_to_robot_cmd_frame_ in 
  ¬(timestamped.isZero transform0)

/-
  Helper class used in relevant call to formalize error - in particular, contains a member function of interest, "getCommandFrameTransform" that is called
-/
structure Servo := 
  (servo_calcs : ServoCalcs)

/-
bool Servo::getCommandFrameTransform(Eigen::Isometry3d& transform)
{
  return servo_calcs_->getCommandFrameTransform(transform);
}

Contains a call to ServoCalcs.getCommandFrameTransform, which handles an assignment to the transform variable
-/

def Servo.getCommandFrameTransform (s : Servo) : timestamped utc.coords (pose3d world.coords) → bool := 
  λtransform,
  s.servo_calcs.getCommandFrameTransform transform

/-

  // moveit_servo::Servo instance. Public so we can access member functions like setPaused()
  std::unique_ptr<moveit_servo::Servo> servo_;
  
  // Transforms w.r.t. planning_frame_
  Eigen::Isometry3d command_frame_transform_;

  geometry_msgs::PoseStamped target_pose_;
  
  boost::optional<double> angular_error_;

  // Flag that a different thread has requested a stop.
  std::atomic<bool> stop_requested_;

  Definition of PoseTracking object, whose member functions "targetPoseCallback" in conjunction with "moveToPose" produce the error
  described in the issue.
-/
structure PoseTracking := 
  (servo_ : Servo)
  (target_pose_ : timestamped utc.coords (pose3d base_link.coords))
  (command_frame_transform_ : timestamped utc.coords (pose3d world.coords))
  (command_frame_transform_stamp_ : time utc.coords)
  (angular_error_ : scalar)
  (stop_request_ : bool)

/-
bool PoseTracking::satisfiesPoseTolerance(const Eigen::Vector3d& positional_tolerance, const double angular_tolerance)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  double x_error = target_pose_.pose.position.x - command_frame_transform_.translation()(0);
  double y_error = target_pose_.pose.position.y - command_frame_transform_.translation()(1);
  double z_error = target_pose_.pose.position.z - command_frame_transform_.translation()(2);

  return ((std::abs(x_error) < positional_tolerance(0)) && (std::abs(y_error) < positional_tolerance(1)) &&
          (std::abs(z_error) < positional_tolerance(2)) && (std::abs(angular_error_) < angular_tolerance));
}

Helper method - referenced in pertinent function "moveToPose" - checks if the "target_pose" is within some threshold
-/

def PoseTracking.satisfiesPoseTolerance (pt : PoseTracking) : displacement3d world.coords → scalar → bool :=
  λpositional_tolerance,
  λangular_tolerance, 
  let x_error := pt.target_pose_.value.position.x - pt.command_frame_transform_.value.position.x in
  let y_error := pt.target_pose_.value.position.y - pt.command_frame_transform_.value.position.y in
  let z_error := pt.target_pose_.value.position.z - pt.command_frame_transform_.value.position.z in
  (abs x_error) < positional_tolerance.x ∧ 
  (abs y_error) < positional_tolerance.y ∧ 
  (abs z_error) < positional_tolerance.z ∧ 
  pt.angular_error_ < angular_tolerance



/-
void PoseTracking::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  target_pose_ = *msg;

  // If the target pose is not defined in planning frame, transform the target pose.
  if (target_pose_.header.frame_id != planning_frame_)
  {
    try
    {
      geometry_msgs::TransformStamped target_to_planning_frame = transform_buffer_.lookupTransform(
          planning_frame_, target_pose_.header.frame_id, ros::Time(0), ros::Duration(0.1));
      tf2::doTransform(target_pose_, target_pose_, target_to_planning_frame);
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_STREAM_NAMED(LOGNAME, ex.what());
      return;
    }
  }
}
-/

/-
Define a time series of transforms from world to base link 
This time series is, in this case, analogous to TF's internal list of transforms that are maintained
In this case, we've populated this time series with one, single entry - which is intended to have
derived from a URDF file, and thus, it is static and has a timestamp of "0" 
-/
def world_to_base_link : geom3d_transform_discrete utc.coords world.coords base_link.coords := [
		⟨(mk_time utc.coords 0.000000),(world.coords.mk_geom3d_transform_to base_link.coords)⟩]

/-
In this method, we see that the target_pose property of PoseTracking will effectively always have a timestamp of 0, thus, calling "haveRecentTargetPose",
defined below, will fail under all likely values of the timespan argument.

Unfortunately, in this current formalization, we do not have a way of manifesting a type error or constraint failure. As witnessed below, the semantics
of the in-place assignment fail, so calling targetPoseCallback would not actually effect any state change on the PoseTracking object.
-/
def PoseTracking.targetPoseCallback (pt : PoseTracking) : pose3d world.coords → punit := 
  λ msg,
  let target_to_planning_frame : timestamped utc.coords (geom3d_transform world.coords base_link.coords) := world_to_base_link.latest in
  let pt0 : PoseTracking := {
    target_pose_ := ⟨target_to_planning_frame.timestamp, target_to_planning_frame.value.transform_pose3d msg⟩,
    ..pt
  } in 
  punit.star


/-
bool PoseTracking::haveRecentTargetPose(const double timespan)
{
  std::lock_guard<std::mutex> lock(target_pose_mtx_);
  return ((ros::Time::now() - target_pose_.header.stamp).toSec() < timespan);
}

Here, a constraint may fail, in that under certain conditions where we would expect that this method
should return "true", it will continuously return false and change control flow unexpectedly
-/

def PoseTracking.haveRecentTargetPose (pt : PoseTracking) : scalar → bool := 
  λtimespan, 
  (((mk_time utc.coords 10) -ᵥ pt.target_pose_.timestamp) : duration utc.coords).coord < timespan


/-
bool PoseTracking::haveRecentEndEffectorPose(const double timespan)
{
  return ((ros::Time::now() - command_frame_transform_stamp_).toSec() < timespan);
}

Helper method used in moveToPose - not relevant for issue
-/

def PoseTracking.haveRecentEndEffectorPose (pt : PoseTracking) : scalar → bool := 
  λtimespan, 
  (((mk_time utc.coords 10) -ᵥ pt.command_frame_transform_.timestamp) : duration utc.coords).coord < timespan



/-
bool PoseTracking::getCommandFrameTransform(geometry_msgs::TransformStamped& transform)
{
  return servo_->getCommandFrameTransform(transform);
}

Helper method used in moveToPose - not relevant for issue
-/
def PoseTracking.getCommandFrameTransform (pt : PoseTracking) : timestamped utc.coords (pose3d world.coords) → bool := 
  λtransform, 
  pt.servo_.getCommandFrameTransform transform

/-
void PoseTracking::doPostMotionReset()
{
  stop_requested_ = false;
  angular_error_ = 0;

  // Reset error integrals and previous errors of PID controllers
  cartesian_position_pids_[0].reset();
  cartesian_position_pids_[1].reset();
  cartesian_position_pids_[2].reset();
  cartesian_orientation_pids_[0].reset();
}

Helper method used in moveToPose - not relevant for issue
-/

def PoseTracking.doPostMotionReset (pt : PoseTracking) : punit := 
  let pt0 := {
    angular_error_ := 0, 
    ..pt
  } in 
  punit.star

def duration.sleep (d : duration utc.coords) : punit := punit.star 

/-
The primary method in which erroneous state is retrieved and where a failure manifests, in that 
the servo arm will not move to the indicated "target_pose_", as said pose will always be perceived by the
system to have a timestamp of "0" - implying that it is not recent and should be ignored. More specifically, 
the call to haveRecentTargetPose will continously fail, and the while loop will not terminate, hence, the 
robot will spin wait rather than move.
-/
def PoseTracking.moveToPose (pt : PoseTracking) : displacement3d world.coords → scalar → scalar → punit :=
  λpositional_tolerance,
  λangular_tolerance, 
  λtarget_pose_timeout,

  let start_time : time utc.coords := mk_time _ 5 in

  /-
  while ((!haveRecentTargetPose(target_pose_timeout) || !haveRecentEndEffectorPose(target_pose_timeout)) &&
         ((ros::Time::now() - start_time).toSec() < target_pose_timeout))
  {
    if (servo_->getCommandFrameTransform(command_frame_transform_))
    {
      command_frame_transform_stamp_ = ros::Time::now();
    }
    ros::Duration(0.001).sleep();
  }
  -/
  let while0 := 
    if 
      ¬pt.haveRecentTargetPose target_pose_timeout ∨ 
      ¬pt.haveRecentEndEffectorPose target_pose_timeout ∨ 
      (((mk_time utc.coords 10) -ᵥ start_time : duration _).coord < target_pose_timeout) 
    then 
      let if0 := 
        if pt.servo_.getCommandFrameTransform pt.command_frame_transform_ then 
          let pt0 : PoseTracking := {
            command_frame_transform_stamp_ := mk_time _ 10,
            ..pt
          } in 
          punit.star
        else punit.star in
        let sleepCall : punit := (mk_duration utc.coords 5).sleep in
        punit.star
    else 
      punit.star in
    /-
    if (!haveRecentTargetPose(target_pose_timeout))
    {
      return PoseTrackingStatusCode::NO_RECENT_TARGET_POSE;
    }
    -/
    let if0 := 
      if ¬pt.haveRecentTargetPose target_pose_timeout then 
        punit.star 
      else 
        punit.star in 
    /-
    while (ros::ok() && !satisfiesPoseTolerance(positional_tolerance, angular_tolerance))
    {
      if (servo_->getCommandFrameTransform(command_frame_transform_))
      {
        command_frame_transform_stamp_ = ros::Time::now();
      }
      if (!haveRecentEndEffectorPose(target_pose_timeout))
      {
        doPostMotionReset();
        return PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE;
      }

      if (stop_requested_)
      {
        doPostMotionReset();
        return PoseTrackingStatusCode::STOP_REQUESTED;
      }
      twist_stamped_pub_.publish(calculateTwistCommand());
    }
    -/
    let while0 := 
      if true ∧ pt.satisfiesPoseTolerance positional_tolerance angular_tolerance then 
        let if0 := 
          if pt.servo_.getCommandFrameTransform pt.command_frame_transform_ then 
            let pt0 : PoseTracking := {
              command_frame_transform_stamp_ := mk_time _ 10,
              ..pt
            } in 
            punit.star
          else 
            punit.star in 
        let if0 := 
          if ¬pt.haveRecentEndEffectorPose target_pose_timeout then 
            let doPostMotionResetCall := pt.doPostMotionReset in
            punit.star
          else 
            punit.star in 
        let if0 := 
          if ¬pt.stop_request_ then 
            let doPostMotionResetCall := pt.doPostMotionReset in
            punit.star
          else 
            punit.star in 
          punit.star
      else 
        punit.star in 
    let doPostMotionResetCall := pt.doPostMotionReset in
    punit.star
/-
PoseTrackingStatusCode PoseTracking::moveToPose(const Eigen::Vector3d& positional_tolerance,
                                                const double angular_tolerance, const double target_pose_timeout)
{
  while ((!haveRecentTargetPose(target_pose_timeout) || !haveRecentEndEffectorPose(target_pose_timeout)) &&
         ((ros::Time::now() - start_time).toSec() < target_pose_timeout))
  {
    if (servo_->getCommandFrameTransform(command_frame_transform_))
    {
      command_frame_transform_stamp_ = ros::Time::now();
    }
    ros::Duration(0.001).sleep();
  }
  if (!haveRecentTargetPose(target_pose_timeout))
  {
    return PoseTrackingStatusCode::NO_RECENT_TARGET_POSE;
  }
  while (ros::ok() && !satisfiesPoseTolerance(positional_tolerance, angular_tolerance))
  {
    if (servo_->getCommandFrameTransform(command_frame_transform_))
    {
      command_frame_transform_stamp_ = ros::Time::now();
    }
    if (!haveRecentEndEffectorPose(target_pose_timeout))
    {
      doPostMotionReset();
      return PoseTrackingStatusCode::NO_RECENT_END_EFFECTOR_POSE;
    }

    if (stop_requested_)
    {
      doPostMotionReset();
      return PoseTrackingStatusCode::STOP_REQUESTED;
    }
    twist_stamped_pub_.publish(calculateTwistCommand());
  }
  doPostMotionReset();
  return PoseTrackingStatusCode::SUCCESS;
}
-/
