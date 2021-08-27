import ..phys.time.time
import ..phys.time_series.geom3d
import ..std.time_std
import ..std.geom3d_std
import ..phys.time_series.geom3d
import data.real.basic
noncomputable theory


open classical
local attribute [instance] prop_decidable
-- TODO: Should come from resp. std libraries and be distributed to them accordingly
namespace std
def time (p : scalar) : time time_std_space := mk_time time_std_space p
def duration (d : scalar) : duration time_std_space := mk_duration _ d
def position (x y z : scalar) : position3d geom3d_std_space := mk_position3d _ x y z
def displacement (x y z : scalar) : displacement3d geom3d_std_space := mk_displacement3d _ x y z
end std

-- Geometric world
/-
The issue refers to a target frame, which can take on the value of "odom".
Odom is a conventional world-fixed frame in ROS, from which we might 
estimate the base link pose via various sources, such as an IMU.

Thus, we are interpreting one of our standard spaces - rice440 - as the coordinate ACS
for our "world" space
-/
namespace odom  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := std.position 0 0 0      -- looking in from doorway, the back lower left corner  
def basis_0 := std.displacement 1 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := std.displacement 0 1 0 -- to door along weset wall; 1m; right
def basis_2 := std.displacement 0 0 1 -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end odom

/-
The base link frame is documented in the issue as being the frame that imu pose readings 
are expressed in.

We define the robot to be 3 meters to the right of the left wall, 4 meters
in north of the bottom wall, and 1 meter above the ground. It's rotated 1 radian
around its yaw axis relative to the standard frame.
-/
namespace base_link 
def origin := std.position 3 4 1 
def basis_0 := std.displacement 0.54 (-0.84) 0
def basis_1 := std.displacement 0.84 0.54 0
def basis_2 := std.displacement 0 0 1
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end base_link

namespace utc
def origin := std.time 0   -- origin; first instant of January 1, 1970
def basis := std.duration 1    -- basis; "second;" the smallest non-variable unit in UTC
def frame := mk_time_frame origin basis -- recall why "time" is part of the constructor name? factor out?
def coords := mk_time_space frame  -- "cosys"?
def time (t : scalar) := mk_time coords t
def duration (d : scalar) := mk_duration coords d
end utc



/-

    //! @brief Parameter that specifies the how long we wait for a transform to become available.
    //!
    ros::Duration tfTimeout_;

    //! @brief Stores the last measurement from a given topic for differential integration
    //!
    //! To carry out differential integration, we have to (1) transform
    //! that into the target frame (probably the frame specified by
    //! @p odomFrameId_), (2) "subtract"  the previous measurement from
    //! the current measurement, and then (3) transform it again by the previous
    //! state estimate. This holds the measurements used for step (2).
    //!
    std::map<std::string, tf2::Transform> previousMeasurements_;

    Primary class containing method (::preparePose) in which error occurs.
-/

structure RosFilter := 
  (tfTimeout_ : duration utc.coords)

def orientation3d.setValue {f : geom3d_frame} {sp : geom3d_space f} (o : orientation3d sp) (x y z w : scalar) : punit := 
  let o0 : orientation3d sp := mk_orientation3d_from_quaternion _ x y z w in 
  punit.star

def pose3d.setRotation {f : geom3d_frame} {sp : geom3d_space f} (p: pose3d sp) : orientation3d sp → punit := 
  λ o, 
  let p0 : pose3d sp := {
    orientation := o,
    ..p
  } in 
  punit.star

axiom odom.coords_to_odom.coords : geom3d_transform_discrete utc.coords odom.coords odom.coords
structure tf2_rosBuffer :=
  mk ::

def geom3d_transform.setIdentity {f1 f2 : geom3d_frame} {sp1 : geom3d_space f1} {sp2 : geom3d_space f2} (tr : geom3d_transform sp1 sp2) := 
  punit.star

/-
bool lookupTransformSafe(const tf2_ros::Buffer &buffer,
                           const std::string &targetFrame,
                           const std::string &sourceFrame,
                           const ros::Time &time,
                           const ros::Duration &timeout,
                           tf2::Transform &targetFrameTrans,
                           const bool silent)
-/

--def fromMsg : 


def RosFilterUtilities.lookupTransformSafe : time utc.coords → duration utc.coords → geom3d_transform odom.coords odom.coords → bool := 
  λ time,
  λ duration,
  λ targetFrameTrans,
  /-
  bool retVal = true;
    try
    {
      tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, time, timeout).transform,
                   targetFrameTrans);
    }
    catch (tf2::TransformException &ex)
    {
      try
      {
        tf2::fromMsg(buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0)).transform,
                     targetFrameTrans);

        if (!silent)
        {
        }
      }
      catch(tf2::TransformException &ex)
      {
        if (!silent)
        {
        }

        retVal = false;
      }
    }
    -/
    let retVal : bool := true in
    let try0 : punit :=
      if tt then 
        let targetFrameTrans0 := odom.coords_to_odom.coords.sample time in 
        punit.star
      else 
        let targetFrameTrans0 := odom.coords_to_odom.coords.latest in 
        punit.star  in
    /-
    if (!retVal)
    {
      if (targetFrame == sourceFrame)
      {
        targetFrameTrans.setIdentity();
        retVal = true;
      }
    }

    return retVal;
  -/ 
    let if0 : punit := 
      if ¬retVal then 
        let if1 : punit := 
          if true then 
            let setIdentityCall := targetFrameTrans.setIdentity in 
            let retVal0 := true in 
            punit.star 
          else 
            punit.star in 
        punit.star 
      else punit.star in
    retVal 

/-
  template<typename T>
  bool RosFilter<T>::preparePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg,
                                 const std::string &topicName,
                                 const std::string &targetFrame,
                                 const bool differential,
                                 const bool relative,
                                 const bool imuData,
                                 std::vector<int> &updateVector,
                                 Eigen::VectorXd &measurement,
                                 Eigen::MatrixXd &measurementCovariance)

  Primary method in which error occurs.
-/
def RosFilter.preparePose (rf : RosFilter) : 
  timestamped utc.coords (pose3d base_link.coords) → bool → bool → bool →  punit := 
  λ msg, 
  λ differential,
  λ relative,
  λ imuData,

/-
bool retVal = false;
    tf2::Stamped<tf2::Transform> poseTmp;

    tf2::Transform curMeasurement;

-/
  let retVal : bool := false in 
  let poseTmp : timestamped utc.coords (pose3d odom.coords) := inhabited.default _ in 

  let curMeasurement : nat := 0 in 
  /-
    std::string finalTargetFrame;
    if (targetFrame == "" && msg->header.frame_id == "")
    {
      finalTargetFrame = worldFrameId_;
      poseTmp.frame_id_ = finalTargetFrame;
    }
    else if (targetFrame == "")
    {
      finalTargetFrame = msg->header.frame_id;
      poseTmp.frame_id_ = finalTargetFrame;
    }
    else
    {
      finalTargetFrame = targetFrame;
      poseTmp.frame_id_ = (differential && !imuData ? finalTargetFrame : msg->header.frame_id);
    }-/
  let if0 := 
    if true ∧ true then 
     punit.star 
    else if true then  
      punit.star
    else 
      punit.star in
  /-
    poseTmp.stamp_ = msg->header.stamp;
    poseTmp.setOrigin(tf2::Vector3(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z));
  -/
  let poseTmp0 : timestamped utc.coords (pose3d odom.coords)  := ⟨poseTmp.timestamp, {
    position := (mk_position3d base_link.coords msg.value.position.x msg.value.position.y msg.value.position.z )
    ..poseTmp
  }⟩ in
  /-
    tf2::Quaternion orientation;
  -/
  let orientation : orientation3d base_link.coords  := inhabited.default _ in 
  /-
    if (msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 &&
       msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
    {
      orientation.setValue(0.0, 0.0, 0.0, 1.0);
      if (updateVector[StateMemberRoll] || updateVector[StateMemberPitch] || updateVector[StateMemberYaw])
      {
      }
    }
    else
    {
      tf2::fromMsg(msg->pose.pose.orientation, orientation);
      if (fabs(orientation.length() - 1.0) > 0.01)
      {
        orientation.normalize();
      }
    }
  -/
  let if0 : punit := 
    if 
      poseTmp0.value.orientation.to_orientation.to_vectr_basis.basis_vectrs 0 ≠ 0 ∧ 
      poseTmp0.value.orientation.to_orientation.to_vectr_basis.basis_vectrs 1 ≠ 0 ∧ 
      poseTmp0.value.orientation.to_orientation.to_vectr_basis.basis_vectrs 2 ≠ 0 then 
      let setValueCall := orientation.setValue 0 0 0 1 in 
      punit.star
    else 
      let orientation := msg.value.orientation in 
      /-
      if (fabs(orientation.length() - 1.0) > 0.01)
      {
        orientation.normalize();
      }
      -/
      punit.star in 
      /-
    poseTmp.setRotation(orientation);
      -/
      let setRotationCall := poseTmp0.value.setRotation orientation in
      /-
    tf2::Transform targetFrameTrans;
    bool canTransform = RosFilterUtilities::lookupTransformSafe(tfBuffer_,
                                                                finalTargetFrame,
                                                                poseTmp.frame_id_,
                                                                poseTmp.stamp_,
                                                                tfTimeout_,
                                                                targetFrameTrans);

    if (canTransform)
    {
      tf2::Matrix3x3 maskPosition(updateVector[StateMemberX], 0, 0,
                                  0, updateVector[StateMemberY], 0,
                                  0, 0, updateVector[StateMemberZ]);

      tf2::Matrix3x3 maskOrientation(updateVector[StateMemberRoll], 0, 0,
                                     0, updateVector[StateMemberPitch], 0,
                                     0, 0, updateVector[StateMemberYaw]);

      -/
      let targetFrameTrans : geom3d_transform odom.coords odom.coords := inhabited.default _ in 
      let canTransform : bool := RosFilterUtilities.lookupTransformSafe poseTmp0.timestamp rf.tfTimeout_ targetFrameTrans in

      punit.star
      /-
      To Dr. Sullivan - the method goes on for 300 lines of c++ code, some of which looks like a quite a task to parse and model. 
      So, in the interest of time, I am pausing and moving on for now unless this comment is read and I am informed to do otherwise.
      -/ 


