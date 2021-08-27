import .phys.time.time
import .phys.time_series.geom3d
import .std.time_std
import .std.geom3d_std
import .phys.time_series.geom3d
import data.real.basic
noncomputable theory

def milliseconds := (0.001)           -- names not clear, inverted?
def milliseconds_to_seconds := 1000   -- names not clear, inverted?
def seconds := 1                      -- think about this more
def nanoseconds := (0.000000001)
def nanoseconds_to_seconds := 1000000000

-- TODO: Should come from resp. std libraries and be distributed to them accordingly
namespace std
def time (p :scalar) : time time_std_space := mk_time time_std_space p
def duration (d :scalar) : duration time_std_space := mk_duration _ d
def position (x y z :scalar) : position3d geom3d_std_space := mk_position3d _ x y z
def displacement (x y z :scalar) : displacement3d geom3d_std_space := mk_displacement3d _ x y z
end std
/-
We need to assume a physical interpretation of the data
representing our coordinate system on time. 

The only referenced time coordinate system in this example is a system time - which
is not a UTC timestamp, rather, a zero-initiated time expressed in milliseconds, indicating
the time since the robot was booted. We define this ACS in terms of UTC (to avoid using std_space directly as requested last week?).

(1) ORIGIN: 0

(2) BASIS VECTORS
    basis0 
      - points to the future
      - unit length is 1 millisecond 
(3) ACS is given by [Origin, b0]
-/

namespace system_boot_time_ms
axiom δ : scalar
axiom ε : scalar
def origin := std.time δ   
def basis := std.duration (ε*milliseconds)    
def frame := mk_time_frame origin basis
def coords := mk_time_space frame
def time (t :scalar) := mk_time coords t
def duration (d :scalar) := mk_duration coords d
end system_boot_time_ms

/-
We also need an ACS for calls to "synchronize_stamp". These calls return actual system times in UTC,
so we shift up the coordinate to a more reasonable origin point and now define the units in seconds.

(1) ORIGIN: 1629311979 (current timestamp)

(2) BASIS VECTORS
    basis0 
      - points to the future
      - unit length is 1 second (as in UTC)
(3) ACS is given by [Origin, b0]
-/

namespace utc
axiom δ : scalar
axiom ε : scalar
def origin := std.time δ   
def basis := std.duration (ε*seconds)    
def frame := mk_time_frame origin basis
def coords := mk_time_space frame
def time (t :scalar) := mk_time coords t
def duration (d :scalar) := mk_duration coords d
end utc


/-
Lastly, we need a system time in nanoseconds, used in calls to "synchronize stamp".

(1) ORIGIN: 0

(2) BASIS VECTORS
    basis0 
      - points to the future
      - unit length is 1 nanoseconds (as in UTC)
(3) ACS is given by [Origin, b0]
-/

namespace utc_ns
def origin := utc.time 0   
def basis := utc.duration (nanoseconds)    
def frame := mk_time_frame origin basis
def coords := mk_time_space frame
def time (t :scalar) := mk_time coords t
def duration (d :scalar) := mk_duration coords d
end utc_ns


/-
We need to assume a physical interpretation of the data
representing our coordinate system on geom3d. See geom3d_std.lean
for more details on the coordinate system and physical interpretation.


We define a world frame. It's not fully clear if this is necessary from the codebase, 
but we will use this world space and define several body frames in terms of it. In the name,
we recognize that the rice420.coords implementation's origin can be interpreted as ENU. 

Note also:
https://git.scc.kit.edu/uqdpy/mavros/tree/master/mavros

"The translation from GPS coordinates to local geocentric coordinates require the definition
of a local origin on the map frame, in ECEF, and calculate the offset to it in ENU. All
the conversions are supported by GeographicLib classes and methods and implemented in the
global_position plugin."
-/
namespace map_enu  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := std.position 0 0 0      -- looking in from doorway, the back lower left corner  
def basis_0 := std.displacement 1 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := std.displacement 0 1 0 -- to door along weset wall; 1m; right
def basis_2 := std.displacement 0 0 1 -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end map_enu

/-

The local origin frame is used, *I believe* is a world-fixed frame, whereas "fcu" would be the pose, perhaps
coming from IMU data, representing the current pose/ACS of the robot itself. 

We define the robot to be 3 meters to the right of the left wall, 4 meters
in north of the bottom wall, and 1 meter above the ground. It's orientation is "local NED", which
actually should be referred to as "ESD" more like. Please see diagram in one of the related issues:
https://github.com/mavlink/mavros/issues/216


Note also:
https://git.scc.kit.edu/uqdpy/mavros/tree/master/mavros

"The translation from GPS coordinates to local geocentric coordinates require the definition
of a local origin on the map frame, in ECEF, and calculate the offset to it in ENU. All
the conversions are supported by GeographicLib classes and methods and implemented in the
global_position plugin."
-/
namespace local_origin_local_ned  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := map_enu.position 0 0 0      -- looking in from doorway, the back lower left corner  
def basis_0 := map_enu.displacement 1 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := map_enu.displacement 0 1 0 -- to door along weset wall; 1m; right
def basis_2 := map_enu.displacement 0 0 1 -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end local_origin_local_ned

/-
We define a separate ACS for the local origin now oriented in terms of ENU, as necessary to integrate with ROS

For short-range Cartesian representations of geographic locations, use the east north up [5] (ENU) convention:

X east
Y north
Z up

, required for such things as visualization of local positions in RVIZ.

-/
namespace local_origin_enu  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := local_origin_local_ned.position 0 0 0      -- looking in from doorway, the back lower left corner  
def basis_0 := local_origin_local_ned.displacement 1 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := local_origin_local_ned.displacement 0 (-1) 0 -- to door along weset wall; 1m; right
def basis_2 := local_origin_local_ned.displacement 0 0 (-1) -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end local_origin_enu


/-
https://github.com/mavlink/mavros/issues/216
-/



open classical
local attribute [instance] prop_decidable

structure UAS := 
  (imu_orientation : orientation3d local_origin_enu.coords)

/-

tf::Quaternion UAS::get_attitude_orientation()
{
	lock_guard lock(mutex);
	return imu_orientation;
}

-/
def UAS.get_attitude_orientation (uas : UAS) : orientation3d local_origin_enu.coords := 
  uas.imu_orientation

/-
ros::Time UAS::synchronise_stamp(uint32_t time_boot_ms) {
	// copy offset from atomic var
	uint64_t offset_ns = time_offset;

	if (offset_ns > 0) {
		uint64_t stamp_ns = static_cast<uint64_t>(time_boot_ms) * 1000000UL + offset_ns;
		return ros_time_from_ns(stamp_ns);
	}
	else
		return ros::Time::now();
}
-/

#check time_std_space.mk_time_transform_to utc.coords 

#check system_boot_time_ms.coords 

axiom time_offset : scalar 
def UAS.synchronise_stamp (uas : UAS) : time system_boot_time_ms.coords → time utc.coords := 
  λ time_boot_ms,
  let offset_ns : duration utc_ns.coords := mk_duration _ time_offset in
 
  if offset_ns > 0 then 
    let stamp_ns := 
      (system_boot_time_ms.coords.mk_time_transform_to utc_ns.coords).transform_time time_boot_ms in 
    (utc_ns.coords.mk_time_transform_to utc.coords).transform_time stamp_ns
  else 
    mk_time _ 0

/-


	ros::Publisher local_position;
	tf::TransformBroadcaster tf_broadcaster;
-/

structure Publisher :=
  mk::

def Publisher.publish
  (p : Publisher) : timestamped utc.coords (pose3d local_origin_enu.coords) → punit := 
  λp, punit.star 

structure TransformBroadcaster :=
  mk::

def TransformBroadcaster.sendTransform 
  (tb : TransformBroadcaster) : timestamped utc.coords (pose3d local_origin_enu.coords) → punit := 
  λp, punit.star 

structure LocalPositionPlugin := 
 (uas : UAS)
 (local_position : Publisher)
 (tf_broadcaster : TransformBroadcaster)


def pose3d.setOrigin {f : geom3d_frame} {sp : geom3d_space f} (p: pose3d sp) : position3d sp → punit := 
  λ pos, 
  let p0 : pose3d sp := {
    position := pos, 
    ..p
  } in 
  punit.star

def pose3d.setRotation {f : geom3d_frame} {sp : geom3d_space f} (p: pose3d sp) : orientation3d sp → punit := 
  λ ort, 
  let p0 : pose3d sp := {
    orientation := ort, 
    ..p
  } in 
  punit.star 

def poseTFToMsg {f : geom3d_frame} {sp : geom3d_space f} (p1 p2 : pose3d sp ) : punit := 
  let p2_0 := p1 in 
  punit.star 

/-

  void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
  
  This functions accepts one argument, of type "mavlink_message_t", which is a dataframe that can contain
  various types of data. In this case, it's a position in the "local_origin" frame. So, we interpret the argument
  as such.
-/
def LocalPositionPlugin.handle_local_position_ned (lpp : LocalPositionPlugin) : 
  timestamped system_boot_time_ms.coords (position3d local_origin_local_ned.coords) → punit := 
  λ msg, 
  /-
  
    static inline void mavlink_msg_local_position_ned_decode(const mavlink_message_t* msg, mavlink_local_position_ned_t* local_position_ned)
    {
    #if MAVLINK_NEED_BYTE_SWAP
      local_position_ned->time_boot_ms = mavlink_msg_local_position_ned_get_time_boot_ms(msg);
      local_position_ned->x = mavlink_msg_local_position_ned_get_x(msg);
      local_position_ned->y = mavlink_msg_local_position_ned_get_y(msg);
      local_position_ned->z = mavlink_msg_local_position_ned_get_z(msg);
      local_position_ned->vx = mavlink_msg_local_position_ned_get_vx(msg);
      local_position_ned->vy = mavlink_msg_local_position_ned_get_vy(msg);
      local_position_ned->vz = mavlink_msg_local_position_ned_get_vz(msg);
    #else
      memcpy(local_position_ned, _MAV_PAYLOAD(msg), 28);
    #endif

    mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);

    As seen above, we are simply moving the generic dataframe type into a more-specific position type, "mavlink_local_position_ned_t"
    To do that, we use the mavlink_msg_local_position_ned_decode function, which, as seen in the definition (omitted some parts), 
    requires indexing into a byte array at certain offsets. So, I am avoiding formalizing that for now. Regardless,
    the semantics of the call is simply that we're assigning to pos_ned from the value contained in msg - which is treated
    as a simple assignment given we're intepreting both as local_origin positions.
  -/
  let pos_ned : timestamped system_boot_time_ms.coords (position3d local_origin_local_ned.coords) := inhabited.default _ in 
  let pos_ned0 : timestamped system_boot_time_ms.coords (position3d local_origin_local_ned.coords) := msg in 
  /-
  tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos_ned.y, pos_ned.x, -pos_ned.z));
		transform.setRotation(uas->get_attitude_orientation());-/
  let transform : pose3d local_origin_enu.coords := inhabited.default _ in 
  let setOriginCall := transform.setOrigin (mk_position3d _ (pos_ned.value.y) (pos_ned.value.x) (-pos_ned.value.z)) in
  let setRotationCall := transform.setRotation lpp.uas.get_attitude_orientation in
  /-
	auto pose = boost::make_shared<geometry_msgs::PoseStamped>();

	tf::poseTFToMsg(transform, pose->pose);
  -/
  let pose : timestamped utc.coords (pose3d local_origin_enu.coords) := inhabited.default _ in 
  let poseTFToMsgCall := poseTFToMsg transform pose.value in
  let pose0 : timestamped utc.coords (pose3d local_origin_enu.coords) := {
    timestamp := lpp.uas.synchronise_stamp pos_ned.timestamp,
    ..pose
  } in 

  let if0 : punit :=
    if true then 
      punit.star
    else 
      punit.star in 
  
  let publishCall := lpp.local_position.publish pose in 
  /-
  pose->header.frame_id = frame_id;
		pose->header.stamp = uas->synchronise_stamp(pos_ned.time_boot_ms);

		if (send_tf)
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						pose->header.stamp,
						frame_id, child_frame_id));

		local_position.publish(pose);
  -/



  /-
  tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos_ned.y, pos_ned.x, -pos_ned.z));
		transform.setRotation(uas->get_attitude_orientation());

		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();

		tf::poseTFToMsg(transform, pose->pose);
		pose->header.frame_id = frame_id;
		pose->header.stamp = uas->synchronise_stamp(pos_ned.time_boot_ms);

		if (send_tf)
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						pose->header.stamp,
						frame_id, child_frame_id));

		local_position.publish(pose);
  -/

/-
  	void handle_local_position_ned(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_local_position_ned_t pos_ned;
		mavlink_msg_local_position_ned_decode(msg, &pos_ned);

    
    static inline void mavlink_msg_local_position_ned_decode(const mavlink_message_t* msg, mavlink_local_position_ned_t* local_position_ned)
    {
    #if MAVLINK_NEED_BYTE_SWAP
      local_position_ned->time_boot_ms = mavlink_msg_local_position_ned_get_time_boot_ms(msg);
      local_position_ned->x = mavlink_msg_local_position_ned_get_x(msg);
      local_position_ned->y = mavlink_msg_local_position_ned_get_y(msg);
      local_position_ned->z = mavlink_msg_local_position_ned_get_z(msg);
      local_position_ned->vx = mavlink_msg_local_position_ned_get_vx(msg);
      local_position_ned->vy = mavlink_msg_local_position_ned_get_vy(msg);
      local_position_ned->vz = mavlink_msg_local_position_ned_get_vz(msg);
    #else
      memcpy(local_position_ned, _MAV_PAYLOAD(msg), 28);
    #endif
    }
    

		ROS_DEBUG_THROTTLE_NAMED(10, "position", "Local position NED: boot_ms:%06d "
				"position:(%1.3f %1.3f %1.3f) speed:(%1.3f %1.3f %1.3f)",
				pos_ned.time_boot_ms,
				pos_ned.x, pos_ned.y, pos_ned.z,
				pos_ned.vx, pos_ned.vy, pos_ned.vz);

		/* TODO: check convertion to ENU
		 * I think XZY is not body-fixed, but orientation does.
		 * Perhaps this adds additional errorprone to us.
		 * Need more tests. Issue #49.
		 *
		 * orientation in ENU, body-fixed
		 */
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos_ned.y, pos_ned.x, -pos_ned.z));
		transform.setRotation(uas->get_attitude_orientation());

		auto pose = boost::make_shared<geometry_msgs::PoseStamped>();

		tf::poseTFToMsg(transform, pose->pose);
		pose->header.frame_id = frame_id;
		pose->header.stamp = uas->synchronise_stamp(pos_ned.time_boot_ms);

		if (send_tf)
			tf_broadcaster.sendTransform(
					tf::StampedTransform(
						transform,
						pose->header.stamp,
						frame_id, child_frame_id));

		local_position.publish(pose);
	}


	void send_vision_transform(const tf::Transform &transform, const ros::Time &stamp) {
		// origin and RPY in ENU frame
		tf::Vector3 position = transform.getOrigin();
		double roll, pitch, yaw;
		tf::Matrix3x3 orientation(transform.getBasis());
		orientation.getRPY(roll, pitch, yaw);

		/* Issue #60.
		 * Note: this now affects pose callbacks too, but i think its not big deal.
		 */
		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		// TODO: check conversion. Issue #49.
		vision_position_estimate(stamp.toNSec() / 1000,
				position.y(), position.x(), -position.z(),
				roll, -pitch, -yaw);	// ??? please check!
	}
-/
