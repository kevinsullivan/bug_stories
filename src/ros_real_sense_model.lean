

import ..phys.time.time
import ..phys.time_series.geom3d
import ..std.time_std
import ..std.geom3d_std

noncomputable theory

-- TODO: Should come from some library
def milliseconds := (0.001)           -- names not clear, inverted?
def milliseconds_per_second := 1000   -- names not clear, inverted?
def seconds := 1                      -- think about this more

-- TODO: Should come from resp. std libraries and be distributed to them accordingly
namespace std
def time (p : scalar) : time time_std_space := mk_time time_std_space p
def duration (d : scalar) : duration time_std_space := mk_duration _ d
def position (x y z : scalar) : position3d geom3d_std_space := mk_position3d _ x y z
def displacement (x y z : scalar) : displacement3d geom3d_std_space := mk_displacement3d _ x y z
end std

-- Geometric world
namespace world  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := std.position 0 0 0      -- looking in from doorway, the back lower left corner  
def basis_0 := std.displacement 1 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := std.displacement 0 1 0 -- to door along weset wall; 1m; right
def basis_2 := std.displacement 0 0 1 -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end world

-- Camera in world
/-
We assume the camera is 1 meters in front of the rear wall, 
1 meter to the right, and 1 meter above the ground.
It's orientation is found by performing a roll and pitch rotation of 1 radian each,
relative to the standard orientation of Rice described above. 
-/
namespace camera
def origin := world.position 2 1 1
def basis_0 := world.displacement 0.54 0 0.84
def basis_1 := world.displacement 0.71 0.54 (-0.45)
def basis_2 := world.displacement (-0.45) 0.84 0.29
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords : geom3d_space _ := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end camera

/-
We define an IMU space. It is located on the and thus defined in terms of the camera, although
it has a modified orientation, as described in the link below.
https://www.intelrealsense.com/how-to-getting-imu-data-from-d435i-and-t265/
-/
namespace imu 
def origin := camera.position 2 1 1
def basis_0 := camera.displacement 1 0 0
def basis_1 := camera.displacement 0 0 (-1)
def basis_2 := camera.displacement 0 1 0
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords : geom3d_space _ := mk_geom3d_space frame
def position (x y z : scalar) := mk_position3d coords x y z
def displacement (x y z : scalar) := mk_displacement3d coords x y z
end imu

namespace utc
def origin := std.time 0   -- origin; first instant of January 1, 1970
def basis := std.duration 1    -- basis; "second;" the smallest non-variable unit in UTC
def frame := mk_time_frame origin basis -- recall why "time" is part of the constructor name? factor out?
def coords := mk_time_space frame  -- "cosys"?
def time (t : scalar) := mk_time coords t
def duration (d : scalar) := mk_duration coords d
end utc


/-
We model the camera's OS clock, referred to as providing
"System Time" in RealSense nomenclature.  The clock 
nomimally reports time through the camera API in 
milliseconds elapsed since the UTC origin. That said, 
the clock can be off by a little in both origin and rate.
The camera clock thus has its own affine frame, with its
origin varying from the UTC origin by an unknown small
amount, and the speed of its clock also varying from the 
nominal, again probably by a small but unknown amount. We
model these unknows as assumed scalar values, δ and ε.

See the documentation for the RealSense data frame (not 
affine frame!) class specification, here:
https://intelrealsense.github.io/librealsense/doxygen/rs__frame_8h.html#a55750afe3461ea7748fbb2ef6fb19e8a
-/

namespace camera_os_system_time_ms
axioms (δ ε : scalar)
def origin := utc.time δ     
def basis := mk_duration utc.coords (milliseconds*ε) 
def frame := mk_time_frame origin basis
def coords := mk_time_space frame
def time (t : scalar) := mk_time coords x 
def duration (d : scalar) := mk_duration coords d 
end camera_os_system_time_ms


/-
As we must convert from milliseconds (as retrieved through the rs2::Frame API) to seconds,
we express a new ACS which simply conveys the camera_system_time.coords, defined and described
just above, with units in seconds rather than milliseconds.
-/

namespace camera_os_system_time_seconds
def orig := mk_time camera_os_system_time_ms.coords 0 
def base := mk_duration camera_os_system_time_ms.coords milliseconds_per_second
def frame := mk_time_frame orig base
def coords : time_space _ := mk_time_space frame
def time (t : scalar) := mk_time coords x 
def duration (d : scalar) := mk_duration coords d 
end camera_os_system_time_seconds


/-
Next we model the frame of reference for the "hardware time" 
of the RealSense Camera, which is meant to convey how much

time has passed, stored in microseconds but exposed in milliseconds over the API, since the camera has begun
transmitting data.
(https://intelrealsense.github.io/librealsense/doxygen/rs__frame_8h.html#a55750afe3461ea7748fbb2ef6fb19e8a)
See also these descriptions on the "hardware time":
https://github.com/IntelRealSense/librealsense/issues/4525
"The hardware timestamps are all 32bits wide and are all zero-initiated. 
  In order to convert the HW time to 64bit EPOCH you should probably use the global_time that was added since then."
"These metadata attribute timestamps start from 0 (i.e. device startup time) "
See also this comment from rs_frame.hpp in librealsense:
/**
* retrieve the time at which the frame was captured
* During the frame's lifetime it receives timestamps both at the device and host levels.
* The different timestamps are gathered and managed under the frame's Metadata attributes.
* Chronologically the list of timestamps comprises of:
* SENSOR_TIMESTAMP  - Device clock. For video sensors designates the middle of exposure. Requires metadata support.
* FRAME_TIMESTAMP   - Device clock. Stamped at the beginning of frame readout and transfer. Requires metadata support.
* BACKEND_TIMESTAMP - Host (EPOCH) clock in scalarernel space. Frame transfer from USB Controller to the USB Driver.
* TIME_OF_ARRIVAL   - Host (EPOCH) clock in User space. Frame transfer from the USB Driver to Librealsense.
*
* During runtime the SDK dynamically selects the most correct representaion, based on both device and host capabilities:
* In case the frame metadata is not configured:
*   -   The function call provides the TIME_OF_ARRIVAL stamp.
* In case the metadata is available the function returns:
*   -   `HW Timestamp` (FRAME_TIMESTAMP), or
*   -   `Global Timestamp`  Host-corrected derivative of `HW Timestamp` required for multi-sensor/device synchronization
*   -   The user can select between the unmodified and the host-calculated Hardware Timestamp by toggling
*       the `RS2_OPTION_GLOBAL_TIME_ENABLED` option.
* To query which of the three alternatives is active use `get_frame_timestamp_domain()` function call
* \return            the timestamp of the frame, in milliseconds according to the elaborated flow
*/
This is a zero-initiated time. It does not reflect the current time in UTC, rather, it conveys how much
time has passed since the camera has begun transmitting data.

We define this in terms of UTC time, giving it an origin of 0, with no intrinsic "drift". We share the same
dilation factor as camera_system_time.coords, as we are making the assumption that both of these measurements
share the same rate of error.
(1) ORIGIN: 0, reflecting that each run of the camera begins at the UTC origin, rather than at the current time
(2) BASIS VECTORS
    basis0
      - points to the future
      - unit length is 1 millisecond
      - A dilation factor, ε₁, scales the basis vector,
        to convey the speed of the clock relative to an atomic approximation of UTC time
(3) ACS is given by [Origin, b0]
-/

namespace camera_hardware_time_ms
axioms (start_time ε : scalar) -- clock offset and scaling error factors; not used here (TODO)
def origin := mk_time utc.coords start_time        -- interp:
def basis := mk_duration utc.coords (milliseconds*ε) -- interp:  

def frame := mk_time_frame origin basis
def coords : time_space _ := mk_time_space frame
def time (t : scalar) := mk_time coords t
def duration (d: scalar) := mk_duration coords d
end  camera_hardware_time_ms


namespace camera_hardware_time_seconds
def origin := camera_hardware_time_ms.time 0
def basis := camera_hardware_time_ms.duration milliseconds_per_second
def frame := mk_time_frame origin basis
def coords := mk_time_space frame
def time (t : scalar) := mk_time coords t
def duration (d: scalar) := mk_duration coords d
end camera_hardware_time_seconds

/-
This is the ROS client (of the RealSense camera) node's system time,
an OS approximation of the current UTC time expressed in units seconds
One gap to mention here is that this will be used to annotate the system time,
which has a native representation of seconds + nanoseconds, so there is
arguably some mismatch when we annotate the ros::Time variable simply as having
units in seconds. TODO: Innovate 2D representations of time, so a frame in a
2D space but with coincident axes, one for seconds, one for nanoseconds.
-/

namespace ros_time_in_seconds  
  axiom δ₂ : scalar
  axiom ε₂ : scalar
  def origin := mk_time utc.coords δ₂
  def basis := mk_duration utc.coords (seconds*ε₂)  
  def frame := mk_time_frame origin basis
  def coords := mk_time_space frame
  def time (t : scalar) := mk_time coords t
end ros_time_in_seconds

/-
This frame gives the camera's estimate of what its times mean to the
robot/client. Lastly formalize the "global time." This is a conversion of the "camera
hardware time", adjusted such so that it's "dilation rate" has been adjusted
from that of the camera's clock to that of a client's clock. The camera figures
out how to map its hardware time into the client-robot OS system clock.
(https://intelrealsense.github.io/librealsense/doxygen/rs__frame_8h.html#a55750afe3461ea7748fbb2ef6fb19e8a)
This is done in RealSense by a simply linear regression algorithm, and in
reality may be subject to some error if the clock varies over time, but we
are only able to model linear relationships, and in this case, the necessary
conversion reduces to adjusting the hardware's basis by a factor of ε₂/ε₁.
The origin remains at zero while the basis vector includes a dilation factor,
ε₂/ε₁, which removes the scaling of the camera's clock, and applies the ROS
node's clock dilation. TODO: Double check and document.
See a useful explanation here:
https://github.com/IntelRealSense/librealsense/issues/4505
When using global timestamps the timestamp returned is the device hardware timestamp synchronized with the host time. In order to do this a linear regression model is created that takes samples. These samples are (x,y) pairs, where x is the device time and y is the host time. A linear regression is performed on the samples to generate a baisc line mode (y=mx+b). This sampling process takes place in:
-/

namespace camera_global_time_ms
  axiom δ₁ : scalar
  axioms ε₁ ε₂ : scalar
  def origin := mk_time camera_hardware_time_ms.coords δ₁
  def basis := mk_duration camera_hardware_time_ms.coords (ε₂/ε₁)
  def frame := mk_time_frame origin basis
  def coords := mk_time_space frame
  def time (t : scalar) := mk_time coords t
  def duration (t : scalar) := mk_duration coords t
end camera_global_time_ms

/-
CODE FORMALIZATION OVERVIEW
The parameter of the method imu_callback_sync, "frame", is either
timestamped Acceleration or Angular Velocity Vector. We have no
implementation for either in Peirce (or for sum types for that matter).
Per discussion on last 8/6, this is replaced with a Displacement3D.
We model two versions of this method, reflecting that different
sorts of errors that can occur. The gap/limitation that causes us to
need to construct two versions of this method is that Peirce, as it
is currently constructed, annotates a single execution path. Thus,
to model two execution paths, we need two versions of a formalism.
In the first instance, we presume that we've received
"_camera_time_base" in the camera_hardware_time.coords ACS,
but that we've just received a dataframe whose timestamp
"dataframe.timestamp" domain is expressed in camera_system_time.coords.
What transpires is that we will subtract two timestamps -
"dataframe.timestamp - _camera_time_base", which will yield a
type error. In the actual C++ code, this computation will not
yield a type error,  rather, it will produce an extremely large
"change in timestamps" (doubling of the UTC timestamp) which
is physically meaningless and can lead to client failures.
The dilation rates + units of the respective ACS's of the
operands are compatibile, in this case, but the origins are
obviously off by dramatic amounts.
In the second instance, we presume that we've received
"_camera_time_base" in the camera_global_time.coords ACS,
but that we've just received a dataframe whose timestamp
"dataframe.timestamp" domain is in camera_hardware_time.coords.
The only difference from above is that, the error will
result in a smaller error - to the tune of a few hundred
MS  (varying over time, as the dilation rates are not in
equal in this case).

There are several other others that manifest in either treatment. One is that we annotate a variable suffixed "_ms"
  with a frame expressed in "milliseconds" units, whereas its actual computation is very clearly expressed in seconds.
  We formalize this such that there is clearly a conversion
  to (a) seconds (ACS), whereas the variable is typed in (a) milliseconds (ACS), such that a type error manifests.
Next, when constructing a timestamped datum (CImuData imu_data), the developers are treating a double value which clearly represents a duration, not a
  time, as a timestamp. In our formalization, a type error manifests when constructing the timestamped value.
Lastly, when adding a coordinate from a time expressed in our platform time WITH a coordinate from a time expressed in one of the
  camera's timestamp domains, there is no error in our formalization (currently - discussed extensively with Dr. Sullivan),
  although we would like one to be. The specific error that we would like to capture is that, due to differences in dilation rates,
  the addition of _ros_time_base with imu_msg.timestamp (an alias of elapsed_camera_ms), will not produce an accurate timestamp,
  even when elapsed_camera_ms is computed properly (using matching timestamp domains for both timestamp domains).
 
void BaseRealSenseNode::imu_callback_sync(rs2::frame dataframe, imu_sync_method sync_method)
-/
axiom now : scalar  -- TODO: Eliminate this
axiom first_send_time : scalar

/-
Transcribed "Class" of property "synced_imu_publisher of BaseRealSenseNode"
-/
structure Publisher_v1 :=
  mk::

/-
This method is called in imu_callback_sync, where the precisely modeled error is located
-/
def Publisher_v1.Publish (p : Publisher_v1) : 
  timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) → punit := 
  λ tp, 
  punit.star

/-
Transcribed "Class" which contains member function "imu_callback_sync", where the precisely modeled error is located
-/
structure BaseRealSenseNode_v1 := 
  (ros_time_base : time ros_time_in_seconds.coords)
  (camera_time_base : time camera_hardware_time_ms.coords)
  (synced_imu_publisher : Publisher_v1)

/-
Member function setBaseTime, which is used to set "_ros_time_base" and "_camera_time_base" - critical components of the error
-/
def BaseRealSenseNode_v1.setBaseTime (b : BaseRealSenseNode_v1) : time camera_hardware_time_ms.coords → punit := 
  λ camera_time_base,
  let b0 : BaseRealSenseNode_v1 := {
    ros_time_base := ros_time_in_seconds.time now,
    ..b 
  } in
  let b1 : BaseRealSenseNode_v1 := {
    camera_time_base := camera_time_base,
    ..b0
  } in 
  punit.star 

/-
Helper method used in imu_callback_sync - adds a value to the "imu_msgs" container, which 
eventually gets populated with potentially incorrect timestamps.
-/
def BaseRealSenseNode_v1.FillImuData_Copy (b : BaseRealSenseNode_v1) 
  : 
  timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) → 
  list (timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords)) → punit := 
  λimu_data, λ imu_msgs, 
  let imu_msgs0 := imu_msgs ++ [imu_data] in 
  punit.star

/-
Helper method used in imu_callback_sync
-/

def BaseRealSenseNode_v1.ImuMessage_AddDefaultValues (b : BaseRealSenseNode_v1) 
  : 
  timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) → punit := 
  λimu_msg, 
  punit.star



-- time-stamped accelerometer reading OR gyroscope reading
def BaseRealSenseNode_v1.imu_callback_sync_v1 (b : BaseRealSenseNode_v1) : timestamped camera_os_system_time_ms.coords (displacement3d imu.coords) → punit :=

/-
  We define the argument to the method, dataframe. It has an interpretation of
    timestamped camera_time.coords (displacement3d camera_imu.coords)
  , although it's actual physical type manifest in the code would be an Acceleration or Angular Velocity Vector, representing
  a timestamped reading coming from a Gyroscope or Accelerometer.
  -/

  λ dataframe : timestamped camera_os_system_time_ms.coords (displacement3d imu.coords),
  /-
    In the original code, it represents a timestamped Acceleration 
    or Angular Velocity vector coming from a Gyroscope or Accelerometer.
    scalarevin: A bit more work could add value here.
  -/
  
  /-
    double frame_time = frame.get_timestamp();
    In this line, we extract the timestamp of the dataframe, which represents the timestamp at which the IMU data was gathered.
  -/
  let dataframe_time := dataframe.timestamp in 

  /-setBaseTime(frame_time, frame.get_frame_timestamp_domain());
    _ros_time_base = ros::Time::now();
    _camera_time_base = frame_time;
    A call is made to the method "setBaseTime". setBaseTime contains two salient lines: setting both _ros_time_base and
    _camera_time_base. _ros_time_base is intended to represent the first time point at which the camera
    has sent an IMU data reading, expressed in terms of of the local system clock that is reading
    data from the RealSense camera data feed. _camera_time_base is intended to represent the first time point at which the camera
    has sent an IMU data reading, expressed in terms of the clock directly on the camera.
  -/

  let setBaseTimeCall := b.setBaseTime (camera_hardware_time_ms.time first_send_time) in

  /-
    double elapsed_camera_ms = (/*ms*/ frame_time - /*ms*/ _camera_time_base) / 1000.0; -- NB: bad naming in code
    We take the difference between the first camera measurement, when the method "imu_callback_sync"
    was first called, and the current camera measurement, as contained in "dataframe_time". Thus, the
    resulting difference is a duration in time. The variable name suggests that the resulting computation
    is in milliseconds. The dataframe_time and the _camera_time_base are expressed in milliseconds due to
    the implementation of the camera's clock, however, the actual units are expressed in seconds, as there
    is a division by 1000. Thus, we transform the duration from its native millisecond frame to a seconds ACS.
    Mathematically, we might represent this as Tₘˢ(t₂-t₁). Finally, there is a type error, as we are attempting
    to assign a value in an ACS representing seconds to a variable in an ACS representing milliseconds, which
    portrays a misconception by the developer when naming this variable.
  -/
  let elapsed_camera_ms : duration camera_os_system_time_seconds.coords  
    := ((camera_os_system_time_ms.coords).mk_time_transform_to camera_os_system_time_seconds.coords).transform_duration
    (((dataframe_time -ᵥ b.camera_time_base) : duration camera_os_system_time_ms.coords)) in
  /-
  if (0 != _synced_imu_publisher->getNumSubscribers())
  {
  -/
  let if0 := 
    if true then 
    /-
        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        There are some uneventful assignments that occur here.
        The first casts (via static_casting) the vector-quantity data that the frame argument encapsulates to a "float3 object"
        and stores it into a variable named "crnt_reading".
        The second converts the prior "float3" object into an "Eigen::Vector3d" object, by extracting its x, y, and z coordinates,
        using those respectively in the constructor to Eigen::Vector3d, and binding the constructed value into a variable called v.
        We model in this in Lean simply by defining a value called "crnt_reading" and assigning the vector-valued data property stored in the
        timestamped dataframe method argument. Since the physical type of the dataframe is "timestamped camera_time.coords (displacement3d camera_imu.coords)",
        we know that the type of crnt_reading must simply be "displacement3d camera_imu.coords".
        Next, we construct the vector v in the code. We define a variable v, which, again, has the physical type "displacement3d camera_imu.coords",
          since it is built by simply constructing a new displacement3d using the exact same x, y, and z coordinates of the prior value, crnt_reading.
    -/
      let crnt_reading : displacement3d imu.coords := dataframe.value in
      let v : displacement3d imu.coords := mk_displacement3d imu.coords crnt_reading.x crnt_reading.y crnt_reading.z in
      
      /-
      private:
        class CimuData
        {
            public:
                CimuData() : m_time(-1) {};
                CimuData(const stream_index_pair type, Eigen::Vector3d data, double time):
                    m_type(type),
                    m_data(data),
                    m_time(time){};
                bool is_set() {return m_time > 0;};
            public:
                stream_index_pair m_type;
                Eigen::Vector3d m_data;
                double          m_time;
        };
      -/

      --CimuData imu_data(stream_index, v, elapsed_camera_ms);
      /-
      scalarevin: Here a duration (actually in seconds!) in a noisy UTC frame is being used as a point in time to serve as a timestamp for v.

      The constructor of CimuData in the next line of code simply re-packages the vector data stored in the original frame argument,
      whose physical interpretation was a timestamped displacement3d in the camera, back into an another object which represents
      a timestamped displacement3d, but this time, the timestamp is expressed in terms of the camera_time_seconds ACS, as the developer
      explicitly converted from milliseconds into seconds when constructing what is intended to be the timestamp, elapsed_camera_seconds.
    
      Thus, in order to formalize this in Lean, declare a variable called "imu_data" with type "timestamped camera_time_seconds (displacement3d camera_imu.coords)",
      and populate it using the vector-valued data v (which is, again, the same displacement3d encapsulated in the argument to the method),
      as well as the variable "elapsed_camera_ms" as a timestamp, which, again is a duration, not a point,
      as it is the result of subtracting two time variables, and so, we see an error here in our formalization.
      -/
      let imu_data : timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) := ⟨elapsed_camera_ms, v⟩ in
      /-
      std::deque<sensor_msgs::Imu> imu_msgs;
      switch (sync_method)
      {
          case NONE: //Cannot really be NONE. Just to avoid compilation warning.
          case COPY:
              FillImuData_Copy(imu_data, imu_msgs);
              break;
          case LINEAR_INTERPOLATION:
              FillImuData_LinearInterpolation(imu_data, imu_msgs);
              break;
      }
      -/
      /-
      We define a double-ended queue called "imu_msgs" and attempt to populate it. A call is made to the procedure "FillImuData_Copy"
      or "FillImuData_LinearInterpolation", which is omitted here. The purpose of these procedures includes are to construct IMU messages,
      specifically tuples of 2 3-dimensional (non-geometric) vectors, along with a linear interpolation that are beyond
      the scope of what we can currently formalize, and the resulting IMU messages stores either 0 (if the dataframe argument came from an accelerometer),
      1 (if the dataframe argument came from a gyroscope and sync_method is set to "COPY"), or n (if the dataframe argument came from a gyroscope
      and sync_method is set to "LINEAR_INTERPOLATION") into deque "imu_msgs". The purpose of the method call is that entries are added to imu_msgs, so this is
      simulated by simply instantiating the list with an initial value: imu_data - the timestamped value that we've constructed above. Note that
      due to our limitations in Peirce, we have annotated the type of imu_msgs as being the type of the data that we have available,
        "(timestamped camera_time_seconds (displacement3d camera_imu.coords))", as opposed to a timestamped IMU message, since we are not yet able to formalize the latter.
      -/

      let imu_msgs : list (timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords)) := [] in
      /-
      switch (sync_method)
      {
          case NONE: //Cannot really be NONE. Just to avoid compilation warning.
          case COPY:
              FillImuData_Copy(imu_data, imu_msgs);
              break;
          case LINEAR_INTERPOLATION:
              FillImuData_LinearInterpolation(imu_data, imu_msgs);
              break;
      }
      -/
      let FillImuData_CopyCall := b.FillImuData_Copy imu_data imu_msgs in 
      /-
      
        while (imu_msgs.size())
        {
          sensor_msgs::Imu imu_msg = imu_msgs.front();
  
          We now process each entry in the deque. Each entry has its timestamp updated, then it is published, and then it is removed from the queue, until the queue is empty.
          Firstly, we retrieve the front of the queue, which is simply a call to "list.head" in Lean, and,
          since "imu_msgs" has type "list (timestamped camera_time_seconds (displacement3d camera_imu.coords))",
          the resulting expression is simply of type "(timestamped camera_time_seconds (displacement3d camera_imu.coords))"
      -/
      let while0 := 
        if imu_msgs.length > 0 then 
          let imu_msg : timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) := imu_msgs.head in
          --ros::Time t(_ros_time_base.toSec() + imu_msg.header.stamp.toSec());
          /-
          The developers now construct a new timestamp for the IMU message first by
          retrieving "base time" of the "platform"/local system time (which was computed
          earlier, specifically in the first call to this method (imu_callback_sync)),
          stored in variable "_ros_time_base", and then converting its value into seconds
          along with extracting its underlying coordinate via a "toSec" call. Next, they
          retrieve timestamp of imu_msg, stored as "imu_msg.header.stamp" in C++ or
          "imu_msg.timestamp" in our formalization, whose value is an alias of
          elapsed_camera_ms, which represents the "offset"/difference between the current
          hardware/camera timestamp and the "base" of the hardware/camera time (which,
          again, was computed specifically in the first call to this method) with the overall
          expression expressed in seconds. The coordinates of this object are extracted
          via the "toSec" call. These two coordinates are added together and used as an
          argument in the construction of the ros::Time object.
        
          We've formalized this by interpreting t as a time expressed in the hardware_time.coords.
          We bind a value to it by constructing a new value of type "time camera_time_seconds"
            via our mk_time call. The complexity resides in how we compute the coordinates to
            the new time.
          We define an overload of the "toSec" call for both _ros_time_base and imu_msg.timestamp,
          whether to provide a global or context-dependent interpretation for toSec is a nuanced
          issue. Regardless, both overloads of "toSec", "_ros_time_base_toSec" and "_imu_msg_timestamp_toSec",
          simply extract the respective coordinates of _ros_time_base and _imu_msg_timestamp. Thus, the
          respective overloads are applied using the respective values, _ros_time_base and _imu_msg_timestamp,
          and the resulting expressions, which have type "scalar", are added together and supplied as an
          argument to the newly constructed time.
          Note here that, although there is no error here, there should be. The coordinates composing the addition operation hail from two different spaces: platform_time.coords and camera_time_seconds,
          which should yield a type error - as these coordinates hail from different ACSes.
          -/

          let t : time camera_os_system_time_seconds.coords :=
            mk_time _
            ((
              let _ros_time_base_toSec : time ros_time_in_seconds.coords → scalar :=
              λt,
                (t).coord in
              _ros_time_base_toSec b.ros_time_base
            )
            +   -- scalarEVIN: A frame error occurs here in a sense. Need to rethink why not detected here.
            (
              --casting time to duration discussed
              --Whether or not to first convert "imu_msg.timestamp" from a time (point) to a duration (vector) should be confirmed by Dr. S
              --let imu_time_as_duration := mk_duration camera_time_seconds imu_msg.timestamp.coord in
              let _imu_msg_timestamp_toSec : time camera_os_system_time_seconds.coords → scalar :=
                λt,
                  t.coord in
              _imu_msg_timestamp_toSec imu_msg.timestamp--imu_time_as_duration
            )) in 
          /-
          Finally, we update the timestamp of imu_msg with our newly misconstructed time, t
          There is no error at this position.
          imu_msg.header.stamp = t;
          -/
          let imu_msg0 : timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) := {
            timestamp := t,
            ..imu_msg
          } in
          let ImuMessage_AddDefaultValuesCall := b.ImuMessage_AddDefaultValues imu_msg in 
          let PublishCall := b.synced_imu_publisher.Publish imu_msg in 
          punit.star 
        else 
          punit.star in 
        punit.star
      else 
        punit.star in
    punit.star


/-
As discussed above, a second version of this method is presented. The only difference is
that, rather than "_camera_time_base" being in the camera_hardware_time.coords ACS and
"dataframe.timestamp" domain being expressed in the camera_system_time.coords ACS,
we have "_camera_time_base" being in the camera_global_time.coords ACS and
"dataframe.timestamp" domain being expressed in the camera_system_time.coords ACS, which reflects an error
where we have a "time dilation" discrepancy in the subtraction operation, rather than just a massive origin mismatch.
The reason, again, for maintaining two versions of the method is due to a limitation in our formalization, in that
we can only express one execution path. Thus, describing a different execution path, in this case inputs with differing
timestamp domains, must be formalized separately.
Verbose comments are omitted from this version - as they only differ in terms of the ACS of the "dataframe" argument to the method
(particularly, the ACS of the timestamp),
as well as the _camera_time_base's type (particularly, the dependent type's ACS value)
void BaseRealSenseNode::imu_callback_sync(rs2::frame dataframe, imu_sync_method sync_method)
-/
structure Publisher_v2 :=
  mk::

def Publisher_v2.Publish (p : Publisher_v2) : 
  timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) → punit := 
  λ tp, 
  punit.star

structure BaseRealSenseNode_v2 := 
  (ros_time_base : time ros_time_in_seconds.coords)
  (camera_time_base : time camera_os_system_time_ms.coords)
  (synced_imu_publisher : Publisher_v2)

def BaseRealSenseNode_v2.setBaseTime (b : BaseRealSenseNode_v2) : time camera_os_system_time_ms.coords → punit := 
  λ camera_time_base,
  let b0 : BaseRealSenseNode_v2 := {
    ros_time_base := ros_time_in_seconds.time now,
    ..b 
  } in
  let b1 : BaseRealSenseNode_v2 := {
    camera_time_base := camera_time_base,
    ..b0
  } in 
  punit.star 

def BaseRealSenseNode_v2.FillImuData_Copy (b : BaseRealSenseNode_v2) 
  : 
  timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) → 
  list (timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords)) → punit := 
  λimu_data, λ imu_msgs, 
  let imu_msgs0 := imu_msgs ++ [imu_data] in 
  punit.star

def BaseRealSenseNode_v2.ImuMessage_AddDefaultValues (b : BaseRealSenseNode_v2) 
  : 
  timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) → punit := 
  λimu_msg, 
  punit.star


def BaseRealSenseNode_v2.imu_callback_sync_v2 (b : BaseRealSenseNode_v2) : timestamped camera_global_time_ms.coords (displacement3d imu.coords) → punit :=

  λ dataframe : timestamped camera_global_time_ms.coords (displacement3d imu.coords),


  let dataframe_time := dataframe.timestamp in 
  let setBaseTimeCall := b.setBaseTime (camera_os_system_time_ms.time first_send_time) in
  let elapsed_camera_ms : duration camera_os_system_time_seconds.coords  
    := ((camera_global_time_ms.coords).mk_time_transform_to camera_os_system_time_seconds.coords).transform_duration
    (((dataframe_time -ᵥ b.camera_time_base) : duration camera_global_time_ms.coords)) in
  let if0 := 
    if true then 
      let crnt_reading : displacement3d imu.coords := dataframe.value in
      let v : displacement3d imu.coords := mk_displacement3d imu.coords crnt_reading.x crnt_reading.y crnt_reading.z in

      let imu_data : timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) := ⟨elapsed_camera_ms, v⟩ in
      let imu_msgs : list (timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords)) := [] in
      let FillImuData_CopyCall := b.FillImuData_Copy imu_data imu_msgs in 

      let while0 := 
        if imu_msgs.length > 0 then 
          let imu_msg : timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) := imu_msgs.head in
          let t : time camera_os_system_time_seconds.coords :=
            mk_time _
            ((
              let _ros_time_base_toSec : time ros_time_in_seconds.coords → scalar :=
              λt,
                (t).coord in
              _ros_time_base_toSec b.ros_time_base
            )
            +
            (
              let _imu_msg_timestamp_toSec : time camera_os_system_time_seconds.coords → scalar :=
                λt,
                  t.coord in
              _imu_msg_timestamp_toSec imu_msg.timestamp--imu_time_as_duration
            )) in 
          let imu_msg0 : timestamped camera_os_system_time_seconds.coords (displacement3d imu.coords) := {
            timestamp := t,
            ..imu_msg
          } in
          let ImuMessage_AddDefaultValuesCall := b.ImuMessage_AddDefaultValues imu_msg in 
          let PublishCall := b.synced_imu_publisher.Publish imu_msg in 
          punit.star 
        else 
          punit.star in 
        punit.star
      else 
        punit.star in
    punit.star
    
