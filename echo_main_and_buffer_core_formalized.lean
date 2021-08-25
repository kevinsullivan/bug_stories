import .phys.time.time
import .phys.time_series.geom3d
import .std.time_std
import .std.geom3d_std
noncomputable theory

/-
We need to assume a physical interpretation of the data
representing our coordinate system on time. We derive a new ACS from 
"coordinated_universal_time_in_seconds" - see time_std.lean 
for a more details on the coordinate system and physical interpretation
(note : this is a conventional UTC ACS expressed with units in seconds)

The gist of this interpretation is that we need an ACS whose units are seconds -
for this example, we define a space centered at "now" - 
August 18th, 2:40 PM, 


(1) ORIGIN: We move the origin up to 1629311979

(2) BASIS VECTORS
    basis0 
      - points to the future
      - unit length is 1 second (as in UTC)
(3) ACS is given by [Origin, b0]
-/

-- TODO: Should come from some library
def nanoseconds := (0.000000001)           -- names not clear, inverted?
def nanoseconds_per_second := 1000000000   -- names not clear, inverted?
def seconds := 1                      -- think about this more

-- TODO: Should come from resp. std libraries and be distributed to them accordingly
namespace std
def time (p : K) : time time_std_space := mk_time time_std_space p
def duration (d : K) : duration time_std_space := mk_duration _ d
def position (x y z : K) : position3d geom3d_std_space := mk_position3d _ x y z
def displacement (x y z : K) : displacement3d geom3d_std_space := mk_displacement3d _ x y z
end std

namespace utc
def origin := std.time 0   -- origin; first instant of January 1, 1970
def basis := std.duration 1    -- basis; "second;" the smallest non-variable unit in UTC
def frame := mk_time_frame origin basis -- recall why "time" is part of the constructor name? factor out?
def coords := mk_space frame  -- "cosys"?
def time (t : K) := mk_time coords t
def duration (d : K) := mk_duration coords d
end utc

/-

We define a second time ACS which is intended to represent UTC, at the same
time origin as our prior ACS, but whose units represent nanoseconds.

(1) ORIGIN: We do not shift the origin - it remains at 0

(2) BASIS VECTORS
    basis0 
      - points to the future
      - unit length is 1 nanosecond
(3) ACS is given by [Origin, b0]
-/

namespace utc_ns
def origin := utc.time 0   -- origin; first instant of January 1, 1970
def basis := utc.duration nanoseconds    -- basis; "second;" the smallest non-variable unit in UTC
def frame := mk_time_frame origin basis -- recall why "time" is part of the constructor name? factor out?
def coords := mk_space frame  -- "cosys"?
def time (t : K) := mk_time coords t
def duration (d : K) := mk_duration coords d
end utc_ns


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




/-


TEST(BufferCore_lookupTransform, i_configuration)
{
  double epsilon = 1e-6;
  


  rostest::Permuter permuter;

  std::vector<builtin_interfaces::msg::Time> times;
  times.push_back(builtin_interfaces::msg::Time(1.0));
  times.push_back(builtin_interfaces::msg::Time(10.0));
  times.push_back(builtin_interfaces::msg::Time(0.0));
  builtin_interfaces::msg::Time eval_time;
  permuter.addOptionSet(times, &eval_time);

  std::vector<tf2::Duration> durations;
  durations.push_back(tf2::Duration(1.0));
  durations.push_back(tf2::Duration(0.001));
  durations.push_back(tf2::Duration(0.1));

-/

/-
We provide a static interpretation for constructors builtin_interfaces::msg::Time and 
tf2::Duration. We define the constructor of the former as a function from a scalar to a 
time expressed in the utc.coords ACS. The latter, we define as a function from a scalar
to a duration in utc_ns.coords
-/
def msgTime : scalar → time utc.coords := λs, (inhabited.default _)
def tf2Duration : scalar → duration utc_ns.coords := λs, (inhabited.default _)

/-

We define a method "test" for the test case. In the body, firstly, a scalar is defined. Next, 
a vector of times are defined, several times are then constructed and inserted into the list. 

Next, a vector of durations is defined. Several durations are constructed and then added to the list. 
We notice that several errors occur as the items are added to the list. The tf2Duration constructor returns
a "duration utc_ns.coords", but the type of the durations vector is a "list (duration utc.coords)"

TEST(BufferCore_lookupTransform, i_configuration)
{
-/
def test : punit := 
  --double epsilon = 1e-6;
  let epsilon : scalar := 0.000001 in
  /-
  std::vector<builtin_interfaces::msg::Time> times;
  times.push_back(builtin_interfaces::msg::Time(1.0));
  times.push_back(builtin_interfaces::msg::Time(10.0));
  times.push_back(builtin_interfaces::msg::Time(0.0));
  -/
  let times : list (time utc.coords) := [] in
  let times0 : list (time utc.coords) := times ++ [msgTime 1] in
  let times1 : list (time utc.coords) := times0 ++ [msgTime 10] in 
  let times2 : list (time utc.coords) := times1 ++ [msgTime 0] in

  /-
  std::vector<tf2::Duration> durations;
  durations.push_back(tf2::Duration(1.0));
  durations.push_back(tf2::Duration(0.001));
  durations.push_back(tf2::Duration(0.1));
  -/
  let durations : list (duration utc.coords) := (([]:_)) in
  let durations0 : list (duration utc.coords) := durations ++ [tf2Duration 1] in
  let durations1 : list (duration utc.coords) := durations ++ [tf2Duration 0.001] in
  let durations2 : list (duration utc.coords) := durations ++ [tf2Duration 0.1] in
  punit.star

/-
We define fixed interpretations for the constructor of tf2 Time Point
and the member function "canTransform" of the tf2 Buffer

Firstly, we interpret tf2TimePoint as a function that accepts a scalar
and returns a time expressed in "utc.coords"

Nextly, we interpret canTransform to be a function that accepts both a time expressed in 
"utc.coords" and a duration expressed in "utc.coords".

-/

def tf2TimePoint : scalar → time utc.coords := λs, inhabited.default _
def canTransform : time utc.coords → duration utc.coords → punit := 
  λt d, punit.star
/-
We provide an interpretation for the main method of the tf2 "echo" implementation.
When "canTransform" is called, we supply a constructed value of type tf2Duration,
whichs returns a value of type "duration utc_ns.coords", however,
the call to canTransform expects a parameter of type "duration utc.coords",
thus, an error occurs.

int main(int argc, char ** argv)
{
-/
def main : punit := 
  --double rate_hz;
  -- We have no way of representing hertz
  let rate_hz : scalar := 0 in 
/-
echoListener.buffer_.canTransform(source_frameid, target_frameid, tf2::TimePoint(), tf2::Duration(1.0));

  while(rclcpp::ok())
    {
      try
      {
        geometry_msgs::msg::TransformStamped echo_transform;
        echo_transform = echoListener.buffer_.lookupTransform(source_frameid, target_frameid, tf2::TimePoint());

        auto translation = echo_transform.transform.translation;
        auto rotation = echo_transform.transform.rotation;
-/
  let canTransformCall := canTransform (tf2TimePoint 0) (tf2Duration 0) in
  let echo_transform := world.coords.mk_geom3d_transform_to base_link.coords in
  let translation := echo_transform.translation in 
  let rotation := echo_transform.rotation in 
  punit.star

/-

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc < 3 || argc > 4)
  {
    return -1;
  }
  rclcpp::node::Node::SharedPtr nh = rclcpp::node::Node::make_shared("tf2_echo");

  double rate_hz;
  if (argc == 4)
    rate_hz = atof(argv[3]);
  else
    rate_hz = 1.0;
  rclcpp::rate::Rate rate(rate_hz);

  echoListener echoListener;
  std::string(argv[1]);
  std::string target_frameid = std::string(argv[2]);

  echoListener.buffer_.canTransform(source_frameid, target_frameid, tf2::TimePoint(), tf2::Duration(1.0));

  while(rclcpp::ok())
    {
      try
      {
        geometry_msgs::msg::TransformStamped echo_transform;
        echo_transform = echoListener.buffer_.lookupTransform(source_frameid, target_frameid, tf2::TimePoint());

        auto translation = echo_transform.transform.translation;
        auto rotation = echo_transform.transform.rotation;
      }
      catch(tf2::TransformException& ex)
      {
        
      }
      rate.sleep();
    }

  return 0;
};


-/