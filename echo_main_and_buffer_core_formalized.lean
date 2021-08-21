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

def August18thTwoFortyPMTimestamp : scalar := 1629311979

def current_time_in_UTC : time_space _ := 
  let origin := mk_time coordinated_universal_time_in_seconds 1629311979 in
  let basis := mk_duration coordinated_universal_time_in_seconds 1 in
  mk_time_space (mk_time_frame origin basis)

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

def current_time_in_UTC_nanoseconds : time_space _ := 
  let origin := mk_time current_time_in_UTC 0 in 
  let basis := mk_duration current_time_in_UTC 0.000000001 in
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
time expressed in the current_time_in_UTC ACS. The latter, we define as a function from a scalar
to a duration in current_time_in_UTC_nanoseconds
-/
def msgTime : scalar → time current_time_in_UTC := λs, (inhabited.default _)
def tf2Duration : scalar → duration current_time_in_UTC_nanoseconds := λs, (inhabited.default _)

/-

We define a method "test" for the test case. In the body, firstly, a scalar is defined. Next, 
a vector of times are defined, several times are then constructed and inserted into the list. 

Next, a vector of durations is defined. Several durations are constructed and then added to the list. 
We notice that several errors occur as the items are added to the list. The tf2Duration constructor returns
a "duration current_time_in_UTC_nanoseconds", but the type of the durations vector is a "list (duration current_time_in_UTC)"

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
  let times : list (time current_time_in_UTC) := [] in
  let times0 : list (time current_time_in_UTC) := times ++ [msgTime 1] in
  let times1 : list (time current_time_in_UTC) := times0 ++ [msgTime 10] in 
  let times2 : list (time current_time_in_UTC) := times1 ++ [msgTime 0] in

  /-
  std::vector<tf2::Duration> durations;
  durations.push_back(tf2::Duration(1.0));
  durations.push_back(tf2::Duration(0.001));
  durations.push_back(tf2::Duration(0.1));
  -/
  let durations : list (duration current_time_in_UTC) := (([]:_)) in
  let durations0 : list (duration current_time_in_UTC) := durations ++ [tf2Duration 1] in
  let durations1 : list (duration current_time_in_UTC) := durations ++ [tf2Duration 0.001] in
  let durations2 : list (duration current_time_in_UTC) := durations ++ [tf2Duration 0.1] in
  punit.star

/-
We define fixed interpretations for the constructor of tf2 Time Point
and the member function "canTransform" of the tf2 Buffer

Firstly, we interpret tf2TimePoint as a function that accepts a scalar
and returns a time expressed in "current_time_in_UTC"

Nextly, we interpret canTransform to be a function that accepts both a time expressed in 
"current_time_in_UTC" and a duration expressed in "current_time_in_UTC".

-/

def tf2TimePoint : scalar → time current_time_in_UTC := λs, inhabited.default _
def canTransform : time current_time_in_UTC → duration current_time_in_UTC → punit := 
  λt d, punit.star
/-
We provide an interpretation for the main method of the tf2 "echo" implementation.
When "canTransform" is called, we supply a constructed value of type tf2Duration,
whichs returns a value of type "duration current_time_in_UTC_nanoseconds", however,
the call to canTransform expects a parameter of type "duration current_time_in_UTC",
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
  let echo_transform := geometry3d_acs.mk_geom3d_transform_to base_link_acs in
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