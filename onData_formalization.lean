import .phys.time.time
import .phys.time_series.geom3d
import .std.time_std
import .std.geom3d_std
import data.real.basic

noncomputable theory

/-
PREAMBLE

We're missing a lot of fundamental structure in Phys required to even passably model this. 

-/

-- TODO: Should come from some library
def milliseconds := (0.001)           -- names not clear, inverted?
def milliseconds_per_second := 1000   -- names not clear, inverted?
def seconds := 1      

def millimeters := (0.001)           
def millimeters_to_meters := 1000   
def meters := 1                      


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

namespace roomba_time_in_seconds  
  axiom δ : scalar 
  axiom ε : scalar
  def origin := mk_time utc.coords δ 
  def basis := mk_duration utc.coords (seconds*ε)  
  -- camera_hardware_time.duration milliseconds_to_seconds
  -- closer inspection here to make sure Kevin didn't botch anything 
  def frame := mk_time_frame origin basis
  def coords := mk_time_space frame
  def time (t : K) := mk_time coords t 
end roomba_time_in_seconds


namespace ros_time_in_seconds  
  axiom δ : scalar 
  axiom ε : scalar
  def origin := mk_time utc.coords δ 
  def basis := mk_duration utc.coords (seconds*ε)  
  -- camera_hardware_time.duration milliseconds_to_seconds
  -- closer inspection here to make sure Kevin didn't botch anything 
  def frame := mk_time_frame origin basis
  def coords := mk_time_space frame
  def time (t : K) := mk_time coords t 
end ros_time_in_seconds

-- Geometric world
namespace world  -- it's generic/parametric: for example, world -> Rice 440, as follows  
def origin := std.position 0 0 0      -- looking in from doorway, the back lower left corner  
def basis_0 := std.displacement meters 0 0 -- right/east along wall; unit is 1m; right
def basis_1 := std.displacement 0 meters 0 -- to door along weset wall; 1m; right
def basis_2 := std.displacement 0 0 meters -- up along NW corner; 1m; right handed
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : K) := mk_position3d coords x y z
def displacement (x y z : K) := mk_displacement3d coords x y z
end world


-- Base link frame of the Roomba, expressed in terms of the world frame, with millimeters as units
namespace roomba_mm
def origin := std.position 2 2 0      
def basis_0 := std.displacement millimeters 0 0 
def basis_1 := std.displacement 0 millimeters 0 
def basis_2 := std.displacement 0 0 millimeters 
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : K) := mk_position3d coords x y z
def displacement (x y z : K) := mk_displacement3d coords x y z
end roomba_mm

-- Base link frame of the Roomba, expressed in terms of the world frame, with meters as units
namespace roomba_meters
def origin := std.position 2 2 0      
def basis_0 := std.displacement 1 0 0 
def basis_1 := std.displacement 0 1 0 
def basis_2 := std.displacement 0 0 1 
def frame := mk_geom3d_frame origin basis_0 basis_1 basis_2
def coords := mk_geom3d_space frame
def position (x y z : K) := mk_position3d coords x y z
def displacement (x y z : K) := mk_displacement3d coords x y z
end roomba_meters


/-
This is a 2-D pose. This is a small fix - we do not have a 2d Geometry implementation - but we cannot model this. 
-/
structure Pose := 
  (x : scalar)
  (y : scalar)
  (yaw : scalar)

/-
This is a 2-D twist. We cannot model this. We have not implemented derived spaces.
-/
structure Vel := 
  (x : scalar)
  (y : scalar)
  (yaw : scalar)


structure Create := 
  (firstOnData : bool)
  (prevOnData : time ros_time_in_seconds.coords)
  (prevOnDataTime : time utc.coords)
  (prevTicksLeft : scalar)
  (prevTicksRight : scalar)
  (measuredLeftVel : scalar)
  (measuredRightVel : scalar)
  (totalLeftDist : scalar)
  (totalRightDist : scalar)
  (pose : Pose)
  (vel : Vel)

axiom then_ : scalar 
axiom now : scalar

axiom roomba_delta : scalar
axiom roomba_angle : scalar

axiom model_axel_length : scalar 

axiom prev_ticks_left : scalar 
axiom prev_ticks_right : scalar 

axiom total_ticks_left : scalar 
axiom total_ticks_right : scalar 

axiom V_3_MAX_ENCODER_TICKS : scalar 

axiom V_3_TICKS_PER_REV : scalar

axiom EPS : scalar

def Create.onData (c : Create) :=
  /-
   if (firstOnData) {
      if (model.getVersion() >= V_3) {
        // Initialize tick counts
        prevTicksLeft = GET_DATA(ID_LEFT_ENC);
        prevTicksRight = GET_DATA(ID_RIGHT_ENC);
      }
      prevOnDataTime = std::chrono::steady_clock::now();
      firstOnData = false;
    }
  -/
  let if0 : punit := 
    if c.firstOnData then 
      /-

      if (model.getVersion() >= V_3) {
        // Initialize tick counts
        prevTicksLeft = GET_DATA(ID_LEFT_ENC);
        prevTicksRight = GET_DATA(ID_RIGHT_ENC);
      }
      These refer to ticks of a wheel encoder. A fixed amount of these implies a revolution of the roomba wheel.

      -/
      let if1 :=
        if true then 
          let c0 : Create := {
            prevTicksLeft := prev_ticks_left,
            prevTicksRight := prev_ticks_right,
            ..c
          } in 
          punit.star
        else 
          punit.star in 



      /-

      prevOnDataTime = std::chrono::steady_clock::now();

      https://stackoverflow.com/questions/52421819/does-steady-clocknow-return-seconds

      Unit of value returned by std::chrono::steady_clock::now() is not defined by standard

      So, this does not have any units attached to it. We interpret it as being in UTC, in seconds, nonetheless
      -/
      let c0 := {
        prevOnDataTime := utc.time then_,
        ..c
      } in 

      let c1 := {
        firstOnData := false,
        ..c0
      } in 
      punit.star
    else punit.star in
  /-
    auto curTime = std::chrono::steady_clock::now();
    float dt = static_cast<std::chrono::duration<float>>(curTime - prevOnDataTime).count();

    We take the current time. Again, no units by default, but we interpret it in terms of seconds. 

  -/
  let curTime : time utc.coords := utc.time now in 
  let dt : duration utc.coords := curTime -ᵥ c.prevOnDataTime in 
  /-
  if (model.getVersion() <= V_2) {
      // This is a standards compliant way of doing unsigned to signed conversion
      uint16_t distanceRaw = GET_DATA(ID_DISTANCE);
      int16_t distance;
      std::memcpy(&distance, &distanceRaw, sizeof(distance));
      deltaDist = distance / 1000.0; // mm -> m

      // Angle is processed differently in versions 1 and 2
      uint16_t angleRaw = GET_DATA(ID_ANGLE);
      std::memcpy(&angleField, &angleRaw, sizeof(angleField));
    }
  -/
  /-
    float deltaDist, deltaX, deltaY, deltaYaw, leftWheelDist, rightWheelDist, wheelDistDiff; 

    Relevant variables used in calculation

    deltaDist is a distance. We can't interpret that accurately.
    deltaX is a 1D displacement in the X-direction of a 2-dimensional space. We can't interpret that accurately.
    deltaY is a 1D displacement in the Y-direction of a 2-dimensional space. We can't interpret that accurately.
    
    deltaYaw is an angle representing a rotation along a normal to the X-Y axis
    
    leftWheelDist and rightWheelDist 1D displacements?
  -/
  let deltaDist : scalar := inhabited.default _ in
  let deltaX : scalar := inhabited.default _ in
  let deltaY : scalar := inhabited.default _ in
  let deltaYaw : scalar := inhabited.default _ in
  let leftWheelDist : scalar := inhabited.default _ in
  let rightWheelDist : scalar := inhabited.default _ in
  let wheelDistDiff : scalar := inhabited.default _ in

  /-
    int16_t angleField;
  -/
  let angleField : scalar := inhabited.default _ in 

  /-
    if (model.getVersion() <= V_2) {
  -/
  let if0 : punit := 
    if true then  
      /-
      
      uint16_t distanceRaw = GET_DATA(ID_DISTANCE);
      -/
      let distanceRaw := roomba_mm.displacement roomba_delta in 
      let distance : displacement3d roomba_meters.coords := inhabited.default _ in 
      let deltaDist0 := roomba_mm.coords.mk_geom3d_transform_to roomba_meters.coords in

      /-
      uint16_t angleRaw = GET_DATA(ID_ANGLE);
      -/
      let angleRaw := roomba_angle in 
      /-
      std::memcpy(&angleField, &angleRaw, sizeof(angleField));
      -/
      let angleField0 : scalar := angleRaw in 

      punit.star
    else 
      punit.star in
  /-
    if (model.getVersion() == V_1) {
      wheelDistDiff = 2.0 * angleField / 1000.0;
      leftWheelDist = deltaDist - (wheelDistDiff / 2.0);
      rightWheelDist = deltaDist + (wheelDistDiff / 2.0);
      deltaYaw = wheelDistDiff / model.getAxleLength();
  -/
  let if0 : punit :=
    if true then 
      let wheelDistDiff := (2*angleField)/1000 in 
      let leftWheelDist := deltaDist - (wheelDistDiff/2) in  
      let rightWheelDist := deltaDist + (wheelDistDiff/2) in 
      let deltaYaw := wheelDistDiff / model_axel_length in 
      punit.star 
    else if true then 
      /-
      } else if (model.getVersion() == V_2) {
      deltaYaw = angleField * (util::PI / 180.0); // D2R
      wheelDistDiff = model.getAxleLength() * deltaYaw;
      leftWheelDist = deltaDist - (wheelDistDiff / 2.0);
      rightWheelDist = deltaDist + (wheelDistDiff / 2.0);
      -/
      let deltaYaw0 := angleField * (real.pi/180) in 
      let wheelDistDiff0 := model_axel_length * deltaYaw in 
      let leftWheelDist0 := deltaDist - (wheelDistDiff / 2) in 
      let rightWheelDist0 := deltaDist + (wheelDistDiff / 2) in 


      punit.star 
    /-
    
      else if (model.getVersion() >= V_3) {
      // Get cumulative ticks (wraps around at 65535)
      uint16_t totalTicksLeft = GET_DATA(ID_LEFT_ENC);
      uint16_t totalTicksRight = GET_DATA(ID_RIGHT_ENC);
    -/
    else if true then 
      let totalTicksLeft : scalar := total_ticks_left in 
      let totalTicksRight : scalar := total_ticks_right in 

/-
      int ticksLeft = totalTicksLeft - prevTicksLeft;
      int ticksRight = totalTicksRight - prevTicksRight;
-/
      let ticksLeft : scalar := totalTicksLeft - c.prevTicksLeft in 
      let ticksRight : scalar := totalTicksRight - c.prevTicksRight in 

/-
  prevTicksLeft = totalTicksLeft;
  prevTicksRight = totalTicksRight;
-/
      let c0 : Create := {
        prevTicksLeft := totalTicksLeft, 
        prevTicksRight := totalTicksRight,
        ..c
      } in 
  /-
  // Handle wrap around
      if (fabs(ticksLeft) > 0.9 * util::V_3_MAX_ENCODER_TICKS) {
        ticksLeft = (ticksLeft % util::V_3_MAX_ENCODER_TICKS) + 1;
      }
      if (fabs(ticksRight) > 0.9 * util::V_3_MAX_ENCODER_TICKS) {
        ticksRight = (ticksRight % util::V_3_MAX_ENCODER_TICKS) + 1;
      }
  -/
      let if1 : punit := 
        if (abs ticksLeft) > 0.9 * V_3_MAX_ENCODER_TICKS then
          let ticksLeft0 := (ticksLeft % V_3_MAX_ENCODER_TICKS) in  
          punit.star 
        else punit.star in 
      let if1 : punit := 
        if (abs ticksRight) > 0.9 * V_3_MAX_ENCODER_TICKS then
          let ticksRight0 := (ticksRight % V_3_MAX_ENCODER_TICKS) in  
          punit.star 
        else punit.star in 

      /-
      // Compute distance travelled by each wheel
      leftWheelDist = (ticksLeft / util::V_3_TICKS_PER_REV)
          * model.getWheelDiameter() * util::PI;
      rightWheelDist = (ticksRight / util::V_3_TICKS_PER_REV)
          * model.getWheelDiameter() * util::PI;
      deltaDist = (rightWheelDist + leftWheelDist) / 2.0;

      wheelDistDiff = rightWheelDist - leftWheelDist;
      deltaYaw = wheelDistDiff / model.getAxleLength();
      -/

      let leftWheelDist0 := ticksLeft/V_3_TICKS_PER_REV in 
      let rightWheelDist0 := ticksRight/V_3_TICKS_PER_REV in 
      let deltaDist := (rightWheelDist0 + leftWheelDist0)/2 in 

      let wheelDistDiff0 := rightWheelDist - leftWheelDist in 
      let deltaYaw0 := wheelDistDiff0 / model_axel_length in 
      punit.star 
    else 
      punit.star in 
  
  /-
  
  measuredLeftVel = leftWheelDist / dt;
    measuredRightVel = rightWheelDist / dt;
  
  -/
  let c0 : Create := {
    measuredLeftVel := leftWheelDist/dt, 
    measuredRightVel := rightWheelDist/dt,
    ..c
  } in

  /-
  
    // Moving straight
    if (fabs(wheelDistDiff) < util::EPS) {
      deltaX = deltaDist * cos(pose.yaw);
      deltaY = deltaDist * sin(pose.yaw);
    } else {
      float turnRadius = (model.getAxleLength() / 2.0) * (leftWheelDist + rightWheelDist) / wheelDistDiff;
      deltaX = turnRadius * (sin(pose.yaw + deltaYaw) - sin(pose.yaw));
      deltaY = -turnRadius * (cos(pose.yaw + deltaYaw) - cos(pose.yaw));
    }

  -/

  let if0 : punit := 
    if abs wheelDistDiff < EPS then
      let deltaX0 := deltaDist * (real.cos c.pose.yaw) in 
      let deltaY0 := deltaDist * (real.sin c.pose.yaw) in 
      punit.star 
    else 
      let turnRadius := (model_axel_length / 2) * (leftWheelDist + rightWheelDist) / wheelDistDiff in 
      let deltaX0 := turnRadius * (real.sin (c.pose.yaw + deltaYaw)) - (real.sin(c.pose.yaw)) in 
      let deltaY0 := -turnRadius * (real.cos (c.pose.yaw + deltaYaw)) - (real.cos(c.pose.yaw)) in 
      punit.star in 
/-

    totalLeftDist += leftWheelDist;
    totalRightDist += rightWheelDist
  
-/
  let c1 : Create := {
    totalLeftDist := c0.totalLeftDist + leftWheelDist, 
    totalRightDist := c0.totalRightDist + rightWheelDist,
    ..c0
  } in 
  
  /-

    if (fabs(dt) > util::EPS) {
      vel.x = deltaDist / dt;
      vel.y = 0.0;
      vel.yaw = deltaYaw / dt;
    } else {
      vel.x = 0.0;
      vel.y = 0.0;
      vel.yaw = 0.0;
    }

    // Update covaria
  -/

  let if0 : punit := 
    if (abs dt > EPS) then 

      let c2 : Create := {
        vel := ⟨deltaDist/dt, 0, deltaYaw/dt⟩,
        ..c1
      } in 

      punit.star 

    else 
      let c2 : Create := {
        vel := ⟨0, 0, 0⟩,
        ..c1
      } in 

      punit.star in 

  punit.star


  /-
   The below code is irrelevant and will be an arduous task to formalize.

    // Update covariances
    // Ref: "Introduction to Autonomous Mobile Robots" (Siegwart 2004, page 189)
    float kr = 1.0; // TODO: Perform experiments to find these nondeterministic parameters
    float kl = 1.0;
    float cosYawAndHalfDelta = cos(pose.yaw + (deltaYaw / 2.0)); // deltaX?
    float sinYawAndHalfDelta = sin(pose.yaw + (deltaYaw / 2.0)); // deltaY?
    float distOverTwoWB = deltaDist / (model.getAxleLength() * 2.0);

    Matrix invCovar(2, 2);
    invCovar(0, 0) = kr * fabs(rightWheelDist);
    invCovar(0, 1) = 0.0;
    invCovar(1, 0) = 0.0;
    invCovar(1, 1) = kl * fabs(leftWheelDist);

    Matrix Finc(3, 2);
    Finc(0, 0) = (cosYawAndHalfDelta / 2.0) - (distOverTwoWB * sinYawAndHalfDelta);
    Finc(0, 1) = (cosYawAndHalfDelta / 2.0) + (distOverTwoWB * sinYawAndHalfDelta);
    Finc(1, 0) = (sinYawAndHalfDelta / 2.0) + (distOverTwoWB * cosYawAndHalfDelta);
    Finc(1, 1) = (sinYawAndHalfDelta / 2.0) - (distOverTwoWB * cosYawAndHalfDelta);
    Finc(2, 0) = (1.0 / model.getAxleLength());
    Finc(2, 1) = (-1.0 / model.getAxleLength());
    Matrix FincT = boost::numeric::ublas::trans(Finc);

    Matrix Fp(3, 3);
    Fp(0, 0) = 1.0;
    Fp(0, 1) = 0.0;
    Fp(0, 2) = (-deltaDist) * sinYawAndHalfDelta;
    Fp(1, 0) = 0.0;
    Fp(1, 1) = 1.0;
    Fp(1, 2) = deltaDist * cosYawAndHalfDelta;
    Fp(2, 0) = 0.0;
    Fp(2, 1) = 0.0;
    Fp(2, 2) = 1.0;
    Matrix FpT = boost::numeric::ublas::trans(Fp);

    Matrix velCovar = ublas::prod(invCovar, FincT);
    velCovar = ublas::prod(Finc, velCovar);

    vel.covariance[0] = velCovar(0, 0);
    vel.covariance[1] = velCovar(0, 1);
    vel.covariance[2] = velCovar(0, 2);
    vel.covariance[3] = velCovar(1, 0);
    vel.covariance[4] = velCovar(1, 1);
    vel.covariance[5] = velCovar(1, 2);
    vel.covariance[6] = velCovar(2, 0);
    vel.covariance[7] = velCovar(2, 1);
    vel.covariance[8] = velCovar(2, 2);

    Matrix poseCovarTmp = ublas::prod(poseCovar, FpT);
    poseCovarTmp = ublas::prod(Fp, poseCovarTmp);
    poseCovar = addMatrices(poseCovarTmp, velCovar);

    pose.covariance[0] = poseCovar(0, 0);
    pose.covariance[1] = poseCovar(0, 1);
    pose.covariance[2] = poseCovar(0, 2);
    pose.covariance[3] = poseCovar(1, 0);
    pose.covariance[4] = poseCovar(1, 1);
    pose.covariance[5] = poseCovar(1, 2);
    pose.covariance[6] = poseCovar(2, 0);
    pose.covariance[7] = poseCovar(2, 1);
    pose.covariance[8] = poseCovar(2, 2);

    // Update pose
    pose.x += deltaX;
    pose.y += deltaY;
    pose.yaw = util::normalizeAngle(pose.yaw + deltaYaw);

    prevOnDataTime = curTime;

    // Make user registered callbacks, if any
    // TODO
  }

  -/