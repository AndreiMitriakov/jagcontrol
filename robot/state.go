package robot

import (
	"fmt"
	"errors"
)

/*
	The private class "State":
		i) maintains the robot state in in radians and conv. units;
		ii) allows to set velocity, flipper and arm angles in  conventional units and radians (for angles);
		iii) allows to get velocity, flipper and arm angles in  conventional units and radians (for angles);
		iv) provides conversion from conventional units and radians to inner motor representation and vice versa;
		v) validates, when we set values, if velocity, flipper and arm angles can be set.
*/

const PI float32 = 3.14
const AngArmRes float32 = 5700.0 / (2 * PI)
const AngFlipperRes float32 = 1140.0 / (2 * PI)

type State struct {
	front float32
	rear float32
	linear float32
	angular float32
	arm1 float32
	arm2 float32
	limits map[string]map[string]float32
}

func (s *State) init(prms []float32) {
	// 1140 counts per revolution
	s.linear, s.angular, s.front, s.rear, s.arm1, s.arm2 = prms[0], prms[1], prms[2], prms[3], prms[4], prms[5]
	s.limits["linear"]["min"] = -0.4
	s.limits["linear"]["max"] = 0.4
	s.limits["angular"]["min"] = -0.4
	s.limits["angular"]["max"] = 0.4
	s.limits["front"]["min"] = -PI/4
	s.limits["front"]["max"] = PI/4
	s.limits["rear"]["min"] = -PI/4
	s.limits["rear"]["max"] = PI/4
	s.limits["arm1"]["min"] = -PI/4
	s.limits["arm1"]["max"] = PI/4
	s.limits["arm2"]["min"] = -PI/4
	s.limits["arm2"]["max"] = PI/4
}

func (s *State) setVelocity(lin, ang float32) (int32, int32) {
	// linear velocity
	if lin > s.limits["linear"]["max"] {
		s.linear = s.limits["linear"]["max"]
	} else if lin < s.limits["linear"]["min"] {
		s.linear = s.limits["linear"]["min"]
	}
	if ang > s.limits["angular"]["max"] {
		s.linear = s.limits["angular"]["max"]
	} else if ang < s.limits["angular"]["min"] {
		s.linear = s.limits["angular"]["min"]
	}
	var left, right int32
	return left, right
}
func (s *State) getVelocity() (int32, int32, error) {}
func (s *State) setFlipper(front, rear int32) (int32, int32, error) {}
func (s *State) getFlipper() (int32, int32, error) {}
func (s *State) setArm(arm1, arm2 int32) (int32, int32, error) {}
func (s *State) getArm() (int32, int32, error) {}
func (s *State) validateVelocity(lin, ang int32) bool {}
func (s *State) validateFlipper(front, rear int32) bool {}
func (s *State) validateArm(arm1, arm2 int32) bool {}

func (s *State) Print() string{
	return fmt.Sprintf("Robot state:\n" +
		"Flippers: %d %d\n" +
		"Velocities: %d, %d\n" +
		"Arm: %d %d", p.front, p.rear, p.linear, p.angular, p.arm1, p.arm2)
}

