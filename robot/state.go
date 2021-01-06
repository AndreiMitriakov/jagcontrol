package robot

import (
	"fmt"
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
const MAX = 400
const AngArmRes = 5700.0 / (2 * PI)
const AngFlipperRes = 1140.0 / (2 * PI)

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
	s.linear, s.angular, s.front, s.rear, s.arm1, s.arm2 = prms[0], prms[1], prms[2], prms[3], prms[4], prms[5]

	s.limits = map[string]map[string]float32{
		"wLeft": map[string]float32{"min": -400, "max": 400},
		"wRight": map[string]float32{"min": -400, "max": 400},
		"front": map[string]float32{"min": -PI/4, "max": PI/4},
		"rear": map[string]float32{"min": -PI/4, "max": PI/4},
		"arm1": map[string]float32{"min": -PI/4, "max": PI/4},
		"arm2": map[string]float32{"min": -PI/4, "max": PI/4},
	}
}

func (s *State) setVelocity(lin, ang float32) (int32, int32) {
	// calculation of corresponding left and right motor commands
	var wLeft, wRight float32
	var D, R float32  = 0.6, 0.085
	wLeft = (lin + ang * D / 2) / R / (2*PI) * MAX
	wRight = (lin - ang * D / 2) / R / (2*PI) * MAX
	// fmt.Println("req", lin, ang)
	// fmt.Println("w wished", wLeft, wRight)
	if wLeft > s.limits["wLeft"]["max"] {
		wLeft = s.limits["wLeft"]["max"]
	} else if wLeft < s.limits["wLeft"]["min"] {
		wLeft = s.limits["wLeft"]["min"]
	}
	if wRight > s.limits["wRight"]["max"] {
		wRight = s.limits["wRight"]["max"]
	} else if wRight < s.limits["wRight"]["min"] {
		wRight = s.limits["wRight"]["min"]
	}
	// fmt.Println("w act", wLeft, wRight)
	s.linear = (wLeft + wRight) * R / 2 * (2*PI) / MAX
	s.angular = (wLeft - wRight) * R / D * (2*PI) / MAX
	// fmt.Println("cur", s.linear, s.angular, "\n")
	return int32(wLeft), int32(wRight)
}

func (s *State) getVelocity() (float32, float32) {
	return s.linear, s.angular
}

func (s *State) incFlipper(fD, rD float32) (int32, int32) {
	var fStart, rStart = s.front, s.rear
	var fChange, rChange float32
	if fStart + fD > s.limits["front"]["max"] {
		s.front = s.limits["front"]["max"]
	} else if fStart + fD < s.limits["front"]["min"] {
		s.front = s.limits["front"]["min"]
	} else {
		s.front = fStart + fD
	}

	fChange = s.front - fStart
	if rStart + rD > s.limits["rear"]["max"] {
		s.rear = s.limits["rear"]["max"]
	} else if rStart + rD < s.limits["rear"]["min"] {
		s.rear = s.limits["rear"]["min"]
	} else {
		s.rear = rStart + rD
	}
	rChange = s.rear - rStart
	return int32(fChange * AngFlipperRes), int32(rChange * AngFlipperRes)
}

func (s *State) getFlipper() (float32, float32) {
	return s.front, s.rear
}

func (s *State) incArm(arm1D, arm2D float32) (int32, int32) {
	var arm1Start, arm2Start = s.arm1, s.arm2
	var arm1Change, arm2Change float32
	if arm1Start + arm1D > s.limits["arm1"]["max"] {
		s.arm1 = s.limits["arm1"]["max"]
	} else if arm1Start + arm1D < s.limits["arm1"]["min"] {
		s.arm1 = s.limits["arm1"]["min"]
	} else {
		s.arm1 = arm1Start + arm1D
	}
	arm1Change = s.arm1 - arm1Start
	if arm2Start + arm2D > s.limits["arm2"]["max"] {
		s.arm2 = s.limits["arm2"]["max"]
	} else if arm2Start + arm2D < s.limits["arm2"]["min"] {
		s.arm2 = s.limits["arm2"]["min"]
	} else {
		s.arm2 = arm2Start + arm2D
	}
	arm2Change = s.arm2 - arm2Start
	return int32(arm1Change * AngArmRes), int32(arm2Change * AngArmRes)
}

func (s *State) getArm() (float32, float32) {
	return s.arm1, s.arm2
}

func (s *State) StringRepr() string{
	return fmt.Sprintf(
		"Robot state:\n" +
		"Vel: %f %f\n" +
		"Flp: %f, %f\n" +
		"Arm: %f %f\n", s.linear, s.angular, s.front, s.rear, s.arm1, s.arm2)
}