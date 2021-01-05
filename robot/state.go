package robot

import "fmt"

type State struct {
	front int32
	rear int32
	linear int32
	angular int32
	arm1 int32
	arm2 int32
}

func (p *State) Print() string{
	return fmt.Sprintf("Robot state:\n" +
		"Flippers: %d %d\n" +
		"Velocities: %d, %d\n" +
		"Arm: %d %d", p.front, p.rear, p.linear, p.angular, p.arm1, p.arm2)
}

