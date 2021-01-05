package main

import (
	"github.com/AndreiMitriakov/jagcontrol/robot"
)

func main(){
	r := robot.Robot{}
	r.Init(0, 0, 0, 0, 0, 0)

	r.Operate()

}
