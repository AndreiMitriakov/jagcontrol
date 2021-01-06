package main

import (
	"github.com/AndreiMitriakov/jagcontrol/robot"
)

func main(){
	jaguar := robot.Robot{}
	jaguar.Init(0., 0., 0., 0., 0., 0.)
	defer jaguar.Close()
	done := make(chan bool)
	go jaguar.Keyboard(done)
	go jaguar.RPC()
	<-done
}
