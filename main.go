package main

import (
	"flag"
	"github.com/AndreiMitriakov/jagcontrol/robot"
	"log"
)

func main(){
	boolTest := flag.Bool("test", false, "working in test mode?")
	flag.Parse()
	log.Printf("Working in real mode [%t]\n", !*boolTest)

	jaguar := robot.Robot{}
	jaguar.Init(*boolTest, 0., 0., 0., 0., 0., 0.)
	defer jaguar.Close()
	done := make(chan bool)
	go jaguar.Keyboard(done)
	go jaguar.RPC(done)
	go jaguar.Sensors()
	go jaguar.Security()
	<-done
}
