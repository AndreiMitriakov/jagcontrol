package main

import (
	"fmt"
	"github.com/AndreiMitriakov/jagcontrol/robot"
)

func main(){
	r := robot.State{}
	fmt.Println(r.Print())
}
