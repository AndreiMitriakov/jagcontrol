package main

import (
	"bufio"
	"fmt"
	"io/ioutil"
	"strconv"
	"strings"
)

func main(){
    /*conn, err := net.Dial("tcp", "192.168.0.60:10001")
    defer conn.Close()
    if err != nil {
        panic("No base connection")
    }*/
	dat, _ := ioutil.ReadFile("data")
	scanner := bufio.NewScanner(strings.NewReader(string(dat)))
	//var s string
	s := ""
	sa := []string{}
	ia := []byte{}
	for scanner.Scan() {
		ia = []byte{}
		s = scanner.Text()
		s = strings.TrimLeft(s, "[")
		s = strings.TrimRight(s, "]")
		sa = strings.Split(s, " ")
		for _, el := range sa {
			c, _ := strconv.Atoi(el)
			ia = append(ia, byte(c))
		}
		fmt.Println(string(ia))
	}
	// check mainwindow.cpp at line 149
}