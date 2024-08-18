#!/bin/zsh
make && openocd -f ./openocd.cfg -c "program build/Prac1.elf verify reset exit"
