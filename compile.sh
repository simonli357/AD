#!/bin/bash

catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && ln -s build/compile_commands.json compile_commands.json
