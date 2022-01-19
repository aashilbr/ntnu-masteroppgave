#!/bin/bash

xhost +local:root

docker-compose up --build

xhost -local:root