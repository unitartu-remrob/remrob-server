#!/bin/bash

git clone --recursive https://github.com/unitartu-remrob/remrob-docker
docker build -t remrob:base ./remrob-docker