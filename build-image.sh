#!/bin/bash

git clone --recursive https://github.com/unitartu-remrob/remrob-docker
docker build -t robotont:base ./remrob-docker