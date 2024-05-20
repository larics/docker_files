#!/bin/bash

BOOST_VERSION="$1" 
BOOST_VERSION_=${BOOST_VERSION//./_}

docker build . -t openpose_img:${BOOST_VERSION} --build-arg BOOST_VERSION=${BOOST_VERSION} \
	--build-arg BOOST_VERSION_=${BOOST_VERSION_}
