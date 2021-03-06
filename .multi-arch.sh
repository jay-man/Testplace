#!/bin/sh

VERSION=$1

cat <<EOF >/tmp/multi.yml
image: chalmersrevere/opendlv-device-camera-opencv-multi:$VERSION
manifests:	
  - image: chalmersrevere/opendlv-device-camera-opencv-amd64:$VERSION
    platform:
      architecture: amd64
      os: linux
  - image: chalmersrevere/opendlv-device-camera-opencv-armhf:$VERSION
    platform:
      architecture: arm
      os: linux
  - image: chalmersrevere/opendlv-device-camera-opencv-aarch64:$VERSION
    platform:
      architecture: arm64
      os: linux
EOF
manifest-tool-linux-amd64 push from-spec /tmp/multi.yml
