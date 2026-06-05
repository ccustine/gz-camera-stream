#!/bin/bash
#
# Autonomous patrol flight for the X3 UAV in quadcopter_demo.sdf.
# Sends Twist velocity commands to fly a rectangular patrol pattern.
#
# Usage: ./fly_patrol.sh

TOPIC="/X3/gazebo/command/twist"

send() {
  gz topic -t "$TOPIC" -m gz.msgs.Twist -p "$1"
}

echo "Starting patrol sequence..."

echo "[1/8] Taking off..."
send "linear: {x: 0, y: 0, z: 0.8}"
sleep 6

echo "[2/8] Hovering..."
send " "
sleep 2

echo "[3/8] Flying north..."
send "linear: {x: 2.0, y: 0, z: 0}"
sleep 8

echo "[4/8] Turning east..."
send "linear: {x: 0, y: 0, z: 0} angular: {z: -0.8}"
sleep 2
send "linear: {x: 2.0, y: 0, z: 0}"
sleep 8

echo "[5/8] Turning south..."
send "linear: {x: 0, y: 0, z: 0} angular: {z: -0.8}"
sleep 2
send "linear: {x: 2.0, y: 0, z: 0}"
sleep 8

echo "[6/8] Turning west (heading home)..."
send "linear: {x: 0, y: 0, z: 0} angular: {z: -0.8}"
sleep 2
send "linear: {x: 2.0, y: 0, z: 0}"
sleep 8

echo "[7/8] Hovering over landing pad..."
send " "
sleep 3

echo "[8/8] Landing..."
send "linear: {x: 0, y: 0, z: -0.3}"
sleep 8

echo "Patrol complete. Motors off."
send " "
