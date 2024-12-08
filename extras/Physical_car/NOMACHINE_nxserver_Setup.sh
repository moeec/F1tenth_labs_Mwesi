#!/bin/bash


sudo systemctl disable gdm3 --now

sleep 0.7

sudo /usr/NX/bin/nxserver --restart
