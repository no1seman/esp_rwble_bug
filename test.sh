#!/bin/bash
while true
do
echo -e "Make http request:\n"
curl http://192.168.3.130/ps --connect-timeout 1 --max-time 1
echo -e "\n"
sleep .2
done