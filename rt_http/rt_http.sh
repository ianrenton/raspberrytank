#!/bin/bash

until ./rt_http; do
  echo "Raspberry Tank control code crashed, restarting..."
  sleep 1
done
