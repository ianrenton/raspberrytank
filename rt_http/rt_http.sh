#!/bin/bash

cd "$(dirname "$0")"

until ./rt_http; do
  echo "Raspberry Tank control code crashed, restarting..."
  sleep 1
done
