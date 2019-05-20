#!/bin/bash -e
if [[ $(uname) == "Linux" ]]; then
  sudo apt-get install -y git imagemagick rsync
else
  echo "Platform $(uname) is not supported! Go away!"
fi
