#!/bin/bash -e
if [[ $(uname) == "Linux" ]]; then
  sudo apt-get install -y clang-format
else
  echo "Platform $(uname) is not supported! Go away!"
fi
