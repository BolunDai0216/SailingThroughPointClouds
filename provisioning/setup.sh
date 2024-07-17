#!/bin/zsh
set -e

REQUIRED_DIRS=(
  "$HOME/sailing-through-pcds-devcontainer"
)

for dir in "${REQUIRED_DIRS[@]}"; do
  if [ ! -d "$dir" ]; then
    echo "Creating directory $dir"
    mkdir -p $dir
  fi
done

if [ ! -f "$HOME/sailing-through-pcds-devcontainer/.zsh_history" ]; then
  echo "Creating file $HOME/sailing-through-pcds-devcontainer/.zsh_history"
  touch $HOME/sailing-through-pcds-devcontainer/.zsh_history
fi