#!/bin/sh

set -eu

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VSCODE_SETTING_DIR=~/Library/Application\ Support/Code/User

wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt update
sudo apt install -y code 

if [ ! -d "$VSCODE_SETTING_DIR" ]; then
  mkdir -p "$VSCODE_SETTING_DIR"
fi

if [ -L "${VSCODE_SETTING_DIR}/settings.json" ]; then
  rm "${VSCODE_SETTING_DIR}/settings.json"
fi
ln -s "${SCRIPT_DIR}/settings.json" "${VSCODE_SETTING_DIR}/settings.json"

if [ -L "${VSCODE_SETTING_DIR}/keybindings.json" ]; then
  rm "${VSCODE_SETTING_DIR}/keybindings.json"
fi
ln -s "${SCRIPT_DIR}/keybindings.json" "${VSCODE_SETTING_DIR}/keybindings.json"

if [ -x "$(command -v code)" ]; then
  cat < ./extensions | while read -r line

  do
    code --install-extension "$line"
  done

  code --list-extensions > extensions
else
  echo "VSCode is not installed."
fi
