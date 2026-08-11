#!/usr/bin/env bash
# Caps Lock で US英語 / Mozc日本語を切り替える設定を適用する。
# (GNOME Tweaks: Keyboard & Mouse → Additional Layout Options
#   → Switching to another layout → Caps Lock)
set -euo pipefail

if ! command -v gsettings >/dev/null 2>&1; then
  echo "gsettings not found; skip Caps Lock IME setup" >&2
  exit 0
fi

# 以前試した常駐デーモンが残っていれば止める（大文字ロック副作用の原因になり得る）
if command -v ps >/dev/null 2>&1; then
  ps -eo pid=,args= | awk '/[c]aps-ime-daemon\.py/{print $1}' | while read -r pid; do
    kill "$pid" 2>/dev/null || true
  done
fi
rm -f "$HOME/.config/autostart/caps-ime-switch.desktop"

# 入力ソース: English (US) + Mozc
gsettings set org.gnome.desktop.input-sources sources "[('xkb', 'us'), ('ibus', 'mozc-jp')]"

# Caps Lock でレイアウト/入力ソースグループを切替（大文字ロックにはしない）
gsettings set org.gnome.desktop.input-sources xkb-options "['grp:caps_toggle']"

# 予備の切替ショートカット
gsettings set org.gnome.desktop.wm.keybindings switch-input-source "['<Super>space', 'XF86Keyboard']"
gsettings set org.gnome.desktop.wm.keybindings switch-input-source-backward "['<Shift><Super>space', '<Shift>XF86Keyboard']"

# ibus 側のトリガーは空にして二重切替を避ける
gsettings set org.freedesktop.ibus.general.hotkey triggers "@as []"

echo "Caps Lock IME toggle configured:"
echo "  sources=$(gsettings get org.gnome.desktop.input-sources sources)"
echo "  xkb-options=$(gsettings get org.gnome.desktop.input-sources xkb-options)"
