#!/usr/bin/env bash
set -e

# Install Nix if not available
if ! command -v nix >/dev/null 2>&1; then
  echo "[INFO] Nix not found. Installing Nix..."
  curl -L https://nixos.org/nix/install | sh

  # Add Nix to the current shell session
  if [ -e "$HOME/.nix-profile/etc/profile.d/nix.sh" ]; then
    . "$HOME/.nix-profile/etc/profile.d/nix.sh"
  elif [ -e "/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh" ]; then
    . "/nix/var/nix/profiles/default/etc/profile.d/nix-daemon.sh"
  fi

  # Enable flakes
  mkdir -p ~/.config/nix
  echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
fi

# Enter the dev shell
exec nix develop --impure -c $SHELL

