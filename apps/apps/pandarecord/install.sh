#!/bin/bash

echo "=========================="
echo "Installing App pandarecord"

# Here is where we perform installation of scripts, services, etc.
echo " - Installing scripts..."
# For pandarecord this is handled in libpanda/scripts/panda... for now...

# Enable the installed services on boot:
echo " - Enabling startup scripts..."
#systemctl daemon-reload # if needed
systemctl enable pandarecord

