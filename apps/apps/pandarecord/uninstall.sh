#!/bin/bash

echo "=========================="
echo "Removing App pandarecord"


# Disable the installed services:
echo " - Disabling startup scripts..."
systemctl disable pandarecord

# Here is where we remove scripts, services, etc.
echo " - Removing scripts..."
# For pandarecord this is handled in libpanda/scripts/panda... for now...

#systemctl daemon-reload # if needed

