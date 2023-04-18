#!/bin/bash

echo "=========================="
echo "Installing libpanda Apps"

echo " - Installing apps..."
mkdir -p /etc/libpanda.d/apps
rm -rf /etc/libpanda.d/apps # Danger zone
cp -r apps /etc/libpanda.d/

echo " - Installing app manager..."
if [ ! -f "/etc/libpanda.d/app" ]; then
    echo "None" > /etc/libpanda.d/app
fi
cp libpanda-app-manager.sh /usr/local/sbin/libpanda-app-manager
chmod +x /usr/local/sbin/libpanda-app-manager

echo " - Done."
echo "========================="
