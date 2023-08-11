#!/bin/bash

echo "=========================="
echo "Installing libpanda Apps"

if [ ! -f /usr/local/bin/yq ]; then
    echo " - Installing yq YAML parser..."
    # https://lindevs.com/install-yq-on-raspberry-pi
    sudo wget -qO /usr/local/bin/yq https://github.com/mikefarah/yq/releases/latest/download/yq_linux_arm
    sudo chmod a+x /usr/local/bin/yq
    yq --version
fi

echo " - Installing default apps..."
mkdir -p /etc/libpanda.d/apps # clear any currently installed apps
rm -rf /etc/libpanda.d/apps # Danger zone
mkdir -p /etc/libpanda.d/apps
#cp -r apps /etc/libpanda.d/

mkdir -p /etc/libpanda.d/apps-repositories

echo " - Installing app manager..."
if [ ! -f "/etc/libpanda.d/app" ]; then
    echo "None" > /etc/libpanda.d/app
fi
cp libpanda-app-manager.sh /usr/local/sbin/libpanda-app-manager
chmod +x /usr/local/sbin/libpanda-app-manager

# If this is a first install, then install default apps:
if [ ! -f "/etc/libpanda.d/app-manifest.yaml" ]; then
    /usr/local/sbin/libpanda-app-manager -g jmscslgroup/libpanda-default-apps -b main
else
    /usr/local/sbin/libpanda-app-manager -p # Update apps
fi

echo " - Done."
echo "========================="
