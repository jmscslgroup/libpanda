#!/bin/bash

echo "=========================="
echo "Updating libpanda Apps"


CURRENT_APP=$(libpanda-app-manager -c)
NEED_TO_RESTART=0
libpanda-app-manager -t | grep "running" # check status
if [ "$?" -eq 0 ];
then
    NEED_TO_RESTART=1
    libpanda-app-manager -k # stop app
fi
libpanda-app-manager -u # uninstall
echo "Updating libpanda apps..."
echo "Updating libpanda apps..." > /etc/libpanda.d/logMessage
libpanda-app-manager -p
./install.sh
libpanda-app-manager -i $CURRENT_APP # reintall

if [ "$NEED_TO_RESTART" -eq 1 ];
then
    libpanda-app-manager -s # start
fi


echo " - Done."
echo "========================="
