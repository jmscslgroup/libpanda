#!/bin/bash

INTERACTIVE=True
APP_DIR=/etc/libpanda.d/apps
CURRENT_APP_FILE=/etc/libpanda.d/app

#echo " - Getting app list..."
APPS=$(ls $APP_DIR)
#APPS="${APPS}"

#echo " Apps: ${APPS}"

CURRENT_APP=$(cat $CURRENT_APP_FILE)
#echo " Current App: ${CURRENT_APP}"


do_descriptions () {
    echo -e "App\tService\tEnabled\tRunning\tDescription"
    for APP in $APPS;
    do
        DESCRIPTION=$(cat $APP_DIR/$APP/description.txt)
        ENABLED=$([[ "$APP" == "$CURRENT_APP" ]] && echo "yes" || echo "no")
        SERVICE=$(cat $APP_DIR/$APP/service)
        if [ "$ENABLED" = "yes" ]; then
            RUNNING=$(systemctl status $SERVICE | grep "running" > /dev/null 2>&1 && echo "yes" || echo "no")
        else
            RUNNING="no"
        fi
        echo -e "${APP}\t$SERVICE\t$ENABLED\t$RUNNING\t\"${DESCRIPTION}\""
    done
}

do_start_app () {
    $APP_DIR/$CURRENT_APP/start.sh
}

do_stop_app () {
    $APP_DIR/$CURRENT_APP/stop.sh
}

do_status () {
    SERVICE=$(cat $APP_DIR/$CURRENT_APP/service)
    systemctl status $SERVICE | grep Active | sed "s/.*Active: //g"
}

do_app_uninstall () {
    APP_TO_REMOVE=$CURRENT_APP
    echo "Removing App: $APP_TO_REMOVE"
    
    if [ "$APP_TO_REMOVE" = "None" ]; then
        echo " - Nothing to do!"
        return 0
    fi

    $APP_DIR/$APP_TO_REMOVE/stop.sh
    $APP_DIR/$APP_TO_REMOVE/uninstall.sh
}

do_app_install () {
    APP_TO_INSTALL=$1
    echo "Installing: $APP_TO_INSTALL"
    
    if [ "$APP_TO_Install" = "None" ]; then
        echo " - Nothing to do!"
        return 0
    fi
    
    $APP_DIR/$APP_TO_INSTALL/install.sh
    echo "$APP_TO_INSTALL" > $CURRENT_APP_FILE
}


do_interactive () {
PS3="Enter an app number to install: "
OPTIONS="$APPS None Quit"
OPTION_COUNT=$(echo "${OPTIONS}" | wc -w)
select APP in $OPTIONS
do
    if [ 1 -le "$REPLY" ] && [ "$REPLY" -le $OPTION_COUNT ];
    then
        echo "Selected: $APP"
        break;
    else
        echo "Wrong selection: Select any number from 1-$OPTION_COUNT"
    fi
done

case $APP in
    
    None)
    do_app_uninstall $CURRENT_APP
    echo "$APP" > $CURRENT_APP_FILE
    ;;

    Quit)
    ;;

  *)
    if [ "${APP}" = "${CURRENT_APP}" ]; then
        echo " - $APP is already installed!"
    else
        do_app_uninstall $CURRENT_APP
        do_app_install $APP
    fi
    ;;
esac


    return 0
}

check_root() {
if [ "$EUID" -ne 0 ];
  then echo "Please run as root"
  exit 1
fi
}

usage() {
    # this is clever, from https://stackoverflow.com/questions/16483119/an-example-of-how-to-use-getopts-in-bash
    echo "$0 usage:" && grep "[[:space:]].)\ #" $0 | sed 's/#//' | sed -r 's/([a-z])\)/-\1/'
    exit 1
}

check_app_exists() {
    TO_CHECK=$1
    RET=1
    for APP in $APPS;
    do
#        echo "Checking |${APP}| against |${TO_CHECK}|"
        if [ "$APP" = "$TO_CHECK" ];
        then
#            echo "match!"
            RET=0
        fi
    done
    
#    echo "RET= $RET"
    return $RET
}


#echo "=========================="
#echo "Libpanda App Manager"

#echo "Num arguments: $#"

if [ "$#" -eq 0 ]; then
    check_root
    echo "=========================="
    echo "Libpanda App Manager"
    echo "Doing interactive mode..."
    do_interactive
    echo " - Done."
    echo "========================="
fi




while getopts ":htkdsculi:" o; do
    case "${o}" in
        i) # Specify App to be installed
#            echo "Installing: ${OPTARG}"
            check_root
            check_app_exists $OPTARG
            if [ "$?" -eq 1 ];
            then
                echo "Error: Invalid app.  Please run the following to list available apps"
                echo "$0 -l"
            else
                do_app_uninstall
                do_app_install $OPTARG
            fi
            
            ;;
        u) # Uninstall current App
            echo "Removing: ${CURRENT_APP}"
            check_root
            do_app_uninstall
            ;;
        l) # List available Apps
            echo "$APPS"
            ;;
        c) # Currently installed App
            echo "$CURRENT_APP"
            ;;
        d) # Descriptions of Apps
            do_descriptions
            ;;
        s) # Start the current App
            do_start_app
            ;;
        k) # Kill the current App
            do_stop_app
            ;;
        t) # sTatus of current app
            do_status
            ;;
        h | *) # Help
            usage
            ;;
    esac
done


exit 0

#echo " - Done."
#echo "========================="
