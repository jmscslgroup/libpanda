#!/bin/bash



init () {
    INTERACTIVE=True
    APP_DIR=/etc/libpanda.d/apps    # where apps are copied (not a great structure TBH)
    APP_REPOSITORIES=/etc/libpanda.d/apps-repositories  # where apps are checked out
    APP_MANIFEST=/etc/libpanda.d/app-manifest.yaml

    #echo " - Getting app list..."
#    APPS=$(ls $APP_DIR)
    
    if [ -f $APP_MANIFEST ]; then
        MANIFEST_VERSION=$(yq e '.version' $APP_MANIFEST)
        CURRENT_APP=$(yq e '.current' $APP_MANIFEST)
        
        get_apps_from_manifest
#        echo "APPS: ${APPS[*]}"
    fi
}

get_apps_from_manifest() {
    APPS2=()
    APPS=()
    REPOSITORES=$(yq e '.repositories | to_entries | .[] | .key' $APP_MANIFEST)
    
    for REPOSITORY in $REPOSITORES;
    do
        cd $APP_REPOSITORIES/$REPOSITORY
        if [ -f libpanda-apps.yaml ]; then
            indices=$(yq e '.apps | to_entries | .[] | .key' libpanda-apps.yaml)
            for index in ${indices}
            do
                APP_NAME=$(yq e ".apps[${index}].name" libpanda-apps.yaml)
                
                APPS2+=("$REPOSITORY/$APP_NAME")
                APPS+=("$APP_NAME")
#                APP_PATH=$(yq e ".apps[${index}] | select(has(\"path\")) | .path" libpanda-apps.yaml)
#                if [ -z "$APP_PATH" ]; then
#                    APP_PATH=$APP_NAME
#                fi
            done

        else
            echo "Error!  Repository $APP_TO_UPDATE is missing libpanda-app.yaml...." >&2
        fi
        
    done
}


do_descriptions () {
    
    echo -e "Repository\tBranch\tApp\tService\tEnabled\tRunning\tDescription"

    REPOSITORES=$(yq e '.repositories | to_entries | .[] | .key' $APP_MANIFEST)
    
    for REPOSITORY in $REPOSITORES;
    do
        BRANCH=$(yq e '.repositories.'"${REPOSITORY}"'.branch' $APP_MANIFEST)
        
        if [ ! -z "${REPOSITORY}" ]; then # Sanity check to not reading empy lines
            #do_repo_update ${REPOSITORY} ${BRANCH}
                
            cd $APP_REPOSITORIES/$REPOSITORY
            
                # now parse the yaml file, if it exists
            if [ -f libpanda-apps.yaml ]; then
                indices=$(yq e '.apps | to_entries | .[] | .key' libpanda-apps.yaml)

                for index in ${indices}
                do
#                    echo ""
#                    echo "Element $index"
#                   yq e ".apps[${index}]" libpanda-apps.yaml
                    APP_NAME=$(yq e ".apps[${index}].name" libpanda-apps.yaml)
                    APP_PATH=$(yq e ".apps[${index}] | select(has(\"path\")) | .path" libpanda-apps.yaml)
                    if [ -z "$APP_PATH" ]; then
                        APP_PATH=$APP_NAME
                    fi
                    #echo "Copying App \"${APP_NAME}\" from repository path ${APP_PATH} into ${APP_DIR}/"
            
#                    cp -r ${APP_PATH} ${APP_DIR}/
                    APP=${APP_PATH%%/*}
                    
                        
                    DESCRIPTION=$(cat $APP_DIR/$APP/description.txt)
                    ENABLED=$([[ "$APP" == "$CURRENT_APP" ]] && echo "yes" || echo "no")
                    SERVICE=$(cat $APP_DIR/$APP/service)
                    if [ "$ENABLED" = "yes" ]; then
                        RUNNING=$(systemctl status $SERVICE | grep "running" > /dev/null 2>&1 && echo "yes" || echo "no")
                    else
                        RUNNING="no"
                    fi
                    
                    echo -e "${REPOSITORY}\t${BRANCH}\t${APP}\t$SERVICE\t$ENABLED\t$RUNNING\t\"${DESCRIPTION}\""
                done

#            else
                #echo "Error!  Repository $APP_TO_UPDATE is missing libpanda-app.yaml...."
            fi
            
        fi
    done
#    done < $APP_MANIFEST
}

do_start_app () {
    if [ "${CURRENT_APP}" != "None" ]; then
        $APP_DIR/$CURRENT_APP/start.sh
    fi
}

do_stop_app () {
    if [ "${CURRENT_APP}" != "None" ]; then
        $APP_DIR/$CURRENT_APP/stop.sh
    fi
}

do_status () {
    if [ "${CURRENT_APP}" == "None" ]; then
        echo "inactive (none installed)"
    else
        SERVICE=$(cat $APP_DIR/$CURRENT_APP/service)
        systemctl status $SERVICE | grep Active | sed "s/.*Active: //g"
    fi
}

do_repo_update () {
    APP_TO_UPDATE=$1
    APP_BRANCH=$2
    THIS_APP_DIRECTORY=${APP_TO_UPDATE%%/*}
    
    
    echo "Updating App ${APP_TO_UPDATE} with branch ${APP_BRANCH}..."

    mkdir -p $APP_REPOSITORIES/$THIS_APP_DIRECTORY
    if [ ! -d $APP_REPOSITORIES/$APP_TO_UPDATE ]; then
        cd $APP_REPOSITORIES/$THIS_APP_DIRECTORY
        git clone https://github.com/$APP_TO_UPDATE
        cd ../$APP_TO_UPDATE
    else
        echo "Entering ${APP_REPOSITORIES}/${APP_TO_UPDATE}"
        cd $APP_REPOSITORIES/$APP_TO_UPDATE
        git pull
    fi
                
#    TEST_BRANCH=$(sudo git rev-parse --verify $APP_BRANCH)
    TEST_BRANCH=$(sudo git ls-remote origin $APP_BRANCH)
    
    if [ -z "${TEST_BRANCH}" ]; then
        echo "Error: branch ${APP_BRANCH} does not exist in repository ${APP_TO_ADD}" >&2
        do_repo_remove $APP_TO_UPDATE
        return 0
    fi
    
#    echo "TEST_BRANCH = ${TEST_BRANCH}"
    
    git checkout $APP_BRANCH
    
    pwd
    # now parse the yaml file, if it exists
    if [ -f libpanda-apps.yaml ]; then
        indices=$(yq e '.apps | to_entries | .[] | .key' libpanda-apps.yaml)

        for index in ${indices}
        do
#            echo ""
#            echo "Element $index"
#           yq e ".apps[${index}]" libpanda-apps.yaml
            APP_NAME=$(yq e ".apps[${index}].name" libpanda-apps.yaml)
            APP_PATH=$(yq e ".apps[${index}] | select(has(\"path\")) | .path" libpanda-apps.yaml)
            if [ -z "$APP_PATH" ]; then
                APP_PATH=$APP_NAME
            fi
            echo "Copying App \"${APP_NAME}\" from repository path ${APP_PATH} into ${APP_DIR}/"
            
            cp -r ${APP_PATH} ${APP_DIR}/
        done

    else
        echo "Error!  Repository $APP_TO_UPDATE is missing libpanda-app.yaml...." >&2
    fi
}

do_repo_update_all () {
    echo "Updating all apps..."
    rm -rf $APP_DIR # Danger!
    mkdir -p $APP_DIR

    REPOSITORES=$(yq e '.repositories | to_entries | .[] | .key' $APP_MANIFEST)
        
    for REPOSITORY in $REPOSITORES;
    do
        BRANCH=$(yq e '.repositories.'"${REPOSITORY}"'.branch' $APP_MANIFEST)
        
#        if [ ! -z "${REPOSITORY}" ]; then
        do_repo_update ${REPOSITORY} ${BRANCH}
#        fi
    done
    
}

do_repo_add () {
    APP_TO_ADD=$1
    
    if [ "$#" -eq 2 ]; then
#        echo "A branch was provided: $2"
        APP_BRANCH=$2
        echo "Attempting to add repository: ${APP_TO_ADD} with branch: ${APP_BRANCH}"
    else
        APP_BRANCH="main"
        echo "Attempting to add repository: ${APP_TO_ADD}"
    fi
    
    # check format:
    re='([^[:space:]\/]+\/[^[:space:]\/]+)'
    if  [[ "$APP_TO_ADD" =~ $re ]]; then
        echo "You provided: ${BASH_REMATCH[1]}"
    else
        echo "Repository unrecognized: ${APP_TO_ADD}" >&2
        echo "Please provide in the format of <owner>/<repository>" >&2
        echo "For example: https://github.com/jmscslgroup/libpanda --> jmscslgroup/libpanda"
        return 0
    fi
    
    GIT_REPO="https://github.com/${APP_TO_ADD}"
    if wget -q --method=HEAD ${GIT_REPO}; then
        echo "Repository found and available!"
    else
        echo "Error getting repository, either it does not exist or internet is not available" >&2
        echo "Unable to resolve ${GIT_REPO}" >&2
        return 0
    fi
    
    
    touch $APP_MANIFEST
    yq -i '.repositories.'"${APP_TO_ADD}"'.branch = "'"${APP_BRANCH}"'"' $APP_MANIFEST

    do_repo_update $APP_TO_ADD $APP_BRANCH
}

do_repo_remove () {
    APP_TO_REMOVE=$1
    
    #do_app_uninstall $APP_TO_REMOVE
    echo "Removing app repository: $APP_TO_REMOVE"
    
#    echo ":$APP_TO_REMOVE,:d"
#    sed -i "\:$APP_TO_REMOVE,:d" $APP_MANIFEST
    yq -i 'del(.repositories.'"${APP_TO_REMOVE}"')' $APP_MANIFEST
    
    if [ -d $APP_REPOSITORIES//$APP_TO_REMOVE ]; then
        rm -rf $APP_REPOSITORIES/$APP_TO_REMOVE # Dangerzone
    fi
    
    do_repo_update_all  # not elegant but whatever
}

do_app_uninstall () {
    APP_TO_REMOVE=$CURRENT_APP
    echo "Uninstalling App: $APP_TO_REMOVE"
    
    if [ "$APP_TO_REMOVE" = "None" ]; then
        echo " - Nothing to do!"
        return 0
    fi

    $APP_DIR/$APP_TO_REMOVE/stop.sh
    $APP_DIR/$APP_TO_REMOVE/uninstall.sh
#    echo "None" > $CURRENT_APP_FILE
    yq -i '.current = "None"' $APP_MANIFEST
}

do_app_install () {
    APP_TO_INSTALL=$1
    echo "Installing: $APP_TO_INSTALL"
    
    if [ "$APP_TO_INSTALL" = "None" ]; then
        echo " - Nothing to do!"
        return 0
    fi
    
    $APP_DIR/$APP_TO_INSTALL/install.sh
#    echo "$APP_TO_INSTALL" > $CURRENT_APP_FILE
    yq -i '.current = "'"$APP_TO_INSTALL"'"' $APP_MANIFEST
}

do_migrate_empty () {
    check_root
    touch $APP_MANIFEST
}

do_migrate_0_0 () {
    check_root
    # old methods of init:
    CURRENT_APP_FILE=/etc/libpanda.d/app
    CURRENT_APP=$(cat $CURRENT_APP_FILE)

    APP_MANIFEST_OLD="${APP_MANIFEST}.bak"
    mv $APP_MANIFEST $APP_MANIFEST_OLD
    touch $APP_MANIFEST
    
    yq -i '.version = 0.0' $APP_MANIFEST
    
    # move repositories located at .[] into .repositories[]:
    REPOSITORES=$(yq e ' to_entries | .[] | .key' $APP_MANIFEST_OLD)
    for REPOSITORY in $REPOSITORES;
    do
        BRANCH=$(yq e '.'"${REPOSITORY}"'.branch' $APP_MANIFEST_OLD)
#        echo "Copying ${REPOSITORY}"
        yq -i '.repositories.'"${REPOSITORY}"'.branch = "'"${BRANCH}"'"' $APP_MANIFEST
    done
    yq -i '.current = "'"$CURRENT_APP"'"' $APP_MANIFEST
}

do_migrate () {
    if [ -z "$MANIFEST_VERSION" ]; then
#        echo "Manifest does not exist!  nothing to migrate"
        do_migrate_empty
    elif [ "$MANIFEST_VERSION" == "null" ]; then
#        echo "App version out of date!"
        do_migrate_0_0
    else    # Current version is 0.0
        return
    fi

        
    init
    do_migrate
}


do_interactive () {
    PS3="Enter an app number to install: "
    OPTIONS="${APPS[*]} None Quit"
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
#           echo "$APP" > $CURRENT_APP_FILE
            yq -i '.current = '"$APP"'' $APP_MANIFEST
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
    init
    do_migrate
    do_interactive
    echo " - Done."
    echo "========================="
else    # lame
    init
    do_migrate
fi

THIRD_PARTY_REPO=
THIRD_PARTY_BRANCH=
while getopts ":htkdscupli:g:b:r:" o; do
    case "${o}" in
        i) # Specify App to be installed
#            echo "Installing: ${OPTARG}"
            check_root
            check_app_exists $OPTARG
            if [ "$?" -eq 1 ];
            then
                echo "Error: Invalid app.  Please run the following to list available apps" >&2
                echo "$0 -l"
            else
                do_app_uninstall
                do_app_install $OPTARG
            fi
            
            ;;
        g) # Add 3rd Party App Repository (in form <owner>/<repo>). Example: -g jmscslgroup/libpanda
            check_root
            THIRD_PARTY_REPO=$OPTARG
            ;;
        b) # Specify Branch for 3rd Party App Repository (in addition to repository). Example: -g jmscslgroup/libpanda -b test_branch
            check_root
            THIRD_PARTY_BRANCH=$OPTARG
            ;;
        r) # Remove 3rd Party App Repository (in form <owner>/<repo>). Example: -r jmscslgroup/libpanda
            check_root
            do_repo_remove $OPTARG
            ;;
        p) # Updates all apps from their repositories
            check_root
            do_repo_update_all
            ;;
        u) # Uninstall current App
            check_root
            echo "Removing: ${CURRENT_APP}"
            do_app_uninstall
            ;;
        l) # List available Apps
            echo "${APPS[*]}"
            ;;
        c) # Currently installed App
            echo "$CURRENT_APP"
            ;;
        d) # Descriptions of Apps
            do_descriptions
            ;;
        s) # Start the current App
            check_root
            do_start_app
            ;;
        k) # Kill the current App
            check_root
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

if [ ! -z "$THIRD_PARTY_REPO" ];
then
    do_repo_add $THIRD_PARTY_REPO $THIRD_PARTY_BRANCH
else
    if [ ! -z "$THIRD_PARTY_BRANCH" ];
    then
        usage
    fi
fi

exit 0

#echo " - Done."
#echo "========================="
