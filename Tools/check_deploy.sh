#!/bin/bash

#is tagged release
git describe --tags --exact-match
if [ $? -ne 0 ]; then
    echo "check_deploy.sh: no release, no deployment, exit 0";
    exit 0;
else
    echo "check_deploy.sh: release"
fi

version_tag=$(git describe --tags --exact-match)

# check whether version tag starts with v[0-9]
if [[ $version_tag =~ ^v[0-9] ]]; then
    echo "check_deploy.sh: tag starts with v[0-9], travis deploy on"
else
    echo "check_deploy.sh: tag does not start with v[0-9], no deploy. exit 0"
    exit 0
fi

# check whether version tag is valid
version_tag=$(git describe --tags --exact-match)
if [[ $version_tag =~ ^v[0-9]+.[0-9]+.[0-9]+-[0-9]+.[0-9]+.[0-9]+ ]]; then
    numbers=$(echo "$version_tag" | egrep -o '[0-9]+')
    arr=($numbers)

    if [ ${arr[0]} -gt 255 ] || [ ${arr[1]} -gt 255 ] || [ ${arr[2]} -gt 255 ]; then
        echo "check_deploy.sh: error, version too high: $version_tag"
        exit 1;
    fi

    if [ ${arr[3]} -gt 255 ] || [ ${arr[4]} -gt 255 ] || [ ${arr[5]} -gt 255 ]; then
        echo "check_deploy.sh: error, vendor version too high: $version_tag"
        exit 1;
    fi

    echo "check_deploy.sh: version tag is valid: $version_tag"
else
    echo "check_deploy.sh: version tag is invalid: $version_tag, exit 1"
    exit 1;
fi

# check whether changelog entry exists for current tag
if [[ $(cat CHANGELOG.md | grep "## \[$version_tag") ]]; then
    echo "check_deploy.sh: changelog entry exists"
else
    echo "check_deploy.sh: changelog entry does not exist or has wrong format, exit 1"
    exit 1;
fi