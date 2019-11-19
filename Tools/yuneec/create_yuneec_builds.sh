#!/bin/bash

# $1 tap-v2-key
# $2 tap-v3-key
# $3 build-directory
# $4 destination-directory

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
destination_dir=$3/autopilot


if [[ $4 ]]; then
	destination_dir=$4
fi

mkdir -p $destination_dir

for d in $3/*; do
	build_name="$(basename "$d")"
	if [[ "${build_name:0:6}" == "yuneec" ]]; then
		board_label=${build_name#*_}
		label=${board_label#*_}
		board=${board_label%_*}
		mkdir -p $destination_dir/${build_name}

		if [[ "${board}" == "tap-v2" ]]; then
			${script_dir}/generate_yuneec.py $3/${build_name}/${build_name}.px4 $destination_dir/${build_name}/autopilot.yuneec $1
			openssl aes-256-cbc -in $3/${build_name}/${build_name}.elf -out $destination_dir/${build_name}/autopilot.elf.encrypted -k $1
		elif [[ "${board}" == "tap-v3" ]]; then
			${script_dir}/generate_yuneec.py $3/${build_name}/${build_name}.px4 $destination_dir/${build_name}/autopilot.yuneec $2;
			openssl aes-256-cbc -in $3/${build_name}/${build_name}.elf -out $destination_dir/${build_name}/autopilot.elf.encrypted -k $2
		fi

		fwversion=$(${script_dir}/yuneec_version.sh)
		echo "$fwversion" > $destination_dir/${build_name}/version
		date > $destination_dir/${build_name}/timestamp
		cp ${script_dir}/../../CHANGELOG.md $destination_dir/${build_name}/changelog
		git log -50 > $destination_dir/${build_name}/gitlog
		sha256sum $destination_dir/${build_name}/autopilot.yuneec | head -c 64 > $destination_dir/${build_name}/hash
	fi
done
