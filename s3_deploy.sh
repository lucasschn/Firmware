#!/bin/bash

# exit 1 on error
set -e

ARG_GIT_COMMIT=$1 		# commit hash
ARG_GIT_BRANCH=$2 		# base branch
ARG_BUILD_FLAVOR=$3 		# autopilot build flavor

# Check environemnt variables
if [ -z ${AWS_ACCESS_KEY} ]; then echo "AWS_ACCESS_KEY is unset" && exit 1; fi
if [ -z ${AWS_SECRET_ACCESS_KEY} ]; then echo "AWS_SECRET_ACCESS_KEY is unset" && exit 1; fi
if [ -z ${AUTOPILOT_TAP_V2_KEY} ]; then echo "AUTOPILOT_TAP_V2_KEY is unset" && exit 1; fi
if [ -z ${AUTOPILOT_TAP_V3_KEY} ]; then echo "AUTOPILOT_TAP_V3_KEY is unset" && exit 1; fi
if [ -z ${S3_BUCKET_CI_ARCHIVE} ]; then echo "S3_BUCKET_CI_ARCHIVE is unset" && exit 1; fi
if [ -z ${S3_BUCKET_DEVELOPMENT} ]; then echo "S3_BUCKET_DEVELOPMENT is unset" && exit 1; fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "build target: ${ARG_BUILD_FLAVOR}"

if [[ "${ARG_BUILD_FLAVOR}" = *"autopilot"* ]]; then

	./Tools/yuneec/check_deploy.sh;

	mkdir -p s3autopilot;

	# create build.yuneec and meta-data
	./Tools/yuneec/create_yuneec_builds.sh ${AUTOPILOT_TAP_V2_KEY} ${AUTOPILOT_TAP_V3_KEY} ${SCRIPT_DIR}/build ${SCRIPT_DIR}/s3autopilot
	gitversion=$(git describe --always --tags);

	# upload to s3
	for d in s3autopilot/*; do
		folder_name="$(basename "$d")"
		if [[ "${folder_name:0:6}" == "yuneec" ]]
		then
			# add travis commit
			echo "$ARG_GIT_COMMIT" > ${SCRIPT_DIR}/s3autopilot/${folder_name}/git;

			function upload_file {
				s3cmd $4 -m $3 --access_key=${AWS_ACCESS_KEY} --secret_key=${AWS_SECRET_ACCESS_KEY} --add-header="Cache-Control:public, max-age=0" put $1 $2;
			}

			function upload_package {
				echo "uploading firmware package to $2..."
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/autopilot.yuneec           $2/autopilot.yuneec application/octet-stream $3;
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/version                    $2/version text/plain $3;
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/hash                       $2/hash text/plain $3;
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/git                        $2/git text/plain $3;
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/gitlog                     $2/gitlog text/plain $3;
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/timestamp                  $2/timestamp text/plain $3;
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/changelog                  $2/changelog text/plain $3;
				upload_file ${SCRIPT_DIR}/s3autopilot/$1/autopilot.elf.encrypted    $2/autopilot.elf.encrypted text/plain $3;
			};

			board_label=${folder_name#*_}
			label=${board_label#*_}
			board=${board_label%_*}
			flavor=${ARG_BUILD_FLAVOR#*_}
			root=autopilot

			# Deployments to S3_BUCKET_DEVELOPMENT
			if [[ "${ARG_GIT_BRANCH}" = "develop" ]] && [[ "${board}" = "tap-v2" ]] && [[ "${label}" = "h520" ]] && [[ "${flavor}" = "yuneec" ]]; then
				upload_package ${folder_name} s3://$S3_BUCKET_DEVELOPMENT/H520-develop/autopilot --acl-public
			fi;

			if [[ "${ARG_GIT_BRANCH}" = "develop" ]] && [[ "${board}" = "tap-v3" ]] && [[ "${label}" = "h520mk2" ]] && [[ "${flavor}" = "yuneec" ]]; then
				upload_package ${folder_name} s3://$S3_BUCKET_DEVELOPMENT/H520mk2-develop/autopilot --acl-public
			fi;

			# Deployments to S3_BUCKET_CI_ARCHIVE
			upload_package ${folder_name} s3://${S3_BUCKET_CI_ARCHIVE}/${root}/${board}/${label}/${flavor}/${ARG_GIT_BRANCH}/latest
			upload_package ${folder_name} s3://${S3_BUCKET_CI_ARCHIVE}/${root}/${board}/${label}/${flavor}/${ARG_GIT_BRANCH}/${gitversion}
		fi
	done
fi;
