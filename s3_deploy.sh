#!/bin/bash

# exit 1 on error
set -e

S3_BUCKET_NAME=eaa9124f-6497-4f9f-a408-78eac05b5744
S3_DEVEL_BUCKET=cf2e449e-50b3-48b5-ad6c-47abfa70e116
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"


echo "build target: $BUILD_FLAVOR"

if [[ "${BUILD_FLAVOR}" = *"autopilot"* && "${TRAVIS_BRANCH}" != "coverity" ]]; then

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
			echo "$TRAVIS_COMMIT" > ${SCRIPT_DIR}/s3autopilot/${folder_name}/git;

			function upload_file {
				s3cmd $4 --add-header=x-amz-meta-firmware-version:${TRAVIS_TAG} -m $3 --access_key=${AWS_ACCESS_KEY_ID} --secret_key=${AWS_SECRET_ACCESS_KEY} --add-header="Cache-Control:public, max-age=0" put $1 $2;
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
			flavor=${BUILD_FLAVOR#*_}
			root=autopilot

			if [[ ! -z "${TRAVIS_PULL_REQUEST_BRANCH}" ]]; then
				# Folder structure
				# root / board / label / flavor / base-branch / PR-branch / git-version
				upload_package ${folder_name} s3://${S3_BUCKET_NAME}/${root}/${board}/${label}/${flavor}/${TRAVIS_BRANCH}/${TRAVIS_PULL_REQUEST_BRANCH}/${gitversion};
			else
		if [[ "${TRAVIS_BRANCH}" = "develop" ]]; then
					# Folder structure
					# release-name / root / board / label / flavor
					upload_package ${folder_name} s3://$S3_DEVEL_BUCKET/H520-develop/${root}/${board}/${label}/${flavor} --acl-public
				fi;

				# root / board / label / flavor / branch / latest
				upload_package ${folder_name} s3://${S3_BUCKET_NAME}/${root}/${board}/${label}/${flavor}/${TRAVIS_BRANCH}/latest
				# root / board / label / flavor / branch / git-version
				upload_package ${folder_name} s3://${S3_BUCKET_NAME}/${root}/${board}/${label}/${flavor}/${TRAVIS_BRANCH}/${gitversion}
			fi;
		fi
	done
fi;
