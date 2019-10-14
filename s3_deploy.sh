#!/bin/bash

# exit 1 on error
set -e

S3_BUCKET_NAME=eaa9124f-6497-4f9f-a408-78eac05b5744
S3_DEVEL_BUCKET=cf2e449e-50b3-48b5-ad6c-47abfa70e116

echo "build target: $BUILD_TARGET"

if [[ "${BUILD_TARGET}" = *"autopilot"* && "${TRAVIS_BRANCH}" != "coverity" ]]; then

	./Tools/check_deploy.sh;

	mkdir -p s3autopilot;
	./Tools/generate_yuneec.py build/yuneec_tap-v2_default/yuneec_tap-v2_default.px4 s3autopilot/autopilot.yuneec ${AUTOPILOT_AES_KEY};
	fwversion=$(./yuneec_version.sh);
	gitversion=$(git describe --always --tags);
	echo "$fwversion" > s3autopilot/version;
	echo "$TRAVIS_COMMIT" > s3autopilot/git;
	date > s3autopilot/timestamp;
	cp CHANGELOG.md s3autopilot/changelog;
	git log -50 > s3autopilot/gitlog;
	sha256sum s3autopilot/autopilot.yuneec | head -c 64 > s3autopilot/hash;
	openssl aes-256-cbc -in build/yuneec_tap-v2_default/yuneec_tap-v2_default.elf -out s3autopilot/autopilot.elf.encrypted -k ${AUTOPILOT_AES_KEY}

	function upload_file {
		s3cmd $4 --add-header=x-amz-meta-firmware-version:${TRAVIS_TAG} -m $3 --access_key=${AWS_ACCESS_KEY_ID} --secret_key=${AWS_SECRET_ACCESS_KEY} --add-header="Cache-Control:public, max-age=0" put $1 $2;
	}
	
	function upload_package {
		echo "uploading firmware package to $1..."
		upload_file s3autopilot/autopilot.yuneec           $1/autopilot.yuneec application/octet-stream $2;
		upload_file s3autopilot/version                    $1/version text/plain $2;
		upload_file s3autopilot/hash                       $1/hash text/plain $2;
		upload_file s3autopilot/git                        $1/git text/plain $2;
		upload_file s3autopilot/gitlog                     $1/gitlog text/plain $2;
		upload_file s3autopilot/timestamp                  $1/timestamp text/plain $2;
		upload_file s3autopilot/changelog                  $1/changelog text/plain $2;
		upload_file s3autopilot/autopilot.elf.encrypted    $1/autopilot.elf.encrypted text/plain $2;
	};

	if [[ ! -z "${TRAVIS_PULL_REQUEST_BRANCH}" ]]; then
	upload_package s3://${S3_BUCKET_NAME}/${BUILD_TARGET}/${TRAVIS_BRANCH}/${TRAVIS_PULL_REQUEST_BRANCH}/${gitversion};
	else
		if [[ "${TRAVIS_BRANCH}" = "develop" ]]; then
			upload_package s3://$S3_DEVEL_BUCKET/H520-develop/${BUILD_TARGET} --acl-public
		fi;

		upload_package s3://${S3_BUCKET_NAME}/${BUILD_TARGET}/${TRAVIS_BRANCH}/latest

		upload_package s3://${S3_BUCKET_NAME}/${BUILD_TARGET}/${TRAVIS_BRANCH}/${gitversion}
	fi;

	if [[ is_release -eq 1 && "${BUILD_TARGET}" = "autopilot" ]]; then
	./Tools/publish_slack_notification.sh ${TRAVIS_TAG};
	fi;
fi;