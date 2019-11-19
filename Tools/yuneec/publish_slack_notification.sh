#! /bin/bash

curl -X POST -H 'Content-type: application/json' --data '{"text" : "New Autopilot Version Available: '$1'"}' https://hooks.slack.com/services/${SLACK_RELEASES_WEBHOOK};
