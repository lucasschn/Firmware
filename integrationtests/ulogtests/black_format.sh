#!/bin/bash

ulogtestdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

options="-l 79"
black run_tests.py ${option}
