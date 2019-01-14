#!/bin/bash

ulogtestdir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

options="-l 79"
black tests/* ${options}
black run_tests.py ${option}
