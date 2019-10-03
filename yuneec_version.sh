git describe --always --tags --long | cut -f1,2,3 -d'-' | sed 's/\(.*\)-/\1./'
