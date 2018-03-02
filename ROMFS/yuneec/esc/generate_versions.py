#!/usr/bin/env python

binaries = ['H520_ESC.bin', 'OB_ESC.bin', 'V18S_ESC.bin']
output_file = 'versions.md'

def read_version(file_path):
    f = open(file_path, 'r')
    b = f.read()
    f.close()
    return (ord(b[512]) + (ord(b[513]) << 8) + (ord(b[514]) << 16) + (ord(b[515]) << 24)) * 0.01


f = open(output_file, 'w')
f.write('# ESC firmware binary versions\n\n')
for b in binaries:
    ver = read_version(b)
    f.write(b +': \t' + str(ver) + "\n")

f.close()
