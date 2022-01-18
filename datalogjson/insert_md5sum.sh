#!/bin/bash

# Simple script to add a hash to a file to id it before checking in and pushing.
# The hash is put in a define so it can be seen by looking at the file,
# and the program can display it.
# If a hash is in the file, it is removed before the hash is calulated
# so the script can be run repeatedly without issue.
# Walter Spurgiasz 2022

filename=datalogjson.ino # file to insert hash into

sed -i '/define md5HASH/d' $filename # https://www.baeldung.com/linux/delete-lines-containing-string-from-file
md5HASH=($(md5sum $filename| cut -d ' ' -f 1)) # https://gist.github.com/baamenabar/7411d3774829590040d9

md5_define="#define md5HASH $md5HASH"
echo "$md5_define"

sed -i "1i $md5_define" $filename # https://unix.stackexchange.com/questions/99350/how-to-insert-text-before-the-first-line-of-a-file
