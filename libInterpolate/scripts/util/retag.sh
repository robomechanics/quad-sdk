#! /bin/bash

# retag all commits
  
tagfile=${1:-/dev/stdin}

cat $tagfile | grep -v "^#" | grep '.' | while read line
do
  tag=$(echo $line | cut -d' ' -f2)
  sha=$(echo $line | cut -d' ' -f1)
  git tag $tag $sha
done
