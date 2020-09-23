#! /bin/bash

tag=$1
shift

if [[ -n $(git status --porcelain) ]]
then
  echo "ERROR: working directory is not clean"
  echo "commit all changes and try again."
  exit 1
fi

if [[ -z $tag ]]
then
  echo "ERROR: version number required"
  echo "example: $0 v1.0.1"
  echo "current tags:"
  git tag
  exit 1
fi


function exit_on_error()
{
  echo "There was an error. Commit will not be tagged."
}

trap exit_on_error ERR

set -e
root=$(git rev-parse --show-toplevel)

echo "cd'ing to root directory ($root)"
cd $root

echo "looking for pre-tag-release.sh to run"
$(find ./ -name 'pre-tag-release.sh')

echo "writing ${tag} to version.txt"
echo "${tag}" > version.txt
echo "commiting change"
git add version.txt
git commit -m "housekeeping: bumped version in version.txt"
echo "tagging with ${tag}"
git tag ${tag}
git tag | grep ${tag}
echo "Successfully tagged commit."
