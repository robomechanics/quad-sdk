#!/bin/bash

directory="."   # Specify the directory containing the files
search_string="flat_005realtime" # Specify the string to search for
replacement_string="flat_offline_005realtime"   # Specify the string to replace it with

cd "$directory" || exit 1         # Navigate to the directory or exit if it doesn't exist

for file in *"$search_string"*; do
    new_name="${file//$search_string/$replacement_string}"  # Replace the search string with the replacement string
    mv "$file" "$new_name"                                  # Rename the file
done

echo "File renaming completed."
