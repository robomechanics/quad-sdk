file=".aspell.en.pws"
echo "personal_ws-1.1 en $(wc -l ${file}.txt | cut -d' ' -f1)" > "${file}"
cat "${file}.txt" | sort | uniq > ".${file}.tmp"
mv ".${file}.tmp" "${file}.txt"
cat "${file}.txt" >> ${file}
