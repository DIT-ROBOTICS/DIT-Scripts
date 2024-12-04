echo

tail -n 3 /var/log/DIT/$HOSTNAME.log | awk -v max_len=65 '{
    gsub(/User: [^,]+, IP: [^,]*, /, "");
    gsub(/Message: { /, "");
    gsub(/ }$/, "");
    if (length($0) > max_len) {
        print substr($0, 1, max_len) " ...";
    } else {
        print $0;
    }
}'

echo
