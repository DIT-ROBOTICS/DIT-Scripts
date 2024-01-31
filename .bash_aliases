echo

tail -n 5 /var/log/DIT/$HOSTNAME.log | awk -v max_len=75 '{
    sub(/, IP: [^,]+/, "");
    sub(/User: [^,]+, /, "");
    if (length($0) > max_len) {
        print substr($0, 1, max_len) "... }";
    } else {
        print $0;
    }
}'

echo
