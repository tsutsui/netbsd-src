BEGIN {
	total = 0
}

/^S1/ {
	# Remove header (two characters).
	s = substr($0, 3)

	# The first two digits shows bytelen in this line
	# (including the last checksum byte).
	len = hex2bin(substr(s, 1, 2))

	# The first two digits are bytelen.
	# The next four digits are address offset.
	for (i = 3; i < len - 1; i++) {
		val = substr(s, 1 + 2 * i, 2)
		if (val == "") {
			break
		}

		printf "0x%s,", val
		total++
		if (total % 8 == 0) {
			printf "\n"
		} else {
			printf " "
		}
	}
}

END {
	if (total % 8 != 0)
		printf "\n"
}

function hex2bin(s, v)
{
	for (i = 1; i <= length(s); i++) {
		x = index("0123456789ABCDEF", substr(s, i, 1))
		if (x == 0)
			return 0
		v = (16 * v) + (x - 1)
	}
	return v
}
