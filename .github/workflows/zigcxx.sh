#!/bin/bash
for arg do # https://unix.stackexchange.com/a/258514
	shift
	[ "$arg" = "-Wl,--exclude-libs,ALL" ] && continue
	set -- "$@" "$arg"
done
zig c++ -target x86-linux-gnu.2.26 "$@"
