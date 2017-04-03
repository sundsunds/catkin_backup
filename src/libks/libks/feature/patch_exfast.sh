#!/bin/bash
sed -e 's/pixel\[\(.*\)\]\] < c_b/pixel\[\1\]\] < p\[pixel\[(\1+8)%16]\] - b/' extendedfast-autogen.h | \
sed -e 's/pixel\[\(.*\)\]\] > cb/pixel\[\1\]\] > p\[pixel\[(\1+8)%16]\] + b/' > extendedfast-autogen_patched.h

for((i=0;i<16;i++)); do
	((j=(i+8)%16))
	sed -e "s/($i+8)%16/$j/" extendedfast-autogen_patched.h > tmp.h
	mv -f tmp.h extendedfast-autogen_patched.h
done

