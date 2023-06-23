#!/bin/bash

################################################################################

rm -f out_cpplint.txt

FILTER="-build/c++11,+build/c++14,-readability/multiline_comment,-readability/alt_tokens,-whitespace/parens,-whitespace/braces,-whitespace/indent,-whitespace/comments,-whitespace/newline,-whitespace/blank_line,-whitespace/comma,-whitespace/line_length,-build/include_order"

cpplint \
    --filter=$FILTER \
    --linelength=100 \
    --extensions=h,cpp --root=./ \
    ./mcsim/*.h \
    ./mcsim/*/*.h \
    ./mcsim/*/*.cpp \
    2> out_cpplint.txt
EXIT_CODE=$?
if [ $EXIT_CODE -ne 0 ]; then echo "cpplint returned with exit code " $EXIT_CODE; exit $EXIT_CODE; fi

################################################################################
