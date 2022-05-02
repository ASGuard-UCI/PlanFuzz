/apollo/clang/bin/clang -I /apollo/clang/include -c -Wall -Werror -fpic foo.c
/apollo/clang/bin/clang -shared -o foo.so foo.o
