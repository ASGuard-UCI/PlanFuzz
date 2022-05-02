/apollo/clang/bin/clang -D_GLIBCXX_USE_CXX11_ABI=0 -I /apollo/clang/include -c -Wall -Werror -fno-rtti -fpic afl-llvm-pass.so.cc
/apollo/clang/bin/clang -Wl,-znodelete -shared -o afl-llvm-pass.so afl-llvm-pass.so.o
