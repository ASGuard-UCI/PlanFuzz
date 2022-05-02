/apollo/clang/bin/clang -D_GLIBCXX_USE_CXX11_ABI=0 -I /apollo/clang/include -c -Wall  -fno-rtti -fpic rename_pass.cc
/apollo/clang/bin/clang -Wl,-znodelete -shared -o rename_pass.so rename_pass.o
