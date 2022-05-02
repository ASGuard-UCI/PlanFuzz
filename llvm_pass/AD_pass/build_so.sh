/apollo/clang/bin/clang -D_GLIBCXX_USE_CXX11_ABI=0 -I /apollo/clang/include -c -Wall  -fno-rtti -fpic AD_pass.so.cc
/apollo/clang/bin/clang -Wl,-znodelete -shared -o AD_pass.so AD_pass.so.o
