/apollo/clang/bin/clang -D_GLIBCXX_USE_CXX11_ABI=0 -I /apollo/clang/include -c -Wall  -fno-rtti -fpic init_target_pass.cc
/apollo/clang/bin/clang -Wl,-znodelete -shared -o init_target_pass.so init_target_pass.o
