# https://github.com/fish-shell/fish-shell/issues/5865
include(CheckCXXSourceCompiles)
CHECK_CXX_SOURCE_COMPILES("
#include <atomic>
struct big { int foo[64]; };
std::atomic<big> x;
int main() {
   return x.load().foo[13];
}"
LIBATOMIC_NOT_NEEDED)
IF (NOT LIBATOMIC_NOT_NEEDED)
    SET(ATOMIC_LIBRARY "atomic")
ENDIF()