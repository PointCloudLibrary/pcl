#
# .rst: FindClangFormat
# ---------------
#
# The module defines the following variables
#
# ``ClangFormat_EXECUTABLE`` Path to clang-format executable
# ``ClangFormat_FOUND`` True if the clang-format executable was found.
# ``ClangFormat_VERSION`` The version of clang-format found
#
# Example usage:
#
# .. code-block:: cmake
#
# find_package(ClangFormat)
# if(ClangFormat_FOUND)
# message("clang-format executable found: ${ClangFormat_EXECUTABLE}\n"
#         "version: ${ClangFormat_VERSION}")
# endif()

find_program(ClangFormat_EXECUTABLE
             NAMES
             # unreleased versions
                   clang-format-14
                   clang-format-13
                   clang-format-12
                   clang-format-11
             # current latest
                   clang-format-10
                   clang-format-9
             # since clang-format-8, only major version is prefixed
                   clang-format-8
                   clang-format-8.0
                   clang-format-7
                   clang-format-7.0
                   clang-format-6.0
                   clang-format-5.0
                   clang-format-4.0
                   clang-format-3.9
                   clang-format-3.8
                   clang-format-3.7
                   clang-format-3.6
                   clang-format-3.5
                   clang-format-3.4
                   clang-format-3.3
                   clang-format  # least priority
             DOC "clang-format executable")
mark_as_advanced(ClangFormat_EXECUTABLE)

# Extract version from command "clang-format -version"
if(ClangFormat_EXECUTABLE)
  execute_process(COMMAND ${ClangFormat_EXECUTABLE} -version
                  OUTPUT_VARIABLE clang_format_version
                  ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()
set(version_regex "^.*clang-format version ([.0-9]+).*")
if(clang_format_version MATCHES ${version_regex})
  # clang_format_version samples:
  # * clang-format version 3.9.1-4ubuntu3~16.04.1 (tags/RELEASE_391/rc2)
  # * Alpine clang-format version 8.0.0 (tags/RELEASE_800/final) (based on LLVM 8.0.0)
  string(REGEX
          REPLACE ${version_regex}
                  "\\1"
                  ClangFormat_VERSION
                  "${clang_format_version}")
  # ClangFormat_VERSION sample: "3.9.1", "8.0.0"
endif()
unset(clang_format_version)
unset(version_regex)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  ClangFormat
  REQUIRED_VARS
    ClangFormat_EXECUTABLE
    ClangFormat_VERSION
  VERSION_VAR
    ClangFormat_VERSION
)
