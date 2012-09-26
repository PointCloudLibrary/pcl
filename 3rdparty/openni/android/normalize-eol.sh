if type dos2unix > /dev/null; then
  DOS2UNIX=dos2unix
elif type fromdos > /dev/null; then
  DOS2UNIX=fromdos
else
  echo "You will need either dos2unix or fromdos for this to work."
  exit 1
fi

normalize_eol() {
  find . -type f '(' \
   -name '*.h' -o -name '*.c' -o -name '*.cpp' -o -name '*.java' -o -name '*.py' \
   -o -name '*.sh' -o -name '*.ini' ')' \
    -exec $DOS2UNIX {} ';'
}
