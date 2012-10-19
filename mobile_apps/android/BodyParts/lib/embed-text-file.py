import sys

if len(sys.argv) != 3:
  print("Usage: %s array-name file-name\n" % sys.argv[0])
  sys.exit(1)

(array_name, file_name) = sys.argv[1:]

with open(file_name, 'rb') as f:
  text = f.read()

text += "\0"

print "extern const char %s[] = { %s };\n" % (
  array_name,
  ", ".join([hex(ord(b)) for b in text])
)

