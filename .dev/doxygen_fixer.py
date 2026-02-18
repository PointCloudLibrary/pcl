#! /usr/bin/env python3

from fileinput import FileInput
import sys

# https://libclang.readthedocs.io/en/latest/_modules/clang/cindex.html
import clang.cindex
from clang.cindex import Index, SourceLocation, SourceRange, TokenKind

from doxy_commands import DOXYGEN_SPECIAL_COMMAND_SET


def srcrangestr(x):
    return "%s:%d:%d - %s:%d:%d" % (
        x.start.file,
        x.start.line,
        x.start.column,
        x.end.file,
        x.end.line,
        x.end.column,
    )


def fix_comment(keywords, comment_line, comment_prefix="*"):
    """Adds a newline before any keywords in the comment found anywhere except the beginning of the line

    Args:
        keywords (set(str)): keywords to detect and apply fix on
        comment (str): Verbatim comment, just 1 line with no new-lines
        comment_prefix (str, optional): The string to go at the beginning of the added newlines if any. Defaults to "*".

    Returns:
        str: Fixed comment line
    """
    spaces = 0
    for i in comment_line:
        if i == " ":
            spaces += 1
        else:
            break
    line_prefix = " " * spaces
    comment_words = comment_line.split()
    common_words = set(comment_words) & keywords

    for word in common_words:
        # if keyword is 2nd word, it's not problematic
        pos = comment_words.index(word)
        if pos != 1:
            # add a new line before the keyword
            comment_words.insert(pos, "\n" + line_prefix + comment_prefix)

    return line_prefix + " ".join(comment_words)


def main():
    filename = sys.argv[1]
    with open(filename, "r") as f:
        filedata = f.read()

    index = Index.create()
    tu = index.parse(filename, args=["-x", "c++"])

    comment_token = [
        token for token in tu.cursor.get_tokens() if token.kind == TokenKind.COMMENT
    ]

    # adjust for indexing from 0 and not 1
    fix_offset = lambda x: x - 1
    # line already has the ending \n, no need to add 1 more
    file_print = lambda string: print(string, end="")

    debug = False
    if debug:
        FileOpener = lambda fn, **kwargs: open(fn, "r")
        # file_print = lambda string: True
        debug_print = lambda string: True
    else:
        FileOpener = FileInput
        debug_print = lambda string: True

    with FileOpener(filename, inplace=True) as file:
        token_idx = 0
        for line_num, line in enumerate(file):
            if token_idx == len(comment_token):
                # no more comments left
                file_print(line)
                continue

            token = comment_token[token_idx]
            start_line = fix_offset(token.extent.start.line)
            end_line = fix_offset(token.extent.end.line)

            # if this is after start, the comment starts from the very beginning
            if line_num == start_line:
                start_col = fix_offset(token.extent.start.column)
            else:
                start_col = None

            # if this is before the end, the comment continues until the end
            if line_num == end_line:
                end_col = fix_offset(token.extent.end.column)
            else:
                end_col = None

            if start_line > line_num:
                # don't edit this line
                file_print(line)
                continue
            if end_line == line_num:
                # the comment ends on this line
                token_idx += 1

            if start_col is not None:
                pre_comment = line[:start_col]
                file_print(pre_comment)
            else:
                pre_comment = ""

            comment = line[start_col:end_col]
            # search and replace here
            fixed_comment = fix_comment(DOXYGEN_SPECIAL_COMMAND_SET, comment)
            file_print(fixed_comment)
            if comment[-1] == "\n":
                # since split strips away newlines
                file_print("\n")

            if end_col is not None:
                post_comment = line[end_col:]
                file_print(post_comment)
            else:
                post_comment = ""

            debug_print(f"Pre: {pre_comment}, Comment: {comment}, Post: {post_comment}")
            debug_print(f"Fixed: {fixed_comment}")


if __name__ == "__main__":
    main()
