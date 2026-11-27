#! /usr/bin/env python

'''Usage: yml2xml (-h | --help | -v | --version | --test)
       yml2xml (- | <input-file>) [<output-file>]
       yml2xml (- | -i <input-file>) [-o <output-file>]
       yml2xml -s <input-data> [-o <output-file>]

yml2xml v0.1.0

Options:
  -i <input-file>, --input <input-file>   Input file, default: stdin
  -o <outpu-file>, --output <output-file> Output file, default: stdout
  -s <input-data>                         YML string to convert to XML
  --test                                  Test internal functionality
  -v, --version                           Version information
  -h, --help                              Show this help
'''

from dict2xml import dict2xml
from docopt import docopt
import sys
import yaml


def yml2xml(data=None):
    '''
    >>> yml2xml()
    '<all>None</all>'

    >>> yml2xml({'Hello': 'World'})
    '<all>\\n  <Hello>World</Hello>\\n</all>'
    '''
    return dict2xml(data, wrap='all', indent='  ')


def str2yml(string):
    '''
    >>> str2yml('')

    >>> str2yml("{'Hello': 'World'}")
    {'Hello': 'World'}
    '''
    return yaml.safe_load(string)


def get_input(filename, already_open=False):
    if already_open:
        return str2yml(filename)
    with open(filename, 'r') as inp_file:
        return str2yml(inp_file)


def post_output(data, stream):
    print(data, file=stream)


def main():
    arguments = docopt(__doc__, version='0.1.0')

    if arguments['--test']:
        import doctest
        doctest.testmod(verbose=True)
        return

    input_file = sys.stdin if arguments['-'] else arguments['--input']

    if arguments['-s']:
        data = str2yml(arguments['-s'])
    else:
        data = get_input(input_file, arguments['-'])

    with open(arguments['--output'], 'w') if arguments['--output'] \
            else sys.stdout as out_file:
        post_output(yml2xml(data), out_file)


if __name__ == '__main__':
    main()
