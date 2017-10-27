#!/usr/bin/python
"""
Parses a folder's .cpp files and prints all the translatable error messages from switcher to scenic.

Usage: ./update_translatable_errors [OPTION]

Argument(s):
-d, --dir[=DIRECTORY]    (Mandatory) Indicates which folder to parse to create translatable error strings list.
"""
import sys
import os
import getopt
import re


error_pattern = 'message\(\s*"ERROR:(.*)".*\);'
error_regex = re.compile(error_pattern)


def parse_error_messages(file):
    with open(file, 'r') as f:
        file_text = f.read()
        matches = error_regex.findall(file_text)
        for match in matches:
            print match


def usage():
    print __doc__


if __name__ == '__main__':
    directory = ''

    try:
        opts, args = getopt.getopt(sys.argv[1:], "d", ["dir="])
    except getopt.GetoptError as err:
        print err
        usage()
        exit(2)
    for o, a in opts:
        if o in ["-d", "--dir"]:
            if os.path.exists(a):
                directory = a
            else:
                usage()
                sys.stderr.write('Provided folder {} does not exist.'.format(a))
                exit(2)
        else:
            usage()
            sys.stderr.write('Unhandled option.')
            exit(2)

    if not directory:
        sys.stderr.write('Directory is mandatory.')
        usage()
        exit(2)

    os.chdir(directory)
    for root, dirs, files in os.walk(directory):
        for file in files:
            target_file = os.path.join(root, file)
            if target_file.endswith('.cpp'):
                parse_error_messages(target_file)
