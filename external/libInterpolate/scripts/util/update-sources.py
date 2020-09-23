#! /usr/bin/env python

from pyparsing import *
import argparse
import os
import glob
import re
import shutil
import pprint
import fnmatch

arg_parser = argparse.ArgumentParser(description="Replaces source files in a CMakeLitst.txt file.")

arg_parser.add_argument("cmake_files", metavar="CMAKELISTS-FILE", nargs="*", default=['CMakeLists.txt'],help="Cmake files to be modified. Stdin is used if no files are given.")
arg_parser.add_argument("source_dirs", metavar="SOURCE-DIRECTORY", nargs="*", default=['./src/'],help="Directory to search for source files.")
arg_parser.add_argument("source_exts", metavar="EXT", nargs="*", default=['.hpp','.cpp','.h'],help="Source file extensions.")
args = arg_parser.parse_args()


class parser:
  cmd_name = Literal('target_sources')
  arguments = QuotedString( quoteChar='(', endQuoteChar=')',multiline=True )
  source_cmd_parser = WordStart() + cmd_name('cmd_name') + arguments('arguments')
  arguments_parser = OneOrMore(Word(printables))





for cmake_file in (args.cmake_files if len(args.cmake_files) > 0  else ('-',)):
  cmake_dir = os.path.dirname(cmake_file)
  if len(cmake_dir) == 0:
    cmake_dir = "."

  # find source files
  source_files = []
  for dir in args.source_dirs:
    for root, directories, filenames in os.walk(dir):
      for filename in filenames: 
        for ext in args.source_exts:
          if fnmatch.fnmatch(filename, '*'+ext):
              source_files.append( os.path.join(root,filename) )

  print "Detected Sources:"
  pprint.pprint(source_files)
  print "Directories Searched:"
  pprint.pprint(args.source_dirs)

  # parse cmake file
  with open(cmake_file,'r') as f:
    cmake_text = f.read()

  target_source_commands = parser.source_cmd_parser.searchString(cmake_text)
  if len(target_source_commands) > 1:
    print "ERROR: more than one target_source command found. I don't know what to do."

  target_source_arguments = parser.arguments_parser.parseString(target_source_commands[0]['arguments'])
  lines = []
  lines.append(target_source_commands[0]['cmd_name']+"(")
  i = 0
  while i < len(target_source_arguments) and re.search( "CMAKE_CURRENT_SOURCE_DIR", target_source_arguments[i]) == None:
    lines.append("  "+target_source_arguments[i])
    i += 1
  for source in source_files:
    lines.append( "  $<BUILD_INTERFACE:%s>"%re.sub("^./","${CMAKE_CURRENT_SOURCE_DIR}/",source,count=1) )
  lines.append(")")

  new = "\n".join(lines)
  old = originalTextFor(parser.source_cmd_parser).searchString(cmake_text)[0][0]
  cmake_text = cmake_text.replace(old,new)

  # make a copy of file
  shutil.copyfile( cmake_file, cmake_file+".bak" )
  with open(cmake_file,'w') as f:
    pass
    f.write(cmake_text)


  

