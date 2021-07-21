#!/usr/bin/python

import sys, re, os

def usage(arg0):
    print ("Usage: {} <input.template.pyx> <output.pyx>".format(arg0))
    sys.exit(-1)

wrapper_command_strings = {
        "ClassDefinitions" : """cdef class _%PARENT%_%MEMBER%_%TYPE%_VectorWrapper:
    cdef crbdl.%PARENT% *parent

    def __cinit__ (self, uintptr_t ptr):
        self.parent = <crbdl.%PARENT% *> ptr

    def __getitem__(self, key):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            return [%TYPE%.fromPointer (<uintptr_t> &(self.parent.%MEMBER%[i])) for i in xrange (*key.indices(len(self)))]
        else:
            return %TYPE%.fromPointer (<uintptr_t> &(self.parent.%MEMBER%[key]))

    def __setitem__(self, key, value):
        if isinstance( key, slice ) :
            #Get the start, stop, and step from the slice
            src_index = 0
            for i in xrange (*key.indices(len(self))):
                assert isinstance (value[src_index], %TYPE%), "Invalid type! Expected %TYPE%, but got " + str(type(value[src_index])) + "."
                self.parent.%MEMBER%[i] = (<%TYPE%> value[src_index]).thisptr[0]
                src_index = src_index + 1
        else:
            assert isinstance (value, %TYPE%), "Invalid type! Expected %TYPE%, but got " + str(type(value)) + "."
            self.parent.%MEMBER%[key] = (<%TYPE%> value).thisptr[0]

    def __len__(self):
        return self.parent.%MEMBER%.size()
""",

    "MemberDefinitions" : """    cdef _%PARENT%_%MEMBER%_%TYPE%_VectorWrapper %MEMBER%""",

    "CInitCode" : """        self.%MEMBER% = _%PARENT%_%MEMBER%_%TYPE%_VectorWrapper (<uintptr_t> self.thisptr)""",

    
    "AddProperty" : """    property %MEMBER%:
        def __get__ (self):
            vector_size = self.thisptr.%MEMBER%.size()
            prop = [%TYPE% (address=<uintptr_t> &(self.thisptr.%MEMBER%[i])) for i in range(vector_size)]
            return prop
"""
}


def parse_line (line_str):
    command = ""
    args = {}

    # remove comments
    line_str = line_str.split("#")[0]

    wrapper_line_str_match = re.search ("%VectorWrapper(\S*)\s*\((.*)\).*%", line_str)
    if (wrapper_line_str_match):
        command = wrapper_line_str_match.group(1)
        arg_str = wrapper_line_str_match.group(2)
        arg_match = re.findall("(\s*(\S*)\s*=\s*(\w*)\s*,?)", arg_str)
        if len(arg_match) > 0:
            for arg in arg_match:
                if len(arg) != 3:
                    print ("Invalid command args at line_str {}".format
                            (line_number))
                    sys.exit(-1)

                args[arg[1]] = arg[2]

        return command, args

    return False, None

if __name__ == "__main__":
    if len(sys.argv) != 3:
        usage (sys.argv[0])

    infilename = sys.argv[1]
    outfilename = sys.argv[2]

#    print ("Processing {} to generate {}".format (infilename, outfilename))
    infile = open(infilename)
    outfile = open(outfilename, "w")
    outfile.write ("""# WARNING! 
#
# This file was automatically created from {} using {}.
# Do not modify this file directly. Edit original source instead!!

""".format (os.path.basename(infilename), os.path.basename(sys.argv[0])))

    template = infile.read()
    template_lines = template.split ("\n")

     # find the classes that will contain generated code
    generated_parent_classes = []
    generated_parent_members = {}
    for line_number, line_str in enumerate (template_lines):
        command, args = parse_line (line_str)
        if command:
            if args["PARENT"] not in generated_parent_classes:
                generated_parent_classes.append (args["PARENT"])
                generated_parent_members[args["PARENT"]] = []

            if command=="AddProperty":
               generated_parent_members[args["PARENT"]].append ({
                   "TYPE": args["TYPE"],
                   "MEMBER": args["MEMBER"]
                   })


    # generate code
    for line_number, line_str in enumerate (template_lines):
        command, args = parse_line (line_str)
        if not command:
            outfile.write (line_str + "\n")
        else:
            if command in wrapper_command_strings.keys():
                parent = args["PARENT"]
                if command == "AddProperty":
                        content_type = args["TYPE"]
                        member_name = args["MEMBER"]
                        command_code = wrapper_command_strings[command][:]
                        command_code = command_code.replace (
                                "%PARENT%", parent).replace (
                                "%MEMBER%", member_name).replace (
                                "%TYPE%", content_type)
                        outfile.write (command_code + "\n")
                else:
                    for member in generated_parent_members[parent]:
                        content_type = member["TYPE"]
                        member_name = member["MEMBER"]
                        command_code = wrapper_command_strings[command][:]
                        
                        command_code = command_code.replace (
                                "%PARENT%", parent).replace (
                                "%MEMBER%", member_name).replace (
                                "%TYPE%", content_type)
                        outfile.write (command_code + "\n")
