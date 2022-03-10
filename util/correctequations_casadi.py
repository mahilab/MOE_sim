import string

fileheader = ["#pragma once\n",
              "#include <vector>\n",
              "#include <casadi/casadi.hpp>\n",
              "#include <Mahi/Casadi/mass_properties.hpp>\n",
              "using namespace casadi;\n",
              "\n"]

variables = ["\tauto q0 = qs(0,0);\n",
             "\tauto q1 = qs(1,0);\n",
             "\tauto q2 = qs(2,0);\n",
             "\tauto q3 = qs(3,0);\n",
             "\tauto qd0 = qs(4,0);\n",
             "\tauto qd1 = qs(5,0);\n",
             "\tauto qd2 = qs(6,0);\n",
             "\tauto qd3 = qs(7,0);\n",
             "\n"]

filenames = ["M","V","G"]

for filename in filenames:
    print("Writing " + filename)
    with open("matlab/Equations/"+filename+".txt") as read_file:
        content = read_file.read()
        read_file.close()
    
    mat_name = filename
    content = string.replace(content,"  t","\tauto t")
    content = string.replace(content,"  A0","\t" + mat_name) # replacing Ao with matrix name
    content = string.replace(content,"][",",")               # fixing syntax around the matrix definition
    content = string.replace(content,"[","(")
    content = string.replace(content,"]",")")
    
    write_file = open("C:/Git/mahi-mpc/include/Mahi/Casadi/" + mat_name + ".hpp","w")
    write_file.writelines(fileheader)
    if mat_name != "G": 
        write_file.write("SX get_" + mat_name + "(const SX& qs){\n")
        write_file.write("\tSX " + mat_name + " = SX::zeros(4,4); \n\n")

    else: 
        write_file.write("SX get_" + mat_name + "(const SX& qs){\n")
        write_file.write("\tSX " + mat_name + " = SX::zeros(4,1); \n\n")

    write_file.writelines(variables)

    write_file.write(content + "\n")
    write_file.write("\treturn " + mat_name + ";\n}")

    write_file.close()