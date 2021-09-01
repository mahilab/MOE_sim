import string

fileheader = ["#pragma once\n",
              "#include <vector>\n",
              "#include <Eigen/Dense>\n",
              "#include <mass_properties.hpp>\n",
              "using namespace mahi::util;\n",
              "\n"]

variables = ["\tconst double q0 = qs[0];\n",
             "\tconst double q1 = qs[1];\n",
             "\tconst double q2 = qs[2];\n",
             "\tconst double q3 = qs[3];\n",
             "\tconst double qd0 = qs[4];\n",
             "\tconst double qd1 = qs[5];\n",
             "\tconst double qd2 = qs[6];\n",
             "\tconst double qd3 = qs[7];\n",
             "\n"]

filenames = ["M","V","G"]

for filename in filenames:
    with open("matlab/Equations/"+filename+".txt") as read_file:
        content = read_file.read()
        read_file.close()
    
    mat_name = filename
    content = string.replace(content,"  t","\tconst double t")
    content = string.replace(content,"  A0","\t" + mat_name) # replacing Ao with matrix name
    content = string.replace(content,"][",",")               # fixing syntax around the matrix definition
    content = string.replace(content,"[","(")
    content = string.replace(content,"]",")")
    
    write_file = open("include/" + mat_name + ".hpp","w")
    write_file.writelines(fileheader)
    if mat_name != "G": 
        write_file.write("inline Eigen::MatrixXd get_" + mat_name + "(const std::vector<double>& qs){\n")
        write_file.write("\tEigen::MatrixXd " + mat_name + " = Eigen::MatrixXd::Zero(4,4); \n\n")

    else: 
        write_file.write("inline Eigen::VectorXd get_" + mat_name + "(const std::vector<double>& qs){\n")
        write_file.write("\tEigen::VectorXd " + mat_name + " = Eigen::VectorXd::Zero(4); \n\n")

    write_file.writelines(variables)

    write_file.write(content + "\n")
    write_file.write("\treturn " + mat_name + ";\n}")

    write_file.close()