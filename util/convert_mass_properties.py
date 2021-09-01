import json

filecontent = ["#pragma once\n",
               "#include <Mahi/Util.hpp>\n\n"
               "constexpr double g = 9.80665;\n\n"]

mps = ["Pcx",
       "Pcy",
       "Pcz",
       "m",
       "Icxx",
       "Icyy",
       "Iczz",
       "Icxy",
       "Icxz",
       "Icyz"]

joint_mp = []

for i in range(4):

    with open("util/mass_properties/Joint"+str(i)+"_mass_properties.json") as read_file:
        content = read_file.read()
        read_file.close()

    joint_mp.append(json.loads(content))

    for mp in mps:
        filecontent.append("constexpr double " + mp + str(i) + " = " + str(joint_mp[i][mp]) + ";\n")

    filecontent.append("\n")
        
write_file = open("include/mass_properties.hpp","w")
write_file.writelines(filecontent)
write_file.close()