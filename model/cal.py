import py
#size="0.118 0.09 0.055"
a=2*0.118
b=2*0.09
c=2*0.055
mass = 3.9

Ixx = mass*(b*b+c*c)/12
Iyy = mass*(a*a+c*c)/12
Izz = mass*(a*a+b*b)/12

print("Ixx: ",Ixx," Iyy: ",Iyy," Izz: ",Izz)








