from building import *
import os

cwd = GetCurrentDir()

src = Glob('*.c')
src += Glob("qmc5883l/qmc5883l.c")
src += Glob("algorithm/imu_madgwick.c")
src += Glob("algorithm/imu_mahony.c")

CPPPATH = [cwd]
CPPPATH += [cwd + "/qmc5883l"]

group = DefineGroup('imu_sensor', src, depend = [''], CPPPATH = CPPPATH)

list = os.listdir(cwd)
for item in list:
    if os.path.isfile(os.path.join(cwd, item, 'SConscript')):
        group = group + SConscript(os.path.join(item, 'SConscript'))

Return('group')
