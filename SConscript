from building import *
import os

cwd = GetCurrentDir()

src = Glob('*.c')
src += Glob("qmc5883l/qmc5883l.c")
src += Glob("adlx345/adlx345.c")
src += Glob("itg3205/itg3205.c")
src += Glob("algorithm/imu_madgwick.c")
src += Glob("algorithm/imu_mahony.c")
src += Glob("algorithm/imu_complementary_filter.c")


CPPPATH = [cwd]
CPPPATH += [cwd + "/qmc5883l"]
CPPPATH += [cwd + "/adlx345"]
CPPPATH += [cwd + "/itg3205"]

group = DefineGroup('imu_sensor', src, depend = [''], CPPPATH = CPPPATH)

list = os.listdir(cwd)
for item in list:
    if os.path.isfile(os.path.join(cwd, item, 'SConscript')):
        group = group + SConscript(os.path.join(item, 'SConscript'))

Return('group')
