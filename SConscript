from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add lsm303agr src files.
if GetDepend('PKG_USING_LSM303AGR_ACCE'):
    src += Glob('sensor_st_lsm303agr_acc.c')

if GetDepend('PKG_USING_LSM303AGR_MAG'):
    src += Glob('sensor_st_lsm303agr_mag.c')

src += Glob('libraries/lsm303agr.c')
src += Glob('libraries/lsm303agr_reg.c')

# add lsm303agr include path.
path  = [cwd, cwd + '/libraries']

# add src and include to group.
group = DefineGroup('lsm303agr', src, depend = ['PKG_USING_LSM303AGR'], CPPPATH = path)

Return('group')
