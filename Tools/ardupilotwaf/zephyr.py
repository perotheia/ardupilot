# encoding: utf-8

"""
WAF tool for Zephyr / West builds (AP_HAL_Zephyr).

WAF compiles ArduPilot → static libraries (.a), then
Zephyr's CMake build links them with the Zephyr kernel to produce zephyr.elf.

Pre-build phase
---------------
1. cmake configure for AP_HAL_Zephyr/app — generates all Zephyr headers
   (autoconf.h, devicetree_generated.h, …).
2. cmake builds the 'showinc' target → writes zephyr_build/includes.list
   containing every Zephyr include directory.
3. load_generated_includes task reads includes.list and prepends the paths
   to bld.env.INCLUDES before any WAF C++ compilation task runs.

Post-link phase
---------------
4. zephyr_firmware feature hook: after WAF links the vehicle static library
   (libarducopter.a etc.), cmake --build zephyr_build --target all is run.
   cmake links the WAF .a files with the Zephyr kernel → zephyr.elf.

Which program is linked
-----------------------
The program is the vehicle task generator that this build really posts
(its name is `bin/<program>`), whatever the waf command was: `./waf rover`,
`./waf heli`, `./waf build --targets bin/ardurover`. Its own static library
and the vehicle library it uses are handed to cmake as ARDUPILOT_PROGRAM_LIB
and ARDUPILOT_VEHICLE_LIB, with ARDUPILOT_CMD = the program name; when the
cmake cache holds other values (pre_build() runs before the targets are
known and can only guess) cmake is re-configured before the final link.
With several programs in one build (`./waf copter` builds arducopter and
arducopter-heli, `./waf build` every vehicle) the one the command names is
linked; when that is ambiguous nothing is linked and the build fails with
the list, instead of leaving a stale or wrong zephyr.elf.

Board files as dependencies
---------------------------
hwdef/<board>/hwdef.h (-include) and the generated autoconf.h (-imacros,
from the board .conf) are in every object's gcc dependency file (waf's
gccdeps), and ninja re-runs the Zephyr configuration when the .conf or the
overlay changes: an edit of any of them rebuilds what depends on it (in
practice every object) without a clean build.

"""

import glob
import os
import re
import sys
from collections import OrderedDict

from waflib import Logs, Task
from waflib.TaskGen import before_method, after_method, feature


@feature('zephyr_ap_library', 'zephyr_ap_program')
@before_method('process_source')
def zephyr_dynamic_env(self):
    if self.bld.cmd == 'list':
        return


def configure(cfg):
    """Called by boards.zephyr.configure_env() via cfg.load('zephyr')."""
    bldnode = cfg.bldnode.make_node(cfg.variant)
    env = cfg.env

    ardupilot_root = cfg.srcnode.abspath()

    zephyr_base = os.environ.get('ZEPHYR_BASE', '')
    if not zephyr_base:
        auto = cfg.srcnode.make_node('modules/zephyr').abspath()
        if os.path.isdir(auto):
            zephyr_base = auto
    if not zephyr_base:
        cfg.fatal(
            'ZEPHYR_BASE not set and modules/zephyr submodule not found at '
            + cfg.srcnode.make_node('modules/zephyr').abspath()
        )
    env.ZEPHYR_BASE = zephyr_base

    # Locate the AP_HAL_Zephyr cmake application.
    app_node = cfg.srcnode.find_dir('libraries/AP_HAL_Zephyr/app')
    if app_node is None:
        cfg.fatal(
            'Could not locate libraries/AP_HAL_Zephyr/app in the ArduPilot '
            'source tree. Expected at: '
            + cfg.srcnode.make_node('libraries/AP_HAL_Zephyr/app').abspath()
        )
    env.ZEPHYR_APP_DIR = app_node.abspath()

    # Build output directory for the inner CMake / West build.
    env.ZEPHYR_BUILD_DIR = bldnode.make_node('zephyr_build').abspath()

    board_name = cfg.env.BOARD
    hwdef_path = os.path.join(
        ardupilot_root,
        'libraries/AP_HAL_Zephyr/hwdef',
        board_name,
        'hwdef.dat',
    )
    if not os.path.exists(hwdef_path):
        cfg.fatal('AP_HAL_Zephyr: hwdef.dat not found at %s' % hwdef_path)
    env.ZEPHYR_BOARD = ''
    with open(hwdef_path) as fh:
        for line in fh:
            m = re.match(r'\s*define\s+ZEPHYR_BOARD\s+(\S+)', line)
            if m:
                env.ZEPHYR_BOARD = m.group(1)
                break
    if not env.ZEPHYR_BOARD:
        cfg.fatal(
            'AP_HAL_Zephyr: %s must contain `define ZEPHYR_BOARD <target>` '
            '(Zephyr board target, e.g. `nucleo_h743zi/stm32h743xx` or '
            '`mpfs_icicle/polarfire/u54`)' % hwdef_path
        )

    boards_root = cfg.srcnode.make_node('libraries/AP_HAL_Zephyr/boards').abspath()
    _matches = glob.glob(os.path.join(boards_root, '*', board_name))
    if len(_matches) != 1:
        cfg.fatal(
            'AP_HAL_Zephyr: expected exactly one boards/*/%s directory, '
            'found %d: %r' % (board_name, len(_matches), _matches)
        )
    env.ZEPHYR_BOARD_HAL_DIR = _matches[0]

    # DTS overlay — lives in the board HAL dir.  Exactly one .overlay must
    # be present.
    _overlays = glob.glob(os.path.join(env.ZEPHYR_BOARD_HAL_DIR, '*.overlay'))
    if len(_overlays) != 1:
        cfg.fatal(
            'AP_HAL_Zephyr: expected exactly one *.overlay in %s, found %d: %r'
            % (env.ZEPHYR_BOARD_HAL_DIR, len(_overlays), _overlays)
        )
    env.DTC_OVERLAY_FILE = _overlays[0]

    # Board-specific Kconfig fragment — same base name, .conf extension.
    board_conf = env.DTC_OVERLAY_FILE[:-len('.overlay')] + '.conf'
    env.ZEPHYR_BOARD_CONF = board_conf if os.path.exists(board_conf) else ''

    # Build variants: an extra devicetree overlay and Kconfig fragment from the
    # environment, applied after the board's own (e.g. a sensor set that must
    # not be enabled in the default build).
    _extra_dts = os.environ.get('ZEPHYR_EXTRA_DTC_OVERLAY', '')
    _extra_conf = os.environ.get('ZEPHYR_EXTRA_CONF', '')
    for _f in (_extra_dts, _extra_conf):
        if _f and not os.path.exists(_f):
            cfg.fatal('AP_HAL_Zephyr: build variant file not found: %s' % _f)
    if _extra_dts:
        env.DTC_OVERLAY_FILE = env.DTC_OVERLAY_FILE + ';' + _extra_dts
    if _extra_conf:
        env.ZEPHYR_BOARD_CONF = ';'.join(f for f in (env.ZEPHYR_BOARD_CONF, _extra_conf) if f)

    env.CMAKE_PROJECT_NAME   = 'ardupilot-zephyr'
    env.Zephyr_DIR           = os.path.join(zephyr_base, 'share/zephyr-package/cmake')
    _workspace_root = os.path.dirname(ardupilot_root)

    _env_modules = os.environ.get('ZEPHYR_MODULES', '')
    if _env_modules:
        env.ZEPHYR_MODULES = _env_modules
    else:
        _module_parents = [
            os.path.join(ardupilot_root, 'modules/modules'),
            os.path.join(_workspace_root, 'modules/modules'),
        ]
        _module_roots = []
        for _parent in _module_parents:
            if not os.path.isdir(_parent):
                continue
            for _yml in glob.glob(os.path.join(_parent, '**', 'zephyr', 'module.yml'),
                                  recursive=True):
                _root = os.path.dirname(os.path.dirname(_yml))
                if _root not in _module_roots:
                    _module_roots.append(_root)
        env.ZEPHYR_MODULES = ';'.join(_module_roots)
    _module_yml = os.path.join(env.ZEPHYR_BOARD_HAL_DIR, 'zephyr', 'module.yml')
    env.ZEPHYR_EXTRA_MODULES = env.ZEPHYR_BOARD_HAL_DIR if os.path.exists(_module_yml) else ''

    _sdk = os.environ.get('ZEPHYR_SDK_INSTALL_DIR', '').rstrip('/')
    _cross = os.environ.get('CROSS_COMPILE', '')
    if _sdk:
        env.ZEPHYR_TOOLCHAIN_VARIANT = 'zephyr'
        env.ZEPHYR_SDK_INSTALL_DIR = _sdk
        env.CROSS_COMPILE = ''
        env.CROSS_COMPILE_TOOLCHAIN_PATH = ''
    elif _cross:
        env.ZEPHYR_TOOLCHAIN_VARIANT = 'cross-compile'
        env.ZEPHYR_SDK_INSTALL_DIR = ''
        env.CROSS_COMPILE = _cross
        env.CROSS_COMPILE_TOOLCHAIN_PATH = os.environ.get(
            'CROSS_COMPILE_TOOLCHAIN_PATH',
            os.path.dirname(os.path.dirname(_cross.rstrip('/'))))
    else:
        cfg.fatal(
            'AP_HAL_Zephyr: set ZEPHYR_SDK_INSTALL_DIR (Zephyr SDK) or '
            'CROSS_COMPILE (e.g. export CROSS_COMPILE=arm-zephyr-eabi-  or  '
            'export CROSS_COMPILE=riscv64-unknown-elf-)'
        )
    _venv_py = os.path.join(_workspace_root, 'sdk/.venv/bin/python3')
    env.Python3_EXECUTABLE = os.environ.get(
        'PYTHON3', _venv_py if os.path.isfile(_venv_py) else sys.executable
    )

    env.CMAKE_TRY_COMPILE_TARGET_TYPE = 'STATIC_LIBRARY'
    env.CMAKE_C_STANDARD_LIBRARIES    = ''
    env.CMAKE_CXX_STANDARD_LIBRARIES  = ''
    env.AP_PROGRAM_FEATURES += ['zephyr_ap_program']

    cfg.load('cmake')

    cfg.msg('ZEPHYR_BASE',               env.ZEPHYR_BASE)
    cfg.msg('ZEPHYR_APP_DIR',            env.ZEPHYR_APP_DIR)
    cfg.msg('ZEPHYR_BUILD_DIR',          env.ZEPHYR_BUILD_DIR)
    cfg.msg('ZEPHYR_BOARD',              env.ZEPHYR_BOARD)
    cfg.msg('ZEPHYR_BOARD_HAL_DIR',      env.ZEPHYR_BOARD_HAL_DIR)
    cfg.msg('ZEPHYR_TOOLCHAIN_VARIANT',  env.ZEPHYR_TOOLCHAIN_VARIANT)
    if env.ZEPHYR_SDK_INSTALL_DIR:
        cfg.msg('ZEPHYR_SDK_INSTALL_DIR', env.ZEPHYR_SDK_INSTALL_DIR)
    else:
        cfg.msg('CROSS_COMPILE',         env.CROSS_COMPILE)
    cfg.msg('DTC_OVERLAY_FILE',          env.DTC_OVERLAY_FILE)
    cfg.msg('Zephyr_DIR',                env.Zephyr_DIR)
    cfg.msg('ZEPHYR_MODULES',            env.ZEPHYR_MODULES)


def pre_build(bld):
    cmake_vars = OrderedDict()
    cmake_vars['BOARD']                          = bld.env.ZEPHYR_BOARD
    cmake_vars['ZEPHYR_BASE']                    = bld.env.ZEPHYR_BASE
    cmake_vars['ZEPHYR_TOOLCHAIN_VARIANT']       = bld.env.ZEPHYR_TOOLCHAIN_VARIANT
    # Zephyr SDK mode: hand CMake the SDK dir and let it resolve the per-arch
    # compiler; do NOT send CROSS_COMPILE* (empty values would override the SDK
    # variant's native resolution).
    if bld.env.ZEPHYR_SDK_INSTALL_DIR:
        cmake_vars['ZEPHYR_SDK_INSTALL_DIR']     = bld.env.ZEPHYR_SDK_INSTALL_DIR
    if bld.env.CROSS_COMPILE:
        cmake_vars['CROSS_COMPILE']              = bld.env.CROSS_COMPILE
        cmake_vars['CROSS_COMPILE_TOOLCHAIN_PATH'] = bld.env.CROSS_COMPILE_TOOLCHAIN_PATH
    cmake_vars['DTC_OVERLAY_FILE']               = bld.env.DTC_OVERLAY_FILE
    cmake_vars['Zephyr_DIR']                     = bld.env.Zephyr_DIR
    cmake_vars['Python3_EXECUTABLE']             = bld.env.Python3_EXECUTABLE
    if bld.env.ZEPHYR_MODULES:
        cmake_vars['ZEPHYR_MODULES']             = bld.env.ZEPHYR_MODULES
    if bld.env.ZEPHYR_EXTRA_MODULES:
        cmake_vars['ZEPHYR_EXTRA_MODULES']       = bld.env.ZEPHYR_EXTRA_MODULES
    cmake_vars['ZEPHYR_BOARD_HAL_DIR']           = bld.env.ZEPHYR_BOARD_HAL_DIR
    cmake_vars['CMAKE_TRY_COMPILE_TARGET_TYPE']  = bld.env.CMAKE_TRY_COMPILE_TARGET_TYPE
    cmake_vars['CMAKE_C_STANDARD_LIBRARIES']     = bld.env.CMAKE_C_STANDARD_LIBRARIES
    cmake_vars['CMAKE_CXX_STANDARD_LIBRARIES']   = bld.env.CMAKE_CXX_STANDARD_LIBRARIES
    if bld.env.ZEPHYR_BOARD_CONF:
        cmake_vars['OVERLAY_CONFIG']             = bld.env.ZEPHYR_BOARD_CONF
    cmake_vars['ARDUPILOT_LIB']  = bld.bldnode.find_or_declare('lib/').abspath()
    cmake_vars['ARDUPILOT_BIN']  = bld.bldnode.find_or_declare('lib/bin').abspath()
    # The program is not known yet (the targets are resolved later): a guess
    # from the command line, in the same form zephyr_firmware() uses, so that
    # the usual build needs no second configure.
    cmake_vars.update(_program_cmake_vars(bld, _guess_program(bld), None, None))

    app_src_node = bld.root.find_dir(bld.env.ZEPHYR_APP_DIR)
    if app_src_node is None:
        bld.fatal('ZEPHYR_APP_DIR not found: %s' % bld.env.ZEPHYR_APP_DIR)

    zephyr_cmake = bld.cmake(
        name='ardupilot-zephyr',
        cmake_src=app_src_node,
        cmake_bld='zephyr_build',
        cmake_vars=cmake_vars,
    )

    showinc_tg = zephyr_cmake.build('showinc', target='zephyr_build/includes.list')
    showinc_tg.post()
    showdefs_tg = zephyr_cmake.build('showdefs', target='zephyr_build/compile_defs.list')
    showdefs_tg.post()
    # ABI flags (-march / -mabi / -mcmodel / -mcpu / -mfpu / -mfloat-abi / -mthumb)
    # are declared by the board HAL CMakeLists.txt via AP_BOARD_ABI_CFLAGS.

    class load_generated_includes(Task.Task):
        always_run = True

        def run(tsk):
            bld = tsk.generator.bld
            inc_node = bld.bldnode.find_or_declare('zephyr_build/includes.list')
            includes = [p for p in inc_node.read().split() if p]
            bld.env.prepend_value('INCLUDES', includes)
            defs_node = bld.bldnode.find_or_declare('zephyr_build/compile_defs.list')
            dflags = ['-D' + d for d in defs_node.read().split() if d]
            bld.env.prepend_value('CXXFLAGS', dflags)
            bld.env.prepend_value('CFLAGS', dflags)
            rsp_node = bld.bldnode.find_or_declare('zephyr_build/ap_board_abi.rsp')
            rsp_arg = '@' + rsp_node.abspath()
            bld.env.prepend_value('CFLAGS',   [rsp_arg])
            bld.env.prepend_value('CXXFLAGS', [rsp_arg])

    tsk = load_generated_includes(env=bld.env)
    tsk.set_inputs([
        bld.bldnode.find_or_declare('zephyr_build/includes.list'),
        bld.bldnode.find_or_declare('zephyr_build/compile_defs.list'),
        bld.bldnode.find_or_declare('zephyr_build/ap_board_abi.rsp'),
    ])
    bld.add_to_group(tsk)


# waf command -> the vehicle program it means ('rover' builds bin/ardurover)
_CMD_PROGRAMS = {'build': 'arducopter', 'copter': 'arducopter', 'heli': 'arducopter-heli',
                 'plane': 'arduplane', 'rover': 'ardurover', 'sub': 'ardusub'}
# program -> the vehicle library it links (only for the guess in pre_build();
# the final link takes it from the task generator)
_PROGRAM_VEHICLE_LIBS = {'arducopter': 'ArduCopter_libs', 'arducopter-heli': 'ArduCopter_libs',
                         'arduplane': 'ArduPlane_libs', 'ardurover': 'Rover_libs',
                         'ardusub': 'ArduSub_libs'}


def _guess_program(bld):
    """The program a command line asks for, before the targets are resolved."""
    for t in (bld.targets or '').split(','):
        t = t.strip()
        if t.startswith('bin/'):
            return t[len('bin/'):]
    return _CMD_PROGRAMS.get(bld.cmd, bld.cmd)


def _program_cmake_vars(bld, program, program_lib, vehicle_lib):
    """ARDUPILOT_CMD and the two archives of `program` (see app/CMakeLists.txt).

    Paths not given are the ones waf uses for a vehicle program built as a
    static library: lib/bin/lib<program>.a and lib/lib<Vehicle>_libs.a."""
    lib_dir = bld.bldnode.find_or_declare('lib/').abspath()
    if program_lib is None:
        program_lib = os.path.join(lib_dir, 'bin', 'lib%s.a' % program)
    if vehicle_lib is None:
        _name = _PROGRAM_VEHICLE_LIBS.get(program)
        vehicle_lib = os.path.join(lib_dir, 'lib%s.a' % _name) if _name else ''
    return OrderedDict([
        ('ARDUPILOT_CMD', program),
        ('ARDUPILOT_PROGRAM_LIB', program_lib),
        ('ARDUPILOT_VEHICLE_LIB', vehicle_lib),
    ])


def _cmake_cache_get(cmake_bld, key):
    try:
        with open(os.path.join(cmake_bld, 'CMakeCache.txt')) as fh:
            for line in fh:
                m = re.match(r'%s(?::[A-Z]+)?=(.*)$' % re.escape(key), line.rstrip('\n'))
                if m:
                    return m.group(1)
    except EnvironmentError:
        pass
    return None


def _select_program(bld, programs):
    """The one program to link out of the posted vehicle programs, or None."""
    if len(programs) == 1:
        return programs[0]
    wanted = _guess_program(bld)
    for tg in programs:
        if tg.program_name == wanted:
            return tg
    return None


def _cmake_final_link(bld):
    import subprocess
    programs = getattr(bld, 'zephyr_programs', [])
    if not programs:
        return
    tg = _select_program(bld, programs)
    if tg is None:
        bld.fatal('AP_HAL_Zephyr: %d vehicle programs were built (%s) and the command '
                  'names none of them: zephyr.elf NOT linked. Build one program: '
                  './waf <vehicle>  or  ./waf build --targets bin/<program>'
                  % (len(programs), ', '.join(t.name for t in programs)))
    if len(programs) > 1:
        Logs.warn('AP_HAL_Zephyr: %d vehicle programs built, zephyr.elf is %s'
                  % (len(programs), tg.name))

    # the archives of this program, from its task generator
    program_lib = tg.link_task.outputs[0].abspath()
    vehicle_lib = ''
    for use in tg.to_list(getattr(tg, 'use', [])):
        try:
            use_tg = bld.get_tgen_by_name(use)
        except Exception:
            continue
        if getattr(use_tg, 'link_task', None) is not None:
            vehicle_lib = use_tg.link_task.outputs[0].abspath()
            break
    wanted = _program_cmake_vars(bld, tg.program_name, program_lib, vehicle_lib)

    cmake_bld = bld.bldnode.make_node('zephyr_build').abspath()
    cmake_bin = bld.env.get_flat('CMAKE')
    if any(_cmake_cache_get(cmake_bld, k) != v for k, v in wanted.items()):
        Logs.info('AP_HAL_Zephyr: configuring the Zephyr link for %s' % tg.name)
        ret = subprocess.call([cmake_bin] + ['-D%s=%s' % kv for kv in wanted.items()]
                              + [cmake_bld])
        if ret != 0:
            bld.fatal('cmake configure for %s failed (exit %d)' % (tg.name, ret))
    ret = subprocess.call([cmake_bin, '--build', cmake_bld, '--target', 'all'])
    if ret != 0:
        bld.fatal('cmake final link failed (exit %d)' % ret)
    # hwdef/<board>/defaults.parm -> the embedded parameter defaults
    # (AP_PARAM_MAX_EMBEDDED_PARAM > 0 in hwdef.h), like ChibiOS boards
    _defaults = os.path.join(bld.srcnode.abspath(), 'libraries/AP_HAL_Zephyr/hwdef',
                             bld.env.BOARD, 'defaults.parm')
    if os.path.exists(_defaults):
        _elf = os.path.join(cmake_bld, 'zephyr', 'zephyr.elf')
        _tool = os.path.join(bld.srcnode.abspath(), 'Tools/scripts/apj_tool.py')
        ret = subprocess.call([sys.executable, _tool, _elf, '--set-file', _defaults])
        if ret != 0:
            bld.fatal('apj_tool --set-file %s failed (exit %d)' % (_defaults, ret))


@feature('zephyr_ap_program')
@after_method('process_source')
def zephyr_firmware(self):
    """
    Runs for every program task generator this build posts. The vehicle
    programs (bin/<program>) are collected; one post-build function links
    zephyr.elf (Zephyr kernel + WAF-compiled ArduPilot) from the selected one.
    """
    if getattr(self, 'program_dir', None) != 'bin' or not getattr(self, 'link_task', None):
        return  # examples, tools, tests: no firmware

    self.link_task.always_run = True

    bld = self.bld
    if not hasattr(bld, 'zephyr_programs'):
        bld.zephyr_programs = []
        bld.add_post_fun(_cmake_final_link)
    bld.zephyr_programs.append(self)
