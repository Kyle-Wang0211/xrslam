#!/usr/bin/env python3
# [pw] Standalone ELF/Mach-O dynamic-symbol auditor.
# Reason it exists: the acceptance criterion for symbol hiding must be BINARY,
# and we must not depend on readelf/nm being installed (macOS ships neither
# GNU readelf nor GNU nm by default).  Pure stdlib struct parsing.
import struct, sys, json

def parse_elf(data):
    assert data[:4] == b'\x7fELF', 'not ELF'
    ei_class = data[4]; ei_data = data[5]
    end = '<' if ei_data == 1 else '>'
    is64 = ei_class == 2
    if is64:
        (e_type, e_machine, e_version, e_entry, e_phoff, e_shoff, e_flags,
         e_ehsize, e_phentsize, e_phnum, e_shentsize, e_shnum,
         e_shstrndx) = struct.unpack_from(end + 'HHIQQQIHHHHHH', data, 16)
    else:
        (e_type, e_machine, e_version, e_entry, e_phoff, e_shoff, e_flags,
         e_ehsize, e_phentsize, e_phnum, e_shentsize, e_shnum,
         e_shstrndx) = struct.unpack_from(end + 'HHIIIIIHHHHHH', data, 16)
    secs = []
    for i in range(e_shnum):
        off = e_shoff + i * e_shentsize
        if is64:
            f = struct.unpack_from(end + 'IIQQQQIIQQ', data, off)
        else:
            f = struct.unpack_from(end + 'IIIIIIIIII', data, off)
        secs.append(dict(name=f[0], type=f[1], flags=f[2], addr=f[3],
                         offset=f[4], size=f[5], link=f[6], info=f[7],
                         align=f[8], entsize=f[9]))
    shstr = secs[e_shstrndx]
    def strat(tab_off, idx):
        e = data.index(b'\x00', tab_off + idx)
        return data[tab_off + idx:e].decode('utf-8', 'replace')
    for s in secs:
        s['sname'] = strat(shstr['offset'], s['name'])
    out = {'format': 'ELF', 'sections': [s['sname'] for s in secs]}
    # program headers -> max p_align (16KB page alignment check)
    aligns = []
    for i in range(e_phnum):
        off = e_phoff + i * e_phentsize
        if is64:
            p_type, p_flags, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_align = \
                struct.unpack_from(end + 'IIQQQQQQ', data, off)
        else:
            p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align = \
                struct.unpack_from(end + 'IIIIIIII', data, off)
        if p_type == 1:  # PT_LOAD
            aligns.append(p_align)
    out['pt_load_aligns'] = aligns
    out['max_pt_load_align'] = max(aligns) if aligns else 0
    dynsym = next((s for s in secs if s['sname'] == '.dynsym'), None)
    if dynsym is None:
        out['error'] = 'no .dynsym'
        return out
    strtab = secs[dynsym['link']]
    n = dynsym['size'] // dynsym['entsize']
    syms = []
    for i in range(n):
        off = dynsym['offset'] + i * dynsym['entsize']
        if is64:
            st_name, st_info, st_other, st_shndx, st_value, st_size = \
                struct.unpack_from(end + 'IBBHQQ', data, off)
        else:
            st_name, st_value, st_size, st_info, st_other, st_shndx = \
                struct.unpack_from(end + 'IIIBBH', data, off)
        nm = strat(strtab['offset'], st_name)
        syms.append(dict(name=nm, shndx=st_shndx, bind=st_info >> 4,
                         type=st_info & 0xf, other=st_other & 0x3))
    out['symbols'] = syms
    return out

def parse_macho(data):
    magic = struct.unpack_from('<I', data, 0)[0]
    base = 0
    if magic in (0xcafebabe, 0xbebafeca):  # fat
        nfat = struct.unpack_from('>I', data, 4)[0]
        base = struct.unpack_from('>I', data, 8 + 8)[0]  # first arch offset
        magic = struct.unpack_from('<I', data, base)[0]
    is64 = magic == 0xfeedfacf
    end = '<'
    if is64:
        _, cputype, cpusub, filetype, ncmds, sizeofcmds, flags, _r = \
            struct.unpack_from(end + 'IiiIIIII', data, base)
        off = base + 32
    else:
        _, cputype, cpusub, filetype, ncmds, sizeofcmds, flags = \
            struct.unpack_from(end + 'IiiIIII', data, base)
        off = base + 28
    out = {'format': 'Mach-O', 'symbols': []}
    symoff = nsyms = stroff = 0
    for _ in range(ncmds):
        cmd, cmdsize = struct.unpack_from(end + 'II', data, off)
        if cmd == 0x2:  # LC_SYMTAB
            _, _, symoff, nsyms, stroff, strsize = struct.unpack_from(end + 'IIIIII', data, off)
        off += cmdsize
    syms = []
    esz = 16 if is64 else 12
    for i in range(nsyms):
        o = base + symoff + i * esz
        if is64:
            n_strx, n_type, n_sect, n_desc, n_value = struct.unpack_from(end + 'IBBHQ', data, o)
        else:
            n_strx, n_type, n_sect, n_desc, n_value = struct.unpack_from(end + 'IBBhI', data, o)
        e = data.index(b'\x00', base + stroff + n_strx)
        nm = data[base + stroff + n_strx:e].decode('utf-8', 'replace')
        N_EXT = 0x01
        N_TYPE = n_type & 0x0e
        ext = bool(n_type & N_EXT)
        undef = (N_TYPE == 0x0)
        if ext:  # only external symbols are the "dynamic" surface
            syms.append(dict(name=nm, shndx=(0 if undef else 1), bind=1,
                             type=0, other=0))
    out['symbols'] = syms
    return out

def main(path):
    data = open(path, 'rb').read()
    if data[:4] == b'\x7fELF':
        r = parse_elf(data)
    else:
        r = parse_macho(data)
    syms = r.get('symbols', [])
    # [pw] Mach-O 给 C 符号加一个前导下划线;归一化后两种格式用同一套判据。
    macho = (r['format'] == 'Mach-O')
    for s in syms:
        s['api'] = s['name'][1:] if (macho and s['name'].startswith('_')) else s['name']
    defined = [s for s in syms if s['shndx'] != 0]
    undef = [s for s in syms if s['shndx'] == 0]
    ours = [s for s in defined if s['api'].startswith('XRSLAM') or s['api'].startswith('xrslam_')]
    print('file            :', path)
    print('format          :', r['format'])
    if 'max_pt_load_align' in r:
        print('PT_LOAD aligns  :', [hex(a) for a in r['pt_load_aligns']],
              '=> max', hex(r['max_pt_load_align']),
              '(16KB-ready)' if r['max_pt_load_align'] >= 0x4000 else '(NOT 16KB)')
    print('dynsym total    :', len(syms))
    print('DEFINED         :', len(defined))
    print('UNDEF           :', len(undef))
    print('XRSLAM*/xrslam_*:', len(ours))
    probes = ['_Znwm', '_Znam', '_ZdlPv', '_ZdaPv', '__cxa_throw',
              '__cxa_begin_catch', '__cxa_end_catch', '__cxa_allocate_exception',
              '_ZTVN10__cxxabiv117__class_type_infoE']
    print('--- interposition probes ---')
    for p in probes:
        d = any(s['api'] == p or s['name'] == p for s in defined)
        u = any(s['api'] == p or s['name'] == p for s in undef)
        state = 'DEFINED(BAD)' if d else ('UND(ok)' if u else 'absent(ok)')
        print(f'  {p:42s} {state}')
    print('--- namespace leak counts (DEFINED) ---')
    import re
    for label, pat in [('std::__ndk1', r'^_ZNSt6__ndk1'), ('std::__1', r'^_ZNSt3__1'),
                       ('cv::', r'^_ZN2cv'), ('tbb::', r'^_ZN3tbb'),
                       ('spdlog::', r'^_ZN6spdlog'), ('fmt::', r'^_ZN3fmt'),
                       ('ceres::', r'^_ZN5ceres'), ('YAML::', r'^_ZN4YAML')]:
        c = sum(1 for s in defined if re.match(pat, s['name']) or re.match(pat, s['name'][1:] if s['name'].startswith('_') else ''))
        print(f'  {label:14s} {c}')
    print('--- exported XRSLAM/xrslam symbols ---')
    for s in sorted(ours, key=lambda x: x['api']):
        print('  ', s['name'])
    strays = [s for s in defined if s not in ours]
    print('--- NON-API defined symbols still exported (should be 0) ---')
    print('   count =', len(strays))
    for s in sorted(strays, key=lambda x: x['name'])[:40]:
        print('  ', s['name'])
    return r, defined, ours, strays

# [pw] --assert 模式:把上面那堆人眼判据变成退出码,才能当 CI 闸门用。
# 判据是**二进制的**,不匹配任何注释/标识符 —— 本仓注释里满是「[pw] 已删除 XXX」,
# 按名字 grep 必然自证成功,所以这里只看 ELF/Mach-O 的符号表本身。
def check(path):
    r, defined, ours, strays = main(path)
    fails = []
    if strays:
        fails.append('%d non-API symbols still exported (first: %s)'
                     % (len(strays), strays[0]['name']))
    if not ours:
        fails.append('no XRSLAM*/xrslam_* exported at all -- '
                     'hidden visibility applied without an export list?')
    probes = ['_Znwm', '_Znam', '_ZdlPv', '_ZdaPv', '__cxa_throw',
              '__cxa_begin_catch', '__cxa_end_catch', '__cxa_allocate_exception']
    bad = [p for p in probes if any(s['api'] == p for s in defined)]
    if bad:
        fails.append('interposable runtime symbols DEFINED: ' + ','.join(bad))
    if r['format'] == 'ELF' and r.get('max_pt_load_align', 0) < 0x4000:
        fails.append('PT_LOAD p_align=%s < 16KB'
                     % hex(r.get('max_pt_load_align', 0)))
    print('=== ASSERT ===')
    for f in fails:
        print('  FAIL:', f)
    if not fails:
        print('  PASS: %d exported, all XRSLAM*/xrslam_*' % len(ours))
    return 1 if fails else 0

if __name__ == '__main__':
    if '--assert' in sys.argv:
        sys.exit(check([a for a in sys.argv[1:] if a != '--assert'][0]))
    main(sys.argv[1])
