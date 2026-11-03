"""Derive the shipping experiment images from their generated bases.

The images TYRA_VU1_EXP_EE_LIGHT_FOLD and TYRA_VU1_EXP_ENV_NORMALIZED link
(stapip_vu1_experiments.hpp) are the base programs from
`tyrax-editor --vu-emit` with one thing taken out:

- the `_fold` images (cull D and TD, clip C and TC): the lit path's
  light-matrix multiply, because the EE uploads the light directions already
  multiplied by it;
- the `_envn` image (cull TCE): the env normal's renormalise, because the
  normal reaches the program already unit length (x 127 in the compact
  format, which the env basis's 1/127 undoes).

`--vu-check` holds every image to its description (vugen::foldedLightsImage,
vugen::unitEnvNormalsImage) and proves it against its float twin. Re-run this
after copying new cull or clip bases into the engine:

    python tools/derive-fold-images.py vendor/tyra/engine [--check]
"""
import argparse
import re
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('engine', type=Path)
parser.add_argument('--check', action='store_true')
args = parser.parse_args()
programs = args.engine / 'src/renderer/3d/pipeline/static/core/programs'

FOLDED = '  ; CalculateTyraDirectionalLights (folded: EE uploads D*M)'
MATRIX_LOADS = ('    lq.xyz     lightMatrix[0], 4(vi00)  ; LoadTyraDirectionalLights\n'
                '    lq.xyz     lightMatrix[1], 5(vi00)\n'
                '    lq.xyz     lightMatrix[2], 6(vi00)\n')
UNIT = '  ; CalculateTyraEnvStqUnit: the EE normalised it'


def finish(stem, out, tag, symbol, header):
    """Rename the image, put the header on, write it (or check it)."""
    name = re.search(r'^\.name (\w+)$', out, re.M)[1]
    out = out.replace('.name ' + name, '.name ' + name + '_' + symbol, 1)
    out = re.sub(r'^(#vuprog \w+)$', r'\g<1>' + symbol, out, count=1, flags=re.M)
    base = stem.split('/')[-1]
    out = (header + '\n'
           '; Linked only with its toggle on (stapip_vu1_experiments.hpp);\n'
           f'; derived from {base}.vclpp - re-derive, do not edit.\n' + out)
    dest = programs / f'{stem.replace("_vu1", "_" + tag + "_vu1")}.vclpp'
    if args.check:
        assert dest.read_text() == out, f'{dest} is stale'
    else:
        dest.write_bytes(out.encode())
    print(dest)


def derive(stem, path_name, matrix, dirs, corners, drop_loads):
    out = (programs / f'{stem}.vclpp').read_text()
    # The light-matrix multiply on each corner goes; the first direction
    # product inherits the macro's comment.
    for c in corners:
        old = (f'    mula.xyz   acc, {matrix[0]}, {c}[x]  ; CalculateTyraDirectionalLights\n'
               f'    madda.xyz  acc, {matrix[1]}, {c}[y]\n'
               f'    madd.xyz   {c}, {matrix[2]}, {c}[z]\n'
               f'    mula.xyz   acc, {dirs}, {c}[x]\n')
        assert out.count(old) == 1, (stem, c, out.count(old))
        out = out.replace(old, f'    mula.xyz   acc, {dirs}, {c}[x]{FOLDED}\n')
    if drop_loads:  # an image that loads a matrix it no longer reads
        assert out.count(MATRIX_LOADS) == 1, (stem, 'loads')
        out = out.replace(MATRIX_LOADS, '')
    path = f' (the {path_name} path)' if path_name else ''
    finish(stem, out, 'fold', 'FOLD',
           f'; Added by TyraX: EXPERIMENT IMAGE - (b) light matrix folded on the EE{path}')


def derive_envn(stem, corners):
    out = (programs / f'{stem}.vclpp').read_text()
    # The renormalise goes: the basis multiplies the normal itself.
    for c in corners:
        old = re.compile(
            rf'    mul\.xyz    tyraEnvLen, {c}, {c}  ; CalculateTyraEnvStq: renormalize \(uses Q\)\n'
            r'    add\.x      tyraEnvLen, tyraEnvLen, tyraEnvLen\[y\]\n'
            r'    add\.x      tyraEnvLen, tyraEnvLen, tyraEnvLen\[z\]\n'
            r'    rsqrt      q, \w+\[w\], tyraEnvLen\[x\][^\n]*\n'
            rf'    mul\.xyz    tyraEnvNrm, {c}, q\n'
            r'    mula\.xy    acc, envBasisX, tyraEnvNrm\[x\]\n'
            r'    madda\.xy   acc, envBasisY, tyraEnvNrm\[y\]\n'
            rf'    madd\.xy    {c}, envBasisZ, tyraEnvNrm\[z\]\n')
        assert len(old.findall(out)) == 1, (stem, c, len(old.findall(out)))
        out = old.sub(f'    mula.xy    acc, envBasisX, {c}[x]{UNIT}\n'
                      f'    madda.xy   acc, envBasisY, {c}[y]\n'
                      f'    madd.xy    {c}, envBasisZ, {c}[z]\n', out)
    finish(stem, out, 'envn', 'ENVN',
           '; Added by TyraX: EXPERIMENT IMAGE - (c) env normals normalized once on the EE')


NORMALS = ['normal1', 'normal2', 'normal3']
LIGHT_MATRIX = ['lightMatrix[0]', 'lightMatrix[1]', 'lightMatrix[2]']
derive('cull/stapip_cull_d_vu1', None, LIGHT_MATRIX, 'lightDirections[0]', NORMALS, True)
derive('cull/stapip_cull_td_vu1', None, LIGHT_MATRIX, 'lightDirections[0]', NORMALS, True)
derive_envn('cull/stapip_cull_tce_vu1', ['stq1', 'stq2', 'stq3'])
derive('clip/stapip_clip_c_vu1', 'D', LIGHT_MATRIX, 'lightDirections[0]', NORMALS, True)
derive('clip/stapip_clip_tc_vu1', 'TD', ['envBasisX', 'envBasisY', 'envBasisZ'],
       'spotPos', ['color1', 'color2', 'color3'], False)
