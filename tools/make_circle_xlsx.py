#!/usr/bin/env python3
"""
Three-drone circle as an Excel workbook plus the CSVs and fleet yaml the player reads.

    python3 tools/make_circle_xlsx.py                  # defaults below
    python3 tools/make_circle_xlsx.py --radius 0.4 --omega-dps 20 --laps 2
    python3 tools/make_circle_xlsx.py --xlsx-to-csv trajectories/examples/circulo-3-drones.xlsx

traj_player reads CSV (x, y, z, t), one file per vehicle, never xlsx. The workbook
is the friendly copy: one sheet per drone (cf1, cf2, cf3) with the same four
columns, plus a "notas" sheet with the parameters. Edit the workbook in Excel,
then run --xlsx-to-csv to get the CSVs back. Standard library only (zipfile + XML).

Geometry: drone k sits at angle theta0 + k * 120 deg + angle(t), counterclockwise
seen from above, on a circle of the given radius around (cx, cy) at height z above
the pad. The angular speed follows a cosine ramp from rest over ramp_s, cruises at
omega, and ramps back to rest, so every file starts and ends at rest (the player's
rest-to-rest accel check) and the three drones keep exactly 120 deg between them.
Yaw is not a column: fly with `yaw: path` (the nose follows the tangent, theta + 90).
"""
import argparse
import math
import os
import re
import xml.etree.ElementTree as ET
import zipfile
from xml.sax.saxutils import escape

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), 'trajectories', 'examples')
NS = {'m': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
NAMES = ('cf1', 'cf2', 'cf3')


def angle(t, omega, ramp, total):
    """Swept angle (rad) at time t: cosine ramp up, cruise, cosine ramp down."""
    if t <= ramp:
        return omega * (t / 2.0 - ramp / (2.0 * math.pi) * math.sin(math.pi * t / ramp))
    t_end_cruise = total - ramp
    if t <= t_end_cruise:
        return omega * (ramp / 2.0 + (t - ramp))
    u = t - t_end_cruise
    return omega * (ramp / 2.0 + (t_end_cruise - ramp) + u / 2.0
                    + ramp / (2.0 * math.pi) * math.sin(math.pi * u / ramp))


def rows_for(k, p):
    omega = math.radians(p['omega_dps'])
    ramp = p['ramp_s']
    total = 2.0 * math.pi * p['laps'] / omega + ramp     # ramps each add half a ramp of angle
    n = int(round(total / p['dt']))
    out = []
    for i in range(n + 1):
        t = total * i / n
        th = math.radians(p['theta0_deg'] + k * 120.0) + angle(t, omega, ramp, total)
        out.append((p['cx'] + p['radius'] * math.cos(th), p['cy'] + p['radius'] * math.sin(th),
                    p['z'], t))
    return out


def fmt(rows):
    return [[f'{x:.6f}', f'{y:.6f}', f'{z:.6f}', f'{t:.4f}'] for x, y, z, t in rows]


def write_csv(path, rows, note):
    with open(path, 'w', encoding='utf-8') as f:
        f.write(f'# {note}\nx,y,z,t\n')
        for r in fmt(rows):
            f.write(','.join(r) + '\n')


def _cell(ref, v):
    if isinstance(v, str) and not re.fullmatch(r'-?\d+(\.\d+)?', v):
        return f'<c r="{ref}" t="inlineStr"><is><t>{escape(v)}</t></is></c>'
    return f'<c r="{ref}"><v>{v}</v></c>'


def _sheet(table, widths):
    cols = ''.join(f'<col min="{i}" max="{i}" width="{w}" customWidth="1"/>'
                   for i, w in enumerate(widths, 1))
    rows = ''.join(
        f'<row r="{r}">' + ''.join(_cell(f'{chr(64 + c)}{r}', v) for c, v in enumerate(row, 1))
        + '</row>' for r, row in enumerate(table, 1))
    return ('<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
            f'<worksheet xmlns="{NS["m"]}"><cols>{cols}</cols><sheetData>{rows}</sheetData>'
            '</worksheet>')


def write_xlsx(path, sheets):
    """sheets: list of (name, table, widths); table = list of rows of str/number."""
    ct = ''.join(f'<Override PartName="/xl/worksheets/sheet{i}.xml" ContentType="application/'
                 'vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
                 for i in range(1, len(sheets) + 1))
    rid = ''.join(f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/'
                  f'officeDocument/2006/relationships/worksheet" Target="worksheets/sheet{i}.xml"/>'
                  for i in range(1, len(sheets) + 1))
    sh = ''.join(f'<sheet name="{n}" sheetId="{i}" r:id="rId{i}"/>'
                 for i, (n, _, _) in enumerate(sheets, 1))
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml',
                   '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.'
                   'openxmlformats.org/package/2006/content-types"><Default Extension="rels" '
                   'ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
                   '<Default Extension="xml" ContentType="application/xml"/><Override '
                   'PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-'
                   f'officedocument.spreadsheetml.sheet.main+xml"/>{ct}</Types>')
        z.writestr('_rels/.rels',
                   '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.'
                   'openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" '
                   'Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/'
                   'officeDocument" Target="xl/workbook.xml"/></Relationships>')
        z.writestr('xl/workbook.xml',
                   f'<?xml version="1.0" encoding="UTF-8"?><workbook xmlns="{NS["m"]}" '
                   'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
                   f'<sheets>{sh}</sheets></workbook>')
        z.writestr('xl/_rels/workbook.xml.rels',
                   '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.'
                   f'openxmlformats.org/package/2006/relationships">{rid}</Relationships>')
        for i, (_, table, widths) in enumerate(sheets, 1):
            z.writestr(f'xl/worksheets/sheet{i}.xml', _sheet(table, widths))


def read_xlsx(path):
    """{sheet name: rows of str}; handles inline and shared strings (Excel-saved files)."""
    with zipfile.ZipFile(path) as z:
        shared = []
        if 'xl/sharedStrings.xml' in z.namelist():
            for si in ET.fromstring(z.read('xl/sharedStrings.xml')).findall('m:si', NS):
                shared.append(''.join(t.text or '' for t in si.iter(f'{{{NS["m"]}}}t')))
        wb = ET.fromstring(z.read('xl/workbook.xml'))
        rels = ET.fromstring(z.read('xl/_rels/workbook.xml.rels'))
        target = {r.get('Id'): r.get('Target') for r in rels}
        rkey = '{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id'
        out = {}
        for s in wb.find('m:sheets', NS):
            tgt = target[s.get(rkey)].lstrip('/')
            root = ET.fromstring(z.read(tgt if tgt.startswith('xl/') else 'xl/' + tgt))
            rows = []
            for r in root.iter(f'{{{NS["m"]}}}row'):
                row = []
                for c in r.findall('m:c', NS):
                    col = ord(re.match(r'[A-Z]+', c.get('r')).group()[0]) - 65
                    row += [''] * (col - len(row))
                    if c.get('t') == 'inlineStr':
                        v = ''.join(t.text or '' for t in c.iter(f'{{{NS["m"]}}}t'))
                    else:
                        e = c.find('m:v', NS)
                        v = '' if e is None else (shared[int(e.text)] if c.get('t') == 's'
                                                  else e.text)
                    row.append(v)
                rows.append(row)
            out[s.get('name')] = rows
        return out


def xlsx_to_csv(path, outdir):
    for name, rows in read_xlsx(path).items():
        if name not in NAMES:
            continue
        dst = os.path.join(outdir, f'circulo-3-drones_{name}.csv')
        with open(dst, 'w', encoding='utf-8') as f:
            f.write(f'# {name} from {os.path.basename(path)} (tools/make_circle_xlsx.py)\n')
            for r in rows:
                if any(c != '' for c in r):
                    f.write(','.join(r[:4]) + '\n')
        print(dst)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    for name, default, h in (
            ('radius', 0.5, 'circle radius (m)'), ('omega-dps', 30.0, 'angular speed (deg/s)'),
            ('cx', 0.0, 'centre x, Motive world (m)'), ('cy', 0.0, 'centre y, Motive world (m)'),
            ('z', 1.0, 'height above the pad (m)'), ('laps', 3.0, 'full laps at the cruise rate'),
            ('ramp-s', 4.0, 'cosine ramp from and to rest (s)'),
            ('theta0-deg', 0.0, 'angle of cf1 at t=0; cf2 +120, cf3 +240'),
            ('dt', 0.05, 'row spacing (s); heading steps per row must stay under yaw_rate_max_dps x 20 ms')):
        ap.add_argument('--' + name, type=float, default=default, help=f'{h} (default {default})')
    ap.add_argument('--outdir', default=OUT)
    ap.add_argument('--xlsx-to-csv', metavar='XLSX', help='only convert an edited workbook to CSVs')
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    if a.xlsx_to_csv:
        return xlsx_to_csv(a.xlsx_to_csv, a.outdir)
    p = {k.replace('-', '_'): v for k, v in vars(a).items()}
    omega = math.radians(p['omega_dps'])
    vlin = omega * p['radius']
    total = 2 * math.pi * p['laps'] / omega + p['ramp_s']
    sheets, note = [], (f'circle r {p["radius"]} m around ({p["cx"]}, {p["cy"]}), z {p["z"]} m, '
                        f'{p["omega_dps"]} deg/s, 120 deg apart')
    for k, n in enumerate(NAMES):
        rows = rows_for(k, p)
        sheets.append((n, [['x', 'y', 'z', 't']] + fmt(rows), (12, 12, 8, 10)))
        write_csv(os.path.join(a.outdir, f'circulo-3-drones_{n}.csv'), rows, f'{n}: {note}')
    notas = [
        ['parametro', 'valor', 'detalle'],
        ['radio_m', p['radius'], 'radio del circulo (diametro = 2 x radio)'],
        ['centro_x_m', p['cx'], 'mundo Motive'],
        ['centro_y_m', p['cy'], 'mundo Motive'],
        ['altura_z_m', p['z'], 'sobre el pad (el player suma z0)'],
        ['omega_grados_por_s', p['omega_dps'], 'velocidad angular, antihoraria vista desde arriba'],
        ['vueltas', p['laps'], 'vueltas a velocidad de crucero'],
        ['rampa_s', p['ramp_s'], 'arranque y frenado suaves desde y hasta el reposo'],
        ['theta0_grados', p['theta0_deg'], 'angulo de cf1 en t=0; cf2 +120, cf3 +240'],
        ['velocidad_lineal_m_s', f'{vlin:.4f}', 'omega x radio'],
        ['duracion_s', f'{total:.2f}', 'incluye las dos rampas'],
        [],
        ['formato', '', 'hojas cf1, cf2, cf3 con encabezado exacto x,y,z,t (m, m, m sobre el pad, s)'],
        ['yaw', '', 'no es columna: volar con yaw: path (morro en la tangente, theta + 90 grados)'],
        ['exportar', '', 'por cada hoja cf1, cf2, cf3: Archivo > Guardar como > CSV UTF-8 (delimitado por comas), '
                         'punto decimal; nombre circulo-3-drones_cf1.csv, _cf2, _cf3'],
        ['reglas', '', 'primera fila t = 0, t creciente, velocidad inicial y final cero, z entre 0.2 y 2.0, '
                       'x,y dentro de +-1.75; verificar con rr traj check ARCHIVO.csv --yaw path'],
        ['sin Excel', '', 'python3 tools/make_circle_xlsx.py --xlsx-to-csv ARCHIVO.xlsx'],
        ['fleet', '', 'circulo-3-drones_fleet.yaml apunta a /traj/examples/circulo-3-drones_cf*.csv'],
    ]
    sheets.append(('notas', notas, (24, 14, 80)))
    write_xlsx(os.path.join(a.outdir, 'circulo-3-drones.xlsx'), sheets)
    print(f'{a.outdir}: {len(rows)} rows per drone, {total:.1f} s, {vlin:.3f} m/s')


if __name__ == '__main__':
    main()
