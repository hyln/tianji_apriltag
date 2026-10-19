"""Rebuild the supplied vector AprilTag PDF with a 45 x 100 mm centered cut area.

Preserves the original tag drawing commands, positions, A4 page and 50 mm check line.
The center guide is outside the cut area; the printable tags stay untouched.
"""
import argparse
import base64
import re
import zlib
from pathlib import Path

PT_PER_MM = 72 / 25.4
PAGE_WIDTH = 595.2756
CUT_Y = 279.2126
CUT_HEIGHT = 100 * PT_PER_MM
CUT_WIDTH = 45 * PT_PER_MM
CUT_X = (PAGE_WIDTH - CUT_WIDTH) / 2
TAG_CENTERS = ((18, 491.811, 34), (19, 335.9055, 24))


def center_diagram(cut_right):
    """Vector annotation in the paper margin; never touches either tag pattern."""
    diagram_x = 470.0
    lines = [
        'q',
        '.55 .55 .55 RG', '.5 w',
        'BT /F2 10 Tf 12 TL ET',
        'BT 1 0 0 1 411 575 Tm (Tag centers - 1:1 guide) Tj ET',
    ]
    for tag_id, y, size_mm in TAG_CENTERS:
        half = size_mm * PT_PER_MM / 2
        left = diagram_x - half
        lines.extend([
            f'n {left:.4f} {y-half:.4f} {2*half:.4f} {2*half:.4f} re S',
            '[2 2] 0 d',
            f'n {cut_right + 2*PT_PER_MM:.4f} {y:.4f} m {left:.4f} {y:.4f} l S',
            '[] 0 d',
            '1 0 0 RG 1 0 0 rg', '1.1 w',
            f'n {diagram_x-4:.4f} {y:.4f} m {diagram_x+4:.4f} {y:.4f} l S',
            f'n {diagram_x:.4f} {y-4:.4f} m {diagram_x:.4f} {y+4:.4f} l S',
            f'n {diagram_x-1.5:.4f} {y-1.5:.4f} 3 3 re f',
            '0 0 0 rg',
            f'BT /F1 9 Tf 10.8 TL ET BT 1 0 0 1 479 {y+8:.4f} Tm (ID {tag_id}) Tj ET',
            '.55 .55 .55 RG', '.5 w',
        ])
    top = TAG_CENTERS[0][1]
    bottom = TAG_CENTERS[1][1]
    lines.extend([
        '1 0 0 RG 1 w',
        f'n 541 {bottom:.4f} m 541 {top:.4f} l S',
        f'n 537 {bottom:.4f} m 545 {bottom:.4f} l S',
        f'n 537 {top:.4f} m 545 {top:.4f} l S',
        '1 0 0 rg',
        f'BT /F2 10 Tf 12 TL ET BT 1 0 0 1 548 {(top+bottom)/2:.4f} Tm (55 mm) Tj ET',
        'Q',
    ])
    return '\n'.join(lines) + '\n'


def source_commands(path):
    raw = Path(path).read_bytes()
    match = re.search(rb'8 0 obj.*?stream\r?\n(.*?)endstream', raw, re.S)
    if not match:
        raise ValueError('Expected supplied ReportLab PDF stream')
    compressed = base64.a85decode(match.group(1).strip(), adobe=True)
    return zlib.decompress(compressed).decode('latin1')


def updated_commands(original):
    # The original art carries two 8-cell tags; do not draw over either one.
    start = original.index('q\n.65 G\n')
    end = original.index('BT /F2 15 Tf', start)
    x0, x1 = CUT_X, CUT_X + CUT_WIDTH
    y0, y1 = CUT_Y, CUT_Y + CUT_HEIGHT
    mm = PT_PER_MM
    lines = ['q', '.65 G', '.4 w', '[5.669291 4.251969] 0 d',
             f'n {x0:.4f} {y0:.4f} {CUT_WIDTH:.4f} {CUT_HEIGHT:.4f} re S',
             'Q', '.45 .45 .45 RG', '.3 w']
    for y, outward in ((y0, -1), (y1, 1)):
        for x, outward_x in ((x0, -1), (x1, 1)):
            lines.append(f'n {x + outward_x*2*mm:.4f} {y:.4f} m '
                         f'{x + outward_x*6*mm:.4f} {y:.4f} l S')
            lines.append(f'n {x:.4f} {y + outward*2*mm:.4f} m '
                         f'{x:.4f} {y + outward*6*mm:.4f} l S')
    revised = original[:start] + '\n'.join(lines) + '\n' + center_diagram(x1) + original[end:]
    old = 'Cut area: 50 x 100 mm | White margin: at least 8 mm'
    new = 'Cut area: 45 x 100 mm | Minimum white margin: 5.5 mm'
    if revised.count(old) != 1:
        raise ValueError('Expected exactly one cut-area label')
    return revised.replace(old, new)


def write_pdf(commands, path):
    stream = zlib.compress(commands.encode('latin1'))
    objects = [
        b'<< /Type /Catalog /Pages 2 0 R >>',
        b'<< /Type /Pages /Kids [3 0 R] /Count 1 >>',
        (b'<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595.2756 841.8898] '
         b'/Resources << /Font << /F1 4 0 R /F2 5 0 R >> >> /Contents 6 0 R >>'),
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>',
        b'<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>',
        b'<< /Length ' + str(len(stream)).encode() + b' /Filter /FlateDecode >>\nstream\n' + stream + b'\nendstream',
        b'<< /Title (AprilTag tag36h11 IDs 18 and 19 A4 45mm cut width) /Producer (Tianji AprilTag) >>',
    ]
    data = bytearray(b'%PDF-1.4\n%\xe2\xe3\xcf\xd3\n')
    offsets = [0]
    for index, obj in enumerate(objects, 1):
        offsets.append(len(data))
        data.extend(f'{index} 0 obj\n'.encode() + obj + b'\nendobj\n')
    startxref = len(data)
    data.extend(f'xref\n0 {len(offsets)}\n'.encode())
    data.extend(b'0000000000 65535 f \n')
    for offset in offsets[1:]:
        data.extend(f'{offset:010d} 00000 n \n'.encode())
    data.extend(f'trailer\n<< /Size {len(offsets)} /Root 1 0 R /Info 7 0 R >>\n'
                f'startxref\n{startxref}\n%%EOF\n'.encode())
    Path(path).write_bytes(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source')
    parser.add_argument('output')
    args = parser.parse_args()
    write_pdf(updated_commands(source_commands(args.source)), args.output)


if __name__ == '__main__':
    main()
