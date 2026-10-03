from pathlib import Path
import re

p = Path('yundong_part/application/app/src/main/java/com/example/app/MainActivity.kt')
s = p.read_text(encoding='utf-8')
a = s.index('    if (section == 0)')
b = s.index('\n@OptIn', a)
lines = s[a:b].splitlines(True)
inside = False
out = []
for line in lines:
    if line.startswith('    if (section =='):
        inside = True
    elif inside and line == '    }\n':
        inside = False
    elif inside and line.strip():
        line = line[4:]
    out.append(line)
part = ''.join(out)
pattern = r'(    if \(section == \d\) \{\n)(.*?)(    }\n)(?=    if|    error)'
part = re.sub(pattern, lambda m: m[1] + ''.join('    ' + line if line.strip() else line for line in m[2].splitlines(True)) + m[3], part, flags=re.S)
s = s[:a] + part + s[b:]
p.write_text(s, encoding='utf-8')
