"""Build a self-contained car viewer without modifying the sandbox viewer."""
from pathlib import Path
import re
package=Path(__file__).resolve().parents[1]
workspace=package.parents[1]
three=(workspace/'viewer/vendor/three.module.js').read_text(encoding='utf-8')
three,count=re.subn(r'\nexport\s*\{[^}]+\};?\s*$', '\n',three)
assert count==1
template=(package/'scripts/vehicle_preview.template.html').read_text(encoding='utf-8')
data=(workspace/'vehicle_preview/vehicle-data.json').read_text(encoding='utf-8')
notice=(workspace/'THIRD_PARTY_NOTICES.md').read_text(encoding='utf-8')
text=template.replace('__VEHICLE_DATA__',data).replace('__THREE_LIBRARY__','/*\n'+notice+'\n*/\n'+three)
(workspace/'vehicle_preview.html').write_text(text,encoding='utf-8')
print('Vehicle viewer bytes:',len(text.encode('utf-8')))
