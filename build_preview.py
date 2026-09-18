from pathlib import Path
import re
root=Path(__file__).resolve().parent
three=(root/'viewer/vendor/three.module.js').read_text(encoding='utf-8')
# The upstream file is a self-contained ES module with one final export statement.
three,count=re.subn(r'\nexport\s*\{[^}]+\};?\s*$', '\n',three)
assert count==1, 'Unexpected Three.js export format'
template=(root/'viewer/preview.template.html').read_text(encoding='utf-8')
data=(root/'viewer/scene-data.json').read_text(encoding='utf-8')
notice=(root/'THIRD_PARTY_NOTICES.md').read_text(encoding='utf-8')
out=template.replace('__SCENE_DATA__',data).replace('__THREE_LIBRARY__','/*\n'+notice+'\n*/\n'+three)
assert '__SCENE_DATA__' not in out and '__THREE_LIBRARY__' not in out
(root/'preview.html').write_text(out,encoding='utf-8')
print('Offline preview:',len(out.encode('utf-8')),'bytes')
