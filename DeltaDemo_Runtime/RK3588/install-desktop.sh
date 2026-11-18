#!/bin/sh
# Run once from the board's graphical desktop, as the desktop login user.
set -eu
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
command -v python3 >/dev/null 2>&1 || { echo '需要 python3 才能创建桌面图标。'; exit 1; }
python3 - "$here" <<'PY'
import os
from pathlib import Path
import shutil
import subprocess
import sys

folder = Path(sys.argv[1])
for name in ('DeltaDemo', 'start.sh', 'app.png'):
    if not (folder / name).is_file():
        sys.exit('软件文件不完整：' + name)
for name in ('DeltaDemo', 'start.sh'):
    item = folder / name
    item.chmod(item.stat().st_mode | 0o100)

def value(text):
    return str(text).replace('\\', '\\\\').replace('\n', '\\n').replace('\r', '\\r').replace('\t', '\\t')

def argument(text):
    text = str(text).replace('%', '%%')
    for char in ('\\', '"', '`', '$'):
        text = text.replace(char, '\\' + char)
    return value('"' + text + '"')

desktop = Path.home() / 'Desktop'
if shutil.which('xdg-user-dir'):
    result = subprocess.run(['xdg-user-dir', 'DESKTOP'], capture_output=True, text=True)
    if result.returncode == 0 and result.stdout.strip():
        desktop = Path(result.stdout.strip())
applications = Path(os.environ.get('XDG_DATA_HOME', str(Path.home() / '.local/share'))) / 'applications'
content = '\n'.join([
    '[Desktop Entry]', 'Type=Application', 'Version=1.0',
    'Name=Delta 蜘蛛手控制', 'Comment=Delta 控制 Demo — RK3588 · M800',
    'Exec=/bin/sh ' + argument(folder / 'start.sh'),
    'Icon=' + value(folder / 'app.png'),
    'Terminal=false', 'Categories=Utility;', 'StartupNotify=false', '',
])
for destination in (applications, desktop):
    destination.mkdir(parents=True, exist_ok=True)
    launcher = destination / 'DeltaDemo.desktop'
    launcher.write_text(content, encoding='utf-8')
    launcher.chmod(0o755)
    if shutil.which('gio'):
        subprocess.run(['gio', 'set', str(launcher), 'metadata::trusted', 'true'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
print('已创建桌面和应用菜单图标：Delta 蜘蛛手控制')
print('若桌面提示不可信，请右键图标选择“允许启动”。')
print('移动软件文件夹后，请重新运行此安装脚本。')
PY
