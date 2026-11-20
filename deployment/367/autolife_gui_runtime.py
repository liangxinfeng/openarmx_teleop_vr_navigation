"""Select PyQt's desktop plugins after OpenCV imports its bundled Qt environment."""
import os
from pathlib import Path

def configure_qt_plugins():
    from PyQt5.QtCore import QLibraryInfo
    plugins = Path(QLibraryInfo.location(QLibraryInfo.PluginsPath))
    if not (plugins / 'platforms' / 'libqxcb.so').is_file():
        raise RuntimeError('PyQt xcb platform plugin is missing: ' + str(plugins))
    for key in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR'):
        if '/cv2/qt/' in os.environ.get(key, '').replace('\\', '/'):
            os.environ.pop(key, None)
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = str(plugins / 'platforms')
    return str(plugins)
