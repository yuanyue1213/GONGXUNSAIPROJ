"""按Maix应用目录约定打包，只包含app.yaml指定文件、入口和图标。"""
from pathlib import Path
import re
import zipfile

import yaml


def package(root):
    root = Path(root).resolve()
    info = yaml.safe_load((root / "app.yaml").read_text(encoding="utf-8"))
    app_id, version = info["id"], str(info["version"])
    if not re.fullmatch(r"[a-z][a-z0-9_]*", app_id):
        raise ValueError("Invalid app id")
    if not re.fullmatch(r"\d+\.\d+\.\d+", version):
        raise ValueError("Invalid app version")
    files = info.get("files")
    if not isinstance(files, list) or not files:
        raise ValueError("app.yaml must explicitly list files")
    names = list(dict.fromkeys([*files, "main.py", "app.yaml", info["icon"]]))
    for name in names:
        source = (root / name).resolve()
        if not source.is_relative_to(root) or not source.is_file():
            raise ValueError("Missing or invalid app file: " + name)

    output = root / "dist" / f"maix-{app_id}-v{version}.zip"
    output.parent.mkdir(exist_ok=True)
    temporary = output.with_suffix(".zip.tmp")
    with zipfile.ZipFile(temporary, "w", zipfile.ZIP_DEFLATED) as archive:
        for name in names:
            archive.write(root / name, f"{app_id}/{name}")
    with zipfile.ZipFile(temporary) as archive:
        assert archive.testzip() is None
        assert archive.read(f"{app_id}/main.py") == (root / "main.py").read_bytes()
        assert archive.read(f"{app_id}/app.yaml") == (root / "app.yaml").read_bytes()
        assert set(archive.namelist()) == {f"{app_id}/{name}" for name in names}
    temporary.replace(output)
    return output


if __name__ == "__main__":
    print(package(Path(__file__).resolve().parents[1]))
